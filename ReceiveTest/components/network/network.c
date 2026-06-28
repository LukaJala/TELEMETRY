/*
 * network.c
 * Ethernet initialization with static IP + TCP server
 *
 * IP101 (native) : 192.168.2.100  — receives CAN frames from Vega (TCP server :5000)
 * W5500 (SPI)    : 192.168.77.1   — streams pit telemetry to the Pi (TCP client)
 * TCP Port: 5000
 */

#include "network.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/* For TCP sockets */
#include "lwip/sockets.h"

static const char *TAG = "NETWORK";

/* ============================================================
 * CONFIGURATION - Change these if needed
 * ============================================================ */
#define STATIC_IP "192.168.2.100"    /* ESP32 IP on IP101 (native) port — receives CAN from Vega.
                                      * Vega's W5500 is the TCP client at 192.168.2.50 dialing
                                      * this address:5000. The IP101 PHY has Auto-MDIX, so a plain
                                      * straight cable links to Vega's W5500 (which has none). */
#define STATIC_GATEWAY "192.168.2.1" /* no router on this link; gateway unused for on-link Vega */
#define STATIC_NETMASK "255.255.255.0"
#define TCP_PORT 5000

/* W5500 SPI Ethernet (second interface) — carries pit telemetry out to the Pi.
 * The Pi's USB-Ethernet dongle has Auto-MDIX, so a straight cable links to the W5500. */
#define W5500_STATIC_IP "192.168.77.1"
#define W5500_GATEWAY "192.168.77.2" /* the Pi dongle (TCP server we stream to) */

#define W5500_MOSI_GPIO 23
#define W5500_MISO_GPIO 22
#define W5500_SCLK_GPIO 21
#define W5500_CS_GPIO 24
#define W5500_INT_GPIO 25
#define W5500_RST_GPIO 33
#define W5500_SPI_HOST SPI2_HOST
#define W5500_SPI_MHZ 8

/* Pit telemetry → Raspberry Pi Zero 2 W (runs relay.py) reached via its
 * USB-Ethernet adapter on the P4's W5500 SPI port. The Pi is a TCP SERVER on
 * port 5000; we connect to it as a client and stream a fixed 30-byte packet
 * that relay.py + the laptop's server.py already expect:
 *   '>H I f f f f f H H'  =
 *   magic(0xA55A) seq battery_v battery_a rpm temperature speed fault checksum
 * (all big-endian; checksum = sum of bytes[0..27] & 0xFFFF). */
#define PI_IP "192.168.77.2"
#define PI_PORT 5000
#define PIT_MAGIC 0xA55A
#define PIT_PACKET_SIZE 30
#define PI_SEND_INTERVAL_MS 200 /* 5 Hz — low rate lets the RFD900X run a low AIR_SPEED for max range */
#define PI_RECONNECT_DELAY_MS 250 /* short backoff between reconnect attempts so the board
                                   * re-finds the Pi almost immediately after it drops */
#define PI_CONNECT_TIMEOUT_MS 2000 /* bound each connect attempt so a blocking connect()
                                    * can't stall on the full TCP SYN timeout at boot */
#define PI_SEND_TIMEOUT_MS 1500 /* SO_SNDTIMEO: while streaming there is always unacked
                                 * data in flight, which SUPPRESSES TCP keepalive, so a
                                 * dead peer would otherwise only surface after the full
                                 * retransmit chain (tens of seconds). Bounding send()
                                 * guarantees the loop regains control and can reconnect. */
#define PI_LIVENESS_TIMEOUT_MS 2000 /* relay.py echoes a heartbeat byte for every packet it
                                     * receives; if none arrives for this long the Pi is
                                     * assumed dead and the link is torn down + reconnected.
                                     * Detects a silently-dead Pi in ~2s regardless of TCP
                                     * retransmit/keepalive timing — the main fast-reconnect
                                     * path when the Pi power-cycles. */

/* CAN frame over TCP is exactly 14 bytes: [0x01 magic][CAN_ID 4B][DLC 1B][data 8B]
 * The 0x01 prefix distinguishes binary frames from text commands, which are always
 * printable ASCII (>= 0x20) and therefore can never start with 0x01. */
#define CAN_FRAME_MAGIC 0x01
#define CAN_FRAME_SIZE 14

/* recv() read buffer — limits bytes consumed per recv() call */
#define RECV_BUF_SIZE (CAN_FRAME_SIZE * 8) /* 112 bytes */

/* Accumulator must be larger than RECV_BUF_SIZE so that leftover partial-frame
 * bytes plus a full recv() never exceed the buffer (overflow + reset = desync) */
#define ACCUM_SIZE (CAN_FRAME_SIZE * 32) /* 448 bytes */

/* Liveness / dead-peer detection. recv() is given a finite timeout so the
 * server task wakes periodically instead of blocking forever. If no data has
 * arrived for LINK_TIMEOUT_S (cable pull, sender crash, laptop sleep — none of
 * which produce a clean TCP close), the connection is treated as dead and torn
 * down, which fires the disconnect callback and stops the camera. */
#define RECV_TIMEOUT_S 2 /* recv() wakes at least this often */
#define LINK_TIMEOUT_S 5 /* silence longer than this => peer is dead */

/* ============================================================
 * GLOBAL STATE
 * ============================================================ */
static char ip_address_str[16] = "0.0.0.0";
static network_data_callback_t data_callback = NULL;
static network_connect_callback_t connect_callback = NULL;
static network_disconnect_callback_t disconnect_callback = NULL;
static esp_netif_t *eth_netif = NULL;

/* Per-interface driver handles, saved so the shared Ethernet event handler can
 * tell which physical link an event came from and label it VEGA vs PI:
 *   IP101 native port -> VEGA (receives CAN; 192.168.2.100 server)
 *   W5500 SPI module  -> PI   (streams telemetry; 192.168.77.1 client) */
static esp_eth_handle_t s_eth_handle_ip101 = NULL; /* VEGA-facing */
static esp_eth_handle_t s_eth_handle_w5500 = NULL; /* PI-facing  */

/* Latest telemetry snapshot, written by network_set_telemetry() (CAN task)
 * and read by the pit sender task. Guarded by a lightweight spinlock since
 * the copy is tiny. */
static pit_telemetry_t pit_latest;
static portMUX_TYPE pit_mux = portMUX_INITIALIZER_UNLOCKED;

/* True while the pit sender holds a live, streaming TCP connection to the Pi.
 * Single-writer (pit_sender_task), polled by the UI via network_pi_is_connected().
 * volatile so the poller always observes the latest value. */
static volatile bool s_pi_connected = false;

/* ============================================================
 * ETHERNET EVENT HANDLER
 * Called when Ethernet connects/disconnects
 * ============================================================ */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    /* Both interfaces share this handler. Identify which one fired so the monitor
     * says plainly VEGA (the IP101 link to Vega) vs PI (the W5500 link to the Pi). */
    esp_eth_handle_t eth = (event_data != NULL) ? *(esp_eth_handle_t *)event_data : NULL;
    const char *who = "ETH";
    if (eth == s_eth_handle_ip101)
        who = "VEGA link (IP101)";
    else if (eth == s_eth_handle_w5500)
        who = "PI link (W5500)";

    switch (event_id)
    {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "%s: cable connected (link UP)", who);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "%s: cable DISCONNECTED (link down)", who);
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "%s: started", who);
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "%s: stopped", who);
        break;
    default:
        break;
    }
}

/* ============================================================
 * IP EVENT HANDLER
 * Called when we get an IP address
 * ============================================================ */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_ETH_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        char ipbuf[16];
        snprintf(ipbuf, sizeof(ipbuf), IPSTR, IP2STR(&event->ip_info.ip));

        /* Only the IP101 (VEGA-facing) address is shown on the dashboard; don't let
         * the W5500/PI interface clobber it when it also gets its static IP. */
        if (event->esp_netif == eth_netif)
        {
            strncpy(ip_address_str, ipbuf, sizeof(ip_address_str));
            ESP_LOGI(TAG, "VEGA link (IP101) got IP: %s", ipbuf);
        }
        else
        {
            ESP_LOGI(TAG, "PI link (W5500) got IP: %s", ipbuf);
        }
    }
}

/* ============================================================
 * TCP SERVER TASK
 * Runs in background, accepts connections, receives data.
 *
 * Frame dispatch logic (see CAN_FRAME_MAGIC / CAN_FRAME_SIZE):
 *   - Bytes accumulate in a local buffer.
 *   - If the leading byte is 0x01 (CAN_FRAME_MAGIC), consume one 14-byte
 *     binary CAN frame from the front: [0x01][CAN_ID 4B][DLC 1B][data 8B].
 *   - If the leading byte is printable ASCII (>= 0x20), treat the front of
 *     the buffer as a null-terminated text command (e.g. __CAM_START__).
 *   - Any other leading byte is framing noise: drop one byte and resync.
 * ============================================================ */
static void tcp_server_task(void *pvParameters)
{
    uint8_t accum[ACCUM_SIZE];
    int accum_len = 0;
    uint8_t raw[RECV_BUF_SIZE];

    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    ESP_LOGI(TAG, "Starting VEGA server (IP101) on port %d", TCP_PORT);

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) < 0)
    {
        ESP_LOGE(TAG, "Failed to listen");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "VEGA server listening on %s:%d — waiting for Vega @ 192.168.2.50", ip_address_str, TCP_PORT);

    while (1)
    {
        ESP_LOGI(TAG, "Waiting for VEGA to connect...");

        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0)
        {
            /* Brief pause so a persistent accept() failure can't spin the task at
             * 100% CPU and flood the log. */
            ESP_LOGE(TAG, "Failed to accept connection (errno=%d)", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "VEGA CONNECTED from %s", inet_ntoa(client_addr.sin_addr));
        accum_len = 0;

        /* --- Dead-peer detection ---
         * SO_RCVTIMEO makes recv() return periodically so we can check liveness.
         * SO_KEEPALIVE actively probes the peer so a half-open TCP connection is
         * detected even if no data is expected. */
        struct timeval rcv_to = {.tv_sec = RECV_TIMEOUT_S, .tv_usec = 0};
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));
        int ka_enable = 1, ka_idle = 3, ka_intvl = 1, ka_cnt = 3;
        setsockopt(client_sock, SOL_SOCKET, SO_KEEPALIVE, &ka_enable, sizeof(ka_enable));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));
        int64_t last_rx_us = esp_timer_get_time();

        if (connect_callback != NULL)
        {
            connect_callback();
        }

        while (1)
        {
            int space = (int)sizeof(raw);
            int len = recv(client_sock, raw, space, 0);

            if (len == 0)
            {
                ESP_LOGI(TAG, "VEGA disconnected");
                break;
            }
            if (len < 0)
            {
                /* recv() timed out (SO_RCVTIMEO) — not fatal on its own, but if
                 * the peer has been silent too long it is dead (unclean drop).
                 * Tearing down here triggers the disconnect callback => the
                 * camera is stopped and the UI reset instead of hanging. */
                if (errno == EWOULDBLOCK || errno == EAGAIN)
                {
                    if (esp_timer_get_time() - last_rx_us > (int64_t)LINK_TIMEOUT_S * 1000000)
                    {
                        ESP_LOGW(TAG, "No data for %ds — peer assumed dead, dropping connection", LINK_TIMEOUT_S);
                        break;
                    }
                    continue; /* still within grace window, keep waiting */
                }
                ESP_LOGE(TAG, "Receive error (errno=%d) — dropping connection", errno);
                break;
            }

            last_rx_us = esp_timer_get_time();

            /* Append new bytes to accumulator, guard against overflow */
            if (accum_len + len > (int)sizeof(accum))
            {
                ESP_LOGW(TAG, "Accumulator overflow — resetting");
                accum_len = 0;
            }
            memcpy(accum + accum_len, raw, len);
            accum_len += len;

            /* Dispatch all complete units from the front of the accumulator */
            while (accum_len > 0 && data_callback != NULL)
            {

                if (accum[0] == CAN_FRAME_MAGIC)
                {
                    /* Binary CAN frame: [0x01][CAN_ID 4B][DLC 1B][data 8B] */
                    if (accum_len < CAN_FRAME_SIZE)
                    {
                        break; /* wait for more bytes */
                    }
                    data_callback(accum, CAN_FRAME_SIZE);
                    accum_len -= CAN_FRAME_SIZE;
                    memmove(accum, accum + CAN_FRAME_SIZE, accum_len);
                }
                else if (accum[0] >= 0x20)
                {
                    /* Text command: null-terminated printable ASCII string */
                    int cmd_len = 0;
                    while (cmd_len < accum_len && accum[cmd_len] != '\0')
                    {
                        cmd_len++;
                    }
                    if (cmd_len < accum_len && accum[cmd_len] == '\0')
                    {
                        /* Complete — dispatch */
                        ESP_LOGI(TAG, "Text command: %s", (char *)accum);
                        data_callback(accum, cmd_len);
                        int consumed = cmd_len + 1;
                        accum_len -= consumed;
                        memmove(accum, accum + consumed, accum_len);
                    }
                    else
                    {
                        /* Incomplete — wait for more bytes */
                        break;
                    }
                }
                else
                {
                    /* Unknown byte — drop it and resync */
                    ESP_LOGW(TAG, "Unknown framing byte 0x%02X — dropping", accum[0]);
                    accum_len--;
                    memmove(accum, accum + 1, accum_len);
                }
            }
        }

        close(client_sock);

        if (disconnect_callback != NULL)
        {
            disconnect_callback();
        }
    }

    close(listen_sock);
    vTaskDelete(NULL);
}

/* ============================================================
 * W5500 SPI ETHERNET INITIALIZATION
 * Second network interface — streams pit telemetry out to the Pi.
 * ESP32 IP on this interface: 192.168.77.1
 * Pi (relay.py TCP server) is at 192.168.77.2 on its USB-Ethernet dongle.
 * ============================================================ */
static esp_err_t init_w5500(void)
{
    /* The GPIO ISR service (needed for the W5500 INT line) may already be
     * installed by another driver — ESP_ERR_INVALID_STATE just means "already
     * there" and is fine. Only a genuine failure is worth a warning. */
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "GPIO ISR service install failed (0x%x) — W5500 INT may not work", err);
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = W5500_MOSI_GPIO,
        .miso_io_num = W5500_MISO_GPIO,
        .sclk_io_num = W5500_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    /* Graceful, not ESP_ERROR_CHECK: a SPI failure must NOT bootloop the device —
     * the whole point of this path is to degrade to "IP101 only". */
    err = spi_bus_initialize(W5500_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "W5500 SPI bus init failed (0x%x) — running on IP101 only", err);
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 16,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = W5500_SPI_MHZ * 1000 * 1000,
        .spics_io_num = W5500_CS_GPIO,
        .queue_size = 20,
    };

    eth_w5500_config_t w5500_cfg = ETH_W5500_DEFAULT_CONFIG(W5500_SPI_HOST, &devcfg);
    w5500_cfg.int_gpio_num = W5500_INT_GPIO;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = W5500_RST_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);
    if (mac == NULL || phy == NULL)
    {
        ESP_LOGW(TAG, "W5500 MAC/PHY alloc failed — running on IP101 only");
        if (mac)
            mac->del(mac);
        if (phy)
            phy->del(phy);
        spi_bus_free(W5500_SPI_HOST);
        return ESP_ERR_NO_MEM;
    }

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    /* This is the real "is the chip there?" probe — it reads the W5500 version
     * register, so an unwired/unpowered chip fails here. Non-fatal: free what we
     * allocated and fall back to IP101. */
    err = esp_eth_driver_install(&eth_cfg, &eth_handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "PI LINK (W5500) not detected (0x%x) — Pi telemetry off; VEGA/display unaffected", err);
        esp_log_level_set("w5500.mac", ESP_LOG_NONE);
        mac->del(mac);
        phy->del(phy);
        spi_bus_free(W5500_SPI_HOST);
        return err;
    }
    s_eth_handle_w5500 = eth_handle; /* PI-facing handle, for the event handler's labeling */

    /* Program a MAC address. Unlike the internal EMAC (which gets one from efuse),
     * the SPI W5500 powers up as 00:00:00:00:00:00 — an invalid all-zero MAC that
     * breaks ARP: the Pi (or its dongle) won't answer ARP from an all-zero sender,
     * so the outbound connect to the Pi never resolves. Derive a stable,
     * locally-administered unicast MAC from the chip's base MAC, distinct from the
     * IP101 interface's. Must be set before attach/start so the netif and chip
     * advertise it. */
    uint8_t w5500_mac[6];
    esp_read_mac(w5500_mac, ESP_MAC_ETH);
    w5500_mac[0] |= 0x02; /* locally administered, unicast */
    w5500_mac[5] ^= 0x01; /* differ from the IP101 interface MAC */
    if (esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, w5500_mac) != ESP_OK)
    {
        ESP_LOGW(TAG, "W5500 set MAC failed — ARP/peer connect may not work");
    }
    else
    {
        ESP_LOGI(TAG, "PI LINK (W5500) MAC %02x:%02x:%02x:%02x:%02x:%02x",
                 w5500_mac[0], w5500_mac[1], w5500_mac[2],
                 w5500_mac[3], w5500_mac[4], w5500_mac[5]);
    }

    /* Create a separate netif with a unique key so it doesn't conflict with IP101 */
    esp_netif_inherent_config_t w5500_base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    w5500_base.if_key = "ETH_SPI_0";
    w5500_base.if_desc = "eth_spi";
    w5500_base.route_prio = 30; /* lower than IP101 (50) */

    esp_netif_config_t w5500_netif_cfg = {
        .base = &w5500_base,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    esp_netif_t *w5500_netif = esp_netif_new(&w5500_netif_cfg);
    if (w5500_netif == NULL)
    {
        ESP_LOGW(TAG, "W5500 netif create failed — running on IP101 only");
        esp_eth_driver_uninstall(eth_handle);
        mac->del(mac);
        phy->del(phy);
        spi_bus_free(W5500_SPI_HOST);
        return ESP_FAIL;
    }

    /* DHCP client may already be stopped on a static netif; ignore its result. */
    esp_netif_dhcpc_stop(w5500_netif);

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = esp_ip4addr_aton(W5500_STATIC_IP);
    ip_info.gw.addr = esp_ip4addr_aton(W5500_GATEWAY);
    ip_info.netmask.addr = esp_ip4addr_aton(STATIC_NETMASK);

    /* Bring-up steps: graceful so a late failure leaves IP101 untouched rather
     * than aborting the device. */
    err = esp_netif_set_ip_info(w5500_netif, &ip_info);
    if (err == ESP_OK)
        err = esp_netif_attach(w5500_netif, esp_eth_new_netif_glue(eth_handle));
    if (err == ESP_OK)
        err = esp_eth_start(eth_handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "W5500 bring-up failed (0x%x) — running on IP101 only", err);
        return err;
    }

    ESP_LOGI(TAG, "PI LINK (W5500) initialized at %s — ready to reach Pi @ %s", W5500_STATIC_IP, PI_IP);
    return ESP_OK;
}

/* ============================================================
 * PIT TELEMETRY (TCP CLIENT TO THE PI)
 * ============================================================ */

/* Public setter — called from the CAN/decode task with the latest values.
 * Just copies into the shared snapshot; the sender task owns the cadence. */
void network_set_telemetry(const pit_telemetry_t *t)
{
    portENTER_CRITICAL(&pit_mux);
    pit_latest = *t;
    portEXIT_CRITICAL(&pit_mux);
}

/* Append a big-endian IEEE-754 float (matches Python struct '>f'). */
static void put_be_float(uint8_t *p, float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits)); /* ESP32 stores little-endian; reorder below */
    p[0] = (uint8_t)(bits >> 24);
    p[1] = (uint8_t)(bits >> 16);
    p[2] = (uint8_t)(bits >> 8);
    p[3] = (uint8_t)(bits);
}

/* Build the 30-byte '>HIfffffHH' packet relay.py / server.py expect. */
static void build_packet(uint8_t out[PIT_PACKET_SIZE], uint32_t seq, const pit_telemetry_t *t)
{
    out[0] = (uint8_t)(PIT_MAGIC >> 8);
    out[1] = (uint8_t)(PIT_MAGIC);
    out[2] = (uint8_t)(seq >> 24);
    out[3] = (uint8_t)(seq >> 16);
    out[4] = (uint8_t)(seq >> 8);
    out[5] = (uint8_t)(seq);
    put_be_float(&out[6], t->battery_v);
    put_be_float(&out[10], t->battery_a);
    put_be_float(&out[14], t->rpm);
    put_be_float(&out[18], t->temperature);
    put_be_float(&out[22], t->speed);
    out[26] = (uint8_t)(t->fault_code >> 8);
    out[27] = (uint8_t)(t->fault_code);

    uint16_t sum = 0;
    for (int i = 0; i < PIT_PACKET_SIZE - 2; i++)
    {
        sum = (uint16_t)(sum + out[i]);
    }
    out[28] = (uint8_t)(sum >> 8);
    out[29] = (uint8_t)(sum);
}

/* Non-blocking connect bounded by a timeout. A plain blocking connect() to a
 * same-subnet host that isn't answering yet (link still negotiating at boot, or
 * the Pi still booting) stalls for the full TCP SYN timeout — many seconds —
 * before failing. That makes the board look like it never connects. Bounding it
 * lets the task retry promptly and latch onto the Pi the instant it appears. */
static int pi_connect_bounded(int sock, const struct sockaddr_in *dest, int timeout_ms)
{
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int rc = connect(sock, (const struct sockaddr *)dest, sizeof(*dest));
    if (rc == 0)
    {
        fcntl(sock, F_SETFL, flags);
        return 0; /* connected immediately (rare on a non-blocking socket) */
    }
    if (errno != EINPROGRESS)
    {
        /* Hard failure — errno is meaningful (ECONNREFUSED, EHOSTUNREACH, ...) */
        fcntl(sock, F_SETFL, flags);
        return -1;
    }

    /* connect() is in progress: wait (bounded) for the socket to become writable */
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(sock, &wset);
    struct timeval tv = {.tv_sec = timeout_ms / 1000,
                         .tv_usec = (timeout_ms % 1000) * 1000};
    rc = select(sock + 1, NULL, &wset, NULL, &tv);
    fcntl(sock, F_SETFL, flags); /* restore blocking mode for send() */

    if (rc == 0)
    {
        /* SYN went unanswered within the timeout — peer not reachable (ARP/link),
         * NOT a connection refusal. Report it as such instead of stale EINPROGRESS. */
        errno = ETIMEDOUT;
        return -1;
    }
    if (rc < 0)
    {
        return -1; /* select error — errno already set */
    }

    int err = 0;
    socklen_t len = sizeof(err);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0)
    {
        errno = err ? err : errno;
        return -1;
    }
    return 0; /* connected */
}

/* Maintains the TCP connection to the Pi and streams packets at a fixed rate.
 * Runs from boot, independent of any display/CAN traffic — it connects and keeps
 * the link alive even when nothing is being broadcast. Reconnects on any
 * failure; the Pi being down or rebooting is non-fatal. */
static void pit_sender_task(void *pvParameters)
{
    uint32_t seq = 0;

    ESP_LOGI(TAG, "pit_sender started — will connect to Pi %s:%d at boot (independent of display traffic)",
             PI_IP, PI_PORT);

    while (1)
    {
        s_pi_connected = false; /* not connected until the stream loop is entered */

        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(PI_RECONNECT_DELAY_MS));
            continue;
        }

        struct sockaddr_in dest = {
            .sin_family = AF_INET,
            .sin_port = htons(PI_PORT),
            .sin_addr.s_addr = esp_ip4addr_aton(PI_IP),
        };

        if (pi_connect_bounded(sock, &dest, PI_CONNECT_TIMEOUT_MS) < 0)
        {
            ESP_LOGW(TAG, "PI connect %s:%d failed (errno=%d) — retrying in %dms",
                     PI_IP, PI_PORT, errno, PI_RECONNECT_DELAY_MS);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(PI_RECONNECT_DELAY_MS));
            continue;
        }
        /* Disable Nagle's algorithm: each telemetry packet is a fixed 30 bytes
         * sent on a fixed cadence, so there is never more data coming to batch
         * with. Without this, TCP can hold each packet up to ~40ms waiting to
         * coalesce — pure latency for a stream of small, self-contained packets. */
        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        /* Keepalive is a backstop for the *idle* case only. While we are actively
         * streaming there is always unacked data in flight, which suppresses
         * keepalive entirely — so the heartbeat drain below is the real fast-path
         * dead-peer detector. */
        int ka_enable = 1, ka_idle = 2, ka_intvl = 1, ka_cnt = 3;
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &ka_enable, sizeof(ka_enable));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));

        /* Bound send() so a dead link can't wedge the task waiting out the TCP
         * retransmit chain, and give recv() a short timeout so draining the Pi's
         * heartbeat never stalls the send cadence. */
        struct timeval snd_to = {.tv_sec = PI_SEND_TIMEOUT_MS / 1000,
                                 .tv_usec = (PI_SEND_TIMEOUT_MS % 1000) * 1000};
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));
        struct timeval rcv_to = {.tv_sec = 0, .tv_usec = 10 * 1000};
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));

        ESP_LOGI(TAG, "PI CONNECTED %s:%d — streaming telemetry", PI_IP, PI_PORT);
        s_pi_connected = true;

        int64_t last_pi_rx_us = esp_timer_get_time();
        bool hb_seen = false;
        TickType_t next_wake = xTaskGetTickCount();

        while (1)
        {
            /* --- Heartbeat / liveness ---
             * relay.py echoes a byte for every packet it receives. Draining it
             * here detects a clean close (recv == 0) or socket error immediately,
             * and timestamps the peer's liveness for the silent-death case. Until
             * the first byte is seen we DON'T enforce the timeout, so a relay.py
             * that predates the heartbeat still works (falls back to send()/
             * keepalive detection). */
            uint8_t hb[32];
            int hn = recv(sock, hb, sizeof(hb), 0);
            if (hn == 0)
            {
                ESP_LOGW(TAG, "Pi closed connection — reconnecting");
                break;
            }
            if (hn > 0)
            {
                last_pi_rx_us = esp_timer_get_time();
                hb_seen = true;
            }
            else if (errno != EWOULDBLOCK && errno != EAGAIN)
            {
                ESP_LOGW(TAG, "Pi recv error (errno=%d) — reconnecting", errno);
                break;
            }

            if (hb_seen &&
                esp_timer_get_time() - last_pi_rx_us > (int64_t)PI_LIVENESS_TIMEOUT_MS * 1000)
            {
                ESP_LOGW(TAG, "No heartbeat from Pi for %dms — assuming dead, reconnecting",
                         PI_LIVENESS_TIMEOUT_MS);
                break;
            }

            pit_telemetry_t snapshot;
            portENTER_CRITICAL(&pit_mux);
            snapshot = pit_latest;
            portEXIT_CRITICAL(&pit_mux);

            uint8_t packet[PIT_PACKET_SIZE];
            build_packet(packet, seq, &snapshot);

            if (send(sock, packet, sizeof(packet), 0) < 0)
            {
                ESP_LOGW(TAG, "Pi send failed (errno=%d) — reconnecting", errno);
                break;
            }
            seq++;

            /* Fixed-rate cadence: anchoring to an absolute wake time keeps the send
             * interval steady regardless of how long send()/recv() took on a given
             * iteration. vTaskDelay() delays AFTER the work, so its period drifts
             * with the work duration — the source of the inconsistent send rate. */
            xTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PI_SEND_INTERVAL_MS));
        }

        close(sock);
    }
}

/* ============================================================
 * ETHERNET INITIALIZATION
 * Sets up the hardware, static IP, and starts TCP server
 * ============================================================ */
esp_err_t network_init(network_data_callback_t data_cb,
                       network_connect_callback_t connect_cb,
                       network_disconnect_callback_t disconnect_cb)
{
    ESP_LOGI(TAG, "Initializing VEGA link (IP101 native port) server, IP %s", STATIC_IP);

    /* Save the callback functions */
    data_callback = data_cb;
    connect_callback = connect_cb;
    disconnect_callback = disconnect_cb;

    /* Initialize TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create default event loop (for Ethernet events) */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create default Ethernet network interface */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);

    /* --------------------------------------------------------
     * Configure static IP (disable DHCP)
     * -------------------------------------------------------- */
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));

    /* Parse our static IP addresses */
    ip_info.ip.addr = esp_ip4addr_aton(STATIC_IP);
    ip_info.gw.addr = esp_ip4addr_aton(STATIC_GATEWAY);
    ip_info.netmask.addr = esp_ip4addr_aton(STATIC_NETMASK);

    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));

    /* Save IP string for display */
    strncpy(ip_address_str, STATIC_IP, sizeof(ip_address_str));

    /* --------------------------------------------------------
     * Initialize Ethernet MAC and PHY
     * -------------------------------------------------------- */
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    /* ESP32-P4 internal Ethernet MAC */
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    /* IP101 PHY chip (common on ESP32-P4 dev boards like Waveshare) */
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    /* Create Ethernet handle */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
    s_eth_handle_ip101 = eth_handle; /* VEGA-facing handle, for the event handler's labeling */

    /* --------------------------------------------------------
     * Force a FIXED link mode on the IP101 (VEGA-facing) port.
     *
     * Vega's W5500 PHY and this IP101 will not complete auto-negotiation with
     * each other: Vega lights its link LED (it hears us), but the IP101 never
     * declares link, so Vega's connection never lands. Both PHYs link fine with
     * friendlier partners (Vega↔laptop, IP101↔Pi dongle) and two different
     * cables behave identically, so this is a PHY auto-neg interop problem, not
     * wiring. Disabling auto-negotiation and driving a fixed 10 Mbps/half-duplex
     * link sidesteps the failing handshake — the IP101 transmits at a fixed rate
     * and Vega's PHY locks on by parallel detection. 10M/half is the most
     * forgiving mode and is far more than the 14-byte CAN frames need.
     *
     * Order matters: per esp_eth_ioctl docs the driver must be STOPPED (we're
     * post-install, pre-start) and auto-neg must be turned off BEFORE speed/
     * duplex. Non-fatal: if a step fails we log and keep going rather than
     * bootloop — worst case it falls back to the (failing) auto-neg behavior. */
    bool ip101_autonego = false;
    eth_speed_t ip101_speed = ETH_SPEED_10M;
    eth_duplex_t ip101_duplex = ETH_DUPLEX_HALF;
    if (esp_eth_ioctl(eth_handle, ETH_CMD_S_AUTONEGO, &ip101_autonego) != ESP_OK)
        ESP_LOGW(TAG, "VEGA link (IP101): could not disable auto-negotiation");
    if (esp_eth_ioctl(eth_handle, ETH_CMD_S_SPEED, &ip101_speed) != ESP_OK)
        ESP_LOGW(TAG, "VEGA link (IP101): could not force 10Mbps");
    if (esp_eth_ioctl(eth_handle, ETH_CMD_S_DUPLEX_MODE, &ip101_duplex) != ESP_OK)
        ESP_LOGW(TAG, "VEGA link (IP101): could not force half-duplex");
    ESP_LOGI(TAG, "VEGA link (IP101): forced fixed 10Mbps half-duplex (auto-neg OFF) to link Vega's W5500");

    /* Attach Ethernet driver to network interface */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    /* --------------------------------------------------------
     * Register event handlers
     * -------------------------------------------------------- */
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &ip_event_handler, NULL));

    /* Start Ethernet */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    /* --------------------------------------------------------
     * Initialize W5500 SPI Ethernet (second interface)
     * -------------------------------------------------------- */
    if (init_w5500() != ESP_OK)
    {
        ESP_LOGW(TAG, "W5500 unavailable — running on IP101 only");
    }

    /* --------------------------------------------------------
     * Start the pit telemetry sender (TCP client → Pi)
     * -------------------------------------------------------- */
    xTaskCreate(pit_sender_task, "pit_sender", 4096, NULL, 6, NULL);

    /* --------------------------------------------------------
     * Start TCP server in a background task
     * -------------------------------------------------------- */
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Network initialization complete");
    return ESP_OK;
}

/* ============================================================
 * GET IP ADDRESS STRING
 * ============================================================ */
const char *network_get_ip(void)
{
    return ip_address_str;
}

/* ============================================================
 * PI LINK STATUS (for the UI indicator)
 * ============================================================ */
bool network_pi_is_connected(void)
{
    return s_pi_connected;
}
