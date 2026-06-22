/*
 * network.c
 * Ethernet initialization with static IP + TCP server
 *
 * Static IP: 192.168.1.100
 * TCP Port: 5000
 */

#include "network.h"

#include <string.h>
#include <stdint.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
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
#define STATIC_IP "192.168.77.1"     /* ESP32 IP on IP101 (native) port — direct Pi link */
#define STATIC_GATEWAY "192.168.77.2" /* Pi dongle is the only peer on this link */
#define STATIC_NETMASK "255.255.255.0"
#define TCP_PORT 5000

/* W5500 SPI Ethernet (second interface) */
#define W5500_STATIC_IP "192.168.2.100"
#define W5500_GATEWAY "192.168.2.1" /* Laptop IP on W5500 side */

#define W5500_MOSI_GPIO 23
#define W5500_MISO_GPIO 22
#define W5500_SCLK_GPIO 21
#define W5500_CS_GPIO 24
#define W5500_INT_GPIO 25
#define W5500_RST_GPIO 33
#define W5500_SPI_HOST SPI2_HOST
#define W5500_SPI_MHZ 8

/* Pit telemetry → Raspberry Pi Zero 2 W (runs relay.py) reached via its
 * USB-Ethernet adapter on the P4's NATIVE port. The Pi is a TCP SERVER on
 * port 5000; we connect to it as a client and stream a fixed 30-byte packet
 * that relay.py + the laptop's server.py already expect:
 *   '>H I f f f f f H H'  =
 *   magic(0xA55A) seq battery_v battery_a rpm temperature speed fault checksum
 * (all big-endian; checksum = sum of bytes[0..27] & 0xFFFF). */
#define PI_IP "192.168.77.2"
#define PI_PORT 5000
#define PIT_MAGIC 0xA55A
#define PIT_PACKET_SIZE 30
#define PI_SEND_INTERVAL_MS 100 /* 10 Hz — steady cadence for the laptop's seq/loss math */

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

/* Latest telemetry snapshot, written by network_set_telemetry() (CAN task)
 * and read by the pit sender task. Guarded by a lightweight spinlock since
 * the copy is tiny. */
static pit_telemetry_t pit_latest;
static portMUX_TYPE pit_mux = portMUX_INITIALIZER_UNLOCKED;

/* ============================================================
 * ETHERNET EVENT HANDLER
 * Called when Ethernet connects/disconnects
 * ============================================================ */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet cable connected");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet cable disconnected");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet stopped");
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

        /* Convert IP to string and save it */
        snprintf(ip_address_str, sizeof(ip_address_str), IPSTR,
                 IP2STR(&event->ip_info.ip));

        ESP_LOGI(TAG, "Got IP address: %s", ip_address_str);
    }
}

/* ============================================================
 * TCP SERVER TASK
 * Runs in background, accepts connections, receives data.
 *
 * Frame dispatch logic:
 *   - Bytes accumulate in a local buffer.
 *   - If first byte of an accumulated chunk is NOT 0x00, treat the whole
 *     chunk as a null-terminated text command (e.g. __CAM_START__).
 *     All CAN IDs in the spec have 0x00 as their little-endian LSB,
 *     whereas text commands start with printable ASCII (>= 0x20).
 *   - Otherwise consume 13-byte CAN frames from the front of the buffer
 *     one at a time and dispatch each to the callback.
 * ============================================================ */
static void tcp_server_task(void *pvParameters)
{
    uint8_t accum[ACCUM_SIZE];
    int accum_len = 0;
    uint8_t raw[RECV_BUF_SIZE];

    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    ESP_LOGI(TAG, "Starting TCP server on port %d", TCP_PORT);

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

    ESP_LOGI(TAG, "TCP server listening on %s:%d", ip_address_str, TCP_PORT);

    while (1)
    {
        ESP_LOGI(TAG, "Waiting for client connection...");

        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0)
        {
            ESP_LOGE(TAG, "Failed to accept connection");
            continue;
        }

        ESP_LOGI(TAG, "Client connected from %s", inet_ntoa(client_addr.sin_addr));
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
                ESP_LOGI(TAG, "Client disconnected");
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
 * Second network interface — laptop connects via W5500 RJ45
 * ESP32 IP on this interface: 192.168.2.100
 * Set laptop's W5500-side Ethernet adapter to 192.168.2.1
 * ============================================================ */
static esp_err_t init_w5500(void)
{
    gpio_install_isr_service(0);

    spi_bus_config_t buscfg = {
        .mosi_io_num = W5500_MOSI_GPIO,
        .miso_io_num = W5500_MISO_GPIO,
        .sclk_io_num = W5500_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(W5500_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

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

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    esp_err_t err = esp_eth_driver_install(&eth_cfg, &eth_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "W5500 init failed (0x%x) — check wiring. IP101 still active.", err);
        esp_log_level_set("w5500.mac", ESP_LOG_NONE);
        return err;
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

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(w5500_netif));

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = esp_ip4addr_aton(W5500_STATIC_IP);
    ip_info.gw.addr = esp_ip4addr_aton(W5500_GATEWAY);
    ip_info.netmask.addr = esp_ip4addr_aton(STATIC_NETMASK);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(w5500_netif, &ip_info));

    ESP_ERROR_CHECK(esp_netif_attach(w5500_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    ESP_LOGI(TAG, "W5500 initialized at %s", W5500_STATIC_IP);
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

/* Maintains the TCP connection to the Pi and streams packets at a fixed rate.
 * Reconnects on any failure; the Pi being down or rebooting is non-fatal. */
static void pit_sender_task(void *pvParameters)
{
    uint32_t seq = 0;

    while (1)
    {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        struct sockaddr_in dest = {
            .sin_family = AF_INET,
            .sin_port = htons(PI_PORT),
            .sin_addr.s_addr = esp_ip4addr_aton(PI_IP),
        };

        if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) < 0)
        {
            ESP_LOGW(TAG, "Pi connect %s:%d failed (errno=%d) — retrying", PI_IP, PI_PORT, errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "Connected to Pi %s:%d — streaming telemetry", PI_IP, PI_PORT);

        while (1)
        {
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
            vTaskDelay(pdMS_TO_TICKS(PI_SEND_INTERVAL_MS));
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
    ESP_LOGI(TAG, "Initializing Ethernet with static IP: %s", STATIC_IP);

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
    xTaskCreate(pit_sender_task, "pit_sender", 4096, NULL, 4, NULL);

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
