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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"

/* For TCP sockets */
#include "lwip/sockets.h"

static const char *TAG = "NETWORK";

/* ============================================================
 * CONFIGURATION - Change these if needed
 * ============================================================ */
#define STATIC_IP       "192.168.1.100"   /* ESP32's IP address */
#define STATIC_GATEWAY  "192.168.1.1"     /* Your laptop's IP */
#define STATIC_NETMASK  "255.255.255.0"   /* Subnet mask */
#define TCP_PORT        5000               /* Port to listen on */

/* CAN frame over TCP is exactly 14 bytes: [0x01 magic][CAN_ID 4B][DLC 1B][data 8B]
 * The 0x01 prefix distinguishes binary frames from text commands, which are always
 * printable ASCII (>= 0x20) and therefore can never start with 0x01. */
#define CAN_FRAME_MAGIC 0x01
#define CAN_FRAME_SIZE  14

/* Accumulator buffer: hold up to 8 frames worth of bytes to handle
 * recv() returning multiple frames in one call */
#define ACCUM_SIZE      (CAN_FRAME_SIZE * 8)

/* ============================================================
 * GLOBAL STATE
 * ============================================================ */
static char ip_address_str[16] = "0.0.0.0";
static network_data_callback_t       data_callback       = NULL;
static network_connect_callback_t    connect_callback     = NULL;
static network_disconnect_callback_t disconnect_callback  = NULL;
static esp_netif_t *eth_netif = NULL;

/* ============================================================
 * ETHERNET EVENT HANDLER
 * Called when Ethernet connects/disconnects
 * ============================================================ */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
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
    if (event_id == IP_EVENT_ETH_GOT_IP) {
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
    int     accum_len = 0;
    uint8_t raw[ACCUM_SIZE];

    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    ESP_LOGI(TAG, "Starting TCP server on port %d", TCP_PORT);

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Failed to listen");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on %s:%d", ip_address_str, TCP_PORT);

    while (1) {
        ESP_LOGI(TAG, "Waiting for client connection...");

        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Failed to accept connection");
            continue;
        }

        ESP_LOGI(TAG, "Client connected from %s", inet_ntoa(client_addr.sin_addr));
        accum_len = 0;

        if (connect_callback != NULL) {
            connect_callback();
        }

        while (1) {
            int space = (int)sizeof(raw);
            int len = recv(client_sock, raw, space, 0);

            if (len < 0) {
                ESP_LOGE(TAG, "Receive error");
                break;
            } else if (len == 0) {
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }

            /* Append new bytes to accumulator, guard against overflow */
            if (accum_len + len > (int)sizeof(accum)) {
                ESP_LOGW(TAG, "Accumulator overflow — resetting");
                accum_len = 0;
            }
            memcpy(accum + accum_len, raw, len);
            accum_len += len;

            /* Dispatch all complete units from the front of the accumulator */
            while (accum_len > 0 && data_callback != NULL) {

                if (accum[0] == CAN_FRAME_MAGIC) {
                    /* Binary CAN frame: [0x01][CAN_ID 4B][DLC 1B][data 8B] */
                    if (accum_len < CAN_FRAME_SIZE) {
                        break; /* wait for more bytes */
                    }
                    data_callback(accum, CAN_FRAME_SIZE);
                    accum_len -= CAN_FRAME_SIZE;
                    memmove(accum, accum + CAN_FRAME_SIZE, accum_len);

                } else if (accum[0] >= 0x20) {
                    /* Text command: null-terminated printable ASCII string */
                    int cmd_len = 0;
                    while (cmd_len < accum_len && accum[cmd_len] != '\0') {
                        cmd_len++;
                    }
                    if (cmd_len < accum_len && accum[cmd_len] == '\0') {
                        /* Complete — dispatch */
                        ESP_LOGI(TAG, "Text command: %s", (char *)accum);
                        data_callback(accum, cmd_len);
                        int consumed = cmd_len + 1;
                        accum_len -= consumed;
                        memmove(accum, accum + consumed, accum_len);
                    } else {
                        /* Incomplete — wait for more bytes */
                        break;
                    }

                } else {
                    /* Unknown byte — drop it and resync */
                    ESP_LOGW(TAG, "Unknown framing byte 0x%02X — dropping", accum[0]);
                    accum_len--;
                    memmove(accum, accum + 1, accum_len);
                }
            }
        }

        close(client_sock);

        if (disconnect_callback != NULL) {
            disconnect_callback();
        }
    }

    close(listen_sock);
    vTaskDelete(NULL);
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
    data_callback       = data_cb;
    connect_callback    = connect_cb;
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
     * Start TCP server in a background task
     * -------------------------------------------------------- */
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Network initialization complete");
    return ESP_OK;
}

/* ============================================================
 * GET IP ADDRESS STRING
 * ============================================================ */
const char* network_get_ip(void)
{
    return ip_address_str;
}
