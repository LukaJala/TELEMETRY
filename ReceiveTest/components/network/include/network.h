/*
 * network.h
 * Ethernet and TCP server interface
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Callback function type - called when data is received.
 * For binary CAN frames, data is a raw 13-byte buffer (not null-terminated).
 * For text commands, data is null-terminated and length reflects string length. */
typedef void (*network_data_callback_t)(const uint8_t *data, int length);

/* Callback function type - called when a client disconnects */
typedef void (*network_disconnect_callback_t)(void);

/* Callback function type - called when a client connects */
typedef void (*network_connect_callback_t)(void);

/* Initialize Ethernet with static IP and start TCP server */
esp_err_t network_init(network_data_callback_t data_cb,
                       network_connect_callback_t connect_cb,
                       network_disconnect_callback_t disconnect_cb);

/* Get our IP address as a string (for display) */
const char *network_get_ip(void);
