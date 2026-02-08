/*
 * network.h
 * Ethernet and TCP server interface
 */
#pragma once

#include "esp_err.h"

/* Callback function type - called when data is received */
typedef void (*network_data_callback_t)(const char *data, int length);

/* Initialize Ethernet with static IP and start TCP server */
esp_err_t network_init(network_data_callback_t callback);

/* Get our IP address as a string (for display) */
const char* network_get_ip(void);
