#pragma once

#include "esp_cam_sensor.h"

#define IMX219_I2C_ADDR 0x36

void imx219_force_link(void);

esp_err_t imx219_set_gain(esp_cam_sensor_device_t *dev, uint8_t gain);
esp_err_t imx219_set_exposure(esp_cam_sensor_device_t *dev, uint16_t exposure);
