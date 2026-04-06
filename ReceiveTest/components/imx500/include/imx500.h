#pragma once

#include "esp_cam_sensor.h"

/* IMX500 7-bit I2C address (Raspberry Pi AI Camera) */
#define IMX500_I2C_ADDR  0x1A

/* Force the linker to include this driver (call from camera_init) */
void imx500_force_link(void);

/* Returns the sensor device handle after detect, for manual gain/exposure */
esp_cam_sensor_device_t *imx500_get_global_dev(void);
