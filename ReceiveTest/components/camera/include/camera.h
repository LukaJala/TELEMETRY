#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

/*
 * Initialize the IMX219 camera sensor and MIPI CSI pipeline.
 * Must be called once at startup before camera_start().
 */
esp_err_t camera_init(void);

/*
 * Start streaming camera frames to the display panel.
 * Blocks LVGL rendering for the duration of streaming.
 * panel: the DSI panel handle returned by display_init()
 */
esp_err_t camera_start(esp_lcd_panel_handle_t panel);

/*
 * Stop streaming and hand the display back to LVGL.
 */
esp_err_t camera_stop(void);

/*
 * Returns true if the camera is currently streaming.
 */
bool camera_is_running(void);
