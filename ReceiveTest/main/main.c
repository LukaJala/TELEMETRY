// entry point to display data through TCP
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "display_init.h"
#include "display_config.h"
#include "ui.h"
#include "network.h"
#include "camera.h"

static const char *TAG = "ReceiveTest";

/* Panel handle stored globally so the data callback can pass it to camera */
static esp_lcd_panel_handle_t s_panel = NULL;

/* Last telemetry data — shown on display when camera stops */
static char s_last_data[128] = "Waiting...";

/* Commands sent from sender.py to control camera mode */
#define CMD_CAM_START   "__CAM_START__"
#define CMD_CAM_STOP    "__CAM_STOP__"

/*
 * DISCONNECT CALLBACK
 * Called by network component when TCP client disconnects.
 * Resets everything back to the initial waiting state.
 **/
static void on_client_disconnected(void)
{
    ESP_LOGI(TAG, "Client disconnected — resetting to initial state");
    camera_stop();
    ui_refresh();
    ui_set_text(s_last_data);
}

/*
 * DATA CALLBACK
 * Called by network component when TCP data arrives
 **/
static void on_data_received(const char *data, int length)
{
    if (strcmp(data, CMD_CAM_START) == 0) {
        ESP_LOGI(TAG, "Camera start command received");
        if (camera_start(s_panel) != ESP_OK) {
            ui_set_text("Camera unavailable");
        }

    } else if (strcmp(data, CMD_CAM_STOP) == 0) {
        ESP_LOGI(TAG, "Camera stop command received");
        camera_stop();
        /* Force full redraw to clear stale camera frame, show latest data */
        ui_refresh();
        ui_set_text(s_last_data);

    } else {
        /* Always store latest telemetry. If camera is active, it will
         * be shown once the camera stops. */
        strncpy(s_last_data, data, sizeof(s_last_data) - 1);
        s_last_data[sizeof(s_last_data) - 1] = '\0';

        if (!camera_is_running()) {
            ESP_LOGI(TAG, "Updating display with: %s", data);
            ui_set_text(data);
        }
    }
}

// MAIN ENTRY POINT
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ReceiveTest");

    /*
     * Step 1: Initialize display hardware
     * - Turns on backlight
     * - Configures MIPI DSI interface
     * - Returns panel handle for LVGL and camera
     **/
    s_panel = display_init();

    /*
     * Step 2: Initialize LVGL graphics library
     * - Creates background task for rendering
     * - Sets up timers for animations
     **/
    lvgl_port_init(&(lvgl_port_cfg_t)ESP_LVGL_PORT_INIT_CONFIG());

    /*
     * Step 3: Register display with LVGL
     * - Allocates frame buffer in PSRAM (3MB)
     * - Configures resolution and color format
     **/
    lv_display_t *disp = lvgl_port_add_disp_dsi(
        &(lvgl_port_display_cfg_t){
            .panel_handle = s_panel,
            .buffer_size = LCD_H_RES * LCD_V_RES * 3,
            .double_buffer = false,
            .hres = LCD_H_RES,
            .vres = LCD_V_RES,
            .color_format = LV_COLOR_FORMAT_RGB888,
            .flags.buff_spiram = true,
            .flags.sw_rotate = true,
        },
        &(lvgl_port_display_dsi_cfg_t){});

    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

    /*
     * Step 4: Initialize UI
     * - Creates title label
     * - Creates data label (updated when data arrives)
     * - Creates status label (shows IP address)
     **/
    ui_init(disp);

    /*
     * Step 5: Initialize camera (OV5647 via MIPI CSI)
     * - Sets up I2C/SCCB, CSI controller, and ISP
     * - Does not start streaming yet
     **/
    if (camera_init() != ESP_OK) {
        ESP_LOGW(TAG, "Camera init failed - camera mode will not be available");
    }

    /*
     * Step 6: Initialize network
     * - Sets up Ethernet with static IP (192.168.1.100)
     * - Starts TCP server on port 5000
     * - Calls on_data_received() when data arrives
     **/
    network_init(on_data_received, on_client_disconnected);

    /* Update status with IP address */
    char status_msg[64];
    snprintf(status_msg, sizeof(status_msg), "IP: %s  Port: 5000", network_get_ip());
    ui_set_status(status_msg);

    ESP_LOGI(TAG, "System ready - waiting for TCP connections");

    /*
     * Main loop - keeps the task alive
     * All the work happens in background tasks:
     * - LVGL task handles rendering
     * - TCP server task handles network
     * - Camera stream task handles camera (when active)
     **/
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
