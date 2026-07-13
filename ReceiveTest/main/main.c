// entry point to display data through TCP
#include <stdio.h>
#include <string.h>
#include <stdint.h>
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
#include "display_can_spec.h"

static const char *TAG = "ReceiveTest";

/* Panel handle stored globally so the data callback can pass it to camera */
static esp_lcd_panel_handle_t s_panel = NULL;

/* Decoded CAN state — updated by every incoming CAN frame */
static display_data_t s_can_data;

/* Commands sent from broadcaster.py / sender.py.
 * The backup camera is always live in a PiP window; CAM_START/CAM_STOP force
 * it fullscreen / back to PiP (same effect as shifting in/out of reverse). */
#define CMD_CAM_START "__CAM_START__"
#define CMD_CAM_STOP "__CAM_STOP__"
#define CMD_TAB_0 "__TAB_0__"
#define CMD_TAB_1 "__TAB_1__"
#define CMD_TAB_2 "__TAB_2__"

/* Size of a binary CAN-over-TCP frame: [0x01 magic][CAN_ID 4B][DLC 1B][data 8B] */
#define CAN_FRAME_SIZE 14

/* Vega_DriverInputs drive_mode encoding (display_can_spec.h): 0=Park 1=FWD 2=REV */
#define DRIVE_MODE_REVERSE 2

/*
 * CONNECT CALLBACK
 * Called by network component when a TCP client connects.
 **/
static void on_client_connected(void)
{
    ESP_LOGI(TAG, "Client connected");
    /* Begin every session from a known state: camera in PiP unless the new
     * data source says otherwise (reverse gear re-engages fullscreen within
     * one frame of the first Vega_DriverInputs message). */
    camera_set_fullscreen_manual(false);
    ui_set_status("Connected");
}

/*
 * DISCONNECT CALLBACK
 * Called by network component when TCP client disconnects.
 * Resets everything back to the initial waiting state.
 **/
static void on_client_disconnected(void)
{
    ESP_LOGI(TAG, "Client disconnected — resetting to initial state");

    /* Data source is gone — no more drive-mode frames will arrive, so drop
     * any fullscreen back to PiP. The camera itself keeps running. */
    camera_set_fullscreen_manual(false);
    camera_set_reverse(false);

    /* Wait (bounded) for the stream task to actually leave fullscreen and
     * release the LVGL lock — otherwise the zero-out below can't paint and
     * stale values would survive under the repaint. */
    for (int i = 0; i < 50 && camera_is_fullscreen(); i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Data source (Vega/laptop via W5500) is gone — blank every value to 0 so a
     * stale reading can't be mistaken for live telemetry. Zeroing the struct and
     * forcing every update flag makes ui_update_can_data repaint all widgets; with
     * fault_code 0 the fault overlay clears and all status dots go dark too. */
    memset(&s_can_data, 0, sizeof(s_can_data));
    s_can_data.update_flags = 0xFFFFFFFF;
    ui_update_can_data(&s_can_data);
    s_can_data.update_flags = 0;

    ui_refresh();
    ui_set_text("DISCONNECTED — no data");
}

/*
 * DATA CALLBACK
 * Called by network component when TCP data arrives.
 *
 * Two frame types:
 *   - Text command (first byte >= 0x20): __CAM_START__ or __CAM_STOP__
 *   - Binary CAN frame (13 bytes, first byte == 0x00): route through
 *     display_route_frame() which decodes into s_can_data
 */
static void on_data_received(const uint8_t *data, int length)
{
    /* Text command path */
    if (length > 0 && data[0] >= 0x20)
    {
        const char *cmd = (const char *)data;

        if (strcmp(cmd, CMD_CAM_START) == 0)
        {
            ESP_LOGI(TAG, "Camera fullscreen ON command received");
            /* Normally the camera is already streaming (PiP); this also
             * doubles as a bench recovery path if the boot-time start failed. */
            if (!camera_is_running() && camera_start(s_panel) != ESP_OK)
            {
                ui_set_text("Camera unavailable");
            }
            camera_set_fullscreen_manual(true);
        }
        else if (strcmp(cmd, CMD_CAM_STOP) == 0)
        {
            ESP_LOGI(TAG, "Camera fullscreen OFF command received");
            /* Back to PiP — the stream task repaints the dashboard on exit */
            camera_set_fullscreen_manual(false);
        }
        else if (strcmp(cmd, CMD_TAB_0) == 0)
        {
            ui_set_tab(0);
        }
        else if (strcmp(cmd, CMD_TAB_1) == 0)
        {
            ui_set_tab(1);
        }
        else if (strcmp(cmd, CMD_TAB_2) == 0)
        {
            ui_set_tab(2);
        }
        return;
    }

    /* Binary CAN frame path */
    if (length != CAN_FRAME_SIZE)
    {
        ESP_LOGW(TAG, "Unexpected frame length %d — dropping", length);
        return;
    }

    /* Unpack the TCP envelope: [0x01 magic][CAN_ID 4B LE][DLC 1B][payload 8B] */
    uint32_t can_id = (uint32_t)data[1] | ((uint32_t)data[2] << 8) | ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
    /* data[5] is DLC — not needed by the router, payload is always 8B */
    const uint8_t *payload = &data[6];

    display_route_frame(&s_can_data, can_id, payload);

    ESP_LOGD(TAG, "CAN frame 0x%08lX decoded (flags=0x%lX)",
             (unsigned long)can_id, (unsigned long)s_can_data.update_flags);

    /* Reverse gear puts the backup camera fullscreen; any other mode returns
     * it to the PiP window. The stream task switches at the next frame. */
    if (s_can_data.update_flags & DFLAG_VEGA_INPUTS)
    {
        camera_set_reverse(s_can_data.vega_inputs.drive_mode == DRIVE_MODE_REVERSE);
    }

    /* While fullscreen, the camera owns the display — skip widget updates
     * (they'd only time out on the LVGL lock; state still accumulates in
     * s_can_data and repaints when the dashboard comes back). */
    if (!camera_is_fullscreen())
    {
        ui_update_can_data(&s_can_data);
    }

    /* Mirror the decoded state out to the Pi (relay.py) for the pit screen.
     * Sent every frame; the network sender task forwards it at a steady rate. */
    pit_telemetry_t pit = {
        .battery_v = s_can_data.bps_power.pack_voltage_v,
        .battery_a = s_can_data.bps_power.pack_current_a,
        .rpm = (float)s_can_data.mc_status1.speed_rpm,
        .temperature = (float)s_can_data.mc_status2.motor_temp_c,
        .speed = s_can_data.vega_status.speed_kmh,
        .fault_code = (uint16_t)s_can_data.bps_safety.fault_code,
    };
    network_set_telemetry(&pit);

    s_can_data.update_flags = 0;
}

// MAIN ENTRY POINT
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ReceiveTest");
    memset(&s_can_data, 0, sizeof(s_can_data));

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
            .buffer_size = LCD_H_RES * LCD_V_RES * (LCD_BIT_PER_PIXEL / 8),
            .double_buffer = false,
            .hres = LCD_H_RES,
            .vres = LCD_V_RES,
            .color_format = LV_COLOR_FORMAT_RGB565,
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
     * Step 5: Initialize camera (OV5647 via MIPI CSI) and start streaming
     * - Sets up I2C/SCCB, CSI controller, and ISP
     * - Streams continuously from boot: PiP window on the dashboard,
     *   fullscreen whenever the car is in reverse (or __CAM_START__)
     **/
    if (camera_init() == ESP_OK)
    {
        uint32_t pip_w = 0, pip_h = 0;
        void *pip_buf = ui_cam_pip_buffer(&pip_w, &pip_h);
        if (pip_buf != NULL &&
            camera_config_pip(pip_buf, pip_w, pip_h, ui_cam_pip_frame_ready) == ESP_OK &&
            camera_start(s_panel) == ESP_OK)
        {
            ESP_LOGI(TAG, "Backup camera live: PiP %lux%lu, fullscreen in reverse",
                     (unsigned long)pip_w, (unsigned long)pip_h);
        }
        else
        {
            ESP_LOGW(TAG, "Camera PiP unavailable - hiding camera window");
            ui_cam_pip_set_active(false);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Camera init failed - hiding camera window");
        ui_cam_pip_set_active(false);
    }

    /*
     * Step 6: Initialize network
     * - Sets up Ethernet with static IP (192.168.1.100)
     * - Starts TCP server on port 5000
     * - Calls on_data_received() when data arrives
     **/
    network_init(on_data_received, on_client_connected, on_client_disconnected);

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
     * - Camera stream task runs continuously (PiP window, or fullscreen
     *   while in reverse)
     **/
    while (1)
    {
        /* Drive the bottom-left Pi telemetry-link indicator. ui_set_pi_status()
         * de-dupes and is lock-safe, so an unconditional poll here is cheap. */
        ui_set_pi_status(network_pi_is_connected());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
