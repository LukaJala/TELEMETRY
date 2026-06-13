/*
 * camera.c
 * OV5647 MIPI CSI camera streaming to DSI display panel.
 *
 * Pipeline: OV5647 (RAW8, 800x1280) -> CSI -> ISP (RAW8->RGB888) -> DPI frame buffer
 *           Zero-copy: CSI DMA writes directly into the display's frame buffer.
 *
 * When streaming, LVGL is blocked via lvgl_port_lock so the camera
 * task has exclusive access to the display.
 */

#include "camera.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor_xclk.h"
#include "driver/isp.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lvgl_port.h"

static const char *TAG = "CAMERA";

/* ============================================================
 * CONFIGURATION — matches Basic_Cam_Test
 * ============================================================ */

#define CAM_H_RES               800
#define CAM_V_RES               1280
#define CAM_FB_SIZE             (CAM_H_RES * CAM_V_RES * 3)  /* RGB888 */

#define DISP_H_RES              800
#define DISP_V_RES              1280

#define CAM_SCCB_SCL_IO         8
#define CAM_SCCB_SDA_IO         7

#define CAM_XCLK_PIN            20
#define CAM_XCLK_FREQ_HZ       24000000

#define CAM_LANE_BITRATE_MBPS   400

/* Sensor I2C address (both OV5647 and IMX219 use 0x36) */
#define CAM_SENSOR_ADDR         0x36

/* ============================================================
 * STATE
 * ============================================================ */
static esp_cam_ctlr_handle_t    s_cam_handle  = NULL;
static isp_proc_handle_t        s_isp_proc    = NULL;
static esp_cam_sensor_device_t *s_sensor      = NULL;
static volatile bool            s_running     = false;
static TaskHandle_t             s_task        = NULL;
static SemaphoreHandle_t        s_task_ready  = NULL;
static SemaphoreHandle_t        s_task_done   = NULL;

/* ============================================================
 * INTERNAL: XCLK generation via ESP clock router
 * ============================================================ */
static esp_cam_sensor_xclk_handle_t s_xclk = NULL;

static void xclk_init(void)
{
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_allocate(
        ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &s_xclk));

    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .esp_clock_router_cfg = {
            .xclk_pin     = CAM_XCLK_PIN,
            .xclk_freq_hz = CAM_XCLK_FREQ_HZ,
        },
    };
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_start(s_xclk, &xclk_cfg));
    ESP_LOGI(TAG, "XCLK enabled on GPIO %d at %d Hz", CAM_XCLK_PIN, CAM_XCLK_FREQ_HZ);
}

/* ============================================================
 * INTERNAL: camera streaming task (zero-copy)
 * ============================================================ */

typedef struct {
    void *disp_fb;
} cam_task_args_t;

static bool on_trans_finished(esp_cam_ctlr_handle_t handle,
                               esp_cam_ctlr_trans_t *trans, void *user_data)
{
    return false;
}

static void camera_stream_task(void *arg)
{
    cam_task_args_t *ta = (cam_task_args_t *)arg;
    void *disp_fb = ta->disp_fb;
    free(ta);

    esp_cam_ctlr_trans_t trans = {
        .buffer = disp_fb,
        .buflen = CAM_FB_SIZE,
    };

    lvgl_port_lock(portMAX_DELAY);

    memset(disp_fb, 0, CAM_FB_SIZE);

    int stream_on = 1;
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on));
    ESP_ERROR_CHECK(esp_cam_ctlr_start(s_cam_handle));

    xSemaphoreGive(s_task_ready);

    int frame_count = 0;
    ESP_LOGI(TAG, "Entering receive loop, buffer=%p size=%d", disp_fb, CAM_FB_SIZE);
    while (s_running) {
        esp_err_t ret = esp_cam_ctlr_receive(s_cam_handle, &trans, ESP_CAM_CTLR_MAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "receive failed: err=0x%x (%s)", ret, esp_err_to_name(ret));
            continue;
        }

        frame_count++;
        if (frame_count <= 5 || frame_count % 300 == 0) {
            ESP_LOGI(TAG, "Frame %d", frame_count);
        }
    }

    lvgl_port_unlock();

    s_task = NULL;
    xSemaphoreGive(s_task_done);
    vTaskDelete(NULL);
}

/* ============================================================
 * PUBLIC API
 * ============================================================ */

esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "Initializing camera (%dx%d RAW8 -> RGB888)", CAM_H_RES, CAM_V_RES);

    /* --- XCLK --- */
    xclk_init();
    vTaskDelay(pdMS_TO_TICKS(200));

    /* --- Release GPIO7/8 from display's I2C bus --- */
    gpio_reset_pin(CAM_SCCB_SDA_IO);
    gpio_reset_pin(CAM_SCCB_SCL_IO);

    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .sda_io_num             = CAM_SCCB_SDA_IO,
        .scl_io_num             = CAM_SCCB_SCL_IO,
        .i2c_port               = I2C_NUM_0,
        .glitch_ignore_cnt      = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    /* --- I2C bus scan --- */
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=%d SCL=%d)...", CAM_SCCB_SDA_IO, CAM_SCCB_SCL_IO);
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 10) == ESP_OK) {
            ESP_LOGI(TAG, "  I2C device found at 0x%02X", addr);
        }
    }

    /* --- SCCB IO --- */
    esp_sccb_io_handle_t sccb_io = NULL;
    sccb_i2c_config_t sccb_cfg = {
        .device_address = CAM_SENSOR_ADDR,
        .scl_speed_hz   = 100000,
    };
    ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus, &sccb_cfg, &sccb_io));

    /* --- Detect sensor (OV5647 or IMX219 — whichever is connected) --- */
    esp_cam_sensor_config_t sensor_cfg = {
        .sccb_handle = sccb_io,
        .reset_pin   = -1,
        .pwdn_pin    = -1,
        .xclk_pin    = -1,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };

    esp_cam_sensor_device_t *sensor = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; p++) {
        if (p->port != ESP_CAM_SENSOR_MIPI_CSI) continue;
        sensor = p->detect(&sensor_cfg);
        if (sensor) {
            ESP_LOGI(TAG, "Camera sensor detected: %s", sensor->name);
            s_sensor = sensor;
            break;
        }
    }

    if (!sensor) {
        ESP_LOGE(TAG, "No camera sensor found at 0x%02X", CAM_SENSOR_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    /* Apply default format */
    ESP_ERROR_CHECK(esp_cam_sensor_set_format(sensor, NULL));

    /* --- ISP: RAW8 (GBRG bayer) -> RGB888 --- */
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                  = 80 * 1000 * 1000,
        .input_data_source       = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type   = ISP_COLOR_RAW8,
        .output_data_color_type  = ISP_COLOR_RGB888,
        .has_line_start_packet   = true,
        .has_line_end_packet     = true,
        .h_res                   = CAM_H_RES,
        .v_res                   = CAM_V_RES,
        .bayer_order             = COLOR_RAW_ELEMENT_ORDER_GBRG,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_cfg, &s_isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(s_isp_proc));

    /* --- CSI controller --- */
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .clk_src                = MIPI_CSI_PHY_CLK_SRC_DEFAULT,
        .h_res                  = CAM_H_RES,
        .v_res                  = CAM_V_RES,
        .lane_bit_rate_mbps     = CAM_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB888,
        .data_lane_num          = 2,
        .byte_swap_en           = false,
        .queue_items            = 3,
    };
    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_cfg, &s_cam_handle));

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = NULL,
        .on_trans_finished = on_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, NULL));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(s_cam_handle));

    ESP_LOGI(TAG, "Camera ready (%s -> CSI %dx%d RAW8 -> RGB888)",
             sensor->name, CAM_H_RES, CAM_V_RES);
    return ESP_OK;
}

esp_err_t camera_start(esp_lcd_panel_handle_t panel)
{
    if (s_cam_handle == NULL) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting camera stream");

    void *disp_fb = NULL;
    esp_err_t fb_err = esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &disp_fb);
    if (fb_err != ESP_OK || !disp_fb) {
        ESP_LOGE(TAG, "Failed to get DPI frame buffer (err=0x%x)", fb_err);
        return fb_err;
    }
    ESP_LOGI(TAG, "DPI frame buffer @ %p (zero-copy mode)", disp_fb);

    s_running = true;

    if (!s_task_ready) {
        s_task_ready = xSemaphoreCreateBinary();
    }
    if (!s_task_done) {
        s_task_done = xSemaphoreCreateBinary();
    }

    cam_task_args_t *ta = malloc(sizeof(cam_task_args_t));
    ta->disp_fb = disp_fb;
    xTaskCreate(camera_stream_task, "cam_stream", 8192, ta, 6, &s_task);

    xSemaphoreTake(s_task_ready, portMAX_DELAY);

    return ESP_OK;
}

esp_err_t camera_stop(void)
{
    if (!s_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping camera stream");

    /* Signal the task to exit — sensor keeps streaming so receive() returns
     * naturally on the next frame, task checks s_running and breaks out. */
    s_running = false;

    /* Wait for the stream task to actually exit and release LVGL */
    if (s_task_done) {
        xSemaphoreTake(s_task_done, pdMS_TO_TICKS(2000));
    }

    /* Now that the task is gone, safely stop hardware */
    int stream_off = 0;
    esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_off);
    esp_cam_ctlr_stop(s_cam_handle);

    /* Reset CSI controller internal state (flush stale queue entries)
     * so it's ready for a clean start next time */
    esp_cam_ctlr_disable(s_cam_handle);
    esp_cam_ctlr_enable(s_cam_handle);

    if (s_task_ready) {
        vSemaphoreDelete(s_task_ready);
        s_task_ready = NULL;
    }
    if (s_task_done) {
        vSemaphoreDelete(s_task_done);
        s_task_done = NULL;
    }
    ESP_LOGI(TAG, "Camera stream stopped");
    return ESP_OK;
}

bool camera_is_running(void)
{
    return s_running;
}
