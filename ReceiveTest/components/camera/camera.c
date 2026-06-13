/*
 * camera.c
 * OV5647 MIPI CSI camera streaming to DSI display panel.
 *
 * Pipeline: OV5647 (RAW10, 1280x960 full-sensor 2x2 binning)
 *             -> CSI -> ISP (RAW10 -> RGB565) -> capture buffer (PSRAM)
 *             -> PPA scale + 90deg rotate -> DPI frame buffer (RGB565)
 *
 * Performance notes (this path is PSRAM-bandwidth bound):
 *   - The whole path is RGB565 (2 B/px): capture, PPA, and the display buffer.
 *     This minimizes PSRAM traffic — both the per-frame camera work and the
 *     constant DPI scan-out (see display_config.h, LCD_BIT_PER_PIXEL=16).
 *   - Two capture buffers are used ping-pong with a NON-blocking PPA: the CSI
 *     captures frame N+1 while the PPA scales/rotates frame N, so the per-frame
 *     time is max(capture, scale) instead of their sum.
 *
 * Why full-sensor 1280x960 + PPA instead of zero-copy:
 *   The OV5647 is a raw sensor with no scaler — it can only crop + integer-bin.
 *   At an 800-px-wide output it can show at most ~62% of the sensor width, so a
 *   direct 800x1280 sensor mode is permanently "zoomed in". The 1280x960 binning
 *   mode reads ~99% of the array (full field of view), and the ESP32-P4 PPA
 *   scales/rotates that frame down into the 800x1280 display buffer.
 *
 * When streaming, LVGL is blocked via lvgl_port_lock so the camera task has
 * exclusive access to the display frame buffer.
 */

#include "camera.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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
#include "driver/ppa.h"
#include "esp_cache.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lvgl_port.h"

static const char *TAG = "CAMERA";

/* ============================================================
 * CONFIGURATION
 * ============================================================ */

/* Sensor capture resolution — full-sensor 1280x960 RAW10 2x2 binning mode */
#define SENS_H_RES 1280
#define SENS_V_RES 960
#define SENS_BPP 2 /* capture in RGB565 to halve+ PSRAM traffic vs RGB888 */
#define CAP_FB_SIZE (SENS_H_RES * SENS_V_RES * SENS_BPP)
#define CAP_BUF_COUNT 2 /* ping-pong: capture frame N+1 while PPA scales frame N */

/* Display frame buffer (panel native portrait; LVGL presents it rotated).
 * RGB565 to match the display (see display_config.h) — keeps the whole
 * capture->scale->display path at 2 B/px. */
#define DISP_H_RES 800
#define DISP_V_RES 1280
#define DISP_FB_SIZE (DISP_H_RES * DISP_V_RES * 2) /* RGB565 */

/* MIPI lane rate for the 1280x960 RAW10 binning mode: IDI 88.33MHz * 5 ≈ 442 Mbps */
#define CAM_LANE_BITRATE_MBPS 442

/* ---- PPA scale/rotate knobs (tune on the bench if orientation is off) ----
 * The 1280x960 sensor frame is rotated 90deg CCW to fill the 800x1280 portrait
 * buffer. scale_x maps sensor width (1280) -> output height (1280) = 1.0.
 * scale_y maps sensor height (960) -> output width: 960*0.8125 = 780, centered
 * in the 800-wide buffer (~10px black bar each side). 0.8125 = 13/16 lands
 * exactly on the PPA's 1/16 scaling grid, so the whole sensor FOV is preserved
 * with no scaler rounding. If the feed is upside-down/mirrored, change the
 * rotation angle (90 <-> 270) or the mirror flags. */
#define CAM_PPA_ROTATION PPA_SRM_ROTATION_ANGLE_90
#define CAM_PPA_MIRROR_X false
#define CAM_PPA_MIRROR_Y false
#define CAM_PPA_SCALE_X (1.0f)    /* 16/16 */
#define CAM_PPA_SCALE_Y (0.8125f) /* 13/16 */

/* ---- Auto-exposure tuning (fixes bright-light wash-out) ----
 * The 1280x960 mode runs the sensor's auto-exposure with a fairly bright AE
 * target (driver default 0x50/80) and a maxed-out gain ceiling (0x3ff). In
 * bright scenes that blows highlights to white. Lowering the AE target makes
 * the AE aim darker so highlights are preserved; clamping the gain ceiling
 * stops the AGC from over-amplifying. Tune CAM_AE_TARGET DOWN if it still
 * washes out, UP if it's now too dark (valid range 2..235). */
#define CAM_AE_TARGET 0x34     /* 52/255 (driver default 0x50) */
#define CAM_GAIN_CEILING 0x01F8 /* ~half of the mode's 0x3FF ceiling */

#define CAM_SCCB_SCL_IO 8
#define CAM_SCCB_SDA_IO 7

#define CAM_XCLK_PIN 20
#define CAM_XCLK_FREQ_HZ 24000000

/* Sensor I2C address (both OV5647 and IMX219 use 0x36) */
#define CAM_SENSOR_ADDR 0x36

/* ============================================================
 * STATE
 * ============================================================ */
static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static isp_proc_handle_t s_isp_proc = NULL;
static esp_cam_sensor_device_t *s_sensor = NULL;
static ppa_client_handle_t s_ppa = NULL;
static void *s_cap_buf[CAP_BUF_COUNT] = {NULL}; /* RGB565 capture buffers (PSRAM) */
static SemaphoreHandle_t s_ppa_done = NULL;     /* signalled when a PPA transfer completes */
static volatile bool s_running = false;
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_task_ready = NULL;
static SemaphoreHandle_t s_task_done = NULL;

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
            .xclk_pin = CAM_XCLK_PIN,
            .xclk_freq_hz = CAM_XCLK_FREQ_HZ,
        },
    };
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_start(s_xclk, &xclk_cfg));
    ESP_LOGI(TAG, "XCLK enabled on GPIO %d at %d Hz", CAM_XCLK_PIN, CAM_XCLK_FREQ_HZ);
}

/* ============================================================
 * INTERNAL: camera streaming task
 *   ping-pong capture + non-blocking PPA scale/rotate into display buffer
 * ============================================================ */

typedef struct
{
    void *disp_fb;
} cam_task_args_t;

static bool on_trans_finished(esp_cam_ctlr_handle_t handle,
                              esp_cam_ctlr_trans_t *trans, void *user_data)
{
    return false;
}

/* PPA transaction-done callback (ISR context): release the stream task. */
static bool ppa_trans_done_cb(ppa_client_handle_t ppa_client,
                              ppa_event_data_t *edata, void *user_data)
{
    BaseType_t hp_task_woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_data, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

static void camera_stream_task(void *arg)
{
    cam_task_args_t *ta = (cam_task_args_t *)arg;
    void *disp_fb = ta->disp_fb;
    free(ta);

    /* Output footprint after 90deg rotation: width = scale_y*960, height = scale_x*1280.
     * Center the (possibly narrower) block horizontally so any unused columns are
     * symmetric black bars rather than a one-sided gap. */
    uint32_t out_w = (uint32_t)(CAM_PPA_SCALE_Y * SENS_V_RES);
    uint32_t off_x = (DISP_H_RES > out_w) ? (DISP_H_RES - out_w) / 2 : 0;

    ppa_srm_oper_config_t srm = {
        .in = {
            .buffer = NULL, /* set per frame to the current capture buffer */
            .pic_w = SENS_H_RES,
            .pic_h = SENS_V_RES,
            .block_w = SENS_H_RES,
            .block_h = SENS_V_RES,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = disp_fb,
            .buffer_size = DISP_FB_SIZE,
            .pic_w = DISP_H_RES,
            .pic_h = DISP_V_RES,
            .block_offset_x = off_x,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = CAM_PPA_ROTATION,
        .scale_x = CAM_PPA_SCALE_X,
        .scale_y = CAM_PPA_SCALE_Y,
        .mirror_x = CAM_PPA_MIRROR_X,
        .mirror_y = CAM_PPA_MIRROR_Y,
        .mode = PPA_TRANS_MODE_NON_BLOCKING,
        .user_data = s_ppa_done,
    };

    esp_cam_ctlr_trans_t trans = {
        .buffer = NULL,
        .buflen = CAP_FB_SIZE,
    };

    lvgl_port_lock(portMAX_DELAY);

    /* Clear to black and flush to PSRAM — PPA only writes the centered image
     * block, so the side bars must be valid in memory for the DPI scan-out. */
    memset(disp_fb, 0, DISP_FB_SIZE);
    esp_cache_msync(disp_fb, DISP_FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    int stream_on = 1;
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on));
    ESP_ERROR_CHECK(esp_cam_ctlr_start(s_cam_handle));

    xSemaphoreGive(s_task_ready);

    ESP_LOGI(TAG, "Entering receive loop: cap=%dx%d RGB565 (x%d) -> disp=%dx%d, out_block_w=%u off_x=%u",
             SENS_H_RES, SENS_V_RES, CAP_BUF_COUNT, DISP_H_RES, DISP_V_RES,
             (unsigned)out_w, (unsigned)off_x);

    /* Prime: capture the first frame into buffer 0. */
    int cur = 0;
    trans.buffer = s_cap_buf[cur];
    esp_err_t ret = esp_cam_ctlr_receive(s_cam_handle, &trans, ESP_CAM_CTLR_MAX_DELAY);

    int frame_count = 0;
    while (s_running)
    {
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "receive failed: err=0x%x (%s)", ret, esp_err_to_name(ret));
            trans.buffer = s_cap_buf[cur];
            ret = esp_cam_ctlr_receive(s_cam_handle, &trans, ESP_CAM_CTLR_MAX_DELAY);
            continue;
        }

        /* Kick off the scale/rotate of the just-captured frame (non-blocking). */
        srm.in.buffer = s_cap_buf[cur];
        esp_err_t pret = ppa_do_scale_rotate_mirror(s_ppa, &srm);

        /* Capture the next frame into the other buffer while the PPA runs. */
        int nxt = cur ^ 1;
        trans.buffer = s_cap_buf[nxt];
        ret = esp_cam_ctlr_receive(s_cam_handle, &trans, ESP_CAM_CTLR_MAX_DELAY);

        /* Wait for the PPA to finish with s_cap_buf[cur] before it gets reused. */
        if (pret == ESP_OK)
        {
            xSemaphoreTake(s_ppa_done, portMAX_DELAY);
        }
        else
        {
            ESP_LOGW(TAG, "PPA SRM failed: err=0x%x (%s)", pret, esp_err_to_name(pret));
        }

        cur = nxt;
        frame_count++;
        if (frame_count <= 5 || frame_count % 300 == 0)
        {
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
    ESP_LOGI(TAG, "Initializing camera (%dx%d RAW10 -> RGB565, full FOV)", SENS_H_RES, SENS_V_RES);

    /* --- XCLK --- */
    xclk_init();
    vTaskDelay(pdMS_TO_TICKS(200));

    /* --- Release GPIO7/8 from display's I2C bus --- */
    gpio_reset_pin(CAM_SCCB_SDA_IO);
    gpio_reset_pin(CAM_SCCB_SCL_IO);

    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CAM_SCCB_SDA_IO,
        .scl_io_num = CAM_SCCB_SCL_IO,
        .i2c_port = I2C_NUM_0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    /* --- I2C bus scan --- */
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=%d SCL=%d)...", CAM_SCCB_SDA_IO, CAM_SCCB_SCL_IO);
    for (uint8_t addr = 0x03; addr < 0x78; addr++)
    {
        if (i2c_master_probe(i2c_bus, addr, 10) == ESP_OK)
        {
            ESP_LOGI(TAG, "  I2C device found at 0x%02X", addr);
        }
    }

    /* --- SCCB IO --- */
    esp_sccb_io_handle_t sccb_io = NULL;
    sccb_i2c_config_t sccb_cfg = {
        .device_address = CAM_SENSOR_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus, &sccb_cfg, &sccb_io));

    /* --- Detect sensor (OV5647 or IMX219 — whichever is connected) --- */
    esp_cam_sensor_config_t sensor_cfg = {
        .sccb_handle = sccb_io,
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };

    esp_cam_sensor_device_t *sensor = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; p++)
    {
        if (p->port != ESP_CAM_SENSOR_MIPI_CSI)
            continue;
        sensor = p->detect(&sensor_cfg);
        if (sensor)
        {
            ESP_LOGI(TAG, "Camera sensor detected: %s", sensor->name);
            s_sensor = sensor;
            break;
        }
    }

    if (!sensor)
    {
        ESP_LOGE(TAG, "No camera sensor found at 0x%02X", CAM_SENSOR_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    /* --- Select the full-sensor 1280x960 format explicitly (widest FOV) --- */
    esp_cam_sensor_format_array_t fmts = {0};
    ESP_ERROR_CHECK(esp_cam_sensor_query_format(sensor, &fmts));
    const esp_cam_sensor_format_t *chosen = NULL;
    for (uint32_t i = 0; i < fmts.count; i++)
    {
        if (fmts.format_array[i].width == SENS_H_RES &&
            fmts.format_array[i].height == SENS_V_RES)
        {
            chosen = &fmts.format_array[i];
            break;
        }
    }
    if (!chosen)
    {
        ESP_LOGE(TAG, "No %dx%d sensor format available — is the 1280x960 binning "
                      "mode enabled in menuconfig (CAMERA_OV5647_MIPI_RAW10_1280X960_BINNING_45FPS)?",
                 SENS_H_RES, SENS_V_RES);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_ERROR_CHECK(esp_cam_sensor_set_format(sensor, chosen));
    ESP_LOGI(TAG, "Sensor format: %s", chosen->name);

    /* --- Exposure tuning: lower AE target + clamp gain ceiling so bright
     * scenes don't wash out (see CAM_AE_TARGET / CAM_GAIN_CEILING above) --- */
    int ae_target = CAM_AE_TARGET;
    esp_err_t ae_ret = esp_cam_sensor_set_para_value(
        sensor, ESP_CAM_SENSOR_EXPOSURE_VAL, &ae_target, sizeof(ae_target));
    if (ae_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to set AE target: 0x%x (%s)", ae_ret, esp_err_to_name(ae_ret));
    }
    /* Gain ceiling: 0x3A18 = high byte, 0x3A19 = low byte */
    esp_sccb_transmit_reg_a16v8(sccb_io, 0x3A18, (CAM_GAIN_CEILING >> 8) & 0xFF);
    esp_sccb_transmit_reg_a16v8(sccb_io, 0x3A19, CAM_GAIN_CEILING & 0xFF);
    ESP_LOGI(TAG, "Exposure tuned: AE target=0x%02X, gain ceiling=0x%03X",
             ae_target, CAM_GAIN_CEILING);

    /* --- Capture buffers (cache-line aligned for CSI/PPA DMA) --- */
    for (int i = 0; i < CAP_BUF_COUNT; i++)
    {
        s_cap_buf[i] = heap_caps_aligned_calloc(128, 1, CAP_FB_SIZE, MALLOC_CAP_SPIRAM);
        if (!s_cap_buf[i])
        {
            ESP_LOGE(TAG, "Failed to allocate capture buffer %d (%d bytes)", i, CAP_FB_SIZE);
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "Allocated %d capture buffers @ %d bytes each", CAP_BUF_COUNT, CAP_FB_SIZE);

    /* --- ISP: RAW10 (GBRG bayer) -> RGB565 --- */
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW10,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = true,
        .has_line_end_packet = true,
        .h_res = SENS_H_RES,
        .v_res = SENS_V_RES,
        .bayer_order = COLOR_RAW_ELEMENT_ORDER_GBRG,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_cfg, &s_isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(s_isp_proc));

    /* --- CSI controller --- */
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0,
        .clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT,
        .h_res = SENS_H_RES,
        .v_res = SENS_V_RES,
        .lane_bit_rate_mbps = CAM_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW10,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 3,
    };
    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_cfg, &s_cam_handle));

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = NULL,
        .on_trans_finished = on_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, NULL));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(s_cam_handle));

    /* --- PPA client for scale + rotate into the (RGB565) display buffer --- */
    s_ppa_done = xSemaphoreCreateBinary();
    if (!s_ppa_done)
    {
        ESP_LOGE(TAG, "Failed to create PPA-done semaphore");
        return ESP_ERR_NO_MEM;
    }
    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &s_ppa));
    ppa_event_callbacks_t ppa_cbs = {
        .on_trans_done = ppa_trans_done_cb,
    };
    ESP_ERROR_CHECK(ppa_client_register_event_callbacks(s_ppa, &ppa_cbs));

    ESP_LOGI(TAG, "Camera ready (%s -> CSI %dx%d RAW10 -> RGB565 -> PPA -> %dx%d RGB565)",
             sensor->name, SENS_H_RES, SENS_V_RES, DISP_H_RES, DISP_V_RES);
    return ESP_OK;
}

esp_err_t camera_start(esp_lcd_panel_handle_t panel)
{
    if (s_cam_handle == NULL)
    {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_running)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting camera stream");

    void *disp_fb = NULL;
    esp_err_t fb_err = esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &disp_fb);
    if (fb_err != ESP_OK || !disp_fb)
    {
        ESP_LOGE(TAG, "Failed to get DPI frame buffer (err=0x%x)", fb_err);
        return fb_err;
    }
    ESP_LOGI(TAG, "DPI frame buffer @ %p", disp_fb);

    s_running = true;

    if (!s_task_ready)
    {
        s_task_ready = xSemaphoreCreateBinary();
    }
    if (!s_task_done)
    {
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
    if (!s_running)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping camera stream");

    /* Signal the task to exit — sensor keeps streaming so receive() returns
     * naturally on the next frame, task checks s_running and breaks out. */
    s_running = false;

    /* Wait for the stream task to actually exit and release LVGL */
    if (s_task_done)
    {
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

    if (s_task_ready)
    {
        vSemaphoreDelete(s_task_ready);
        s_task_ready = NULL;
    }
    if (s_task_done)
    {
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
