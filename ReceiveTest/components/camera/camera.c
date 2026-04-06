/*
 * camera.c
 * IMX219 MIPI CSI camera streaming to DSI display panel.
 *
 * Pipeline: IMX219 (RAW10, 800x1232) -> CSI -> ISP (-> RGB888) -> frame buffer
 *           -> esp_lcd_panel_draw_bitmap -> DSI display (centered vertically)
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
#include "esp_ldo_regulator.h"
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
#include "esp_lvgl_port.h"

#include "imx219.h"

static const char *TAG = "CAMERA";

/* ============================================================
 * CONFIGURATION
 * ============================================================ */

/* Camera capture resolution — IMX219 center-cropped, 2x2 binned */
#define CAM_H_RES               800
#define CAM_V_RES               1232
#define CAM_FB_SIZE             (CAM_H_RES * CAM_V_RES * 3)  /* RGB888: 3 bytes/pixel */

/* Display panel native resolution (portrait) */
#define DISP_H_RES              800
#define DISP_V_RES              1280

/* Camera width matches display width — no horizontal crop.
 * Vertically center the 1232-row image on the 1280-row display. */
#define Y_OFFSET                ((DISP_V_RES - CAM_V_RES) / 2) /* 24 */

/* SCCB (camera I2C control) pins - Waveshare ESP32-P4-Module-DEV-KIT */
#define CAM_SCCB_SCL_IO         8
#define CAM_SCCB_SDA_IO         7

/* XCLK - IMX219 requires external 24MHz clock */
#define CAM_XCLK_PIN            20
#define CAM_XCLK_FREQ_HZ       24000000

/* MIPI CSI PHY LDO */
#define CAM_MIPI_LDO_CHAN       3
#define CAM_MIPI_LDO_MV        2500

/* CSI lane bitrate: 456 MHz MIPI clock × 2 (DDR) = 912 Mbps/lane */
#define CAM_LANE_BITRATE_MBPS   912

/* ============================================================
 * STATE
 * ============================================================ */
static esp_cam_ctlr_handle_t    s_cam_handle  = NULL;
static isp_proc_handle_t        s_isp_proc    = NULL;
static esp_cam_sensor_device_t *s_sensor      = NULL;   /* Camera sensor handle */
static void                    *s_frame_buf   = NULL;   /* Camera capture buffer */
static volatile bool            s_running     = false;
static TaskHandle_t             s_task        = NULL;
static esp_ldo_channel_handle_t s_ldo         = NULL;
static SemaphoreHandle_t        s_task_ready  = NULL;   /* Signals task is in receive loop */

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
 * INTERNAL: camera streaming task
 * ============================================================ */
static void camera_stream_task(void *arg)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)arg;

    esp_cam_ctlr_trans_t trans = {
        .buffer = s_frame_buf,
        .buflen = CAM_FB_SIZE,
    };

    /*
     * Hold the LVGL lock for the entire streaming session.
     * This blocks LVGL's render task so it won't call draw_bitmap
     * while we are, preventing display corruption.
     */
    lvgl_port_lock(portMAX_DELAY);

    /* Clear black bars at top and bottom using start of frame buffer as scratch */
    memset(s_frame_buf, 0, DISP_H_RES * Y_OFFSET * 3);
    if (Y_OFFSET > 0) {
        esp_lcd_panel_draw_bitmap(panel, 0, 0, DISP_H_RES, Y_OFFSET, s_frame_buf);
        vTaskDelay(pdMS_TO_TICKS(20));  /* Let DMA finish before next draw */
    }
    if (Y_OFFSET + CAM_V_RES < DISP_V_RES) {
        esp_lcd_panel_draw_bitmap(panel, 0, Y_OFFSET + CAM_V_RES,
                                  DISP_H_RES, DISP_V_RES, s_frame_buf);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* Signal that we're ready to receive frames */
    xSemaphoreGive(s_task_ready);

    int frame_count = 0;
    int fail_count = 0;
    while (s_running) {
        /* Block until a frame is ready (500ms timeout so we can check s_running) */
        esp_err_t ret = esp_cam_ctlr_receive(s_cam_handle, &trans, pdMS_TO_TICKS(500));
        if (ret != ESP_OK) {
            fail_count++;
            if (fail_count <= 5 || fail_count % 50 == 0) {
                ESP_LOGW(TAG, "receive failed: err=0x%x (%s), fails=%d",
                         ret, esp_err_to_name(ret), fail_count);
            }
            continue;
        }

        /* No crop needed — camera outputs 800 wide, same as display.
         * Just draw at vertical offset to center on display. */
        esp_lcd_panel_draw_bitmap(panel, 0, Y_OFFSET,
                                  CAM_H_RES, Y_OFFSET + CAM_V_RES, s_frame_buf);

        frame_count++;
        if (frame_count % 100 == 1) {
            ESP_LOGI(TAG, "Frame %d displayed", frame_count);
        }
    }

    /* Release the display back to LVGL */
    lvgl_port_unlock();

    s_task = NULL;
    vTaskDelete(NULL);
}

static bool on_trans_finished(esp_cam_ctlr_handle_t handle,
                               esp_cam_ctlr_trans_t *trans, void *user_data)
{
    return false;
}

/* ============================================================
 * INTERNAL: I2C bus recovery (SMBUS spec / NXP AN10216)
 * If a slave was left mid-transaction it holds SDA low, causing every
 * probe to timeout. Clocking SCL 9 times forces the slave to release SDA.
 * ============================================================ */
static void i2c_recover_bus(int sda_pin, int scl_pin)
{
    /* Read both pins as inputs first to see their true state */
    gpio_config_t both_in_cfg = {
        .pin_bit_mask    = (1ULL << sda_pin) | (1ULL << scl_pin),
        .mode            = GPIO_MODE_INPUT,
        .pull_up_en      = GPIO_PULLUP_ENABLE,
        .pull_down_en    = GPIO_PULLDOWN_DISABLE,
        .intr_type       = GPIO_INTR_DISABLE,
    };
    gpio_config(&both_in_cfg);
    esp_rom_delay_us(100);

    int sda_level = gpio_get_level(sda_pin);
    int scl_level = gpio_get_level(scl_pin);
    ESP_LOGI(TAG, "I2C pin state: SDA(GPIO%d)=%d  SCL(GPIO%d)=%d",
             sda_pin, sda_level, scl_pin, scl_level);

    if (sda_level && scl_level) {
        ESP_LOGI(TAG, "I2C bus already free");
        gpio_reset_pin(sda_pin);
        gpio_reset_pin(scl_pin);
        return;
    }

    if (!scl_level) {
        ESP_LOGE(TAG, "SCL(GPIO%d) stuck LOW — cannot recover in software", scl_pin);
        /* SCL stuck low is a hardware problem: a device is doing clock
         * stretching or SCL is shorted to GND.  Nothing can be done here
         * except report it and let the caller decide. */
        gpio_reset_pin(sda_pin);
        gpio_reset_pin(scl_pin);
        return;
    }

    /* SCL is high but SDA is low — drive SCL open-drain and try recovery */
    gpio_config_t scl_cfg = {
        .pin_bit_mask    = (1ULL << scl_pin),
        .mode            = GPIO_MODE_OUTPUT_OD,
        .pull_up_en      = GPIO_PULLUP_ENABLE,
        .pull_down_en    = GPIO_PULLDOWN_DISABLE,
        .intr_type       = GPIO_INTR_DISABLE,
    };
    gpio_config(&scl_cfg);
    gpio_set_level(scl_pin, 1);

    ESP_LOGW(TAG, "I2C SDA stuck LOW — running 9-clock recovery");
    for (int i = 0; i < 9; i++) {
        gpio_set_level(scl_pin, 0);
        esp_rom_delay_us(5);
        gpio_set_level(scl_pin, 1);
        esp_rom_delay_us(5);
        if (gpio_get_level(sda_pin)) {
            ESP_LOGI(TAG, "SDA released after %d clocks", i + 1);
            break;
        }
    }

    /* Issue STOP condition: SDA low→high while SCL high */
    gpio_config_t sda_out_cfg = {
        .pin_bit_mask    = (1ULL << sda_pin),
        .mode            = GPIO_MODE_OUTPUT_OD,
        .pull_up_en      = GPIO_PULLUP_ENABLE,
        .pull_down_en    = GPIO_PULLDOWN_DISABLE,
        .intr_type       = GPIO_INTR_DISABLE,
    };
    gpio_config(&sda_out_cfg);
    gpio_set_level(scl_pin, 0);
    gpio_set_level(sda_pin, 0);
    esp_rom_delay_us(5);
    gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sda_pin, 1);
    esp_rom_delay_us(5);

    gpio_reset_pin(sda_pin);
    gpio_reset_pin(scl_pin);

    if (gpio_get_level(sda_pin)) {
        ESP_LOGI(TAG, "I2C bus recovery succeeded");
    } else {
        ESP_LOGE(TAG, "I2C bus recovery FAILED — SDA still low");
    }
}

/* ============================================================
 * PUBLIC API
 * ============================================================ */

esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "Initializing IMX219 camera (%dx%d RAW10 -> RGB888)", CAM_H_RES, CAM_V_RES);

    /* Force linker to include IMX219 driver (registers detect function) */
    imx219_force_link();

    /* --- XCLK: IMX219 needs external 24MHz clock before I2C will respond --- */
    xclk_init();
    vTaskDelay(pdMS_TO_TICKS(200));

    /* --- MIPI CSI PHY power (reference counted - OK if DSI already holds it) --- */
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id    = CAM_MIPI_LDO_CHAN,
        .voltage_mv = CAM_MIPI_LDO_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &s_ldo));

    /* --- Release GPIO7/8 from display's I2C bus, then recover if needed --- */
    gpio_reset_pin(CAM_SCCB_SDA_IO);
    gpio_reset_pin(CAM_SCCB_SCL_IO);
    i2c_recover_bus(CAM_SCCB_SDA_IO, CAM_SCCB_SCL_IO);

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

    /* --- I2C bus scan for debugging --- */
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=%d SCL=%d) for devices...", CAM_SCCB_SDA_IO, CAM_SCCB_SCL_IO);
    {
        int found = 0;
        for (uint8_t addr = 0x03; addr < 0x78; addr++) {
            esp_err_t ret = i2c_master_probe(i2c_bus, addr, pdMS_TO_TICKS(50));
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "  I2C device found at 0x%02X", addr);
                found++;
            }
        }
        if (found == 0) {
            ESP_LOGW(TAG, "  No I2C devices found! Check cable and pin connections.");
        }
    }

    /* --- SCCB IO for IMX219 --- */
    esp_sccb_io_handle_t sccb_io = NULL;
    sccb_i2c_config_t sccb_cfg = {
        .device_address = IMX219_I2C_ADDR,
        .scl_speed_hz   = 100000,
    };
    ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus, &sccb_cfg, &sccb_io));

    /* --- Detect IMX219 via the sensor auto-detect table --- */
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
        ESP_LOGE(TAG, "IMX219 not found — check XCLK, SDA/SCL wiring, I2C address (0x%02X)",
                 IMX219_I2C_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    /* Apply default register table (1536x1232 RAW10) */
    ESP_ERROR_CHECK(esp_cam_sensor_set_format(sensor, NULL));
    ESP_LOGI(TAG, "Sensor format set: %dx%d RAW10", CAM_H_RES, CAM_V_RES);

    /* --- CSI controller --- */
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .h_res                  = CAM_H_RES,
        .v_res                  = CAM_V_RES,
        .lane_bit_rate_mbps     = CAM_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW10,
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

    /* --- ISP: RAW10 (GBRG bayer) -> RGB888 --- */
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                  = 80 * 1000 * 1000,
        .input_data_source       = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type   = ISP_COLOR_RAW10,
        .output_data_color_type  = ISP_COLOR_RGB888,
        .has_line_start_packet   = true,
        .has_line_end_packet     = true,
        .h_res                   = CAM_H_RES,
        .v_res                   = CAM_V_RES,
        .bayer_order             = COLOR_RAW_ELEMENT_ORDER_GBRG,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_cfg, &s_isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(s_isp_proc));

    ESP_LOGI(TAG, "Camera ready (IMX219 -> %dx%d RAW10 -> RGB888)",
             CAM_H_RES, CAM_V_RES);
    return ESP_OK;
}

esp_err_t camera_start(esp_lcd_panel_handle_t panel)
{
    if (s_cam_handle == NULL) {
        ESP_LOGE(TAG, "Camera not initialized - cannot start");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting camera stream");

    /* Allocate frame buffers on demand to reduce PSRAM pressure */
    if (!s_frame_buf) {
        s_frame_buf = heap_caps_aligned_calloc(64, 1, CAM_FB_SIZE, MALLOC_CAP_SPIRAM);
        if (!s_frame_buf) {
            ESP_LOGE(TAG, "Camera frame buffer allocation failed (%d bytes)", CAM_FB_SIZE);
            return ESP_ERR_NO_MEM;
        }
    }
    s_running = true;

    /* Create the streaming task first, then wait for it to be ready
     * before starting CSI + sensor.  This prevents CSI queue flooding
     * while the task is still setting up. */
    if (!s_task_ready) {
        s_task_ready = xSemaphoreCreateBinary();
    }
    xTaskCreate(camera_stream_task, "cam_stream", 8192, panel, 6, &s_task);

    /* Wait for task to acquire LVGL lock and be ready to receive */
    xSemaphoreTake(s_task_ready, portMAX_DELAY);

    ESP_ERROR_CHECK(esp_cam_ctlr_start(s_cam_handle));

    /* Start sensor streaming only after CSI controller and receive loop are ready */
    int stream_on = 1;
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on));

    return ESP_OK;
}

esp_err_t camera_stop(void)
{
    if (!s_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping camera stream");
    s_running = false;

    vTaskDelay(pdMS_TO_TICKS(600));

    /* Stop sensor before stopping the CSI controller */
    int stream_off = 0;
    esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_off);

    esp_cam_ctlr_stop(s_cam_handle);

    /* Free frame buffers to reduce PSRAM pressure while not streaming */
    if (s_frame_buf) {
        free(s_frame_buf);
        s_frame_buf = NULL;
    }
    if (s_task_ready) {
        vSemaphoreDelete(s_task_ready);
        s_task_ready = NULL;
    }
    ESP_LOGI(TAG, "Camera stream stopped");
    return ESP_OK;
}
