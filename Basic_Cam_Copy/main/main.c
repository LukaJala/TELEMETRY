/*
 * Basic_Cam_Test — OV5647 camera viewfinder on ESP32-P4
 *
 * Pipeline:
 *   OV5647 (RAW8, 800x1280, 2-lane MIPI CSI)
 *     -> CSI controller
 *     -> ISP (demosaic RAW8 -> RGB888)
 *     -> PSRAM camera buffer
 *     -> memcpy to MIPI DSI DPI display frame buffer (JD9365 10.1", 800x1280)
 *
 * Pin assignments (Waveshare ESP32-P4-Module-DEV-KIT):
 *   GPIO 7  — SCCB SDA (camera I2C data)
 *   GPIO 8  — SCCB SCL (camera I2C clock)
 *   GPIO 20 — XCLK    (24 MHz clock to camera)
 *   GPIO 26 — Display backlight (configured in display_config.h)
 *   GPIO 27 — Display reset     (configured in display_config.h)
 *
 * NOTE: The JD9365 driver briefly uses I2C_NUM_1 on GPIO 7/8 for
 * backlight init, then frees those pins.  The camera uses I2C_NUM_0
 * on the same pins, which is safe because display_init() finishes
 * before camera init begins.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_sccb_i2c.h"
#include "esp_lcd_mipi_dsi.h"

#include "display_init.h"
#include "display_config.h"

static const char *TAG = "BasicCamTest";

/* Camera sensor output — OV5647 800x1280 RAW8 mode */
#define CAM_H_RES           800
#define CAM_V_RES           1280

/*
 * MIPI lane bit rate.  OV5647 800x1280 RAW8 mode:
 * IDI clock = 100 MHz, RAW8 over 2 lanes = 400 Mbps/lane.
 */
#define CAM_LANE_BITRATE_MBPS   400

#define CAM_XCLK_PIN        20
#define CAM_SCCB_SDA_IO     7
#define CAM_SCCB_SCL_IO     8

/* OV5647 I2C address (7-bit) */
#define OV5647_ADDR         0x36

/* ------------------------------------------------------------------ */
/* CSI event callbacks                                                 */
/* ------------------------------------------------------------------ */

static bool on_trans_finished(esp_cam_ctlr_handle_t handle,
                               esp_cam_ctlr_trans_t *trans, void *user_data)
{
    return false;
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* -------------------------------------------------------------- */
    /* 1. Display: init hardware, get frame-buffer pointer            */
    /* -------------------------------------------------------------- */
    esp_lcd_panel_handle_t panel = display_init();

    void *disp_fb = NULL;
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &disp_fb));
    ESP_LOGI(TAG, "Display frame buffer @ %p", disp_fb);

    /* Fill display with black while camera warms up */
    const size_t disp_fb_size = LCD_H_RES * LCD_V_RES * (LCD_BIT_PER_PIXEL / 8);
    memset(disp_fb, 0, disp_fb_size);
    esp_cache_msync(disp_fb, disp_fb_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    /* -------------------------------------------------------------- */
    /* 2. Release GPIO7/8 from the JD9365 backlight-init I2C bus     */
    /* -------------------------------------------------------------- */
    gpio_reset_pin(CAM_SCCB_SDA_IO);
    gpio_reset_pin(CAM_SCCB_SCL_IO);

    /* -------------------------------------------------------------- */
    /* 3. XCLK: OV5647 needs 24 MHz before I2C will respond          */
    /* -------------------------------------------------------------- */
    esp_cam_sensor_xclk_handle_t xclk = NULL;
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_allocate(
        ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &xclk));

    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .esp_clock_router_cfg = {
            .xclk_pin     = CAM_XCLK_PIN,
            .xclk_freq_hz = 24000000,
        },
    };
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_start(xclk, &xclk_cfg));
    ESP_LOGI(TAG, "XCLK started on GPIO%d at 24 MHz", CAM_XCLK_PIN);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* -------------------------------------------------------------- */
    /* 4. I2C bus for camera SCCB                                     */
    /* -------------------------------------------------------------- */
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port                = I2C_NUM_0,
        .sda_io_num              = CAM_SCCB_SDA_IO,
        .scl_io_num              = CAM_SCCB_SCL_IO,
        .clk_source              = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt       = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    /* Bus scan */
    ESP_LOGI(TAG, "I2C scan on SDA=GPIO%d SCL=GPIO%d:", CAM_SCCB_SDA_IO, CAM_SCCB_SCL_IO);
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 10) == ESP_OK) {
            ESP_LOGI(TAG, "  device @ 0x%02X", addr);
        }
    }

    /* -------------------------------------------------------------- */
    /* 5. SCCB IO for OV5647 (7-bit addr 0x36)                       */
    /* -------------------------------------------------------------- */
    esp_sccb_io_handle_t sccb_io = NULL;
    sccb_i2c_config_t sccb_cfg = {
        .device_address = OV5647_ADDR,
        .scl_speed_hz   = 100000,
    };
    ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus, &sccb_cfg, &sccb_io));

    /* -------------------------------------------------------------- */
    /* 6. Detect OV5647 via the sensor auto-detect table              */
    /* -------------------------------------------------------------- */
    esp_cam_sensor_config_t sensor_cfg = {
        .sccb_handle = sccb_io,
        .reset_pin   = -1,
        .pwdn_pin    = -1,
        .xclk_pin    = -1,  /* XCLK already started above */
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };

    esp_cam_sensor_device_t *sensor = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; p++) {
        if (p->port != ESP_CAM_SENSOR_MIPI_CSI) continue;
        sensor = p->detect(&sensor_cfg);
        if (sensor) {
            ESP_LOGI(TAG, "Detected sensor: %s", esp_cam_sensor_get_name(sensor));
            break;
        }
    }
    if (!sensor) {
        ESP_LOGE(TAG, "OV5647 not found — check XCLK, SDA/SCL wiring, I2C address (0x36)");
        return;
    }

    /* Apply 800x1280 RAW8 50fps register table (NULL = default) */
    ESP_ERROR_CHECK(esp_cam_sensor_set_format(sensor, NULL));

    /*
     * The OV5647 800x1280 register table ships with manual AGC at max
     * gain (64775) and max exposure (0x0fffff), causing a blown-out
     * white image.  It also has a very low gain ceiling (0x22) and max
     * AEC exposure (0x100 lines) which makes auto mode too dark.
     *
     * Fix: enable auto AEC + auto AGC, raise the ceilings, and set
     * moderate starting values.
     */
    esp_cam_sensor_reg_val_t reg;

    /* 0x3503: bits[6:5] = 0x3 (preserve AEC timing), bits[1:0] = 0 (auto) */
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3503, .value = 0x60 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));

    /* AGC gain ceiling — raise from 0x22 to 0xF8 so auto-gain has room */
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3a18, .value = 0x00 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3a19, .value = 0xF8 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));

    /* AEC max exposure — raise from 0x0100 to 0x0680 (~1664 lines) */
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3a02, .value = 0x06 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3a03, .value = 0x80 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));

    /* Moderate initial gain */
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x350a, .value = 0x00 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x350b, .value = 0x40 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));

    /* Moderate initial exposure (~1024 lines) */
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3500, .value = 0x00 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3501, .value = 0x04 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));
    reg = (esp_cam_sensor_reg_val_t){ .regaddr = 0x3502, .value = 0x00 };
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_REG, &reg));

    ESP_LOGI(TAG, "AEC/AGC set to auto, gain ceiling=0xF8, max exposure=0x0680");

    /* -------------------------------------------------------------- */
    /* 7. ISP: RAW8 (GBRG bayer) -> RGB888                           */
    /* -------------------------------------------------------------- */
    isp_proc_handle_t isp_proc = NULL;
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
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_cfg, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    /* -------------------------------------------------------------- */
    /* 8. CSI controller                                               */
    /* -------------------------------------------------------------- */
    esp_cam_ctlr_handle_t cam_ctlr = NULL;
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
    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_cfg, &cam_ctlr));

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = NULL,
        .on_trans_finished = on_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam_ctlr, &cbs, NULL));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_ctlr));

    /* -------------------------------------------------------------- */
    /* 9. Start streaming — DMA writes directly to display FB         */
    /* -------------------------------------------------------------- */
    int stream_on = 1;
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor,
                                          ESP_CAM_SENSOR_IOC_S_STREAM,
                                          &stream_on));
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_ctlr));

    ESP_LOGI(TAG, "Streaming: %dx%d RAW8 -> RGB888 -> display (zero-copy)",
             CAM_H_RES, CAM_V_RES);

    esp_cam_ctlr_trans_t trans = {};
    int frame_count = 0;
    while (1) {
        /* DMA the ISP output directly into the display frame buffer,
         * eliminating the 3 MB CPU copy that caused choppiness. */
        trans.buffer = disp_fb;
        trans.buflen = disp_fb_size;
        ESP_ERROR_CHECK(esp_cam_ctlr_receive(cam_ctlr, &trans, ESP_CAM_CTLR_MAX_DELAY));

        if (frame_count % 300 == 0) {
            ESP_LOGI(TAG, "Frame %d", frame_count);
        }
        frame_count++;
    }
}
