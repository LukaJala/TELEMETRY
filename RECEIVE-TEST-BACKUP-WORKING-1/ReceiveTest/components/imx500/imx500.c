#include <string.h>
#include <stdlib.h>
#include "imx500.h"
#include "imx500_regs.h"
#include "esp_log.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imx500";

/* Sony IMX500 model ID — read from registers 0x0016 (MSB) and 0x0017 (LSB) */
#define IMX500_PID  0x0500

static const esp_cam_sensor_format_t imx500_format_2028x1520;

void imx500_force_link(void) { /* forces linker to include this object */ }

/* ------------------------------------------------------------------ */
/* Register helpers                                                     */
/* ------------------------------------------------------------------ */

static esp_err_t imx500_write_reg(esp_cam_sensor_device_t *dev,
                                   uint16_t reg, uint8_t val)
{
    return esp_sccb_transmit_reg_a16v8(dev->sccb_handle, reg, val);
}

static esp_err_t imx500_read_reg(esp_cam_sensor_device_t *dev,
                                  uint16_t reg, uint8_t *val)
{
    return esp_sccb_transmit_receive_reg_a16v8(dev->sccb_handle, reg, val);
}

/* ------------------------------------------------------------------ */
/* Sensor ops                                                           */
/* ------------------------------------------------------------------ */

static esp_err_t imx500_set_format(esp_cam_sensor_device_t *dev,
                                    const esp_cam_sensor_format_t *format)
{
    ESP_LOGI(TAG, "imx500_set_format: %s", format ? format->name : "(null)");

    if (!format) {
        format = dev->cur_format ? dev->cur_format : &imx500_format_2028x1520;
    }

    int num_regs = sizeof(imx500_2028x1520_30fps) / sizeof(imx500_reg_t);
    for (int i = 0; i < num_regs; i++) {
        if (imx500_write_reg(dev, imx500_2028x1520_30fps[i].reg,
                              imx500_2028x1520_30fps[i].val) != ESP_OK) {
            ESP_LOGE(TAG, "Failed writing reg 0x%04X", imx500_2028x1520_30fps[i].reg);
            return ESP_FAIL;
        }
    }

    dev->cur_format = format;
    return ESP_OK;
}

static esp_err_t imx500_get_format(esp_cam_sensor_device_t *dev,
                                    esp_cam_sensor_format_t *format)
{
    const esp_cam_sensor_format_t *src =
        dev->cur_format ? dev->cur_format : &imx500_format_2028x1520;
    memcpy(format, src, sizeof(esp_cam_sensor_format_t));
    return ESP_OK;
}

static esp_err_t imx500_query_support_formats(esp_cam_sensor_device_t *dev,
                                               esp_cam_sensor_format_array_t *parry)
{
    parry->count        = 1;
    parry->format_array = &imx500_format_2028x1520;
    return ESP_OK;
}

static int imx500_get_para_value(esp_cam_sensor_device_t *dev,
                                  uint32_t id, void *arg, size_t size)
{
    if (!arg) return ESP_ERR_INVALID_ARG;
    switch (id) {
        case ESP_CAM_SENSOR_DATA_SEQ:
            if (size != sizeof(int)) return ESP_ERR_INVALID_ARG;
            *(int *)arg = ESP_CAM_SENSOR_DATA_SEQ_NONE;
            return ESP_OK;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t imx500_ioctl(esp_cam_sensor_device_t *dev,
                               uint32_t cmd, void *arg)
{
    switch (cmd) {
        case ESP_CAM_SENSOR_IOC_S_STREAM: {
            int stream_on = *(int *)arg;
            ESP_LOGI(TAG, "Stream: %d", stream_on);

            esp_err_t ret = imx500_write_reg(dev, 0x0100, stream_on ? 0x01 : 0x00);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write stream register");
                return ret;
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            uint8_t rb = 0;
            if (imx500_read_reg(dev, 0x0100, &rb) == ESP_OK) {
                ESP_LOGI(TAG, "Stream reg readback: 0x%02X (expected 0x%02X)",
                         rb, stream_on ? 0x01 : 0x00);
                if (rb != (uint8_t)(stream_on ? 0x01 : 0x00)) {
                    ESP_LOGE(TAG, "Stream register mismatch — sensor may not be streaming");
                }
            }
            break;
        }
        default:
            break;
    }
    return ESP_OK;
}

static int imx500_del(esp_cam_sensor_device_t *dev)
{
    if (dev) {
        imx500_write_reg(dev, 0x0100, 0x00);
        free(dev);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Format descriptor                                                    */
/* ------------------------------------------------------------------ */

static const esp_cam_sensor_format_t imx500_format_2028x1520 = {
    .name   = "2028x1520_30fps",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
    .port   = ESP_CAM_SENSOR_MIPI_CSI,
    .width  = 2028,
    .height = 1520,
    .mipi_info = {
        /*
         * MIPI clock: verify against the RPi kernel driver.
         * 750 MHz is a conservative starting value; the driver will tell
         * you the exact frequency for this mode.
         */
        .mipi_clk  = 750000000,
        .lane_num  = 2,
        .hs_settle = 0,
    },
};

static const esp_cam_sensor_ops_t imx500_ops = {
    .query_support_formats = imx500_query_support_formats,
    .set_format            = imx500_set_format,
    .get_format            = imx500_get_format,
    .get_para_value        = imx500_get_para_value,
    .priv_ioctl            = imx500_ioctl,
    .del                   = imx500_del,
};

/* ------------------------------------------------------------------ */
/* Detection                                                            */
/* ------------------------------------------------------------------ */

static esp_cam_sensor_device_t *s_imx500_dev = NULL;

esp_cam_sensor_device_t *imx500_get_global_dev(void) { return s_imx500_dev; }

/*
 * Model ID for IMX500 is at registers 0x0016 (MSB) and 0x0017 (LSB),
 * following the same convention as the IMX477 and other recent Sony sensors.
 * Expected combined value: 0x0500.
 */
ESP_CAM_SENSOR_DETECT_FN(imx500, ESP_CAM_SENSOR_MIPI_CSI, IMX500_I2C_ADDR)
{
    esp_cam_sensor_config_t *cfg = (esp_cam_sensor_config_t *)config;
    esp_sccb_io_handle_t sccb   = cfg->sccb_handle;

    uint8_t id_h = 0, id_l = 0;

    if (esp_sccb_transmit_receive_reg_a16v8(sccb, 0x0016, &id_h) != ESP_OK) {
        ESP_LOGE(TAG, "I2C error reading model ID high byte");
        return NULL;
    }
    if (esp_sccb_transmit_receive_reg_a16v8(sccb, 0x0017, &id_l) != ESP_OK) {
        ESP_LOGE(TAG, "I2C error reading model ID low byte");
        return NULL;
    }

    uint16_t id = ((uint16_t)id_h << 8) | id_l;
    if (id != IMX500_PID) {
        ESP_LOGE(TAG, "ID mismatch: got 0x%04X, expected 0x%04X", id, IMX500_PID);
        return NULL;
    }

    ESP_LOGI(TAG, "IMX500 detected (ID: 0x%04X)", id);

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (!dev) return NULL;

    dev->name        = "IMX500";
    dev->id.pid      = id;
    dev->ops         = &imx500_ops;
    dev->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    dev->sccb_handle = sccb;
    dev->cur_format  = &imx500_format_2028x1520;

    s_imx500_dev = dev;
    ESP_LOGI(TAG, "Device allocated at %p", dev);
    return dev;
}
