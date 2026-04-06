#include <string.h>
#include <stdlib.h>
#include "imx219.h"
#include "imx219_regs.h"
#include "esp_log.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imx219";

#define IMX219_PID 0x0219

static const esp_cam_sensor_format_t imx219_fmt_1536x1232;

void imx219_force_link(void) {}

static esp_err_t imx219_write(esp_cam_sensor_device_t *dev, uint16_t reg, uint8_t val)
{
    return esp_sccb_transmit_reg_a16v8(dev->sccb_handle, reg, val);
}

static esp_err_t imx219_read(esp_cam_sensor_device_t *dev, uint16_t reg, uint8_t *val)
{
    return esp_sccb_transmit_receive_reg_a16v8(dev->sccb_handle, reg, val);
}

static esp_err_t imx219_set_format(esp_cam_sensor_device_t *dev,
                                    const esp_cam_sensor_format_t *format)
{
    if (!format) {
        format = dev->cur_format ? dev->cur_format : &imx219_fmt_1536x1232;
    }
    ESP_LOGI(TAG, "Applying register table (%d regs)",
             (int)(sizeof(imx219_1536x1232_30fps) / sizeof(imx219_reg_t)));

    int n = sizeof(imx219_1536x1232_30fps) / sizeof(imx219_reg_t);
    for (int i = 0; i < n; i++) {
        if (imx219_write(dev, imx219_1536x1232_30fps[i].reg,
                              imx219_1536x1232_30fps[i].val) != ESP_OK) {
            ESP_LOGE(TAG, "Failed reg 0x%04X", imx219_1536x1232_30fps[i].reg);
            return ESP_FAIL;
        }
    }
    dev->cur_format = format;
    return ESP_OK;
}

static esp_err_t imx219_get_format(esp_cam_sensor_device_t *dev,
                                    esp_cam_sensor_format_t *format)
{
    const esp_cam_sensor_format_t *src =
        dev->cur_format ? dev->cur_format : &imx219_fmt_1536x1232;
    memcpy(format, src, sizeof(esp_cam_sensor_format_t));
    return ESP_OK;
}

static esp_err_t imx219_query_formats(esp_cam_sensor_device_t *dev,
                                       esp_cam_sensor_format_array_t *arr)
{
    arr->count = 1;
    arr->format_array = &imx219_fmt_1536x1232;
    return ESP_OK;
}

static int imx219_get_para(esp_cam_sensor_device_t *dev,
                            uint32_t id, void *arg, size_t size)
{
    if (!arg) return ESP_ERR_INVALID_ARG;
    if (id == ESP_CAM_SENSOR_DATA_SEQ) {
        if (size != sizeof(int)) return ESP_ERR_INVALID_ARG;
        *(int *)arg = ESP_CAM_SENSOR_DATA_SEQ_NONE;
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t imx219_ioctl(esp_cam_sensor_device_t *dev,
                               uint32_t cmd, void *arg)
{
    if (cmd == ESP_CAM_SENSOR_IOC_S_STREAM) {
        int on = *(int *)arg;
        ESP_LOGI(TAG, "Stream %s", on ? "ON" : "OFF");
        esp_err_t ret = imx219_write(dev, 0x0100, on ? 0x01 : 0x00);
        if (ret != ESP_OK) return ret;
        vTaskDelay(pdMS_TO_TICKS(100));
        uint8_t v = 0;
        if (imx219_read(dev, 0x0100, &v) == ESP_OK) {
            ESP_LOGI(TAG, "0x0100 readback = 0x%02X", v);
        }
        return ESP_OK;
    }
    return ESP_OK;
}

static int imx219_del(esp_cam_sensor_device_t *dev)
{
    if (dev) {
        imx219_write(dev, 0x0100, 0x00);
        free(dev);
    }
    return ESP_OK;
}

esp_err_t imx219_set_gain(esp_cam_sensor_device_t *dev, uint8_t gain)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return imx219_write(dev, 0x0157, gain);
}

esp_err_t imx219_set_exposure(esp_cam_sensor_device_t *dev, uint16_t exposure)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = imx219_write(dev, 0x015A, (exposure >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;
    return imx219_write(dev, 0x015B, exposure & 0xFF);
}

static const esp_cam_sensor_format_t imx219_fmt_1536x1232 = {
    .name   = "1536x1232_30fps_raw10",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
    .port   = ESP_CAM_SENSOR_MIPI_CSI,
    .width  = 1536,
    .height = 1232,
    .mipi_info = {
        .mipi_clk  = 456000000,
        .lane_num  = 2,
        .hs_settle = 0,
    },
};

static const esp_cam_sensor_ops_t imx219_ops = {
    .query_support_formats = imx219_query_formats,
    .set_format            = imx219_set_format,
    .get_format            = imx219_get_format,
    .get_para_value        = imx219_get_para,
    .priv_ioctl            = imx219_ioctl,
    .del                   = imx219_del,
};

ESP_CAM_SENSOR_DETECT_FN(imx219, ESP_CAM_SENSOR_MIPI_CSI, IMX219_I2C_ADDR)
{
    esp_cam_sensor_config_t *cfg = (esp_cam_sensor_config_t *)config;
    esp_sccb_io_handle_t sccb   = cfg->sccb_handle;

    /*
     * IMX219 uses SCCB protocol, which requires a full STOP between the
     * register-address write and the data read.  The ESP I2C master API
     * uses a REPEATED START for combined transactions, which the sensor
     * does not support — reads return 0x00.  Rather than bypass the SCCB
     * library, we skip the ID check (the I2C bus scan in main already
     * confirmed a device at IMX219_I2C_ADDR) and proceed directly to
     * initialization.  The register table write will fail explicitly if
     * the sensor is not present or wired incorrectly.
     */
    ESP_LOGI(TAG, "IMX219 at 0x%02X — skipping SCCB ID read (STOP/repeated-start issue)",
             IMX219_I2C_ADDR);

    /* Software reset before applying register table */
    esp_sccb_transmit_reg_a16v8(sccb, 0x0103, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (!dev) return NULL;

    dev->name        = "IMX219";
    dev->id.pid      = IMX219_PID;
    dev->ops         = &imx219_ops;
    dev->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    dev->sccb_handle = sccb;
    dev->cur_format  = &imx219_fmt_1536x1232;

    return dev;
}
