/*
 * ESP32-P4 Display Test with JD9365 10.1" LCD
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_jd9365_10_1.h"

static const char *TAG = "DISPLAY_TEST";

// Display configuration
#define LCD_H_RES               800
#define LCD_V_RES               1280
#define LCD_BIT_PER_PIXEL       24
#define MIPI_DSI_LANE_NUM       2

// Pin configuration
#define PIN_NUM_LCD_RST         27
#define PIN_NUM_BK_LIGHT        26
#define LCD_BK_LIGHT_ON_LEVEL   1
#define LCD_BK_LIGHT_OFF_LEVEL  (!LCD_BK_LIGHT_ON_LEVEL)

// MIPI DSI PHY power
#define MIPI_DSI_PHY_PWR_LDO_CHAN       3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500

#if LCD_BIT_PER_PIXEL == 24
#define MIPI_DPI_PX_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB888
#elif LCD_BIT_PER_PIXEL == 16
#define MIPI_DPI_PX_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB565
#endif

static esp_lcd_panel_handle_t panel_handle = NULL;
static SemaphoreHandle_t refresh_finish = NULL;

// Callback when color transfer is done
static bool IRAM_ATTR notify_refresh_ready(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    BaseType_t need_yield = pdFALSE;
    xSemaphoreGiveFromISR(sem, &need_yield);
    return (need_yield == pdTRUE);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 Display Test Starting...");

    // Configure and turn on backlight
    ESP_LOGI(TAG, "Configuring backlight GPIO");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));

    // Power on MIPI DSI PHY via internal LDO
    ESP_LOGI(TAG, "Powering on MIPI DSI PHY");
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_config, &ldo_mipi_phy));

    // Create MIPI DSI bus
    ESP_LOGI(TAG, "Initializing MIPI DSI bus");
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_config = JD9365_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    // Create panel IO
    ESP_LOGI(TAG, "Installing panel IO");
    esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
    esp_lcd_dbi_io_config_t dbi_config = JD9365_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));

    // Configure and create JD9365 panel
    ESP_LOGI(TAG, "Installing JD9365 LCD driver");
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_800_1280_PANEL_60HZ_DPI_CONFIG(MIPI_DPI_PX_FORMAT);
    jd9365_vendor_config_t vendor_config = {
        .flags = {
            .use_mipi_interface = 1,
        },
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = MIPI_DSI_LANE_NUM,
        },
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9365(mipi_dbi_io, &panel_config, &panel_handle));

    // Reset and initialize the panel
    ESP_LOGI(TAG, "Resetting and initializing panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Set up refresh callback
    refresh_finish = xSemaphoreCreateBinary();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = notify_refresh_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, refresh_finish));

    // Display a test pattern
    ESP_LOGI(TAG, "Displaying vertical color bar test pattern");
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_set_pattern(panel_handle, MIPI_DSI_PATTERN_BAR_VERTICAL));

    ESP_LOGI(TAG, "Display initialized successfully!");

    // Main loop - cycle through test patterns
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG, "Switching to horizontal bars");
        ESP_ERROR_CHECK(esp_lcd_dpi_panel_set_pattern(panel_handle, MIPI_DSI_PATTERN_BAR_HORIZONTAL));

        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG, "Switching to vertical bars");
        ESP_ERROR_CHECK(esp_lcd_dpi_panel_set_pattern(panel_handle, MIPI_DSI_PATTERN_BAR_VERTICAL));
    }
}
