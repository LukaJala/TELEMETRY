/*
 * display_init.c
 * ESP32-P4 MIPI-DSI LCD initialization (JD9365 10.1")
 */

#include "display_init.h"
#include "display_config.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_jd9365_10_1.h"
#include "esp_ldo_regulator.h"

static const char *TAG = "DISPLAY";

#if LCD_BIT_PER_PIXEL == 24
#define MIPI_DPI_PX_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB888
#else
#define MIPI_DPI_PX_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB565
#endif

esp_lcd_panel_handle_t display_init(void)
{
    ESP_LOGI(TAG, "Initializing display hardware");

    /* ----------------------------------------------------------
     * Backlight GPIO
     * ---------------------------------------------------------- */
    gpio_config_t bk_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));

    /* ----------------------------------------------------------
     * MIPI DSI PHY power (internal LDO)
     * ---------------------------------------------------------- */
    esp_ldo_channel_handle_t ldo_mipi = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi));

    /* ----------------------------------------------------------
     * Create MIPI DSI bus
     * ---------------------------------------------------------- */
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = JD9365_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));

    /* ----------------------------------------------------------
     * Create DBI IO for panel commands
     * ---------------------------------------------------------- */
    esp_lcd_panel_io_handle_t panel_io = NULL;
    esp_lcd_dbi_io_config_t io_cfg = JD9365_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &io_cfg, &panel_io));

    /* ----------------------------------------------------------
     * Panel configuration
     * ---------------------------------------------------------- */
    esp_lcd_dpi_panel_config_t dpi_cfg =
        JD9365_800_1280_PANEL_60HZ_DPI_CONFIG(MIPI_DPI_PX_FORMAT);

    jd9365_vendor_config_t vendor_cfg = {
        .flags = {
            .use_mipi_interface = 1,
        },
        .mipi_config = {
            .dsi_bus = dsi_bus,
            .dpi_config = &dpi_cfg,
            .lane_num = MIPI_DSI_LANE_NUM,
        },
    };

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_cfg,
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_jd9365(panel_io, &panel_cfg, &panel)
    );

    /* ----------------------------------------------------------
     * Initialize panel
     * ---------------------------------------------------------- */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    ESP_LOGI(TAG, "Display hardware ready");
    return panel;
}
