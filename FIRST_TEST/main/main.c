#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "display_init.h"
#include "display_config.h"
#include "ui_init.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Booting system");

    esp_lcd_panel_handle_t panel = display_init();

    lvgl_port_init(&(lvgl_port_cfg_t)ESP_LVGL_PORT_INIT_CONFIG());

    lv_display_t *disp = lvgl_port_add_disp_dsi(
        &(lvgl_port_display_cfg_t){
            .panel_handle = panel,
            .buffer_size = LCD_H_RES * 50,
            .double_buffer = true,
            .hres = LCD_H_RES,
            .vres = LCD_V_RES,
            .color_format = LV_COLOR_FORMAT_RGB888,
            .flags.buff_spiram = true,
        },
        &(lvgl_port_display_dsi_cfg_t){});

    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

    ui_init(disp);

    ESP_LOGI(TAG, "System ready");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
