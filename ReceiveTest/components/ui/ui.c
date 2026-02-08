/*
 * ui.c
 * Simple UI with updatable text display and status line
 */

#include "ui.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <stdio.h>
#include <string.h>

/* Widget pointers - stored globally so we can update them later */
static lv_obj_t *data_label = NULL;    /* Main data display (time, number, text) */
static lv_obj_t *status_label = NULL;  /* Status line at bottom (IP address) */

void ui_init(lv_display_t *disp)
{
    /* Lock LVGL - required before any UI changes */
    lvgl_port_lock(0);

    /* Get the active screen (root container) */
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    /* --------------------------------------------------------
     * Background - dark blue
     * -------------------------------------------------------- */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x003366), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* --------------------------------------------------------
     * Title label - "The data being sent is:"
     * White text, 32px, centered near top
     * -------------------------------------------------------- */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "The data being sent is:");
    lv_obj_set_style_text_color(title, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -100);

    /* --------------------------------------------------------
     * Data label - shows the actual data (time, number, text)
     * Green text, 48px, centered
     * -------------------------------------------------------- */
    data_label = lv_label_create(scr);
    lv_label_set_text(data_label, "Waiting...");
    lv_obj_set_style_text_color(data_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
    lv_obj_set_style_text_font(data_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_align(data_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(data_label, LV_ALIGN_CENTER, 0, 0);

    /* --------------------------------------------------------
     * Status label - shows IP address and connection info
     * Gray text, 24px, bottom of screen
     * -------------------------------------------------------- */
    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, "Initializing network...");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -30);

    /* Unlock LVGL - let background task render */
    lvgl_port_unlock();
}

void ui_set_text(const char *text)
{
    /* Safety check */
    if (data_label == NULL || text == NULL) {
        return;
    }

    /* Lock, update, unlock */
    lvgl_port_lock(0);
    lv_label_set_text(data_label, text);
    lvgl_port_unlock();
}

void ui_set_status(const char *status)
{
    /* Safety check */
    if (status_label == NULL || status == NULL) {
        return;
    }

    /* Lock, update, unlock */
    lvgl_port_lock(0);
    lv_label_set_text(status_label, status);
    lvgl_port_unlock();
}
