/*
 * ui_init.c
 * LVGL UI initialization
 */

#include "ui_init.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

void ui_init(lv_display_t *disp)
{
    /* ----------------------------------------------------------
     * All LVGL object creation must be locked
     * ---------------------------------------------------------- */
    lvgl_port_lock(0);

    /* Active screen */
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    /* Background */
    lv_obj_set_style_bg_color(
        scr,
        lv_color_hex(0x003366),   // MSU-style deep blue
        LV_PART_MAIN
    );
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* Main label */
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "MSU Solar Racing Team\nGOATED");
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(label);

    lvgl_port_unlock();
}
