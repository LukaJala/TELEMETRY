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

    /**
        Make screen use flex column layout
    */
    lv_obj_set_layout(scr, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);

    // Remove default padding
    lv_obj_set_style_pad_all(scr, 0, 0);

    /**
        Top Container
    */
    lv_obj_t *top_cont = lv_obj_create(scr);
    lv_obj_set_size(top_cont, LV_PCT(100), LV_PCT(50));

    lv_obj_set_style_bg_color(top_cont, lv_color_hex(0x001F3f), 0);
    lv_obj_set_style_border_width(top_cont, 0, 0);

    /* Center content inside top container */
    lv_obj_set_layout(top_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(top_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(top_cont,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

    /* Add title to top half */
    lv_obj_t *title = lv_label_create(top_cont);
    lv_label_set_text(title, "The data being sent is:");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);

    /* --------------------------------------------------------
    * BOTTOM CONTAINER (50% of screen)
    * -------------------------------------------------------- */
    lv_obj_t *bottom_cont = lv_obj_create(scr);
    lv_obj_set_size(bottom_cont, LV_PCT(100), LV_PCT(50));

    lv_obj_set_style_bg_color(bottom_cont, lv_color_hex(0x003366), 0);
    lv_obj_set_style_border_width(bottom_cont, 0, 0);

    /* Center content */
    lv_obj_set_layout(bottom_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(bottom_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(bottom_cont,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
                        

     /* Data label (big green text) */
    data_label = lv_label_create(bottom_cont);
    lv_label_set_text(data_label, "Waiting...");
    lv_obj_set_style_text_color(data_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(data_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(data_label, LV_TEXT_ALIGN_CENTER, 0);

    /* Status label under data */
    status_label = lv_label_create(bottom_cont);
    lv_label_set_text(status_label, "Initializing network...");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_24, 0);

    /* --------------------------------------------------------
     * Unlock LVGL so it can render
     * -------------------------------------------------------- */
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
