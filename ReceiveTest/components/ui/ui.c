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
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_ROW);

    // Remove default padding
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_style_pad_column(scr, 0, 0);
    lv_obj_set_style_pad_row(scr, 0, 0);

    // Create containers
    lv_obj_t *left_cont = lv_obj_create(scr);
    lv_obj_t *mid_cont = lv_obj_create(scr);
    lv_obj_t *right_cont = lv_obj_create(scr);

    // Update size and fill percent
    lv_obj_set_size(left_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_size(mid_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_size(right_cont, LV_PCT(100), LV_PCT(100));

    lv_obj_set_flex_grow(left_cont, 1);
    lv_obj_set_flex_grow(mid_cont, 2);
    lv_obj_set_flex_grow(right_cont, 1);


    // Update colors and border 
    lv_obj_set_style_bg_color(left_cont, lv_color_hex(0x001F3f), 0);
    lv_obj_set_style_border_width(left_cont, 0, 0);

    lv_obj_set_style_bg_color(mid_cont, lv_color_hex(0x551122), 0);
    lv_obj_set_style_border_width(mid_cont, 0, 0);

    lv_obj_set_style_bg_color(right_cont, lv_color_hex(0x003366), 0);
    lv_obj_set_style_border_width(right_cont, 0, 0);

    

    // Center the content of the containers
    lv_obj_set_layout(left_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(left_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(left_cont,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

    lv_obj_set_layout(mid_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(mid_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(mid_cont,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

    lv_obj_set_layout(right_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(right_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right_cont,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

    /* Add title to top half */
    lv_obj_t *title = lv_label_create(left_cont);
    lv_label_set_text(title, "The data being sent is:");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);

    // Center spedometer arc
    lv_obj_t *arc = lv_arc_create(mid_cont);

    lv_obj_set_size(arc, 220, 220);
    lv_obj_center(arc);

    // Range of arc
    lv_arc_set_range(arc, 0, 100);
    lv_arc_set_value(arc, 65);

    // Start and end angles
    lv_arc_set_bg_angles(arc, 135, 405);   // 270° sweep
    lv_arc_set_rotation(arc, 0);

    // Remove knob
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);

    

    /* Center content */
    
                        

     /* Data label (big green text) */
    data_label = lv_label_create(right_cont);
    lv_label_set_text(data_label, "Waiting...");
    lv_obj_set_style_text_color(data_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(data_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(data_label, LV_TEXT_ALIGN_CENTER, 0);

    /* Status label under data */
    status_label = lv_label_create(right_cont);
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
