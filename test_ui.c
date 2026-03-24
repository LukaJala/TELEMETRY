#include "lvgl.h"

void test_ui_init(lv_display_t * disp)
{
    lv_obj_t * label = lv_label_create(lv_display_get_screen_active(disp));
    lv_label_set_text(label, "LVGL Sim WORKS!");
    lv_obj_center(label);
}
