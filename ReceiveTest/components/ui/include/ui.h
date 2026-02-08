#pragma once
#include "lvgl.h"

/* Initialize the UI with a display */
void ui_init(lv_display_t *disp);

/* Update the main display text (can be number, time, or any string) */
void ui_set_text(const char *text);

/* Set the status line (shows IP address, connection status, etc.) */
void ui_set_status(const char *status);
