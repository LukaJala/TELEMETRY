#pragma once
#include "lvgl.h"

/* Initialize the UI with a display */
void ui_init(lv_display_t *disp);

/* Force a full screen redraw (use after camera stops to clear stale frame) */
void ui_refresh(void);

/* Update the main display text (can be number, time, or any string) */
void ui_set_text(const char *text);

/* Set the status line (shows IP address, connection status, etc.) */
void ui_set_status(const char *status);
