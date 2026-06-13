#pragma once
#include "lvgl.h"
#include "display_can_spec.h"

/* Initialize the dashboard UI */
void ui_init(lv_display_t *disp);

/* Force a full screen redraw (use after camera stops to clear stale frame) */
void ui_refresh(void);

/* Update all dashboard widgets from the latest decoded CAN state.
 * Only widgets whose update_flags bits are set get redrawn. */
void ui_update_can_data(const display_data_t *data);

/* Write a plain text message to the status bar (connection state, errors) */
void ui_set_text(const char *text);

/* Set the status line (shows IP address, connection status, etc.) */
void ui_set_status(const char *status);
