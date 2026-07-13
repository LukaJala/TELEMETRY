#pragma once
#include <stdint.h>
#include <stdbool.h>
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

/* Switch the right-panel tab: 0=BATTERY, 1=SOLAR/MOTOR, 2=GPS/TRIP */
void ui_set_tab(uint8_t tab_index);

/* Update the Pi telemetry-link indicator (bottom-left). Safe to call from any
 * task and at any rate: it de-dupes internally and skips the update (to retry
 * later) if the LVGL lock is held, e.g. while the camera is fullscreen. */
void ui_set_pi_status(bool connected);

/* Camera picture-in-picture window (bottom-right, above every overlay — the
 * backup camera must always be visible). Returns the RGB565 buffer the camera
 * PPA writes into (128-byte aligned, w*h*2 bytes), or NULL if the boot-time
 * allocation failed. Wire it up via camera_config_pip(). */
void *ui_cam_pip_buffer(uint32_t *w, uint32_t *h);

/* Repaint the PiP widget after a new frame landed in the buffer. Called from
 * the camera stream task; skips (drops the repaint, keeps the frame) if the
 * LVGL lock is contended. */
void ui_cam_pip_frame_ready(void);

/* Show/hide the PiP window (hide it if the camera failed at boot). */
void ui_cam_pip_set_active(bool active);
