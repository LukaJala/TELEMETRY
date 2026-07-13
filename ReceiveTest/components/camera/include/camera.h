#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

/*
 * Initialize the camera sensor (OV5647/IMX219) and MIPI CSI pipeline.
 * Must be called once at startup before camera_start().
 */
esp_err_t camera_init(void);

/*
 * Configure the picture-in-picture output target. Call after camera_init()
 * and before camera_start().
 *
 * buf: cache-line-aligned (128 B) RGB565 buffer of w*h*2 bytes. The PPA
 *      scales a centered crop of each sensor frame into it, unrotated —
 *      the buffer is LVGL widget content, so LVGL's sw-rotate orients it.
 * w:   must keep the PPA scale on its 1/16 grid — a multiple of 80 px
 *      (full 1280-px sensor width is always shown).
 * h:   selects a centered vertical crop of the sensor at the same scale;
 *      h*16 must be divisible by the scale numerator (see camera.c log).
 * frame_cb: invoked from the camera task after each PiP frame lands in buf;
 *      use it to invalidate the LVGL widget that displays the buffer.
 */
esp_err_t camera_config_pip(void *buf, uint32_t w, uint32_t h,
                            void (*frame_cb)(void));

/*
 * Start streaming. With a PiP target configured the stream starts in PiP
 * mode and the dashboard keeps running; without one the stream is fullscreen
 * for its whole life (legacy behavior).
 * panel: the DSI panel handle returned by display_init()
 */
esp_err_t camera_start(esp_lcd_panel_handle_t panel);

/*
 * Stop streaming entirely and hand the display back to LVGL.
 */
esp_err_t camera_stop(void);

/*
 * Returns true if the camera is currently streaming (PiP or fullscreen).
 */
bool camera_is_running(void);

/*
 * Fullscreen requests. Effective mode = manual OR reverse; the stream task
 * applies changes at the next frame boundary (within ~1 frame time).
 * While fullscreen is active the stream task holds the LVGL lock, freezing
 * the dashboard; on exit it triggers a full repaint.
 */
void camera_set_reverse(bool in_reverse);
void camera_set_fullscreen_manual(bool on);

/*
 * True once fullscreen is actually active (the stream task owns the DPI
 * frame buffer and holds the LVGL lock). Use to gate dashboard updates.
 */
bool camera_is_fullscreen(void);
