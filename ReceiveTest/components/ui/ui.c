/*
 * ui.c
 * Solar car telemetry dashboard — 1280×800 landscape
 *
 * Layout:
 *
 *   ┌──────────┬──────────────────────────┬──────────┐
 *   │  MODE    │                          │  SOC     │  ← top section (440px)
 *   │  FWD     │     SPEED  (large)       │  bar+%   │
 *   │          │     km/h                 │  V / A   │
 *   │  REGEN   │                          │          │
 *   ├──────────┴──────────────────────────┴──────────┤  ← separator (2px)
 *   │ MOTOR °C │ CTRL °C  │  CELL Δ mV  │  SOLAR W  │  ← bottom tiles (288px)
 *   ├──────────┴──────────┴─────────────┴───────────┤
 *   │  status bar (IP / messages)                    │  ← 70px
 *   └────────────────────────────────────────────────┘
 *
 *   Fault overlay: full-screen red, hidden until BPS reports a fault.
 */

#include "ui.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "display_can_spec.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Colors
 * ========================================================================= */
#define C_BG        lv_color_hex(0x0D0D1A)
#define C_PANEL     lv_color_hex(0x14142B)
#define C_TILE      lv_color_hex(0x1A1A33)
#define C_STATUS_BG lv_color_hex(0x0A0A14)
#define C_DIVIDER   lv_color_hex(0x2A2A4A)
#define C_WHITE     lv_color_white()
#define C_GRAY      lv_color_hex(0x888888)
#define C_GREEN     lv_color_hex(0x00DD55)
#define C_YELLOW    lv_color_hex(0xFFCC00)
#define C_RED       lv_color_hex(0xDD2222)
#define C_BLUE      lv_color_hex(0x00AAFF)

/* =========================================================================
 * Layout (px, landscape 1280×800 after rotation)
 * ========================================================================= */
#define SCR_W      1280
#define SCR_H       800
#define TOP_H       440
#define LEFT_W      260
#define RIGHT_W     260
#define CENTER_W    (SCR_W - LEFT_W - RIGHT_W)   /* 760 */
#define SEP_Y       TOP_H
#define SEP_H       2
#define BOT_Y       (SEP_Y + SEP_H)              /* 442 */
#define STATUS_H    70
#define BOT_H       (SCR_H - BOT_Y - STATUS_H)   /* 288 */
#define STATUS_Y    (SCR_H - STATUS_H)            /* 730 */
#define TILE_W      (SCR_W / 4)                  /* 320 */

/* =========================================================================
 * Widget handles
 * ========================================================================= */
static lv_obj_t *lbl_speed       = NULL;
static lv_obj_t *lbl_drive_mode  = NULL;
static lv_obj_t *lbl_regen       = NULL;
static lv_obj_t *bar_soc         = NULL;
static lv_obj_t *lbl_soc         = NULL;
static lv_obj_t *lbl_pack_v      = NULL;
static lv_obj_t *lbl_pack_a      = NULL;
static lv_obj_t *lbl_motor_temp  = NULL;
static lv_obj_t *lbl_ctrl_temp   = NULL;
static lv_obj_t *lbl_cell_delta  = NULL;
static lv_obj_t *lbl_solar       = NULL;
static lv_obj_t *status_label    = NULL;
static lv_obj_t *fault_overlay   = NULL;
static lv_obj_t *lbl_fault_name  = NULL;
static lv_obj_t *lbl_fault_timer = NULL;

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

static lv_obj_t *make_panel(lv_obj_t *parent,
                             int x, int y, int w, int h,
                             lv_color_t bg)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_pos(p, x, y);
    lv_obj_set_size(p, w, h);
    lv_obj_set_style_bg_color(p, bg, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(p, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(p, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(p, 0, LV_PART_MAIN);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    return p;
}

static lv_obj_t *make_label(lv_obj_t *parent,
                             const char *text,
                             lv_color_t color,
                             const lv_font_t *font,
                             lv_align_t align,
                             int x_ofs, int y_ofs)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_color(l, color, LV_PART_MAIN);
    lv_obj_set_style_text_font(l, font, LV_PART_MAIN);
    lv_obj_align(l, align, x_ofs, y_ofs);
    return l;
}

/*
 * make_tile — creates one bottom tile and returns the value label.
 * Each tile has a caption at the top and a large centered value.
 */
static lv_obj_t *make_tile(lv_obj_t *scr,
                            int tile_idx,
                            const char *caption,
                            const char *initial_val)
{
    int x = tile_idx * TILE_W;
    lv_obj_t *tile = make_panel(scr, x, BOT_Y, TILE_W, BOT_H, C_TILE);

    /* Left-edge divider on all tiles except the first */
    if (tile_idx > 0) {
        lv_obj_set_style_border_side(tile, LV_BORDER_SIDE_LEFT, LV_PART_MAIN);
        lv_obj_set_style_border_width(tile, 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(tile, C_DIVIDER, LV_PART_MAIN);
    }

    make_label(tile, caption, C_GRAY, &lv_font_montserrat_24,
               LV_ALIGN_TOP_MID, 0, 18);

    return make_label(tile, initial_val, C_WHITE, &lv_font_montserrat_48,
                      LV_ALIGN_CENTER, 0, 12);
}

/* =========================================================================
 * ui_init
 * ========================================================================= */
void ui_init(lv_display_t *disp)
{
    lvgl_port_lock(0);

    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_set_style_bg_color(scr, C_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* ------------------------------------------------------------------
     * LEFT PANEL — drive mode + regen indicator
     * ------------------------------------------------------------------ */
    lv_obj_t *left = make_panel(scr, 0, 0, LEFT_W, TOP_H, C_PANEL);

    make_label(left, "MODE", C_GRAY, &lv_font_montserrat_24,
               LV_ALIGN_TOP_MID, 0, 50);
    lbl_drive_mode = make_label(left, "---", C_WHITE, &lv_font_montserrat_48,
                                LV_ALIGN_TOP_MID, 0, 90);

    make_label(left, "REGEN", C_GRAY, &lv_font_montserrat_24,
               LV_ALIGN_TOP_MID, 0, 240);
    lbl_regen = make_label(left, "OFF", C_GRAY, &lv_font_montserrat_32,
                           LV_ALIGN_TOP_MID, 0, 282);

    /* ------------------------------------------------------------------
     * CENTER PANEL — speed
     * ------------------------------------------------------------------ */
    lv_obj_t *center = make_panel(scr, LEFT_W, 0, CENTER_W, TOP_H, C_BG);

    make_label(center, "SPEED", C_GRAY, &lv_font_montserrat_24,
               LV_ALIGN_TOP_MID, 0, 30);
    lbl_speed = make_label(center, "--", C_WHITE, &lv_font_montserrat_48,
                           LV_ALIGN_CENTER, 0, -20);
    make_label(center, "km/h", C_GRAY, &lv_font_montserrat_32,
               LV_ALIGN_CENTER, 0, 55);

    /* ------------------------------------------------------------------
     * RIGHT PANEL — SOC bar + pack electrical
     * ------------------------------------------------------------------ */
    lv_obj_t *right = make_panel(scr, LEFT_W + CENTER_W, 0, RIGHT_W, TOP_H, C_PANEL);

    make_label(right, "SOC", C_GRAY, &lv_font_montserrat_24,
               LV_ALIGN_TOP_MID, 0, 20);

    /* Vertical SOC bar */
    bar_soc = lv_bar_create(right);
    lv_obj_set_size(bar_soc, 22, 180);
    lv_bar_set_range(bar_soc, 0, 100);
    lv_bar_set_value(bar_soc, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_soc, C_DIVIDER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar_soc, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_soc, C_GREEN, LV_PART_INDICATOR);
    lv_obj_align(bar_soc, LV_ALIGN_TOP_MID, -38, 55);

    lbl_soc = make_label(right, "--%", C_GREEN, &lv_font_montserrat_32,
                         LV_ALIGN_TOP_MID, 20, 112);

    lbl_pack_v = make_label(right, "--.- V", C_WHITE, &lv_font_montserrat_24,
                            LV_ALIGN_TOP_MID, 0, 275);
    lbl_pack_a = make_label(right, "--.- A", C_GRAY, &lv_font_montserrat_24,
                            LV_ALIGN_TOP_MID, 0, 315);

    /* ------------------------------------------------------------------
     * Horizontal separator
     * ------------------------------------------------------------------ */
    lv_obj_t *sep = make_panel(scr, 0, SEP_Y, SCR_W, SEP_H, C_DIVIDER);
    (void)sep;

    /* ------------------------------------------------------------------
     * Bottom tiles
     * ------------------------------------------------------------------ */
    lbl_motor_temp = make_tile(scr, 0, "MOTOR TEMP",  "--\xc2\xb0""C");
    lbl_ctrl_temp  = make_tile(scr, 1, "CTRL TEMP",   "--\xc2\xb0""C");
    lbl_cell_delta = make_tile(scr, 2, "CELL SPREAD", "-- mV");
    lbl_solar      = make_tile(scr, 3, "SOLAR IN",    "-- W");

    /* ------------------------------------------------------------------
     * Status bar
     * ------------------------------------------------------------------ */
    lv_obj_t *status_panel = make_panel(scr, 0, STATUS_Y, SCR_W, STATUS_H, C_STATUS_BG);
    status_label = make_label(status_panel, "Initializing...", C_GRAY,
                              &lv_font_montserrat_24, LV_ALIGN_LEFT_MID, 24, 0);

    /* ------------------------------------------------------------------
     * Fault overlay — hidden until BPS reports a fault
     * ------------------------------------------------------------------ */
    fault_overlay = lv_obj_create(scr);
    lv_obj_set_pos(fault_overlay, 0, 0);
    lv_obj_set_size(fault_overlay, SCR_W, SCR_H - STATUS_H);
    lv_obj_set_style_bg_color(fault_overlay, C_RED, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(fault_overlay, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(fault_overlay, 0, LV_PART_MAIN);
    lv_obj_clear_flag(fault_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(fault_overlay, LV_OBJ_FLAG_HIDDEN);

    lbl_fault_name = make_label(fault_overlay, "FAULT", C_WHITE, &lv_font_montserrat_48,
                                LV_ALIGN_CENTER, 0, -50);
    lbl_fault_timer = make_label(fault_overlay, "", C_WHITE, &lv_font_montserrat_32,
                                 LV_ALIGN_CENTER, 0, 40);

    lvgl_port_unlock();
}

/* =========================================================================
 * ui_refresh
 * ========================================================================= */
void ui_refresh(void)
{
    lvgl_port_lock(portMAX_DELAY);
    lv_obj_invalidate(lv_screen_active());
    lvgl_port_unlock();
}

/* =========================================================================
 * ui_set_text — writes a plain message to the status bar
 * ========================================================================= */
void ui_set_text(const char *text)
{
    if (status_label == NULL || text == NULL) return;
    if (!lvgl_port_lock(0)) return;
    lv_label_set_text(status_label, text);
    lvgl_port_unlock();
}

/* =========================================================================
 * ui_set_status
 * ========================================================================= */
void ui_set_status(const char *status)
{
    if (status_label == NULL || status == NULL) return;
    if (!lvgl_port_lock(0)) return;
    lv_label_set_text(status_label, status);
    lvgl_port_unlock();
}

/* =========================================================================
 * ui_update_can_data
 * ========================================================================= */
void ui_update_can_data(const display_data_t *d)
{
    if (d == NULL) return;
    if (!lvgl_port_lock(0)) return;

    uint32_t f = d->update_flags;

    /* ---- Speed -------------------------------------------------------- */
    if (f & DFLAG_VEGA_STATUS) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", (int)d->vega_status.speed_kmh);
        lv_label_set_text(lbl_speed, buf);
    }

    /* ---- Drive mode + regen ------------------------------------------ */
    if (f & DFLAG_VEGA_INPUTS) {
        static const char *const modes[] = {"PARK", "FWD", "REV", "???"};
        uint8_t m = (d->vega_inputs.drive_mode < 3) ? d->vega_inputs.drive_mode : 3;
        lv_label_set_text(lbl_drive_mode, modes[m]);

        if (d->vega_inputs.regen_active) {
            lv_label_set_text(lbl_regen, "ON");
            lv_obj_set_style_text_color(lbl_regen, C_BLUE, LV_PART_MAIN);
        } else {
            lv_label_set_text(lbl_regen, "OFF");
            lv_obj_set_style_text_color(lbl_regen, C_GRAY, LV_PART_MAIN);
        }
    }

    /* ---- SOC + pack electrical --------------------------------------- */
    if (f & DFLAG_BPS_POWER) {
        char buf[32];
        float soc = d->bps_power.soc_pct;
        lv_bar_set_value(bar_soc, (int32_t)soc, LV_ANIM_OFF);

        snprintf(buf, sizeof(buf), "%d%%", (int)soc);
        lv_label_set_text(lbl_soc, buf);

        lv_color_t soc_color = (soc > 30.0f) ? C_GREEN
                             : (soc > 15.0f) ? C_YELLOW
                                             : C_RED;
        lv_obj_set_style_bg_color(bar_soc, soc_color, LV_PART_INDICATOR);
        lv_obj_set_style_text_color(lbl_soc, soc_color, LV_PART_MAIN);

        /* Pack voltage: one decimal place via integer math */
        int v10 = (int)(d->bps_power.pack_voltage_v * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d V", v10 / 10, v10 % 10);
        lv_label_set_text(lbl_pack_v, buf);

        /* Pack current: one decimal place with explicit sign */
        float amps = d->bps_power.pack_current_a;
        int a10 = (int)(amps * 10.0f);
        if (a10 >= 0) snprintf(buf, sizeof(buf), "+%d.%d A",  a10 / 10,  a10 % 10);
        else          snprintf(buf, sizeof(buf), "-%d.%d A", -a10 / 10, -a10 % 10);
        lv_label_set_text(lbl_pack_a, buf);

        /* Blue for charging/regen (negative current), white for discharge */
        lv_obj_set_style_text_color(lbl_pack_a,
            (amps < 0.0f) ? C_BLUE : C_WHITE, LV_PART_MAIN);
    }

    /* ---- Motor + controller temps ------------------------------------ */
    if (f & DFLAG_MC_STATUS2) {
        int mt = d->mc_status2.motor_temp_c;
        /* \xc2\xb0 is the UTF-8 degree symbol ° */
        lv_label_set_text_fmt(lbl_motor_temp, "%d\xc2\xb0""C", mt);
        lv_obj_set_style_text_color(lbl_motor_temp,
            (mt > 80) ? C_RED : (mt > 60) ? C_YELLOW : C_WHITE, LV_PART_MAIN);

        int ct = d->mc_status2.ctrl_temp_c;
        lv_label_set_text_fmt(lbl_ctrl_temp, "%d\xc2\xb0""C", ct);
        lv_obj_set_style_text_color(lbl_ctrl_temp,
            (ct > 75) ? C_RED : (ct > 55) ? C_YELLOW : C_WHITE, LV_PART_MAIN);
    }

    /* ---- Cell voltage spread ----------------------------------------- */
    if (f & DFLAG_CELLS) {
        char buf[16];
        float spread = d->cells.spread_mv;
        snprintf(buf, sizeof(buf), "%d mV", (int)spread);
        lv_label_set_text(lbl_cell_delta, buf);
        lv_obj_set_style_text_color(lbl_cell_delta,
            (spread > 50.0f) ? C_RED : (spread > 20.0f) ? C_YELLOW : C_WHITE,
            LV_PART_MAIN);
    }

    /* ---- Total solar input (sum of 4 MPPTs) -------------------------- */
    if (f & (DFLAG_MPPT1 | DFLAG_MPPT2 | DFLAG_MPPT3 | DFLAG_MPPT4)) {
        char buf[16];
        float total = d->mppt[0].input_power_w + d->mppt[1].input_power_w
                    + d->mppt[2].input_power_w + d->mppt[3].input_power_w;
        snprintf(buf, sizeof(buf), "%d W", (int)total);
        lv_label_set_text(lbl_solar, buf);
        lv_obj_set_style_text_color(lbl_solar,
            (total > 600.0f) ? C_GREEN : C_WHITE, LV_PART_MAIN);
    }

    /* ---- Fault overlay ----------------------------------------------- */
    if (f & DFLAG_BPS_SAFETY) {
        bool fault_active = (d->bps_safety.safety_state != SAFETY_NORMAL);

        if (fault_active) {
            uint8_t fc = d->bps_safety.fault_code;
            const char *name = (fc < 14) ? BPS_FAULT_NAMES[fc] : "UNKNOWN FAULT";
            lv_label_set_text(lbl_fault_name, name);

            if (d->bps_safety.grace_timer_s > 0) {
                lv_label_set_text_fmt(lbl_fault_timer,
                    "Limp home: %ds remaining", d->bps_safety.grace_timer_s);
            } else {
                lv_label_set_text(lbl_fault_timer, "");
            }
            lv_obj_clear_flag(fault_overlay, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(fault_overlay, LV_OBJ_FLAG_HIDDEN);
        }
    }

    lvgl_port_unlock();
}
