/*
 * ui.c — Solar car telemetry dashboard  1280×800 landscape
 *
 * LEFT (380px): permanent Driver HUD
 *   drive mode · speed · power · status dots (2 rows) · precharge/HV_ready
 *   SOC bar · throttle/regen bars · odo/24V · warning strip
 *
 * RIGHT (900px): three touch tabs
 *   BATTERY    — pack V/A/kW, cell extremes, temps, SOH, cell range bar,
 *                balancing, energy, adaptive energy
 *   SOLAR/MOTOR — total solar, 4 MPPT bars, motor RPM/A, temps, net power
 *   GPS/TRIP   — speed, heading, altitude, sats, odo, UTC, lat/lon
 *
 * FAULT OVERLAY: covers right panel only, red bg, non-dismissable
 *
 * CAMERA PiP: bottom-right, topmost (above the fault overlay) — the backup
 * camera must always be visible. An lv_canvas shows the RGB565 buffer the
 * camera PPA writes into; see ui_cam_pip_*.
 */

#include "ui.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "display_can_spec.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "UI";

extern const lv_font_t lv_font_montserrat_72;

/* =========================================================================
 * Colors
 * ========================================================================= */
#define C_BG lv_color_hex(0x0D0D1A)
#define C_PANEL lv_color_hex(0x14142B)
#define C_TILE lv_color_hex(0x1A1A33)
#define C_DIV lv_color_hex(0x2A2A4A)
#define C_WHITE lv_color_white()
#define C_GRAY lv_color_hex(0xAAAAAA)
#define C_DGRAY lv_color_hex(0x444466)
#define C_GREEN lv_color_hex(0x00DD55)
#define C_YELLOW lv_color_hex(0xFFCC00)
#define C_AMBER lv_color_hex(0xFF8800)
#define C_RED lv_color_hex(0xDD2222)
#define C_DARK_RED lv_color_hex(0x7A0000)
#define C_BLUE lv_color_hex(0x00AAFF)
#define C_ORANGE lv_color_hex(0xFF6600)
#define C_GOLD lv_color_hex(0xFFCC00)

/* =========================================================================
 * Layout (px, 1280×800 landscape after software rotation)
 * ========================================================================= */
#define SCR_W 1280
#define SCR_H 800
#define LEFT_W 380
#define RIGHT_X LEFT_W
#define RIGHT_W (SCR_W - LEFT_W) /* 900 */
#define TAB_H 50
#define DOT_SZ 26    /* main status indicator dots */
#define DOT_SZ_SM 18 /* contactor / precharge dots */
#define BAR_H 20
#define TBAR_W ((LEFT_W - 2 * LP_PAD - 8) / 2) /* throttle/regen bar width each (~172px) */
#define CELL_BAR_W 840                         /* cell voltage range bar width */
#define MPPT_BAR_W 280                         /* per-MPPT bar width */

/* Camera PiP window (bottom-right corner). 480x240 shows the full sensor
 * width at the PPA's 6/16 scale with a centered vertical crop — a wide
 * rear-view-mirror strip. Both dims must keep the camera's PPA scale on its
 * 1/16 grid: width a multiple of 80 px (see camera_config_pip()). The bottom
 * band under the tab content (screen y >= ~540) is clear on every tab. */
#define CAM_PIP_W 480
#define CAM_PIP_H 240
#define CAM_PIP_BORDER 2
#define CAM_PIP_MARGIN 4 /* gap to the screen's bottom-right corner */

/* Left panel column x anchors */
#define LP_PAD 12

/* =========================================================================
 * Left-panel widget handles
 * ========================================================================= */
static lv_obj_t *lbl_drive_mode = NULL;

/* System status dots: Fault, Warn, DCDC, Batt, MC */
static lv_obj_t *dot_fault = NULL;
static lv_obj_t *dot_warn = NULL;
static lv_obj_t *dot_dcdc_sys = NULL;
static lv_obj_t *dot_batt_sys = NULL;
static lv_obj_t *dot_mc_sys = NULL;

/* Contactor relay dots: HV+, HV-, MC, WC, DCDC */
static lv_obj_t *dot_hv_pos = NULL;
static lv_obj_t *dot_hv_neg = NULL;
static lv_obj_t *dot_mc_relay = NULL;
static lv_obj_t *dot_wc_relay = NULL;
static lv_obj_t *dot_dcdc_relay = NULL;

/* Precharge / HV Ready */
static lv_obj_t *dot_prech = NULL;
static lv_obj_t *dot_hv_rdy = NULL;

static lv_obj_t *lbl_speed = NULL;
static lv_obj_t *lbl_power = NULL;
static lv_obj_t *bar_soc = NULL;
static lv_obj_t *lbl_soc = NULL;
/* throttle bar = lv_bar (L→R green); regen = manual fill object inside container */
static lv_obj_t *bar_throttle = NULL;
static lv_obj_t *regen_fill = NULL; /* blue fill inside regen container */
static lv_obj_t *lbl_odo = NULL;
static lv_obj_t *lbl_24v = NULL;
static lv_obj_t *warn_strip = NULL;
static lv_obj_t *status_label = NULL;

/* Pi telemetry-link indicator (bottom-left corner) */
static lv_obj_t *dot_pi = NULL;
static lv_obj_t *lbl_pi = NULL;

/* =========================================================================
 * Battery-tab widget handles
 * ========================================================================= */
static lv_obj_t *lbl_batt_pack_v = NULL;
static lv_obj_t *lbl_batt_pack_a = NULL;
static lv_obj_t *lbl_batt_hi_cell = NULL;
static lv_obj_t *lbl_batt_lo_cell = NULL;
static lv_obj_t *lbl_batt_hi_temp = NULL;
static lv_obj_t *lbl_batt_lo_temp = NULL;
static lv_obj_t *lbl_batt_soh = NULL;
static lv_obj_t *lbl_batt_spread = NULL;
static lv_obj_t *cell_range_fill = NULL; /* green fill between lo and hi ticks */
static lv_obj_t *tick_cell_hi = NULL;    /* orange tick on cell range bar */
static lv_obj_t *tick_cell_lo = NULL;
static lv_obj_t *dot_bal_active = NULL;
static lv_obj_t *lbl_bal_text = NULL;
static lv_obj_t *lbl_pack_ah = NULL;
static lv_obj_t *lbl_total_cap = NULL;
static lv_obj_t *lbl_dod = NULL;
static lv_obj_t *lbl_adapt_soc = NULL;
static lv_obj_t *lbl_adapt_ah = NULL;
static lv_obj_t *lbl_adapt_cap = NULL;

/* =========================================================================
 * Solar/Motor-tab widget handles
 * ========================================================================= */
static lv_obj_t *lbl_solar_total = NULL;
static lv_obj_t *bar_mppt[4] = {NULL};
static lv_obj_t *lbl_mppt_w[4] = {NULL};
static lv_obj_t *lbl_sol_rpm = NULL;
static lv_obj_t *lbl_sol_motor_a = NULL;
static lv_obj_t *lbl_sol_ctrl_temp = NULL;
static lv_obj_t *lbl_sol_motor_temp = NULL;
static lv_obj_t *lbl_net_power = NULL;

/* cached for net power calculation */
static float s_solar_total_w = 0.0f;
static float s_pack_power_kw = 0.0f;

/* =========================================================================
 * GPS-tab widget handles
 * ========================================================================= */
static lv_obj_t *lbl_gps_speed = NULL;
static lv_obj_t *lbl_gps_head = NULL;
static lv_obj_t *lbl_gps_alt = NULL;
static lv_obj_t *lbl_gps_sats = NULL;
static lv_obj_t *lbl_gps_odo = NULL;
static lv_obj_t *lbl_gps_utc = NULL;
static lv_obj_t *lbl_gps_lat = NULL;
static lv_obj_t *lbl_gps_lon = NULL;

/* =========================================================================
 * Tabview handle (for programmatic tab switching)
 * ========================================================================= */
static lv_obj_t *tabview = NULL;

/* =========================================================================
 * Fault overlay widget handles
 * ========================================================================= */
static lv_obj_t *fault_overlay = NULL;
static lv_obj_t *lbl_fault_name = NULL;
static lv_obj_t *lbl_fault_sub = NULL;
static lv_obj_t *fault_limp_cont = NULL; /* grace timer + derate, hidden when not limp */
static lv_obj_t *lbl_fault_timer = NULL;
static lv_obj_t *lbl_fault_derate = NULL;
static lv_obj_t *lbl_fault_thr = NULL;

/* =========================================================================
 * Camera PiP widget handles
 * ========================================================================= */
static lv_obj_t *cam_pip_frame = NULL;  /* border frame (position/show/hide) */
static lv_obj_t *cam_pip_canvas = NULL; /* canvas bound to cam_pip_buf */
static void *cam_pip_buf = NULL;        /* RGB565, PPA-written, PSRAM */

/* =========================================================================
 * Fault subtitles (matched to BPS_FAULT_NAMES index)
 * ========================================================================= */
static const char *BPS_FAULT_SUBTITLES[] = {
    "",
    "Supplemental battery undervolt",
    "Supplemental battery overvolt",
    "Supplemental battery undertemp",
    "Supplemental battery overtemp",
    "DCDC converter undervolt",
    "DCDC converter overvolt",
    "HV contactors will open",
    "Current sensor failure",
    "HV contactors will open",
    "Pack undervolt — HV open",
    "Pack overvolt — HV open",
    "CAN bus error",
    "CAN message timeout",
};

/* =========================================================================
 * Helper: panel (no scroll, no padding)
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

/* =========================================================================
 * Helper: label with explicit position (no alignment)
 * ========================================================================= */
static lv_obj_t *make_lbl(lv_obj_t *parent,
                          const char *text,
                          lv_color_t color,
                          const lv_font_t *font,
                          int x, int y)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_color(l, color, LV_PART_MAIN);
    lv_obj_set_style_text_font(l, font, LV_PART_MAIN);
    lv_obj_set_pos(l, x, y);
    return l;
}

/* =========================================================================
 * Helper: status dot (round, solid, no border)
 * ========================================================================= */
static lv_obj_t *make_dot(lv_obj_t *parent, int x, int y, lv_color_t color, int sz)
{
    lv_obj_t *d = lv_obj_create(parent);
    lv_obj_set_pos(d, x, y);
    lv_obj_set_size(d, sz, sz);
    lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(d, color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(d, 0, LV_PART_MAIN);
    lv_obj_clear_flag(d, LV_OBJ_FLAG_SCROLLABLE);
    return d;
}

static void dot_set(lv_obj_t *d, lv_color_t c)
{
    lv_obj_set_style_bg_color(d, c, LV_PART_MAIN);
}

/* =========================================================================
 * Helper: heading to 16-point cardinal string
 * ========================================================================= */
static const char *heading_cardinal(float deg)
{
    static const char *n[] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int d = ((int)(deg + 11.25f) % 360 + 360) % 360;
    return n[d * 16 / 360];
}

/* =========================================================================
 * Helper: two-column data block (caption + value)
 * Used inside tabs for consistent data rows.
 * ========================================================================= */
static lv_obj_t *tab_val(lv_obj_t *parent, const char *cap, const char *init,
                         int x, int y)
{
    make_lbl(parent, cap, C_GRAY, &lv_font_montserrat_14, x, y);
    return make_lbl(parent, init, C_WHITE, &lv_font_montserrat_24, x, y + 18);
}

/* =========================================================================
 * Helper: thin horizontal divider line
 * ========================================================================= */
static void make_divider(lv_obj_t *parent, int y)
{
    lv_obj_t *d = lv_obj_create(parent);
    lv_obj_set_pos(d, LP_PAD, y);
    lv_obj_set_size(d, LEFT_W - 2 * LP_PAD, 1);
    lv_obj_set_style_bg_color(d, C_DIV, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(d, 0, LV_PART_MAIN);
    lv_obj_clear_flag(d, LV_OBJ_FLAG_SCROLLABLE);
}

/* =========================================================================
 * Helper: centered dot row — creates N dots + labels, returns nothing.
 * lbl_w: fixed pixel width of each label cell (for center-align under dot).
 * ========================================================================= */
static void make_dot_row(lv_obj_t *parent, int y, int sz,
                         lv_obj_t **handles, const char **labels, int n)
{
    int gap = 28;
    int spacing = sz + gap;
    int row_w = n * sz + (n - 1) * gap;
    int x0 = (LEFT_W - row_w) / 2;
    int lbl_cell = sz + gap;

    for (int i = 0; i < n; i++)
    {
        handles[i] = make_dot(parent, x0 + i * spacing, y, C_DGRAY, sz);
        lv_obj_t *l = lv_label_create(parent);
        lv_label_set_text(l, labels[i]);
        lv_obj_set_style_text_color(l, C_GRAY, LV_PART_MAIN);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_pos(l, x0 + i * spacing - (lbl_cell - sz) / 2, y + sz + 3);
        lv_obj_set_width(l, lbl_cell);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    }
}

/* =========================================================================
 * Build left panel — Driver HUD
 *
 * Layout (y, 380×800):
 *   10  : Drive mode (top-left, 32px)
 *   50  : Speed (72px, centered) — "mph" inset to the right at digit baseline
 *  158  : Power (48px, centered)
 *  220  : STATUS label + 5 system dots (26px) + labels
 *  295  : CONTACTORS label + 5 relay dots (18px) + labels
 *  354  : Precharge / HV Ready inline row
 *  382  : ─ divider ─
 *  392  : SOC label + %
 *  420  : SOC bar
 *  454  : ─ divider ─
 *  467  : THROTTLE / REGEN labels
 *  485  : throttle bar + regen bar
 *  517  : Odometer + 24V
 *  537  : Status / IP label
 *  745  : Warning strip
 * ========================================================================= */
static void build_left_panel(lv_obj_t *scr)
{
    lv_obj_t *lp = make_panel(scr, 0, 0, LEFT_W, SCR_H, C_PANEL);

    /* ── Drive mode (top-left) ─────────────────────────────────────────── */
    lbl_drive_mode = make_lbl(lp, "PARK", C_GRAY, &lv_font_montserrat_32, LP_PAD, 10);

    /* ── Speed (72px, centered) ────────────────────────────────────────── */
    lbl_speed = lv_label_create(lp);
    lv_label_set_text(lbl_speed, "--");
    lv_obj_set_style_text_color(lbl_speed, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_speed, &lv_font_montserrat_72, LV_PART_MAIN);
    lv_obj_align(lbl_speed, LV_ALIGN_TOP_MID, 0, 50);

    /* "mph" centered directly below the speed number */
    lv_obj_t *lbl_unit = lv_label_create(lp);
    lv_label_set_text(lbl_unit, "mph");
    lv_obj_set_style_text_color(lbl_unit, C_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_unit, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(lbl_unit, LV_ALIGN_TOP_MID, 0, 140);

    /* ── Power (48px, centered) ────────────────────────────────────────── */
    lbl_power = lv_label_create(lp);
    lv_label_set_text(lbl_power, "--.- kW");
    lv_obj_set_style_text_color(lbl_power, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_power, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align(lbl_power, LV_ALIGN_TOP_MID, 0, 175);

    /* ── System status dots (Fault/Warn/DCDC/Batt/MC) ─────────────────── */
    make_lbl(lp, "STATUS", C_GRAY, &lv_font_montserrat_14, LP_PAD, 222);
    {
        lv_obj_t *h[5];
        static const char *lbl[5] = {"FAULT", "WARN", "DCDC", "BATT", "MC"};
        make_dot_row(lp, 240, DOT_SZ, h, lbl, 5);
        dot_fault = h[0];
        dot_warn = h[1];
        dot_dcdc_sys = h[2];
        dot_batt_sys = h[3];
        dot_mc_sys = h[4];
    }

    /* ── Contactor relay dots (HV+/HV-/MC/WC/DCDC) ────────────────────── */
    make_lbl(lp, "CONTACTORS", C_GRAY, &lv_font_montserrat_14, LP_PAD, 296);
    {
        lv_obj_t *h[5];
        static const char *lbl[5] = {"HV+", "HV-", "MC", "WC", "DCDC"};
        make_dot_row(lp, 312, DOT_SZ_SM, h, lbl, 5);
        dot_hv_pos = h[0];
        dot_hv_neg = h[1];
        dot_mc_relay = h[2];
        dot_wc_relay = h[3];
        dot_dcdc_relay = h[4];
    }

    /* ── Precharge / HV Ready inline ───────────────────────────────────── */
    /* Centered: [●] PRECH   [●] HV RDY */
    /* Total width: (18+6+42) + 20 + (18+6+48) = 158px → x0 = (380-158)/2 = 111 */
    {
        int pc_y = 354;
        int x0 = 108;
        dot_prech = make_dot(lp, x0, pc_y, C_DGRAY, DOT_SZ_SM);
        make_lbl(lp, "PRECH", C_GRAY, &lv_font_montserrat_14, x0 + DOT_SZ_SM + 5, pc_y + 2);
        dot_hv_rdy = make_dot(lp, x0 + 100, pc_y, C_DGRAY, DOT_SZ_SM);
        make_lbl(lp, "HV RDY", C_GRAY, &lv_font_montserrat_14, x0 + 100 + DOT_SZ_SM + 5, pc_y + 2);
    }

    make_divider(lp, 384);

    /* ── SOC: label left + % right, bar below ──────────────────────────── */
    make_lbl(lp, "SOC", C_GRAY, &lv_font_montserrat_14, LP_PAD, 395);
    lbl_soc = make_lbl(lp, "--%", C_GREEN, &lv_font_montserrat_32,
                       LEFT_W - LP_PAD - 62, 389);

    bar_soc = lv_bar_create(lp);
    lv_obj_set_size(bar_soc, LEFT_W - 2 * LP_PAD, BAR_H + 4);
    lv_bar_set_range(bar_soc, 0, 100);
    lv_bar_set_value(bar_soc, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_soc, C_DIV, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar_soc, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_soc, C_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_soc, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(bar_soc, 4, LV_PART_INDICATOR);
    lv_obj_set_pos(bar_soc, LP_PAD, 422);

    make_divider(lp, 456);

    /* ── Throttle (L→R green) + Regen (R→L blue) side by side ─────────── */
    int bar_w = (LEFT_W - 2 * LP_PAD - 8) / 2;

    make_lbl(lp, "THROTTLE", C_GRAY, &lv_font_montserrat_14, LP_PAD, 468);
    make_lbl(lp, "REGEN", C_GRAY, &lv_font_montserrat_14,
             LP_PAD + bar_w + 8, 468);

    bar_throttle = lv_bar_create(lp);
    lv_obj_set_size(bar_throttle, bar_w, BAR_H);
    lv_bar_set_range(bar_throttle, 0, 255);
    lv_bar_set_value(bar_throttle, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_throttle, C_DIV, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_throttle, C_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_throttle, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(bar_throttle, 4, LV_PART_INDICATOR);
    lv_obj_set_pos(bar_throttle, LP_PAD, 486);

    lv_obj_t *regen_bg = make_panel(lp, LP_PAD + bar_w + 8, 486, bar_w, BAR_H, C_DIV);
    lv_obj_set_style_radius(regen_bg, 4, LV_PART_MAIN);
    regen_fill = lv_obj_create(regen_bg);
    lv_obj_set_size(regen_fill, 0, BAR_H);
    lv_obj_set_pos(regen_fill, bar_w, 0);
    lv_obj_set_style_bg_color(regen_fill, C_BLUE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(regen_fill, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(regen_fill, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(regen_fill, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(regen_fill, 0, LV_PART_MAIN);
    lv_obj_clear_flag(regen_fill, LV_OBJ_FLAG_SCROLLABLE);

    /* ── Odometer + 24V ────────────────────────────────────────────────── */
    lbl_odo = make_lbl(lp, "ODO -- km", C_GRAY, &lv_font_montserrat_14, LP_PAD, 519);
    lbl_24v = make_lbl(lp, "24V --.- V", C_GRAY, &lv_font_montserrat_14,
                       LEFT_W / 2, 519);

    /* ── Status / IP label ─────────────────────────────────────────────── */
    status_label = make_lbl(lp, "Initializing...", C_GRAY,
                            &lv_font_montserrat_14, LP_PAD, 539);

    /* ── Warning strip (hidden; shown on derating) ─────────────────────── */
    warn_strip = make_panel(lp, 0, SCR_H - 55, LEFT_W, 55, C_AMBER);
    lv_obj_add_flag(warn_strip, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *warn_lbl = lv_label_create(warn_strip);
    lv_label_set_text(warn_lbl, "DERATING ACTIVE - SOFT LIMIT");
    lv_obj_set_style_text_color(warn_lbl, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(warn_lbl, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(warn_lbl, LV_ALIGN_CENTER, 0, 0);

    /* ── Pi telemetry-link indicator (bottom-left corner) ──────────────────
     * Created last so it sits on top of the warn strip on the rare occasions
     * that overlaps. Dark chip keeps it readable over any background. Starts in
     * the disconnected state; main.c polls network_pi_is_connected() to update. */
    {
        lv_obj_t *pi_chip = make_panel(lp, LP_PAD, SCR_H - 30, 158, 26, C_TILE);
        lv_obj_set_style_radius(pi_chip, 6, LV_PART_MAIN);
        dot_pi = make_dot(pi_chip, 7, 6, C_RED, 14);
        lbl_pi = lv_label_create(pi_chip);
        lv_label_set_text(lbl_pi, "PI OFFLINE");
        lv_obj_set_style_text_color(lbl_pi, C_GRAY, LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_pi, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_pos(lbl_pi, 28, 5);
    }
}

/* =========================================================================
 * Build right panel with three tabs
 * ========================================================================= */
static void build_right_panel(lv_obj_t *scr)
{
    /* Tabview fills the right side of the screen */
    tabview = lv_tabview_create(scr);
    lv_obj_t *tv = tabview;
    lv_obj_set_pos(tv, RIGHT_X, 0);
    lv_obj_set_size(tv, RIGHT_W, SCR_H);
    lv_tabview_set_tab_bar_position(tv, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tv, TAB_H);
    lv_obj_set_style_bg_color(tv, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(tv, 0, LV_PART_MAIN);

    /* Style the tab bar background */
    lv_obj_t *tab_bar = lv_tabview_get_tab_bar(tv);
    lv_obj_set_style_bg_color(tab_bar, C_PANEL, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_side(tab_bar, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_color(tab_bar, C_DIV, LV_PART_MAIN);
    lv_obj_set_style_border_width(tab_bar, 2, LV_PART_MAIN);

    /* Inactive tab items: white text, subtle background, right-side divider */
    lv_obj_set_style_text_color(tab_bar, C_WHITE, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(tab_bar, &lv_font_montserrat_14, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(tab_bar, lv_color_hex(0x1E1E38), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(tab_bar, C_DIV, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(tab_bar, LV_BORDER_SIDE_RIGHT, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(tab_bar, 1, LV_PART_ITEMS | LV_STATE_DEFAULT);

    /* Active tab item: white text, green bottom underline, slightly lighter bg */
    lv_obj_set_style_text_color(tab_bar, C_WHITE, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_font(tab_bar, &lv_font_montserrat_14, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(tab_bar, C_TILE, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(tab_bar, C_GREEN, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_side(tab_bar, LV_BORDER_SIDE_BOTTOM, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tab_bar, 3, LV_PART_ITEMS | LV_STATE_CHECKED);

    lv_obj_t *t_batt = lv_tabview_add_tab(tv, "BATTERY");
    lv_obj_t *t_solar = lv_tabview_add_tab(tv, "SOLAR / MOTOR");
    lv_obj_t *t_gps = lv_tabview_add_tab(tv, "GPS / TRIP");

    /* ---- Content page background and no-scroll ---- */
    lv_color_t tabs[] = {C_BG, C_BG, C_BG};
    lv_obj_t *tabs_arr[] = {t_batt, t_solar, t_gps};
    for (int i = 0; i < 3; i++)
    {
        lv_obj_set_style_bg_color(tabs_arr[i], tabs[i], LV_PART_MAIN);
        lv_obj_set_style_bg_opa(tabs_arr[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_pad_all(tabs_arr[i], 0, LV_PART_MAIN);
        lv_obj_clear_flag(tabs_arr[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    /* ==========================================================
     * BATTERY TAB
     * ========================================================== */
    int col_l = 20, col_r = 460;

    /* Pack voltage + pack current */
    make_lbl(t_batt, "PACK VOLTAGE", C_GRAY, &lv_font_montserrat_14, col_l, 12);
    lbl_batt_pack_v = make_lbl(t_batt, "--.- V", C_WHITE,
                               &lv_font_montserrat_32, col_l, 30);

    make_lbl(t_batt, "PACK CURRENT", C_GRAY, &lv_font_montserrat_14, col_r, 12);
    lbl_batt_pack_a = make_lbl(t_batt, "--.- A (--.- kW)", C_WHITE,
                               &lv_font_montserrat_24, col_r, 32);

    /* Cell extremes */
    make_lbl(t_batt, "HIGH CELL", C_GRAY, &lv_font_montserrat_14, col_l, 84);
    lbl_batt_hi_cell = make_lbl(t_batt, "-.--- V  Cell --",
                                C_ORANGE, &lv_font_montserrat_24, col_l, 102);

    make_lbl(t_batt, "LOW CELL", C_GRAY, &lv_font_montserrat_14, col_r, 84);
    lbl_batt_lo_cell = make_lbl(t_batt, "-.--- V  Cell --",
                                C_BLUE, &lv_font_montserrat_24, col_r, 102);

    /* Temperature extremes */
    make_lbl(t_batt, "HIGH TEMP", C_GRAY, &lv_font_montserrat_14, col_l, 150);
    lbl_batt_hi_temp = make_lbl(t_batt, "--\xc2\xb0"
                                        "C  T--",
                                C_AMBER, &lv_font_montserrat_24, col_l, 168);

    make_lbl(t_batt, "LOW TEMP", C_GRAY, &lv_font_montserrat_14, col_r, 150);
    lbl_batt_lo_temp = make_lbl(t_batt, "--\xc2\xb0"
                                        "C  T--",
                                C_WHITE, &lv_font_montserrat_24, col_r, 168);

    /* SOH + Spread */
    make_lbl(t_batt, "SOH", C_GRAY, &lv_font_montserrat_14, col_l, 214);
    lbl_batt_soh = make_lbl(t_batt, "--%", C_WHITE, &lv_font_montserrat_24, col_l, 232);

    make_lbl(t_batt, "CELL SPREAD", C_GRAY, &lv_font_montserrat_14, col_r, 214);
    lbl_batt_spread = make_lbl(t_batt, "-- mV", C_WHITE, &lv_font_montserrat_24, col_r, 232);

    /* Cell voltage range bar */
    make_lbl(t_batt, "CELL VOLTAGE RANGE", C_GRAY, &lv_font_montserrat_14, col_l, 275);
    make_lbl(t_batt, "2.5V", C_GRAY, &lv_font_montserrat_14, col_l, 296);
    make_lbl(t_batt, "4.2V", C_GRAY, &lv_font_montserrat_14, col_l + CELL_BAR_W - 28, 296);

    /* Bar background */
    lv_obj_t *crb = make_panel(t_batt, col_l, 314, CELL_BAR_W, 18, C_DIV);
    lv_obj_set_style_radius(crb, 3, LV_PART_MAIN);
    /* Green fill between lo and hi ticks — updated dynamically */
    cell_range_fill = make_panel(crb, 0, 0, 0, 18, C_GREEN);
    /* Orange boundary ticks — drawn on top (later z-order) */
    tick_cell_hi = make_panel(crb, 0, 0, 4, 18, C_ORANGE);
    tick_cell_lo = make_panel(crb, 0, 0, 4, 18, C_ORANGE);

    /* Cell balancing active */
    dot_bal_active = make_dot(t_batt, col_l, 344, C_DGRAY, DOT_SZ_SM);
    lbl_bal_text = make_lbl(t_batt, "CELL BALANCING: OFF",
                            C_GRAY, &lv_font_montserrat_14, col_l + DOT_SZ + 6, 348);

    /* R_BMS_Energy row */
    int col3 = 300, col4 = 580;
    make_lbl(t_batt, "PACK Ah", C_GRAY, &lv_font_montserrat_14, col_l, 378);
    make_lbl(t_batt, "TOTAL CAP", C_GRAY, &lv_font_montserrat_14, col3, 378);
    make_lbl(t_batt, "DOD", C_GRAY, &lv_font_montserrat_14, col4, 378);
    lbl_pack_ah = make_lbl(t_batt, "--.- Ah", C_WHITE, &lv_font_montserrat_24, col_l, 396);
    lbl_total_cap = make_lbl(t_batt, "--.- Ah", C_WHITE, &lv_font_montserrat_24, col3, 396);
    lbl_dod = make_lbl(t_batt, "--.-%", C_WHITE, &lv_font_montserrat_24, col4, 396);

    /* R_BMS_AdaptEnergy row */
    make_lbl(t_batt, "ADAPT SOC", C_GRAY, &lv_font_montserrat_14, col_l, 438);
    make_lbl(t_batt, "ADAPT Ah", C_GRAY, &lv_font_montserrat_14, col3, 438);
    make_lbl(t_batt, "ADAPT CAP", C_GRAY, &lv_font_montserrat_14, col4, 438);
    lbl_adapt_soc = make_lbl(t_batt, "--.-%", C_WHITE, &lv_font_montserrat_24, col_l, 456);
    lbl_adapt_ah = make_lbl(t_batt, "--.- Ah", C_WHITE, &lv_font_montserrat_24, col3, 456);
    lbl_adapt_cap = make_lbl(t_batt, "--.- Ah", C_WHITE, &lv_font_montserrat_24, col4, 456);

    /* ==========================================================
     * SOLAR / MOTOR TAB
     * ========================================================== */
    int sol_col_l = 20, sol_col_r = 480;
    int mppt_max_w = 400; /* scale: 400W = full bar */

    make_lbl(t_solar, "TOTAL SOLAR INPUT", C_GRAY, &lv_font_montserrat_14, sol_col_l, 12);
    lbl_solar_total = make_lbl(t_solar, "-- W", C_GOLD, &lv_font_montserrat_48, sol_col_l, 30);

    /* 4 MPPT bars */
    static const char *mppt_names[] = {"A", "B", "C", "D"};
    for (int i = 0; i < 4; i++)
    {
        int my = 110 + i * 36;
        make_lbl(t_solar, mppt_names[i], C_GRAY, &lv_font_montserrat_14, sol_col_l, my);
        bar_mppt[i] = lv_bar_create(t_solar);
        lv_obj_set_size(bar_mppt[i], MPPT_BAR_W, 16);
        lv_bar_set_range(bar_mppt[i], 0, mppt_max_w);
        lv_bar_set_value(bar_mppt[i], 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(bar_mppt[i], C_DIV, LV_PART_MAIN);
        lv_obj_set_style_bg_color(bar_mppt[i], C_GOLD, LV_PART_INDICATOR);
        lv_obj_set_style_radius(bar_mppt[i], 3, LV_PART_MAIN);
        lv_obj_set_style_radius(bar_mppt[i], 3, LV_PART_INDICATOR);
        lv_obj_set_pos(bar_mppt[i], sol_col_l + 18, my + 2);
        lbl_mppt_w[i] = make_lbl(t_solar, "-- W", C_WHITE, &lv_font_montserrat_14,
                                 sol_col_l + 18 + MPPT_BAR_W + 6, my + 2);
    }

    /* Motor side */
    make_lbl(t_solar, "MOTOR RPM", C_GRAY, &lv_font_montserrat_14, sol_col_r, 12);
    make_lbl(t_solar, "MOTOR CURRENT", C_GRAY, &lv_font_montserrat_14, sol_col_r + 200, 12);
    lbl_sol_rpm = make_lbl(t_solar, "----", C_WHITE, &lv_font_montserrat_32, sol_col_r, 30);
    lbl_sol_motor_a = make_lbl(t_solar, "--.- A", C_WHITE, &lv_font_montserrat_32, sol_col_r + 200, 30);

    make_lbl(t_solar, "CTRL TEMP", C_GRAY, &lv_font_montserrat_14, sol_col_r, 110);
    make_lbl(t_solar, "MOTOR TEMP", C_GRAY, &lv_font_montserrat_14, sol_col_r + 200, 110);
    lbl_sol_ctrl_temp = make_lbl(t_solar, "--\xc2\xb0"
                                          "C",
                                 C_WHITE, &lv_font_montserrat_32, sol_col_r, 128);
    lbl_sol_motor_temp = make_lbl(t_solar, "--\xc2\xb0"
                                           "C",
                                  C_WHITE, &lv_font_montserrat_32, sol_col_r + 200, 128);

    make_lbl(t_solar, "NET POWER", C_GRAY, &lv_font_montserrat_14, sol_col_r, 195);
    lbl_net_power = make_lbl(t_solar, "Solar -- W\nMotor -- W\n= -- W",
                             C_WHITE, &lv_font_montserrat_24, sol_col_r, 214);

    /* ==========================================================
     * GPS / TRIP TAB
     * ========================================================== */
    int g_col_l = 20, g_col_r = 460;

    lbl_gps_speed = tab_val(t_gps, "GPS SPEED", "--.- mph", g_col_l, 12);
    lbl_gps_head = tab_val(t_gps, "HEADING", "---\xc2\xb0 --", g_col_r, 12);

    lbl_gps_alt = tab_val(t_gps, "ALTITUDE", "--- m MSL", g_col_l, 74);
    lbl_gps_sats = tab_val(t_gps, "SATELLITES", "--  -- fix", g_col_r, 74);

    lbl_gps_odo = tab_val(t_gps, "ODOMETER", "---.- km", g_col_l, 136);
    lbl_gps_utc = tab_val(t_gps, "UTC TIME", "--:--:--", g_col_r, 136);

    make_lbl(t_gps, "LATITUDE", C_GRAY, &lv_font_montserrat_14, g_col_l, 198);
    make_lbl(t_gps, "LONGITUDE", C_GRAY, &lv_font_montserrat_14, g_col_r, 198);
    lbl_gps_lat = make_lbl(t_gps, "---.----\xc2\xb0 N/S", C_WHITE, &lv_font_montserrat_24, g_col_l, 216);
    lbl_gps_lon = make_lbl(t_gps, "---.----\xc2\xb0 E/W", C_WHITE, &lv_font_montserrat_24, g_col_r, 216);
}

/* =========================================================================
 * Build fault overlay (right panel only, initially hidden)
 * ========================================================================= */
static void build_fault_overlay(lv_obj_t *scr)
{
    fault_overlay = lv_obj_create(scr);
    lv_obj_set_pos(fault_overlay, RIGHT_X, 0);
    lv_obj_set_size(fault_overlay, RIGHT_W, SCR_H);
    lv_obj_set_style_bg_color(fault_overlay, C_DARK_RED, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(fault_overlay, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(fault_overlay, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(fault_overlay, 0, LV_PART_MAIN);
    lv_obj_clear_flag(fault_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(fault_overlay, LV_OBJ_FLAG_HIDDEN);

    /* Warning icon "(!)" */
    lv_obj_t *icon_bg = lv_obj_create(fault_overlay);
    lv_obj_set_size(icon_bg, 64, 64);
    lv_obj_set_style_radius(icon_bg, 32, LV_PART_MAIN);
    lv_obj_set_style_bg_color(icon_bg, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(icon_bg, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(icon_bg, 0, LV_PART_MAIN);
    lv_obj_clear_flag(icon_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(icon_bg, LV_ALIGN_CENTER, 0, -165);
    lv_obj_t *icon_lbl = lv_label_create(icon_bg);
    lv_label_set_text(icon_lbl, "!");
    lv_obj_set_style_text_color(icon_lbl, C_DARK_RED, LV_PART_MAIN);
    lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align(icon_lbl, LV_ALIGN_CENTER, 0, 0);

    /* Fault name — large */
    lbl_fault_name = lv_label_create(fault_overlay);
    lv_label_set_text(lbl_fault_name, "FAULT");
    lv_obj_set_style_text_color(lbl_fault_name, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_fault_name, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align(lbl_fault_name, LV_ALIGN_CENTER, 0, -98);

    /* Fault code + subtitle */
    lbl_fault_sub = lv_label_create(fault_overlay);
    lv_label_set_text(lbl_fault_sub, "CODE 0");
    lv_obj_set_style_text_color(lbl_fault_sub, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_fault_sub, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(lbl_fault_sub, LV_ALIGN_CENTER, 0, -50);

    /* Limp-home container (grace timer + derate + throttle limit) */
    fault_limp_cont = lv_obj_create(fault_overlay);
    lv_obj_set_size(fault_limp_cont, RIGHT_W - 40, 200);
    lv_obj_set_style_bg_opa(fault_limp_cont, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(fault_limp_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(fault_limp_cont, 0, LV_PART_MAIN);
    lv_obj_clear_flag(fault_limp_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(fault_limp_cont, LV_ALIGN_CENTER, 0, 60);

    int half = (RIGHT_W - 40) / 2;
    make_lbl(fault_limp_cont, "GRACE TIMER", C_GRAY, &lv_font_montserrat_14, 0, 0);
    make_lbl(fault_limp_cont, "DERATING", C_GRAY, &lv_font_montserrat_14, half, 0);
    lbl_fault_timer = make_lbl(fault_limp_cont, "--",
                               C_WHITE, &lv_font_montserrat_48, 0, 18);
    make_lbl(fault_limp_cont, "sec", C_GRAY, &lv_font_montserrat_24, 90, 40);
    lbl_fault_derate = make_lbl(fault_limp_cont, "--",
                                C_WHITE, &lv_font_montserrat_48, half, 18);
    make_lbl(fault_limp_cont, "%", C_GRAY, &lv_font_montserrat_24, half + 90, 40);

    /* Throttle limit pill */
    lv_obj_t *thr_box = make_panel(fault_limp_cont, 0, 110, RIGHT_W - 40, 44, lv_color_hex(0xAA0000));
    lv_obj_set_style_radius(thr_box, 8, LV_PART_MAIN);
    lbl_fault_thr = lv_label_create(thr_box);
    lv_label_set_text(lbl_fault_thr, "THROTTLE LIMITED TO --- / 255");
    lv_obj_set_style_text_color(lbl_fault_thr, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_fault_thr, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(lbl_fault_thr, LV_ALIGN_CENTER, 0, 0);
}

/* =========================================================================
 * Build camera PiP window — bottom-right, created last so it stays on top of
 * every tab and the fault overlay (the backup camera must always be visible;
 * in reverse the camera goes fullscreen and covers everything anyway).
 * ========================================================================= */
static void build_cam_pip(lv_obj_t *scr)
{
    /* PPA DMA target: 128-byte aligned, size is a multiple of 128 (480*240*2) */
    cam_pip_buf = heap_caps_aligned_calloc(128, 1, CAM_PIP_W * CAM_PIP_H * 2,
                                           MALLOC_CAP_SPIRAM);
    if (!cam_pip_buf)
    {
        ESP_LOGE(TAG, "PiP buffer alloc failed (%d bytes) — no camera window",
                 CAM_PIP_W * CAM_PIP_H * 2);
        return;
    }

    cam_pip_frame = make_panel(scr,
                               SCR_W - CAM_PIP_W - 2 * CAM_PIP_BORDER - CAM_PIP_MARGIN,
                               SCR_H - CAM_PIP_H - 2 * CAM_PIP_BORDER - CAM_PIP_MARGIN,
                               CAM_PIP_W + 2 * CAM_PIP_BORDER,
                               CAM_PIP_H + 2 * CAM_PIP_BORDER,
                               C_DIV);

    cam_pip_canvas = lv_canvas_create(cam_pip_frame);
    lv_canvas_set_buffer(cam_pip_canvas, cam_pip_buf, CAM_PIP_W, CAM_PIP_H,
                         LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(cam_pip_canvas, CAM_PIP_BORDER, CAM_PIP_BORDER);
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

    build_left_panel(scr);
    build_right_panel(scr);
    build_fault_overlay(scr); /* above the panels */
    build_cam_pip(scr);       /* must be last — highest z-order */

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
 * ui_set_text / ui_set_status — write to the status bar in left panel
 *
 * Bounded lock: when the camera is fullscreen the stream task holds the LVGL
 * lock for the whole phase, so a plain lock here would stall the caller (the
 * TCP task) until reverse ends. Waiting a few ms rides out a normal render
 * tick; beyond that the update is dropped on purpose.
 * NOTE: lvgl_port_lock(0) means "wait forever", NOT try-lock.
 * ========================================================================= */
#define UI_LOCK_TIMEOUT_MS 50

void ui_set_text(const char *text)
{
    if (!status_label || !text)
        return;
    if (!lvgl_port_lock(UI_LOCK_TIMEOUT_MS))
        return;
    lv_label_set_text(status_label, text);
    lvgl_port_unlock();
}

void ui_set_status(const char *status)
{
    if (!status_label || !status)
        return;
    if (!lvgl_port_lock(UI_LOCK_TIMEOUT_MS))
        return;
    lv_label_set_text(status_label, status);
    lvgl_port_unlock();
}

void ui_set_tab(uint8_t tab_index)
{
    if (!tabview)
        return;
    if (!lvgl_port_lock(UI_LOCK_TIMEOUT_MS))
        return;
    lv_tabview_set_active(tabview, tab_index, LV_ANIM_OFF);
    lvgl_port_unlock();
}

/* =========================================================================
 * Camera PiP plumbing — see camera_config_pip() in the camera component
 * ========================================================================= */
void *ui_cam_pip_buffer(uint32_t *w, uint32_t *h)
{
    if (w)
        *w = CAM_PIP_W;
    if (h)
        *h = CAM_PIP_H;
    return cam_pip_buf;
}

void ui_cam_pip_frame_ready(void)
{
    if (!cam_pip_canvas)
        return;
    /* Short bounded wait: if LVGL is mid-render just drop this repaint — the
     * buffer already holds the newest frame and the next callback retries. */
    if (!lvgl_port_lock(10))
        return;
    lv_obj_invalidate(cam_pip_canvas);
    lvgl_port_unlock();
}

void ui_cam_pip_set_active(bool active)
{
    if (!cam_pip_frame)
        return;
    if (!lvgl_port_lock(100))
        return;
    if (active)
        lv_obj_clear_flag(cam_pip_frame, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(cam_pip_frame, LV_OBJ_FLAG_HIDDEN);
    lvgl_port_unlock();
}

void ui_set_pi_status(bool connected)
{
    if (!dot_pi || !lbl_pi)
        return;

    /* De-dupe: only touch widgets on an actual state change. shown_state stays
     * unchanged if the lock can't be taken (camera fullscreen), so the caller's
     * next poll retries until it sticks. -1 = nothing shown yet. */
    static int shown_state = -1;
    if (shown_state == (int)connected)
        return;
    if (!lvgl_port_lock(20))
        return;

    dot_set(dot_pi, connected ? C_GREEN : C_RED);
    lv_label_set_text(lbl_pi, connected ? "PI ONLINE" : "PI OFFLINE");
    lv_obj_set_style_text_color(lbl_pi, connected ? C_GREEN : C_GRAY, LV_PART_MAIN);

    shown_state = (int)connected;
    lvgl_port_unlock();
}

/* =========================================================================
 * ui_update_can_data — update all widgets from latest CAN state
 * ========================================================================= */
void ui_update_can_data(const display_data_t *d)
{
    if (!d)
        return;
    /* Bounded: outwaits a normal render tick, but drops the update instead of
     * stalling the TCP task if the camera grabbed the lock for fullscreen in
     * the window between main.c's camera_is_fullscreen() check and here.
     * Dropped updates self-heal on the next frame of the same CAN ID. */
    if (!lvgl_port_lock(UI_LOCK_TIMEOUT_MS))
        return;

    uint32_t f = d->update_flags;
    char buf[64];

    /* ---- Speed (Vega_VehicleStatus) ------------------------------------ */
    if (f & DFLAG_VEGA_STATUS)
    {
        float mph = d->vega_status.speed_kmh * 0.621371f;
        snprintf(buf, sizeof(buf), "%d", (int)mph);
        lv_label_set_text(lbl_speed, buf);

        /* GPS tab odometer */
        int odo10 = (int)(d->vega_status.odometer_km * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d km", odo10 / 10, odo10 % 10);
        lv_label_set_text(lbl_odo, buf);
        lv_label_set_text(lbl_gps_odo, buf);
    }

    /* ---- Drive mode + throttle/regen (Vega_DriverInputs) --------------- */
    if (f & DFLAG_VEGA_INPUTS)
    {
        static const char *mode_str[] = {"PARK", "FWD", "REV", "???"};
        (void)0; /* mode color computed inline below */
        uint8_t m = (d->vega_inputs.drive_mode < 3) ? d->vega_inputs.drive_mode : 3;
        lv_label_set_text(lbl_drive_mode, mode_str[m]);
        lv_color_t mc = (m == 1) ? C_GREEN : (m == 2) ? C_ORANGE
                                                      : C_GRAY;
        lv_obj_set_style_text_color(lbl_drive_mode, mc, LV_PART_MAIN);

        /* Throttle bar */
        lv_bar_set_value(bar_throttle, d->vega_inputs.throttle_out, LV_ANIM_OFF);

        /* Regen bar (R→L) */
        int rw = (int)((float)d->vega_inputs.regen_out / 255.0f * TBAR_W);
        lv_obj_set_size(regen_fill, rw, BAR_H);
        lv_obj_set_pos(regen_fill, TBAR_W - rw, 0);
    }

    /* ---- SOC + pack electrical (BPS_PowerLimit) ------------------------ */
    if (f & DFLAG_BPS_POWER)
    {
        float soc = d->bps_power.soc_pct;
        lv_bar_set_value(bar_soc, (int32_t)soc, LV_ANIM_OFF);
        snprintf(buf, sizeof(buf), "%d%%", (int)soc);
        lv_label_set_text(lbl_soc, buf);
        lv_color_t sc = (soc > 30.0f) ? C_GREEN : (soc > 15.0f) ? C_YELLOW
                                                                : C_RED;
        lv_obj_set_style_bg_color(bar_soc, sc, LV_PART_INDICATOR);
        lv_obj_set_style_text_color(lbl_soc, sc, LV_PART_MAIN);

        /* Power kW (center-dominant) */
        float kw = d->bps_power.pack_power_kw;
        int kw10 = (int)(kw >= 0 ? kw * 10.0f : -kw * 10.0f);
        if (kw >= 0)
            snprintf(buf, sizeof(buf), "+%d.%d kW", kw10 / 10, kw10 % 10);
        else
            snprintf(buf, sizeof(buf), "-%d.%d kW", kw10 / 10, kw10 % 10);
        lv_label_set_text(lbl_power, buf);
        lv_obj_set_style_text_color(lbl_power,
                                    (kw < 0.0f) ? C_BLUE : C_WHITE, LV_PART_MAIN);

        /* Battery tab: pack V + pack A */
        int v10 = (int)(d->bps_power.pack_voltage_v * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d V", v10 / 10, v10 % 10);
        lv_label_set_text(lbl_batt_pack_v, buf);

        float a = d->bps_power.pack_current_a;
        int a10 = (int)(a >= 0 ? a * 10.0f : -a * 10.0f);
        int pkw10 = (int)(kw >= 0 ? kw * 100.0f : -kw * 100.0f);
        if (a >= 0)
            snprintf(buf, sizeof(buf), "+%d.%d A (+%d.%02d kW)",
                     a10 / 10, a10 % 10, pkw10 / 100, pkw10 % 100);
        else
            snprintf(buf, sizeof(buf), "-%d.%d A (-%d.%02d kW)",
                     a10 / 10, a10 % 10, pkw10 / 100, pkw10 % 100);
        lv_label_set_text(lbl_batt_pack_a, buf);
        lv_obj_set_style_text_color(lbl_batt_pack_a,
                                    (a < 0.0f) ? C_BLUE : C_WHITE, LV_PART_MAIN);

        /* Cache pack power for net calculation */
        s_pack_power_kw = kw;
    }

    /* ---- 24V supply (BPS_SupplBattery) --------------------------------- */
    if (f & DFLAG_SUPPL)
    {
        int sv100 = (int)(d->suppl.voltage_v * 100.0f);
        snprintf(buf, sizeof(buf), "24V %d.%02d V", sv100 / 100, sv100 % 100);
        lv_label_set_text(lbl_24v, buf);
    }

    /* ---- Cell extremes (R_BMS_CellExtremes) ---------------------------- */
    if (f & DFLAG_CELLS)
    {
        int hv = (int)(d->cells.hi_cell_v * 1000.0f);
        snprintf(buf, sizeof(buf), "%d.%03d V  Cell %02d",
                 hv / 1000, hv % 1000, d->cells.hi_cell_id);
        lv_label_set_text(lbl_batt_hi_cell, buf);
        lv_obj_set_style_text_color(lbl_batt_hi_cell, C_ORANGE, LV_PART_MAIN);

        int lv_ = (int)(d->cells.lo_cell_v * 1000.0f);
        snprintf(buf, sizeof(buf), "%d.%03d V  Cell %02d",
                 lv_ / 1000, lv_ % 1000, d->cells.lo_cell_id);
        lv_label_set_text(lbl_batt_lo_cell, buf);
        lv_obj_set_style_text_color(lbl_batt_lo_cell, C_BLUE, LV_PART_MAIN);

        float spread = d->cells.spread_mv;
        snprintf(buf, sizeof(buf), "%d mV", (int)spread);
        lv_label_set_text(lbl_batt_spread, buf);
        lv_obj_set_style_text_color(lbl_batt_spread,
                                    (spread > 50.0f) ? C_RED : (spread > 20.0f) ? C_YELLOW
                                                                                : C_WHITE,
                                    LV_PART_MAIN);

        /* Cell voltage range bar ticks */
        float range_v = 4.2f - 2.5f; /* 1.7V */
        int xhi = (int)((d->cells.hi_cell_v - 2.5f) / range_v * CELL_BAR_W);
        int xlo = (int)((d->cells.lo_cell_v - 2.5f) / range_v * CELL_BAR_W);
        if (xhi < 0)
            xhi = 0;
        if (xhi > CELL_BAR_W - 4)
            xhi = CELL_BAR_W - 4;
        if (xlo < 0)
            xlo = 0;
        if (xlo > CELL_BAR_W - 4)
            xlo = CELL_BAR_W - 4;
        lv_obj_set_pos(tick_cell_hi, xhi, 0);
        lv_obj_set_pos(tick_cell_lo, xlo, 0);
        /* Green fill between lo right-edge and hi left-edge */
        int fill_x = xlo + 4;
        int fill_w = xhi - xlo - 4;
        if (fill_w < 0)
            fill_w = 0;
        lv_obj_set_pos(cell_range_fill, fill_x, 0);
        lv_obj_set_width(cell_range_fill, fill_w);
    }

    /* ---- Temperature extremes (R_BMS_TempExtremes) --------------------- */
    if (f & DFLAG_TEMPS)
    {
        snprintf(buf, sizeof(buf), "%d\xc2\xb0"
                                   "C  T%02d",
                 d->temps.hi_temp_c, d->temps.hi_temp_id);
        lv_label_set_text(lbl_batt_hi_temp, buf);
        lv_obj_set_style_text_color(lbl_batt_hi_temp,
                                    (d->temps.hi_temp_c > 50) ? C_RED : C_AMBER, LV_PART_MAIN);

        snprintf(buf, sizeof(buf), "%d\xc2\xb0"
                                   "C  T%02d",
                 d->temps.lo_temp_c, d->temps.lo_temp_id);
        lv_label_set_text(lbl_batt_lo_temp, buf);
    }

    /* ---- Motor + controller temps (MC_Status2) ------------------------- */
    if (f & DFLAG_MC_STATUS2)
    {
        int mt = d->mc_status2.motor_temp_c;
        int ct = d->mc_status2.ctrl_temp_c;

        snprintf(buf, sizeof(buf), "%d\xc2\xb0"
                                   "C",
                 mt);
        lv_label_set_text(lbl_sol_motor_temp, buf);
        lv_obj_set_style_text_color(lbl_sol_motor_temp,
                                    (mt > 80) ? C_RED : (mt > 60) ? C_YELLOW
                                                                  : C_WHITE,
                                    LV_PART_MAIN);

        snprintf(buf, sizeof(buf), "%d\xc2\xb0"
                                   "C",
                 ct);
        lv_label_set_text(lbl_sol_ctrl_temp, buf);
        lv_obj_set_style_text_color(lbl_sol_ctrl_temp,
                                    (ct > 75) ? C_RED : (ct > 55) ? C_YELLOW
                                                                  : C_WHITE,
                                    LV_PART_MAIN);
    }

    /* ---- MC Status1 — RPM + motor current (MC_Status1) ----------------- */
    if (f & DFLAG_MC_STATUS1)
    {
        snprintf(buf, sizeof(buf), "%d", d->mc_status1.speed_rpm);
        lv_label_set_text(lbl_sol_rpm, buf);

        int ma10 = (int)(d->mc_status1.motor_current_a * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d A", ma10 / 10, ma10 % 10);
        lv_label_set_text(lbl_sol_motor_a, buf);
    }

    /* ---- MPPT power (MPPT1-4) ------------------------------------------ */
    if (f & (DFLAG_MPPT1 | DFLAG_MPPT2 | DFLAG_MPPT3 | DFLAG_MPPT4))
    {
        float total = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            float pw = d->mppt[i].input_power_w;
            if (pw < 0.0f)
                pw = 0.0f;
            total += pw;
            lv_bar_set_value(bar_mppt[i], (int32_t)pw, LV_ANIM_OFF);
            snprintf(buf, sizeof(buf), "%d W", (int)pw);
            lv_label_set_text(lbl_mppt_w[i], buf);
        }
        s_solar_total_w = total;

        snprintf(buf, sizeof(buf), "%d W", (int)total);
        lv_label_set_text(lbl_solar_total, buf);

        /* Net power = solar - pack discharge */
        float motor_w = s_pack_power_kw * 1000.0f; /* positive = draining pack */
        float net_w = total - motor_w;
        int sol_int = (int)total;
        int mot_int = (int)motor_w;
        int net_int = (int)net_w;
        if (net_w >= 0)
            snprintf(buf, sizeof(buf), "Solar %dW\nMotor %dW\n= +%dW", sol_int, mot_int, net_int);
        else
            snprintf(buf, sizeof(buf), "Solar %dW\nMotor %dW\n= %dW", sol_int, mot_int, net_int);
        lv_label_set_text(lbl_net_power, buf);
        lv_obj_set_style_text_color(lbl_net_power,
                                    (net_w >= 0.0f) ? C_GREEN : C_RED, LV_PART_MAIN);
    }

    /* ---- GPS position (Altair_GPS_Pos) --------------------------------- */
    if (f & DFLAG_GPS_POS)
    {
        double lat = d->gps_pos.latitude_deg;
        double lon = d->gps_pos.longitude_deg;
        char dir_lat = (lat >= 0) ? 'N' : 'S';
        char dir_lon = (lon >= 0) ? 'E' : 'W';
        if (lat < 0)
            lat = -lat;
        if (lon < 0)
            lon = -lon;
        int lat_d = (int)lat, lat_f = (int)((lat - lat_d) * 10000.0);
        int lon_d = (int)lon, lon_f = (int)((lon - lon_d) * 10000.0);
        snprintf(buf, sizeof(buf), "%d.%04d\xc2\xb0 %c", lat_d, lat_f, dir_lat);
        lv_label_set_text(lbl_gps_lat, buf);
        snprintf(buf, sizeof(buf), "%d.%04d\xc2\xb0 %c", lon_d, lon_f, dir_lon);
        lv_label_set_text(lbl_gps_lon, buf);
    }

    /* ---- GPS navigation (Altair_GPS_Nav) ------------------------------- */
    if (f & DFLAG_GPS_NAV)
    {
        int spd10 = (int)(d->gps_nav.speed_kmh * 0.621371f * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d mph", spd10 / 10, spd10 % 10);
        lv_label_set_text(lbl_gps_speed, buf);

        int hdg = (int)d->gps_nav.heading_deg;
        snprintf(buf, sizeof(buf), "%d\xc2\xb0 %s", hdg,
                 heading_cardinal(d->gps_nav.heading_deg));
        lv_label_set_text(lbl_gps_head, buf);

        int alt10 = (int)(d->gps_nav.altitude_m * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d m MSL", alt10 / 10, alt10 % 10);
        lv_label_set_text(lbl_gps_alt, buf);

        static const char *fix_str[] = {"No fix", "No fix", "2D fix", "3D fix", "GNSS+DR", "Time"};
        uint8_t ft = d->gps_nav.fix_type;
        if (ft > 5)
            ft = 1;
        snprintf(buf, sizeof(buf), "%d  %s", d->gps_nav.num_sats, fix_str[ft]);
        lv_label_set_text(lbl_gps_sats, buf);
    }

    /* ---- GPS time (Altair_GPS_Time) ------------------------------------ */
    if (f & DFLAG_GPS_TIME)
    {
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                 d->gps_time.hour, d->gps_time.minute, d->gps_time.second);
        lv_label_set_text(lbl_gps_utc, buf);
    }

    /* ---- BMS fault (R_BMS_Fault) — cell balancing ---------------------- */
    if (f & DFLAG_BMS_FAULT)
    {
        if (d->bms_fault.cell_balancing_active)
        {
            dot_set(dot_bal_active, C_GREEN);
            lv_label_set_text(lbl_bal_text, "CELL BALANCING: ACTIVE");
            lv_obj_set_style_text_color(lbl_bal_text, C_GREEN, LV_PART_MAIN);
        }
        else
        {
            dot_set(dot_bal_active, C_DGRAY);
            lv_label_set_text(lbl_bal_text, "CELL BALANCING: OFF");
            lv_obj_set_style_text_color(lbl_bal_text, C_GRAY, LV_PART_MAIN);
        }
    }

    /* ---- BMS energy (R_BMS_Energy) ------------------------------------- */
    if (f & DFLAG_BMS_ENERGY)
    {
        int pah10 = (int)(d->bms_energy.pack_ah * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d Ah", pah10 / 10, pah10 % 10);
        lv_label_set_text(lbl_pack_ah, buf);

        int cap10 = (int)(d->bms_energy.total_cap_ah * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d Ah", cap10 / 10, cap10 % 10);
        lv_label_set_text(lbl_total_cap, buf);

        int dod10 = (int)(d->bms_energy.dod_pct * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d%%", dod10 / 10, dod10 % 10);
        lv_label_set_text(lbl_dod, buf);
    }

    /* ---- BMS adaptive energy (R_BMS_AdaptEnergy) ----------------------- */
    if (f & DFLAG_BMS_ADAPT_ENERGY)
    {
        int soc10 = (int)(d->bms_adapt.adapt_soc_pct * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d%%", soc10 / 10, soc10 % 10);
        lv_label_set_text(lbl_adapt_soc, buf);

        int aah10 = (int)(d->bms_adapt.adapt_ah * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d Ah", aah10 / 10, aah10 % 10);
        lv_label_set_text(lbl_adapt_ah, buf);

        int cap10 = (int)(d->bms_adapt.adapt_cap_ah * 10.0f);
        snprintf(buf, sizeof(buf), "%d.%d Ah", cap10 / 10, cap10 % 10);
        lv_label_set_text(lbl_adapt_cap, buf);
    }

    /* ---- BPS safety state — dots + fault overlay ----------------------- */
    if (f & DFLAG_BPS_SAFETY)
    {
        const display_bps_safety_t *s = &d->bps_safety;

        /* Row 1 system dots */
        dot_set(dot_fault, (s->fault_code != BPS_FAULT_NONE) ? C_RED : C_DGRAY);
        dot_set(dot_warn, s->cockpit_warning_yellow ? C_YELLOW : C_DGRAY);
        dot_set(dot_dcdc_sys, s->cockpit_dcdc_on ? C_GREEN : C_DGRAY);
        dot_set(dot_batt_sys, s->cockpit_batt_on ? C_GREEN : C_DGRAY);
        dot_set(dot_mc_sys, (s->relay_mc || s->relay_mc_precharge) ? C_GREEN : C_DGRAY);

        /* Contactor relay dots */
        dot_set(dot_hv_pos, s->relay_hv_pos ? C_GREEN : C_DGRAY);
        dot_set(dot_hv_neg, s->relay_hv_neg ? C_GREEN : C_DGRAY);
        dot_set(dot_mc_relay, s->relay_mc ? C_GREEN : C_DGRAY);
        dot_set(dot_wc_relay, s->relay_wc ? C_GREEN : C_DGRAY);
        dot_set(dot_dcdc_relay, s->relay_dcdc ? C_GREEN : C_DGRAY);

        /* Precharge / HV Ready */
        dot_set(dot_prech, s->precharge_active ? C_AMBER : C_DGRAY);
        dot_set(dot_hv_rdy, s->hv_ready ? C_GREEN : C_DGRAY);

        /* Fault overlay */
        bool fault_active = (s->fault_code != BPS_FAULT_NONE);
        bool limp_active = (s->safety_state == SAFETY_LIMPHOME);

        if (fault_active)
        {
            uint8_t fc = (uint8_t)s->fault_code;
            const char *name = (fc < 14) ? BPS_FAULT_NAMES[fc] : "UNKNOWN FAULT";
            const char *sub = (fc < 14) ? BPS_FAULT_SUBTITLES[fc] : "";
            lv_label_set_text(lbl_fault_name, name);
            snprintf(buf, sizeof(buf), "CODE %d - %s", fc, sub);
            lv_label_set_text(lbl_fault_sub, buf);

            if (limp_active)
            {
                snprintf(buf, sizeof(buf), "%d", s->grace_timer_s);
                lv_label_set_text(lbl_fault_timer, buf);
                snprintf(buf, sizeof(buf), "%d", (int)s->derating_pct);
                lv_label_set_text(lbl_fault_derate, buf);
                snprintf(buf, sizeof(buf), "THROTTLE LIMITED TO %d / 255", d->bps_power.max_throttle);
                lv_label_set_text(lbl_fault_thr, buf);
                lv_obj_clear_flag(fault_limp_cont, LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(fault_limp_cont, LV_OBJ_FLAG_HIDDEN);
            }

            lv_obj_clear_flag(fault_overlay, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(warn_strip, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(fault_overlay, LV_OBJ_FLAG_HIDDEN);

            if (s->cockpit_warning_yellow)
                lv_obj_clear_flag(warn_strip, LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(warn_strip, LV_OBJ_FLAG_HIDDEN);
        }
    }

    lvgl_port_unlock();
}
