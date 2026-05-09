/**
 * ============================================================================
 * display_test_broadcaster.c — Simulated CAN Data for Display Testing
 * MSU Solar Racing Team #13 — FSGP/ASC 2026
 * ============================================================================
 *
 * Broadcasts fake CAN frames over TCP to the driver display for testing
 * the decode → render pipeline without any vehicle hardware.
 *
 * Usage:
 *   1. Flash this onto any ESP32 on the same WiFi as the display ESP32-P4
 *   2. Set DISPLAY_IP to the display's IP address
 *   3. The broadcaster connects and sends frames at realistic rates
 *   4. It cycles through scenarios every 30 seconds:
 *      - Scenario 0: Normal cruising (47 km/h, 72% SOC, moderate solar)
 *      - Scenario 1: Acceleration (speed ramps up, high throttle)
 *      - Scenario 2: Regen braking (speed drops, regen active, current negative)
 *      - Scenario 3: Warning state (derating, yellow light, low SOC)
 *      - Scenario 4: Fault (precharge fault, grace timer countdown, red overlay)
 *      - Scenario 5: Charging (parked, charger connected, current negative)
 *
 * TCP frame format (13 bytes per frame):
 *   [0-3]  uint32_t  CAN ID (little-endian)
 *   [4]    uint8_t   DLC
 *   [5-12] uint8_t   data[8]
 *
 * Build: idf.py build (ESP-IDF) or adapt for desktop with POSIX sockets
 * ============================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* --------------------------------------------------------------------------
 * Platform abstraction — change these for desktop vs ESP32
 * -------------------------------------------------------------------------- */
#ifdef ESP_PLATFORM
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
    #include "lwip/sockets.h"
    #define SLEEP_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
    #define LOG(fmt, ...) ESP_LOGI("TEST", fmt, ##__VA_ARGS__)
#else
    /* Desktop (Linux/macOS) */
    #include <unistd.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #define SLEEP_MS(ms) usleep((ms) * 1000)
    #define LOG(fmt, ...) printf("[TEST] " fmt "\n", ##__VA_ARGS__)
#endif

/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */
#define DISPLAY_IP    "192.168.4.1"   /* Display ESP32-P4 IP */
#define DISPLAY_PORT  5000            /* TCP port */
#define SCENARIO_DURATION_S  30       /* Seconds per scenario */
#define NUM_SCENARIOS  6

/* CAN IDs (from display_can_spec.h) */
#define ID_BPS_SAFETY    0x08FF1000
#define ID_BPS_POWER     0x0CFF2000
#define ID_BPS_SUPPL     0x14FF3300
#define ID_BPS_CHARGING  0x14FF3600
#define ID_VEGA_INPUTS   0x0CFF211E
#define ID_VEGA_STATUS   0x18FF101E
#define ID_MC_STATUS1    0x0CF11E05
#define ID_MC_STATUS2    0x0CF11F05
#define ID_CELL_EXTR     0x14FF3200
#define ID_TEMP_EXTR     0x14FF4100
#define ID_MPPT1         0x00007800
#define ID_MPPT2         0x00007900
#define ID_MPPT3         0x00007A00
#define ID_MPPT4         0x00007B00
#define ID_GPS_POS       0x14FF0150
#define ID_GPS_NAV       0x14FF0250
#define ID_GPS_TIME      0x18FF0350

/* --------------------------------------------------------------------------
 * TCP send helper
 * -------------------------------------------------------------------------- */
static int sock_fd = -1;

static int tcp_connect(void) {
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) return -1;
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DISPLAY_PORT),
    };
    inet_pton(AF_INET, DISPLAY_IP, &addr.sin_addr);
    if (connect(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sock_fd); sock_fd = -1; return -1;
    }
    LOG("Connected to display at %s:%d", DISPLAY_IP, DISPLAY_PORT);
    return 0;
}

static void send_frame(uint32_t id, uint8_t dlc, const uint8_t data[8]) {
    if (sock_fd < 0) return;
    uint8_t pkt[13];
    memcpy(&pkt[0], &id, 4);    /* little-endian on ESP32 */
    pkt[4] = dlc;
    memcpy(&pkt[5], data, 8);
    send(sock_fd, pkt, 13, 0);
}

/* --------------------------------------------------------------------------
 * Encode helpers — mirror the byte packing from display_can_spec.h
 * -------------------------------------------------------------------------- */
static uint8_t alive_counter = 0;

static void enc_bps_safety(uint8_t d[8], uint8_t vstate, uint8_t sstate,
                           int hv_rdy, int hazard, uint8_t fault,
                           int red, int yellow, int dcdc, int batt,
                           uint8_t grace, uint8_t relays0, uint8_t relays1,
                           int prech_done, int vcs_mia, int bms_mia, uint8_t derate_pct) {
    d[0] = (vstate & 0x07) | ((sstate & 0x03) << 3) | (hv_rdy << 5) | (hazard << 7);
    d[1] = (fault & 0x0F) | (red << 4) | (yellow << 5) | (dcdc << 6) | (batt << 7);
    d[2] = grace;
    d[3] = relays0;
    d[4] = relays1;
    d[5] = prech_done | (vcs_mia << 1) | (bms_mia << 2) | (((derate_pct / 4) & 0x1F) << 3);
    d[6] = 0;
    d[7] = alive_counter++;
}

static void enc_bps_power(uint8_t d[8], uint8_t max_thr, uint8_t max_reg,
                          float pack_v, float pack_i, float soc, uint8_t reason) {
    uint16_t v = (uint16_t)(pack_v / 0.1f);
    int16_t  i = (int16_t)(pack_i / 0.1f);
    d[0] = max_thr;
    d[1] = max_reg;
    d[2] = v & 0xFF; d[3] = v >> 8;
    d[4] = i & 0xFF; d[5] = (i >> 8) & 0xFF;
    d[6] = (uint8_t)(soc / 0.5f);
    d[7] = reason;
}

static void enc_vega_inputs(uint8_t d[8], uint16_t pedal, uint16_t brake,
                            uint8_t thr_out, uint8_t reg_out,
                            int brk_sw, int thr_sw, uint8_t mode, int regen) {
    d[0] = pedal & 0xFF; d[1] = pedal >> 8;
    d[2] = brake & 0xFF; d[3] = brake >> 8;
    d[4] = thr_out;
    d[5] = reg_out;
    d[6] = brk_sw | (thr_sw << 1) | ((mode & 0x03) << 2) | (regen << 4);
    d[7] = alive_counter++;
}

static void enc_vega_status(uint8_t d[8], float speed_kmh, float odo_km,
                            uint8_t mode, int regen, int sd_log, uint8_t lh_timer) {
    uint16_t s = (uint16_t)(speed_kmh / 0.1f);
    uint32_t o = (uint32_t)(odo_km / 0.1f);
    d[0] = s & 0xFF; d[1] = s >> 8;
    d[2] = o & 0xFF; d[3] = (o >> 8) & 0xFF; d[4] = (o >> 16) & 0xFF;
    d[5] = (mode & 0x03) | (regen << 2) | (sd_log << 3);
    d[6] = lh_timer;
    d[7] = alive_counter++;
}

static void enc_mc_status1(uint8_t d[8], uint16_t rpm, float motor_a,
                           float batt_v, uint16_t err) {
    uint16_t mi = (uint16_t)(motor_a / 0.1f);
    uint16_t bv = (uint16_t)(batt_v / 0.1f);
    d[0] = rpm & 0xFF; d[1] = rpm >> 8;
    d[2] = mi & 0xFF; d[3] = mi >> 8;
    d[4] = bv & 0xFF; d[5] = bv >> 8;
    d[6] = err & 0xFF; d[7] = err >> 8;
}

static void enc_mc_status2(uint8_t d[8], uint8_t thr_fb, int8_t ctrl_c, int8_t motor_c) {
    d[0] = thr_fb;
    d[1] = (uint8_t)(ctrl_c + 40);
    d[2] = (uint8_t)(motor_c + 30);
    d[3] = 0; d[4] = 0x05; d[5] = 0; d[6] = 0; d[7] = 0;
}

static void enc_cells(uint8_t d[8], float hi_v, uint8_t hi_id, float lo_v, uint8_t lo_id) {
    uint16_t hv = (uint16_t)(hi_v / 0.0001f);
    uint16_t lv = (uint16_t)(lo_v / 0.0001f);
    d[0] = hv & 0xFF; d[1] = hv >> 8;
    d[2] = hi_id;
    d[3] = lv & 0xFF; d[4] = lv >> 8;
    d[5] = lo_id;
    d[6] = 0; d[7] = 0; /* CRC placeholder */
}

static void enc_temps(uint8_t d[8], int8_t hi, uint8_t hi_id, int8_t lo, uint8_t lo_id,
                      int8_t avg, int8_t board) {
    d[0] = (uint8_t)hi; d[1] = hi_id;
    d[2] = (uint8_t)lo; d[3] = lo_id;
    d[4] = (uint8_t)avg; d[5] = (uint8_t)board;
    d[6] = 0; d[7] = 0;
}

static void enc_suppl(uint8_t d[8], float v, float a, float soc, uint8_t status, int8_t temp) {
    uint16_t sv = (uint16_t)(v / 0.01f);
    int16_t  si = (int16_t)(a / 0.01f);
    d[0] = sv & 0xFF; d[1] = sv >> 8;
    d[2] = si & 0xFF; d[3] = (si >> 8) & 0xFF;
    d[4] = (uint8_t)(soc / 0.5f); d[5] = status;
    d[6] = (uint8_t)temp; d[7] = alive_counter++;
}

static void enc_mppt(uint8_t d[8], float vin, float iin, float vout, float iout) {
    int16_t iv = (int16_t)(vin / 0.01f);
    int16_t ii = (int16_t)(iin / 0.0005f);
    int16_t ov = (int16_t)(vout / 0.01f);
    int16_t oi = (int16_t)(iout / 0.0005f);
    d[0] = iv & 0xFF; d[1] = (iv >> 8) & 0xFF;
    d[2] = ii & 0xFF; d[3] = (ii >> 8) & 0xFF;
    d[4] = ov & 0xFF; d[5] = (ov >> 8) & 0xFF;
    d[6] = oi & 0xFF; d[7] = (oi >> 8) & 0xFF;
}

static void enc_gps_pos(uint8_t d[8], double lat, double lon) {
    int32_t la = (int32_t)(lat / 1e-7);
    int32_t lo = (int32_t)(lon / 1e-7);
    memcpy(&d[0], &la, 4);
    memcpy(&d[4], &lo, 4);
}

static void enc_gps_nav(uint8_t d[8], float speed_ms, float heading, float alt_m,
                        uint8_t fix, uint8_t sats) {
    uint16_t sp = (uint16_t)(speed_ms / 0.01f);
    uint16_t hd = (uint16_t)(heading / 0.01f);
    int16_t  al = (int16_t)(alt_m / 0.1f);
    d[0] = sp & 0xFF; d[1] = sp >> 8;
    d[2] = hd & 0xFF; d[3] = hd >> 8;
    d[4] = al & 0xFF; d[5] = (al >> 8) & 0xFF;
    d[6] = fix; d[7] = sats;
}

static void enc_gps_time(uint8_t d[8], uint8_t h, uint8_t m, uint8_t s,
                         int fix_ok, float pdop) {
    uint16_t pd = (uint16_t)(pdop / 0.01f);
    d[0] = h; d[1] = m; d[2] = s;
    d[3] = fix_ok ? 0x01 : 0x00;
    d[4] = pd & 0xFF; d[5] = pd >> 8;
    d[6] = 10; d[7] = 5; /* speed/head accuracy */
}

static void enc_charging(uint8_t d[8], int active, int connected,
                         float out_v, float out_a, int8_t temp) {
    d[0] = active | (connected << 1);
    uint16_t cv = (uint16_t)(out_v / 0.1f);
    int16_t  ci = (int16_t)((out_a + 320.0f) / 0.1f);
    d[1] = cv & 0xFF; d[2] = cv >> 8;
    d[3] = ci & 0xFF; d[4] = (ci >> 8) & 0xFF;
    d[5] = (uint8_t)(temp + 40);
    d[6] = 0; d[7] = alive_counter++;
}

/* --------------------------------------------------------------------------
 * Scenario definitions
 * -------------------------------------------------------------------------- */

typedef struct {
    const char *name;
    float speed;        /* mph */
    float throttle_pct; /* 0-1 */
    float regen_pct;    /* 0-1 */
    float pack_v;       /* V */
    float pack_i;       /* A (positive=discharge) */
    float soc;          /* % */
    float hi_cell_v;
    float lo_cell_v;
    int8_t hi_temp;
    int8_t lo_temp;
    float solar_w[4];   /* per MPPT */
    uint8_t fault_code;
    uint8_t safety_state;
    uint8_t grace_timer;
    uint8_t derate_pct;
    int warning;
    int charging;
} scenario_t;

static const scenario_t scenarios[NUM_SCENARIOS] = {
    { /* 0: Normal cruising */
        .name = "Normal Cruising",
        .speed = 47.0f, .throttle_pct = 0.35f, .regen_pct = 0.0f,
        .pack_v = 97.2f, .pack_i = 18.5f, .soc = 72.0f,
        .hi_cell_v = 3.612f, .lo_cell_v = 3.589f,
        .hi_temp = 34, .lo_temp = 28,
        .solar_w = {218, 215, 211, 198},
        .fault_code = 0, .safety_state = 0, .grace_timer = 0, .derate_pct = 0,
        .warning = 0, .charging = 0,
    },
    { /* 1: Acceleration */
        .name = "Acceleration",
        .speed = 72.0f, .throttle_pct = 0.85f, .regen_pct = 0.0f,
        .pack_v = 94.1f, .pack_i = 45.2f, .soc = 68.0f,
        .hi_cell_v = 3.520f, .lo_cell_v = 3.485f,
        .hi_temp = 38, .lo_temp = 30,
        .solar_w = {225, 222, 219, 210},
        .fault_code = 0, .safety_state = 0, .grace_timer = 0, .derate_pct = 0,
        .warning = 0, .charging = 0,
    },
    { /* 2: Regen braking */
        .name = "Regen Braking",
        .speed = 35.0f, .throttle_pct = 0.0f, .regen_pct = 0.65f,
        .pack_v = 99.8f, .pack_i = -8.5f, .soc = 74.0f,
        .hi_cell_v = 3.700f, .lo_cell_v = 3.680f,
        .hi_temp = 36, .lo_temp = 29,
        .solar_w = {190, 188, 185, 180},
        .fault_code = 0, .safety_state = 0, .grace_timer = 0, .derate_pct = 0,
        .warning = 0, .charging = 0,
    },
    { /* 3: Warning / derating */
        .name = "Warning - Low SOC Derating",
        .speed = 30.0f, .throttle_pct = 0.20f, .regen_pct = 0.0f,
        .pack_v = 82.4f, .pack_i = 12.0f, .soc = 15.0f,
        .hi_cell_v = 3.120f, .lo_cell_v = 3.020f,
        .hi_temp = 41, .lo_temp = 33,
        .solar_w = {105, 98, 102, 95},
        .fault_code = 0, .safety_state = 0, .grace_timer = 0, .derate_pct = 0,
        .warning = 1, .charging = 0,
    },
    { /* 4: Fault with grace timer */
        .name = "FAULT - Precharge",
        .speed = 25.0f, .throttle_pct = 0.0f, .regen_pct = 0.0f,
        .pack_v = 96.0f, .pack_i = 2.0f, .soc = 65.0f,
        .hi_cell_v = 3.580f, .lo_cell_v = 3.550f,
        .hi_temp = 35, .lo_temp = 28,
        .solar_w = {200, 195, 190, 185},
        .fault_code = 9, .safety_state = 1, .grace_timer = 90, .derate_pct = 0,
        .warning = 0, .charging = 0,
    },
    { /* 5: Charging (parked) */
        .name = "Charging",
        .speed = 0.0f, .throttle_pct = 0.0f, .regen_pct = 0.0f,
        .pack_v = 108.5f, .pack_i = -12.0f, .soc = 88.0f,
        .hi_cell_v = 4.020f, .lo_cell_v = 4.005f,
        .hi_temp = 32, .lo_temp = 27,
        .solar_w = {0, 0, 0, 0},
        .fault_code = 0, .safety_state = 0, .grace_timer = 0, .derate_pct = 0,
        .warning = 0, .charging = 1,
    },
};

/* --------------------------------------------------------------------------
 * Main broadcast loop
 * -------------------------------------------------------------------------- */

static void broadcast_scenario(const scenario_t *sc, uint32_t elapsed_ms) {
    uint8_t d[8];
    float t = (float)elapsed_ms / 1000.0f;

    /* Add gentle variation to make it feel alive */
    float wobble = sinf(t * 0.5f) * 0.5f;
    float speed = sc->speed + wobble * 2.0f;
    if (speed < 0) speed = 0;

    /* Grace timer countdown for fault scenario */
    uint8_t grace = sc->grace_timer;
    uint8_t derate = sc->derate_pct;
    if (sc->fault_code != 0 && sc->grace_timer > 0) {
        uint32_t elapsed_s = elapsed_ms / 1000;
        grace = (elapsed_s < sc->grace_timer) ? (sc->grace_timer - elapsed_s) : 0;
        derate = (uint8_t)((float)(sc->grace_timer - grace) / (float)sc->grace_timer * 100.0f);
    }

    /* Relays: all closed in PRIMARY, all open in fault after timer expires */
    uint8_t relays0 = 0xFF;  /* all closed */
    uint8_t relays1 = 0x55;  /* MPPT A-D closed, precharges off */
    if (grace == 0 && sc->fault_code != 0) {
        relays0 = 0x00; relays1 = 0x00;
    }

    /* === 100ms messages === */
    /* BPS_SafetyState */
    enc_bps_safety(d,
        sc->speed > 0 ? 4 : 1,     /* PRIMARY if moving, STANDBY if parked */
        sc->safety_state, 1,
        sc->fault_code != 0,        /* hazard */
        sc->fault_code,
        sc->fault_code != 0,        /* red LED */
        sc->warning,                /* yellow LED */
        1, 1,                       /* DCDC on, batt on */
        grace, relays0, relays1,
        1, 0, 0, derate);
    send_frame(ID_BPS_SAFETY, 8, d);

    /* BPS_PowerLimit */
    uint8_t max_thr = 255 - (derate * 255 / 100);
    uint8_t max_reg = (sc->regen_pct > 0) ? 200 : 0;
    uint8_t reason = 0;
    if (sc->warning) reason |= 0x10;  /* LowSOC */
    if (derate > 0) reason |= 0x40;   /* LimpHome */
    if (max_reg == 0) reason |= 0x20; /* RegenDisabled */
    enc_bps_power(d, max_thr, max_reg,
        sc->pack_v + wobble, sc->pack_i + wobble * 0.5f,
        sc->soc, reason);
    send_frame(ID_BPS_POWER, 8, d);

    /* Vega_DriverInputs */
    uint8_t thr_out = (uint8_t)(sc->throttle_pct * (float)max_thr);
    uint8_t reg_out = (uint8_t)(sc->regen_pct * (float)max_reg);
    enc_vega_inputs(d,
        (uint16_t)(sc->throttle_pct * 3600 + 200),
        (uint16_t)(sc->regen_pct > 0 ? sc->regen_pct * 3200 + 500 : 100),
        thr_out, reg_out,
        sc->regen_pct > 0, 1,
        sc->speed > 0 ? 1 : 0,
        sc->regen_pct > 0);
    send_frame(ID_VEGA_INPUTS, 8, d);

    /* Vega_SafetyState (for completeness — display may show this) */

    /* MC_Status1 */
    uint16_t rpm = (uint16_t)(speed / (1.6f * 60.0f / 1000.0f));
    enc_mc_status1(d, rpm, sc->pack_i * 0.8f + wobble,
        sc->pack_v + wobble, 0);
    send_frame(ID_MC_STATUS1, 8, d);

    /* MC_Status2 */
    enc_mc_status2(d, thr_out,
        (int8_t)(sc->hi_temp + 8 + (int)(wobble)),
        (int8_t)(sc->hi_temp + 17 + (int)(wobble)));
    send_frame(ID_MC_STATUS2, 8, d);
}

static void broadcast_slow(const scenario_t *sc, uint32_t elapsed_ms) {
    uint8_t d[8];
    float wobble = sinf((float)elapsed_ms * 0.001f * 0.3f) * 0.3f;

    /* Vega_VehicleStatus (200ms) */
    enc_vega_status(d, sc->speed + wobble * 2,
        127.4f + (float)elapsed_ms * 0.00001f,
        sc->speed > 0 ? 1 : 0,
        sc->regen_pct > 0, 1, 0);
    send_frame(ID_VEGA_STATUS, 8, d);

    /* R_BMS_CellExtremes (200ms) */
    enc_cells(d, sc->hi_cell_v + wobble * 0.002f, 14,
              sc->lo_cell_v + wobble * 0.001f, 3);
    send_frame(ID_CELL_EXTR, 8, d);
}

static void broadcast_500ms(const scenario_t *sc, uint32_t elapsed_ms) {
    uint8_t d[8];
    float wobble = sinf((float)elapsed_ms * 0.001f * 0.2f);

    /* R_BMS_TempExtremes */
    enc_temps(d, sc->hi_temp + (int8_t)wobble, 7,
              sc->lo_temp, 12,
              (sc->hi_temp + sc->lo_temp) / 2,
              sc->lo_temp + 5);
    send_frame(ID_TEMP_EXTR, 8, d);

    /* MPPT 1-4 */
    for (int i = 0; i < 4; i++) {
        float pwr = sc->solar_w[i] + wobble * 5.0f;
        if (pwr < 0) pwr = 0;
        float vin = 38.0f + wobble;
        float iin = pwr / (vin > 0 ? vin : 1.0f);
        float vout = 97.0f;
        float iout = pwr / (vout > 0 ? vout : 1.0f);
        enc_mppt(d, vin, iin, vout, iout);
        uint32_t ids[4] = {ID_MPPT1, ID_MPPT2, ID_MPPT3, ID_MPPT4};
        send_frame(ids[i], 8, d);
    }
}

static void broadcast_1s(const scenario_t *sc, uint32_t elapsed_ms) {
    uint8_t d[8];

    /* BPS_SupplBattery */
    enc_suppl(d, 25.1f, -0.3f, 84.0f, 0, 28);
    send_frame(ID_BPS_SUPPL, 8, d);

    /* BPS_ChargingInfo */
    if (sc->charging) {
        enc_charging(d, 1, 1, sc->pack_v, -12.0f, 35);
    } else {
        enc_charging(d, 0, 0, 0, 0, 25);
    }
    send_frame(ID_BPS_CHARGING, 8, d);

    /* GPS */
    float t = (float)elapsed_ms / 1000.0f;
    double lat = 42.7284 + t * 0.00001;  /* slow drift for visual interest */
    double lon = -84.4823 + t * 0.000015;
    enc_gps_pos(d, lat, lon);
    send_frame(ID_GPS_POS, 8, d);

    float heading = 247.0f + sinf(t * 0.1f) * 5.0f;
    enc_gps_nav(d, sc->speed / 3.6f, heading, 312.0f, 3, 12);
    send_frame(ID_GPS_NAV, 8, d);

    uint32_t sec = (uint32_t)t;
    enc_gps_time(d, 14, 32 + (sec / 60) % 60, sec % 60, 1, 1.2f);
    send_frame(ID_GPS_TIME, 8, d);
}

/* --------------------------------------------------------------------------
 * Entry point
 * -------------------------------------------------------------------------- */

#ifdef ESP_PLATFORM
void app_main(void) {
    /* TODO: WiFi init here */
#else
int main(void) {
#endif
    LOG("Lysander Display Test Broadcaster");
    LOG("Connecting to %s:%d ...", DISPLAY_IP, DISPLAY_PORT);

    while (tcp_connect() < 0) {
        LOG("Connection failed, retrying in 2s...");
        SLEEP_MS(2000);
    }

    int scenario_idx = 0;
    uint32_t scenario_start = 0;
    uint32_t tick = 0;
    uint32_t last_100ms = 0;
    uint32_t last_200ms = 0;
    uint32_t last_500ms = 0;
    uint32_t last_1s    = 0;

    LOG("Starting scenario: %s", scenarios[0].name);

    while (1) {
        uint32_t elapsed = tick - scenario_start;
        const scenario_t *sc = &scenarios[scenario_idx];

        /* Cycle scenarios */
        if (elapsed >= SCENARIO_DURATION_S * 1000) {
            scenario_idx = (scenario_idx + 1) % NUM_SCENARIOS;
            scenario_start = tick;
            elapsed = 0;
            sc = &scenarios[scenario_idx];
            LOG("Scenario %d: %s", scenario_idx, sc->name);
        }

        /* 100ms messages (BPS_Safety, BPS_Power, Vega_Inputs, MC) */
        if (tick - last_100ms >= 100) {
            broadcast_scenario(sc, elapsed);
            last_100ms = tick;
        }

        /* 200ms messages (Vega_Status, CellExtremes) */
        if (tick - last_200ms >= 200) {
            broadcast_slow(sc, elapsed);
            last_200ms = tick;
        }

        /* 500ms messages (TempExtremes, MPPTs) */
        if (tick - last_500ms >= 500) {
            broadcast_500ms(sc, elapsed);
            last_500ms = tick;
        }

        /* 1s messages (Suppl, Charging, GPS) */
        if (tick - last_1s >= 1000) {
            broadcast_1s(sc, elapsed);
            last_1s = tick;
        }

        SLEEP_MS(10);
        tick += 10;
    }

#ifndef ESP_PLATFORM
    return 0;
#endif
}
