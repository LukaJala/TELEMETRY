/**
 * ============================================================================
 * display_can_spec.h — CAN Messages for Lysander Driver Display
 * MSU Solar Racing Team #13 — FSGP/ASC 2026
 * ============================================================================
 *
 * This file contains ONLY the messages and decode functions needed by the
 * driver display. The display receives these as raw CAN frames from Vega
 * over TCP. Each struct has a decode function that unpacks the 8-byte
 * CAN payload into named fields with physical units.
 *
 * All messages use 29-bit extended CAN IDs, little-endian byte order.
 *
 * CAN database version: v7
 * Generated for: ESP32-P4 + 10.1" DSI Touch (1280x800 landscape)
 */

#ifndef DISPLAY_CAN_SPEC_H
#define DISPLAY_CAN_SPEC_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * CAN IDs — use these to route incoming frames to the correct decoder
 * ========================================================================= */

/* BPS (SA=0x00) on CAN_VEHICLE */
#define CAN_ID_BPS_SAFETY_STATE    0x08FF1000
#define CAN_ID_BPS_POWER_LIMIT     0x0CFF2000
#define CAN_ID_BPS_SUPPL_BATTERY   0x14FF3300
#define CAN_ID_BPS_CHARGING_INFO   0x14FF3600

/* Vega (SA=0x1E) on CAN_VEHICLE */
#define CAN_ID_VEGA_DRIVER_INPUTS  0x0CFF211E
#define CAN_ID_VEGA_VEHICLE_STATUS 0x18FF101E

/* Kelly MC (predefined) on CAN_VEHICLE */
#define CAN_ID_MC_STATUS1          0x0CF11E05
#define CAN_ID_MC_STATUS2          0x0CF11F05

/* Relayed BMS via BPS on CAN_VEHICLE */
#define CAN_ID_R_BMS_CELL_EXTREMES 0x14FF3200
#define CAN_ID_R_BMS_TEMP_EXTREMES 0x14FF4100

/* MPPT Open-SEC (DevID=120-123, Pkt=0) on CAN_VEHICLE */
#define CAN_ID_MPPT1_POWER_MEAS    0x00007800
#define CAN_ID_MPPT2_POWER_MEAS    0x00007900
#define CAN_ID_MPPT3_POWER_MEAS    0x00007A00
#define CAN_ID_MPPT4_POWER_MEAS    0x00007B00

/* Altair (SA=0x50) on CAN_BODY (relayed to display via Vega TCP) */
#define CAN_ID_ALTAIR_GPS_POS      0x14FF0150
#define CAN_ID_ALTAIR_GPS_NAV      0x14FF0250
#define CAN_ID_ALTAIR_GPS_TIME     0x18FF0350

/* =========================================================================
 * Enums
 * ========================================================================= */

typedef enum {
    VSTATE_OFF       = 0,
    VSTATE_STANDBY   = 1,
    VSTATE_ACCESSORY = 2,
    VSTATE_PRIMARY   = 4,
} vehicle_state_t;

typedef enum {
    SAFETY_NORMAL   = 0,
    SAFETY_LIMPHOME = 1,
    SAFETY_FAILSAFE = 2,
} safety_state_t;

typedef enum {
    BPS_FAULT_NONE               = 0,
    BPS_FAULT_SUPPL_UV           = 1,
    BPS_FAULT_SUPPL_OV           = 2,
    BPS_FAULT_SUPPL_UT           = 3,
    BPS_FAULT_SUPPL_OT           = 4,
    BPS_FAULT_DCDC_UV            = 5,
    BPS_FAULT_DCDC_OV            = 6,
    BPS_FAULT_OVERCURRENT        = 7,
    BPS_FAULT_CURRENT_SENSOR     = 8,
    BPS_FAULT_PRECHARGE          = 9,
    BPS_FAULT_MAIN_PACK_UV       = 10,
    BPS_FAULT_MAIN_PACK_OV       = 11,
    BPS_FAULT_CAN                = 12,
    BPS_FAULT_CAN_TIMEOUT        = 13,
} bps_fault_t;

/* Human-readable fault names for display */
static const char *BPS_FAULT_NAMES[] = {
    "NO FAULT",
    "SUPPL UNDERVOLTAGE",
    "SUPPL OVERVOLTAGE",
    "SUPPL UNDERTEMP",
    "SUPPL OVERTEMP",
    "DCDC UNDERVOLTAGE",
    "DCDC OVERVOLTAGE",
    "BPS OVERCURRENT",
    "CURRENT SENSOR FAIL",
    "PRECHARGE FAULT",
    "MAIN PACK UNDERVOLT",
    "MAIN PACK OVERVOLT",
    "CAN BUS FAULT",
    "CAN TIMEOUT",
};

/* =========================================================================
 * Message Structs + Decode Functions
 * ========================================================================= */

/* ---- BPS_SafetyState (8B, 100ms) ----------------------------------------
 * Primary safety message. Contains vehicle state, fault info, cockpit LED
 * states, relay/contactor status, grace timer, and derating percentage.
 * This is the message that triggers the fault overlay on the display.
 *
 * Byte 0: [2:0]=VehicleState [4:3]=SafetyState [5]=HV_Ready
 *         [6]=PrechargeActive [7]=HazardActivate
 * Byte 1: [3:0]=FaultCode [4]=CockpitFaultRed [5]=CockpitWarningYellow
 *         [6]=CockpitDCDC_On [7]=CockpitBattOn
 * Byte 2: GraceTimer (seconds, 90→0 during LimpHome)
 * Byte 3: Relays_B0
 *         [0]=HV+ [1]=HV- [2]=MC [3]=MC_Pre [4]=WC
 *         [5]=DCDC_Pre [6]=DCDC [7]=DCDC_On
 * Byte 4: Relays_B1
 *         [0]=MPPTA [1]=MPPTA_Pre [2]=MPPTB [3]=MPPTB_Pre
 *         [4]=MPPTC [5]=MPPTC_Pre [6]=MPPTD [7]=MPPTD_Pre
 * Byte 5: [0]=PrechargeComplete [1]=VCS_MIA [2]=BMS_MIA [7:3]=DeratingPct/4
 * Byte 6: Reserved
 * Byte 7: StateAlive (rolling counter)
 * ----------------------------------------------------------------------- */
typedef struct {
    vehicle_state_t vehicle_state;
    safety_state_t  safety_state;
    bool            hv_ready;
    bool            precharge_active;
    bool            hazard_activate;

    bps_fault_t     fault_code;
    bool            cockpit_fault_red;
    bool            cockpit_warning_yellow;
    bool            cockpit_dcdc_on;
    bool            cockpit_batt_on;

    uint8_t         grace_timer_s;

    /* Relay states (byte 3) */
    bool            relay_hv_pos;
    bool            relay_hv_neg;
    bool            relay_mc;
    bool            relay_mc_precharge;
    bool            relay_wc;
    bool            relay_dcdc_precharge;
    bool            relay_dcdc;
    bool            dcdc_on;

    /* MPPT relays (byte 4) */
    bool            relay_mppt_a;
    bool            relay_mppt_b;
    bool            relay_mppt_c;
    bool            relay_mppt_d;

    bool            precharge_complete;
    bool            vcs_mia_detected;
    bool            bms_mia_detected;
    uint8_t         derating_pct;       /* 0-100, 4% steps */

    uint8_t         alive;
} display_bps_safety_t;

static inline void display_decode_bps_safety(display_bps_safety_t *out, const uint8_t d[8]) {
    out->vehicle_state        = (vehicle_state_t)(d[0] & 0x07);
    out->safety_state         = (safety_state_t)((d[0] >> 3) & 0x03);
    out->hv_ready             = (d[0] >> 5) & 1;
    out->precharge_active     = (d[0] >> 6) & 1;
    out->hazard_activate      = (d[0] >> 7) & 1;

    out->fault_code           = (bps_fault_t)(d[1] & 0x0F);
    out->cockpit_fault_red    = (d[1] >> 4) & 1;
    out->cockpit_warning_yellow = (d[1] >> 5) & 1;
    out->cockpit_dcdc_on      = (d[1] >> 6) & 1;
    out->cockpit_batt_on      = (d[1] >> 7) & 1;

    out->grace_timer_s        = d[2];

    out->relay_hv_pos         = d[3] & 1;
    out->relay_hv_neg         = (d[3] >> 1) & 1;
    out->relay_mc             = (d[3] >> 2) & 1;
    out->relay_mc_precharge   = (d[3] >> 3) & 1;
    out->relay_wc             = (d[3] >> 4) & 1;
    out->relay_dcdc_precharge = (d[3] >> 5) & 1;
    out->relay_dcdc           = (d[3] >> 6) & 1;
    out->dcdc_on              = (d[3] >> 7) & 1;

    out->relay_mppt_a         = d[4] & 1;
    out->relay_mppt_b         = (d[4] >> 2) & 1;
    out->relay_mppt_c         = (d[4] >> 4) & 1;
    out->relay_mppt_d         = (d[4] >> 6) & 1;

    out->precharge_complete   = d[5] & 1;
    out->vcs_mia_detected     = (d[5] >> 1) & 1;
    out->bms_mia_detected     = (d[5] >> 2) & 1;
    out->derating_pct         = ((d[5] >> 3) & 0x1F) * 4;

    out->alive                = d[7];
}

/* ---- BPS_PowerLimit (8B, 100ms) -----------------------------------------
 * Throttle/regen limits and pack electrical data.
 *
 * Byte 0: MaxThrottle (0-255)
 * Byte 1: MaxRegen (0-255, 0=regen disabled)
 * Byte 2-3: PackVoltage (U16, factor 0.1V)
 * Byte 4-5: PackCurrent (S16, factor 0.1A, +=discharge -=charge)
 * Byte 6: SOC (U8, factor 0.5%)
 * Byte 7: LimitReason bitmap
 *         b0=OverI b1=OverV b2=UnderV b3=OverTemp b4=LowSOC
 *         b5=RegenDisabled b6=LimpHomeActive
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t  max_throttle;
    uint8_t  max_regen;
    float    pack_voltage_v;
    float    pack_current_a;   /* positive=discharge, negative=charge/regen */
    float    soc_pct;
    uint8_t  limit_reason;
    float    pack_power_kw;    /* derived: V * I / 1000 */
} display_bps_power_t;

static inline void display_decode_bps_power(display_bps_power_t *out, const uint8_t d[8]) {
    out->max_throttle   = d[0];
    out->max_regen      = d[1];
    out->pack_voltage_v = (float)(d[2] | (d[3] << 8)) * 0.1f;
    out->pack_current_a = (float)(int16_t)(d[4] | (d[5] << 8)) * 0.1f;
    out->soc_pct        = (float)d[6] * 0.5f;
    out->limit_reason   = d[7];
    out->pack_power_kw  = out->pack_voltage_v * out->pack_current_a / 1000.0f;
}

/* ---- Vega_DriverInputs (8B, 20ms) ---------------------------------------
 * Raw pedal/brake ADC + computed throttle/regen output.
 *
 * Byte 0-1: PedalRaw (U16, 0-4095)
 * Byte 2-3: BrakeRaw (U16, 0-4095)
 * Byte 4: ThrottleOut (0-255, what Kelly actually gets)
 * Byte 5: RegenOut (0-255, what Kelly Brake_AD gets)
 * Byte 6: Switches
 *         [0]=BrakeSW [1]=ThrottleSW(deadman) [3:2]=DriveMode [4]=RegenActive
 * Byte 7: InputAlive
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t pedal_raw;
    uint16_t brake_raw;
    uint8_t  throttle_out;
    uint8_t  regen_out;
    bool     brake_switch;
    bool     throttle_switch;
    uint8_t  drive_mode;      /* 0=Park 1=FWD 2=REV */
    bool     regen_active;
    uint8_t  alive;
} display_vega_inputs_t;

static inline void display_decode_vega_inputs(display_vega_inputs_t *out, const uint8_t d[8]) {
    out->pedal_raw       = d[0] | (d[1] << 8);
    out->brake_raw       = d[2] | (d[3] << 8);
    out->throttle_out    = d[4];
    out->regen_out       = d[5];
    out->brake_switch    = d[6] & 1;
    out->throttle_switch = (d[6] >> 1) & 1;
    out->drive_mode      = (d[6] >> 2) & 0x03;
    out->regen_active    = (d[6] >> 4) & 1;
    out->alive           = d[7];
}

/* ---- Vega_VehicleStatus (8B, 200ms) -------------------------------------
 * Byte 0-1: Speed (U16, factor 0.1 km/h)
 * Byte 2-4: Odometer (U24, factor 0.1 km)
 * Byte 5: Flags [1:0]=DriveMode [2]=RegenActive [3]=SD_Logging [4]=Tele
 * Byte 6: LHTimer (seconds)
 * Byte 7: VehAlive
 * ----------------------------------------------------------------------- */
typedef struct {
    float    speed_kmh;
    float    odometer_km;
    uint8_t  drive_mode;
    bool     regen_active;
    bool     sd_logging;
    uint8_t  lh_timer_s;
    uint8_t  alive;
} display_vega_status_t;

static inline void display_decode_vega_status(display_vega_status_t *out, const uint8_t d[8]) {
    out->speed_kmh    = (float)(d[0] | (d[1] << 8)) * 0.1f;
    out->odometer_km  = (float)(d[2] | (d[3] << 8) | (d[4] << 16)) * 0.1f;
    out->drive_mode   = d[5] & 0x03;
    out->regen_active = (d[5] >> 2) & 1;
    out->sd_logging   = (d[5] >> 3) & 1;
    out->lh_timer_s   = d[6];
    out->alive        = d[7];
}

/* ---- MC_Status1 (Kelly Msg1, 8B, 50ms) ----------------------------------
 * Byte 0-1: Speed (U16, RPM)
 * Byte 2-3: MotorCurrent (U16, factor 0.1A)
 * Byte 4-5: BattVoltage (U16, factor 0.1V)
 * Byte 6-7: ErrorCode (U16, bitfield)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t speed_rpm;
    float    motor_current_a;
    float    batt_voltage_v;
    uint16_t error_code;
} display_mc_status1_t;

static inline void display_decode_mc_status1(display_mc_status1_t *out, const uint8_t d[8]) {
    out->speed_rpm       = d[0] | (d[1] << 8);
    out->motor_current_a = (float)(d[2] | (d[3] << 8)) * 0.1f;
    out->batt_voltage_v  = (float)(d[4] | (d[5] << 8)) * 0.1f;
    out->error_code      = d[6] | (d[7] << 8);
}

/* ---- MC_Status2 (Kelly Msg2, 8B, 50ms) ----------------------------------
 * Byte 0: ThrottleFB (0-255)
 * Byte 1: CtrlTemp (raw - 40 = °C)
 * Byte 2: MotorTemp (raw - 30 = °C)
 * Byte 3: Reserved
 * Byte 4: DirStatus [1:0]=FBDir [3:2]=CmdDir
 * Byte 5: Switches (hall sensors, brake, fwd, rev, etc.)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t throttle_fb;
    int8_t  ctrl_temp_c;
    int8_t  motor_temp_c;
} display_mc_status2_t;

static inline void display_decode_mc_status2(display_mc_status2_t *out, const uint8_t d[8]) {
    out->throttle_fb  = d[0];
    out->ctrl_temp_c  = (int8_t)d[1] - 40;
    out->motor_temp_c = (int8_t)d[2] - 30;
}

/* ---- R_BMS_CellExtremes (8B, 200ms) ------------------------------------
 * Byte 0-1: HiCellV (U16, factor 0.0001V = 0.1mV resolution)
 * Byte 2: HiCellVID (cell number 0-27)
 * Byte 3-4: LoCellV (U16, factor 0.0001V)
 * Byte 5: LoCellVID
 * Byte 6-7: CRC16 (not checked by display, BPS already validated)
 * ----------------------------------------------------------------------- */
typedef struct {
    float   hi_cell_v;
    uint8_t hi_cell_id;
    float   lo_cell_v;
    uint8_t lo_cell_id;
    float   spread_mv;   /* derived: (hi - lo) * 1000 */
} display_cell_extremes_t;

static inline void display_decode_cell_extremes(display_cell_extremes_t *out, const uint8_t d[8]) {
    out->hi_cell_v  = (float)(d[0] | (d[1] << 8)) * 0.0001f;
    out->hi_cell_id = d[2];
    out->lo_cell_v  = (float)(d[3] | (d[4] << 8)) * 0.0001f;
    out->lo_cell_id = d[5];
    out->spread_mv  = (out->hi_cell_v - out->lo_cell_v) * 1000.0f;
}

/* ---- R_BMS_TempExtremes (8B, 500ms) ------------------------------------
 * Byte 0: HiTemp (S8, °C)
 * Byte 1: HiTempID (thermistor 0-18)
 * Byte 2: LoTemp (S8, °C)
 * Byte 3: LoTempID
 * Byte 4: AvgTemp (S8, °C)
 * Byte 5: IntTemp (S8, °C, BMS board)
 * Byte 6-7: CRC16
 * ----------------------------------------------------------------------- */
typedef struct {
    int8_t  hi_temp_c;
    uint8_t hi_temp_id;
    int8_t  lo_temp_c;
    uint8_t lo_temp_id;
    int8_t  avg_temp_c;
    int8_t  int_temp_c;
} display_temp_extremes_t;

static inline void display_decode_temp_extremes(display_temp_extremes_t *out, const uint8_t d[8]) {
    out->hi_temp_c  = (int8_t)d[0];
    out->hi_temp_id = d[1];
    out->lo_temp_c  = (int8_t)d[2];
    out->lo_temp_id = d[3];
    out->avg_temp_c = (int8_t)d[4];
    out->int_temp_c = (int8_t)d[5];
}

/* ---- BPS_SupplBattery (8B, 1000ms) --------------------------------------
 * Byte 0-1: SupplV (U16, factor 0.01V)
 * Byte 2-3: SupplI (S16, factor 0.01A)
 * Byte 4: SupplSOC (U8, factor 0.5%)
 * Byte 5: Status (0=OK 1=UV 2=OV 3=OT 4=Disconnected)
 * Byte 6: SupplTemp (S8, °C)
 * Byte 7: Alive
 * ----------------------------------------------------------------------- */
typedef struct {
    float   voltage_v;
    float   current_a;
    float   soc_pct;
    uint8_t status;
    int8_t  temp_c;
} display_suppl_batt_t;

static inline void display_decode_suppl_batt(display_suppl_batt_t *out, const uint8_t d[8]) {
    out->voltage_v = (float)(d[0] | (d[1] << 8)) * 0.01f;
    out->current_a = (float)(int16_t)(d[2] | (d[3] << 8)) * 0.01f;
    out->soc_pct   = (float)d[4] * 0.5f;
    out->status    = d[5];
    out->temp_c    = (int8_t)d[6];
}

/* ---- MPPT PowerMeas (Open-SEC Pkt0, 8B, 500ms, x4 units) ---------------
 * Byte 0-1: InputV (S16, factor 0.01V)
 * Byte 2-3: InputI (S16, factor 0.0005A)
 * Byte 4-5: OutputV (S16, factor 0.01V)
 * Byte 6-7: OutputI (S16, factor 0.0005A)
 * ----------------------------------------------------------------------- */
typedef struct {
    float input_v;
    float input_a;
    float output_v;
    float output_a;
    float input_power_w;    /* derived */
    float output_power_w;   /* derived */
} display_mppt_power_t;

static inline void display_decode_mppt_power(display_mppt_power_t *out, const uint8_t d[8]) {
    out->input_v       = (float)(int16_t)(d[0] | (d[1] << 8)) * 0.01f;
    out->input_a       = (float)(int16_t)(d[2] | (d[3] << 8)) * 0.0005f;
    out->output_v      = (float)(int16_t)(d[4] | (d[5] << 8)) * 0.01f;
    out->output_a      = (float)(int16_t)(d[6] | (d[7] << 8)) * 0.0005f;
    out->input_power_w = out->input_v * out->input_a;
    out->output_power_w = out->output_v * out->output_a;
}

/* ---- Altair_GPS_Pos (8B, 200ms) -----------------------------------------
 * Byte 0-3: Latitude (S32, factor 1e-7 degrees)
 * Byte 4-7: Longitude (S32, factor 1e-7 degrees)
 * ----------------------------------------------------------------------- */
typedef struct {
    double latitude_deg;
    double longitude_deg;
} display_gps_pos_t;

static inline void display_decode_gps_pos(display_gps_pos_t *out, const uint8_t d[8]) {
    int32_t lat = (int32_t)(d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24));
    int32_t lon = (int32_t)(d[4] | (d[5] << 8) | (d[6] << 16) | (d[7] << 24));
    out->latitude_deg  = (double)lat * 1e-7;
    out->longitude_deg = (double)lon * 1e-7;
}

/* ---- Altair_GPS_Nav (8B, 200ms) -----------------------------------------
 * Byte 0-1: GroundSpeed (U16, factor 0.01 m/s)
 * Byte 2-3: Heading (U16, factor 0.01 degrees)
 * Byte 4-5: AltMSL (S16, factor 0.1 m)
 * Byte 6: FixType (0=No 2=2D 3=3D 4=GNSS+DR 5=Time)
 * Byte 7: NumSats
 * ----------------------------------------------------------------------- */
typedef struct {
    float   speed_ms;
    float   speed_kmh;    /* derived */
    float   heading_deg;
    float   altitude_m;
    uint8_t fix_type;
    uint8_t num_sats;
} display_gps_nav_t;

static inline void display_decode_gps_nav(display_gps_nav_t *out, const uint8_t d[8]) {
    out->speed_ms    = (float)(d[0] | (d[1] << 8)) * 0.01f;
    out->speed_kmh   = out->speed_ms * 3.6f;
    out->heading_deg = (float)(d[2] | (d[3] << 8)) * 0.01f;
    out->altitude_m  = (float)(int16_t)(d[4] | (d[5] << 8)) * 0.1f;
    out->fix_type    = d[6];
    out->num_sats    = d[7];
}

/* ---- Altair_GPS_Time (8B, 1000ms) ---------------------------------------
 * Byte 0: Hour (0-23)
 * Byte 1: Minute (0-59)
 * Byte 2: Second (0-60)
 * Byte 3: FixValid flags [0]=gnssFixOK [1]=diffSoln
 * Byte 4-5: PDOP (U16, factor 0.01)
 * Byte 6: SpeedAccuracy (U8, factor 0.1 m/s)
 * Byte 7: HeadingAccuracy (U8, factor 0.1 deg)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    bool    fix_ok;
    float   pdop;
} display_gps_time_t;

static inline void display_decode_gps_time(display_gps_time_t *out, const uint8_t d[8]) {
    out->hour   = d[0];
    out->minute = d[1];
    out->second = d[2];
    out->fix_ok = d[3] & 1;
    out->pdop   = (float)(d[4] | (d[5] << 8)) * 0.01f;
}

/* ---- BPS_ChargingInfo (8B, 1000ms) --------------------------------------
 * Byte 0: [0]=ChargingActive [1]=ChargerConnected [2]=FaultTemp
 *         [3]=FaultInput [4]=FaultHW [5]=FaultComm
 * Byte 1-2: ChargerOutV (U16, factor 0.1V)
 * Byte 3-4: ChargerOutI (U16, factor 0.1A, offset -3200)
 * Byte 5: ChargerTemp (S8, °C, offset -40)
 * Byte 6: Reserved
 * Byte 7: Alive
 * ----------------------------------------------------------------------- */
typedef struct {
    bool    charging_active;
    bool    charger_connected;
    bool    fault_temp;
    bool    fault_input;
    bool    fault_hw;
    bool    fault_comm;
    float   output_v;
    float   output_a;
    int8_t  charger_temp_c;
} display_charging_t;

static inline void display_decode_charging(display_charging_t *out, const uint8_t d[8]) {
    out->charging_active   = d[0] & 1;
    out->charger_connected = (d[0] >> 1) & 1;
    out->fault_temp        = (d[0] >> 2) & 1;
    out->fault_input       = (d[0] >> 3) & 1;
    out->fault_hw          = (d[0] >> 4) & 1;
    out->fault_comm        = (d[0] >> 5) & 1;
    out->output_v          = (float)(d[1] | (d[2] << 8)) * 0.1f;
    out->output_a          = (float)(int16_t)(d[3] | (d[4] << 8)) * 0.1f + (-3200.0f * 0.1f);
    out->charger_temp_c    = (int8_t)d[5] - 40;
}

/* =========================================================================
 * Frame router — call this for every incoming CAN frame
 * ========================================================================= */

typedef struct {
    display_bps_safety_t     bps_safety;
    display_bps_power_t      bps_power;
    display_vega_inputs_t    vega_inputs;
    display_vega_status_t    vega_status;
    display_mc_status1_t     mc_status1;
    display_mc_status2_t     mc_status2;
    display_cell_extremes_t  cells;
    display_temp_extremes_t  temps;
    display_suppl_batt_t     suppl;
    display_mppt_power_t     mppt[4];
    display_gps_pos_t        gps_pos;
    display_gps_nav_t        gps_nav;
    display_gps_time_t       gps_time;
    display_charging_t       charging;
    uint32_t                 update_flags;  /* bitmask of which structs were updated */
} display_data_t;

/* Update flag bits */
#define DFLAG_BPS_SAFETY   (1u << 0)
#define DFLAG_BPS_POWER    (1u << 1)
#define DFLAG_VEGA_INPUTS  (1u << 2)
#define DFLAG_VEGA_STATUS  (1u << 3)
#define DFLAG_MC_STATUS1   (1u << 4)
#define DFLAG_MC_STATUS2   (1u << 5)
#define DFLAG_CELLS        (1u << 6)
#define DFLAG_TEMPS        (1u << 7)
#define DFLAG_SUPPL        (1u << 8)
#define DFLAG_MPPT1        (1u << 9)
#define DFLAG_MPPT2        (1u << 10)
#define DFLAG_MPPT3        (1u << 11)
#define DFLAG_MPPT4        (1u << 12)
#define DFLAG_GPS_POS      (1u << 13)
#define DFLAG_GPS_NAV      (1u << 14)
#define DFLAG_GPS_TIME     (1u << 15)
#define DFLAG_CHARGING     (1u << 16)

static inline void display_route_frame(display_data_t *dd, uint32_t id, const uint8_t data[8]) {
    switch (id) {
    case CAN_ID_BPS_SAFETY_STATE:    display_decode_bps_safety(&dd->bps_safety, data);    dd->update_flags |= DFLAG_BPS_SAFETY; break;
    case CAN_ID_BPS_POWER_LIMIT:     display_decode_bps_power(&dd->bps_power, data);      dd->update_flags |= DFLAG_BPS_POWER;  break;
    case CAN_ID_VEGA_DRIVER_INPUTS:  display_decode_vega_inputs(&dd->vega_inputs, data);  dd->update_flags |= DFLAG_VEGA_INPUTS; break;
    case CAN_ID_VEGA_VEHICLE_STATUS: display_decode_vega_status(&dd->vega_status, data);  dd->update_flags |= DFLAG_VEGA_STATUS; break;
    case CAN_ID_MC_STATUS1:          display_decode_mc_status1(&dd->mc_status1, data);    dd->update_flags |= DFLAG_MC_STATUS1; break;
    case CAN_ID_MC_STATUS2:          display_decode_mc_status2(&dd->mc_status2, data);    dd->update_flags |= DFLAG_MC_STATUS2; break;
    case CAN_ID_R_BMS_CELL_EXTREMES: display_decode_cell_extremes(&dd->cells, data);     dd->update_flags |= DFLAG_CELLS; break;
    case CAN_ID_R_BMS_TEMP_EXTREMES: display_decode_temp_extremes(&dd->temps, data);     dd->update_flags |= DFLAG_TEMPS; break;
    case CAN_ID_BPS_SUPPL_BATTERY:   display_decode_suppl_batt(&dd->suppl, data);        dd->update_flags |= DFLAG_SUPPL; break;
    case CAN_ID_MPPT1_POWER_MEAS:    display_decode_mppt_power(&dd->mppt[0], data);      dd->update_flags |= DFLAG_MPPT1; break;
    case CAN_ID_MPPT2_POWER_MEAS:    display_decode_mppt_power(&dd->mppt[1], data);      dd->update_flags |= DFLAG_MPPT2; break;
    case CAN_ID_MPPT3_POWER_MEAS:    display_decode_mppt_power(&dd->mppt[2], data);      dd->update_flags |= DFLAG_MPPT3; break;
    case CAN_ID_MPPT4_POWER_MEAS:    display_decode_mppt_power(&dd->mppt[3], data);      dd->update_flags |= DFLAG_MPPT4; break;
    case CAN_ID_ALTAIR_GPS_POS:      display_decode_gps_pos(&dd->gps_pos, data);         dd->update_flags |= DFLAG_GPS_POS; break;
    case CAN_ID_ALTAIR_GPS_NAV:      display_decode_gps_nav(&dd->gps_nav, data);         dd->update_flags |= DFLAG_GPS_NAV; break;
    case CAN_ID_ALTAIR_GPS_TIME:     display_decode_gps_time(&dd->gps_time, data);       dd->update_flags |= DFLAG_GPS_TIME; break;
    case CAN_ID_BPS_CHARGING_INFO:   display_decode_charging(&dd->charging, data);       dd->update_flags |= DFLAG_CHARGING; break;
    }
}

#endif /* DISPLAY_CAN_SPEC_H */
