"""
custom_broadcaster.py — Lysander Display *Manual* Broadcaster

Unlike broadcaster.py (which cycles fixed scenarios), this keeps a single live
state of every display parameter and lets you set each one individually from the
prompt. It then re-broadcasts that state continuously at the correct CAN rates.

Type  help  for the command list, or  show  to dump the current values.

Set a parameter:           <name> <value>        e.g.  speed 65
Set all 4 solar arrays:    solar 150
Set one solar array:       solar2 0
Booleans accept:           on/off true/false 1/0  e.g.  charging on
Throttle/regen are 0-100:  throttle 80   regen 50

Other commands:
  show / list        Print every parameter and its current value
  reset              Restore all parameters to defaults
  preset <name>      Load a built-in preset (normal accel regen warn fault charge)
  tab <0|1|2>        Switch display tab (BATTERY / SOLAR-MOTOR / GPS-TRIP)
  t                  Cycle display tab
  cam on|off  / v    Camera feed on/off (v toggles)
  wobble on|off      Add gentle sine jitter to live values (default off)
  help / ?           Show help
  quit / q           Exit
"""

import socket
import time
import struct
import math
import threading
import queue

# ============================================================
# Configuration
# ============================================================
DISPLAY_IP = "192.168.2.100"
DISPLAY_PORT = 5000

# ============================================================
# CAN IDs (from display_can_spec.h)
# ============================================================
ID_BPS_SAFETY = 0x08FF1000
ID_BPS_POWER = 0x0CFF2000
ID_BPS_SUPPL = 0x14FF3300
ID_BPS_CHARGING = 0x14FF3600
ID_VEGA_INPUTS = 0x0CFF211E
ID_VEGA_STATUS = 0x18FF101E
ID_MC_STATUS1 = 0x0CF11E05
ID_MC_STATUS2 = 0x0CF11F05
ID_CELL_EXTR = 0x14FF3200
ID_TEMP_EXTR = 0x14FF4100
ID_MPPT1 = 0x00007800
ID_MPPT2 = 0x00007900
ID_MPPT3 = 0x00007A00
ID_MPPT4 = 0x00007B00
ID_GPS_POS = 0x14FF0150
ID_GPS_NAV = 0x14FF0250
ID_GPS_TIME = 0x18FF0350

MPPT_IDS = [ID_MPPT1, ID_MPPT2, ID_MPPT3, ID_MPPT4]

TAB_NAMES = ["BATTERY", "SOLAR/MOTOR", "GPS/TRIP"]

# Fault names (mirrors BPS_FAULT_NAMES in display_can_spec.h)
FAULT_NAMES = [
    "NO FAULT", "SUPPL UV", "SUPPL OV", "SUPPL UT", "SUPPL OT",
    "DCDC UV", "DCDC OV", "OVERCURRENT", "CURRENT SENSOR", "PRECHARGE",
    "MAIN PACK UV", "MAIN PACK OV", "CAN FAULT", "CAN TIMEOUT",
]


# ============================================================
# Value casters
# ============================================================
def _b(v):
    s = str(v).strip().lower()
    if s in ("1", "on", "true", "yes", "y", "t"):
        return True
    if s in ("0", "off", "false", "no", "n", "f"):
        return False
    raise ValueError(f"expected on/off, got '{v}'")


def _pct100(v):
    """Accept 0-100 (or a trailing %) and store as 0.0-1.0."""
    s = str(v).strip().rstrip("%")
    return max(0.0, min(1.0, float(s) / 100.0))


# ============================================================
# Parameter registry — (key, caster, help). Defaults live in DEFAULTS.
# Grouped only for nice `show` output.
# ============================================================
PARAM_GROUPS = [
    ("DRIVE", [
        ("speed",       float,   "km/h"),
        ("drive_mode",  int,     "0=Park 1=FWD 2=REV"),
        ("throttle",    _pct100, "0-100 (% of max throttle)"),
        ("regen",       _pct100, "0-100 (% of max regen, 0=regen off)"),
        ("odometer",    float,   "km"),
    ]),
    ("PACK / BPS", [
        ("pack_v",      float,   "Volts"),
        ("pack_i",      float,   "Amps (+discharge / -charge)"),
        ("soc",         float,   "0-100 %"),
        ("warning",     _b,      "yellow cockpit warning"),
        ("vehicle_state", int,   "0=OFF 1=STBY 2=ACC 4=PRIMARY"),
        ("safety_state", int,    "0=Normal 1=LimpHome 2=Failsafe"),
        ("fault_code",  int,     "0-13 (see fault list)"),
        ("grace_timer", int,     "seconds (limp-home countdown)"),
        ("derate_pct",  int,     "0-100 % (4% steps)"),
    ]),
    ("CELLS", [
        ("hi_cell_v",   float,   "Volts"),
        ("lo_cell_v",   float,   "Volts"),
        ("hi_cell_id",  int,     "cell # 0-27"),
        ("lo_cell_id",  int,     "cell # 0-27"),
    ]),
    ("TEMPS", [
        ("hi_temp",     int,     "deg C (pack hi)"),
        ("lo_temp",     int,     "deg C (pack lo)"),
        ("motor_temp",  int,     "deg C"),
        ("ctrl_temp",   int,     "deg C"),
    ]),
    ("SOLAR (MPPT W)", [
        ("solar1",      float,   "Watts"),
        ("solar2",      float,   "Watts"),
        ("solar3",      float,   "Watts"),
        ("solar4",      float,   "Watts"),
    ]),
    ("SUPPL BATTERY", [
        ("suppl_v",     float,   "Volts"),
        ("suppl_i",     float,   "Amps"),
        ("suppl_soc",   float,   "0-100 %"),
        ("suppl_temp",  int,     "deg C"),
    ]),
    ("CHARGING", [
        ("charging",    _b,      "charger active+connected"),
        ("charge_v",    float,   "charger output Volts"),
        ("charge_i",    float,   "charger output Amps"),
        ("charge_temp", int,     "deg C"),
    ]),
    ("GPS / NAV", [
        ("gps_lat",     float,   "degrees"),
        ("gps_lon",     float,   "degrees"),
        ("gps_heading", float,   "degrees"),
        ("gps_alt",     float,   "meters MSL"),
        ("gps_fix",     int,     "0=No 2=2D 3=3D"),
        ("gps_sats",    int,     "satellite count"),
        ("gps_hour",    int,     "0-23"),
        ("gps_min",     int,     "0-59"),
        ("gps_sec",     int,     "0-59"),
    ]),
]

# Flatten registry: key -> caster
CASTERS = {}
for _grp, _items in PARAM_GROUPS:
    for _k, _cast, _h in _items:
        CASTERS[_k] = _cast

DEFAULTS = {
    "speed": 47.0, "drive_mode": 1, "throttle": 0.35, "regen": 0.0, "odometer": 127.4,
    "pack_v": 97.2, "pack_i": 18.5, "soc": 72.0, "warning": False,
    "vehicle_state": 4, "safety_state": 0, "fault_code": 0, "grace_timer": 0, "derate_pct": 0,
    "hi_cell_v": 3.612, "lo_cell_v": 3.589, "hi_cell_id": 14, "lo_cell_id": 3,
    "hi_temp": 34, "lo_temp": 28, "motor_temp": 51, "ctrl_temp": 42,
    "solar1": 218.0, "solar2": 215.0, "solar3": 211.0, "solar4": 198.0,
    "suppl_v": 25.1, "suppl_i": -0.3, "suppl_soc": 84.0, "suppl_temp": 28,
    "charging": False, "charge_v": 0.0, "charge_i": 0.0, "charge_temp": 25,
    "gps_lat": 42.7284, "gps_lon": -84.4823, "gps_heading": 247.0, "gps_alt": 312.0,
    "gps_fix": 3, "gps_sats": 12, "gps_hour": 14, "gps_min": 32, "gps_sec": 0,
}

# Built-in presets: only the keys that differ from DEFAULTS.
PRESETS = {
    "normal": {},
    "accel": {"speed": 72.0, "throttle": 0.85, "pack_i": 45.2, "soc": 68.0,
              "hi_cell_v": 3.520, "lo_cell_v": 3.485, "hi_temp": 38},
    "regen": {"speed": 35.0, "throttle": 0.0, "regen": 0.65, "pack_i": -8.5,
              "soc": 74.0, "hi_cell_v": 3.700, "lo_cell_v": 3.680},
    "warn":  {"speed": 30.0, "throttle": 0.20, "pack_v": 82.4, "pack_i": 12.0,
              "soc": 15.0, "hi_cell_v": 3.120, "lo_cell_v": 3.020, "hi_temp": 41,
              "warning": True,
              "solar1": 105.0, "solar2": 98.0, "solar3": 102.0, "solar4": 95.0},
    "fault": {"speed": 25.0, "throttle": 0.0, "pack_v": 96.0, "pack_i": 2.0,
              "soc": 65.0, "fault_code": 9, "safety_state": 1, "grace_timer": 90},
    "charge": {"speed": 0.0, "throttle": 0.0, "pack_v": 108.5, "pack_i": -12.0,
               "soc": 88.0, "hi_cell_v": 4.020, "lo_cell_v": 4.005,
               "charging": True, "charge_v": 108.5, "charge_i": -12.0, "charge_temp": 35,
               "solar1": 0.0, "solar2": 0.0, "solar3": 0.0, "solar4": 0.0},
}

state = dict(DEFAULTS)

# ============================================================
# TCP
# ============================================================
_sock = None


def tcp_connect():
    global _sock
    _sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    _sock.settimeout(5)
    _sock.connect((DISPLAY_IP, DISPLAY_PORT))
    _sock.settimeout(None)
    print(f"Connected to {DISPLAY_IP}:{DISPLAY_PORT}")


def send_frame(can_id, payload):
    pkt = b"\x01" + struct.pack("<IB", can_id, 8) + bytes(payload)
    _sock.send(pkt)


def send_cmd(cmd: str):
    _sock.send(cmd.encode() + b"\x00")


# ============================================================
# Encode helpers (byte-level packing, from display_can_spec.h)
# ============================================================
_alive = 0


def _next_alive():
    global _alive
    v = _alive
    _alive = (_alive + 1) & 0xFF
    return v


def _u8(v):
    return max(0, min(255, int(v)))


def _s16(v):
    return struct.pack("<h", max(-32768, min(32767, int(v))))


def enc_bps_safety(vstate, sstate, hv_rdy, hazard, fault, red, yellow, dcdc, batt,
                   grace, relays0, relays1, prech_done, vcs_mia, bms_mia, derate_pct):
    d = bytearray(8)
    d[0] = (vstate & 0x07) | ((sstate & 0x03) << 3) | (int(hv_rdy) << 5) | (int(hazard) << 7)
    d[1] = (fault & 0x0F) | (int(red) << 4) | (int(yellow) << 5) | (int(dcdc) << 6) | (int(batt) << 7)
    d[2] = grace & 0xFF
    d[3] = relays0 & 0xFF
    d[4] = relays1 & 0xFF
    d[5] = int(prech_done) | (int(vcs_mia) << 1) | (int(bms_mia) << 2) | (((derate_pct // 4) & 0x1F) << 3)
    d[6] = 0
    d[7] = _next_alive()
    return d


def enc_bps_power(max_thr, max_reg, pack_v, pack_i, soc, reason):
    v = int(pack_v / 0.1) & 0xFFFF
    d = bytearray(8)
    d[0] = _u8(max_thr)
    d[1] = _u8(max_reg)
    d[2] = v & 0xFF
    d[3] = (v >> 8) & 0xFF
    d[4:6] = _s16(pack_i / 0.1)
    d[6] = _u8(soc / 0.5)
    d[7] = reason & 0xFF
    return d


def enc_vega_inputs(pedal, brake, thr_out, reg_out, brk_sw, thr_sw, mode, regen):
    d = bytearray(8)
    d[0] = pedal & 0xFF
    d[1] = (pedal >> 8) & 0xFF
    d[2] = brake & 0xFF
    d[3] = (brake >> 8) & 0xFF
    d[4] = _u8(thr_out)
    d[5] = _u8(reg_out)
    d[6] = int(brk_sw) | (int(thr_sw) << 1) | ((mode & 0x03) << 2) | (int(regen) << 4)
    d[7] = _next_alive()
    return d


def enc_vega_status(speed_kmh, odo_km, mode, regen, sd_log, lh_timer):
    s = int(speed_kmh / 0.1) & 0xFFFF
    o = int(odo_km / 0.1) & 0xFFFFFF
    d = bytearray(8)
    d[0] = s & 0xFF
    d[1] = (s >> 8) & 0xFF
    d[2] = o & 0xFF
    d[3] = (o >> 8) & 0xFF
    d[4] = (o >> 16) & 0xFF
    d[5] = (mode & 0x03) | (int(regen) << 2) | (int(sd_log) << 3)
    d[6] = lh_timer & 0xFF
    d[7] = _next_alive()
    return d


def enc_mc_status1(rpm, motor_a, batt_v, err):
    mi = int(motor_a / 0.1) & 0xFFFF
    bv = int(batt_v / 0.1) & 0xFFFF
    d = bytearray(8)
    d[0] = rpm & 0xFF
    d[1] = (rpm >> 8) & 0xFF
    d[2] = mi & 0xFF
    d[3] = (mi >> 8) & 0xFF
    d[4] = bv & 0xFF
    d[5] = (bv >> 8) & 0xFF
    d[6] = err & 0xFF
    d[7] = (err >> 8) & 0xFF
    return d


def enc_mc_status2(thr_fb, ctrl_c, motor_c):
    d = bytearray(8)
    d[0] = _u8(thr_fb)
    d[1] = _u8(ctrl_c + 40)
    d[2] = _u8(motor_c + 30)
    d[3] = 0
    d[4] = 0x05
    d[5] = 0
    d[6] = 0
    d[7] = 0
    return d


def enc_cells(hi_v, hi_id, lo_v, lo_id):
    hv = int(hi_v / 0.0001) & 0xFFFF
    lv = int(lo_v / 0.0001) & 0xFFFF
    d = bytearray(8)
    d[0] = hv & 0xFF
    d[1] = (hv >> 8) & 0xFF
    d[2] = hi_id & 0xFF
    d[3] = lv & 0xFF
    d[4] = (lv >> 8) & 0xFF
    d[5] = lo_id & 0xFF
    return d


def enc_temps(hi, hi_id, lo, lo_id, avg, board):
    d = bytearray(8)
    d[0] = hi & 0xFF
    d[1] = hi_id & 0xFF
    d[2] = lo & 0xFF
    d[3] = lo_id & 0xFF
    d[4] = avg & 0xFF
    d[5] = board & 0xFF
    return d


def enc_suppl(v, a, soc, status, temp):
    sv = int(v / 0.01) & 0xFFFF
    d = bytearray(8)
    d[0] = sv & 0xFF
    d[1] = (sv >> 8) & 0xFF
    d[2:4] = _s16(a / 0.01)
    d[4] = _u8(soc / 0.5)
    d[5] = status & 0xFF
    d[6] = _u8(temp)
    d[7] = _next_alive()
    return d


def enc_mppt(vin, iin, vout, iout):
    d = bytearray(8)
    d[0:2] = _s16(vin / 0.01)
    d[2:4] = _s16(iin / 0.0005)
    d[4:6] = _s16(vout / 0.01)
    d[6:8] = _s16(iout / 0.0005)
    return d


def enc_gps_pos(lat, lon):
    return bytearray(struct.pack("<ii", int(lat / 1e-7), int(lon / 1e-7)))


def enc_gps_nav(speed_ms, heading, alt_m, fix, sats):
    sp = int(speed_ms / 0.01) & 0xFFFF
    hd = int(heading / 0.01) & 0xFFFF
    d = bytearray(8)
    d[0] = sp & 0xFF
    d[1] = (sp >> 8) & 0xFF
    d[2] = hd & 0xFF
    d[3] = (hd >> 8) & 0xFF
    d[4:6] = _s16(alt_m / 0.1)
    d[6] = fix & 0xFF
    d[7] = sats & 0xFF
    return d


def enc_gps_time(h, m, s, fix_ok, pdop):
    pd = int(pdop / 0.01) & 0xFFFF
    d = bytearray(8)
    d[0] = h & 0xFF
    d[1] = m & 0xFF
    d[2] = s & 0xFF
    d[3] = 0x01 if fix_ok else 0x00
    d[4] = pd & 0xFF
    d[5] = (pd >> 8) & 0xFF
    d[6] = 10
    d[7] = 5
    return d


def enc_charging(active, connected, out_v, out_a, temp):
    cv = int(out_v / 0.1) & 0xFFFF
    d = bytearray(8)
    d[0] = int(active) | (int(connected) << 1)
    d[1] = cv & 0xFF
    d[2] = (cv >> 8) & 0xFF
    d[3:5] = _s16((out_a + 320.0) / 0.1)
    d[5] = _u8(temp + 40)
    d[7] = _next_alive()
    return d


# ============================================================
# Broadcast — read the live `state` and emit frames
# ============================================================
_wobble_on = False


def _wob(t, freq, amp):
    return math.sin(t * freq) * amp if _wobble_on else 0.0


def broadcast_100ms(s, t):
    speed = max(0.0, s["speed"] + _wob(t, 0.5, 1.0))
    fault = s["fault_code"]
    grace = s["grace_timer"]
    derate = s["derate_pct"]

    relays0, relays1 = 0xFF, 0x55
    if fault != 0 and grace == 0:
        relays0, relays1 = 0x00, 0x00

    max_thr = 255 - (derate * 255 // 100)
    max_reg = 200 if s["regen"] > 0 else 0
    thr_out = int(s["throttle"] * max_thr)
    reg_out = int(s["regen"] * max_reg)

    reason = 0
    if s["warning"]:
        reason |= 0x10
    if derate > 0:
        reason |= 0x40
    if max_reg == 0:
        reason |= 0x20

    send_frame(ID_BPS_SAFETY, enc_bps_safety(
        s["vehicle_state"], s["safety_state"], True, fault != 0, fault,
        fault != 0, s["warning"], True, True, grace, relays0, relays1,
        True, False, False, derate))

    send_frame(ID_BPS_POWER, enc_bps_power(
        max_thr, max_reg, s["pack_v"] + _wob(t, 0.5, 0.5),
        s["pack_i"] + _wob(t, 0.5, 0.3), s["soc"], reason))

    send_frame(ID_VEGA_INPUTS, enc_vega_inputs(
        int(s["throttle"] * 3600 + 200),
        int(s["regen"] * 3200 + 500) if s["regen"] > 0 else 100,
        thr_out, reg_out, s["regen"] > 0, True, s["drive_mode"], s["regen"] > 0))

    rpm = int(speed / (1.6 * 60.0 / 1000.0))
    send_frame(ID_MC_STATUS1, enc_mc_status1(
        rpm, s["pack_i"] * 0.8, s["pack_v"] + _wob(t, 0.5, 0.5), 0))

    send_frame(ID_MC_STATUS2, enc_mc_status2(thr_out, s["ctrl_temp"], s["motor_temp"]))


def broadcast_200ms(s, t):
    send_frame(ID_VEGA_STATUS, enc_vega_status(
        s["speed"] + _wob(t, 0.3, 0.6), s["odometer"],
        s["drive_mode"], s["regen"] > 0, True, s["grace_timer"]))

    send_frame(ID_CELL_EXTR, enc_cells(
        s["hi_cell_v"] + _wob(t, 0.3, 0.002), s["hi_cell_id"],
        s["lo_cell_v"] + _wob(t, 0.3, 0.001), s["lo_cell_id"]))


def broadcast_500ms(s, t):
    avg = (s["hi_temp"] + s["lo_temp"]) // 2
    send_frame(ID_TEMP_EXTR, enc_temps(
        s["hi_temp"], 7, s["lo_temp"], 12, avg, s["lo_temp"] + 5))

    solar = [s["solar1"], s["solar2"], s["solar3"], s["solar4"]]
    for mppt_id, pwr in zip(MPPT_IDS, solar):
        pwr = max(0.0, pwr + _wob(t, 0.2, 5.0))
        vin = 38.0
        vout = 97.0
        send_frame(mppt_id, enc_mppt(vin, pwr / vin, vout, pwr / vout))


def broadcast_1s(s, t):
    send_frame(ID_BPS_SUPPL, enc_suppl(
        s["suppl_v"], s["suppl_i"], s["suppl_soc"], 0, s["suppl_temp"]))

    if s["charging"]:
        send_frame(ID_BPS_CHARGING, enc_charging(
            True, True, s["charge_v"], s["charge_i"], s["charge_temp"]))
    else:
        send_frame(ID_BPS_CHARGING, enc_charging(False, False, 0.0, 0.0, s["charge_temp"]))

    send_frame(ID_GPS_POS, enc_gps_pos(s["gps_lat"], s["gps_lon"]))
    send_frame(ID_GPS_NAV, enc_gps_nav(
        s["speed"] / 3.6, s["gps_heading"], s["gps_alt"], s["gps_fix"], s["gps_sats"]))
    send_frame(ID_GPS_TIME, enc_gps_time(
        s["gps_hour"], s["gps_min"], s["gps_sec"], s["gps_fix"] >= 3, 1.2))


# ============================================================
# Command handling
# ============================================================
_cmd_queue = queue.Queue()


def _input_thread():
    while True:
        try:
            line = input()
            _cmd_queue.put(line)
        except EOFError:
            break


def _fmt(v):
    if isinstance(v, bool):
        return "on" if v else "off"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


def show_state():
    print()
    print("  ── Current parameters " + "─" * 30)
    for grp, items in PARAM_GROUPS:
        print(f"  [{grp}]")
        for k, _cast, helptxt in items:
            line = f"    {k:<14} = {_fmt(state[k]):<12}"
            if k == "fault_code" and 0 <= state[k] < len(FAULT_NAMES):
                line += f"  ({FAULT_NAMES[state[k]]})"
            else:
                line += f"  {helptxt}"
            print(line)
    print(f"  wobble = {'on' if _wobble_on else 'off'}")
    print("  " + "─" * 51)
    print()


def print_help():
    print(__doc__)
    print("  Parameter names:")
    for grp, items in PARAM_GROUPS:
        keys = ", ".join(k for k, _c, _h in items)
        print(f"    {grp:<16}: {keys}")
    print()
    print("  Fault codes:")
    for i, name in enumerate(FAULT_NAMES):
        print(f"    {i:>2} = {name}")
    print()


def apply_preset(name):
    if name not in PRESETS:
        print(f"  Unknown preset '{name}'. Options: {', '.join(PRESETS)}")
        return
    state.update(DEFAULTS)
    state.update(PRESETS[name])
    print(f"  Loaded preset '{name}'.")


_current_tab = 0
_cam_active = False


def handle_command(line):
    global _current_tab, _cam_active, _wobble_on
    line = line.strip()
    if not line:
        return
    parts = line.split()
    cmd = parts[0].lower()
    args = parts[1:]

    if cmd in ("help", "?", "h"):
        print_help()
    elif cmd in ("show", "list", "ls", "get"):
        show_state()
    elif cmd in ("quit", "q", "exit"):
        raise KeyboardInterrupt
    elif cmd == "reset":
        state.update(DEFAULTS)
        print("  Reset to defaults.")
    elif cmd == "preset":
        if args:
            apply_preset(args[0].lower())
        else:
            print(f"  presets: {', '.join(PRESETS)}")
    elif cmd == "tab":
        if args and args[0].isdigit() and int(args[0]) < len(TAB_NAMES):
            _current_tab = int(args[0])
            send_cmd(f"__TAB_{_current_tab}__")
            print(f"  Tab -> {TAB_NAMES[_current_tab]}")
        else:
            print(f"  usage: tab <0..{len(TAB_NAMES) - 1}>")
    elif cmd == "t":
        _current_tab = (_current_tab + 1) % len(TAB_NAMES)
        send_cmd(f"__TAB_{_current_tab}__")
        print(f"  Tab -> {TAB_NAMES[_current_tab]}")
    elif cmd == "cam":
        if args and args[0].lower() in ("on", "off"):
            _cam_active = args[0].lower() == "on"
            send_cmd("__CAM_START__" if _cam_active else "__CAM_STOP__")
            print(f"  Camera {'ON' if _cam_active else 'OFF'}")
        else:
            print("  usage: cam on|off")
    elif cmd == "v":
        _cam_active = not _cam_active
        send_cmd("__CAM_START__" if _cam_active else "__CAM_STOP__")
        print(f"  Camera {'ON' if _cam_active else 'OFF'}")
    elif cmd == "wobble":
        if args:
            _wobble_on = _b(args[0])
            print(f"  Wobble {'on' if _wobble_on else 'off'}")
        else:
            print(f"  wobble is {'on' if _wobble_on else 'off'}")
    elif cmd == "solar" and len(args) == 1:
        # convenience: set all 4 arrays at once
        val = float(args[0])
        for k in ("solar1", "solar2", "solar3", "solar4"):
            state[k] = val
        print(f"  solar1..4 = {val:g}")
    elif cmd in CASTERS:
        if not args:
            print(f"  {cmd} = {_fmt(state[cmd])}")
            return
        try:
            state[cmd] = CASTERS[cmd](args[0])
            print(f"  {cmd} = {_fmt(state[cmd])}")
        except ValueError as e:
            print(f"  bad value for {cmd}: {e}")
    else:
        print(f"  Unknown command '{cmd}'  (type help)")


# ============================================================
# Entry point
# ============================================================
def main():
    print("=" * 52)
    print("  Lysander Manual Broadcaster")
    print("=" * 52)
    print(f"  Target : {DISPLAY_IP}:{DISPLAY_PORT}")
    print("  W5500-side Ethernet adapter must be set to 192.168.2.1")
    print("  Type 'help' for commands, 'show' for current values.")
    print("=" * 52)

    while True:
        try:
            tcp_connect()
            break
        except Exception as e:
            print(f"  Connection failed ({e}) — retrying in 2s...")
            time.sleep(2)

    threading.Thread(target=_input_thread, daemon=True).start()

    # Set the initial tab
    send_cmd(f"__TAB_{_current_tab}__")
    print("  Ctrl+C (or 'quit') to stop.\n")

    start = time.monotonic()
    last_100 = last_200 = last_500 = last_1s = 0.0

    try:
        while True:
            now = time.monotonic()
            t = now - start

            while not _cmd_queue.empty():
                handle_command(_cmd_queue.get_nowait())

            if now - last_100 >= 0.100:
                broadcast_100ms(state, t)
                last_100 = now
            if now - last_200 >= 0.200:
                broadcast_200ms(state, t)
                last_200 = now
            if now - last_500 >= 0.500:
                broadcast_500ms(state, t)
                last_500 = now
            if now - last_1s >= 1.0:
                broadcast_1s(state, t)
                last_1s = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n  Stopped.")
    except Exception as e:
        print(f"\n  Error: {e}")
    finally:
        if _sock:
            _sock.close()


if __name__ == "__main__":
    main()
