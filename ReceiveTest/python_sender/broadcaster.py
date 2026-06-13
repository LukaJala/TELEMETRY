"""
broadcaster.py — Lysander Display Test Broadcaster

Sends simulated CAN frames over TCP to the ESP32-P4 display.

Interactive commands (type + Enter while running):
  0-5   Pin to scenario (only that scenario runs)
  c     Resume cycling through all scenarios
  t     Cycle display tab (BATTERY → SOLAR/MOTOR → GPS/TRIP → ...)
  v     Toggle camera feed on/off

Scenarios:
  0 - Normal cruising
  1 - Acceleration
  2 - Regen braking
  3 - Warning (low SOC, derating)
  4 - Fault (precharge, grace timer)
  5 - Charging
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
SCENARIO_DURATION_S = 30

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

# Tab shown automatically when cycling to each scenario
SCENARIO_TAB = [0, 1, 1, 0, 0, 2]

TAB_NAMES = ["BATTERY", "SOLAR/MOTOR", "GPS/TRIP"]

# ============================================================
# Scenarios
# ============================================================
SCENARIOS = [
    {
        "name": "Normal Cruising",
        "speed": 47.0,
        "throttle_pct": 0.35,
        "regen_pct": 0.0,
        "pack_v": 97.2,
        "pack_i": 18.5,
        "soc": 72.0,
        "hi_cell_v": 3.612,
        "lo_cell_v": 3.589,
        "hi_temp": 34,
        "lo_temp": 28,
        "solar_w": [218, 215, 211, 198],
        "fault_code": 0,
        "safety_state": 0,
        "grace_timer": 0,
        "derate_pct": 0,
        "warning": False,
        "charging": False,
    },
    {
        "name": "Acceleration",
        "speed": 72.0,
        "throttle_pct": 0.85,
        "regen_pct": 0.0,
        "pack_v": 94.1,
        "pack_i": 45.2,
        "soc": 68.0,
        "hi_cell_v": 3.520,
        "lo_cell_v": 3.485,
        "hi_temp": 38,
        "lo_temp": 30,
        "solar_w": [225, 222, 219, 210],
        "fault_code": 0,
        "safety_state": 0,
        "grace_timer": 0,
        "derate_pct": 0,
        "warning": False,
        "charging": False,
    },
    {
        "name": "Regen Braking",
        "speed": 35.0,
        "throttle_pct": 0.0,
        "regen_pct": 0.65,
        "pack_v": 99.8,
        "pack_i": -8.5,
        "soc": 74.0,
        "hi_cell_v": 3.700,
        "lo_cell_v": 3.680,
        "hi_temp": 36,
        "lo_temp": 29,
        "solar_w": [190, 188, 185, 180],
        "fault_code": 0,
        "safety_state": 0,
        "grace_timer": 0,
        "derate_pct": 0,
        "warning": False,
        "charging": False,
    },
    {
        "name": "Warning - Low SOC Derating",
        "speed": 30.0,
        "throttle_pct": 0.20,
        "regen_pct": 0.0,
        "pack_v": 82.4,
        "pack_i": 12.0,
        "soc": 15.0,
        "hi_cell_v": 3.120,
        "lo_cell_v": 3.020,
        "hi_temp": 41,
        "lo_temp": 33,
        "solar_w": [105, 98, 102, 95],
        "fault_code": 0,
        "safety_state": 0,
        "grace_timer": 0,
        "derate_pct": 0,
        "warning": True,
        "charging": False,
    },
    {
        "name": "FAULT - Precharge",
        "speed": 25.0,
        "throttle_pct": 0.0,
        "regen_pct": 0.0,
        "pack_v": 96.0,
        "pack_i": 2.0,
        "soc": 65.0,
        "hi_cell_v": 3.580,
        "lo_cell_v": 3.550,
        "hi_temp": 35,
        "lo_temp": 28,
        "solar_w": [200, 195, 190, 185],
        "fault_code": 9,
        "safety_state": 1,
        "grace_timer": 90,
        "derate_pct": 0,
        "warning": False,
        "charging": False,
    },
    {
        "name": "Charging",
        "speed": 0.0,
        "throttle_pct": 0.0,
        "regen_pct": 0.0,
        "pack_v": 108.5,
        "pack_i": -12.0,
        "soc": 88.0,
        "hi_cell_v": 4.020,
        "lo_cell_v": 4.005,
        "hi_temp": 32,
        "lo_temp": 27,
        "solar_w": [0, 0, 0, 0],
        "fault_code": 0,
        "safety_state": 0,
        "grace_timer": 0,
        "derate_pct": 0,
        "warning": False,
        "charging": True,
    },
]

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
# Encode helpers
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


def enc_bps_safety(
    vstate,
    sstate,
    hv_rdy,
    hazard,
    fault,
    red,
    yellow,
    dcdc,
    batt,
    grace,
    relays0,
    relays1,
    prech_done,
    vcs_mia,
    bms_mia,
    derate_pct,
):
    d = bytearray(8)
    d[0] = (
        (vstate & 0x07)
        | ((sstate & 0x03) << 3)
        | (int(hv_rdy) << 5)
        | (int(hazard) << 7)
    )
    d[1] = (
        (fault & 0x0F)
        | (int(red) << 4)
        | (int(yellow) << 5)
        | (int(dcdc) << 6)
        | (int(batt) << 7)
    )
    d[2] = grace & 0xFF
    d[3] = relays0 & 0xFF
    d[4] = relays1 & 0xFF
    d[5] = (
        int(prech_done)
        | (int(vcs_mia) << 1)
        | (int(bms_mia) << 2)
        | (((derate_pct // 4) & 0x1F) << 3)
    )
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
    d[6] = 0
    d[7] = 0
    return d


def enc_temps(hi, hi_id, lo, lo_id, avg, board):
    d = bytearray(8)
    d[0] = hi & 0xFF
    d[1] = hi_id & 0xFF
    d[2] = lo & 0xFF
    d[3] = lo_id & 0xFF
    d[4] = avg & 0xFF
    d[5] = board & 0xFF
    d[6] = 0
    d[7] = 0
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
    d[0] = h
    d[1] = m
    d[2] = s
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
    d[6] = 0
    d[7] = _next_alive()
    return d


# ============================================================
# Rate-grouped broadcast functions
# ============================================================


def broadcast_100ms(sc, elapsed_s):
    wobble = math.sin(elapsed_s * 0.5) * 0.5
    speed = max(0.0, sc["speed"] + wobble * 2.0)

    grace = sc["grace_timer"]
    derate = sc["derate_pct"]
    if sc["fault_code"] != 0 and sc["grace_timer"] > 0:
        elapsed_int = int(elapsed_s)
        grace = max(0, sc["grace_timer"] - elapsed_int)
        derate = int((sc["grace_timer"] - grace) / sc["grace_timer"] * 100)

    relays0 = 0xFF
    relays1 = 0x55
    if grace == 0 and sc["fault_code"] != 0:
        relays0 = 0x00
        relays1 = 0x00

    vstate = 4 if sc["speed"] > 0 else 1
    max_thr = 255 - (derate * 255 // 100)
    max_reg = 200 if sc["regen_pct"] > 0 else 0
    thr_out = int(sc["throttle_pct"] * max_thr)
    reg_out = int(sc["regen_pct"] * max_reg)

    reason = 0
    if sc["warning"]:
        reason |= 0x10
    if derate > 0:
        reason |= 0x40
    if max_reg == 0:
        reason |= 0x20

    send_frame(
        ID_BPS_SAFETY,
        enc_bps_safety(
            vstate,
            sc["safety_state"],
            True,
            sc["fault_code"] != 0,
            sc["fault_code"],
            sc["fault_code"] != 0,
            sc["warning"],
            True,
            True,
            grace,
            relays0,
            relays1,
            True,
            False,
            False,
            derate,
        ),
    )

    send_frame(
        ID_BPS_POWER,
        enc_bps_power(
            max_thr,
            max_reg,
            sc["pack_v"] + wobble,
            sc["pack_i"] + wobble * 0.5,
            sc["soc"],
            reason,
        ),
    )

    send_frame(
        ID_VEGA_INPUTS,
        enc_vega_inputs(
            int(sc["throttle_pct"] * 3600 + 200),
            int(sc["regen_pct"] * 3200 + 500) if sc["regen_pct"] > 0 else 100,
            thr_out,
            reg_out,
            sc["regen_pct"] > 0,
            True,
            1 if sc["speed"] > 0 else 0,
            sc["regen_pct"] > 0,
        ),
    )

    rpm = int(speed / (1.6 * 60.0 / 1000.0))
    send_frame(
        ID_MC_STATUS1,
        enc_mc_status1(rpm, sc["pack_i"] * 0.8 + wobble, sc["pack_v"] + wobble, 0),
    )

    send_frame(
        ID_MC_STATUS2,
        enc_mc_status2(
            thr_out, int(sc["hi_temp"] + 8 + wobble), int(sc["hi_temp"] + 17 + wobble)
        ),
    )


def broadcast_200ms(sc, elapsed_s):
    wobble = math.sin(elapsed_s * 0.3) * 0.3

    send_frame(
        ID_VEGA_STATUS,
        enc_vega_status(
            sc["speed"] + wobble * 2,
            127.4 + elapsed_s * 0.00001,
            1 if sc["speed"] > 0 else 0,
            sc["regen_pct"] > 0,
            True,
            0,
        ),
    )

    send_frame(
        ID_CELL_EXTR,
        enc_cells(
            sc["hi_cell_v"] + wobble * 0.002, 14, sc["lo_cell_v"] + wobble * 0.001, 3
        ),
    )


def broadcast_500ms(sc, elapsed_s):
    wobble = math.sin(elapsed_s * 0.2)

    send_frame(
        ID_TEMP_EXTR,
        enc_temps(
            sc["hi_temp"] + int(wobble),
            7,
            sc["lo_temp"],
            12,
            (sc["hi_temp"] + sc["lo_temp"]) // 2,
            sc["lo_temp"] + 5,
        ),
    )

    for i, mppt_id in enumerate(MPPT_IDS):
        pwr = max(0.0, sc["solar_w"][i] + wobble * 5.0)
        vin = 38.0 + wobble
        iin = pwr / vin if vin > 0 else 0.0
        vout = 97.0
        iout = pwr / vout if vout > 0 else 0.0
        send_frame(mppt_id, enc_mppt(vin, iin, vout, iout))


def broadcast_1s(sc, elapsed_s):
    send_frame(ID_BPS_SUPPL, enc_suppl(25.1, -0.3, 84.0, 0, 28))

    if sc["charging"]:
        send_frame(ID_BPS_CHARGING, enc_charging(True, True, sc["pack_v"], -12.0, 35))
    else:
        send_frame(ID_BPS_CHARGING, enc_charging(False, False, 0.0, 0.0, 25))

    lat = 42.7284 + elapsed_s * 0.00001
    lon = -84.4823 + elapsed_s * 0.000015
    send_frame(ID_GPS_POS, enc_gps_pos(lat, lon))

    heading = 247.0 + math.sin(elapsed_s * 0.1) * 5.0
    send_frame(ID_GPS_NAV, enc_gps_nav(sc["speed"] / 3.6, heading, 312.0, 3, 12))

    sec = int(elapsed_s)
    send_frame(
        ID_GPS_TIME, enc_gps_time(14, 32 + (sec // 60) % 60, sec % 60, True, 1.2)
    )


# ============================================================
# Non-blocking input thread
# ============================================================
_cmd_queue = queue.Queue()


def _input_thread():
    while True:
        try:
            line = input().strip().lower()
            if line:
                _cmd_queue.put(line)
        except EOFError:
            break


_current_tab = 0
_cam_active = False


def _switch_scenario(idx, pin_label):
    global _current_tab
    sc = SCENARIOS[idx]
    tab = SCENARIO_TAB[idx]
    _current_tab = tab
    print(f"  Scenario {idx}: {sc['name']}  [{TAB_NAMES[tab]}]  {pin_label}")
    send_cmd(f"__TAB_{tab}__")
    return idx, time.monotonic()


def _print_help():
    print()
    print("  ┌─────────────────────────────────────────────────┐")
    print("  │  SCENARIO KEYS  (pin to one; only it loops)     │")
    print("  │                                                  │")
    for i, sc in enumerate(SCENARIOS):
        tab_name = {0: "BATT", 1: "SOLAR", 2: "GPS"}[SCENARIO_TAB[i]]
        print(f"  │  [{i}]  {sc['name']:<30s}  [{tab_name}]  │")
    print("  │                                                  │")
    print("  │  [c]   Resume cycling through all scenarios      │")
    print("  ├─────────────────────────────────────────────────┤")
    print("  │  [t]   Cycle display tab (BATTERY → SOLAR →     │")
    print("  │         GPS/TRIP → BATTERY ...)                  │")
    print("  │  [v]   Toggle camera feed on/off                 │")
    print("  └─────────────────────────────────────────────────┘")
    print()


# ============================================================
# Entry point
# ============================================================
def main():
    print("=" * 52)
    print("  Lysander Display Test Broadcaster")
    print("=" * 52)
    print(f"  Target : {DISPLAY_IP}:{DISPLAY_PORT}")
    print(f"  W5500-side Ethernet adapter must be set to 192.168.2.1")
    print("=" * 52)
    _print_help()

    while True:
        try:
            tcp_connect()
            break
        except Exception as e:
            print(f"  Connection failed ({e}) — retrying in 2s...")
            time.sleep(2)

    # Start background input thread
    t = threading.Thread(target=_input_thread, daemon=True)
    t.start()

    scenario_idx = 0
    pin_scenario = None  # None = cycling, int = pinned
    scenario_start = time.monotonic()
    last_100ms = last_200ms = last_500ms = last_1s = 0.0

    scenario_idx, scenario_start = _switch_scenario(0, "(cycling)")
    print("  Ctrl+C to stop.\n")

    try:
        while True:
            now = time.monotonic()
            elapsed = now - scenario_start

            # ── Process any queued commands ──────────────────────────────
            while not _cmd_queue.empty():
                cmd = _cmd_queue.get_nowait()

                if cmd in ("0", "1", "2", "3", "4", "5"):
                    new_idx = int(cmd)
                    pin_scenario = new_idx
                    scenario_idx, scenario_start = _switch_scenario(new_idx, "(pinned)")
                    elapsed = 0.0

                elif cmd == "c":
                    pin_scenario = None
                    print(
                        f"  Cycling mode — next advance in "
                        f"{max(0, SCENARIO_DURATION_S - elapsed):.0f}s"
                    )

                elif cmd == "t":
                    global _current_tab
                    _current_tab = (_current_tab + 1) % len(TAB_NAMES)
                    send_cmd(f"__TAB_{_current_tab}__")
                    print(f"  Tab -> {TAB_NAMES[_current_tab]}")

                elif cmd == "v":
                    global _cam_active
                    _cam_active = not _cam_active
                    if _cam_active:
                        send_cmd("__CAM_START__")
                        print("  Camera ON")
                    else:
                        send_cmd("__CAM_STOP__")
                        print("  Camera OFF")

                else:
                    print(f"  Unknown command '{cmd}'")
                    _print_help()

            # ── Scenario advance (cycling mode only) ─────────────────────
            if pin_scenario is None and elapsed >= SCENARIO_DURATION_S:
                scenario_idx = (scenario_idx + 1) % len(SCENARIOS)
                scenario_idx, scenario_start = _switch_scenario(
                    scenario_idx, "(cycling)"
                )
                elapsed = 0.0

            # ── Broadcast ────────────────────────────────────────────────
            sc = SCENARIOS[scenario_idx]

            if now - last_100ms >= 0.100:
                broadcast_100ms(sc, elapsed)
                last_100ms = now

            if now - last_200ms >= 0.200:
                broadcast_200ms(sc, elapsed)
                last_200ms = now

            if now - last_500ms >= 0.500:
                broadcast_500ms(sc, elapsed)
                last_500ms = now

            if now - last_1s >= 1.0:
                broadcast_1s(sc, elapsed)
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
