# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

**ReceiveTest** — Solar car telemetry display + reverse camera for the Lysander vehicle (MSU Solar Racing, FSGP/ASC 2026). Runs on a Waveshare ESP32-P4-Module-DEV-KIT with a 10.1" JD9365 DSI display (800×1280, landscape 1280×800) and an OV5647 CSI camera.

## Build & Flash

This is an ESP-IDF project. All commands require the IDF environment to be activated first.

```bash
# Build
idf.py build

# Flash and monitor
idf.py flash monitor

# Flash only
idf.py flash

# Serial monitor only
idf.py monitor
```

The board's static IP is `192.168.1.100`. The laptop Ethernet adapter must be set to `192.168.1.1`.

## Testing with Simulated Data

Run the Python broadcaster from `python_sender/` to send simulated CAN frames over TCP:

```bash
python python_sender/broadcaster.py
```

This cycles through 6 driving scenarios (normal, acceleration, regen, warning, fault, charging) every 30 seconds. The broadcaster connects to `192.168.1.100:5000`. Interactive keys while it runs: `0-5` pin a scenario, `c` resume cycling, `t` cycle tabs, `r` toggle reverse gear (camera should go fullscreen), `v` toggle manual camera fullscreen.

To send a single ad-hoc message or test camera commands, use `python_sender/sender.py` or `python_sender/senderCamera.py`.

## Architecture

### Data Flow

```
[Vega ECU / broadcaster.py]
        │ TCP port 5000
        ▼
  network.c (tcp_server_task)
        │ calls on_data_received() in main.c
        ▼
  Two frame types:
  ┌────────────────────────────────────────────────────────────┐
  │ Text command (data[0] >= 0x20)                             │
  │   __CAM_START__  → camera_set_fullscreen_manual(true)      │
  │   __CAM_STOP__   → camera_set_fullscreen_manual(false)     │
  ├────────────────────────────────────────────────────────────┤
  │ Binary CAN frame (14 bytes, magic byte 0x01)               │
  │   [0x01][CAN_ID 4B LE][DLC 1B][payload 8B]                │
  │   → display_route_frame() → display_data_t                 │
  │   → camera_set_reverse(drive_mode == 2) on Vega inputs     │
  │   → ui_update_can_data() (skipped while fullscreen)        │
  └────────────────────────────────────────────────────────────┘
```

### Camera / Display Modes

The backup camera is **always visible**: it starts at boot and streams continuously in one of two modes, switched by the stream task at frame boundaries (effective fullscreen = manual command OR reverse gear):

- **PiP (default)** — the PPA scales a centered crop of each sensor frame (full width, trimmed top/bottom — a wide mirror strip) into a private RGB565 buffer displayed by an `lv_canvas` at the bottom-right, above every overlay including the fault overlay. LVGL runs normally and telemetry keeps updating. Only every `CAM_PIP_FRAME_DIV`-th frame is converted (bandwidth). No PPA rotation: LVGL's sw-rotate orients the widget; if the PiP is 180° off on the bench, flip `CAM_PIP_ROTATION`.
- **Fullscreen (reverse / `__CAM_START__`)** — zero-copy: the PPA rotates frames directly into the DPI frame buffer. The stream task **holds the LVGL lock** for the whole fullscreen phase, freezing the dashboard (CAN state still accumulates in `s_can_data`). On exit it invalidates the screen so LVGL repaints over the stale camera frame — no manual `ui_refresh()` needed.

Locking rule: `lvgl_port_lock(0)` means **wait forever**, not try-lock. All `ui_*` entry points therefore take the lock with a short bounded timeout and drop the update if the camera holds it (fullscreen) — this is intentional; dropped updates self-heal on the next frame of the same CAN ID.

### Components

| Component | Role |
|---|---|
| `components/display/` | MIPI DSI hardware init — backlight GPIO 26, LDO ch3 2500mV, JD9365 at 60MHz, returns `esp_lcd_panel_handle_t` |
| `components/ui/` | LVGL dashboard: speed (center, large), SOC bar + pack V/I (right), drive mode + regen (left), 4 bottom tiles (motor temp, ctrl temp, cell spread, solar W), status bar, fault overlay |
| `components/camera/` | OV5647 CSI pipeline: XCLK on GPIO 20, I2C on GPIO 7/8 (addr 0x36), ISP RAW10→RGB565, PPA into the PiP buffer (default) or directly into the DPI frame buffer (fullscreen) |
| `components/network/` | Ethernet static IP via IP101 PHY, TCP server on port 5000, stream accumulator for partial/multi-frame recv() |
| `components/can_spec/` | Header-only wrapper for `display_can_spec.h` (CAN IDs, decode functions, `display_data_t`, `display_route_frame()`) |

### CAN Message Handling

`display_can_spec.h` (in `CAN_STUFF/` and exposed via `can_spec` component) is the authoritative source. It defines:
- All 29-bit extended CAN IDs (BPS, Vega, Kelly MC, BMS relayed, 4× MPPT, Altair GPS)
- Decode structs and inline decode functions for each message
- `display_data_t` — aggregated state with `update_flags` bitmask
- `display_route_frame()` — dispatches by CAN ID, sets the corresponding `DFLAG_*` bit

After calling `display_route_frame()`, `main.c` clears `update_flags` to zero so the next frame starts fresh.

### Key sdkconfig Constraints

- `CONFIG_CAMERA_OV5647=y` — must stay on; enables OV5647 driver from `espressif__esp_cam_sensor`
- `CONFIG_DMA2D_OPERATION_FUNC_IN_IRAM=y` — required for DMA2D performance
- `CONFIG_DMA2D_ISR_IRAM_SAFE` must remain **disabled** — LVGL's `on_job_picked` callback is not in IRAM; enabling this crashes rendering

## Known Issues / Future Work

- **PiP orientation needs bench verification**: the PiP path skips PPA rotation and relies on LVGL's sw-rotate. If the PiP image is 180° off relative to the fullscreen view, change `CAM_PIP_ROTATION` in `camera.c` to `PPA_SRM_ROTATION_ANGLE_180`.
- **PiP frame rate**: `CAM_PIP_FRAME_DIV 2` (~22 fps) is a bandwidth guess; verify dashboard responsiveness on hardware and tune.
- **Fault grace timer display**: `broadcaster.py` scenario 4 has `grace_timer=90s` but the scenario only runs 30s, so the countdown barely moves. Fix: lower grace timer to ≤25 or raise `SCENARIO_DURATION_S` to ≥95.
- **BPS fault codes 14–15** (`BMS_Shutdown`, `VCU_MIA`) exist in the CAN database but are not in the `bps_fault_t` enum — the display shows "UNKNOWN FAULT" for these.
- **Pit telemetry system** (not yet built): needs to decode the full CAN database including MPPT status/sweep/commands, Vega fault, BPS emergency, R_BMS energy/resistance, and all messages not currently in `display_can_spec.h`.
- **CAN-over-TCP protocol for pit system**: same 14-byte framing as the display (`[0x01][CAN_ID 4B LE][DLC 1B][payload 8B]`); Vega forwards all bus traffic.
