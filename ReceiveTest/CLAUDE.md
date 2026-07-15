# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

**ReceiveTest** — Solar car telemetry display + reverse camera for the Lysander vehicle (MSU Solar Racing, FSGP/ASC 2026). Runs on a Waveshare ESP32-P4-Module-DEV-KIT with a 10.1" JD9365 DSI display (800×1280, landscape 1280×800) and a backup camera. The camera has two interchangeable backends behind one `camera.h` API (see [Camera Backends](#camera-backends)): the on-board **OV5647 CSI** sensor (default) or a **USB UVC webcam** (e.g. Logitech C920) on the ESP32-P4 High-Speed USB OTG port.

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

Locking rule: `lvgl_port_lock(0)` means **wait forever**, not try-lock. All `ui_*` entry points therefore take the lock with a short bounded timeout and drop the update if the camera holds it (fullscreen) — this is intentional; dropped updates self-heal on the next frame of the same CAN ID. Exception: `ui_cam_pip_frame_ready()` never touches the lock — it runs in the camera stream task at the one moment no capture is queued, so even a bounded wait drops sensor frames. It just sets a flag; an 8 ms `lv_timer` in LVGL context does the actual invalidate.

### Camera Backends

The camera source is a build-time choice; `main.c` and the UI are backend-agnostic (they only call the `camera.h` API and `ui_cam_pip_*`). Select with `idf.py menuconfig` → **Backup camera** → *Camera backend* (`CONFIG_CAMERA_BACKEND_CSI` default / `CONFIG_CAMERA_BACKEND_UVC`). `components/camera/CMakeLists.txt` compiles exactly one `.c` and pulls only that backend's driver dependencies.

- **`camera_csi.c` (CSI, default)** — the proven OV5647 path. Lowest latency, ~15–30 cm FPC ribbon reach. Unchanged from the original single-backend `camera.c` (renamed).
- **`camera_uvc.c` (UVC)** — a USB webcam (C920) on the P4 **High-Speed** USB OTG. Requests MJPEG at `CONFIG_CAMERA_UVC_H_RES`×`_V_RES`@`_FPS` (default 1280×720@30 — 1280 keeps the 480-wide PiP on the PPA 1/16 grid), decodes each frame in the hardware JPEG engine to RGB565, then reuses the same PPA scale/rotate + PiP/fullscreen + LVGL-lock logic as the CSI path. Requires `espressif/usb_host_uvc` (fetched via `main/idf_component.yml`, only compiled when this backend is selected).

  UVC bench bring-up (see the header comment in `camera_uvc.c`): the host port must **supply VBUS** (~500 mA for a C920 — use a powered hub if it browns out); leave `CONFIG_CAMERA_UVC_VID/PID` at 0 first and read the real IDs from the enumeration log before pinning; flip `CAM_UVC_PPA_ROTATION`/`MIRROR_*` for orientation (a rear-view feel usually wants `MIRROR_X`); flip `CAM_UVC_JPEG_RGB_ORDER` (RGB↔BGR) if red/blue are swapped. **Bench-verified working** (July 2026) with a C920 at 640×480@30 — but only with the **local patches to `managed_components/espressif__usb_host_uvc/uvc_isoc.c`** (see [Patched usb_host_uvc component](#patched-usb_host_uvc-component)). An occasional `JPEG decode failed` (~1 per 20 s) is a truncated frame from lossy isochronous transfer — the app drops it and continues; harmless.

### Components

| Component | Role |
|---|---|
| `components/display/` | MIPI DSI hardware init — backlight GPIO 26, LDO ch3 2500mV, JD9365 at 60MHz, returns `esp_lcd_panel_handle_t` |
| `components/ui/` | LVGL dashboard: speed (center, large), SOC bar + pack V/I (right), drive mode + regen (left), 4 bottom tiles (motor temp, ctrl temp, cell spread, solar W), status bar, fault overlay |
| `components/camera/` | Backup camera behind `camera.h`. Two backends, one compiled (Kconfig `CAMERA_BACKEND_*`): **`camera_csi.c`** (default) — OV5647 CSI, XCLK GPIO 20, I2C GPIO 7/8 (0x36), ISP RAW10→RGB565; **`camera_uvc.c`** — USB UVC webcam on HS OTG, MJPEG→RGB565 via the P4 hardware JPEG decoder. Both feed the same PPA→PiP/fullscreen output path. |
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

- `CONFIG_CAMERA_OV5647=y` — must stay on for the **CSI backend**; enables OV5647 driver from `espressif__esp_cam_sensor` (irrelevant when `CAMERA_BACKEND_UVC` is selected)
- `CONFIG_DMA2D_OPERATION_FUNC_IN_IRAM=y` — required for DMA2D performance
- `CONFIG_DMA2D_ISR_IRAM_SAFE` must remain **disabled** — LVGL's `on_job_picked` callback is not in IRAM; enabling this crashes rendering
- **UVC backend only**: the USB Host Library must run on the P4 **High-Speed** OTG for a webcam to have enough bandwidth. The C920's large config descriptor can need a bigger control-transfer buffer (`CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE`) if enumeration fails.

### Patched usb_host_uvc component

`managed_components/espressif__usb_host_uvc/uvc_isoc.c` (v2.5.1) carries **two local bug fixes without which the UVC backend shows a black screen** — bench-proven with a C920 (isoc, MJPEG). Both are upstream bugs in `isoc_transfer_callback()`; the failure modes were confirmed with on-wire packet statistics:

1. **EoF-in-header-only-payload swallowed**: the C920 signals End of Frame in a header-only payload (zero data bytes) on *every* frame. Upstream skips zero-data payloads *before* checking the EoF bit, so no frame ever completes → `missed EoF` warning every 33 ms. Fix: fall through to the EoF handling when a header-only payload has the EoF bit set.
2. **Empty ISOC packets poison every frame**: ~6000+ of ~8000 microframes/s are empty (`actual_num_bytes == 0`) — normal isoc idling. The payload-header validation added upstream in 2.4.1 rejects them and sets `skip_current_frame`, silently discarding every frame (no warning — the reject log is debug-level). Fix: skip empty packets before header validation without flagging the frame.

Maintenance rules: `idf.py fullclean` is **safe** (it only clears `build/`; the component's `CHECKSUMS.json` was updated to match the patch, so reconfigure keeps it). The patch is **lost** if the component is re-downloaded — i.e. if `managed_components/espressif__usb_host_uvc/` is deleted or the version constraint changes. After editing any file in a managed component, refresh its entry in that component's `CHECKSUMS.json` (sha256 + size), or `idf.py` refuses to build. Should be reported upstream to espressif/esp-usb (their PR #340 / v2.4.1 introduced the second bug and exposed the first).

## Known Issues / Future Work

- **usb_host_uvc patches not upstreamed**: the two `uvc_isoc.c` fixes above should be filed against espressif/esp-usb so a future component update doesn't regress the camera.
- **PiP orientation needs bench verification**: the PiP path skips PPA rotation and relies on LVGL's sw-rotate. If the PiP image is 180° off relative to the fullscreen view, change `CAM_PIP_ROTATION` (in `camera_csi.c`, or `CAM_UVC_PIP_ROTATION` in `camera_uvc.c`) to `PPA_SRM_ROTATION_ANGLE_180`.
- **PiP frame rate**: `CAM_PIP_FRAME_DIV 1` (~45 fps) with LVGL refreshing at 60Hz (`CONFIG_LV_DEF_REFR_PERIOD=16`); if the dashboard gets sluggish on hardware, raise the divider to 2 (~22 fps) first. If dashboard + PiP still contend, the untested escape hatch is `CONFIG_LVGL_PORT_ENABLE_PPA=y` (hardware-rotated flush; experimental, shares the PPA with the camera).
- **Fault grace timer display**: `broadcaster.py` scenario 4 has `grace_timer=90s` but the scenario only runs 30s, so the countdown barely moves. Fix: lower grace timer to ≤25 or raise `SCENARIO_DURATION_S` to ≥95.
- **BPS fault codes 14–15** (`BMS_Shutdown`, `VCU_MIA`) exist in the CAN database but are not in the `bps_fault_t` enum — the display shows "UNKNOWN FAULT" for these.
- **Pit telemetry system** (not yet built): needs to decode the full CAN database including MPPT status/sweep/commands, Vega fault, BPS emergency, R_BMS energy/resistance, and all messages not currently in `display_can_spec.h`.
- **CAN-over-TCP protocol for pit system**: same 14-byte framing as the display (`[0x01][CAN_ID 4B LE][DLC 1B][payload 8B]`); Vega forwards all bus traffic.
