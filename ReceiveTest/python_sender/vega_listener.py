"""
vega_listener.py — Bench test: is Vega's W5500 actually sending anything?

Vega connects as a TCP CLIENT to 192.168.2.100:5000 (hardcoded in its app.cpp).
This script makes your laptop that server, so you can plug Vega straight into the
laptop's Ethernet port and watch what comes across — no ESP32 in the loop.

It prints every parsed CAN frame, hex-dumps anything that doesn't parse, and
prints a once-a-second heartbeat so "connected but silent" is obvious.

────────────────────────────────────────────────────────────────────────────
SETUP (do this first, or Vega can't reach you):
  1. Set your laptop's Ethernet adapter to a STATIC IP:
         IP:      192.168.2.100      <- must match Vega's serverIp
         Mask:    255.255.255.0
         Gateway: (leave blank)
     (Vega is 192.168.2.50 on the same subnet.)
  2. Plug Vega's W5500 into the laptop's Ethernet port (a straight cable is fine;
     modern laptop NICs auto-MDIX). Wait for a link light.
  3. Run:  python vega_listener.py
  4. Power/boot Vega. You should see "Vega connected from ('192.168.2.50', ...)".

The frame format expected (same as broadcaster.py / the ESP32):
    [0x01 magic][CAN_ID 4B little-endian][DLC 1B][payload 8B]  = 14 bytes
────────────────────────────────────────────────────────────────────────────
"""

import socket
import struct
import time

# ── Config ──────────────────────────────────────────────────────────────────
LISTEN_HOST = "0.0.0.0"   # accept on every interface (so the adapter IP doesn't matter)
LISTEN_PORT = 5000
FRAME_SIZE = 14
FRAME_MAGIC = 0x01
RAW_DUMP = False          # set True to hex-dump every raw chunk (ground truth, verbose)

# Friendly names for the IDs the display/Vega use (from display_can_spec.h).
CAN_NAMES = {
    0x08FF1000: "BPS_SAFETY",
    0x0CFF2000: "BPS_POWER",
    0x14FF3300: "BPS_SUPPL",
    0x14FF3600: "BPS_CHARGING",
    0x0CFF211E: "VEGA_INPUTS",
    0x18FF101E: "VEGA_STATUS",
    0x0CF11E05: "MC_STATUS1",
    0x0CF11F05: "MC_STATUS2",
    0x14FF3200: "CELL_EXTREMES",
    0x14FF4100: "TEMP_EXTREMES",
    0x00007800: "MPPT1",
    0x00007900: "MPPT2",
    0x00007A00: "MPPT3",
    0x00007B00: "MPPT4",
    0x14FF0150: "GPS_POS",
    0x14FF0250: "GPS_NAV",
    0x18FF0350: "GPS_TIME",
}


def _ts():
    return time.strftime("%H:%M:%S", time.localtime()) + f".{int((time.time() % 1) * 1000):03d}"


def _hex(b):
    return " ".join(f"{x:02X}" for x in b)


def decode_extra(can_id, payload):
    """Decode a couple of fields so the numbers are sanity-checkable on the bench."""
    if can_id == 0x18FF101E and len(payload) >= 2:  # VEGA_STATUS: speed in 0.1 km/h LE
        speed_kmh = (payload[0] | (payload[1] << 8)) * 0.1
        return f"speed={speed_kmh:.1f} km/h ({speed_kmh * 0.621371:.1f} mph)"
    return ""


def handle_client(conn, addr):
    print(f"\n[{_ts()}] *** Vega connected from {addr} ***\n")
    conn.settimeout(1.0)  # so the heartbeat prints even when Vega is silent

    buf = bytearray()
    total_frames = 0
    total_bytes = 0
    window_frames = 0
    last_report = time.time()
    last_rx = time.time()

    while True:
        try:
            chunk = conn.recv(4096)
            if not chunk:
                print(f"[{_ts()}] *** Vega disconnected (clean close) ***")
                return
            total_bytes += len(chunk)
            last_rx = time.time()
            if RAW_DUMP:
                print(f"[{_ts()}] RAW {len(chunk):3d}B: {_hex(chunk)}")
            buf += chunk
        except socket.timeout:
            chunk = None  # fall through to heartbeat
        except (ConnectionResetError, OSError) as e:
            print(f"[{_ts()}] *** connection error: {e} ***")
            return

        # Parse complete units from the front of the buffer.
        while buf:
            if buf[0] == FRAME_MAGIC:
                if len(buf) < FRAME_SIZE:
                    break  # wait for the rest of this frame
                frame = bytes(buf[:FRAME_SIZE])
                del buf[:FRAME_SIZE]

                can_id = struct.unpack("<I", frame[1:5])[0]
                dlc = frame[5]
                payload = frame[6:14]
                name = CAN_NAMES.get(can_id, "?")
                extra = decode_extra(can_id, payload)
                print(f"[{_ts()}] ID 0x{can_id:08X} {name:<13} DLC {dlc} "
                      f"data {_hex(payload)}   {extra}")
                total_frames += 1
                window_frames += 1

            elif buf[0] >= 0x20:
                # Printable: maybe a text command (e.g. __CAM_START__). Wait for NUL.
                nul = buf.find(b"\x00")
                if nul == -1:
                    if len(buf) > 256:
                        print(f"[{_ts()}] !! 256B of printable junk, no NUL — flushing")
                        buf.clear()
                    break
                print(f"[{_ts()}] TEXT: {bytes(buf[:nul]).decode('ascii', 'replace')!r}")
                del buf[:nul + 1]

            else:
                # Not a frame and not text — misalignment / garbage. Show it, resync.
                print(f"[{_ts()}] !! unknown lead byte 0x{buf[0]:02X} — dropping. "
                      f"head: {_hex(buf[:min(len(buf), FRAME_SIZE)])}")
                del buf[0]

        # ── Heartbeat / stats once per second ──
        now = time.time()
        if now - last_report >= 1.0:
            silent = now - last_rx
            if total_frames == 0 and total_bytes == 0:
                print(f"[{_ts()}] ...connected but NOTHING received yet "
                      f"(silent {silent:.0f}s)")
            else:
                print(f"[{_ts()}] [stats] {window_frames}/s | "
                      f"{total_frames} frames, {total_bytes} bytes total | "
                      f"buffered {len(buf)}B"
                      + (f" | silent {silent:.0f}s" if silent > 1.5 else ""))
            window_frames = 0
            last_report = now


def main():
    print("=" * 70)
    print("  Vega W5500 listener — laptop acts as the TCP server Vega dials into")
    print(f"  Listening on {LISTEN_HOST}:{LISTEN_PORT}")
    print("  >>> Set this laptop's Ethernet adapter to 192.168.2.100 / 255.255.255.0")
    print("  >>> Vega should be 192.168.2.50 and connect to 192.168.2.100:5000")
    print("  Ctrl+C to quit.")
    print("=" * 70)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((LISTEN_HOST, LISTEN_PORT))
    srv.listen(1)

    try:
        while True:
            print(f"\n[{_ts()}] Waiting for Vega to connect...")
            conn, addr = srv.accept()
            try:
                handle_client(conn, addr)
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
