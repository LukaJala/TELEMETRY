"""
relay.py — Solar Car Telemetry Relay (Raspberry Pi Zero 2 W)

Runs on the Pi inside the car. Receives decoded telemetry packets from the
ESP32-P4 over Ethernet and forwards each valid packet out the RFD900x radio.

Data flow:
    ESP32 --(TCP 192.168.77.2:5000)--> Pi --(UART /dev/serial0)--> RFD900x ))) laptop server.py

Packet format MUST match build_packet() in network.c and the laptop's server.py:
    '>H I f f f f f H H' =
    magic(0xA55A) seq battery_v battery_a rpm temperature speed fault checksum
    big-endian; checksum = sum(bytes[0..27]) & 0xFFFF

Network setup (one-time, on the Pi):
    The ESP32 native port is 192.168.77.1; this Pi's USB-Ethernet dongle must be
    192.168.77.2/24 (no gateway — keep internet on wlan0):
        sudo nmcli con add type ethernet con-name pi-eth ifname eth0
        sudo nmcli con mod pi-eth ipv4.method manual ipv4.addresses 192.168.77.2/24
        sudo nmcli con up pi-eth

UART setup (one-time, on the Pi):
    /dev/serial0 must map to the PL011 (ttyAMA0), not the mini-UART. In
    /boot/firmware/config.txt add:  enable_uart=1  and  dtoverlay=disable-bt
    then disable the serial login console (raspi-config) and reboot.

Runs automatically on boot via /etc/systemd/system/telemetry.service.
"""

import socket
import struct
import serial
import time

# ============================================================
# Settings
# ============================================================
LISTEN_HOST = '0.0.0.0'   # accept the ESP32 from any interface
LISTEN_PORT = 5000        # the ESP32 connects here

RFD_PORT = '/dev/serial0'  # CHANGED: was '/dev/ttyAMA0' — use the stable symlink (-> ttyAMA0 after disable-bt)
RFD_BAUD = 57600           # must match the radio's configured serial baud

# Must match build_packet() in network.c and server.py
PACKET_FORMAT = '>HIfffffHH'
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)   # 30 bytes
MAGIC         = 0xA55A


# ============================================================
# Checksum (matches the ESP32's sum-of-bytes scheme)
# ============================================================
def verify_checksum(raw):
    calculated = sum(raw[:-2]) & 0xFFFF
    received   = struct.unpack('>H', raw[-2:])[0]
    return calculated == received


# ============================================================
# Relay loop — drains one client connection, forwards to radio
# ============================================================
# CHANGED: signature was relay_loop(conn) using a module-global `rfd`;
#          `rfd` is now passed in explicitly instead of being a global.
def relay_loop(conn, rfd):
    buffer = b''   # CHANGED: this init was missing/misplaced in the on-Pi version —
                   #          its absence caused "cannot access local variable 'buffer'"
    while True:
        chunk = conn.recv(256)
        if not chunk:
            print("[RELAY] ESP32 disconnected.")
            break
        buffer += chunk

        # Process every complete packet at the front of the buffer
        while len(buffer) >= PACKET_SIZE:
            magic = struct.unpack('>H', buffer[:2])[0]
            if magic != MAGIC:
                buffer = buffer[1:]   # not aligned — resync one byte at a time
                continue

            raw = buffer[:PACKET_SIZE]
            buffer = buffer[PACKET_SIZE:]

            if not verify_checksum(raw):
                print("[RELAY] Bad checksum — packet dropped.")
                continue

            # Forward the validated packet over the radio
            rfd.write(raw)

            # Debug print
            try:
                _, seq, bv, ba, rpm, temp, spd, fault, _ = struct.unpack(PACKET_FORMAT, raw)
                print(f"[PKT #{seq}] {bv:.1f}V {ba:.1f}A {rpm:.0f}RPM "
                      f"{temp:.1f}C {spd:.1f}km/h fault={fault}")
            except struct.error:
                pass


# ============================================================
# Entry point
# ============================================================
# CHANGED: the radio-open and TCP-server setup used to run at module top level.
#          Wrapped in main() so the radio opens BEFORE the server binds, and so
#          an import never starts the server by accident.
def main():
    print(f"[RELAY] Opening radio on {RFD_PORT} at {RFD_BAUD} baud...")
    rfd = serial.Serial(RFD_PORT, baudrate=RFD_BAUD, timeout=1)
    print("[RELAY] Radio ready.")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((LISTEN_HOST, LISTEN_PORT))
    server.listen(1)
    print(f"[RELAY] TCP server on port {LISTEN_PORT}. Waiting for ESP32...")

    while True:
        conn = None   # CHANGED: init before the try so the finally below can't hit
                      #          a NameError on conn if accept() itself fails
        try:
            conn, addr = server.accept()
            print(f"[RELAY] ESP32 connected from {addr}")
            relay_loop(conn, rfd)   # CHANGED: pass rfd explicitly (was a global)
        except Exception as e:
            print(f"[RELAY] Error: {e}")
        finally:
            if conn is not None:   # CHANGED: guard the close (conn may be None)
                conn.close()
            print("[RELAY] Waiting for ESP32 to reconnect...")
            time.sleep(1)


if __name__ == '__main__':
    main()
