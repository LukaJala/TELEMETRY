"""
relay.py - Solar Car Telemetry Relay (Pi Zero 2W version)
Runs on the raspberry PI Zero 2W inside the car.

Data flow:
        ESP32 --(TCPI/IP over ethernet)>   PI Zero --(UART)>  RFD900X -->  laptop 
This script:
        1. opens a TCP server and waits for the ESP32 to connect.
        2. Receives telemetry packets from the ESP32.
        3. Validates each packet (magic number + sum)
        4. Forwards valid packets over the RFD900x radio via UART serial port.
Designed to run automatically on boot via system.
"""
import socket
import struct
import serial
import time

# Setting

# TCP server - the ESP32 connects to this
LISTEN_HOST = '0.0.0.0'         # accepst any connection from any IP
LISTEN_PORT = 5000              # ESP32 must connect to this port

# RFD900X radio on the PI zero GPIO UART PINS
RFD_PORT = '/dev/serial0' # UART pins and not USB
RFD_BAUD = 57600

# PACKET format - MUST match the ESP32 and the laptop server.py
# > big -endian
# H magic (0xA55A)
# I sequence 
# f Battery_v
# f battery_a
# f rpm
# f temperature
# f speed
# H fault_code
# H checksum
PACKET_FORMAT = '>HIfffffHH'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
MAGIC   = 0xA55A

# CHECKSUM

def verify_checksum(raw):
        """sum checks of all bytes except the final 2 cheksum bytes. """
        calculated = sum(raw[:-2]) & 0xFFFF
        received = struct.unpack('>H', raw[-2:])[0]
        return calculated == received

# DEAD-PEER DETECTION

def enable_keepalive(conn):
        """Detect a silently-dead ESP32 and recover.

        When the board reboots (e.g. restarting `idf.py monitor` pulses the
        auto-reset line) it drops its TCP socket WITHOUT a clean FIN/RST. The
        recv() in relay_loop would then block forever on that zombie connection,
        so we'd never return to accept() and the board could never reconnect
        (it sees ECONNRESET/ETIMEDOUT). Linux TCP keepalive tears the dead
        socket down in ~5s, which makes recv() raise and kicks us back to
        accept() to take the board's new connection."""
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 2)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)


# OPEN THE RADIO

def relay_loop(conn, rfd):
        buffer = b''
        while True:
                chunk = conn.recv(256)
                if not chunk:
                        print("[RELAY] ESP32 disconnected")
                        break
                buffer += chunk

                # Heartbeat: echo a single byte for every chunk we receive. The
                # ESP32 watches for this and reconnects fast if it stops (Pi power
                # loss / relay crash leave no clean FIN/RST, so the board can't
                # tell otherwise until the slow TCP retransmit timeout). Sent
                # non-blocking and best-effort: if the board isn't draining it
                # (e.g. older firmware) we drop the byte instead of letting it
                # back up and block the relay. It never goes over the radio.
                try:
                        conn.send(b'\x00', socket.MSG_DONTWAIT)
                except (BlockingIOError, OSError):
                        pass

                # Pull every complete, valid packet out of the buffer. If several
                # have piled up because the radio is falling behind at long range,
                # keep only the NEWEST — stale telemetry is useless, and forwarding
                # the whole backlog is exactly what lets a slow radio wedge the relay.
                latest = None
                while len(buffer) >= PACKET_SIZE:
                        #check for magic at the start
                        magic = struct.unpack('>H', buffer[:2])[0]
                        if magic != MAGIC:
                                # not aligned - drop one byte and resync
                                buffer = buffer[1:]
                                continue
                        raw = buffer[:PACKET_SIZE]
                        buffer = buffer[PACKET_SIZE:]

                        if not verify_checksum(raw):
                                print(f"[RELAY] Bad Checksum - paket dropped.")
                                continue
                        latest = raw

                # Forward the freshest packet. The bounded write_timeout (set when
                # the port is opened) means a saturated/blocked radio raises instead
                # of blocking forever — we drop the frame and keep draining the
                # socket, so the board's TCP connection never backs up and wedges.
                if latest is not None:
                        try:
                                rfd.write(latest)
                        except serial.SerialTimeoutException:
                                print("[RELAY] Radio write timed out - frame dropped (link saturated / far away?)")

#TCP SERVER - wait for the ESP32, relay, reconnect on the drop
def main():
        print(f"[RELAY] Opening Radio on {RFD_PORT} at {RFD_BAUD} baud  .... ")
        # write_timeout bounds rfd.write() so a backed-up radio (serial flow
        # control at long range) can't block the relay forever and wedge the
        # board's TCP connection. On timeout pyserial raises SerialTimeoutException,
        # which relay_loop catches and drops the frame.
        rfd = serial.Serial(RFD_PORT, baudrate=RFD_BAUD, timeout=1, write_timeout=0.5)
        print("[RELAY] Radio Ready.")

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((LISTEN_HOST, LISTEN_PORT))
        server.listen(5)  # backlog >1 so a reconnect SYN isn't dropped while an old conn is torn down
        print("[RELAY] Waiting for the ESP32 to connect ...")

        while True:
                conn =None
                try:
                        conn, addr = server.accept()
                        enable_keepalive(conn)
                        print(f"[RELAY] ESP32 connected from {addr}")
                        relay_loop(conn, rfd)
                except Exception as e:
                        print(f"[RELAY] Error: {e}")

                finally:
                        if conn is not None:
                                conn.close()
                        print("[RELAY] Waiting for ESP32 to reconnect...")
                        time.sleep(1)
if __name__ == '__main__':
        main()