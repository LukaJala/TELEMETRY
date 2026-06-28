"""
server.py — Solar Car Ground Station Flask Server

Reads telemetry packets from the RFD900X radio modem via serial port,
parses them, and streams live data to the browser dashboard via SSE.

Run with:
    python server.py

Then open your browser to:
    http://localhost:5000
"""

import threading
import struct
import time
import json
import logging
from collections import deque
from flask import Flask, render_template, Response

# Optional serial import
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARN] pyserial not installed. Running in DEMO mode with fake data.")

# SETTINGS — CHANGE THESE TO MATCH YOUR SETUP

# Your RFD900X serial port.
# Windows:   'COM?'  (check Device Manager)
# Mac:       '/dev/tty.usbserial-XXXXXX'
# Linux:     '/dev/ttyUSB0'
RFD_PORT = 'COM6'
RFD_BAUD = 57600

# Packet format — UPDATE THIS once your team agrees on the structure.
# Current format (20 bytes total):
#   H = magic number   (2 bytes)  — 0xA55A
#   I = sequence       (4 bytes)  — packet counter
#   f = battery_v      (4 bytes)  — volts
#   f = battery_a      (4 bytes)  — amps
#   f = rpm            (4 bytes)  — motor RPM
#   f = temperature    (4 bytes)  — celsius
#   f = speed          (4 bytes)  — km/h
#   H = fault_code     (2 bytes)  — 0 = no fault
#   H = checksum       (2 bytes)
PACKET_FORMAT = '>HI fffff HH'
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)
MAGIC         = 0xA55A

# Alert thresholds — edit these to match your car's safe operating ranges
THRESHOLDS = {
    'battery_v':  {'min': 30.0,  'max': 58.8,  'unit': 'V'},
    'battery_a':  {'min': -5.0,  'max': 60.0,  'unit': 'A'},
    'rpm':        {'min': 0.0,   'max': 5000.0, 'unit': 'RPM'},
    'temperature':{'min': -10.0, 'max': 60.0,  'unit': '°C'},
    'speed':      {'min': 0.0,   'max': 130.0,  'unit': 'km/h'},
}

# Fault code lookup — add your VCU's actual fault codes here
FAULT_CODES = {
    0x0000: 'OK',
    0x0001: 'Battery overvoltage',
    0x0002: 'Battery undervoltage',
    0x0004: 'Motor overtemp',
    0x0008: 'Battery overtemp',
    0x0010: 'Motor overcurrent',
    0x0020: 'Communication loss',
    0x0040: 'BMS fault',
}

# GLOBAL STATE

# Latest data — shared between serial thread and Flask routes
latest = {
    'battery_v':   0.0,
    'battery_a':   0.0,
    'rpm':         0.0,
    'temperature': 0.0,
    'speed':       0.0,
    'fault_code':  0,
    'fault_text':  'OK',
    'seq':         0,
    'packets_received': 0,
    'packets_lost':     0,
    'link_quality':     100.0,
    'last_packet_time': None,
    'alerts':      [],
    'connected':   False,
}

# History for sparkline charts (last 60 readings)
history = {
    'battery_v':   deque(maxlen=60),
    'battery_a':   deque(maxlen=60),
    'rpm':         deque(maxlen=60),
    'temperature': deque(maxlen=60),
    'speed':       deque(maxlen=60),
}

# Lock so the serial thread and Flask don't clash writing to latest
data_lock = threading.Lock()

# SSE subscribers — each connected browser gets an entry
subscribers = []
subscribers_lock = threading.Lock()

# HELPER FUNCTIONS

def check_alerts(data):
    """Return a list of alert strings for any values outside safe range."""
    alerts = []
    for field, limits in THRESHOLDS.items():
        val = data.get(field, 0)
        if val < limits['min']:
            alerts.append(f"{field.replace('_',' ').title()} LOW: {val:.1f}{limits['unit']}")
        elif val > limits['max']:
            alerts.append(f"{field.replace('_',' ').title()} HIGH: {val:.1f}{limits['unit']}")
    return alerts


def decode_fault(code):
    """Return a human-readable fault string."""
    if code == 0:
        return 'OK'
    messages = []
    for bit, text in FAULT_CODES.items():
        if bit != 0 and (code & bit):
            messages.append(text)
    return ' | '.join(messages) if messages else f'Unknown fault: 0x{code:04X}'


def verify_checksum(raw_bytes):
    """Simple sum checksum — must match how the ESP32 builds the packet."""
    calculated = sum(raw_bytes[:-2]) & 0xFFFF
    received   = struct.unpack('>H', raw_bytes[-2:])[0]
    return calculated == received


def broadcast(data_dict):
    """Push a JSON update to every connected browser."""
    message = f"data: {json.dumps(data_dict)}\n\n"
    with subscribers_lock:
        dead = []
        for q in subscribers:
            try:
                q.append(message)
            except Exception:
                dead.append(q)
        for q in dead:
            subscribers.remove(q)


# SERIAL READER THREAD

def serial_reader():
    """
    Runs in a background thread.
    Opens the serial port and continuously reads + parses packets.
    Falls back to demo mode if serial is not available.
    """
    global latest

    if not SERIAL_AVAILABLE:
        _demo_mode()
        return

    while True:
        try:
            print(f"[SERIAL] Connecting to {RFD_PORT} at {RFD_BAUD} baud...")
            ser = serial.Serial(RFD_PORT, baudrate=RFD_BAUD, timeout=1)
            print(f"[SERIAL] Connected.")

            with data_lock:
                latest['connected'] = True

            buffer = b''
            last_seq = None

            while True:
                # Read available bytes
                waiting = ser.in_waiting
                if waiting > 0:
                    buffer += ser.read(waiting)

                # Process every complete packet in the buffer
                while len(buffer) >= PACKET_SIZE:
                    raw = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]

                    # Check magic number
                    magic = struct.unpack('>H', raw[:2])[0]
                    if magic != MAGIC:
                        # Bad sync — discard one byte and try to resync
                        buffer = raw[1:] + buffer
                        break

                    # Verify checksum
                    if not verify_checksum(raw):
                        print("[WARN] Checksum failed — packet discarded")
                        continue

                    # Unpack all fields
                    try:
                        _, seq, batt_v, batt_a, rpm, temp, speed, fault, _ = \
                            struct.unpack(PACKET_FORMAT, raw)
                    except struct.error as e:
                        print(f"[ERROR] Unpack failed: {e}")
                        continue

                    # Calculate packet loss
                    lost = 0
                    if last_seq is not None:
                        expected = last_seq + 1
                        if seq > expected:
                            lost = seq - expected
                    last_seq = seq

                    # Build the data snapshot
                    with data_lock:
                        latest['battery_v']        = round(batt_v, 2)
                        latest['battery_a']        = round(batt_a, 2)
                        latest['rpm']              = round(rpm, 0)
                        latest['temperature']      = round(temp, 1)
                        latest['speed']            = round(speed, 1)
                        latest['fault_code']       = fault
                        latest['fault_text']       = decode_fault(fault)
                        latest['seq']              = seq
                        latest['packets_received'] += 1
                        latest['packets_lost']     += lost
                        total = latest['packets_received'] + latest['packets_lost']
                        latest['link_quality']     = round(
                            (latest['packets_received'] / total) * 100, 1
                        ) if total > 0 else 100.0
                        latest['last_packet_time'] = time.time()
                        latest['alerts']           = check_alerts(latest)
                        latest['connected']        = True

                        # Update history
                        history['battery_v'].append(batt_v)
                        history['battery_a'].append(batt_a)
                        history['rpm'].append(rpm)
                        history['temperature'].append(temp)
                        history['speed'].append(speed)

                        snapshot = dict(latest)
                        snapshot['history'] = {k: list(v) for k, v in history.items()}

                    broadcast(snapshot)

                time.sleep(0.01)

        except Exception as e:
            print(f"[SERIAL] Error: {e} — retrying in 3 seconds...")
            with data_lock:
                latest['connected'] = False
            broadcast({'connected': False, 'alerts': ['Radio link disconnected']})
            time.sleep(3)


def _demo_mode():
    """Generate fake data for testing the dashboard without hardware."""
    import math
    print("[DEMO] Running with simulated data. No hardware required.")
    t = 0
    while True:
        t += 0.1
        with data_lock:
            latest['battery_v']        = round(48.0 + 5 * math.sin(t * 0.3), 2)
            latest['battery_a']        = round(20.0 + 10 * math.sin(t * 0.7), 2)
            latest['rpm']              = round(2500 + 800 * math.sin(t * 0.5), 0)
            latest['temperature']      = round(35.0 + 5 * math.sin(t * 0.2), 1)
            latest['speed']            = round(60.0 + 20 * math.sin(t * 0.4), 1)
            latest['fault_code']       = 0
            latest['fault_text']       = 'OK'
            latest['seq']             += 1
            latest['packets_received'] += 1
            latest['link_quality']     = 98.5
            latest['last_packet_time'] = time.time()
            latest['alerts']           = check_alerts(latest)
            latest['connected']        = True

            history['battery_v'].append(latest['battery_v'])
            history['battery_a'].append(latest['battery_a'])
            history['rpm'].append(latest['rpm'])
            history['temperature'].append(latest['temperature'])
            history['speed'].append(latest['speed'])

            snapshot = dict(latest)
            snapshot['history'] = {k: list(v) for k, v in history.items()}

        broadcast(snapshot)
        time.sleep(0.5)


# FLASK APP

app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)  # suppress Flask request logs


@app.route('/')
def index():
    """Serve the dashboard page."""
    return render_template('dashboard.html')


@app.route('/stream')
def stream():
    """
    Server-Sent Events endpoint.
    The browser connects here once and receives a continuous stream of updates.
    """
    q = deque(maxlen=10)

    with subscribers_lock:
        subscribers.append(q)

    def event_stream():
        # Send current state immediately on connect
        with data_lock:
            snapshot = dict(latest)
            snapshot['history'] = {k: list(v) for k, v in history.items()}
        yield f"data: {json.dumps(snapshot)}\n\n"

        while True:
            if q:
                yield q.popleft()
            else:
                # Send a heartbeat every second so the connection stays alive
                yield ": heartbeat\n\n"	
                time.sleep(0.1)

    return Response(event_stream(), mimetype='text/event-stream')


@app.route('/api/data')
def api_data():
    """REST endpoint — returns current data as JSON (for debugging)."""
    with data_lock:
        snapshot = dict(latest)
        snapshot['history'] = {k: list(v) for k, v in history.items()}
    from flask import jsonify
    return jsonify(snapshot)


# starting the system

if __name__ == '__main__':
    # Start the serial reader in a background thread
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    print("=" * 50)
    print("  Solar Car Ground Station")
    print("  Open your browser to: http://localhost:5000")
    print("=" * 50)

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
