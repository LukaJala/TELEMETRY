"""
ESP32 Data Sender
Sends text, numbers, or time to ESP32 display via TCP

By the goat himself

Usage:
1. Connect laptop to ESP32 via Ethernet cable
2. Set laptop's IP to 192.168.1.1 (see instructions below)
3. Run this script: python sender.py
"""

import socket
import time
import tkinter as tk
from tkinter import ttk
from datetime import datetime
import threading

# ============================================================
# CONFIGURATION - Must match ESP32 settings
# ============================================================
ESP32_IP = "192.168.1.100"
ESP32_PORT = 5000

# ============================================================
# MAIN APPLICATION CLASS
# ============================================================
class SenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Data Sender")
        self.root.geometry("400x350")

        self.socket = None
        self.connected = False
        self.sending_time = False

        self.create_widgets()

    def create_widgets(self):
        # --------------------------------------------------------
        # Connection Frame
        # --------------------------------------------------------
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text=f"ESP32: {ESP32_IP}:{ESP32_PORT}").pack(side="left")

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="right")

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side="right", padx=10)

        # --------------------------------------------------------
        # Send Text Frame
        # --------------------------------------------------------
        text_frame = ttk.LabelFrame(self.root, text="Send Text/Number", padding=10)
        text_frame.pack(fill="x", padx=10, pady=5)

        self.text_entry = ttk.Entry(text_frame, width=30)
        self.text_entry.pack(side="left", padx=5)
        self.text_entry.bind("<Return>", lambda e: self.send_text())  # Enter key sends

        ttk.Button(text_frame, text="Send", command=self.send_text).pack(side="left", padx=5)

        # --------------------------------------------------------
        # Send Time Frame
        # --------------------------------------------------------
        time_frame = ttk.LabelFrame(self.root, text="Send Time", padding=10)
        time_frame.pack(fill="x", padx=10, pady=5)

        self.time_btn = ttk.Button(time_frame, text="Start Sending Time", command=self.toggle_time)
        self.time_btn.pack(side="left", padx=5)

        self.time_label = ttk.Label(time_frame, text="--:--:--")
        self.time_label.pack(side="left", padx=10)

        # --------------------------------------------------------
        # Quick Buttons Frame
        # --------------------------------------------------------
        quick_frame = ttk.LabelFrame(self.root, text="Quick Send", padding=10)
        quick_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(quick_frame, text="Solar", command=lambda: self.send("This is MSU Solar Car!")).pack(side="left", padx=2)
        ttk.Button(quick_frame, text="67", command=lambda: self.send("67")).pack(side="left", padx=2)
        ttk.Button(quick_frame, text="Who's GOATed?", command=lambda: self.send("The Telly team")).pack(side="left", padx=2)
        ttk.Button(quick_frame, text="Clear", command=lambda: self.send("---")).pack(side="left", padx=2)

        # --------------------------------------------------------
        # Log Frame
        # --------------------------------------------------------
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, height=6, width=40)
        self.log_text.pack(fill="both", expand=True)

    def log(self, message):
        """Add message to log window"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")  # Auto-scroll to bottom

    def toggle_connection(self):
        """Connect or disconnect from ESP32"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        """Establish TCP connection to ESP32"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)  # 5 second timeout
            self.socket.connect((ESP32_IP, ESP32_PORT))

            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Connected", foreground="green")
            self.log(f"Connected to {ESP32_IP}:{ESP32_PORT}")

        except Exception as e:
            self.log(f"Connection failed: {e}")
            self.socket = None

    def disconnect(self):
        """Close TCP connection"""
        self.sending_time = False

        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.time_btn.config(text="Start Sending Time")
        self.log("Disconnected")

    def send(self, data):
        """Send data to ESP32"""
        if not self.connected:
            self.log("Not connected!")
            return False

        try:
            self.socket.send(data.encode('utf-8'))
            self.log(f"Sent: {data}")
            return True
        except Exception as e:
            self.log(f"Send failed: {e}")
            self.disconnect()
            return False

    def send_text(self):
        """Send text from entry box"""
        text = self.text_entry.get()
        if text:
            self.send(text)
            self.text_entry.delete(0, "end")  # Clear entry after sending

    def toggle_time(self):
        """Start or stop sending time"""
        if self.sending_time:
            self.sending_time = False
            self.time_btn.config(text="Start Sending Time")
        else:
            if not self.connected:
                self.log("Connect first!")
                return
            self.sending_time = True
            self.time_btn.config(text="Stop Sending Time")
            # Start time sending in background thread
            threading.Thread(target=self.time_sender_thread, daemon=True).start()

    def time_sender_thread(self):
        """Background thread that sends time every second"""
        while self.sending_time and self.connected:
            current_time = datetime.now().strftime("%H:%M:%S")

            # Update label in UI (must use after() for thread safety)
            self.root.after(0, lambda t=current_time: self.time_label.config(text=t))

            if not self.send(current_time):
                self.sending_time = False
                break

            time.sleep(1)

        self.root.after(0, lambda: self.time_btn.config(text="Start Sending Time"))

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    print("=" * 50)
    print("ESP32 Data Sender")
    print("=" * 50)
    print(f"Will connect to: {ESP32_IP}:{ESP32_PORT}")
    print()
    print("IMPORTANT: Set your device Ethernet IP to 192.168.1.1")
    print("  Windows: Network Settings > Ethernet > IP Settings > Manual")
    print("  Set IP: 192.168.1.1, Subnet: 255.255.255.0")
    print("=" * 50)
    print()

    root = tk.Tk()
    app = SenderApp(root)
    root.mainloop()
