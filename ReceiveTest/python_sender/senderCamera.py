"""
ESP32 Camera Sender
Connects to ESP32 and activates camera feed on button press.

Usage:
1. Connect laptop to ESP32 via Ethernet cable
2. Set laptop's IP to 192.168.1.1
3. Run: python senderCamera.py
"""

import socket
import tkinter as tk
from tkinter import ttk
from datetime import datetime

ESP32_IP = "192.168.1.100"
ESP32_PORT = 5000

CMD_CAM_START = "__CAM_START__"
CMD_CAM_STOP = "__CAM_STOP__"


class CameraSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Camera Control")
        self.root.geometry("350x250")

        self.socket = None
        self.connected = False
        self.camera_active = False

        self.create_widgets()

    def create_widgets(self):
        # Connection
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text=f"ESP32: {ESP32_IP}:{ESP32_PORT}").pack(side="left")
        self.connect_btn = ttk.Button(
            conn_frame, text="Connect", command=self.toggle_connection
        )
        self.connect_btn.pack(side="right")
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side="right", padx=10)

        # Camera
        cam_frame = ttk.LabelFrame(self.root, text="Camera", padding=10)
        cam_frame.pack(fill="x", padx=10, pady=5)

        self.cam_btn = ttk.Button(
            cam_frame, text="Start Camera", command=self.toggle_camera
        )
        self.cam_btn.pack(side="left", padx=5)
        self.cam_status = ttk.Label(cam_frame, text="Off", foreground="gray")
        self.cam_status.pack(side="left", padx=10)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.log_text = tk.Text(log_frame, height=4, width=40)
        self.log_text.pack(fill="both", expand=True)

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((ESP32_IP, ESP32_PORT))
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Connected", foreground="green")
            self.log(f"Connected to {ESP32_IP}:{ESP32_PORT}")
        except Exception as e:
            self.log(f"Connection failed: {e}")
            self.socket = None

    def disconnect(self):
        if self.camera_active:
            self.send(CMD_CAM_STOP)
            self.camera_active = False
            self.cam_btn.config(text="Start Camera")
            self.cam_status.config(text="Off", foreground="gray")

        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.log("Disconnected")

    def send(self, data):
        if not self.connected:
            self.log("Not connected!")
            return False
        try:
            self.socket.send(data.encode("utf-8"))
            self.log(f"Sent: {data}")
            return True
        except Exception as e:
            self.log(f"Send failed: {e}")
            self.disconnect()
            return False

    def toggle_camera(self):
        if self.camera_active:
            if self.send(CMD_CAM_STOP):
                self.camera_active = False
                self.cam_btn.config(text="Start Camera")
                self.cam_status.config(text="Off", foreground="gray")
        else:
            if not self.connected:
                self.log("Connect first!")
                return
            if self.send(CMD_CAM_START):
                self.camera_active = True
                self.cam_btn.config(text="Stop Camera")
                self.cam_status.config(text="Live", foreground="green")


if __name__ == "__main__":
    root = tk.Tk()
    app = CameraSenderApp(root)
    root.mainloop()
