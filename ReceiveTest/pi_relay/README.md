# Pi telemetry relay

`relay.py` runs on the Raspberry Pi Zero 2 W inside the car. It accepts the
ESP32-P4's TCP telemetry stream and forwards each validated packet out the
RFD900X radio (UART `/dev/serial0`, 57600 baud) to the laptop.

```
ESP32-P4  --(TCP 192.168.77.x:5000)-->  Pi Zero 2W  --(UART/RFD900X)-->  laptop
```

## Install as a service (auto-start on boot, auto-restart on crash)

`relay.service` is the systemd unit. It keeps the relay running across reboots
and restarts it within 2 s if it ever exits.

1. **Put the code where the unit expects it** (or edit the three marked lines in
   `relay.service` to point at wherever you keep it). Defaults assume
   `/home/pi/relay/relay.py` running as user `pi`:
   ```bash
   mkdir -p /home/pi/relay
   cp relay.py /home/pi/relay/
   ```

2. **Dependencies** — relay.py needs pyserial, and the user needs serial access:
   ```bash
   sudo apt install -y python3-serial
   sudo usermod -aG dialout pi          # one-time; lets relay open /dev/serial0
   ```

3. **Make sure the UART is free for the RFD900X.** On Raspberry Pi OS, disable
   the serial login console but keep the hardware UART enabled:
   ```bash
   sudo raspi-config nonint do_serial_hw 0      # enable serial port hardware
   sudo raspi-config nonint do_serial_cons 1    # disable serial login shell
   ```
   (Equivalent to: Interface Options -> Serial Port -> login shell **No**,
   hardware **Yes**. Reboot afterwards.)

4. **Install and start the service:**
   ```bash
   sudo cp relay.service /etc/systemd/system/relay.service
   sudo systemctl daemon-reload
   sudo systemctl enable --now relay        # start now + on every boot
   ```

## Operating it

```bash
systemctl status relay        # is it running?
journalctl -u relay -f        # live logs (connects, bad checksums, disconnects)
sudo systemctl restart relay  # after editing relay.py
sudo systemctl stop relay     # stop it (e.g. to run relay.py by hand)
```

You should see `[RELAY] ESP32 connected from ('192.168.77.1', ...)` once the
board links up. If the board reboots (e.g. restarting `idf.py monitor`), the
keepalive in relay.py drops the dead socket within ~5 s and the board
reconnects automatically — no manual restart needed.
