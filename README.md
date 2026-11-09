# UWB, IMU, and lidar over UDP

Install the requirements for the relevant device from the repository root.

## Raspberry Pi

```powershell
python -m pip install -r pi/requirements.txt
python pi_launch.py
```

The launcher opens separate terminal windows for
`pi/pi_sensors.py` and `pi/pi_control_monitor.py`. The control monitor listens
for UDP control packets on `0.0.0.0:5007` by default; override this with its
`--host` and `--port` options when running it directly.

The sensor process reads the lidar live from `/dev/serial0` at 230400 baud by
default and records the received bytes in `pi/lidar_capture.bin`. UWB and IMU
remain in their existing playback mode unless live mode is enabled in
`pi/pi_sensors.py`. To replay a lidar capture instead, run
`python pi/pi_sensors.py --replay-lidar`. Use `--lidar-port` to select a
different serial device.

Before starting the Pi software:

1. In `sudo raspi-config`, open **Interface Options > Serial Port**, disable
   the serial login shell, enable the serial hardware, and reboot.
2. Ensure the Pi user can access serial devices. If needed, run
   `sudo usermod -aG dialout "$USER"` and log out and back in.
3. Connect the lidar's UART TX to Pi pin 10 (GPIO15/RXD), UART RX to Pi pin 8
   (GPIO14/TXD) if the lidar needs a return channel, and connect ground to
   ground. Use 3.3 V UART logic; do not apply 5 V logic to the Pi GPIO pins.
4. Confirm `/dev/serial0` exists, then run `python pi_launch.py`.

## Windows PC

```powershell
python -m pip install -r pc/requirements.txt
python pc_launch.py --pi-ip 192.168.15.16
```

The PC display sends control packets over UDP to the Pi's sensor receiver on
port `5006` and the control monitor on port `5007`. The default Pi address is
`192.168.15.16` (the `PI_IP` setting in `common.py`); override it with
`--pi-ip` or update `PI_IP`. If changing the monitor port, keep the monitor's
`--port` value and `CONTROL_MONITOR_PORT` in `common.py` in sync.
PC senders first complete a HELLO/ACK handshake with the monitor on port `5007`;
control packets are not sent until the Pi confirms the connection. Senders
retry the handshake automatically if the connection is lost.

Close any other PC display that is already using UDP port `5005`. Allow inbound
UDP traffic on that port in the PC firewall. Set `PC_IP` in `common.py` or pass
`--pc-ip` on the Pi to match the PC's address.