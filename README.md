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
default, reads UWB live from `/dev/ttyUSB0`, and reads IMU data from the Sense
HAT. It records incoming sensor data under `pi/`, but does not replay those
recordings. Use `--lidar-port` to select a different LiDAR serial device.

Before starting the Pi software:

1. In `sudo raspi-config`, open **Interface Options > Serial Port**, disable
   the serial login shell, and enable the serial hardware. On Raspberry Pi 5,
   add `dtoverlay=uart0-pi5` to `/boot/firmware/config.txt` to route UART0 to
   the GPIO header pins 8 and 10; then reboot.
2. Ensure the Pi user can access serial devices. If needed, run
   `sudo usermod -aG dialout "$USER"` and log out and back in.
3. Connect the lidar's UART TX to Pi pin 10 (GPIO15/RXD). If the lidar needs
   a return channel, connect its UART RX to Pi pin 8 (GPIO14/TXD); pin 12 is
   GPIO18, not UART TX. Connect ground to ground and use 3.3 V UART logic.
4. Confirm `/dev/serial0` exists, connect the UWB device over USB as
   `/dev/ttyUSB0`, then run `python pi_launch.py`.

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