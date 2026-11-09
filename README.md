# UWB, IMU, and lidar over UDP

Install the requirements for the relevant device from the repository root.

## Raspberry Pi

```powershell
python -m pip install -r pi/requirements.txt
$PcIp = "192.168.15.14" # replace with the PC address reachable from the Pi
python pi_launch.py --pc-ip $PcIp
```

`pi_launch.py` runs the sensor sender and control monitor together in the
current terminal (including over SSH); use Ctrl+C to stop both. Sensor and
control-monitor output appears in the same session. `--pc-ip` must be the
Windows PC's current address on the network shared with the Pi.
The monitor listens for control packets on `0.0.0.0:5007`.

The legacy `Threaded_UWB_IMU_and_Lidar_with_file_reader.py` displays plots on
the machine where it runs; it does not transmit sensor telemetry to the PC
dashboard. Use `pi_launch.py` for remote telemetry and control monitoring.

The sensor process reads the lidar live from `/dev/serial0` at 230400 baud by
default, reads UWB live from `/dev/ttyUSB0`, and reads IMU data from the Sense
HAT. It records incoming sensor data under `pi/`, but does not replay those
recordings. Use `--lidar-port` or `--data-dir` on `pi_launch.py` to override
the LiDAR serial device or recording directory.

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
   `/dev/ttyUSB0`, then run `python pi_launch.py --pc-ip $PcIp`.

## Windows PC

```powershell
python -m pip install -r pc/requirements.txt
python pc_launch.py --pi-ip 192.168.137.115
```

The PC display sends control packets over UDP to the Pi's sensor receiver on
port `5006` and the control monitor on port `5007`. The default Pi address is
the `PI_IP` setting in `common.py`; override it with `--pi-ip` to match the
Pi's current address. Both machines must be on mutually reachable networks:
the PC must be able to reach the Pi, and the Pi must be able to send UDP
telemetry to the PC address passed to `pi_launch.py --pc-ip`.
PC senders first complete a HELLO/ACK handshake with the monitor on port `5007`;
control packets are not sent until the Pi confirms the connection. Senders
retry the handshake automatically if the connection is lost.

Close any other PC display that is already using UDP port `5005`. Allow inbound
UDP traffic on port `5005` in the PC firewall, and allow inbound UDP traffic on
ports `5006` and `5007` on the Pi.