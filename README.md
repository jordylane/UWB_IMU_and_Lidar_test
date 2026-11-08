# UWB, IMU, and lidar over UDP

Install the requirements for the relevant device from the repository root.

## Raspberry Pi

```powershell
python -m pip install -r pi/requirements.txt
python launch.py
```

The launcher starts both `pi/pi_control_monitor.py` and `pi/pi_sensors.py`
under the current Python interpreter, prefixes each output line, and stops both
processes if either one exits. To run only the sensor process, use
`python -m pi.pi_sensors --pc-ip 192.168.1.115`; the `--pc-ip` option is
optional and defaults to `192.168.1.115`.

## Windows PC

```powershell
python -m pip install -r pc/requirements.txt
python -m pc.pc_display
```

Close any other PC display that is already using UDP port `5005`. Allow inbound
UDP traffic on that port in the PC firewall. Set `PC_IP` in `common.py` or pass
`--pc-ip` on the Pi to match the PC's address.