# UWB, IMU, and lidar over UDP

Install the requirements for the relevant device from the repository root.

## Raspberry Pi

```powershell
python -m pip install -r pi/requirements.txt
python -m pi.pi_sensors --pc-ip 192.168.1.100
```

`--pc-ip` is optional; the default destination is `192.168.1.100`. Keep
`USE_LIVE_DATA = False` in `pi/pi_sensors.py` to replay the recorded capture
files, or set it to `True` to read from the sensors.

## Windows PC

```powershell
python -m pip install -r pc/requirements.txt
python -m pc.pc_display
```

Allow inbound UDP traffic on port `5005` in the PC firewall. Set `PC_IP` in
`common.py` or pass `--pc-ip` on the Pi to match the PC's address.