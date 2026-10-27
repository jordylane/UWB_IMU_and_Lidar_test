import json

import numpy as np


PC_IP = "192.168.1.115"
PI_IP = "192.168.1.116"
UDP_PORT = 5005
CONTROL_PORT = 5006
SEND_RATE_HZ = 20
CONTROL_RATE_HZ = 20
MAX_PACKET_SIZE = 60 * 1024
anchors = np.array([
    [0.0, 0.0, 1.2],
    [0.0, 4.0, 0.75],
    [4.0, 4.0, 1.2],
    [4.0, 0.0, 0.75]
])


def _to_builtin(value):
    if isinstance(value, np.ndarray):
        return _to_builtin(value.tolist())
    if isinstance(value, np.generic):
        return _to_builtin(value.item())
    if isinstance(value, dict):
        return {key: _to_builtin(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_builtin(item) for item in value]
    return value


def _round_floats(value):
    if isinstance(value, float):
        return round(value, 3)
    if isinstance(value, dict):
        return {key: _round_floats(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_round_floats(item) for item in value]
    return value


def encode_packet(packet):
    """Encode a packet as compact JSON, reducing float precision if needed."""
    payload = _to_builtin(packet)
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")

    if len(encoded) > MAX_PACKET_SIZE:
        payload = _round_floats(payload)
        encoded = json.dumps(
            payload,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")

    return encoded


def decode_packet(packet):
    """Decode JSON bytes and require the top-level packet to be an object."""
    decoded = json.loads(packet)
    if not isinstance(decoded, dict):
        raise ValueError("UDP packet must contain a JSON object")
    return decoded
