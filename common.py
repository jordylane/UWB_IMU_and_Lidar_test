import json
import math
import struct
from collections.abc import Collection

try:
    import numpy as np
except ModuleNotFoundError:
    np = None


PC_IP = "192.168.1.115"
PI_IP = "192.168.1.116"
UDP_PORT = 5005
CONTROL_PORT = 5006
SEND_RATE_HZ = 20
CONTROL_RATE_HZ = 20
CONTROL_TIMEOUT_SECONDS = 0.3
TRACK_WIDTH = 0.35  # metres between the tank's tracks
MAX_SPEED = 1.0  # metres per second at full track command
CONTROL_PACKET_FORMAT = "!IdB"  # sequence, PC perf_counter timestamp, key bits
CONTROL_KEY_BITS = {"up": 0x01, "down": 0x02, "left": 0x04, "right": 0x08}
MAX_PACKET_SIZE = 60 * 1024
_ANCHOR_COORDINATES = (
    [0.0, 0.0, 1.2],
    [0.0, 4.0, 0.75],
    [4.0, 4.0, 1.2],
    [4.0, 0.0, 0.75],
)
anchors = np.array(_ANCHOR_COORDINATES) if np is not None else _ANCHOR_COORDINATES


def mix(keys: Collection[str]) -> tuple[float, float]:
    """Mix held arrow keys into normalized left/right differential-track speeds."""
    unknown_keys = set(keys) - CONTROL_KEY_BITS.keys()
    if unknown_keys:
        raise ValueError(f"Unknown control keys: {sorted(unknown_keys)}")

    throttle = float("up" in keys) - float("down" in keys)
    steering = float("right" in keys) - float("left" in keys)
    left = throttle + steering
    right = throttle - steering
    scale = max(1.0, abs(left), abs(right))
    return left / scale, right / scale


def pack_control_packet(
    sequence: int,
    pc_timestamp: float,
    keys: Collection[str],
) -> bytes:
    unknown_keys = set(keys) - CONTROL_KEY_BITS.keys()
    if unknown_keys:
        raise ValueError(f"Unknown control keys: {sorted(unknown_keys)}")
    key_bits = 0
    for key in keys:
        key_bits |= CONTROL_KEY_BITS[key]
    return struct.pack(CONTROL_PACKET_FORMAT, sequence, pc_timestamp, key_bits)


def unpack_control_packet(packet: bytes) -> tuple[int, float, set[str]]:
    packet_struct = struct.Struct(CONTROL_PACKET_FORMAT)
    if len(packet) != packet_struct.size:
        raise ValueError(f"Expected {packet_struct.size}-byte control packet")

    sequence, pc_timestamp, key_bits = packet_struct.unpack(packet)
    if key_bits & ~0x0F:
        raise ValueError("Control packet contains unknown key bits")
    if not math.isfinite(pc_timestamp):
        raise ValueError("Control packet timestamp must be finite")

    keys = {
        key for key, bit in CONTROL_KEY_BITS.items()
        if key_bits & bit
    }
    return sequence, pc_timestamp, keys


def _to_builtin(value):
    if np is not None and isinstance(value, np.ndarray):
        return _to_builtin(value.tolist())
    if np is not None and isinstance(value, np.generic):
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
