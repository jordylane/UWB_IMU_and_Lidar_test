import argparse
import socket
import time

from common import (
    CONTROL_PORT,
    CONTROL_RATE_HZ,
    CONTROL_TIMEOUT_SECONDS,
    mix,
    unpack_control_packet,
)

HEARTBEAT_SECONDS = 1.0
KEY_ORDER = ("up", "down", "left", "right")


def print_status(start, sequence, keys, age_ms):
    names = "+".join(key.upper() for key in KEY_ORDER if key in keys) or "NONE"
    left, right = mix(keys)
    print(
        f"[{time.perf_counter() - start:.3f}] seq={sequence} keys={names} "
        f"left={left:.2f} right={right:.2f} age={age_ms:.1f}ms",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Monitor PC vehicle-control packets.")
    parser.add_argument("--all", action="store_true", help="Print every valid packet.")
    args = parser.parse_args()
    start = time.perf_counter()
    last_packet_time = None
    last_print_time = 0.0
    last_keys = None
    last_sequence = None
    received = dropped = missing = 0
    signal_lost = False

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control_socket:
        try:
            control_socket.bind(("0.0.0.0", CONTROL_PORT))
        except OSError as exc:
            # Test with this script instead of the main Pi script, or configure
            # SO_REUSEADDR/SO_REUSEPORT; UDP may still deliver to only one socket
            # unless multicast/broadcast is used.
            print(
                f"Cannot bind UDP {CONTROL_PORT}: {exc}. Stop the main Pi script "
                "for testing, or configure socket reuse (UDP packet delivery "
                "may be exclusive).",
                flush=True,
            )
            return

        control_socket.settimeout(0.05)
        print(f"Listening on 0.0.0.0:{CONTROL_PORT} at {CONTROL_RATE_HZ} Hz")
        try:
            while True:
                try:
                    packet, _ = control_socket.recvfrom(1024)
                except socket.timeout:
                    now = time.perf_counter()
                    reference = last_packet_time if last_packet_time is not None else start
                    if not signal_lost and now - reference >= CONTROL_TIMEOUT_SECONDS:
                        print(f"[{now - start:.3f}] NO SIGNAL - STOP", flush=True)
                        signal_lost = True
                        last_keys = None
                    elif (
                        not signal_lost and last_keys is not None
                        and now - last_print_time >= HEARTBEAT_SECONDS
                    ):
                        age_ms = (now - last_packet_time) * 1000.0
                        print_status(start, last_sequence, last_keys, age_ms)
                        last_print_time = now
                    continue

                now = time.perf_counter()
                received += 1
                try:
                    sequence, _, keys = unpack_control_packet(packet)
                except ValueError:
                    dropped += 1
                    continue

                if last_sequence is not None:
                    difference = (sequence - last_sequence) & 0xFFFFFFFF
                    if difference == 0 or difference >= 0x80000000:
                        dropped += 1
                        print(f"[{now - start:.3f}] WARNING out-of-order "
                              f"dropped={dropped} missing={missing}", flush=True)
                        continue
                    missing += difference - 1
                    if difference > 1:
                        print(f"[{now - start:.3f}] WARNING sequence gap "
                              f"dropped={dropped} missing={missing}", flush=True)

                age_ms = (
                    (now - last_packet_time) * 1000.0
                    if last_packet_time is not None else 0.0
                )
                changed = keys != last_keys
                restored = signal_lost
                if restored:
                    print(f"[{now - start:.3f}] SIGNAL RESTORED", flush=True)
                    signal_lost = False

                last_sequence = sequence
                last_packet_time = now
                if args.all or changed or restored:
                    print_status(start, sequence, keys, age_ms)
                    last_print_time = now
                last_keys = keys
        except KeyboardInterrupt:
            elapsed = time.perf_counter() - start
            rate = received / elapsed if elapsed else 0.0
            print(f"Stopped: received={received} dropped={dropped} "
                  f"missing={missing} average_rate={rate:.2f} packets/s")


if __name__ == "__main__":
    main()
