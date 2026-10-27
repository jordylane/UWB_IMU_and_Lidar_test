import struct
import unittest

from common import (
    CONTROL_PACKET_FORMAT,
    mix,
    unpack_control_packet,
)


class MixTests(unittest.TestCase):
    def test_all_nine_key_combinations(self):
        cases = {
            frozenset(): (0.0, 0.0),
            frozenset({"up"}): (1.0, 1.0),
            frozenset({"down"}): (-1.0, -1.0),
            frozenset({"left"}): (-1.0, 1.0),
            frozenset({"right"}): (1.0, -1.0),
            frozenset({"up", "left"}): (0.0, 1.0),
            frozenset({"up", "right"}): (1.0, 0.0),
            frozenset({"down", "left"}): (-1.0, 0.0),
            frozenset({"down", "right"}): (0.0, -1.0),
        }

        for keys, expected in cases.items():
            with self.subTest(keys=keys):
                self.assertEqual(mix(keys), expected)


class ControlPacketTests(unittest.TestCase):
    def test_unpack_control_packet(self):
        packet = struct.pack(CONTROL_PACKET_FORMAT, 42, 12.5, 0x05)
        self.assertEqual(
            unpack_control_packet(packet),
            (42, 12.5, {"up", "left"}),
        )

    def test_rejects_malformed_packet(self):
        with self.assertRaises(ValueError):
            unpack_control_packet(b"short")


if __name__ == "__main__":
    unittest.main()
