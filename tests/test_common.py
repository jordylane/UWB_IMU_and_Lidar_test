import struct
import unittest

from common import (
    ACK_MAGIC,
    CONTROL_PACKET_FORMAT,
    HELLO_MAGIC,
    control_destinations,
    is_ack_packet,
    is_hello_packet,
    make_ack_packet,
    make_hello_packet,
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

    def test_auxiliary_commands_do_not_change_drive_mixing(self):
        self.assertEqual(mix({"up", "grabber_up"}), (1.0, 1.0))


class ControlPacketTests(unittest.TestCase):
    def test_unpack_control_packet(self):
        packet = struct.pack(CONTROL_PACKET_FORMAT, 42, 12.5, 0x05)
        self.assertEqual(
            unpack_control_packet(packet),
            (42, 12.5, {"up", "left"}),
        )

    def test_unpack_control_packet_with_grabber_and_claw_commands(self):
        packet = struct.pack(CONTROL_PACKET_FORMAT, 43, 12.6, 0xF0)
        self.assertEqual(
            unpack_control_packet(packet),
            (
                43,
                12.6,
                {"grabber_up", "grabber_down", "claw_open", "claw_close"},
            ),
        )

    def test_rejects_malformed_packet(self):
        with self.assertRaises(ValueError):
            unpack_control_packet(b"short")

    def test_control_destinations_include_sensor_and_monitor_udp_ports(self):
        self.assertEqual(
            control_destinations("192.0.2.10"),
            (("192.0.2.10", 5006), ("192.0.2.10", 5007)),
        )

    def test_hello_and_ack_packets_are_distinct_from_control_packets(self):
        hello = make_hello_packet()
        ack = make_ack_packet()
        control = struct.pack(CONTROL_PACKET_FORMAT, 1, 1.0, 0)

        self.assertEqual(hello, HELLO_MAGIC + b"CONNECTION OK")
        self.assertEqual(ack, ACK_MAGIC + b"CONNECTION ACK")
        self.assertTrue(is_hello_packet(hello))
        self.assertTrue(is_ack_packet(ack))
        self.assertFalse(is_hello_packet(ack))
        self.assertFalse(is_hello_packet(control))
        self.assertFalse(is_ack_packet(hello))
        self.assertFalse(is_ack_packet(control))
        with self.assertRaises(ValueError):
            unpack_control_packet(hello)


if __name__ == "__main__":
    unittest.main()
