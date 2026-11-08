import unittest
from unittest.mock import patch

from pi import pi_control_monitor


class FakeSocket:
    def __init__(self):
        self.bound_address = None

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def bind(self, address):
        self.bound_address = address

    def settimeout(self, _timeout):
        pass

    def recvfrom(self, _size):
        raise KeyboardInterrupt


class ControlMonitorTests(unittest.TestCase):
    def _run_monitor(self, arguments):
        fake_socket = FakeSocket()
        with (
            patch(
                "pi.pi_control_monitor.socket.socket",
                return_value=fake_socket,
            ) as socket_factory,
            patch("builtins.print") as print_output,
        ):
            pi_control_monitor.main(arguments)
        socket_factory.assert_called_once_with(
            pi_control_monitor.socket.AF_INET,
            pi_control_monitor.socket.SOCK_DGRAM,
        )
        return fake_socket.bound_address, print_output

    def test_defaults_bind_udp_on_all_interfaces_port_5007(self):
        address, output = self._run_monitor([])

        self.assertEqual(address, ("0.0.0.0", 5007))
        self.assertTrue(
            any(
                "Listening on 0.0.0.0:5007 at" in str(call)
                for call in output.call_args_list
            )
        )

    def test_host_and_port_can_be_overridden(self):
        address, output = self._run_monitor(
            ["--host", "192.0.2.10", "--port", "6000"]
        )

        self.assertEqual(address, ("192.0.2.10", 6000))
        self.assertTrue(
            any(
                "Listening on 192.0.2.10:6000 at" in str(call)
                for call in output.call_args_list
            )
        )

    def test_status_accepts_auxiliary_control_keys(self):
        with patch("builtins.print") as print_output:
            pi_control_monitor.print_status(
                0.0, 1, {"up", "grabber_up", "claw_open"}, 10.0
            )

        self.assertIn(
            "keys=UP+GRABBER_UP+CLAW_OPEN",
            str(print_output.call_args),
        )


if __name__ == "__main__":
    unittest.main()
