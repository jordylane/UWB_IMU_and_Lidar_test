import io
import unittest
from unittest.mock import patch

import launch


class FakeProcess:
    def __init__(self, exit_code=None):
        self.stdout = io.StringIO("[child] ready\n")
        self.returncode = None
        self.exit_code = exit_code
        self.terminated = False

    def poll(self):
        if self.exit_code is not None:
            self.returncode = self.exit_code
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = -15

    def kill(self):
        self.returncode = -9

    def wait(self, timeout=None):
        if self.returncode is None:
            self.returncode = -15
        return self.returncode


class UnresponsiveProcess(FakeProcess):
    def __init__(self):
        super().__init__()
        self.killed = False

    def wait(self, timeout=None):
        if not self.killed:
            raise launch.subprocess.TimeoutExpired("fake-child", timeout)
        return super().wait(timeout)

    def kill(self):
        self.killed = True
        super().kill()


class LauncherTests(unittest.TestCase):
    @patch("launch.time.sleep")
    @patch("launch.subprocess.Popen")
    def test_child_exit_stops_sibling_and_reports_failure(
        self, popen, _sleep
    ):
        failed = FakeProcess(exit_code=2)
        sibling = FakeProcess()
        popen.side_effect = [failed, sibling]

        with patch("builtins.print") as print_output:
            result = launch.main()

        self.assertEqual(result, 1)
        self.assertTrue(sibling.terminated)
        self.assertEqual(popen.call_count, 2)
        self.assertEqual(
            popen.call_args_list[0].args[0],
            [
                launch.sys.executable,
                str(launch.ROOT / "pi" / "pi_control_monitor.py"),
                "--forwarded",
            ],
        )
        self.assertEqual(
            popen.call_args_list[0].kwargs["cwd"],
            launch.ROOT / "pi",
        )
        self.assertEqual(
            popen.call_args_list[1].args[0],
            [
                launch.sys.executable,
                str(launch.ROOT / "pi" / "pi_sensors.py"),
            ],
        )
        self.assertTrue(
            any("[control] exited with code 2" in str(call)
                for call in print_output.call_args_list)
        )

    @patch("launch.subprocess.Popen")
    def test_keyboard_interrupt_stops_children_without_failure(self, popen):
        control = FakeProcess()
        sensors = FakeProcess()
        popen.side_effect = [control, sensors]

        with patch("launch.time.sleep", side_effect=KeyboardInterrupt):
            result = launch.main()

        self.assertEqual(result, 0)
        self.assertTrue(control.terminated)
        self.assertTrue(sensors.terminated)

    @patch("launch.subprocess.Popen")
    def test_start_failure_stops_already_started_child(self, popen):
        control = FakeProcess()
        popen.side_effect = [control, OSError("spawn failed")]

        with patch("builtins.print"):
            result = launch.main()

        self.assertEqual(result, 1)
        self.assertTrue(control.terminated)

    def test_unresponsive_child_is_killed_after_timeout(self):
        process = UnresponsiveProcess()

        launch.stop_processes([process])

        self.assertTrue(process.terminated)
        self.assertTrue(process.killed)
        self.assertEqual(process.returncode, -9)


if __name__ == "__main__":
    unittest.main()
