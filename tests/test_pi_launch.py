import os
import unittest
from unittest.mock import patch

import pi_launch


class PiLauncherTests(unittest.TestCase):
    @patch("pi_launch.subprocess.Popen")
    @patch("pi_launch.shutil.which", return_value="/usr/bin/lxterminal")
    def test_launches_both_scripts_in_separate_titled_windows(
        self, _which, popen
    ):
        with patch.dict(os.environ, {"DISPLAY": ":0"}):
            result = pi_launch.main()

        self.assertEqual(result, 0)
        self.assertEqual(popen.call_count, 2)
        self.assertEqual(
            popen.call_args_list[0].args[0],
            pi_launch.terminal_command(
                "lxterminal",
                "Pi Sensors",
                pi_launch.ROOT / "pi" / "pi_sensors.py",
            ),
        )
        self.assertEqual(
            popen.call_args_list[1].args[0],
            pi_launch.terminal_command(
                "lxterminal",
                "Pi Control Monitor",
                pi_launch.ROOT / "pi" / "pi_control_monitor.py",
                ("--host", "0.0.0.0", "--port", "5007"),
            ),
        )
        for call in popen.call_args_list:
            self.assertEqual(call.kwargs["cwd"], pi_launch.ROOT)
            self.assertIn(pi_launch.sys.executable, call.args[0][-1])

    @patch("pi_launch.subprocess.Popen")
    @patch("pi_launch.shutil.which", return_value=None)
    def test_reports_missing_terminal_emulator(self, _which, popen):
        with patch.dict(os.environ, {"DISPLAY": ":0"}):
            with patch("builtins.print") as print_output:
                result = pi_launch.main()

        self.assertEqual(result, 1)
        popen.assert_not_called()
        self.assertIn("none of", str(print_output.call_args))

    @patch("pi_launch.subprocess.Popen")
    @patch("pi_launch.shutil.which")
    def test_reports_missing_display(self, which, popen):
        with patch.dict(os.environ, {}, clear=True):
            with patch("builtins.print") as print_output:
                result = pi_launch.main()

        self.assertEqual(result, 1)
        which.assert_not_called()
        popen.assert_not_called()
        self.assertIn("DISPLAY is not set", str(print_output.call_args))

    def test_terminal_command_variants_keep_failed_script_window_open(self):
        script = pi_launch.ROOT / "pi" / "pi_sensors.py"
        for terminal in pi_launch.TERMINALS:
            with self.subTest(terminal=terminal):
                command = pi_launch.terminal_command(
                    terminal, "Pi Sensors", script
                )
                self.assertIn("Pi Sensors", command)
                self.assertIn(pi_launch.sys.executable, " ".join(command))
                self.assertIn("Press Enter to close", " ".join(command))


if __name__ == "__main__":
    unittest.main()
