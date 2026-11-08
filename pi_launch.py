import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TERMINALS = ("lxterminal", "x-terminal-emulator", "gnome-terminal", "xterm")
WINDOWS = (
    ("Pi Sensors", ROOT / "pi" / "pi_sensors.py"),
    (
        "Pi Control Monitor",
        ROOT / "pi" / "pi_control_monitor.py",
        ("--host", "0.0.0.0", "--port", "5007"),
    ),
)


def terminal_command(terminal, title, script, arguments=()):
    script_command = shlex.join([sys.executable, str(script), *arguments])
    shell_script = (
        f"{script_command}; status=$?; "
        'if [ "$status" -ne 0 ]; then '
        "printf '\\nScript exited with status %s.\\n' \"$status\"; "
        "read -r -p 'Press Enter to close this window...' _; "
        "fi; exit \"$status\""
    )

    if terminal == "lxterminal":
        command = f"bash -lc {shlex.quote(shell_script)}"
        return [terminal, "--title", title, "--command", command]
    if terminal == "x-terminal-emulator":
        return [
            terminal, "-T", title, "-e", "bash", "-lc", shell_script,
        ]
    if terminal == "gnome-terminal":
        return [
            terminal, "--title", title, "--", "bash", "-lc", shell_script,
        ]
    if terminal == "xterm":
        return [terminal, "-T", title, "-e", "bash", "-lc", shell_script]
    raise ValueError(f"Unsupported terminal emulator: {terminal}")


def main():
    if not os.environ.get("DISPLAY"):
        print(
            "Cannot open Raspberry Pi terminal windows: DISPLAY is not set. "
            "Run this launcher from a graphical desktop session.",
            file=sys.stderr,
        )
        return 1

    terminal = next(
        (name for name in TERMINALS if shutil.which(name)),
        None,
    )
    if terminal is None:
        print(
            "Cannot open Raspberry Pi terminal windows: none of "
            f"{', '.join(TERMINALS)} is installed.",
            file=sys.stderr,
        )
        return 1

    for window in WINDOWS:
        title, script = window[:2]
        if not script.is_file():
            print(f"Cannot launch {title}: script not found: {script}",
                  file=sys.stderr)
            return 1

    processes = []
    try:
        for window in WINDOWS:
            title, script = window[:2]
            arguments = window[2] if len(window) > 2 else ()
            process = subprocess.Popen(
                terminal_command(terminal, title, script, arguments),
                cwd=ROOT,
            )
            processes.append(process)
    except OSError as error:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        print(f"Cannot open Raspberry Pi terminal window: {error}",
              file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
