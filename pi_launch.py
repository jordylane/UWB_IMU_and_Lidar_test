import subprocess
import sys
import threading
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CHILDREN = (
    ("control", ROOT / "pi" / "pi_control_monitor.py", ("--forwarded",)),
    ("sensors", ROOT / "pi" / "pi_sensors.py", ()),
)
SHUTDOWN_TIMEOUT_SECONDS = 3.0


def stream_output(name, process):
    if process.stdout is None:
        return
    for line in process.stdout:
        print(f"[{name}] {line}", end="", flush=True)


def stop_processes(processes):
    active = [process for process in processes if process.poll() is None]
    for process in active:
        process.terminate()

    deadline = time.monotonic() + SHUTDOWN_TIMEOUT_SECONDS
    for process in active:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    for name, script, _ in CHILDREN:
        if not script.is_file():
            print(f"[{name}] script not found: {script}", file=sys.stderr)
            return 1

    processes = []
    output_threads = []
    failed = False
    try:
        for name, script, arguments in CHILDREN:
            process = subprocess.Popen(
                [sys.executable, str(script), *arguments],
                cwd=script.parent,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            processes.append(process)
            output_thread = threading.Thread(
                target=stream_output,
                args=(name, process),
                name=f"{name}-output",
            )
            output_thread.start()
            output_threads.append(output_thread)

        while True:
            exited = [
                (name, process)
                for (name, _, _), process in zip(CHILDREN, processes)
                if process.poll() is not None
            ]
            if exited:
                for name, process in exited:
                    print(
                        f"[{name}] exited with code {process.returncode}",
                        flush=True,
                    )
                failed = any(process.returncode != 0 for _, process in exited)
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping child processes...", flush=True)
    except OSError as error:
        print(f"Unable to start child process: {error}", file=sys.stderr)
        failed = True
    finally:
        stop_processes(processes)
        for output_thread in output_threads:
            output_thread.join()

    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
