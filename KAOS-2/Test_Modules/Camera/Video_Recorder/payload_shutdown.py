#!/usr/bin/env python3

import subprocess

from gpiozero import Button


SHUTDOWN_GPIO = 20


def main() -> None:
    # The Pico pulls GPIO20 LOW to request a payload shutdown.
    shutdown_input = Button(
        SHUTDOWN_GPIO,
        pull_up=True,
        bounce_time=0.1,
    )

    print(
        f"Watching GPIO{SHUTDOWN_GPIO} for shutdown request",
        flush=True,
    )
    shutdown_input.wait_for_press()
    print("Shutdown request received", flush=True)

    # Finish the current chunk and close any open camera files first.
    subprocess.run(
        ["systemctl", "stop", "kaos-camera.service"],
        check=False,
    )

    # The Pico can remove payload power only after the Pi halts cleanly.
    subprocess.run(["shutdown", "-h", "now"], check=False)


if __name__ == "__main__":
    main()
