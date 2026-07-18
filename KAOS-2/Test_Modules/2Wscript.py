import serial
import time
import os

pico = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=1
)

time.sleep(2)

print("KAOS Pi controller started")

while True:

    if pico.in_waiting:

        command = pico.readline().decode().strip()

        print("Pico says:", command)

        if command == "SHUTOFF":

            print("Shutdown command received")

            # Stop your camera/video

            print("Camera stopped")

            # Tell Pico that the Pi is ready to lose power
            pico.write(b"DONE\n")

            print("DONE sent to Pico")

            # Shut down 2W
            os.system("sudo shutdown -h now")

            break

    time.sleep(0.1)
