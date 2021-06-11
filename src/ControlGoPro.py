#!/usr/bin/env python3
from goprocam import GoProCamera
import serial
import time

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

arduino_serial = None
isRecording = False

# GoPro
goproCamera = GoProCamera.GoPro()


def openSerial():
    global arduino_serial
    if arduino_serial == None:
        try:
            arduino_serial = serial.Serial(PORT, BAUDRATE, timeout=0.1)
            print("Opened ", arduino_serial.name)
            time.sleep(3)
            arduino_serial.flushInput()
            time.sleep(0.1)
        except serial.SerialException as e:
            print(e)
            exit(0)
        except TypeError as e:
            print(e)
            arduino_serial.close()
            exit(0)

def main():
    global arduino_serial, isRecording, goproCamera
    openSerial()
    while 1:
        ser_bytes = arduino_serial.readline()
        decoded_bytes = ser_bytes[0 : len(ser_bytes) - 2].decode("utf-8", errors="ignore")
        print(decoded_bytes)
        if "Begin" in decoded_bytes:
            # TODO: Shutter GoPro
            goproCamera.shoot_video(1)
            isRecording = True
        elif "Stop" in decoded_bytes and isRecording:
            # TODO: Stop Video recording
            goproCamera.shoot_video(0)
            isRecording = False
        time.sleep(3)


if __name__ == "__main__":
    main()
