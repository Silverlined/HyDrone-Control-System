#!/usr/bin/env python3

import serial
import time
import csv
from collections import deque

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# How many sensor samples we want to store
SAMPLE_SIZE = 1000

arduino_serial = None

# Deque for axes
# deque provides an O(1) time complexity for append and pop operations as compared to list which provides O(n) time complexity.
accel_x = deque(maxlen=SAMPLE_SIZE)
accel_y = deque(maxlen=SAMPLE_SIZE)
accel_z = deque(maxlen=SAMPLE_SIZE)

gyro_x = deque(maxlen=SAMPLE_SIZE)
gyro_y = deque(maxlen=SAMPLE_SIZE)
gyro_z = deque(maxlen=SAMPLE_SIZE)

previos_x_gyro = 0
previos_y_gyro = 0
previos_z_gyro = 0

isSending = False
isReady = False


def filterValues(data):
    global accel_x, accel_y, accel_z
    global gyro_x, gyro_y, gyro_z
    global previos_x_gyro, previos_y_gyro, previos_z_gyro

    try:
        x_accel, y_accel, z_accel = data[0:3]
        x_gyro, y_gyro, z_gyro = data[3:6]
    except ValueError as e:
        print(e)
        return None

    accel_x.append(x_accel)
    accel_y.append(y_accel)
    accel_z.append(z_accel)

    if abs(x_gyro) < abs(5 * previos_x_gyro) and abs(y_gyro) < abs(5 * previos_y_gyro) and abs(z_gyro) < abs(5 * previos_z_gyro):
        gyro_x.append(x_gyro)
        gyro_y.append(y_gyro)
        gyro_z.append(z_gyro)

    previos_x_gyro, previos_y_gyro, previos_z_gyro = x_gyro, y_gyro, z_gyro


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


def collectData():
    global arduino_serial, isSending, isReady
    # Poll the serial port
    ser_bytes = arduino_serial.readline()
    decoded_bytes = ser_bytes[0 : len(ser_bytes) - 2].decode("utf-8", errors="ignore")
    print(decoded_bytes)
    if "Stop" in decoded_bytes:
        isSending = False
        isReady = True
    if not "Data:" in decoded_bytes:
        return None
    values = []
    for i in decoded_bytes.replace("Data:", "").strip().split(","):
        try:
            values.append(float(i))
        except ValueError as e:
            print(e)
            return None
    filterValues(values)


def main():
    global arduino_serial, isSending, isReady
    openSerial()
    while 1:
        ser_bytes = arduino_serial.readline()
        decoded_bytes = ser_bytes[0 : len(ser_bytes) - 2].decode("utf-8", errors="ignore")
        print(decoded_bytes)
        if "Begin" in decoded_bytes:
            isSending = True

        # Collect Data
        while isSending:
            collectData()

        # Store Data
        if isReady:
            print("Data Collection Finished")
            with open("accelerometer_data.csv", "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                for x, y, z in zip(accel_x, accel_y, accel_z):
                    if x and y and z is not None:
                        values = [x, y, z]
                        writer.writerow(values)
            with open("gyro_data.csv", "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                for x, y, z in zip(gyro_x, gyro_y, gyro_z):
                    if x and y and z is not None:
                        values = [x, y, z]
                        writer.writerow(values)
            break

    # Cleanup & Exit
    if csv_file:
        csv_file.close()
    if arduino_serial:
        arduino_serial.close()
    exit(0)


if __name__ == "__main__":
    main()
