# -*- coding: utf-8 -*-
pip install pyserial
import serial

ser = serial.Serial('COM4', 115200)

with open("data.csv", "a") as f:
    while True:
        line = ser.readline().decode("utf-8").strip()
        f.write(line + "\n")
