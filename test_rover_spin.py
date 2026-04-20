#!/usr/bin/env python3
import json
import serial
import time

port = "/dev/rover_base"
baud = 115200

ser = serial.Serial(port, baud, timeout=0.1)
ser.setRTS(False)
ser.setDTR(False)

def send(payload):
    line = json.dumps(payload, separators=(",", ":")) + "\n"
    print("Sending:", line.strip())
    ser.write(line.encode("utf-8"))

time.sleep(1.0)

send({"T": 143, "cmd": 1})
time.sleep(0.5)

send({"T": 1, "L": -0.35, "R": 0.35})
time.sleep(2.0)
send({"T": 1, "L": 0.0, "R": 0.0})
time.sleep(1.0)

send({"T": 1, "L": 0.35, "R": -0.35})
time.sleep(2.0)
send({"T": 1, "L": 0.0, "R": 0.0})

ser.close()

