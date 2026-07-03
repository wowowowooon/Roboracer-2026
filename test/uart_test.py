import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("UART TEST START")

while True:
    ser.write(b"ping\n")
    line = ser.readline().decode(errors="ignore").strip()
    print("RX:", line)
    time.sleep(1)
    