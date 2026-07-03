import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

print("RC UART TEST START")
print(f"PORT: {PORT}, BAUD: {BAUD}")

while True:
    line = ser.readline().decode(errors="ignore").strip()

    if line:
        print(line)

    time.sleep(0.01)

