#!/usr/bin/env python3
import serial
import time

ESP_PORT = "/dev/ttyTHS1"
ESP_BAUD = 9600

def send_steer(ser, steer):
    steer = max(-1.0, min(1.0, steer))
    msg = f"S:{steer:.3f}\n"
    ser.write(msg.encode())
    print("send:", msg.strip())

def main():
    print("Opening ESP32 serial...")
    ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=1)
    time.sleep(2)

    print("Steering test start")

    try:
        while True:
            send_steer(ser, 0.0)
            time.sleep(1)

            send_steer(ser, 1.0)
            time.sleep(1)

            send_steer(ser, 0.0)
            time.sleep(1)

            send_steer(ser, -1.0)
            time.sleep(1)

            send_steer(ser, 0.0)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nreturn center")
        send_steer(ser, 0.0)
        time.sleep(0.2)

    finally:
        ser.close()

if __name__ == "__main__":
    main()