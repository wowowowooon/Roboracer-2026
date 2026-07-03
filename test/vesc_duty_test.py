import time
import serial
import struct

PORT = "/dev/ttyACM0"
BAUD = 115200
COMM_SET_DUTY = 5


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def make_packet(payload):
    packet = bytearray()
    packet.append(2)
    packet.append(len(payload))
    packet.extend(payload)

    crc = crc16(payload)
    packet.append((crc >> 8) & 0xFF)
    packet.append(crc & 0xFF)
    packet.append(3)

    return packet


def set_duty(ser, duty):
    value = int(duty * 100000)

    payload = bytearray()
    payload.append(COMM_SET_DUTY)
    payload.extend(struct.pack(">i", value))

    ser.write(make_packet(payload))
    ser.flush()


print("START RAMP TO 20% DUTY TEST")

ser = None

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print("Serial opened:", ser.name)

    time.sleep(1)

    for i in range(0, 21):
        duty = i / 100.0
        print(f"Duty {i}%")
        set_duty(ser, duty)
        time.sleep(0.5)

    print("Hold 20% for 10 sec")
    set_duty(ser, 0.20)
    time.sleep(10)

    print("Stop")
    set_duty(ser, 0.0)
    time.sleep(1)

    ser.close()
    print("DONE")

except KeyboardInterrupt:
    print("Emergency stop by keyboard")
    if ser is not None:
        set_duty(ser, 0.0)
        time.sleep(0.2)
        ser.close()

except Exception as e:
    print("ERROR:", repr(e))
    if ser is not None:
        set_duty(ser, 0.0)
        ser.close()