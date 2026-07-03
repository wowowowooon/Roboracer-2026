import serial
import struct
import time

# =========================
# 포트 설정
# =========================
ESP_PORT = "/dev/ttyTHS1"      # ESP32 UART
VESC_PORT = "/dev/ttyACM0"     # VESC USB

ESP_BAUD = 115200
VESC_BAUD = 115200

# =========================
# 조종기 설정
# =========================
CENTER_CH2 = 1497
DEADZONE = 30
MAX_DUTY = 0.15

CH5_MANUAL_THRESHOLD = 1500
CH6_ESTOP_THRESHOLD = 1500

# =========================
# VESC RAW 패킷 설정
# =========================
COMM_SET_DUTY = 5


def crc16(data):
    crc = 0

    for b in data:
        crc ^= b << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

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


def set_duty_packet(duty):
    value = int(duty * 100000)

    payload = bytearray()
    payload.append(COMM_SET_DUTY)
    payload.extend(struct.pack(">i", value))

    return make_packet(payload)


def send_duty(vesc, duty):
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    packet = set_duty_packet(duty)
    vesc.write(packet)


# =========================
# RC 파싱
# ESP32 형식: RC,ch1,ch2,ch5,ch6
# =========================
def parse_rc_line(line):
    parts = line.strip().split(",")

    if len(parts) != 5:
        return None

    if parts[0] != "RC":
        return None

    try:
        ch1 = int(parts[1])
        ch2 = int(parts[2])
        ch5 = int(parts[3])
        ch6 = int(parts[4])

        return ch1, ch2, ch5, ch6

    except ValueError:
        return None


def ch2_to_duty(ch2):
    if ch2 == 0:
        return 0.0

    error = ch2 - CENTER_CH2

    if abs(error) < DEADZONE:
        return 0.0

    duty = error / 500.0 * MAX_DUTY

    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))

    return duty


def main():
    print("RC TO VESC RAW START")
    print(f"ESP_PORT  = {ESP_PORT}")
    print(f"VESC_PORT = {VESC_PORT}")
    print(f"CENTER_CH2 = {CENTER_CH2}")
    print(f"MAX_DUTY   = {MAX_DUTY}")

    esp = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.1)
    vesc = serial.Serial(VESC_PORT, VESC_BAUD, timeout=0.1)

    last_rx_time = time.time()
    last_print_time = 0

    try:
        while True:
            line = esp.readline().decode(errors="ignore").strip()

            if line:
                data = parse_rc_line(line)

                if data is not None:
                    ch1, ch2, ch5, ch6 = data
                    last_rx_time = time.time()

                    manual_mode = ch5 < CH5_MANUAL_THRESHOLD
                    estop = ch6 < CH6_ESTOP_THRESHOLD

                    if estop:
                        duty = 0.0

                    elif manual_mode:
                        duty = ch2_to_duty(ch2)

                    else:
                        duty = 0.0

                    send_duty(vesc, duty)

                    now = time.time()
                    if now - last_print_time > 0.1:
                        last_print_time = now

                        print(
                            f"CH1={ch1} CH2={ch2} CH5={ch5} CH6={ch6} "
                            f"MANUAL={manual_mode} ESTOP={estop} DUTY={duty:.3f}"
                        )

            # ESP 신호 끊기면 정지
            if time.time() - last_rx_time > 0.3:
                send_duty(vesc, 0.0)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("STOP BY KEYBOARD")

    finally:
        print("VESC DUTY 0")
        send_duty(vesc, 0.0)
        time.sleep(0.1)

        esp.close()
        vesc.close()


if __name__ == "__main__":
    main()