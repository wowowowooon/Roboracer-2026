#!/usr/bin/env python3
"""
Interactive VESC duty command and CSV logger.

This is a standalone script for bench testing duty vs RPM on a Jetson. It does
not use ROS2. Keep the car lifted before the first test and start with small
duty values.
"""

import argparse
import csv
import math
import queue
import struct
import threading
import time
from dataclasses import dataclass
from datetime import datetime

try:
    import serial
except ImportError as exc:
    print("ERROR: pyserial is required. Install it with: python3 -m pip install pyserial")
    raise SystemExit(1) from exc


COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
DUTY_OUTPUT_SIGN = -1.0

STOP_COMMANDS = {"stop", "q", "quit", "exit"}
ZERO_COMMANDS = {"z", "zero"}

CSV_FIELDS = [
    "timestamp_iso",
    "elapsed_sec",
    "target_duty",
    "applied_duty",
    "vesc_duty_now",
    "erpm",
    "motor_rpm",
    "wheel_rpm",
    "speed_mps",
    "input_voltage",
    "current_motor",
    "current_in",
    "temp_mos",
    "temp_motor",
    "event",
    "note",
    "tachometer",
    "tachometer_abs",
    "tacho_delta",
    "tacho_dt",
    "tacho_erpm",
    "tacho_motor_rpm",
    "tacho_wheel_rpm",
    "tacho_speed_mps",
]


@dataclass
class VescValues:
    """Parsed subset of COMM_GET_VALUES response."""

    temp_mos: float = float("nan")
    temp_motor: float = float("nan")
    current_motor: float = float("nan")
    current_in: float = float("nan")
    duty_now: float = float("nan")
    erpm: float = float("nan")
    input_voltage: float = float("nan")
    tachometer: float = float("nan")
    tachometer_abs: float = float("nan")


@dataclass
class RuntimeState:
    """Values shared with the input thread for status printing."""

    target_duty: float = 0.0
    applied_duty: float = 0.0
    vesc_duty_now: float = float("nan")
    erpm: float = float("nan")
    speed_mps: float = float("nan")
    tacho_speed_mps: float = float("nan")
    input_voltage: float = float("nan")
    # current_motor: motor phase/current-side value, useful for motor load.
    # current_in: battery/input current, use this for battery 30A limit checking.
    current_motor: float = float("nan")
    current_in: float = float("nan")


def crc16_ccitt(data):
    """CRC16-CCITT used by VESC packets."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_packet(payload):
    """Build a VESC small packet: 0x02 | len | payload | crc_hi | crc_lo | 0x03."""
    if len(payload) > 255:
        raise ValueError("small VESC packet payload must be <= 255 bytes")

    crc = crc16_ccitt(payload)
    packet = bytearray()
    packet.append(0x02)
    packet.append(len(payload))
    packet.extend(payload)
    packet.append((crc >> 8) & 0xFF)
    packet.append(crc & 0xFF)
    packet.append(0x03)
    return bytes(packet)


def clamp(value, low, high):
    return max(low, min(high, value))


def move_toward(current, target, step):
    """Ramp current toward target by at most step."""
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


def set_vesc_duty(ser, duty):
    """
    Send COMM_SET_DUTY.

    DUTY_OUTPUT_SIGN reverses the motor direction in software. With the current
    setting, terminal input +0.05 is sent to the VESC as -0.05.
    """
    vesc_duty = duty * DUTY_OUTPUT_SIGN
    duty_raw = int(vesc_duty * 100000)
    payload = bytearray()
    payload.append(COMM_SET_DUTY)
    payload.extend(struct.pack(">i", duty_raw))
    ser.write(make_packet(payload))
    ser.flush()


def read_exact(ser, length):
    """Read exactly length bytes or return None on timeout."""
    data = bytearray()
    deadline = time.monotonic() + max(ser.timeout or 0.0, 0.02)
    while len(data) < length and time.monotonic() < deadline:
        chunk = ser.read(length - len(data))
        if chunk:
            data.extend(chunk)
        else:
            time.sleep(0.001)
    if len(data) != length:
        return None
    return bytes(data)


def read_vesc_packet(ser):
    """
    Read one VESC packet and validate CRC.

    COMM_GET_VALUES normally fits in a small packet. Large-packet support is
    included defensively so a longer firmware response does not confuse the
    stream parser.
    """
    while True:
        start = ser.read(1)
        if not start:
            return None
        start_byte = start[0]
        if start_byte in (0x02, 0x03):
            break

    if start_byte == 0x02:
        length_bytes = read_exact(ser, 1)
        if length_bytes is None:
            return None
        payload_len = length_bytes[0]
    else:
        length_bytes = read_exact(ser, 2)
        if length_bytes is None:
            return None
        payload_len = (length_bytes[0] << 8) | length_bytes[1]

    payload = read_exact(ser, payload_len)
    crc_bytes = read_exact(ser, 2)
    end = read_exact(ser, 1)
    if payload is None or crc_bytes is None or end is None:
        return None
    if end[0] != 0x03:
        return None

    received_crc = (crc_bytes[0] << 8) | crc_bytes[1]
    if crc16_ccitt(payload) != received_crc:
        return None
    return payload


class PayloadReader:
    """Bounds-checked big-endian parser for VESC payloads."""

    def __init__(self, payload):
        self.payload = payload
        self.offset = 0

    def u8(self):
        if self.offset + 1 > len(self.payload):
            raise ValueError("payload too short for u8")
        value = self.payload[self.offset]
        self.offset += 1
        return value

    def i16(self, scale=1.0):
        if self.offset + 2 > len(self.payload):
            raise ValueError("payload too short for i16")
        value = struct.unpack_from(">h", self.payload, self.offset)[0]
        self.offset += 2
        return value / scale

    def i32(self, scale=1.0):
        if self.offset + 4 > len(self.payload):
            raise ValueError("payload too short for i32")
        value = struct.unpack_from(">i", self.payload, self.offset)[0]
        self.offset += 4
        return value / scale


def parse_get_values(payload):
    """
    Parse the front, stable part of COMM_GET_VALUES.

    Common VESC order:
      command, temp_mos, temp_motor, current_motor, current_in, avg_id, avg_iq,
      duty_now, erpm, input_voltage, amp_hours, amp_hours_charged, watt_hours,
      watt_hours_charged, tachometer, tachometer_abs, ...
    Later fields vary by firmware, so tachometer parsing is optional.
    """
    reader = PayloadReader(payload)
    command = reader.u8()
    if command != COMM_GET_VALUES:
        raise ValueError("not a COMM_GET_VALUES response")

    values = VescValues()
    values.temp_mos = reader.i16(scale=10.0)
    values.temp_motor = reader.i16(scale=10.0)
    values.current_motor = reader.i32(scale=100.0)
    values.current_in = reader.i32(scale=100.0)

    # avg_id and avg_iq are present before duty_now in common firmware. They
    # are not needed for this experiment, but consuming them aligns the parser.
    reader.i32(scale=100.0)
    reader.i32(scale=100.0)

    values.duty_now = reader.i16(scale=1000.0)
    values.erpm = reader.i32(scale=1.0)
    values.input_voltage = reader.i16(scale=10.0)

    try:
        # Skip amp-hour and watt-hour counters. They are useful for energy
        # logging, but this experiment only needs the tachometer counters.
        reader.i32(scale=10000.0)
        reader.i32(scale=10000.0)
        reader.i32(scale=10000.0)
        reader.i32(scale=10000.0)
        values.tachometer = reader.i32(scale=1.0)
        values.tachometer_abs = reader.i32(scale=1.0)
    except ValueError:
        # Older or different firmware may return a shorter payload. Keep NaN
        # tacho fields and let the logger continue.
        pass

    return values


def get_values(ser):
    """Request COMM_GET_VALUES and return VescValues, or None on timeout/CRC/parse failure."""
    ser.write(make_packet(bytes([COMM_GET_VALUES])))
    ser.flush()
    payload = read_vesc_packet(ser)
    if payload is None:
        return None
    try:
        return parse_get_values(payload)
    except ValueError:
        return None


def compute_speed(erpm, pole_pairs, gear_ratio, wheel_diameter, invert_erpm):
    """Convert ERPM to motor RPM, wheel RPM, and linear speed."""
    if erpm is None or math.isnan(erpm):
        return float("nan"), float("nan"), float("nan")

    signed_erpm = -erpm if invert_erpm else erpm
    motor_rpm = signed_erpm / pole_pairs
    wheel_rpm = motor_rpm / gear_ratio
    speed_mps = wheel_rpm / 60.0 * math.pi * wheel_diameter
    return motor_rpm, wheel_rpm, speed_mps


def compute_tacho_speed(current_tacho, previous_tacho, dt, pole_pairs, gear_ratio, wheel_diameter):
    """
    Convert tachometer delta to an independent RPM/speed estimate.

    speed_mps is ERPM-based speed. tacho_speed_mps is tachometer-delta-based
    speed. Comparing both at low speed helps decide which signal is more
    stable. If tacho_speed differs from ERPM speed by a constant factor, adjust
    tacho scale.
    """
    nan = float("nan")
    if (
        current_tacho is None
        or previous_tacho is None
        or dt is None
        or math.isnan(current_tacho)
        or math.isnan(previous_tacho)
        or dt <= 1e-6
    ):
        return nan, nan, nan, nan, nan, nan

    tacho_delta = current_tacho - previous_tacho
    tacho_erpm = (tacho_delta / dt) * 60.0
    tacho_motor_rpm = tacho_erpm / pole_pairs
    tacho_wheel_rpm = tacho_motor_rpm / gear_ratio
    tacho_speed_mps = tacho_wheel_rpm / 60.0 * math.pi * wheel_diameter
    return tacho_delta, dt, tacho_erpm, tacho_motor_rpm, tacho_wheel_rpm, tacho_speed_mps


def input_worker(command_queue, stop_event):
    """
    Blocking stdin reader running in a daemon thread.

    The main thread owns serial and CSV I/O; this thread only passes commands.
    """
    while not stop_event.is_set():
        try:
            line = input("> ")
        except EOFError:
            command_queue.put(("stop", "stdin closed"))
            return
        except KeyboardInterrupt:
            command_queue.put(("stop", "keyboard interrupt"))
            return

        command_queue.put(("line", line.strip()))


def print_startup_help(args):
    print()
    print("============================================================")
    print("WARNING: 바퀴를 띄운 상태에서 먼저 테스트하세요.")
    print("         Start with small duty values and be ready to stop.")
    print("============================================================")
    print(f"Port: {args.port}, baud: {args.baud}, CSV: {args.csv}")
    print(f"max-duty: +/-{args.max_duty:.3f}, ramp-step: {args.ramp_step:.4f}")
    print("duty-output: inverted (+input sends -VESC duty)")
    print(f"pole-pairs: {args.pole_pairs:.1f}")
    print(f"gear-ratio: {args.gear_ratio:.1f}")
    print()
    print("Commands:")
    print("  number          duty 변경, 예: 0.05, 0.08, -0.03")
    print("  zero or z       target duty를 0으로 변경, 프로그램은 계속 실행")
    print("  mark <memo>     CSV event/note 컬럼에 메모 한 줄 기록")
    print("  status          현재 target/applied/ERPM/speed/voltage/current 출력")
    print("  stop/q/quit/exit 안전 종료")
    print()


def print_status(state, prefix="STATUS"):
    print(
        f"{prefix}: target={state.target_duty:+.4f} "
        f"applied={state.applied_duty:+.4f} "
        f"vesc_duty_now={state.vesc_duty_now:+.4f} "
        f"erpm={state.erpm:+.0f} "
        f"speed={state.speed_mps:+.3f} m/s "
        f"tacho_speed={state.tacho_speed_mps:+.3f} m/s "
        f"voltage={state.input_voltage:.2f} V "
        f"motor_current={state.current_motor:+.2f} A "
        f"input_current={state.current_in:+.2f} A"
    )


def make_csv_row(
    start_time,
    target_duty,
    applied_duty,
    values,
    motor_rpm,
    wheel_rpm,
    speed_mps,
    tacho_delta,
    tacho_dt,
    tacho_erpm,
    tacho_motor_rpm,
    tacho_wheel_rpm,
    tacho_speed_mps,
    event="",
    note="",
):
    now = datetime.now()
    elapsed = time.monotonic() - start_time
    return {
        "timestamp_iso": now.isoformat(timespec="milliseconds"),
        "elapsed_sec": f"{elapsed:.6f}",
        "target_duty": f"{target_duty:.6f}",
        "applied_duty": f"{applied_duty:.6f}",
        "vesc_duty_now": f"{values.duty_now:.6f}",
        "erpm": f"{values.erpm:.3f}",
        "motor_rpm": f"{motor_rpm:.3f}",
        "wheel_rpm": f"{wheel_rpm:.3f}",
        "speed_mps": f"{speed_mps:.6f}",
        "input_voltage": f"{values.input_voltage:.3f}",
        "current_motor": f"{values.current_motor:.3f}",
        "current_in": f"{values.current_in:.3f}",
        "temp_mos": f"{values.temp_mos:.2f}",
        "temp_motor": f"{values.temp_motor:.2f}",
        "event": event,
        "note": note,
        "tachometer": f"{values.tachometer:.3f}",
        "tachometer_abs": f"{values.tachometer_abs:.3f}",
        "tacho_delta": f"{tacho_delta:.3f}",
        "tacho_dt": f"{tacho_dt:.6f}",
        "tacho_erpm": f"{tacho_erpm:.3f}",
        "tacho_motor_rpm": f"{tacho_motor_rpm:.3f}",
        "tacho_wheel_rpm": f"{tacho_wheel_rpm:.3f}",
        "tacho_speed_mps": f"{tacho_speed_mps:.6f}",
    }


def write_csv_row(writer, csv_file, row):
    writer.writerow(row)
    csv_file.flush()


def safe_stop_vesc(ser, applied_duty, ramp_step, loop_hz):
    """Ramp down, then send explicit duty 0 several times before closing."""
    if ser is None:
        return

    print("Safe stop: ramping duty to 0.0")
    sleep_dt = 1.0 / max(loop_hz, 1.0)
    duty = applied_duty

    try:
        while abs(duty) > 1e-6:
            duty = move_toward(duty, 0.0, max(ramp_step, 0.0005))
            set_vesc_duty(ser, duty)
            time.sleep(sleep_dt)

        for _ in range(3):
            set_vesc_duty(ser, 0.0)
            time.sleep(0.05)
    except Exception as exc:
        print(f"WARNING: stop command failed: {exc!r}")


def process_command(line, target_duty, max_duty, pending_events, stop_event, state):
    """Handle one user command and return the possibly updated target duty."""
    if not line:
        return target_duty

    lower = line.lower()
    if lower in STOP_COMMANDS:
        print("Stop command received.")
        pending_events.put(("stop", lower))
        stop_event.set()
        return 0.0

    if lower in ZERO_COMMANDS:
        print("Target duty set to 0.0")
        pending_events.put(("zero", lower))
        return 0.0

    if lower.startswith("mark "):
        note = line[5:].strip()
        if note:
            print(f"Marked: {note}")
            pending_events.put(("mark", note))
        else:
            print("Usage: mark <memo>")
        return target_duty

    if lower == "status":
        print_status(state)
        return target_duty

    try:
        requested = float(line)
    except ValueError:
        print(f"Unknown command: {line!r}")
        return target_duty

    clamped = clamp(requested, -max_duty, max_duty)
    if clamped != requested:
        print(f"Requested duty {requested:+.4f} clamped to {clamped:+.4f}")
    else:
        print(f"Target duty set to {clamped:+.4f}")
    pending_events.put(("target_duty", f"{clamped:+.6f}"))
    return clamped


def make_default_csv_name():
    """Create the default timestamp-based CSV filename."""
    return datetime.now().strftime("vesc_interactive_duty_%Y%m%d_%H%M%S.csv")


def resolve_csv_path(csv_arg):
    """Resolve CSV output path from --csv or an interactive prompt."""
    if csv_arg:
        csv_path = csv_arg
    else:
        default_csv = make_default_csv_name()
        csv_path = input(f"CSV filename? [default: {default_csv}]: ").strip()
        if not csv_path:
            csv_path = default_csv

    if not csv_path.lower().endswith(".csv"):
        csv_path += ".csv"

    print(f"CSV will be saved to: {csv_path}")
    return csv_path


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Standalone interactive VESC duty logger for Jetson."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="VESC serial port")
    parser.add_argument("--baud", default=115200, type=int, help="VESC serial baudrate")
    parser.add_argument("--csv", default=None, help="CSV output path")
    parser.add_argument("--loop-hz", default=50.0, type=float, help="control loop rate")
    parser.add_argument("--log-hz", default=20.0, type=float, help="CSV log rate")
    parser.add_argument("--print-hz", default=1.0, type=float, help="terminal print rate")
    parser.add_argument("--max-duty", default=0.20, type=float, help="absolute duty clamp")
    parser.add_argument("--ramp-step", default=0.002, type=float, help="duty change per loop")
    # 3974-2500KV RC car sensored BLDC is assumed to be 4-pole, so pole_pairs=2.
    # Verify with motor spec or wheel RPM test.
    parser.add_argument("--pole-pairs", default=2.0, type=float, help="motor pole pairs")
    parser.add_argument("--gear-ratio", default=12.0, type=float, help="motor to wheel gear ratio")
    parser.add_argument("--wheel-diameter", default=0.10, type=float, help="wheel diameter in meters")
    parser.add_argument(
        "--invert-erpm",
        action="store_true",
        help="invert ERPM sign for speed calculation",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    args.csv = resolve_csv_path(args.csv)

    if args.loop_hz <= 0 or args.log_hz <= 0 or args.print_hz <= 0:
        raise SystemExit("ERROR: --loop-hz, --log-hz, and --print-hz must be > 0")
    if args.max_duty <= 0:
        raise SystemExit("ERROR: --max-duty must be > 0")
    if args.ramp_step <= 0:
        raise SystemExit("ERROR: --ramp-step must be > 0")
    if args.pole_pairs == 0 or args.gear_ratio == 0:
        raise SystemExit("ERROR: --pole-pairs and --gear-ratio must be non-zero")

    print_startup_help(args)

    ser = None
    target_duty = 0.0
    applied_duty = 0.0
    last_values = VescValues()
    prev_tacho = None
    prev_tacho_time = None
    tacho_delta = float("nan")
    tacho_dt = float("nan")
    tacho_erpm = float("nan")
    tacho_motor_rpm = float("nan")
    tacho_wheel_rpm = float("nan")
    tacho_speed_mps = float("nan")
    state = RuntimeState()
    command_queue = queue.Queue()
    pending_events = queue.Queue()
    stop_event = threading.Event()
    input_thread = threading.Thread(
        target=input_worker,
        args=(command_queue, stop_event),
        daemon=True,
    )

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.02)
        print(f"Serial opened: {ser.name}")
        time.sleep(0.2)

        input_thread.start()

        with open(args.csv, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            csv_file.flush()

            start_time = time.monotonic()
            next_loop_time = start_time
            next_log_time = start_time
            next_print_time = start_time
            log_period = 1.0 / args.log_hz
            print_period = 1.0 / args.print_hz

            print("Logging started. Enter a duty value or command.")
            while not stop_event.is_set():
                loop_start = time.monotonic()

                while True:
                    try:
                        kind, payload = command_queue.get_nowait()
                    except queue.Empty:
                        break

                    if kind == "stop":
                        pending_events.put(("stop", payload))
                        target_duty = 0.0
                        stop_event.set()
                    elif kind == "line":
                        target_duty = process_command(
                            payload,
                            target_duty,
                            args.max_duty,
                            pending_events,
                            stop_event,
                            state,
                        )

                target_duty = clamp(target_duty, -args.max_duty, args.max_duty)
                applied_duty = move_toward(applied_duty, target_duty, args.ramp_step)
                set_vesc_duty(ser, applied_duty)

                should_log = loop_start >= next_log_time
                values = get_values(ser)
                if values is not None:
                    last_values = values
                    current_tacho_time = time.monotonic()
                    if not math.isnan(values.tachometer):
                        if prev_tacho is not None and prev_tacho_time is not None:
                            (
                                tacho_delta,
                                tacho_dt,
                                tacho_erpm,
                                tacho_motor_rpm,
                                tacho_wheel_rpm,
                                tacho_speed_mps,
                            ) = compute_tacho_speed(
                                values.tachometer,
                                prev_tacho,
                                current_tacho_time - prev_tacho_time,
                                args.pole_pairs,
                                args.gear_ratio,
                                args.wheel_diameter,
                            )
                        prev_tacho = values.tachometer
                        prev_tacho_time = current_tacho_time
                    else:
                        tacho_delta = float("nan")
                        tacho_dt = float("nan")
                        tacho_erpm = float("nan")
                        tacho_motor_rpm = float("nan")
                        tacho_wheel_rpm = float("nan")
                        tacho_speed_mps = float("nan")

                motor_rpm, wheel_rpm, speed_mps = compute_speed(
                    last_values.erpm,
                    args.pole_pairs,
                    args.gear_ratio,
                    args.wheel_diameter,
                    args.invert_erpm,
                )

                state.target_duty = target_duty
                state.applied_duty = applied_duty
                state.vesc_duty_now = last_values.duty_now
                state.erpm = last_values.erpm
                state.speed_mps = speed_mps
                state.tacho_speed_mps = tacho_speed_mps
                state.input_voltage = last_values.input_voltage
                state.current_motor = last_values.current_motor
                state.current_in = last_values.current_in

                if should_log:
                    next_log_time += log_period
                    row = make_csv_row(
                        start_time,
                        target_duty,
                        applied_duty,
                        last_values,
                        motor_rpm,
                        wheel_rpm,
                        speed_mps,
                        tacho_delta,
                        tacho_dt,
                        tacho_erpm,
                        tacho_motor_rpm,
                        tacho_wheel_rpm,
                        tacho_speed_mps,
                    )
                    write_csv_row(writer, csv_file, row)

                while True:
                    try:
                        event, note = pending_events.get_nowait()
                    except queue.Empty:
                        break

                    row = make_csv_row(
                        start_time,
                        target_duty,
                        applied_duty,
                        last_values,
                        motor_rpm,
                        wheel_rpm,
                        speed_mps,
                        tacho_delta,
                        tacho_dt,
                        tacho_erpm,
                        tacho_motor_rpm,
                        tacho_wheel_rpm,
                        tacho_speed_mps,
                        event=event,
                        note=note,
                    )
                    write_csv_row(writer, csv_file, row)

                if loop_start >= next_print_time:
                    next_print_time += print_period
                    print_status(state, prefix="RUN")

                next_loop_time += 1.0 / args.loop_hz
                sleep_time = next_loop_time - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_loop_time = time.monotonic()

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received.")
        stop_event.set()
    finally:
        stop_event.set()
        safe_stop_vesc(ser, applied_duty, args.ramp_step, args.loop_hz)
        if ser is not None:
            try:
                ser.close()
                print("Serial closed.")
            except Exception as exc:
                print(f"WARNING: serial close failed: {exc!r}")
        print(f"CSV saved: {args.csv}")


if __name__ == "__main__":
    main()


# Example:
# python3 vesc_interactive_duty_logger.py --port /dev/ttyACM0 --baud 115200 --max-duty 0.20 --pole-pairs 2 --gear-ratio 12 --wheel-diameter 0.10
