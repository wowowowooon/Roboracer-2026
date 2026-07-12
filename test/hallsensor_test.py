#!/usr/bin/env python3
"""
VESC hall sensor / ERPM proof test using raw UART packets.

Real VESC communication uses pyserial only. Mock mode runs without serial or
hardware.
"""

import argparse
import csv
import glob
import math
import random
import statistics
import struct
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional


COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
PORT_CANDIDATES = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]

CSV_COLUMNS = [
    "timestamp",
    "elapsed_time",
    "mode",
    "mock",
    "duty_cmd",
    "measured_erpm",
    "motor_rpm",
    "wheel_rpm",
    "vehicle_speed_mps",
    "duty_now",
    "motor_current",
    "input_current",
    "input_voltage",
    "warning",
]


class PacketError(Exception):
    pass


class ParseError(Exception):
    pass


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def make_packet(payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("small VESC packet payload must be <= 255 bytes")
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + struct.pack(">H", crc) + bytes([0x03])


def read_packet(serial_port, timeout: float = 0.2) -> bytes:
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        start = serial_port.read(1)
        if not start:
            continue
        if start == b"\x02":
            break
    else:
        raise PacketError("timeout waiting for VESC packet start")

    length_raw = _read_exact(serial_port, 1, deadline)
    length = length_raw[0]
    payload = _read_exact(serial_port, length, deadline)
    crc_raw = _read_exact(serial_port, 2, deadline)
    end = _read_exact(serial_port, 1, deadline)

    if end != b"\x03":
        raise PacketError(f"bad packet end byte: {end.hex()}")

    got_crc = struct.unpack(">H", crc_raw)[0]
    expected_crc = crc16(payload)
    if got_crc != expected_crc:
        raise PacketError(f"crc mismatch got=0x{got_crc:04x} expected=0x{expected_crc:04x}")

    return payload


def _read_exact(serial_port, size: int, deadline: float) -> bytes:
    chunks = bytearray()
    while len(chunks) < size and time.monotonic() < deadline:
        chunk = serial_port.read(size - len(chunks))
        if chunk:
            chunks.extend(chunk)
    if len(chunks) != size:
        raise PacketError(f"timeout reading {size} bytes")
    return bytes(chunks)


def send_get_values(serial_port) -> None:
    serial_port.write(make_packet(bytes([COMM_GET_VALUES])))
    flush = getattr(serial_port, "flush", None)
    if flush is not None:
        flush()


def send_set_duty(serial_port, duty: float) -> None:
    value = int(duty * 100000)
    payload = bytes([COMM_SET_DUTY]) + struct.pack(">i", value)
    serial_port.write(make_packet(payload))
    flush = getattr(serial_port, "flush", None)
    if flush is not None:
        flush()


def parse_get_values(payload: bytes, layout: str = "auto") -> Dict[str, Optional[float]]:
    if not payload:
        raise ParseError("empty payload")
    if payload[0] != COMM_GET_VALUES:
        raise ParseError(f"unexpected response command {payload[0]}")

    if layout == "modern":
        return _parse_get_values_modern(payload)
    if layout == "legacy":
        return _parse_get_values_legacy(payload)

    modern_error: Optional[Exception] = None
    try:
        modern = _parse_get_values_modern(payload)
        if _values_look_reasonable(modern):
            return modern
    except Exception as exc:
        modern_error = exc

    try:
        legacy = _parse_get_values_legacy(payload)
        if _values_look_reasonable(legacy):
            legacy["layout_warning"] = "auto_legacy"
        else:
            legacy["layout_warning"] = "auto_legacy_suspicious"
        return legacy
    except Exception as legacy_error:
        if modern_error is not None:
            raise ParseError(f"modern failed: {modern_error}; legacy failed: {legacy_error}") from legacy_error
        raise ParseError(f"legacy failed: {legacy_error}") from legacy_error


def _parse_get_values_modern(payload: bytes) -> Dict[str, Optional[float]]:
    if len(payload) < 29:
        raise ParseError(f"modern payload too short: {len(payload)}")
    return {
        "motor_current": _i32(payload, 5) / 100.0,
        "input_current": _i32(payload, 9) / 100.0,
        "duty_now": _i16(payload, 21) / 1000.0,
        "measured_erpm": float(_i32(payload, 23)),
        "input_voltage": _i16(payload, 27) / 10.0,
    }


def _parse_get_values_legacy(payload: bytes) -> Dict[str, Optional[float]]:
    if len(payload) < 21:
        raise ParseError(f"legacy payload too short: {len(payload)}")
    return {
        "motor_current": _i32(payload, 5) / 100.0,
        "input_current": _i32(payload, 9) / 100.0,
        "duty_now": _i16(payload, 13) / 1000.0,
        "measured_erpm": float(_i32(payload, 15)),
        "input_voltage": _i16(payload, 19) / 10.0,
    }


def _i16(payload: bytes, offset: int) -> int:
    return struct.unpack(">h", payload[offset : offset + 2])[0]


def _i32(payload: bytes, offset: int) -> int:
    return struct.unpack(">i", payload[offset : offset + 4])[0]


def _values_look_reasonable(values: Dict[str, Optional[float]]) -> bool:
    erpm = values.get("measured_erpm")
    duty = values.get("duty_now")
    vin = values.get("input_voltage")
    motor_current = values.get("motor_current")
    input_current = values.get("input_current")

    if erpm is None or duty is None or vin is None:
        return False
    if abs(erpm) > 500000:
        return False
    if abs(duty) > 1.2:
        return False
    if not (0.0 <= vin <= 100.0):
        return False
    if motor_current is not None and abs(motor_current) > 1000.0:
        return False
    if input_current is not None and abs(input_current) > 1000.0:
        return False
    return True


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ramp_value(start: float, target: float, elapsed: float, ramp_time: float) -> float:
    if ramp_time <= 0:
        return target
    return start + (target - start) * clamp(elapsed / ramp_time, 0.0, 1.0)


def erpm_to_motor_rpm(erpm: Optional[float], pole_pairs: float) -> Optional[float]:
    if erpm is None:
        return None
    if pole_pairs <= 0:
        raise ValueError("--pole-pairs must be greater than 0")
    return erpm / pole_pairs


def erpm_to_vehicle_speed(
    erpm: Optional[float],
    pole_pairs: float,
    gear_ratio: float,
    wheel_radius: float,
) -> Optional[float]:
    motor_rpm = erpm_to_motor_rpm(erpm, pole_pairs)
    if motor_rpm is None:
        return None
    if gear_ratio <= 0:
        raise ValueError("--gear-ratio must be greater than 0")
    if wheel_radius < 0:
        raise ValueError("--wheel-radius must be 0 or greater")
    wheel_rpm = motor_rpm / gear_ratio
    return wheel_rpm * 2.0 * math.pi * wheel_radius / 60.0


def parse_step_duty_list(raw: Optional[str]) -> List[float]:
    if not raw:
        return []
    values: List[float] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        value = float(item)
        if value < 0:
            raise ValueError("negative duty values are not allowed")
        values.append(value)
    return values


def find_vesc_ports() -> List[str]:
    ports: List[str] = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports.extend(glob.glob(pattern))
    ordered = [p for p in PORT_CANDIDATES if p in ports]
    ordered.extend(sorted(p for p in ports if p not in ordered))
    return ordered


def check_serial_dependency() -> bool:
    import importlib.util

    return importlib.util.find_spec("serial") is not None


@dataclass
class Sample:
    timestamp: float
    elapsed_time: float
    mode: str
    mock: bool
    duty_cmd: float
    measured_erpm: Optional[float]
    motor_rpm: Optional[float]
    wheel_rpm: Optional[float]
    vehicle_speed_mps: Optional[float]
    duty_now: Optional[float]
    motor_current: Optional[float]
    input_current: Optional[float]
    input_voltage: Optional[float]
    warning: str = ""


class MockVESC:
    def __init__(self) -> None:
        self._start = time.monotonic()
        self._duty_cmd = 0.0
        self._erpm_state = 0.0

    def set_duty(self, duty: float) -> None:
        self._duty_cmd = duty

    def get_values(self, mode: str = "hand-roll", duty_cmd: Optional[float] = None) -> Dict[str, Optional[float]]:
        t = time.monotonic() - self._start
        if mode == "hand-roll":
            erpm = self._hand_roll_erpm(t)
            return {
                "measured_erpm": erpm,
                "duty_now": 0.0,
                "motor_current": random.uniform(-0.12, 0.12),
                "input_current": random.uniform(-0.06, 0.06),
                "input_voltage": 16.0 + random.gauss(0.0, 0.08),
                "warning": "",
            }

        cmd = self._duty_cmd if duty_cmd is None else duty_cmd
        target_erpm = cmd * 50000.0
        self._erpm_state += (target_erpm - self._erpm_state) * 0.18
        if abs(cmd) < 1e-6:
            self._erpm_state *= 0.78
        return {
            "measured_erpm": self._erpm_state + random.gauss(0.0, 55.0),
            "duty_now": cmd + random.gauss(0.0, 0.002),
            "motor_current": max(0.0, cmd * 34.0 + random.gauss(0.0, 0.15)),
            "input_current": max(0.0, cmd * 16.0 + random.gauss(0.0, 0.08)),
            "input_voltage": 16.0 + random.gauss(0.0, 0.08),
            "warning": "",
        }

    @staticmethod
    def _hand_roll_erpm(t: float) -> float:
        phase = t % 12.0
        if phase < 3.5:
            envelope = math.sin(math.pi * phase / 3.5)
            erpm = 2400.0 * envelope + 450.0 * math.sin(t * 5.0)
        elif phase < 5.0:
            erpm = 0.0
        elif phase < 8.5:
            local = phase - 5.0
            envelope = math.sin(math.pi * local / 3.5)
            erpm = -2100.0 * envelope + 350.0 * math.sin(t * 4.3)
        else:
            erpm = 0.0
        return erpm + random.gauss(0.0, 35.0)

    def close(self) -> None:
        pass


class RawVESC:
    def __init__(self, port: str, baud: int, layout: str, raw_debug: bool) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Missing dependency: install with `python3 -m pip install pyserial`") from exc

        self.port = port
        self.baud = baud
        self.layout = layout
        self.raw_debug = raw_debug
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.05)

    def set_duty(self, duty: float) -> None:
        send_set_duty(self.serial, duty)

    def get_values(self, mode: str = "hand-roll", duty_cmd: Optional[float] = None) -> Dict[str, Optional[float]]:
        del mode, duty_cmd
        send_get_values(self.serial)
        try:
            payload = read_packet(self.serial, timeout=0.25)
            if self.raw_debug:
                print(f"raw_payload_hex={payload.hex()}")
            values = parse_get_values(payload, self.layout)
            warning = str(values.pop("layout_warning", "") or "")
            values["warning"] = warning
            return values
        except (PacketError, ParseError, struct.error) as exc:
            warning = "parse_error"
            print(f"WARNING: {warning}: {exc}", file=sys.stderr)
            return {
                "measured_erpm": None,
                "duty_now": None,
                "motor_current": None,
                "input_current": None,
                "input_voltage": None,
                "warning": warning,
            }

    def close(self) -> None:
        self.serial.close()


def detect_warning(
    mode: str,
    duty_cmd: float,
    erpm: Optional[float],
    prev_erpm: Optional[float],
    hz: float,
    extra_warning: str = "",
) -> str:
    warnings: List[str] = [w for w in extra_warning.split("|") if w]
    if erpm is None:
        if "no_erpm" not in warnings:
            warnings.append("no_erpm")
        return "|".join(warnings)

    if prev_erpm is not None:
        delta = abs(erpm - prev_erpm)
        spike_threshold = max(2500.0, 35000.0 / max(hz, 1.0))
        if delta > spike_threshold:
            warnings.append("spike")

    if mode == "duty" and duty_cmd > 0.01 and abs(erpm) < 80.0:
        warnings.append("dropout")

    if mode == "hand-roll":
        if abs(erpm) < 80.0:
            warnings.append("near_zero")
        elif erpm > 0:
            warnings.append("positive_erpm")
        else:
            warnings.append("negative_erpm")

    return "|".join(warnings)


def write_csv_row(writer: csv.DictWriter, sample: Sample) -> None:
    writer.writerow(
        {
            "timestamp": f"{sample.timestamp:.6f}",
            "elapsed_time": f"{sample.elapsed_time:.3f}",
            "mode": sample.mode,
            "mock": str(sample.mock),
            "duty_cmd": f"{sample.duty_cmd:.5f}",
            "measured_erpm": _fmt(sample.measured_erpm, 2),
            "motor_rpm": _fmt(sample.motor_rpm, 2),
            "wheel_rpm": _fmt(sample.wheel_rpm, 2),
            "vehicle_speed_mps": _fmt(sample.vehicle_speed_mps, 4),
            "duty_now": _fmt(sample.duty_now, 5),
            "motor_current": _fmt(sample.motor_current, 3),
            "input_current": _fmt(sample.input_current, 3),
            "input_voltage": _fmt(sample.input_voltage, 3),
            "warning": sample.warning,
        }
    )


def _fmt(value: Optional[float], digits: int) -> str:
    if value is None:
        return ""
    return f"{value:.{digits}f}"


def print_sample(sample: Sample) -> None:
    print(
        f"{sample.elapsed_time:7.3f}s "
        f"mode={sample.mode:9s} "
        f"duty_cmd={sample.duty_cmd: .4f} "
        f"erpm={_fmt(sample.measured_erpm, 1):>9s} "
        f"motor_rpm={_fmt(sample.motor_rpm, 1):>8s} "
        f"wheel_rpm={_fmt(sample.wheel_rpm, 1):>8s} "
        f"speed={_fmt(sample.vehicle_speed_mps, 3):>7s}m/s "
        f"vin={_fmt(sample.input_voltage, 2):>6s}V "
        f"motor_i={_fmt(sample.motor_current, 2):>7s}A "
        f"warning={sample.warning}"
    )


def build_sample(
    args: argparse.Namespace,
    mode: str,
    start_time: float,
    duty_cmd: float,
    values: Dict[str, Optional[float]],
    warning: str,
) -> Sample:
    erpm = values.get("measured_erpm")
    motor_rpm = erpm_to_motor_rpm(erpm, args.pole_pairs)
    wheel_rpm = None if motor_rpm is None else motor_rpm / args.gear_ratio
    speed = erpm_to_vehicle_speed(erpm, args.pole_pairs, args.gear_ratio, args.wheel_radius)
    return Sample(
        timestamp=time.time(),
        elapsed_time=time.monotonic() - start_time,
        mode=mode,
        mock=args.mock,
        duty_cmd=duty_cmd,
        measured_erpm=erpm,
        motor_rpm=motor_rpm,
        wheel_rpm=wheel_rpm,
        vehicle_speed_mps=speed,
        duty_now=values.get("duty_now"),
        motor_current=values.get("motor_current"),
        input_current=values.get("input_current"),
        input_voltage=values.get("input_voltage"),
        warning=warning,
    )


def run_hand_roll_mode(args: argparse.Namespace, vesc: object, writer: csv.DictWriter) -> List[Sample]:
    print("HAND-ROLL MODE: no motor command will be sent")
    print("Only COMM_GET_VALUES is sent. Rotate wheel forward/backward by hand and watch ERPM sign.")
    samples: List[Sample] = []
    start_time = time.monotonic()
    prev_erpm: Optional[float] = None
    interval = 1.0 / args.hz

    while time.monotonic() - start_time < args.duration:
        loop_start = time.monotonic()
        values = vesc.get_values(mode="hand-roll", duty_cmd=0.0)  # type: ignore[attr-defined]
        warning = detect_warning(
            "hand-roll",
            0.0,
            values.get("measured_erpm"),
            prev_erpm,
            args.hz,
            str(values.get("warning") or ""),
        )
        sample = build_sample(args, "hand-roll", start_time, 0.0, values, warning)
        write_csv_row(writer, sample)
        print_sample(sample)
        samples.append(sample)
        if sample.measured_erpm is not None:
            prev_erpm = sample.measured_erpm
        _sleep_remaining(interval, loop_start)

    return samples


def run_duty_mode(args: argparse.Namespace, vesc: object, writer: csv.DictWriter) -> List[Sample]:
    if args.mock:
        print("DUTY MODE MOCK: no real motor command will be sent.")
    else:
        print("DUTY MODE SAFETY WARNING: 바퀴를 공중에 띄우라")
        print("Only COMM_SET_DUTY is sent. set_current/set_rpm/brake commands are not used.")
        for remaining in (3, 2, 1):
            print(f"Starting in {remaining}...")
            time.sleep(1.0)

    samples: List[Sample] = []
    start_time = time.monotonic()
    prev_erpm: Optional[float] = None
    interval = 1.0 / args.hz

    step_duties = parse_step_duty_list(args.step_duty_list)
    if step_duties:
        plan = [(clamp(d, 0.0, args.max_duty), args.step_duration) for d in step_duties]
    else:
        requested = args.duty if args.duty is not None else 0.05
        if requested < 0:
            raise ValueError("negative duty values are not allowed")
        plan = [(clamp(requested, 0.0, args.max_duty), args.duration)]

    last_duty = 0.0
    for target_duty, hold_duration in plan:
        ramp_start = time.monotonic()
        while time.monotonic() - ramp_start < args.ramp_time:
            loop_start = time.monotonic()
            duty_cmd = ramp_value(last_duty, target_duty, time.monotonic() - ramp_start, args.ramp_time)
            _set_duty(vesc, duty_cmd)
            prev_erpm = _record_duty_sample(args, vesc, writer, samples, start_time, duty_cmd, prev_erpm)
            _sleep_remaining(interval, loop_start)

        hold_start = time.monotonic()
        while time.monotonic() - hold_start < hold_duration:
            loop_start = time.monotonic()
            duty_cmd = target_duty
            _set_duty(vesc, duty_cmd)
            prev_erpm = _record_duty_sample(args, vesc, writer, samples, start_time, duty_cmd, prev_erpm)
            _sleep_remaining(interval, loop_start)

        last_duty = target_duty

    return samples


def _record_duty_sample(
    args: argparse.Namespace,
    vesc: object,
    writer: csv.DictWriter,
    samples: List[Sample],
    start_time: float,
    duty_cmd: float,
    prev_erpm: Optional[float],
) -> Optional[float]:
    values = vesc.get_values(mode="duty", duty_cmd=duty_cmd)  # type: ignore[attr-defined]
    warning = detect_warning(
        "duty",
        duty_cmd,
        values.get("measured_erpm"),
        prev_erpm,
        args.hz,
        str(values.get("warning") or ""),
    )
    sample = build_sample(args, "duty", start_time, duty_cmd, values, warning)
    write_csv_row(writer, sample)
    print_sample(sample)
    samples.append(sample)
    return sample.measured_erpm if sample.measured_erpm is not None else prev_erpm


def _set_duty(vesc: object, duty: float) -> None:
    vesc.set_duty(duty)  # type: ignore[attr-defined]


def safe_stop(vesc: object, mode: str, mock: bool) -> None:
    if mode != "duty":
        return
    print("Safe stop: sending duty 0 at least 3 times.")
    for idx in range(3):
        try:
            _set_duty(vesc, 0.0)
            print(f"  duty 0 sent [{idx + 1}/3]")
        except Exception as exc:
            print(f"  duty 0 failed [{idx + 1}/3]: {exc}", file=sys.stderr)
        time.sleep(0.05 if mock else 0.1)


def print_summary(samples: List[Sample], csv_path: str) -> None:
    erpms = [s.measured_erpm for s in samples if s.measured_erpm is not None]
    motor_rpms = [s.motor_rpm for s in samples if s.motor_rpm is not None]
    speeds = [s.vehicle_speed_mps for s in samples if s.vehicle_speed_mps is not None]
    spike_count = sum(1 for s in samples if "spike" in s.warning.split("|"))
    dropout_count = sum(1 for s in samples if "dropout" in s.warning.split("|"))

    print("\n=== SUMMARY ===")
    print(f"total_samples: {len(samples)}")
    print(f"avg_erpm: {_summary_mean(erpms)}")
    print(f"max_erpm: {_summary_max(erpms)}")
    print(f"min_erpm: {_summary_min(erpms)}")
    print(f"avg_motor_rpm: {_summary_mean(motor_rpms)}")
    print(f"avg_vehicle_speed_mps: {_summary_mean(speeds)}")
    print(f"erpm_stddev: {_summary_stddev(erpms)}")
    print(f"spike_count: {spike_count}")
    print(f"dropout_count: {dropout_count}")
    print(f"csv_path: {csv_path}")


def _summary_mean(values: List[float]) -> str:
    return "n/a" if not values else f"{statistics.mean(values):.3f}"


def _summary_max(values: List[float]) -> str:
    return "n/a" if not values else f"{max(values):.3f}"


def _summary_min(values: List[float]) -> str:
    return "n/a" if not values else f"{min(values):.3f}"


def _summary_stddev(values: List[float]) -> str:
    if len(values) < 2:
        return "n/a"
    return f"{statistics.stdev(values):.3f}"


def _sleep_remaining(interval: float, loop_start: float) -> None:
    remaining = interval - (time.monotonic() - loop_start)
    if remaining > 0:
        time.sleep(remaining)


def make_vesc(args: argparse.Namespace) -> object:
    if args.mock:
        return MockVESC()
    return RawVESC(args.port, args.baud, args.get_values_layout, args.raw_debug)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Raw UART VESC hall sensor / ERPM proof test")
    parser.add_argument("--mode", choices=("hand-roll", "duty"), required=True)
    parser.add_argument("--mock", action="store_true", help="run without real VESC hardware")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--csv", default="hallsensor_raw.csv")
    parser.add_argument("--pole-pairs", type=float, default=7.0)
    parser.add_argument("--gear-ratio", type=float, default=1.0)
    parser.add_argument("--wheel-radius", type=float, default=0.05)
    parser.add_argument("--duty", type=float, default=None)
    parser.add_argument("--ramp-time", type=float, default=1.0)
    parser.add_argument("--max-duty", type=float, default=0.15)
    parser.add_argument("--step-duty-list", default=None)
    parser.add_argument("--step-duration", type=float, default=5.0)
    parser.add_argument("--list-ports", action="store_true", help="print detected VESC-like serial ports and exit")
    parser.add_argument("--get-values-layout", choices=("auto", "modern", "legacy"), default="auto")
    parser.add_argument("--raw-debug", action="store_true", help="print raw COMM_GET_VALUES response payload hex")
    args = parser.parse_args(argv)

    if args.hz <= 0:
        parser.error("--hz must be greater than 0")
    if args.duration <= 0:
        parser.error("--duration must be greater than 0")
    if args.ramp_time < 0:
        parser.error("--ramp-time must be 0 or greater")
    if args.max_duty < 0:
        parser.error("--max-duty must be 0 or greater")
    if args.step_duration <= 0:
        parser.error("--step-duration must be greater than 0")
    if args.duty is not None and args.duty < 0:
        parser.error("negative duty values are not allowed")
    try:
        parse_step_duty_list(args.step_duty_list)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    if args.list_ports:
        ports = find_vesc_ports()
        print("Detected ports: " + (", ".join(ports) if ports else "none"))
        print("Common candidates: " + ", ".join(PORT_CANDIDATES))
        print(f"Dependency check: serial={check_serial_dependency()}")
        if not check_serial_dependency():
            print("For real VESC mode, install: python3 -m pip install pyserial")
        return 0

    if args.mock:
        print("Dependency check: serial=not_required")
    else:
        serial_ok = check_serial_dependency()
        print(f"Dependency check: serial={serial_ok}")
        if not serial_ok:
            print("For real VESC mode, install: python3 -m pip install pyserial")

    vesc: Optional[object] = None
    samples: List[Sample] = []
    exit_code = 0

    try:
        vesc = make_vesc(args)
        with open(args.csv, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            if args.mode == "hand-roll":
                samples = run_hand_roll_mode(args, vesc, writer)
            else:
                samples = run_duty_mode(args, vesc, writer)
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        exit_code = 130
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if vesc is not None:
            safe_stop(vesc, args.mode, args.mock)
            close = getattr(vesc, "close", None)
            if close is not None:
                close()
        print_summary(samples, args.csv)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())


# Commands
#
# Mock check:
# python3 hallsensor_test.py --mock --mode hand-roll --duration 5 --csv mock_raw_check.csv
#
# Port check:
# python3 hallsensor_test.py --mode hand-roll --list-ports
#
# Real hand-roll first:
# python3 hallsensor_test.py --mode hand-roll --port /dev/ttyACM0 --baud 115200 --hz 20 --duration 20 --pole-pairs 7 --gear-ratio 1.0 --wheel-radius 0.05 --csv real_hand_roll_raw.csv
#
# Raw debug if parsing looks wrong:
# python3 hallsensor_test.py --mode hand-roll --port /dev/ttyACM0 --baud 115200 --hz 5 --duration 5 --raw-debug --csv debug_hand_roll_raw.csv
#
# Duty only after hand-roll succeeds and with the wheel lifted:
# python3 hallsensor_test.py --mode duty --port /dev/ttyACM0 --baud 115200 --hz 20 --duty 0.05 --duration 5 --ramp-time 1.5 --max-duty 0.10 --pole-pairs 7 --gear-ratio 1.0 --wheel-radius 0.05 --csv real_duty_005_raw.csv

