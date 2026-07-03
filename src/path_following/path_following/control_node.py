#!/usr/bin/env python3
"""
실차 하드웨어 제어: /drive (AckermannDriveStamped) → ESP32 조향 + VESC duty.

CH5 (PPM index [4] ONLY) 로 수동/자율:
  - CH5 <= 1300 (1000, 수동): CH1→ESP, CH2→VESC
  - CH5 >= 1700 (2000, 자율): /drive→VESC + S:→ESP

ESP → Jetson: RC,ch1_us,ch2_us,ch5_us,0  (raw PWM us, 1000~2000)

터미널에서 Space → 비상정지(래치), r → 해제.
"""
from __future__ import annotations

import math
import select
import struct
import sys
import termios
import threading
import time
import tty

import rclpy
import serial
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# ============================================================
# USER TUNING — vehicle control (여기만 수정)
# ============================================================
CFG = {
    "drive_topic": "/drive",
    "esp_port": "/dev/ttyTHS1",
    "vesc_port": "/dev/ttyACM0",
    "esp_baud": 115200,         # esp32_steer_rc_uart.ino (RC 텔레메트리 + S: 동일 UART)
    "vesc_baud": 115200,
    # Stanley max_drive_speed / max_steering_angle 과 맞추면 1:1 스케일
    "max_speed_mps": 5.0,
    "max_steering_angle_rad": 0.6981,  # ±40° — ESP normToAngle: S±1 → 50°/130°
    "max_duty": 0.20,           # VESC duty 상한 20% (송신기 풀스틱 = 최대 20%)
    "speed_scale": 1.0,         # 추가 감쇠 (1.0=끔)
    "min_move_duty": 0.06,      # 정지마찰 극복용 최소 duty (speed>threshold 일 때)
    "min_move_speed_mps": 0.08,
    "debug_log_hz": 1.0,
    "max_steer": 1.0,           # ESP 조향 명령 범위 ±1.0 (jetson_steer_send 와 동일)
    "steer_cmd_format": "prefixed",  # plain: "0.500\n" | prefixed: "S:0.500\n"
    "invert_speed": True,       # VESC: 양수 duty 후진이면 True
    # 원본 ESP normToAngle: S:-1→좌(50°), S:+1→우(130°) — INVERT_RC_STEER 미적용
    # Stanley +steer=좌 → S:- 로 보내야 함 (False면 AUTO 조향 반대 → 옆으로 밀림)
    "invert_steer": False,
    "cmd_timeout_sec": 0.25,
    "timer_period_sec": 0.02,     # ESP loop 20ms — AUTO 조향 응답
    "serial_open_delay_sec": 2.0,
    "enable_keyboard_estop": True,
    "estop_reset_key": "r",
    # RC (ESP -> RC,ch1_us,ch2_us,mode_us,0)  raw PWM us
    "ch5_manual_us": 1300,        # CH5 <= 1300 수동(1000)
    "ch5_auto_us": 1700,          # CH5 >= 1700 자율(2000)
    "rc_center_ch2": 1500,
    "rc_min_val": 0,
    "rc_max_val": 3000,
    "rc_deadzone": 30,
    "rc_timeout_sec": 0.30,
    "invert_rc_throttle": True,   # 송신기 CH2: 앞=높은 us → 전진
    "auto_duty_ramp_sec": 1.0,    # AUTO: /drive duty → VESC (1초에 목표까지)
    "telemetry_topic": "/vehicle/telemetry",  # drive_monitor.py 구독
}


class VehicleControlNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_control_node")

        self._drive_topic = str(CFG["drive_topic"])
        self._max_speed_mps = max(float(CFG["max_speed_mps"]), 1e-3)
        self._max_steering_angle_rad = max(
            float(CFG["max_steering_angle_rad"]), 1e-3
        )
        self._max_duty = float(CFG["max_duty"])
        self._speed_scale = self.clamp(float(CFG["speed_scale"]), 0.0, 1.0)
        self._min_move_duty = max(0.0, float(CFG["min_move_duty"]))
        self._min_move_speed_mps = max(0.0, float(CFG["min_move_speed_mps"]))
        self._debug_log_hz = max(0.0, float(CFG["debug_log_hz"]))
        self._max_steer = float(CFG["max_steer"])
        self._steer_cmd_format = str(CFG.get("steer_cmd_format", "plain")).lower()
        self._invert_speed = bool(CFG["invert_speed"])
        self._invert_steer = bool(CFG["invert_steer"])
        self._cmd_timeout = float(CFG["cmd_timeout_sec"])
        self._estop_reset_key = str(CFG["estop_reset_key"]).lower()[:1] or "r"
        self._ch5_manual_us = int(CFG["ch5_manual_us"])
        self._ch5_auto_us = int(CFG["ch5_auto_us"])
        self._mode_auto_latched = False
        self._rc_center_ch2 = int(CFG["rc_center_ch2"])
        self._rc_min_val = int(CFG["rc_min_val"])
        self._rc_max_val = int(CFG["rc_max_val"])
        self._rc_deadzone = int(CFG["rc_deadzone"])
        self._rc_timeout = float(CFG["rc_timeout_sec"])
        self._invert_rc_throttle = bool(CFG.get("invert_rc_throttle", False))
        self._auto_duty_ramp_sec = max(0.0, float(CFG.get("auto_duty_ramp_sec", 1.0)))

        self._estop_lock = threading.Lock()
        self._estop_latched = False
        self._keyboard_running = False
        self._keyboard_thread: threading.Thread | None = None
        self._stdin_termios_old = None

        self.last_cmd_time = time.time()
        self._last_timer_time = time.time()
        self._auto_duty = 0.0
        self._auto_duty_applied = 0.0
        self._auto_steer = 0.0
        self.current_duty = 0.0
        self.current_steer = 0.0
        self._last_duty_int = None
        self._last_duty_packet = None
        self._last_speed_mps = 0.0
        self._last_steering_rad = 0.0
        self._last_debug_log_time = 0.0

        self._rc_ch1 = 1497
        self._rc_ch2 = 1497
        self._rc_ch5 = 1000
        self._last_rc_time = 0.0
        self._esp_rx_buffer = bytearray()
        self._control_mode = "INIT"

        self.get_logger().info(f"Opening ESP32 serial: {CFG['esp_port']}")
        self.esp = serial.Serial(
            str(CFG["esp_port"]), int(CFG["esp_baud"]), timeout=0.0
        )

        self.get_logger().info(f"Opening VESC serial: {CFG['vesc_port']}")
        self.vesc = serial.Serial(
            str(CFG["vesc_port"]), int(CFG["vesc_baud"]), timeout=0.0
        )

        time.sleep(float(CFG["serial_open_delay_sec"]))

        self.create_subscription(
            AckermannDriveStamped,
            self._drive_topic,
            self.drive_callback,
            10,
        )
        self.create_timer(float(CFG["timer_period_sec"]), self.timer_callback)
        tel_topic = str(CFG.get("telemetry_topic", "/vehicle/telemetry"))
        self._telemetry_pub = self.create_publisher(Float64MultiArray, tel_topic, 10)

        if bool(CFG["enable_keyboard_estop"]):
            self._start_keyboard_estop()

        self.get_logger().info("Vehicle control node started")
        self.get_logger().info(f"Subscribing: {self._drive_topic} (AckermannDriveStamped)")
        self.get_logger().info(
            f"Scale: speed≤{self._max_speed_mps} m/s × scale={self._speed_scale} "
            f"→ duty±{self._max_duty}, min_move_duty={self._min_move_duty}, "
            f"steer≤{self._max_steering_angle_rad} rad → cmd±{self._max_steer}"
        )
        self.get_logger().info("Output: ESP32 steering + VESC duty (CH5 mode switch)")
        self.get_logger().info(
            f"RC manual: CH5<={self._ch5_manual_us} -> CH2->VESC, "
            f"CH5>={self._ch5_auto_us} -> /drive->VESC + S:->ESP"
        )
        if bool(CFG["enable_keyboard_estop"]) and sys.stdin.isatty():
            self.get_logger().info(
                f"Keyboard ESTOP: Space=stop(latch), {self._estop_reset_key.upper()}=reset"
            )

    def _is_estop_latched(self) -> bool:
        with self._estop_lock:
            return self._estop_latched

    def _set_estop_latched(self, latched: bool) -> None:
        with self._estop_lock:
            changed = self._estop_latched != latched
            self._estop_latched = latched
        if not changed:
            return
        self.current_duty = 0.0
        self.current_steer = 0.0
        self._auto_duty_applied = 0.0
        self._last_duty_int = None
        self._last_duty_packet = None
        if latched:
            self.get_logger().warn("ESTOP latched — output forced to zero (press "
                                   f"{self._estop_reset_key.upper()} to reset)")
        else:
            self.get_logger().info("ESTOP cleared — /drive commands accepted again")

    def _start_keyboard_estop(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "Keyboard ESTOP disabled: stdin is not a TTY "
                "(run `ros2 run path_following control_node` in a terminal)"
            )
            return

        self._keyboard_running = True
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener,
            name="control_node_keyboard_estop",
            daemon=True,
        )
        self._keyboard_thread.start()

    def _keyboard_listener(self) -> None:
        fd = sys.stdin.fileno()
        self._stdin_termios_old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._keyboard_running:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                if ch == " ":
                    self._set_estop_latched(True)
                elif ch.lower() == self._estop_reset_key:
                    self._set_estop_latched(False)
        except Exception as e:
            self.get_logger().error(f"Keyboard ESTOP thread failed: {e}")
        finally:
            if self._stdin_termios_old is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, self._stdin_termios_old)

    def _stop_keyboard_estop(self) -> None:
        self._keyboard_running = False
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=0.5)
            self._keyboard_thread = None

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        if self._is_estop_latched():
            return

        self.last_cmd_time = time.time()

        speed_mps = float(msg.drive.speed)
        steering_rad = float(msg.drive.steering_angle)

        if self._invert_speed:
            speed_mps = -speed_mps
        if self._invert_steer:
            steering_rad = -steering_rad

        speed_norm = self.clamp(speed_mps / self._max_speed_mps, -1.0, 1.0)
        steer_norm = self.clamp(
            steering_rad / self._max_steering_angle_rad, -1.0, 1.0
        )

        duty = speed_norm * self._max_duty * self._speed_scale
        if (
            self._min_move_duty > 0.0
            and abs(speed_mps) >= self._min_move_speed_mps
            and abs(duty) < self._min_move_duty
        ):
            duty = math.copysign(self._min_move_duty, duty if duty != 0.0 else speed_mps)

        self._last_speed_mps = speed_mps
        self._last_steering_rad = steering_rad
        self._auto_duty = duty
        self._auto_steer = steer_norm * self._max_steer

    @staticmethod
    def _parse_rc_line(line: str):
        parts = line.strip().split(",")
        if len(parts) != 5 or parts[0] != "RC":
            return None
        try:
            ch1 = int(parts[1])
            ch2 = int(parts[2])
            ch5 = int(parts[3])
            return ch1, ch2, ch5
        except ValueError:
            return None

    def _read_esp_rc(self) -> None:
        waiting = self.esp.in_waiting
        if waiting <= 0:
            return

        self._esp_rx_buffer.extend(self.esp.read(waiting))
        if len(self._esp_rx_buffer) > 512:
            del self._esp_rx_buffer[:-512]

        while True:
            nl = self._esp_rx_buffer.find(b"\n")
            if nl < 0:
                break
            line = self._esp_rx_buffer[:nl].decode(errors="ignore")
            del self._esp_rx_buffer[: nl + 1]
            parsed = self._parse_rc_line(line)
            if parsed is None:
                continue
            ch1, ch2, ch5 = parsed
            self._rc_ch1 = ch1
            self._rc_ch2 = ch2
            self._rc_ch5 = ch5
            self._last_rc_time = time.time()

    def _is_autonomous_mode(self) -> bool:
        if self._last_rc_time <= 0.0:
            return False
        ch5 = self._rc_ch5
        if ch5 <= 0:
            return self._mode_auto_latched
        if ch5 <= self._ch5_manual_us:
            self._mode_auto_latched = False
            return False
        if ch5 >= self._ch5_auto_us:
            self._mode_auto_latched = True
            return True
        return self._mode_auto_latched

    def _rc_ch2_to_duty(self, ch2: int) -> float:
        if ch2 <= 0:
            return 0.0
        # ESP raw us (PPM index 1)
        if 800 <= ch2 <= 2200:
            center = self._rc_center_ch2
            error = ch2 - center
            if abs(error) < self._rc_deadzone:
                return 0.0
            duty = (error / 500.0) * self._max_duty
            if self._invert_rc_throttle:
                duty = -duty
            return self.clamp(duty, -self._max_duty, self._max_duty)

        # legacy 0..3000 scale fallback
        ch2 = int(self.clamp(float(ch2), self._rc_min_val, self._rc_max_val))
        error = ch2 - self._rc_center_ch2
        if abs(error) < self._rc_deadzone:
            return 0.0

        if error > 0:
            span = max(self._rc_max_val - self._rc_center_ch2, 1)
        else:
            span = max(self._rc_center_ch2 - self._rc_min_val, 1)

        duty = (error / span) * self._max_duty
        if self._invert_rc_throttle:
            duty = -duty
        return self.clamp(duty, -self._max_duty, self._max_duty)

    def _slew_auto_duty(self, target: float, dt: float) -> float:
        """AUTO duty를 ramp_sec 동안 선형으로 목표까지 올림/내림."""
        if self._auto_duty_ramp_sec <= 0.0:
            return target

        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = (self._max_duty / self._auto_duty_ramp_sec) * dt
        diff = target - self._auto_duty_applied
        if abs(diff) <= max_step:
            return target
        return self._auto_duty_applied + math.copysign(max_step, diff)

    def timer_callback(self) -> None:
        now = time.time()
        dt = now - self._last_timer_time
        self._last_timer_time = now

        self._read_esp_rc()
        autonomous = self._is_autonomous_mode()
        self._control_mode = "AUTO" if autonomous else "MANUAL"

        if self._is_estop_latched():
            self.current_duty = 0.0
            self.current_steer = 0.0
            self._auto_duty_applied = 0.0
        elif autonomous:
            if now - self.last_cmd_time > self._cmd_timeout:
                target_duty = 0.0
                self.current_steer = 0.0
            else:
                target_duty = self._auto_duty
                self.current_steer = self._auto_steer
            self._auto_duty_applied = self._slew_auto_duty(target_duty, dt)
            self.current_duty = self._auto_duty_applied
            self.send_steering(self.current_steer)
        else:
            self._auto_duty_applied = 0.0
            rc_fresh = (
                self._last_rc_time > 0.0
                and (time.time() - self._last_rc_time) <= self._rc_timeout
            )
            self.current_duty = (
                self._rc_ch2_to_duty(self._rc_ch2) if rc_fresh else 0.0
            )
            self.current_steer = 0.0
            # 수동: 조향은 ESP/RC CH1, Jetson은 UART 조향 안 보냄

        self.set_vesc_duty(self.current_duty)
        self._publish_telemetry(autonomous)
        self._maybe_log_debug()

    def _publish_telemetry(self, autonomous: bool) -> None:
        msg = Float64MultiArray()
        msg.data = [
            float(self._last_speed_mps),
            float(self._last_steering_rad),
            float(self.current_duty),
            float(self._auto_duty),
            float(self.current_steer),
            float(self._rc_ch5),
            1.0 if autonomous else 0.0,
            1.0 if self._is_estop_latched() else 0.0,
            float(self._rc_ch1),
            float(self._rc_ch2),
        ]
        self._telemetry_pub.publish(msg)

    def _maybe_log_debug(self) -> None:
        if self._debug_log_hz <= 0.0:
            return
        now = time.time()
        if now - self._last_debug_log_time < 1.0 / self._debug_log_hz:
            return
        self._last_debug_log_time = now
        self.get_logger().info(
            f"mode={self._control_mode} "
            f"/drive speed={self._last_speed_mps:.2f} m/s "
            f"steer_rad={self._last_steering_rad:.3f} → "
            f"duty={self.current_duty:.3f} target_duty={self._auto_duty:.3f} "
            f"esp_steer={self.current_steer:.3f} "
            f"RC CH1={self._rc_ch1} CH2={self._rc_ch2} CH5={self._rc_ch5}"
        )

    def send_steering(self, steer: float) -> None:
        steer = self.clamp(steer, -self._max_steer, self._max_steer)
        if self._steer_cmd_format == "prefixed":
            line = f"S:{steer:.3f}\n"
        else:
            # jetson_steer_send.py 와 동일: 숫자만 + 줄바꿈
            line = f"{steer:.3f}\n"
        self.esp.write(line.encode())

    def set_vesc_duty(self, duty: float) -> None:
        duty = self.clamp(duty, -self._max_duty, self._max_duty)
        duty_int = int(duty * 100000)

        if duty_int == self._last_duty_int and self._last_duty_packet is not None:
            self.vesc.write(self._last_duty_packet)
            return

        payload = bytearray()
        payload.append(5)
        payload.extend(struct.pack(">i", duty_int))

        packet = self.make_vesc_packet(payload)
        self._last_duty_int = duty_int
        self._last_duty_packet = packet
        self.vesc.write(packet)

    def stop_vehicle(self) -> None:
        self.set_vesc_duty(0.0)
        self.send_steering(0.0)

    @staticmethod
    def make_vesc_packet(payload: bytearray) -> bytearray:
        packet = bytearray()
        packet.append(0x02)
        packet.append(len(payload))
        packet.extend(payload)

        crc = VehicleControlNode.crc16_ccitt(payload)
        packet.append((crc >> 8) & 0xFF)
        packet.append(crc & 0xFF)
        packet.append(0x03)
        return packet

    @staticmethod
    def crc16_ccitt(data: bytearray) -> int:
        crc = 0
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    def destroy_node(self) -> None:
        self._stop_keyboard_estop()
        self.get_logger().info("Stopping vehicle...")
        try:
            self.stop_vehicle()
            time.sleep(0.1)
            self.stop_vehicle()
        except Exception as e:
            self.get_logger().error(f"Stop failed: {e}")

        try:
            self.esp.close()
            self.vesc.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
