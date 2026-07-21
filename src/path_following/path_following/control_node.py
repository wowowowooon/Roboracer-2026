#!/usr/bin/env python3
"""
실차 하드웨어 제어: /drive (AckermannDriveStamped) → ESP32 조향 + VESC duty.

CH5 (PPM index [4] ONLY) 로 수동/자율:
  - CH5 >= 1700 (2000, 자율): /drive.steering → ESP, 속도는 max_target_speed_mps PI→VESC
  - CH5 <= 1300 (1000, 수동): CH1→ESP, CH2→VESC

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
from std_msgs.msg import Float64, Float64MultiArray


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
    "max_duty": 0.4,           # MANUAL VESC duty 상한 50% (송신기 풀스틱)
    "speed_scale": 1.0,         # 추가 감쇠 (1.0=끔)
    "min_move_duty": 0.08,      # 정지마찰 극복용 최소 duty (speed>threshold 일 때)
    "min_move_speed_mps": 0.10,
    "debug_log_hz": 1.0,
    "max_steer": 1.0,           # ESP 조향 명령 범위. 1.0 = 서보 ±40° 풀사용
    "steer_rate_limit_per_sec": 4.0,
    "steer_cmd_format": "prefixed",  # plain: "0.500\n" | prefixed: "S:0.500\n"
    "invert_speed": False,      # legacy AUTO sign flag; prefer auto_duty_output_sign
    # 원본 ESP normToAngle: S:-1→좌(50°), S:+1→우(130°) — INVERT_RC_STEER 미적용
    # Stanley +steer=좌 → S:- 로 보내야 함 (False면 AUTO 조향 반대 → 옆으로 밀림)
    "invert_steer": False,
    "cmd_timeout_sec": 0.25,
    "timer_period_sec": 0.02,     # ESP loop 20ms — AUTO 조향 응답
    "serial_open_delay_sec": 2.0,
    "enable_keyboard_estop": True,
    "estop_reset_key": "r",
    # RC (ESP -> RC,ch1_us,ch2_us,mode_us,0)  raw PWM us
    "ch5_auto_us": 1700,          # CH5 >= 1700 자율(2000)
    "ch5_manual_us": 1300,        # CH5 <= 1300 수동(1000)
    "rc_center_ch2": 1500,
    "rc_min_val": 0,
    "rc_max_val": 3000,
    "rc_deadzone": 30,
    "rc_timeout_sec": 0.30,
    "ch6_estop_us": 1700,         # ESP 다섯 번째 값이 CH6이면 >=1700 ESTOP latch
    "invert_rc_throttle": True,   # 송신기 CH2: 앞=높은 us → 전진
    "auto_duty_ramp_sec": 1.0,    # AUTO: /drive duty → VESC (1초에 목표까지)
    "telemetry_topic": "/vehicle/telemetry",  # drive_monitor.py 구독
    "speed_topic": "/vehicle/speed_mps",
    # AUTO closed-loop speed control (/drive.speed is target speed [m/s])
    "max_auto_duty": 0.45, # 0.2 최대 5당 0.1
    "max_target_speed_mps": 2.0, # m/s 속도 
    "speed_ff_duty_per_mps": 1.0 / 14.2,  #듀티가 0.05늘때마다 1늘리기
    "auto_duty_output_sign": -1.0,  # 이 차량은 전진 목표속도 -> 음수 VESC raw duty
    "speed_kp": 0.04,
    "speed_ki": 0.015,
    "integral_limit": 0.5,
    "duty_rate_limit_per_sec": 0.15,       # AUTO 가속 duty 변화율
    "duty_decel_rate_limit_per_sec": 0.20, # AUTO 감속: 최대 duty→0 약 0.6초
    "max_auto_brake_duty": 0.03,           # 속도 오버슈트 시 역방향 제동 duty 제한
    "vesc_telemetry_timeout_sec": 0.3,
    "vesc_poll_period_sec": 0.05,
    "invert_speed_sign": True,
    "pole_pairs": 2,
    "gear_ratio": 12.0,
    "wheel_diameter": 0.10,
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
        self._steer_rate_limit_per_sec = max(
            0.0, float(CFG.get("steer_rate_limit_per_sec", 1.5))
        )
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
        self._ch6_estop_us = int(CFG.get("ch6_estop_us", 1700))
        self._invert_rc_throttle = bool(CFG.get("invert_rc_throttle", False))
        self._auto_duty_ramp_sec = max(0.0, float(CFG.get("auto_duty_ramp_sec", 1.0)))
        self._max_auto_duty = max(0.0, float(CFG.get("max_auto_duty", 0.30)))
        self._max_target_speed_mps = max(0.0, float(CFG.get("max_target_speed_mps", 3.0)))
        self._speed_ff_duty_per_mps = float(CFG.get("speed_ff_duty_per_mps", 1.0 / 14.2))
        self._auto_duty_output_sign = -1.0 if float(
            CFG.get("auto_duty_output_sign", -1.0)
        ) < 0.0 else 1.0
        self._speed_kp = float(CFG.get("speed_kp", 0.04))
        self._speed_ki = float(CFG.get("speed_ki", 0.015))
        self._integral_limit = max(0.0, float(CFG.get("integral_limit", 0.5)))
        self._duty_rate_limit_per_sec = max(
            0.0, float(CFG.get("duty_rate_limit_per_sec", 0.15))
        )
        self._duty_decel_rate_limit_per_sec = max(
            0.0, float(CFG.get("duty_decel_rate_limit_per_sec", 0.20))
        )
        self._max_auto_brake_duty = max(
            0.0, float(CFG.get("max_auto_brake_duty", 0.03))
        )
        self._vesc_telemetry_timeout = max(
            0.0, float(CFG.get("vesc_telemetry_timeout_sec", 0.3))
        )
        self._vesc_poll_period = max(0.0, float(CFG.get("vesc_poll_period_sec", 0.05)))
        self._invert_speed_sign = bool(CFG.get("invert_speed_sign", True))
        self._pole_pairs = max(1e-6, float(CFG.get("pole_pairs", 2)))
        self._gear_ratio = max(1e-6, float(CFG.get("gear_ratio", 12.0)))
        self._wheel_diameter = max(1e-6, float(CFG.get("wheel_diameter", 0.10)))

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
        self._auto_steer_applied = 0.0
        self._target_speed_mps = 0.0
        self._speed_error = 0.0
        self._speed_integral = 0.0
        self._duty_ff = 0.0
        self._speed_duty_cmd = 0.0
        self._last_auto_duty_cmd = 0.0
        self.current_duty = 0.0
        self.current_steer = 0.0
        self._last_duty_int = None
        self._last_duty_packet = None
        self._last_speed_mps = 0.0
        self._last_steering_rad = 0.0
        self._last_debug_log_time = 0.0
        self._last_vesc_poll_time = 0.0
        self._last_vesc_telemetry_time = 0.0
        self._vesc_rx_buffer = bytearray()
        self._erpm = 0.0
        self._measured_speed_mps = 0.0
        self._current_motor = 0.0
        self._current_in = 0.0
        self._input_voltage = 0.0
        self._vesc_duty_now = 0.0
        self._vesc_temp = math.nan
        self._motor_temp = math.nan
        self._fault_code = 0

        self._rc_ch1 = 1497
        self._rc_ch2 = 1497
        self._rc_ch5 = 1000
        self._rc_ch6 = 0
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
        speed_topic = str(CFG.get("speed_topic", "/vehicle/speed_mps"))
        self._speed_pub = self.create_publisher(Float64, speed_topic, 10)
        self._vesc_status_pub = self.create_publisher(
            Float64MultiArray, "/vesc/status", 10
        )
        self._rc_state_pub = self.create_publisher(
            Float64MultiArray, "/rc/state", 10
        )
        self._control_state_pub = self.create_publisher(
            Float64MultiArray, "/control/state", 10
        )
        self._safety_state_pub = self.create_publisher(
            Float64MultiArray, "/safety/state", 10
        )

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
            f"RC auto: CH5>={self._ch5_auto_us} -> /drive->VESC + S:->ESP (auto), "
            f"CH5<={self._ch5_manual_us} -> CH2->VESC (manual)"
        )
        self.get_logger().info(
            f"AUTO speed PI: target={self._max_target_speed_mps:.2f} m/s "
            f"(/drive.speed ignore, steer-only), "
            f"duty≤{self._max_auto_duty:.2f}, kp={self._speed_kp:.3f}, "
            f"ki={self._speed_ki:.3f}, ff={self._speed_ff_duty_per_mps:.4f}, "
            f"accel≤{self._duty_rate_limit_per_sec:.2f}/s, "
            f"decel≤{self._duty_decel_rate_limit_per_sec:.2f}/s, "
            f"brake duty≤{self._max_auto_brake_duty:.2f}"
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
        self._reset_speed_controller()
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

        # /drive 는 조향만. AUTO 목표속도 = max_target_speed_mps
        target_speed_mps = self._max_target_speed_mps
        steering_rad = float(msg.drive.steering_angle)

        if self._invert_steer:
            steering_rad = -steering_rad

        steer_norm = self.clamp(
            steering_rad / self._max_steering_angle_rad, -1.0, 1.0
        )

        self._target_speed_mps = target_speed_mps
        self._last_speed_mps = target_speed_mps
        self._last_steering_rad = steering_rad
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
            ch6 = int(parts[4])
            return ch1, ch2, ch5, ch6
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
            ch1, ch2, ch5, ch6 = parsed
            self._rc_ch1 = ch1
            self._rc_ch2 = ch2
            self._rc_ch5 = ch5
            self._rc_ch6 = ch6
            self._last_rc_time = time.time()
            if ch6 >= self._ch6_estop_us:
                self._set_estop_latched(True)

    def _is_autonomous_mode(self) -> bool:
        if self._last_rc_time <= 0.0:
            return False
        ch5 = self._rc_ch5
        if ch5 <= 0:
            return self._mode_auto_latched
        if ch5 >= self._ch5_auto_us:
            self._mode_auto_latched = True
            return True
        if ch5 <= self._ch5_manual_us:
            self._mode_auto_latched = False
            return False
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

    def _compute_speed_from_erpm(self, erpm: float) -> float:
        motor_rpm = float(erpm) / self._pole_pairs
        wheel_rpm = motor_rpm / self._gear_ratio
        raw_speed_mps = wheel_rpm / 60.0 * math.pi * self._wheel_diameter
        if self._invert_speed_sign:
            return -raw_speed_mps
        return raw_speed_mps

    def _reset_speed_controller(self) -> None:
        self._speed_integral = 0.0
        self._speed_error = 0.0
        self._duty_ff = 0.0
        self._speed_duty_cmd = 0.0
        self._auto_duty = 0.0
        self._auto_duty_applied = 0.0
        self._last_auto_duty_cmd = 0.0

    def _reset_auto_steer(self) -> None:
        self._auto_steer = 0.0
        self._auto_steer_applied = 0.0

    def _apply_duty_rate_limit(self, target_duty: float, dt: float) -> float:
        decelerating = (
            target_duty == 0.0
            or target_duty * self._last_auto_duty_cmd < 0.0
            or (
                target_duty * self._last_auto_duty_cmd > 0.0
                and abs(target_duty) < abs(self._last_auto_duty_cmd)
            )
        )
        rate_limit = (
            self._duty_decel_rate_limit_per_sec
            if decelerating
            else self._duty_rate_limit_per_sec
        )
        if rate_limit <= 0.0:
            return target_duty
        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = rate_limit * dt
        diff = target_duty - self._last_auto_duty_cmd
        if abs(diff) <= max_step:
            return target_duty
        return self._last_auto_duty_cmd + math.copysign(max_step, diff)

    def _coast_auto_duty_to_zero(self, dt: float) -> float:
        """일반 정지 명령은 duty를 즉시 끊지 않고 감속 제한을 적용한다."""
        duty_cmd = self._apply_duty_rate_limit(0.0, dt)
        self._speed_integral = 0.0
        self._speed_error = 0.0
        self._duty_ff = 0.0
        self._speed_duty_cmd = duty_cmd
        self._last_auto_duty_cmd = duty_cmd
        self._auto_duty = duty_cmd
        self._auto_duty_applied = duty_cmd
        return duty_cmd

    def _apply_manual_duty_rate_limit(self, target_duty: float, dt: float) -> float:
        """수동 가속은 즉시 반영하고 중립/역방향 감속만 완만하게 제한한다."""
        decelerating = (
            target_duty == 0.0
            or target_duty * self.current_duty < 0.0
            or (
                target_duty * self.current_duty > 0.0
                and abs(target_duty) < abs(self.current_duty)
            )
        )
        if not decelerating:
            return target_duty

        rate_limit = self._duty_decel_rate_limit_per_sec
        if rate_limit <= 0.0:
            return target_duty
        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = rate_limit * dt
        diff = target_duty - self.current_duty
        if abs(diff) <= max_step:
            return target_duty
        return self.current_duty + math.copysign(max_step, diff)

    def _apply_steer_rate_limit(self, target_steer: float, dt: float) -> float:
        target_steer = self.clamp(target_steer, -self._max_steer, self._max_steer)
        if self._steer_rate_limit_per_sec <= 0.0:
            self._auto_steer_applied = target_steer
            return target_steer

        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = self._steer_rate_limit_per_sec * dt
        diff = target_steer - self._auto_steer_applied
        if abs(diff) <= max_step:
            self._auto_steer_applied = target_steer
        else:
            self._auto_steer_applied += math.copysign(max_step, diff)
        return self._auto_steer_applied

    def _update_speed_controller(
        self, target_speed: float, measured_speed: float, dt: float
    ) -> float:
        target_speed = self.clamp(target_speed, 0.0, self._max_target_speed_mps)
        dt = self.clamp(dt, 1e-4, 0.1)

        if target_speed <= 1e-6:
            self._reset_speed_controller()
            return 0.0

        self._speed_error = target_speed - measured_speed
        self._duty_ff = target_speed * self._speed_ff_duty_per_mps
        self._speed_integral += self._speed_error * dt
        self._speed_integral = self.clamp(
            self._speed_integral, -self._integral_limit, self._integral_limit
        )

        duty_cmd = (
            self._duty_ff
            + self._speed_kp * self._speed_error
            + self._speed_ki * self._speed_integral
        )
        duty_cmd *= self._auto_duty_output_sign
        # 진행 duty와 반대 부호는 회생/제동 토크이므로 별도 상한을 적용한다.
        if duty_cmd * self._auto_duty_output_sign < 0.0:
            duty_cmd = self.clamp(
                duty_cmd, -self._max_auto_brake_duty, self._max_auto_brake_duty
            )
        duty_cmd = self.clamp(duty_cmd, -self._max_auto_duty, self._max_auto_duty)
        duty_cmd = self._apply_duty_rate_limit(duty_cmd, dt)
        duty_cmd = self.clamp(duty_cmd, -self._max_auto_duty, self._max_auto_duty)

        self._last_auto_duty_cmd = duty_cmd
        self._speed_duty_cmd = duty_cmd
        self._auto_duty = duty_cmd
        self._auto_duty_applied = duty_cmd
        return duty_cmd

    def _request_vesc_values(self) -> None:
        payload = bytearray([4])  # COMM_GET_VALUES
        self.vesc.write(self.make_vesc_packet(payload))

    def _poll_vesc_telemetry(self, now: float) -> None:
        if self._vesc_poll_period > 0.0 and (
            now - self._last_vesc_poll_time >= self._vesc_poll_period
        ):
            self._last_vesc_poll_time = now
            self._request_vesc_values()

        waiting = self.vesc.in_waiting
        if waiting <= 0:
            return

        self._vesc_rx_buffer.extend(self.vesc.read(waiting))
        if len(self._vesc_rx_buffer) > 2048:
            del self._vesc_rx_buffer[:-2048]
        self._parse_vesc_rx_buffer(now)

    def _parse_vesc_rx_buffer(self, now: float) -> None:
        while True:
            start = self._vesc_rx_buffer.find(b"\x02")
            if start < 0:
                self._vesc_rx_buffer.clear()
                return
            if start > 0:
                del self._vesc_rx_buffer[:start]
            if len(self._vesc_rx_buffer) < 5:
                return

            payload_len = self._vesc_rx_buffer[1]
            packet_len = payload_len + 5
            if len(self._vesc_rx_buffer) < packet_len:
                return
            packet = self._vesc_rx_buffer[:packet_len]
            del self._vesc_rx_buffer[:packet_len]

            if packet[-1] != 0x03:
                continue
            payload = packet[2 : 2 + payload_len]
            rx_crc = (packet[2 + payload_len] << 8) | packet[3 + payload_len]
            if self.crc16_ccitt(payload) != rx_crc:
                continue
            self._handle_vesc_payload(payload, now)

    def _handle_vesc_payload(self, payload: bytes | bytearray, now: float) -> None:
        if not payload or payload[0] != 4:  # COMM_GET_VALUES response
            return
        if len(payload) < 29:
            return

        try:
            vesc_temp = struct.unpack(">h", payload[1:3])[0] / 10.0
            motor_temp = struct.unpack(">h", payload[3:5])[0] / 10.0
            current_motor = struct.unpack(">i", payload[5:9])[0] / 100.0
            current_in = struct.unpack(">i", payload[9:13])[0] / 100.0
            duty_now = struct.unpack(">h", payload[21:23])[0] / 1000.0
            erpm = float(struct.unpack(">i", payload[23:27])[0])
            input_voltage = struct.unpack(">h", payload[27:29])[0] / 10.0
        except struct.error:
            return

        self._erpm = erpm
        self._current_motor = current_motor
        self._current_in = current_in
        self._input_voltage = input_voltage
        self._vesc_duty_now = duty_now
        self._vesc_temp = vesc_temp
        self._motor_temp = motor_temp if motor_temp > -200.0 else math.nan
        self._fault_code = int(payload[53]) if len(payload) > 53 else 0
        self._measured_speed_mps = self._compute_speed_from_erpm(erpm)
        self._last_vesc_telemetry_time = now

    def _vesc_telemetry_fresh(self, now: float) -> bool:
        return (
            self._last_vesc_telemetry_time > 0.0
            and now - self._last_vesc_telemetry_time <= self._vesc_telemetry_timeout
        )

    def timer_callback(self) -> None:
        now = time.time()
        dt = now - self._last_timer_time
        self._last_timer_time = now

        self._read_esp_rc()
        self._poll_vesc_telemetry(now)
        autonomous = self._is_autonomous_mode()
        self._control_mode = "AUTO" if autonomous else "MANUAL"
        rc_fresh = (
            self._last_rc_time > 0.0
            and (now - self._last_rc_time) <= self._rc_timeout
        )
        timeout_active = (
            (autonomous and now - self.last_cmd_time > self._cmd_timeout)
            or (not autonomous and not rc_fresh)
        )

        if self._is_estop_latched():
            self.current_duty = 0.0
            self.current_steer = 0.0
            self._reset_speed_controller()
            self._reset_auto_steer()
        elif autonomous:
            if now - self.last_cmd_time > self._cmd_timeout:
                self._reset_speed_controller()
                self._reset_auto_steer()
                self.current_steer = 0.0
                self.current_duty = 0.0
            else:
                target_speed = self.clamp(
                    self._target_speed_mps, 0.0, self._max_target_speed_mps
                )
                self.current_steer = self._apply_steer_rate_limit(self._auto_steer, dt)
                if target_speed <= 1e-6:
                    self.current_duty = self._coast_auto_duty_to_zero(dt)
                elif not self._vesc_telemetry_fresh(now):
                    self.current_duty = 0.0
                    self._reset_speed_controller()
                else:
                    self.current_duty = self._update_speed_controller(
                        target_speed, self._measured_speed_mps, dt
                    )
            self.send_steering(self.current_steer)
        else:
            self._reset_speed_controller()
            self._reset_auto_steer()
            if rc_fresh:
                manual_target_duty = self._rc_ch2_to_duty(self._rc_ch2)
                self.current_duty = self._apply_manual_duty_rate_limit(
                    manual_target_duty, dt
                )
            else:
                # RC 통신이 끊긴 경우에는 감속감보다 안전 정지를 우선한다.
                self.current_duty = 0.0
            self.current_steer = 0.0
            # 수동: 조향은 ESP/RC CH1, Jetson은 UART 조향 안 보냄

        self.set_vesc_duty(self.current_duty)
        self._publish_telemetry(autonomous)
        self._publish_state_topics(autonomous, timeout_active)
        self._publish_speed()
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
            float(self._measured_speed_mps),
            float(self._target_speed_mps),
            float(self._speed_error),
            float(self._duty_ff),
            float(self._speed_duty_cmd),
            float(self._erpm),
            float(self._current_in),
            float(self._current_motor),
            float(self._input_voltage),
        ]
        self._telemetry_pub.publish(msg)

    def _publish_state_topics(
        self, autonomous: bool, timeout_active: bool
    ) -> None:
        stamp = self.get_clock().now().nanoseconds * 1e-9

        if self._last_vesc_telemetry_time > 0.0:
            vesc_msg = Float64MultiArray()
            vesc_msg.data = [
                stamp,
                float(self._input_voltage),
                float(self._current_in),
                float(self._current_motor),
                float(self._vesc_duty_now),
                float(self._erpm),
                float(self._vesc_temp),
                float(self._motor_temp),
                float(self._fault_code),
            ]
            self._vesc_status_pub.publish(vesc_msg)

        if self._last_rc_time > 0.0:
            ch6 = self._rc_ch6 if self._rc_ch6 > 0 else -1
            rc_estop = ch6 >= self._ch6_estop_us
            rc_msg = Float64MultiArray()
            rc_msg.data = [
                stamp,
                float(self._rc_ch1),
                float(self._rc_ch2),
                float(self._rc_ch5),
                float(ch6),
                0.0 if autonomous else 1.0,
                1.0 if autonomous else 0.0,
                1.0 if rc_estop else 0.0,
            ]
            self._rc_state_pub.publish(rc_msg)

        control_msg = Float64MultiArray()
        control_msg.data = [
            stamp,
            float(self._speed_duty_cmd),
            float(self._last_steering_rad),
        ]
        self._control_state_pub.publish(control_msg)

        estop_active = self._is_estop_latched()
        manual_override = not autonomous
        if estop_active:
            reason_code = 1.0
        elif timeout_active:
            reason_code = 2.0
        elif manual_override:
            reason_code = 3.0
        else:
            reason_code = 0.0
        safety_msg = Float64MultiArray()
        safety_msg.data = [
            stamp,
            1.0 if estop_active else 0.0,
            1.0 if manual_override else 0.0,
            1.0 if timeout_active else 0.0,
            0.0,  # TODO: duty limiter activation state
            float(self._speed_duty_cmd),
            float(self.current_duty),
            float(self.current_duty),
            float(self.current_steer),
            reason_code,
        ]
        self._safety_state_pub.publish(safety_msg)

    def _publish_speed(self) -> None:
        msg = Float64()
        msg.data = float(self._measured_speed_mps)
        self._speed_pub.publish(msg)

    def _maybe_log_debug(self) -> None:
        if self._debug_log_hz <= 0.0:
            return
        now = time.time()
        if now - self._last_debug_log_time < 1.0 / self._debug_log_hz:
            return
        self._last_debug_log_time = now
        self.get_logger().info(
            f"mode={self._control_mode} "
            f"target_speed_mps={self._target_speed_mps:.2f} "
            f"measured_speed_mps={self._measured_speed_mps:.2f} "
            f"speed_error={self._speed_error:.2f} "
            f"steer_rad={self._last_steering_rad:.3f} → "
            f"duty_ff={self._duty_ff:.3f} duty_cmd={self.current_duty:.3f} "
            f"erpm={self._erpm:.0f} current_motor={self._current_motor:.2f}A "
            f"current_in={self._current_in:.2f}A input_voltage={self._input_voltage:.1f}V "
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
        duty_limit = max(abs(self._max_duty), abs(self._max_auto_duty))
        duty = self.clamp(duty, -duty_limit, duty_limit)
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
