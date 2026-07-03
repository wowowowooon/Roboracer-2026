import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from evdev import InputDevice
    from evdev import ecodes
    _EVDEV_IMPORT_OK = True
except ImportError:
    _EVDEV_IMPORT_OK = False

msg = """
-----------------------------------------
RC카 키보드 조종 (카트 스타일 가속)
-----------------------------------------
조향 손떼 즉시 0 : 환경변수 F1TENTH_STEER_EVDEV=/dev/input/eventN (evdev, 키 업 감지)
  예: export F1TENTH_STEER_EVDEV=/dev/input/by-id/usb-_-event-kbd   또는 evtest 로 장치 확인
  pip install evdev / 또는 sudo 를 input 그룹에. 미설정 시 터미널만으로는 리피트 타임아웃 필요(지연 있음)
↑↓·W S : 리피트 유예 2.5초 (전진 유지·조향 동시)
전진만 뗌: 직선만이면 빠르게 감속 / 조향 걸린 채·꺾인 상태면 전진만 뗐을 때만 천천히 감속
Space : 즉시 정지
Ctrl-C : 종료
-----------------------------------------
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_control_node")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.print_status = False

        self.steer_power = 2.0
        # W/S·↑↓ “계속 누름” 판정: 키 리피트 간격이 길어도 전진+조향 동시 조작 유지
        self.throttle_deadman_sec = 2.5
        # 5초 동안 0→±linear_cmd_max (기존 대비 최고 속도 명령 절반)
        self.linear_cmd_max = 0.5
        self.ramp_to_full_sec = 5.0
        self.ramp_rate = self.linear_cmd_max / self.ramp_to_full_sec
        # 전진(또는 후진) 손 뗌: 약 2초에 걸쳐 0까지(coast_rate = 1/초)
        self.coast_to_zero_sec_fast = 1.0
        self.coast_to_zero_sec_slow = 1.0

        self.current_linear = 0.0
        self.current_angular = 0.0
        # 조향: stdin만 쓸 때는 키업 없음 → steer_hold_sec 초 후에야 0. evdev 켜면 실제 키 상태로 즉시 0
        self.steer_hold_sec = 1.05
        self._steer_latched: int = 0
        self._steer_last_event_ts: float = 0.0
        self._use_evdev_steer = False
        self._steer_evdev = None
        self._evdev_codes_down: set = set()
        # 전후 방향이 하드웨어와 반대일 때 True (linear.x 부호 반전)
        self.invert_linear_cmd = True
        self.key_last_time = {
            "up": 0.0,
            "down": 0.0,
            "left": 0.0,
            "right": 0.0,
        }
        self._prev_steer_on: bool = False
        self._prev_up_on: bool = False
        self._prev_down_on: bool = False
        self._coast_linear_slow: bool = False

        self.settings = termios.tcgetattr(sys.stdin)

        ev_path = os.environ.get("F1TENTH_STEER_EVDEV", "").strip()
        if _EVDEV_IMPORT_OK and ev_path:
            try:
                self._steer_evdev = InputDevice(ev_path)
                self._use_evdev_steer = True
                self.get_logger().info(
                    "조향: evdev 사용 (%s) — 손떼면 즉시 0 / pip install evdev 및 장치 권한 확인"
                    % ev_path
                )
            except OSError as e:
                self.get_logger().warning(
                    "F1TENTH_STEER_EVDEV 열기 실패 (%s), 터미널 폴백: %s"
                    % (ev_path, e)
                )
                self._steer_evdev = None
                self._use_evdev_steer = False

    def _poll_steer_evdev(self) -> None:
        """실제 키보드 키 업/다운(evdev) → 조향 래치. 터미널은 키업이 없어 여기만 ‘즉시 0’ 가능."""
        if not self._use_evdev_steer or self._steer_evdev is None:
            return
        left_set = {ecodes.KEY_LEFT, ecodes.KEY_A}
        right_set = {ecodes.KEY_RIGHT, ecodes.KEY_D}
        while True:
            try:
                ev = self._steer_evdev.read_one()
            except BlockingIOError:
                break
            if ev is None:
                break
            if ev.type != ecodes.EV_KEY:
                continue
            c = ev.code
            if c not in left_set and c not in right_set:
                continue
            if ev.value in (1, 2):
                self._evdev_codes_down.add(c)
            else:
                self._evdev_codes_down.discard(c)
        L = bool(self._evdev_codes_down & left_set)
        R = bool(self._evdev_codes_down & right_set)
        if L and not R:
            self._steer_latched = -1
        elif R and not L:
            self._steer_latched = 1
        elif L and R:
            self._steer_latched = 1
        else:
            self._steer_latched = 0

    def _read_single_key_blocking(self, timeout_sec: float) -> str:
        tty.setraw(sys.stdin.fileno())
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout_sec)
            if not rlist:
                return ""
            key = sys.stdin.read(1)
            if key == "\x1b":
                key += sys.stdin.read(2)
            return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def collect_key_events(self) -> list[str]:
        """한 루프에서 대기(최대 0.02s) 후, 버퍼에 남은 키까지 모두 읽는다."""
        keys: list[str] = []
        first = self._read_single_key_blocking(0.02)
        if first:
            keys.append(first)
        while True:
            k = self._read_single_key_blocking(0.0)
            if not k:
                break
            keys.append(k)
        return keys

    def publish_cmd(self, speed: float, steering: float):
        twist_msg = Twist()
        twist_msg.linear.x = float(speed)
        twist_msg.angular.z = float(steering)
        self.publisher_.publish(twist_msg)

    def run(self):
        if self.print_status:
            print(msg)
        last_mono = time.monotonic()
        self._prev_steer_on = False
        self._prev_up_on = False
        self._prev_down_on = False
        self._coast_linear_slow = False
        self._steer_latched = 0
        self._steer_last_event_ts = 0.0
        self._evdev_codes_down.clear()
        try:
            while rclpy.ok():
                now = time.time()
                now_mono = time.monotonic()
                dt = min(now_mono - last_mono, 0.05)
                last_mono = now_mono

                should_exit = False
                up_live_before_keys = (
                    now - self.key_last_time["up"]
                ) <= self.throttle_deadman_sec
                steer_key_events_this_frame = False
                for key in self.collect_key_events():
                    if key == "\x1b[A":  # up
                        self.key_last_time["up"] = now
                    elif key == "\x1b[B":  # down
                        self.key_last_time["down"] = now
                    elif key == "\x1b[D":  # left
                        self.key_last_time["left"] = now
                        steer_key_events_this_frame = True
                        if not self._use_evdev_steer:
                            self._steer_latched = -1
                            self._steer_last_event_ts = now
                    elif key == "\x1b[C":  # right
                        self.key_last_time["right"] = now
                        steer_key_events_this_frame = True
                        if not self._use_evdev_steer:
                            self._steer_latched = 1
                            self._steer_last_event_ts = now
                    elif len(key) == 1 and key.lower() == "w":
                        self.key_last_time["up"] = now
                    elif len(key) == 1 and key.lower() == "s":
                        self.key_last_time["down"] = now
                    elif len(key) == 1 and key.lower() == "a":
                        self.key_last_time["left"] = now
                        steer_key_events_this_frame = True
                        if not self._use_evdev_steer:
                            self._steer_latched = -1
                            self._steer_last_event_ts = now
                    elif len(key) == 1 and key.lower() == "d":
                        self.key_last_time["right"] = now
                        steer_key_events_this_frame = True
                        if not self._use_evdev_steer:
                            self._steer_latched = 1
                            self._steer_last_event_ts = now
                    elif key == " ":
                        for k in self.key_last_time:
                            self.key_last_time[k] = 0.0
                        self.current_linear = 0.0
                        self.current_angular = 0.0
                        self._coast_linear_slow = False
                        self._steer_latched = 0
                        self._steer_last_event_ts = 0.0
                        self._evdev_codes_down.clear()
                    elif key == "\x03":
                        should_exit = True
                        break
                if should_exit:
                    break

                # 전진 키 리피트가 끊겨도 조향 키 이벤트로 전진 “계속 누름”을 보강 (동시에 전진+조향)
                if steer_key_events_this_frame and (
                    self._prev_up_on or up_live_before_keys
                ):
                    if self.key_last_time["up"] >= self.key_last_time["down"]:
                        self.key_last_time["up"] = now

                if not self._use_evdev_steer:
                    if (
                        self._steer_last_event_ts > 0.0
                        and (now - self._steer_last_event_ts) > self.steer_hold_sec
                    ):
                        self._steer_latched = 0
                else:
                    self._poll_steer_evdev()
                    # evdev 조향 시에도 전진 키 리피트 공백 동안 /cmd_vel 선속도 유지
                    if self._steer_latched != 0:
                        up_recent = (
                            now - self.key_last_time["up"]
                        ) <= self.throttle_deadman_sec
                        if self._prev_up_on or up_recent:
                            if self.key_last_time["up"] >= self.key_last_time["down"]:
                                self.key_last_time["up"] = now

                # 조향 해제 보강은 “이번 프레임 키 이벤트 직후” 래치 기준(prev).
                steer_now_on_pre = self._steer_latched != 0

                up_live = (
                    now - self.key_last_time["up"]
                ) <= self.throttle_deadman_sec
                down_live = (
                    now - self.key_last_time["down"]
                ) <= self.throttle_deadman_sec

                # 조향만 뗄 때: 코스트 중 current_linear 만으로 W/S 타임스탬프를 올리면 전진이 다시 붙음 → up_live/down_live 일 때만 보강
                if self._prev_steer_on and not steer_now_on_pre:
                    if not down_live and up_live:
                        self.key_last_time["up"] = now
                    if not up_live and down_live:
                        self.key_last_time["down"] = now
                    up_live = (
                        now - self.key_last_time["up"]
                    ) <= self.throttle_deadman_sec
                    down_live = (
                        now - self.key_last_time["down"]
                    ) <= self.throttle_deadman_sec

                up_on = False
                down_on = False
                if up_live and down_live:
                    if self.key_last_time["up"] >= self.key_last_time["down"]:
                        up_on = True
                    else:
                        down_on = True
                elif up_live:
                    up_on = True
                elif down_live:
                    down_on = True

                prev_up = self._prev_up_on
                prev_down = self._prev_down_on

                steer_now_on = self._steer_latched != 0
                self.current_angular = float(self._steer_latched) * self.steer_power

                if prev_up and not up_on:
                    in_curve = (
                        steer_now_on
                        or self._prev_steer_on
                        or abs(self.current_angular) > 0.08
                    )
                    self._coast_linear_slow = in_curve
                elif prev_down and not down_on:
                    in_curve = (
                        steer_now_on
                        or self._prev_steer_on
                        or abs(self.current_angular) > 0.08
                    )
                    self._coast_linear_slow = in_curve
                if up_on:
                    self._coast_linear_slow = False
                if down_on:
                    self._coast_linear_slow = False

                coast_sec = (
                    self.coast_to_zero_sec_slow
                    if self._coast_linear_slow
                    else self.coast_to_zero_sec_fast
                )
                # 최고 명령(linear_cmd_max)까지 쌓였다가 손 떼면 정확히 coast_sec 초에 0
                coast_rate = self.linear_cmd_max / coast_sec

                # --- 전후 축만 (카트: 손 뗀 스로틀만 줄어듦) ---
                if up_on and not down_on:
                    self.current_linear = min(
                        self.linear_cmd_max,
                        self.current_linear + self.ramp_rate * dt,
                    )
                elif down_on and not up_on:
                    self.current_linear = max(
                        -self.linear_cmd_max,
                        self.current_linear - self.ramp_rate * dt,
                    )
                elif self.current_linear > 0.0:
                    self.current_linear = max(
                        0.0,
                        self.current_linear - coast_rate * dt,
                    )
                    if self.current_linear <= 0.0:
                        self._coast_linear_slow = False
                elif self.current_linear < 0.0:
                    self.current_linear = min(
                        0.0,
                        self.current_linear + coast_rate * dt,
                    )
                    if self.current_linear >= 0.0:
                        self._coast_linear_slow = False

                out_linear = (
                    -self.current_linear if self.invert_linear_cmd else self.current_linear
                )
                self.publish_cmd(out_linear, self.current_angular)
                if self.print_status:
                    pct = (
                        abs(out_linear) / self.linear_cmd_max * 100.0
                        if self.linear_cmd_max > 0.0
                        else 0.0
                    )
                    sway = (
                        abs(self.current_angular) / self.steer_power * 100.0
                        if self.steer_power
                        else 0.0
                    )
                    sys.stdout.write(
                        f"\rcmd_vel linear.x={out_linear:+.2f} (~{pct:.0f}% max) | "
                        f"steer={self.current_angular:+.2f} (~{sway:.0f}% max)   "
                    )
                    sys.stdout.flush()

                self._prev_steer_on = steer_now_on
                self._prev_up_on = up_on
                self._prev_down_on = down_on
        except Exception as e:
            print(f"\n오류 발생: {e}")
        finally:
            self.publish_cmd(0.0, 0.0)
            if self._steer_evdev is not None:
                try:
                    self._steer_evdev.close()
                except OSError:
                    pass
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
