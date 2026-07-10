#!/usr/bin/env bash
# Localization RViz (맵/스캔/2D Pose Estimate 설정 포함). bare rviz2 대신 이걸 쓰세요.
set -eo pipefail

export ROS_LOCALHOST_ONLY=0
# Jetson + SSH X11: XCB error 148 완화
export QT_X11_NO_MITSHM=1

if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/humble/setup.bash
  source /home/nvidia/f1tenth_ajou/install/setup.bash
fi

if [[ -n "${SSH_CONNECTION:-}" ]]; then
  unset XAUTHORITY
fi

if [[ -z "${DISPLAY:-}" ]] && [[ -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi

_can_display() {
  [[ -n "${DISPLAY:-}" ]] && xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1
}

if ! _can_display; then
  unset XAUTHORITY
fi

if ! _can_display; then
  cat <<'EOF'
ERROR: RViz 디스플레이(:0) 연결 실패.

한 번만 실행:
  sudo bash /home/nvidia/f1tenth_ajou/install/localization_layer/lib/localization_layer/fix_rviz_x11_access.sh

그다음:
  unset XAUTHORITY
  export DISPLAY=:0
  ros2 run localization_layer localization_rviz.sh
EOF
  exit 1
fi

if ! ros2 topic info /map 2>/dev/null | grep -qE 'Publisher count: [1-9]'; then
  echo "WARNING: /map publisher 없음 — localization launch를 먼저 켜세요."
  echo "  ros2 launch localization_layer cartographer_localization_launch.py"
  echo ""
fi

if ! ros2 topic info /tf 2>/dev/null | grep -qE 'Publisher count: [1-9]'; then
  echo "WARNING: /tf publisher 없음 — map 프레임이 없으면 스캔이 안 보입니다."
  echo "  localization launch가 켜져 있는지 확인하세요."
  echo ""
fi

CONFIG="$(ros2 pkg prefix localization_layer)/share/localization_layer/rviz/localization.rviz"
echo "RViz localization (Fixed Frame=map, Map=/map, LaserScan=/scan)"
echo "  조금 움직이면 자동으로 위치 찾음. 수동은 2D Pose Estimate."
echo "  DISPLAY=${DISPLAY}  config=${CONFIG}"
exec ros2 run rviz2 rviz2 -d "${CONFIG}"
