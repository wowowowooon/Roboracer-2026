#!/usr/bin/env bash
# LiDAR 네트워크 확인 + (설치된 경우) systemd 서비스로 IP 복구
set -u

IFACE="${1:-enP8p1s0}"
HOST_IP="${2:-192.168.11.3}"
PREFIX="${3:-24}"
LIDAR_IP="${4:-192.168.11.2}"
SERVICE="f1tenth-lidar-network.service"

wait_for_ping() {
  local attempt
  for attempt in $(seq 1 8); do
    if ping -c 1 -W 1 "${LIDAR_IP}" >/dev/null 2>&1; then
      echo "[setup_lidar_network] ping ok: ${LIDAR_IP}"
      return 0
    fi
    sleep 0.25
  done
  return 1
}

run_ip_root() {
  ip "$@" 2>/dev/null && return 0
  sudo -n ip "$@" 2>/dev/null && return 0
  return 1
}

start_systemd_lidar_net() {
  if ! systemctl list-unit-files "${SERVICE}" >/dev/null 2>&1; then
    return 1
  fi
  if ! systemctl is-enabled "${SERVICE}" >/dev/null 2>&1; then
    return 1
  fi
  sudo -n systemctl start "${SERVICE}" 2>/dev/null || systemctl start "${SERVICE}" 2>/dev/null || return 1
  sleep 0.4
  return 0
}

echo "[setup_lidar_network] checking ${LIDAR_IP} via ${IFACE} (${HOST_IP}/${PREFIX})"

if wait_for_ping; then
  exit 0
fi

if start_systemd_lidar_net && wait_for_ping; then
  exit 0
fi

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "[setup_lidar_network] ERROR: interface ${IFACE} not found"
  exit 1
fi

if ! ip addr show dev "${IFACE}" | grep -Fq "inet ${HOST_IP}/"; then
  if run_ip_root addr add "${HOST_IP}/${PREFIX}" dev "${IFACE}" 2>/dev/null; then
    echo "[setup_lidar_network] added ${HOST_IP}/${PREFIX} on ${IFACE}"
  else
    echo "[setup_lidar_network] ERROR: host IP ${HOST_IP} not on ${IFACE}"
    echo "[setup_lidar_network]"
    echo "[setup_lidar_network] === 한 번만 실행 (비밀번호 입력) ==="
    echo "[setup_lidar_network]   sudo bash /home/nvidia/f1tenth_ajou/install/localization_layer/lib/localization_layer/install_lidar_network.sh"
    echo "[setup_lidar_network]"
    echo "[setup_lidar_network] === 또는 이번 세션만 임시 ==="
    echo "[setup_lidar_network]   sudo ip addr add ${HOST_IP}/${PREFIX} dev ${IFACE}"
    echo "[setup_lidar_network]   sudo ip link set ${IFACE} up"
    exit 1
  fi
fi

run_ip_root link set "${IFACE}" up || true

if wait_for_ping; then
  exit 0
fi

echo "[setup_lidar_network] ERROR: cannot ping ${LIDAR_IP}"
echo "[setup_lidar_network] LiDAR 전원/랜선 확인 후:"
echo "[setup_lidar_network]   sudo bash /home/nvidia/f1tenth_ajou/install/localization_layer/lib/localization_layer/install_lidar_network.sh"
exit 1
