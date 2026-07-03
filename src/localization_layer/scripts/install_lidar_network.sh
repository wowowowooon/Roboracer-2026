#!/usr/bin/env bash
# LiDAR 네트워크를 부팅 시 자동 설정하도록 한 번만 설치한다.
set -euo pipefail

IFACE="${1:-enP8p1s0}"
HOST_IP="${2:-192.168.11.3}"
PREFIX="${3:-24}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo (ros2 is not in sudo PATH — use bash directly):"
  echo "  sudo bash $0 ${IFACE} ${HOST_IP} ${PREFIX}"
  exit 1
fi

SERVICE="f1tenth-lidar-network.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE}"
SUDOERS_PATH="/etc/sudoers.d/f1tenth-lidar-network"

cat > "${SERVICE_PATH}" <<EOF
[Unit]
Description=F1Tenth LiDAR host network (${HOST_IP}/${PREFIX} on ${IFACE})
After=network-pre.target
Before=network-online.target
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c 'test -d /sys/class/net/${IFACE} && (ip addr show dev ${IFACE} | grep -Fq "inet ${HOST_IP}/" || ip addr add ${HOST_IP}/${PREFIX} dev ${IFACE}) && ip link set ${IFACE} up'

[Install]
WantedBy=multi-user.target
EOF

cat > "${SUDOERS_PATH}" <<EOF
nvidia ALL=(ALL) NOPASSWD: /usr/sbin/ip, /sbin/ip, /usr/bin/ip
nvidia ALL=(ALL) NOPASSWD: /bin/systemctl start ${SERVICE}, /bin/systemctl restart ${SERVICE}
EOF
chmod 440 "${SUDOERS_PATH}"

systemctl daemon-reload
systemctl enable --now "${SERVICE}"

echo "[install_lidar_network] installed ${SERVICE_PATH}"
echo "[install_lidar_network] enabled f1tenth-lidar-network.service"
echo "[install_lidar_network] configured passwordless sudo for /usr/sbin/ip"
echo "[install_lidar_network] done: mapping launch can run without manual network setup"
