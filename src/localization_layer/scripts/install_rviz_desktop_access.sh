#!/usr/bin/env bash
# Jetson에서 RViz를 SSH/데스크톱 모두 쓰려면 nvidia 데스크톱 자동 로그인 + xhost 설정.
set -euo pipefail

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Run once with sudo:"
  echo "  sudo bash $0"
  exit 1
fi

REAL_USER="${SUDO_USER:-nvidia}"
REAL_HOME="$(getent passwd "${REAL_USER}" | cut -d: -f6)"
AUTOSTART_DIR="${REAL_HOME}/.config/autostart"

mkdir -p "${AUTOSTART_DIR}"
cat > "${AUTOSTART_DIR}/rviz-xhost.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=Allow RViz from SSH
Exec=/usr/bin/xhost +SI:localuser:${REAL_USER}
Hidden=false
NoDisplay=true
X-GNOME-Autostart-enabled=true
EOF
chown -R "${REAL_USER}:${REAL_USER}" "${REAL_HOME}/.config/autostart"

GDM_CONF="/etc/gdm3/custom.conf"
cp "${GDM_CONF}" "${GDM_CONF}.bak.$(date +%Y%m%d_%H%M%S)"

python3 <<'PY'
from pathlib import Path
import re

path = Path("/etc/gdm3/custom.conf")
text = path.read_text(encoding="utf-8")
if "AutomaticLoginEnable" not in text or "#  AutomaticLoginEnable" in text:
    text = re.sub(
        r"(?m)^#?\s*AutomaticLoginEnable\s*=.*$",
        "AutomaticLoginEnable = true",
        text,
        count=1,
    )
    if "AutomaticLoginEnable" not in text:
        text = text.replace(
            "[daemon]\n",
            "[daemon]\nAutomaticLoginEnable = true\nAutomaticLogin = nvidia\n",
            1,
        )
text = re.sub(
    r"(?m)^#?\s*AutomaticLogin\s*=.*$",
    "AutomaticLogin = nvidia",
    text,
    count=1,
)
if "AutomaticLogin = nvidia" not in text:
    text = text.replace(
        "AutomaticLoginEnable = true\n",
        "AutomaticLoginEnable = true\nAutomaticLogin = nvidia\n",
        1,
    )
if "WaylandEnable=false" not in text:
    text = text.replace("[daemon]\n", "[daemon]\nWaylandEnable=false\n", 1)
path.write_text(text, encoding="utf-8")
PY

chown "${REAL_USER}:${REAL_USER}" "${AUTOSTART_DIR}/rviz-xhost.desktop"

SCRIPT_SRC="$(readlink -f "$0" 2>/dev/null || echo "$0")"
PKG_SCRIPTS="$(dirname "${SCRIPT_SRC}")"
BOOT_SCRIPT="${PKG_SCRIPTS}/rviz-xhost-at-boot.sh"
if [[ ! -f "${BOOT_SCRIPT}" ]]; then
  BOOT_SCRIPT="/home/nvidia/f1tenth_ajou/src/localization_layer/scripts/rviz-xhost-at-boot.sh"
fi
install -m 0755 "${BOOT_SCRIPT}" /usr/local/bin/rviz-xhost-at-boot.sh

cat > /etc/systemd/system/rviz-xhost.service <<EOF
[Unit]
Description=Allow ${REAL_USER} X11 access for RViz (SSH)
After=display-manager.service
Wants=display-manager.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/rviz-xhost-at-boot.sh ${REAL_USER}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable rviz-xhost.service
systemctl restart rviz-xhost.service || true

echo "[install_rviz_desktop_access] GDM autologin nvidia enabled"
echo "[install_rviz_desktop_access] xhost autostart -> ${AUTOSTART_DIR}/rviz-xhost.desktop"
echo "[install_rviz_desktop_access] systemd -> rviz-xhost.service (SSH RViz without desktop login)"
echo ""
echo "If RViz still fails NOW (before reboot), run once:"
echo "  sudo bash ${PKG_SCRIPTS}/fix_rviz_x11_access.sh"
echo ""
echo "Then:"
echo "  source /home/nvidia/f1tenth_ajou/install/setup.bash"
echo "  ros2 run localization_layer run_localization_rviz.sh"
