#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo ./fix_ch341_bind.sh" >&2
  exit 1
fi

cd "$(dirname "$0")"

for service in brltty-udev.service brltty.service ModemManager.service; do
  if systemctl is-active --quiet "${service}" 2>/dev/null; then
    echo "Stopping ${service} so it does not grab CH340"
    systemctl stop "${service}" || true
  fi
done

if ! lsmod | grep -q '^ch341 '; then
  if [[ -f ./ch341.ko ]]; then
    insmod ./ch341.ko || true
  else
    modprobe ch341
  fi
fi

found=0
for dev in /sys/bus/usb/devices/*; do
  [[ -f "${dev}/idVendor" && -f "${dev}/idProduct" ]] || continue
  vendor="$(cat "${dev}/idVendor")"
  product="$(cat "${dev}/idProduct")"
  [[ "${vendor}:${product}" == "1a86:7523" ]] || continue

  found=1
  name="$(basename "${dev}")"
  interface="${name}:1.0"

  driver_path="/sys/bus/usb/devices/${interface}/driver"
  if [[ -L "${driver_path}" ]]; then
    driver_name="$(basename "$(readlink -f "${driver_path}")")"
    if [[ "${driver_name}" == "ch341" ]]; then
      echo "CH340 ${interface} already bound to ch341"
    else
      echo "CH340 ${interface} is bound to ${driver_name}; rebinding to ch341"
      echo "${interface}" > "/sys/bus/usb/drivers/${driver_name}/unbind"
      sleep 0.2
      echo "${interface}" > /sys/bus/usb/drivers/ch341/bind
    fi
  else
    echo "Binding CH340 interface ${interface} to ch341"
    echo "${interface}" > /sys/bus/usb/drivers/ch341/bind
  fi
done

if [[ "${found}" -eq 0 ]]; then
  echo "No CH340 device found. Check USB cable/port." >&2
  exit 2
fi

sleep 0.3
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
