#!/usr/bin/env bash
set -euo pipefail

[[ ${EUID} -eq 0 ]] || { echo "请使用 sudo bash install.sh" >&2; exit 1; }
for command_name in bluetoothctl systemctl python3 install cp; do
  command -v "${command_name}" >/dev/null || { echo "缺少命令: ${command_name}" >&2; exit 1; }
done

SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR=/opt/rover-one-ble
install -d -m 0755 -- "${TARGET_DIR}"
install -m 0644 -- "${SOURCE_DIR}/bridge.py" "${SOURCE_DIR}/frame_codec.py" \
  "${SOURCE_DIR}/gateway_lease.py" "${SOURCE_DIR}/pair_event.py" \
  "${SOURCE_DIR}/requirements.txt" "${TARGET_DIR}/"
install -m 0755 -- "${SOURCE_DIR}/pair-3min.sh" "${TARGET_DIR}/pair-3min.sh"
python3 -m venv "${TARGET_DIR}/.venv"
"${TARGET_DIR}/.venv/bin/pip" install --requirement "${TARGET_DIR}/requirements.txt"
"${TARGET_DIR}/.venv/bin/python" -m py_compile \
  "${TARGET_DIR}/bridge.py" "${TARGET_DIR}/frame_codec.py" \
  "${TARGET_DIR}/gateway_lease.py" "${TARGET_DIR}/pair_event.py"

install -m 0644 -- "${SOURCE_DIR}/rover-ble-bridge.service" /etc/systemd/system/
install -d -m 0755 -- /etc/rover-one
if [[ ! -f /etc/rover-one/ble.env ]]; then
  gateway_origin="$(sed -n 's/^ROVER_ALLOWED_ORIGINS=//p' /etc/rover-one/gateway.env 2>/dev/null | cut -d, -f1)"
  [[ -n "${gateway_origin}" ]] || gateway_origin=http://10.42.0.1
  printf 'ROVER_BLE_ORIGIN=%s\n' "${gateway_origin}" > /etc/rover-one/ble.env
  chmod 0644 /etc/rover-one/ble.env
fi
bluetoothctl power on
bluetoothctl system-alias ROVER-ONE-BLE
systemctl daemon-reload
systemctl enable --now rover-ble-bridge.service
systemctl --no-pager --full status rover-ble-bridge.service
