#!/usr/bin/env bash
set -euo pipefail

[[ ${EUID} -eq 0 ]] || { echo "请使用 sudo bash pair-3min.sh" >&2; exit 1; }

cleanup() {
  bluetoothctl discoverable off >/dev/null 2>&1 || true
  bluetoothctl pairable off >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

pair_window_seconds="${ROVER_BLE_PAIR_WINDOW_SECONDS:-180}"
case "${pair_window_seconds}" in
  ''|*[!0-9]*) echo "ROVER_BLE_PAIR_WINDOW_SECONDS 必须是 1..180 的整数" >&2; exit 2 ;;
esac
if (( pair_window_seconds < 1 || pair_window_seconds > 180 )); then
  echo "ROVER_BLE_PAIR_WINDOW_SECONDS 必须是 1..180 的整数" >&2
  exit 2
fi

# Keep bluetoothctl alive so its NoInputNoOutput agent remains registered for
# the whole window. Encrypted GATT writes still require a completed bond.
{
  # BlueZ agent registration is asynchronous. Starting the agent on the
  # bluetoothctl command line and waiting avoids racing default-agent.
  sleep 1
  echo "power on"
  sleep 1
  echo "default-agent"
  sleep 1
  echo "pairable on"
  echo "discoverable-timeout ${pair_window_seconds}"
  echo "discoverable on"
  sleep "${pair_window_seconds}"
  echo "quit"
} | bluetoothctl --agent NoInputNoOutput
