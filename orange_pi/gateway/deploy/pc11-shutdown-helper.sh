#!/usr/bin/env bash
# Root-only GPIO reader.  It has no UART or motor access; the gateway receives
# a loopback request and sends ESTOP/zero before scheduling system poweroff.
set -euo pipefail

readonly GPIO_BIN=/usr/bin/gpio
readonly CURL_BIN=/usr/bin/curl
readonly PIN="${ROVER_SHUTDOWN_GPIO_WPI:-6}"
readonly HOLD_MS="${ROVER_SHUTDOWN_GPIO_HOLD_MS:-3000}"
readonly GATEWAY_URL="http://127.0.0.1:8000/internal/gpio-shutdown"

if [[ -z "${ROVER_GPIO_SHUTDOWN_TOKEN:-}" ]]; then
  echo "ROVER_GPIO_SHUTDOWN_TOKEN is not configured" >&2
  exit 2
fi
if ! [[ "$PIN" =~ ^[0-9]+$ ]] || ! [[ "$HOLD_MS" =~ ^[0-9]+$ ]] || (( HOLD_MS < 1000 )); then
  echo "invalid PC11 GPIO helper configuration" >&2
  exit 2
fi

"$GPIO_BIN" mode "$PIN" in
"$GPIO_BIN" mode "$PIN" up
high_since_ms=0
# A floating/disconnected PC11 reads high.  Require one observed grounded
# baseline after boot so a loose wire can never cause an immediate shutdown.
seen_grounded_baseline=0

while true; do
  value="$("$GPIO_BIN" read "$PIN")"
  now_ms="$(date +%s%3N)"
  if [[ "$value" == "0" ]]; then
    seen_grounded_baseline=1
    high_since_ms=0
  elif [[ "$value" == "1" ]]; then
    if (( seen_grounded_baseline == 0 )); then
      high_since_ms=0
    elif (( high_since_ms == 0 )); then
      high_since_ms="$now_ms"
    elif (( now_ms - high_since_ms >= HOLD_MS )); then
      response="$("$CURL_BIN" --silent --show-error --fail --max-time 2 \
        --request POST --header "X-Rover-Gpio-Token: $ROVER_GPIO_SHUTDOWN_TOKEN" \
        "$GATEWAY_URL")" || {
          echo "gateway did not accept PC11 shutdown request" >&2
          high_since_ms="$now_ms"
          sleep 1
          continue
        }
      if [[ "$response" == *'"scheduled":true'* ]]; then
        exit 0
      fi
      echo "gateway refused PC11 shutdown request: $response" >&2
      high_since_ms="$now_ms"
    fi
  else
    echo "unexpected PC11 value: $value" >&2
    high_since_ms=0
  fi
  sleep 0.05
done
