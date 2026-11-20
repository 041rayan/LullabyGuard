#!/usr/bin/env bash
# Static and compile verification for LullabyGuard.
# Every check is additive: later tasks add checks here before their own code.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SKETCH="$ROOT/LullabyGuard.ino"
FAILED=0

fail() { echo "FAIL: $*"; FAILED=1; }
pass() { echo "PASS: $*"; }
need() {
  local pattern="$1" file="$2" label="$3"
  if grep -Eq "$pattern" "$file"; then pass "$label"; else fail "$label"; fi
}
absent() {
  local pattern="$1" file="$2" label="$3"
  if grep -Eq "$pattern" "$file"; then fail "$label"; else pass "$label"; fi
}

# --- Task 1: credentials must not live in the sketch -------------------------
check_no_secrets() {
  absent '#define (BLYNK_AUTH_TOKEN|WIFI_SSID|WIFI_PASS|ACCUWEATHER_API_KEY)[[:space:]]+"[^"]+"' \
    "$SKETCH" "no credential defines in sketch"
  need '#include "secrets.h"' "$SKETCH" "sketch includes secrets.h"
  if [ -f "$ROOT/.gitignore" ] && grep -qx 'secrets.h' "$ROOT/.gitignore"; then
    pass "secrets.h is gitignored"
  else
    fail "secrets.h is gitignored"
  fi
}

# --- Task 2: WiFi duty-cycling ------------------------------------------------
check_wifi_duty_cycle() {
  need '#define WIFI_WINDOW_ACTIVE_MS[[:space:]]+30000UL' "$SKETCH" \
    "WIFI_WINDOW_ACTIVE_MS is 30000"
  need '#define WIFI_WINDOW_CYCLE_MS[[:space:]]+300000UL' "$SKETCH" \
    "WIFI_WINDOW_CYCLE_MS is 300000"
  need 'wifiDutyCycleUpdate\(now\)' "$SKETCH" "loop calls wifiDutyCycleUpdate(now)"
  local block
  block="$(awk '/WiFi reconnection check/{f=1} f' "$SKETCH")"
  if printf '%s' "$block" | grep -q 'g_wifi_window_on'; then
    pass "reconnect block is gated by g_wifi_window_on"
  else
    fail "reconnect block is gated by g_wifi_window_on"
  fi
}

# --- Compile gate -------------------------------------------------------------
check_compile() {
  local tmp log
  tmp="$(mktemp -d /tmp/opencode/lg-verify.XXXXXX)"
  mkdir -p "$tmp/LullabyGuard"
  cp "$SKETCH" "$tmp/LullabyGuard/"
  if [ -f "$ROOT/secrets.h" ]; then
    cp "$ROOT/secrets.h" "$tmp/LullabyGuard/"
  fi
  log="$tmp/compile.log"
  if (cd "$tmp" && arduino-cli compile --fqbn esp32:esp32:esp32s3 LullabyGuard) >"$log" 2>&1; then
    pass "arduino-cli compile (esp32:esp32:esp32s3)"
  else
    fail "arduino-cli compile (esp32:esp32:esp32s3)"
    tail -n 20 "$log"
  fi
  rm -rf "$tmp"
}

check_no_secrets
check_wifi_duty_cycle
check_compile

if [ "$FAILED" -ne 0 ]; then
  echo "VERIFY FAILED"
  exit 1
fi
echo "VERIFY OK"
