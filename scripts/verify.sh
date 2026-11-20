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

# --- Task 3: light sleep ------------------------------------------------------
check_light_sleep() {
  need '#define ENABLE_LIGHT_SLEEP[[:space:]]+1' "$SKETCH" \
    "ENABLE_LIGHT_SLEEP is 1"
  need 'esp_sleep_enable_timer_wakeup' "$SKETCH" "timer wakeup is armed"
  need 'esp_light_sleep_start' "$SKETCH" "light sleep is entered"
  need 'enterLightSleepIfIdle\(now\)' "$SKETCH" "loop calls enterLightSleepIfIdle(now)"
  local body
  body="$(awk '/void enterLightSleepIfIdle/{f=1} f{print} f && /^}/{exit}' "$SKETCH")"
  if printf '%s' "$body" | grep -q 'g_sample_interval'; then
    pass "sleep duration is bounded by g_sample_interval"
  else
    fail "sleep duration is bounded by g_sample_interval"
  fi
}

# --- Task 4: offline alert queue ---------------------------------------------
check_alert_queue() {
  need '#define ALERT_QUEUE_SIZE[[:space:]]+8' "$SKETCH" \
    "ALERT_QUEUE_SIZE is 8"
  need 'struct PendingAlert' "$SKETCH" "PendingAlert struct exists"
  need 'void queueAlert\(' "$SKETCH" "queueAlert exists"
  need 'void flushAlertQueue\(' "$SKETCH" "flushAlertQueue exists"

  local blynk_body flush_body wifi_body total inside
  blynk_body="$(awk '/^void blynkUpdate\(/{f=1} f{print} f && /^}/{exit}' "$SKETCH")"
  if printf '%s' "$blynk_body" | grep -q 'flushAlertQueue()'; then
    pass "blynkUpdate flushes the queue"
  else
    fail "blynkUpdate flushes the queue"
  fi

  flush_body="$(awk '/^void flushAlertQueue\(/{f=1} f{print} f && /^}/{exit}' "$SKETCH")"
  total=$(grep -c 'Blynk\.logEvent(' "$SKETCH")
  inside=$(printf '%s' "$flush_body" | grep -c 'Blynk\.logEvent(')
  if [ "$total" -eq "$inside" ] && [ "$inside" -ge 1 ]; then
    pass "all Blynk.logEvent calls go through the queue"
  else
    fail "all Blynk.logEvent calls go through the queue (total=$total, in flush=$inside)"
  fi

  wifi_body="$(awk '/^void wifiDutyCycleUpdate\(/{f=1} f{print} f && /^}/{exit}' "$SKETCH")"
  if printf '%s' "$wifi_body" | grep -q 'flushAlertQueue()'; then
    pass "WiFi reconnect flushes the queue"
  else
    fail "WiFi reconnect flushes the queue"
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
check_light_sleep
check_alert_queue
check_compile

if [ "$FAILED" -ne 0 ]; then
  echo "VERIFY FAILED"
  exit 1
fi
echo "VERIFY OK"
