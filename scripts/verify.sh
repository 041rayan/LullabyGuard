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
check_compile

if [ "$FAILED" -ne 0 ]; then
  echo "VERIFY FAILED"
  exit 1
fi
echo "VERIFY OK"
