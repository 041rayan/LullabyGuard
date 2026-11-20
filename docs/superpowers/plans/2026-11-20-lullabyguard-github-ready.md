# LullabyGuard GitHub-Ready Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove live credentials from the sketch, close the four firmware gaps named in the design doc, and produce a push-ready GitHub repository.

**Architecture:** All firmware changes stay in the single existing `LullabyGuard.ino` (the project is deliberately one cooperative, allocation-free Arduino sketch). Secrets move to a gitignored header; new state is a handful of globals plus small fixed-size helpers called from the existing `loop()`. Verification is a compile via `arduino-cli` plus a static invariant script, since the target has no on-device test harness.

**Tech Stack:** Arduino-ESP32 core 3.3.12, ESP32-S3 (`esp32:esp32:esp32s3`), Arduino CLI, `Blynk` 1.4.0, `ArduinoJson` 7.4.3, `DHT sensor library` 1.4.7.

**Spec:** `docs/superpowers/specs/2026-11-20-lullabyguard-github-ready-design.md`

## Global Constraints

- Single-threaded cooperative `loop()`; no dynamic allocation; fixed-size buffers only.
- Do not modify fusion math, DSP, or state-transition thresholds.
- Comfort constants stay at 24–30 °C / 50–70 % RH (max deviation 8 / 30).
- Blynk template macros must remain above `#include <BlynkSimpleEsp32.h>`.
- `secrets.h` must never be tracked or committed.
- Compile target: `arduino-cli compile --fqbn esp32:esp32:esp32s3` (baseline: 1085100 bytes flash / 49520 bytes RAM).
- Work only in `/home/kasumi/lullaby`.

## Review Focus

- **Leaked credentials in tracked files or git history** → `scripts/verify.sh` secret scan runs in Tasks 1 and 7.
- **SAFE_HALT re-enabling a relay on the next loop** → static check in Task 5 asserts an early return at the top of both relay control functions.
- **WiFi duty cycle silencing Blynk manual override in NORMAL** → static check in Task 2 asserts the reconnect block is gated by the window flag.
- **Light sleep oversleeping a sensor/API/Blynk deadline** → Task 3 asserts sleep duration is capped by the next deadline and gated by `ENABLE_LIGHT_SLEEP`.
- **Offline alert queue dropping critical notifications** → Task 4 asserts overwrite-oldest semantics and a flush-on-reconnect call.

---

### Task 1: Secrets externalization and ignore rules

**Files:**
- Modify: `LullabyGuard.ino:17-40` (remove four credential defines, add include)
- Create: `secrets.h` (real values; untracked), `secrets.example.h` (placeholders; tracked), `.gitignore`
- Test: `scripts/verify.sh`

**Interfaces:**
- Produces: `secrets.h` defines `BLYNK_AUTH_TOKEN`, `WIFI_SSID`, `WIFI_PASS`, `ACCUWEATHER_API_KEY`; `LullabyGuard.ino` includes it before the Blynk include.

- [ ] **Step 1: Create `scripts/verify.sh`** with a `fail()` helper and a `check_no_secrets()` function that greps `LullabyGuard.ino` for any `#define` of `BLYNK_AUTH_TOKEN`, `WIFI_SSID`, `WIFI_PASS`, or `ACCUWEATHER_API_KEY` with a quoted non-empty value (`grep -Enq '#define (BLYNK_AUTH_TOKEN|WIFI_SSID|WIFI_PASS|ACCUWEATHER_API_KEY)[[:space:]]+"[^"]+"'`); exit non-zero on match. Never embed the real credential values in the script or plan. Include a compile step that copies the sketch to a temp dir named `LullabyGuard` and runs `arduino-cli compile --fqbn esp32:esp32:esp32s3`.

- [ ] **Step 2: Run `bash scripts/verify.sh` and confirm it fails** on the live secrets currently in the sketch.

- [ ] **Step 3: Write `secrets.h`** with the four real values from the current sketch, and `secrets.example.h` with the same macro names and obvious placeholders. Add `.gitignore` entries for `secrets.h`, `build/`, `.vscode/`, `.DS_Store`, `*.bin`, `*.elf`, `*.hex`.

- [ ] **Step 4: Edit `LullabyGuard.ino`** — delete the four credential `#define` lines, insert `#include "secrets.h"` above `#include <BlynkSimpleEsp32.h>`. Keep `BLYNK_TEMPLATE_ID`, `BLYNK_TEMPLATE_NAME`, `ACCUWEATHER_LOCATION`, `MOSQUITO_INDEX_ID` in the sketch.

- [ ] **Step 5: Run `bash scripts/verify.sh`** — expect secret scan clean and compile success.

- [ ] **Step 6: Commit**

```bash
git add .gitignore scripts/verify.sh secrets.example.h LullabyGuard.ino docs/
git commit -m "chore: externalize credentials and add verification script"
```

---

### Task 2: WiFi duty-cycling (design §G.2)

**Files:**
- Modify: `LullabyGuard.ino` (constants near WiFi block, globals near `g_wifi_connected`, `setup()`, `loop()`)

**Interfaces:**
- Produces: `void wifiDutyCycleUpdate(unsigned long now)`; globals `g_wifi_window_on`, `g_wifi_window_started`; constants `WIFI_WINDOW_ACTIVE_MS` = 30000, `WIFI_WINDOW_CYCLE_MS` = 300000.

- [ ] **Step 1: Add a static assertion to `scripts/verify.sh`** requiring: the two constants with the exact values above; a call to `wifiDutyCycleUpdate(now)` in `loop()`; and that the trailing `!g_wifi_connected` reconnect block contains `g_wifi_window_on`. Run it; expect FAIL.

- [ ] **Step 2: Implement constants, globals, and `wifiDutyCycleUpdate(unsigned long now)`.** Behaviour: if `g_state != STATE_NORMAL` or `!g_wifi_window_on` is false, force the window on and reconnect when needed; in NORMAL, after `WIFI_WINDOW_ACTIVE_MS` call `WiFi.mode(WIFI_OFF)`, set `g_wifi_connected = false`, `g_wifi_window_on = false`; after `WIFI_WINDOW_CYCLE_MS` set the window on, record the start, call `wifiConnect()`, and on success re-run `Blynk.config`/`Blynk.connect` and `flushAlertQueue()`.

- [ ] **Step 3: Wire it up** — call `wifiDutyCycleUpdate(now)` near the top of `loop()`; initialise `g_wifi_window_started = millis()` and `g_wifi_window_on = true` in `setup()` after the first connect; gate the existing 60 s reconnect block on `g_wifi_window_on`.

- [ ] **Step 4: Run `bash scripts/verify.sh`** — expect PASS.

- [ ] **Step 5: Commit** with message `feat: duty-cycle WiFi in NORMAL state`.

---

### Task 3: Light sleep (design §G.3)

**Files:**
- Modify: `LullabyGuard.ino` (config block, near `loop()`)

**Interfaces:**
- Consumes: `g_last_sensor_read`, `g_sample_interval`, `g_last_api_poll`, `g_last_blynk_update`, `g_wifi_connected`, `g_state`.
- Produces: `#define ENABLE_LIGHT_SLEEP 1`; `void enterLightSleepIfIdle(unsigned long now)`.

- [ ] **Step 1: Add a static assertion to `scripts/verify.sh`** requiring `#define ENABLE_LIGHT_SLEEP 1`, a call to `esp_sleep_enable_timer_wakeup`, a call to `esp_light_sleep_start`, and the identifier `g_sample_interval` inside `enterLightSleepIfIdle`. Run it; expect FAIL.

- [ ] **Step 2: Implement `enterLightSleepIfIdle(unsigned long now)`.** Return early unless `ENABLE_LIGHT_SLEEP`, `g_state == STATE_NORMAL`, and WiFi is off. Compute remaining time to the next sensor read from `g_last_sensor_read + g_sample_interval`; return if an API poll or Blynk window is already due; cap the sleep by time to the next WiFi window; return if under 50 ms. Otherwise call `esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000)` then `esp_light_sleep_start()`.

- [ ] **Step 3: Call it at the end of `loop()`**, after the reconnect block.

- [ ] **Step 4: Run `bash scripts/verify.sh`** — expect PASS and successful compile.

- [ ] **Step 5: Commit** with message `feat: add conservative light sleep in NORMAL state`.

---

### Task 4: Offline Blynk alert queue (design §I.2)

**Files:**
- Modify: `LullabyGuard.ino` (near Blynk section; `blynkUpdate`, `systemHealthCheck`)

**Interfaces:**
- Produces: `struct PendingAlert { char code[24]; char msg[96]; bool used; };`, `void queueAlert(const char* code, const char* msg)`, `void flushAlertQueue()`, constant `ALERT_QUEUE_SIZE 8`.

- [ ] **Step 1: Add a static assertion to `scripts/verify.sh`** requiring `ALERT_QUEUE_SIZE 8`, `flushAlertQueue()` called at the top of `blynkUpdate` after the WiFi guard, and no remaining direct `Blynk.logEvent` calls outside `flushAlertQueue`. Run it; expect FAIL.

- [ ] **Step 2: Implement the ring** with a fixed `PendingAlert g_alert_queue[ALERT_QUEUE_SIZE]` and `g_alert_head`. `queueAlert` copies into the first unused slot, or overwrites `g_alert_head` and advances it when full (critical alerts win over older ones). Use `snprintf` for bounded copies.

- [ ] **Step 3: Implement `flushAlertQueue()`** — return when offline; for each used slot call `Blynk.logEvent(code, msg)` and clear it.

- [ ] **Step 4: Replace every `Blynk.logEvent(...)` call** in `blynkUpdate()` and `systemHealthCheck()` with `queueAlert(...)`, preserving the existing message text and rate limits. Add `flushAlertQueue()` immediately after the `if (!g_wifi_connected) return;` guard in `blynkUpdate()`.

- [ ] **Step 5: Run `bash scripts/verify.sh`** — expect PASS.

- [ ] **Step 6: Commit** with message `feat: queue Blynk alerts while offline`.

---

### Task 5: Latched SAFE_HALT (design §I.4)

**Files:**
- Modify: `LullabyGuard.ino` (globals, `controlFanRelay`, `controlMosquitoRelay`, `systemHealthCheck`)

**Interfaces:**
- Produces: `bool g_safe_halt = false;`; relay functions respect it before any manual-override handling.

- [ ] **Step 1: Add a static assertion to `scripts/verify.sh`** that extracts the bodies of `controlFanRelay` and `controlMosquitoRelay` and requires `g_safe_halt` to appear before `g_fan_override` / `g_mosq_override` respectively. Run it; expect FAIL.

- [ ] **Step 2: Declare `bool g_safe_halt = false;`** alongside the other system-state globals.

- [ ] **Step 3: Add the guard at the top of both relay control functions** — when `g_safe_halt`, force the relay OFF, clear its `g_*_relay_on` flag, and return, so neither auto logic nor manual override can re-energise it.

- [ ] **Step 4: Update `systemHealthCheck()`** — on `failed >= 3`, set `g_safe_halt = true` once, force both relays off, and queue `system_fault`; on `g_safe_halt && failed < 2`, clear the latch and queue `system_recovered`. Keep the existing `failed >= 2` degraded warning.

- [ ] **Step 5: Run `bash scripts/verify.sh`** — expect PASS.

- [ ] **Step 6: Commit** with message `fix: latch SAFE_HALT so relays stay off`.

---

### Task 6: Repository documentation and manifest

**Files:**
- Create: `README.md`, `LICENSE`, `sketch.yaml`
- Modify: `LullabyGuard.ino` (header comment: reference design doc and secrets setup)
- Create: `docs/superpowers/specs/...` and `docs/superpowers/plans/...` — already present from earlier steps.

**Interfaces:**
- Consumes: all of the above.

- [ ] **Step 1: Write `sketch.yaml`** with a `default` profile: `fqbn: esp32:esp32:esp32s3`; `platforms: - platform: esp32:esp32 (3.3.12)`; libraries `ArduinoJson (7.4.3)`, `Blynk (1.4.0)`, `DHT sensor library (1.4.7)`; `default_profile: default`. Verify with `arduino-cli compile --profile default` in a temp copy; expect success.

- [ ] **Step 2: Write `LICENSE`** using the MIT text, copyright 2026 LullabyGuard contributors.

- [ ] **Step 3: Write a detailed `README.md`** with: badge row (ESP32-S3, Arduino, MIT), one-line pitch, hero feature grid, Mermaid architecture flowchart and state diagram, complete pin/wiring table, hardware BOM, quick start (`arduino-cli` + Arduino IDE), mandatory secrets setup with rotation warning, Blynk dashboard template pins table, AccuWeather setup, build/flash/serial-monitor commands, calibration behaviour, adaptive sampling table, "Known discrepancies" section naming the comfort-constant mismatch and the unimplemented GPIO sound wake source, troubleshooting, and contributing/licence links. Use consistent headings, tables, and fenced blocks; no external image dependencies.

- [ ] **Step 4: Update the sketch header comment** with two lines: design-doc reference and "copy `secrets.example.h` to `secrets.h` before building".

- [ ] **Step 5: Run `bash scripts/verify.sh`** — expect PASS.

- [ ] **Step 6: Commit** with message `docs: add README, licence, and reproducible build profile`.

---

### Task 7: Initialise git and prepare for push

**Files:**
- Repository root: `/home/kasumi/lullaby`

**Interfaces:**
- Consumes: all tracked files from Tasks 1–6.

- [ ] **Step 1: Run `git init -b main`** in `/home/kasumi/lullaby`.

- [ ] **Step 2: Stage the intended files explicitly** (`git add README.md LICENSE .gitignore sketch.yaml LullabyGuard.ino secrets.example.h docs/ scripts/`), then run `git status --short` and confirm `secrets.h` is absent.

- [ ] **Step 3: Run `bash scripts/verify.sh`** once more; expect PASS.

- [ ] **Step 4: Commit** with message `feat: initial LullabyGuard release`.

- [ ] **Step 5: Verify history is clean** with `git log --stat --oneline`, a `git grep -nE '#define (BLYNK_AUTH_TOKEN|WIFI_SSID|WIFI_PASS|ACCUWEATHER_API_KEY)[[:space:]]+"[^"]+"' $(git rev-list --all) -- LullabyGuard.ino` (expect no matches), and `git log --all --oneline -- secrets.h` (expect no output — the real secrets file was never tracked).

- [ ] **Step 6: Report push readiness** — print `git remote -v` (empty), and give the user the exact `git remote add origin <URL>` and `git push -u origin main` commands plus the credential-rotation reminder. Do not push.
