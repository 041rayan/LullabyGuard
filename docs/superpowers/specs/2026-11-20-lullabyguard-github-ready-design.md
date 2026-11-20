# LullabyGuard — GitHub-Ready + Firmware Gap Closure

Short design, 2026-11-20. Chosen approach: **externalize secrets and implement the
missing firmware features** (the user picked "modify" over "document gaps only").

## Intent / why

`LullabyGuard.ino` currently ships live credentials (Blynk token, WiFi SSID/password,
AccuWeather key) in source, and the firmware omits several behaviours promised by
`LullabyGuard_SystemDesign.md` rev 2.0. The repo also has no git history, license,
dependency manifest, or README. Goal: a safe, self-documenting repository that can be
pushed to GitHub without leaking credentials, with firmware that matches its spec.

## Goals

1. No secret material in tracked files; rotate-exposed credentials documented.
2. Implement §G.2 WiFi duty-cycling, §G.3 light sleep, §I.2 offline alert queueing,
   and a latched §I.4 SAFE_HALT.
3. GitHub-ready artifacts: README, LICENSE, .gitignore, dependency manifest,
   initial git commit.
4. Keep all changes local to `/home/kasumi/lullaby` (the maintained copy).

## Non-goals

- No changes to the fusion math, state thresholds, or DSP algorithms.
- No PlatformIO migration, no RTOS tasks, no dynamic allocation.
- No actual `git push` (no remote configured / no credentials).
- No schematic or PCB work.

## Anti-goals (would make this a failure)

- Publishing working credentials, even in git history.
- A light-sleep or WiFi-cycle change that breaks Blynk control or leaves a relay in an
  unsafe state.
- Silently changing thermal comfort tuning without telling the user.

## Constraints

- Single-threaded cooperative `loop()`, no heap allocation, fixed buffers (design §Appendix B).
- Blynk template defines must stay above the Blynk include.
- ESP32-S3 Arduino framework; must still compile with `arduino-cli`/Arduino IDE.

## Approaches considered

| Approach | Trade-off | Verdict |
|---|---|---|
| A — Docs only, leave firmware as-is | Zero regression risk, but spec gaps and leaked secrets remain | Rejected by user |
| **B — Externalize secrets + implement gaps** | Moderate code change, small regression risk, repo matches spec | **Chosen** |
| C — Full redesign/rewrite | High risk, out of scope | Rejected |

## Decisions (how it gets built)

1. **Secrets:** `secrets.h` (real values, gitignored) + `secrets.example.h`
   (placeholders, tracked). Sketch includes `secrets.h` before the Blynk include.
   README tells the user to rotate the three exposed credentials.
2. **WiFi duty cycle (§G.2):** NORMAL state = 30 s connected window per 5 min;
   MILD/HIGH/CRITICAL = always connected. Implemented with a non-blocking timer in
   `loop()`; `WiFi.mode(WIFI_OFF)` between windows, reconnect at window start.
   A 30 s startup window guarantees first Blynk sync.
3. **Light sleep (§G.3):** only entered in NORMAL state *while the WiFi radio is off*,
   using `esp_sleep_enable_timer_wakeup()` + `esp_light_sleep_start()`. Sleep duration
   is capped by the next scheduled sensor/Blynk/API deadline so nothing is starved.
   No GPIO wake source (I2S threshold wake needs hardware wiring that isn't specified).
4. **Offline queue (§I.2):** small fixed-size ring of pending Blynk `logEvent` messages;
   flushed on reconnect. Telemetry writes stay dropped while offline (they are periodic
   and the next update supersedes them).
5. **SAFE_HALT latch (§I.4):** `g_safe_halt` set when ≥3 sensors fail. While latched,
   both relays are forced OFF on every control call, including manual override.
   Clears only after failures drop below 2 (hysteresis) or on reboot.
6. **Comfort constants:** leave the code's 24–30 °C / 50–70 %RH values unchanged.
   They likely reflect deliberate tropical tuning; the doc's 20–26 °C / 40–60 %RH are
   treated as a doc discrepancy and recorded in the README's "Known discrepancies".
7. **Repo artifacts:** MIT LICENSE, `.gitignore` (secrets + build artifacts),
   `sketch.yaml` dependency manifest, detailed README with architecture/state diagrams,
   pin tables, setup steps, and a "Known gaps / discrepancies" section.
8. **Git:** `git init`, stage, one initial commit. No remote, no push.

## Left to the builder

- Exact timer constants and function placement inside the existing `loop()`.
- Choice of ring-buffer depth for the offline queue (small, fixed — suggested 8).
- Final README wording and diagram styling.
