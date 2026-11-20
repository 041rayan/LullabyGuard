# LullabyGuard: Smart Infant Monitoring System — Complete System Design

## Document Revision: 2.0 (Event-Driven Fusion)
## Target Platform: ESP32-S3-WROOM-1
## Framework: Arduino (ESP-IDF underlying)

---

# A. SYSTEM ARCHITECTURE

## A.1 High-Level Block Diagram (Text)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          ESP32-S3-WROOM-1                                      │
│                                                                                │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                      │
│  │  MPU6050     │  │   DHT22      │  │  INMP441     │                      │
│  │  (I2C)       │  │  (1-Wire)    │  │  (I2S)       │                      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                      │
│         │                  │                  │                              │
│         ▼                  ▼                  ▼                              │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │              SENSOR PREPROCESSING + FEATURE EXTRACTION                  │  │
│  │  • Calibration • Moving Avg • RMS • DC Removal • EMA                   │  │
│  │  • Jerk/Variance/Rhythmic (motion) • ZCR/Envelope/Burst (sound)        │  │
│  │  • Heat Index (environment)                                             │  │
│  └────────────────────────────────┬───────────────────────────────────────┘  │
│                                   │                                          │
│                                   ▼                                          │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │              DYNAMIC SENSOR CONFIDENCE (per-sensor [0..1])              │  │
│  │  Sound: variance-aware │ Motion: rhythm-reject │ Env: staleness-aware  │  │
│  └────────────────────────────────┬───────────────────────────────────────┘  │
│                                   │                                          │
│                                   ▼                                          │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │              EVENT-DRIVEN FUSION ENGINE                                  │  │
│  │  • Confidence-weighted base scores                                      │  │
│  │  • Event detectors: cry, agitation, heat_stress, env_stress             │  │
│  │  • Severity = max(base_DI, event_severity) + temporal + episodes        │  │
│  │  • EMA smoothing → DI [0–100]                                           │  │
│  └────────────────────────────────┬───────────────────────────────────────┘  │
│                                   │                                          │
│                                   ▼                                          │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │              PREDICTION + TEMPORAL MODEL                                │  │
│  │  • Slope (linear regression) • Acceleration (2nd derivative)            │  │
│  │  • Episode intensity (decaying accumulator) • DI instability            │  │
│  └────────────────────────────────┬───────────────────────────────────────┘  │
│                                   │                                          │
│                                   ▼                                          │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │              STATE MACHINE (confidence-gated escalation)                 │  │
│  │  NORMAL → MILD_DISCOMFORT → HIGH_DISCOMFORT → CRITICAL                  │  │
│  │  Escalation: requires conf ≥ 0.4 │ De-escalation: always allowed       │  │
│  └────────────────────────────────┬───────────────────────────────────────┘  │
│                                   │                                          │
│                        ┌──────────┼──────────┐                              │
│                        ▼          ▼          ▼                              │
│                  ┌──────────┐ ┌────────┐ ┌──────────┐                      │
│                  │Relay Fan │ │ Blynk  │ │Relay Mosq│                      │
│                  │          │ │  IoT   │ │Repellent │                      │
│                  └──────────┘ └────────┘ └──────────┘                      │
│                                   ▲                                          │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │              MOSQUITO SUBSYSTEM (INDEPENDENT)                           │  │
│  │  • AccuWeather HTTPS API poll (every 30 min)                            │  │
│  │  • Index threshold → relay activation                                   │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
```

## A.2 Data Flow

1. Sensors sampled at adaptive intervals (250ms–5s based on state)
2. Raw data → preprocessed (filtered, calibrated, DC-removed)
3. DSP feature extraction (ZCR, jerk, heat index, energy variance)
4. Preprocessed data → normalized scores [0, 1]
5. Dynamic confidence estimation per sensor
6. Effective scores = raw scores × confidence
7. Event detection (cry, agitation, heat stress, env stress)
8. Severity = max(base_DI, event_severity) + temporal + episode boosts
9. DI smoothed via EMA (α=0.3)
10. Prediction: slope + acceleration + instability + temporal model
11. State machine transitions (confidence-gated escalation)
12. Actuator control + event-aware Blynk notifications

## A.3 Pin Allocation (ESP32-S3)

| Component       | Signal  | GPIO |
|----------------|---------|------|
| MPU6050 SDA    | I2C     | 8    |
| MPU6050 SCL    | I2C     | 9    |
| DHT22 DATA     | 1-Wire  | 4    |
| INMP441 SCK    | I2S     | 40   |
| INMP441 WS     | I2S     | 39   |
| INMP441 SD     | I2S     | 41   |
| Relay 1 (Fan)  | Digital | 6    |
| Relay 2 (Mosq) | Digital | 7    |

---

# B. SENSOR PREPROCESSING METHODS

## B.1 MPU6050 Feature Extraction

**Raw Input:** 3-axis acceleration (g), 3-axis gyroscope (°/s)

**Processing Pipeline:**
1. Burst-read 6 axes (14 bytes, single I2C transaction)
2. Apply calibration offsets (computed at startup over 500 samples, 10s)
3. Moving average filter (window = 20 samples per axis)
4. Compute filtered magnitude: `|a| = sqrt(fax² + fay² + faz²)`
5. Delta-from-gravity: `delta_g = |filt_mag - 1.0g|`
6. Gyroscope magnitude: `gyro_mag = sqrt(gx² + gy² + gz²)` (raw, unfiltered)

**Extracted Features:**
- `delta_g` — deviation from rest
- `gyro_mag_dps` — rotational activity
- `pitch_deg`, `roll_deg` — orientation
- **v2 DSP:** `jerk` (EMA of |Δdelta_g|), `variance` (rolling 8-sample), `rhythmic` (autocorrelation)

**Normalization (gyro-gated):**
```
accel_score = clamp(delta_g / 2.0, 0, 1)
gyro_score  = clamp(gyro_mag / 30.0, 0, 1)
motion_score = accel_score × gyro_score
```
Gyro gating rejects vibration (high accel, low gyro) — requires both translation AND rotation.

## B.2 Sound Processing (INMP441)

**Raw Input:** I2S 32-bit PCM samples at 16 kHz, 1024 samples/frame

**Processing Pipeline:**
1. Read I2S DMA buffer (1024 × 32-bit samples)
2. Right-shift by 8 bits, compute DC mean for AC coupling
3. AC-coupled RMS: `rms_raw = sqrt(Σ(sample - dc_mean)² / N)`
4. Fast EMA smoothing: `rms_fast = EMA(rms_fast, rms_raw, α=0.5)` (~3 samples to converge)
5. Proportional threshold: `cry_delta = max(100000, floor × 0.3)`
6. Detection threshold: `high_thresh = floor + cry_delta × 0.3`
7. Adaptive noise floor: tracks ambient during quiet, faster in first 2min
8. Sustained detection: high energy persisting >300ms

**Calibration (startup):**
- 5-second I2S settling flush (INMP441 transient rejection)
- 3-second calibration using lower of average and last-frame RMS
- Floor = base_rms × 1.5 (50% safety margin)

**v2 DSP Features (no FFT):**
- `zcr` — zero-crossing rate (distinguishes cry vs click vs hum)
- `energy_variance` — rolling variance of RMS over 6 frames
- `envelope` — EMA-smoothed energy level (α=0.3)
- `burst_count` — spikes >2× envelope (cooldown 5s)

**Normalization:**
```
sound_score = clamp((rms_fast - noise_floor) / cry_delta, 0, 1)
```

## B.3 DHT22 Environmental Processing

**Raw Input:** Temperature (°C, ±0.5°C), Humidity (%RH, ±2%)

**Processing Pipeline:**
1. Read every 2s (DHT22 minimum interval)
2. Validate: reject temp < -10 or > 60, humidity < 0 or > 100
3. Use last valid reading if current fails (max 60s stale)
4. Apply 5-sample median filter (spike rejection)
5. Compute deviations from comfort zone (20–26°C, 40–60%RH)
6. **v2: Compute heat index** (NWS Rothfusz regression, C→F→C conversion)

**v2 Heat Index Algorithm:**
- Below 27°C: heat_index = temperature (no correction needed)
- Above 27°C: Steadman screening → full Rothfusz regression with humidity adjustments
- Captures perceived temperature accounting for humidity stress

**Normalization (Euclidean norm with heat index):**
```
hi_deviation = max(0, heat_index - 26.0)
temp_contrib = clamp(max(temp_deviation, hi_deviation × 0.5) / 6.0, 0, 1)
hum_contrib  = clamp(humidity_deviation / 20.0, 0, 1)
env_score    = clamp(sqrt((temp_contrib² + hum_contrib²) / 2), 0, 1)
```

## B.4 AccuWeather API Parsing

**Endpoint:** `GET /indices/v1/daily/1day/{locationKey}/{indexId}`
- Index ID for mosquito activity: 17

**Response Parse:**
```json
[{"Value": 7, "Category": "High", ...}]
```

**Processing:**
1. HTTP GET every 30 minutes (duty-cycled)
2. Parse JSON → extract `Value` field (0–10 scale)
3. On HTTP failure: retain last valid value for up to 2 hours, then set to moderate (5)
4. On JSON parse failure: discard, keep previous value

**Normalization:**
- `mosquito_index = api_value / 10.0` (already 0–1 scale)

---

# C. FUSION ALGORITHM (v2 — Event-Driven)

## C.1 Overview

The fusion engine uses a **hybrid approach**: a confidence-weighted base score provides the floor,
while behavioral event detectors can elevate severity based on confirmed multi-signal patterns.

## C.2 Step-by-Step Pipeline

```
1. Compute effective scores:     snd_eff = sound_score × conf_sound
                                 mot_eff = motion_score × conf_motion
                                 env_eff = env_score × conf_env

2. Update sensor confidence:     (variance, health, rhythm based)

3. Detect behavioral events:     cry, agitation, heat_stress, env_stress, rapid_escalation

4. Compute severity:
   base_di        = (0.45×snd_eff + 0.30×mot_eff + 0.25×env_eff) × 100
   event_severity = (0.50×cry + 0.30×agit + 0.20×env_stress) × 100
   temporal_boost = clamp(episode_intensity × 5, 0, 20)
   episode_boost  = clamp(cry_episodes × 3, 0, 15)
   severity       = clamp(max(base_di, event_severity) + temporal + episode, 0, 100)

5. DI output:                    di_raw = severity
                                 di_smooth = EMA(di_smooth, di_raw, 0.3)

6. Count active sensors:         effective score > 0.3 threshold
```

## C.3 Base Weight Assignment

| Sensor      | Weight | Rationale                              |
|-------------|--------|----------------------------------------|
| Sound       | 0.45   | Most direct indicator of distress      |
| Motion      | 0.30   | Restlessness correlates with discomfort|
| Environment | 0.25   | Indirect, slow-changing contributor    |

## C.4 Dynamic Sensor Confidence

| Sensor | Degrades when | Recovers when |
|--------|---------------|---------------|
| Sound  | Energy variance ≈ 0 (constant noise), sensor degraded/failed | Varying signal patterns |
| Motion | Rhythmic autocorrelation detected (fan/rocker) | Non-rhythmic motion |
| Env    | Sensor failed or degraded (stale reads) | Good reads |

Confidence adapts via EMA (decay 0.005/cycle, recovery 0.02/cycle). Range: [0, 1].

## C.5 Event Detectors

Events require **multi-signal corroboration** — never single-sensor:

| Event | Conditions | Sustain |
|-------|-----------|---------|
| `cry_event` | snd_eff > 0.4 + sustained + ZCR ∈ [0.05, 0.35] | 2s |
| `agitation_event` | mot_eff > 0.25 + jerk > 0.08 + NOT rhythmic | 3s |
| `heat_stress_event` | heat_index > comfort_high + 1.5°C | instant |
| `env_stress_event` | env_eff > 0.5 OR heat_stress | instant |
| `rapid_escalation` | DI_acceleration > 1.0 + slope > warning | instant |

**Cry event** also tracks episodes: increments counter on cry end, resets after 2min quiet.
Motion correlation boosts cry intensity by 0.15 if mot_eff > 0.15 during cry.

## C.6 Temporal Smoothing

```
DI_smooth[n] = 0.3 × DI_raw[n] + 0.7 × DI_smooth[n-1]
```
~3-sample settling. Fast enough for real events, slow enough to reject single-frame noise.

## C.7 Multi-Sensor Validation Gate

- Active sensors: effective score (confidence-weighted) > 0.3
- **Minimum 2 active** for HIGH, **2 sustained OR 3** for CRITICAL
- Escalation also requires `fusion_confidence ≥ 0.4`

---

# D. STATE MACHINE

## D.1 States

| State             | DI Range  | Condition                                     |
|-------------------|-----------|-----------------------------------------------|
| NORMAL            | 0–25      | DI_smooth < 25                                |
| MILD_DISCOMFORT   | 25–50     | DI_smooth ≥ 25 AND < 50                       |
| HIGH_DISCOMFORT   | 50–75     | DI_smooth ≥ 50 AND ≥2 sensors active          |
| CRITICAL          | 75–100    | DI_smooth ≥ 75 AND multi-modal OR sustained   |

## D.2 Transition Rules (with Hysteresis + Confidence Gate)

```
NORMAL → MILD:      DI ≥ 30 AND conf_ok (≥0.4)
MILD → NORMAL:      DI < 20 (always allowed)

MILD → HIGH:        DI ≥ 55 AND active ≥ 2 AND conf_ok
                    OR rapid_escalation AND cry_event AND conf_ok (v2 bypass)
HIGH → MILD:        DI < 40 OR active < 2 (always allowed)

HIGH → CRITICAL:    DI ≥ 80 AND active ≥ 2 AND sustained >5s AND conf_ok
                    OR DI ≥ 75 AND active ≥ 3 AND conf_ok
CRITICAL → HIGH:    DI < 60 (always allowed)
```

**Key v2 changes:**
- **Escalation requires confidence ≥ 0.4** — prevents false alarms from degraded sensors
- **De-escalation always allowed** regardless of confidence (safety first)
- **Rapid escalation bypass** from MILD→HIGH when cry + acceleration detected

## D.3 Timing Constraints

- Minimum dwell time in any state: 2 seconds (prevents rapid toggling)
- CRITICAL state: minimum dwell = 10 seconds
- All transitions respect dwell before re-evaluating

---

# E. RELAY CONTROL LOGIC

## E.1 Fan/Light Control (Relay 1)

| Condition                                    | Action       |
|----------------------------------------------|--------------|
| State = HIGH_DISCOMFORT AND env_score > 0.5  | Fan ON       |
| State = CRITICAL                             | Fan ON       |
| temp > 28°C (absolute override)             | Fan ON       |
| State returns to NORMAL for > 30s            | Fan OFF      |
| Manual override via Blynk                    | User control |

**De-bounce:** Relay state changes require 5-second hold before toggling back.

## E.2 Mosquito Repellent Control (Relay 2) — INDEPENDENT SUBSYSTEM

| Condition                          | Action           |
|------------------------------------|------------------|
| mosquito_index ≥ 0.6 (API value ≥6)| Repellent ON    |
| mosquito_index < 0.4 (API value <4)| Repellent OFF   |
| API unreachable for >2 hours       | Repellent ON (safe default) |
| Manual override via Blynk          | User control     |

**Key: Mosquito relay is COMPLETELY independent of infant discomfort state.**
It operates solely on AccuWeather API data + safe defaults.

---

# F. PREDICTION + TEMPORAL MODEL

## F.1 Slope-Based Trend Detection

- Circular buffer: last 10 DI samples (sampled at adaptive rate)
- Least-squares linear fit → `slope` (DI-units/sample)
- slope > 3.0 → "RISING TREND" warning
- slope > 5.0 → "RAPID ESCALATION" alert

## F.2 v2 Temporal Behavior Model

| Feature | Computation | Purpose |
|---------|-------------|---------|
| **DI acceleration** | `slope[n] - slope[n-1]` | Detects accelerating trends |
| **Episode intensity** | Decaying accumulator (×0.92/cycle + event input) | Tracks cumulative stress |
| **DI instability** | Variance of DI history buffer | Detects erratic behavior |

**Episode intensity** grows when events are active:
```
input = 0.5×cry_intensity + 0.3×agit_intensity + 0.2×env_intensity
episode_intensity = episode_intensity × 0.92 + input
```
Capped at 5.0, decays naturally during quiet periods.

## F.3 Predictive Alert Logic

- Prediction sends Blynk notifications only (rate-limited to 1 per 5min)
- Does NOT auto-trigger state transitions (conservative safety)
- v2: `rapid_escalation` event feeds into state machine bypass (MILD→HIGH)

---

# G. POWER OPTIMIZATION STRATEGY

## G.1 Adaptive Sampling

| System State    | Sensor Read Interval | Sound Sampling |
|-----------------|---------------------|----------------|
| NORMAL          | 5000 ms             | 2 Hz (low)     |
| MILD_DISCOMFORT | 2000 ms             | 5 Hz           |
| HIGH_DISCOMFORT | 500 ms              | 10 Hz          |
| CRITICAL        | 250 ms              | Continuous     |

## G.2 WiFi Duty Cycling

- NORMAL state: WiFi active 30s every 5 minutes (for Blynk sync + API poll)
- MILD/HIGH: WiFi always on
- CRITICAL: WiFi always on, notifications sent immediately
- API poll: Every 30 minutes regardless of state (batched with WiFi window)

## G.3 Sleep Mode Usage

- Between sensor reads in NORMAL state: light sleep (CPU halted, RTC running)
- Wake sources: Timer (next sample interval) OR GPIO interrupt (sound threshold via I2S)
- **Never** deep sleep — RTC state loss would reset the system

## G.4 Estimated Power Budget

| Mode            | Current (mA) | Duty     | Avg (mA) |
|-----------------|--------------|----------|-----------|
| Active + WiFi   | ~240         | 10%      | 24        |
| Active no WiFi  | ~80          | 30%      | 24        |
| Light sleep     | ~2           | 60%      | 1.2       |
| **Total avg**   |              |          | **~50 mA**|

---

# H. LOOP STRUCTURE (v2)

```
SETUP:
    GPIO init (relays OFF)
    I2C init (400kHz) → MPU6050 init + calibrate (10s)
    Filter init (4× motion MA buffers)
    I2S init → INMP441 sound calibration (5s settle + 3s measure)
    DHT22 init
    WiFi connect → Blynk config
    Initialize v2 state (memset events/temporal/features, confidence=1.0)
    First mosquito API poll

LOOP (non-blocking, single-threaded):
    if (adaptive interval elapsed):
        ├── Read MPU6050 → filter → score → updateMotionFeatures()
        ├── Read INMP441 → RMS → DSP features → score
        ├── Read DHT22 (every 2s) → median → heat index → score
        ├── fusionCompute():
        │     ├── Effective scores (× confidence)
        │     ├── updateSensorConfidence()
        │     ├── detectEvents()
        │     ├── Severity computation
        │     └── EMA smoothing → DI
        ├── predictionUpdate():
        │     ├── Slope (linear regression)
        │     └── updateTemporalModel() (acceleration, instability, episodes)
        ├── stateMachineUpdate() (confidence-gated)
        ├── controlFanRelay() / controlMosquitoRelay()
        ├── systemHealthCheck()
        └── printStatusSerial() (every 5s)

    Mosquito API poll (every 30 min, independent)
    Blynk update (adaptive interval) + Blynk.run()
    WiFi reconnection check (every 60s if disconnected)
```

---

# I. FAILURE HANDLING

## I.1 Sensor Failure Cases

| Sensor   | Failure Detection                          | Fallback                                  |
|----------|-------------------------------------------|-------------------------------------------|
| MPU6050  | I2C NACK or WHO_AM_I mismatch             | Set motion_score=0, log error, retry 3x   |
| DHT22    | Read returns NaN or timeout               | Use last valid reading (max 60s stale)    |
| INMP441  | I2S returns all zeros for >2s             | Set sound_score=0, log error              |

## I.2 WiFi/API Failure Fallback

| Failure                        | Behavior                                          |
|-------------------------------|---------------------------------------------------|
| WiFi disconnect               | Continue all local processing, queue Blynk data   |
| WiFi reconnect fails 5x      | Disable WiFi for 5 min, then retry               |
| API HTTP error (4xx/5xx)      | Keep last valid mosquito index for 2 hours        |
| API timeout                   | Same as HTTP error                                |
| API stale (>2 hours)          | Set mosquito_index = 0.5 (moderate, safe default) |
| Blynk server unreachable      | Log locally, retry on next WiFi window            |

## I.3 Noisy Sensor Handling

| Strategy                                    | Applied To                    |
|--------------------------------------------|-------------------------------|
| Moving average (window=20)                  | MPU6050 axes                  |
| Median filter (window=5)                    | DHT22                         |
| RMS over 1024 samples + DC removal          | INMP441                       |
| Fast EMA (α=0.5) on RMS                    | Sound smoothing               |
| Adaptive noise floor (proportional thresh)  | Sound detection               |
| EMA on fused DI (α=0.3)                   | Final Discomfort Index        |
| Minimum dwell time (2s)                    | State machine transitions     |
| Multi-sensor validation gate               | HIGH/CRITICAL transitions     |
| **v2: Dynamic confidence**                  | All sensors (degrades on noise)|
| **v2: Rhythmic rejection**                  | Motion (autocorrelation)      |
| **v2: Multi-signal event corroboration**    | All event detectors           |

## I.4 Graceful Degradation

If N sensors fail simultaneously:
- 1 sensor failed: System continues with reduced confidence (log warning)
- 2 sensors failed: System enters DEGRADED mode → Blynk alert to parent
- 3 sensors failed: System enters SAFE_HALT → all relays OFF, continuous alert

---

# APPENDIX A: Threshold Summary

## Sensor Thresholds

| Parameter                    | Value    | Unit       | Notes |
|------------------------------|----------|------------|-------|
| Sound noise floor (default)  | 5000     | RMS units  | Adaptive at runtime |
| Sound cry delta              | 100000   | RMS units  | Delta above floor |
| Sound sustained duration     | 300      | ms         | — |
| Motion normalization         | 2.0      | g (delta)  | Full scale |
| Gyro normalization           | 30.0     | °/s        | Full scale |
| Sensor active threshold      | 0.3      | score      | For counting |
| Temp comfort zone            | 20–26    | °C         | — |
| Humidity comfort zone        | 40–60    | %RH        | — |
| Temp max deviation           | 6.0      | °C         | Full env contribution |
| Humidity max deviation       | 20.0     | %RH        | Full env contribution |

## Fusion & State Machine Thresholds

| Parameter                    | Value    | Notes |
|------------------------------|----------|-------|
| DI EMA alpha                 | 0.3      | ~3-sample settling |
| NORMAL → MILD               | DI ≥ 30  | — |
| MILD → NORMAL               | DI < 20  | Hysteresis |
| MILD → HIGH                  | DI ≥ 55  | + 2 sensors + confidence |
| HIGH → MILD                  | DI < 40  | — |
| HIGH → CRITICAL              | DI ≥ 80  | + sustained 5s |
| CRITICAL → HIGH              | DI < 60  | — |
| Confidence gate              | ≥ 0.4   | For escalation only |
| State min dwell              | 2000 ms  | — |
| CRITICAL min dwell           | 10000 ms | — |
| Slope warning                | 3.0      | DI/sample |
| Slope critical               | 5.0      | DI/sample |

## v2 Event Thresholds

| Parameter                    | Value    | Notes |
|------------------------------|----------|-------|
| Cry sound threshold          | 0.4      | Effective score |
| Cry ZCR range                | 0.05–0.35 | Moderate ZCR = voice |
| Cry sustain time             | 2000 ms  | Must persist |
| Cry motion boost             | 0.15     | If motion correlated |
| Agitation motion threshold   | 0.25     | Effective score |
| Agitation jerk threshold     | 0.08     | EMA-smoothed jerk |
| Agitation sustain time       | 3000 ms  | Must persist |
| Heat stress deviation        | 1.5°C    | Above comfort_high |
| Env stress threshold         | 0.5      | Effective score |
| Episode decay                | 0.92     | Per cycle |
| Confidence decay rate        | 0.005    | Per cycle |
| Confidence recovery rate     | 0.02     | Per cycle |

## External System Thresholds

| Parameter                    | Value    | Notes |
|------------------------------|----------|-------|
| Mosquito relay ON            | ≥ 6/10   | API value |
| Mosquito relay OFF           | < 4/10   | Hysteresis |
| API poll interval            | 30 min   | — |
| Blynk update (NORMAL)       | 30 s     | — |
| Blynk update (CRITICAL)     | 5 s      | — |
| Relay debounce               | 5000 ms  | — |

---

# APPENDIX B: Resource Impact

## RAM Usage (v2 additions)

| Component | Bytes | Notes |
|-----------|-------|-------|
| SoundFeatures | ~40 | Rolling buffers + counters |
| MotionFeatures | ~52 | Variance history + jerk |
| SensorConfidence | 12 | 3 floats |
| EventState | ~40 | Flags + intensities + timestamps |
| TemporalState | 16 | 4 floats |
| New globals | 12 | heat_index, severity, fusion_conf |
| **Total v2 overhead** | **~172 bytes** | <0.04% of ESP32-S3 SRAM |

## CPU Impact (per fusion cycle)

| Function | Time | Complexity |
|----------|------|-----------|
| computeZCR | ~30μs | O(1024) single pass |
| pushEnergyVariance | <1μs | O(6) constant |
| updateMotionFeatures | <1μs | O(8) constant |
| updateSensorConfidence | <1μs | 3 EMA ops |
| detectEvents | <2μs | ~20 comparisons |
| updateTemporalModel | <1μs | O(10) variance |
| **Total v2 addition** | **<40μs** | Existing cycle ~5ms |

## Design Constraints (enforced)

- No dynamic allocation (no `new`, `malloc`, STL containers)
- No blocking operations
- No RTOS tasks
- No FFT or ML frameworks
- Fixed-size circular buffers only
- All float arithmetic (ESP32-S3 has FPU)
- Single-threaded cooperative loop

---

# END OF SYSTEM DESIGN DOCUMENT
