// =============================================================================
// LullabyGuard.ino
// Smart Infant Monitoring System — ESP32-S3-WROOM-1
//
// Multi-sensor fusion system with:
//   - MPU6050 (motion)
//   - DHT22 (temperature + humidity)
//   - INMP441 (sound/cry detection)
//   - AccuWeather Mosquito API (external risk)
//   - Blynk IoT (dashboard + alerts)
//   - Dual relay control (fan/light + mosquito repellent)
//
// Architecture: Edge-only inference, weighted fusion, state machine
// Target: ESP32-S3 (Arduino framework)
//
// Full algorithm spec: docs/superpowers/specs/ and LullabyGuard_SystemDesign.md
// Before building: copy secrets.example.h to secrets.h and fill in your own values.
// =============================================================================

#define BLYNK_TEMPLATE_ID   "TMPL66yk5T8ys"
#define BLYNK_TEMPLATE_NAME "LullabyGuard"

#include "secrets.h"  // Credentials — copy secrets.example.h before building

#include <Wire.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>
#include <esp_sleep.h>

// =============================================================================
// CONFIGURATION — EDIT THESE FOR YOUR DEPLOYMENT
// =============================================================================
// AccuWeather API (credentials live in secrets.h; location is deployment-specific)
#define ACCUWEATHER_LOCATION    "29075"
#define MOSQUITO_INDEX_ID       "17"

// =============================================================================
// PIN DEFINITIONS — ESP32-S3
// =============================================================================

// I2C (MPU6050)
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9
#define MPU6050_ADDR        0x68

// I2S (INMP441)
#define PIN_I2S_SCK         40
#define PIN_I2S_WS          39
#define PIN_I2S_SD          41

// DHT22
#define PIN_DHT22           4
#define DHT_TYPE            DHT22

// Relays (active LOW assumed)
#define PIN_RELAY_FAN       6
#define PIN_RELAY_MOSQUITO  7

// =============================================================================
// SYSTEM CONSTANTS
// =============================================================================

// MPU6050
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_WHO_AM_I      0x75
#define ACCEL_SCALE           16384.0f
#define GYRO_SCALE            131.0f

// Sound processing
#define SOUND_SAMPLE_RATE     16000
#define SOUND_BUFFER_SIZE     1024
#define SOUND_NOISE_FLOOR     5000.0f
#define SOUND_CRY_THRESHOLD   100000.0f
#define SOUND_SUSTAINED_MS    300

// Environment comfort zone
#define TEMP_COMFORT_LOW      24.0f
#define TEMP_COMFORT_HIGH     30.0f
#define HUM_COMFORT_LOW       50.0f
#define HUM_COMFORT_HIGH      70.0f
#define TEMP_MAX_DEVIATION    8.0f
#define HUM_MAX_DEVIATION     30.0f

// Fusion weights (must sum to 1.0)
#define W_SOUND               0.45f
#define W_MOTION              0.30f
#define W_ENV                 0.25f

// EMA alpha for Discomfort Index
#define DI_EMA_ALPHA          0.3f

// State machine thresholds (with hysteresis)
#define THRESH_NORMAL_TO_MILD     30.0f
#define THRESH_MILD_TO_NORMAL     20.0f
#define THRESH_MILD_TO_HIGH       55.0f
#define THRESH_HIGH_TO_MILD       40.0f
#define THRESH_HIGH_TO_CRITICAL   80.0f
#define THRESH_CRITICAL_TO_HIGH   60.0f
#define THRESH_CRITICAL_3SENS     75.0f

// Multi-sensor validation
#define SENSOR_ACTIVE_THRESH      0.3f
#define MIN_SENSORS_HIGH          2
#define MIN_SENSORS_CRITICAL_3    3

// Timing
#define STATE_MIN_DWELL_MS        2000
#define CRITICAL_MIN_DWELL_MS     10000
#define HIGH_SUSTAIN_FOR_CRIT_MS  5000

// Prediction
#define PREDICTION_WINDOW_SIZE    10
#define SLOPE_WARNING_THRESH      3.0f
#define SLOPE_CRITICAL_THRESH     5.0f

// Mosquito subsystem
#define API_POLL_INTERVAL_MS      1800000UL  // 30 minutes
#define MOSQUITO_ON_THRESHOLD     6
#define MOSQUITO_OFF_THRESHOLD    4
#define API_STALE_MAX_MS          7200000UL  // 2 hours

// Blynk virtual pins
#define VPIN_DI_SCORE         V0
#define VPIN_STATE            V1
#define VPIN_TEMPERATURE      V2
#define VPIN_HUMIDITY         V3
#define VPIN_SOUND_LEVEL      V4
#define VPIN_MOTION_LEVEL     V5
#define VPIN_MOSQUITO_IDX     V6
#define VPIN_FAN_OVERRIDE     V7
#define VPIN_MOSQ_OVERRIDE    V8
#define VPIN_PREDICTION       V9

// Offline Blynk alert queue depth (fixed, no heap)
#define ALERT_QUEUE_SIZE     8

// Adaptive sampling intervals (ms)
#define SAMPLE_INTERVAL_NORMAL    5000
#define SAMPLE_INTERVAL_MILD      2000
#define SAMPLE_INTERVAL_HIGH      500
#define SAMPLE_INTERVAL_CRITICAL  250

// Filter sizes
#define MOTION_FILTER_SIZE    20
#define SOUND_FILTER_SIZE     5
#define DHT_MEDIAN_SIZE       5

// Relay debounce
#define RELAY_DEBOUNCE_MS     5000

// WiFi
#define WIFI_RETRY_MAX        5
#define WIFI_BACKOFF_MS       300000UL  // 5 min
#define WIFI_WINDOW_ACTIVE_MS 30000UL   // NORMAL: connected 30 s per cycle
#define WIFI_WINDOW_CYCLE_MS  300000UL  // NORMAL: 5 min duty-cycle period

// Power: light sleep between samples in NORMAL state (set 0 to disable)
#define ENABLE_LIGHT_SLEEP    1

// =============================================================================
// EVENT-DRIVEN FUSION CONSTANTS (v2 upgrade)
// =============================================================================

// Event detection thresholds
#define EVT_CRY_SOUND_THRESH      0.4f    // Min sound score to consider cry
#define EVT_CRY_ZCR_LOW           0.05f   // Cry has moderate ZCR (not clicks)
#define EVT_CRY_ZCR_HIGH          0.35f   // Cry ZCR upper bound
#define EVT_CRY_SUSTAIN_MS        2000    // Must persist 2s to confirm cry
#define EVT_CRY_MOTION_BOOST      0.15f   // Motion correlation boost for cry

#define EVT_AGITATION_MOTION_THRESH 0.25f // Min motion for agitation
#define EVT_AGITATION_JERK_THRESH  0.08f  // Jerk threshold for agitation
#define EVT_AGITATION_SUSTAIN_MS   3000   // Must persist 3s

#define EVT_HEAT_STRESS_HI        1.5f    // Heat index deviation for event
#define EVT_ENV_COMPOUND_THRESH   0.5f    // Env score for environmental stress

// Confidence parameters
#define CONF_DECAY_RATE           0.005f  // Confidence decay per cycle on stale data
#define CONF_RECOVERY_RATE        0.02f   // Confidence recovery per good sample
#define CONF_SOUND_VARIANCE_WINDOW 8      // Rolling window for sound variance
#define CONF_MOTION_VARIANCE_WINDOW 8     // Rolling window for motion variance

// Temporal modeling
#define TEMPORAL_EPISODE_WINDOW   12      // Track last 12 event snapshots
#define TEMPORAL_BURST_GAP_MS     30000   // Max gap between bursts (30s)
#define TEMPORAL_EPISODE_DECAY    0.92f   // Episode intensity decay per cycle

// Motion DSP
#define MOTION_JERK_ALPHA         0.4f    // EMA for jerk estimation
#define MOTION_RHYTHMIC_WINDOW    8       // Samples for rhythmic detection
#define MOTION_RHYTHMIC_THRESH    0.7f    // Autocorrelation for rhythmic reject

// Sound DSP (lightweight)
#define SOUND_ZCR_WEIGHT          0.15f   // ZCR contribution to cry confidence
#define SOUND_ENERGY_VAR_WINDOW   6       // Rolling energy variance window
#define SOUND_BURST_COOLDOWN_MS   5000    // Min gap between burst counts

// Fusion v2 weights (event-adjusted)
#define EVT_W_CRY                 0.50f   // Cry event drives DI strongly
#define EVT_W_AGITATION           0.30f   // Agitation event weight
#define EVT_W_ENV_STRESS          0.20f   // Environmental stress weight

// =============================================================================
// ENUMERATIONS
// =============================================================================

enum SystemState {
    STATE_NORMAL,
    STATE_MILD_DISCOMFORT,
    STATE_HIGH_DISCOMFORT,
    STATE_CRITICAL
};

enum SensorHealth {
    SENSOR_OK,
    SENSOR_DEGRADED,
    SENSOR_FAILED
};

// =============================================================================
// DATA STRUCTURES
// =============================================================================

struct MotionData {
    float delta_g;          // Gravity-subtracted magnitude deviation
    float gyro_mag_dps;     // Gyroscope vector magnitude
    float pitch_deg;
    float roll_deg;
    float score;            // Normalized [0, 1]
};

struct SoundData {
    float rms;              // Current smoothed RMS
    bool sustained;         // High energy > 500ms
    float score;            // Normalized [0, 1]
};

struct EnvironmentData {
    float temperature;      // °C
    float humidity;         // %RH
    float temp_deviation;
    float hum_deviation;
    float score;            // Normalized [0, 1]
};

struct FusionResult {
    float di_raw;           // Raw Discomfort Index (0–100)
    float di_smooth;        // EMA-smoothed DI
    uint8_t active_sensors; // Sensors with score > threshold
    float slope;            // Trend slope (DI/sample)
};

struct CalibrationData {
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;
    float sound_noise_floor;
    bool valid;
};

// Circular buffer template
struct FloatBuffer {
    float data[MOTION_FILTER_SIZE];  // Max size used
    uint8_t head;
    uint8_t count;
    uint8_t capacity;
    float sum;
};

// =============================================================================
// EVENT-DRIVEN FUSION STRUCTS (v2 upgrade)
// =============================================================================

// Lightweight DSP features extracted from INMP441 audio frame
struct SoundFeatures {
    float zcr;                  // Zero-crossing rate [0..1]
    float energy_variance;      // Short-term energy stability
    float envelope;             // Smoothed envelope level
    uint8_t burst_count;        // Sound bursts in recent window
    unsigned long last_burst;   // Timestamp of last burst
    float energy_hist[SOUND_ENERGY_VAR_WINDOW]; // Rolling energy buffer
    uint8_t energy_hist_idx;
    uint8_t energy_hist_count;
};

// Motion DSP features
struct MotionFeatures {
    float jerk;                 // Rate of accel change (smoothed)
    float variance;             // Rolling motion variance
    float prev_delta_g;         // Previous delta_g for jerk calc
    bool rhythmic;              // Rhythmic motion detected (fan/rocker)
    float var_hist[MOTION_RHYTHMIC_WINDOW]; // Rolling delta_g history
    uint8_t var_hist_idx;
    uint8_t var_hist_count;
};

// Per-sensor confidence [0..1]
struct SensorConfidence {
    float sound;                // Sound sensor confidence
    float motion;               // Motion sensor confidence
    float env;                  // Environment sensor confidence
};

// Event inference results
struct EventState {
    // Binary event flags
    bool cry_event;             // Confirmed infant cry
    bool agitation_event;       // Sustained agitation
    bool heat_stress_event;     // Thermal discomfort
    bool env_stress_event;      // Combined environmental stress
    bool rapid_escalation;      // Fast-rising DI trend

    // Event intensities [0..1]
    float cry_intensity;        // Cry strength (confidence-weighted)
    float agitation_intensity;  // Agitation strength
    float env_stress_intensity; // Environmental stress level

    // Temporal tracking
    unsigned long cry_onset;    // When current cry started
    unsigned long agit_onset;   // When current agitation started
    uint8_t cry_episodes;       // Cry episodes in recent window
    unsigned long last_cry_end; // When last cry episode ended
};

// Temporal behavior model
struct TemporalState {
    float episode_intensity;    // Decaying accumulator of recent events
    float di_acceleration;      // Second derivative of DI trend
    float prev_slope;           // Previous slope for acceleration calc
    float instability;          // Variance of recent DI values
};

// =============================================================================
// GLOBAL STATE
// =============================================================================

// Sensor data
MotionData      g_motion;
SoundData       g_sound;
EnvironmentData g_env;
FusionResult    g_fusion;
CalibrationData g_calib;

// State machine
SystemState     g_state          = STATE_NORMAL;
SystemState     g_prev_state     = STATE_NORMAL;
unsigned long   g_state_entered  = 0;
unsigned long   g_high_entered   = 0;

// Adaptive sampling
uint32_t        g_sample_interval = SAMPLE_INTERVAL_NORMAL;

// Timing
unsigned long   g_last_sensor_read  = 0;
unsigned long   g_last_blynk_update = 0;
unsigned long   g_last_api_poll     = 0;
unsigned long   g_last_wifi_attempt = 0;

// Blynk update interval (state-dependent)
uint32_t        g_blynk_interval    = 30000;

// Prediction buffer
float           g_di_history[PREDICTION_WINDOW_SIZE];
uint8_t         g_di_history_idx    = 0;
uint8_t         g_di_history_count  = 0;

// Mosquito subsystem
float           g_mosquito_index    = 0.5f;  // Default moderate
unsigned long   g_mosquito_last_valid = 0;
bool            g_mosquito_relay_on = false;

// Relay state
bool            g_fan_relay_on      = false;
unsigned long   g_fan_last_toggle   = 0;
unsigned long   g_mosq_last_toggle  = 0;
bool            g_safe_halt         = false;  // Latched: all relays forced OFF

// Manual overrides from Blynk
int             g_fan_override      = -1;  // -1=auto, 0=off, 1=on
int             g_mosq_override     = -1;

// State change notification flag
bool            g_state_changed    = false;

// Offline Blynk alert queue — alerts raised while disconnected are replayed
struct PendingAlert {
    char code[24];
    char msg[96];
    bool used;
};
PendingAlert    g_alert_queue[ALERT_QUEUE_SIZE];
uint8_t         g_alert_head = 0;
void            flushAlertQueue();  // used by the WiFi reconnect path

// Sensor health
SensorHealth    g_health_mpu       = SENSOR_OK;
SensorHealth    g_health_dht       = SENSOR_OK;
SensorHealth    g_health_sound     = SENSOR_OK;
uint8_t         g_mpu_fail_count   = 0;
uint8_t         g_dht_fail_count   = 0;
uint8_t         g_sound_fail_count = 0;

// WiFi state
bool            g_wifi_connected   = false;
uint8_t         g_wifi_retries     = 0;
bool            g_wifi_window_on   = true;   // NORMAL-state duty-cycle window
unsigned long   g_wifi_window_started = 0;

// Sound sustained detection
unsigned long   g_sound_high_start = 0;
bool            g_sound_is_high    = false;

// --- Event-driven fusion state (v2 upgrade) ---
SoundFeatures   g_snd_feat;
MotionFeatures  g_mot_feat;
SensorConfidence g_confidence;
EventState      g_events;
TemporalState   g_temporal;
float           g_heat_index = 25.0f;       // Computed heat index (°C)
float           g_severity   = 0.0f;        // Separated severity [0..100]
float           g_fusion_confidence = 1.0f;  // Overall fusion confidence [0..1]

// Filter buffers
FloatBuffer     g_filt_ax, g_filt_ay, g_filt_az, g_filt_mag;

// DHT median filter
float           g_temp_buf[DHT_MEDIAN_SIZE];
float           g_hum_buf[DHT_MEDIAN_SIZE];
uint8_t         g_dht_buf_idx = 0;
uint8_t         g_dht_buf_count = 0;

// DHT object
DHT             dht(PIN_DHT22, DHT_TYPE);

// Last valid DHT reading
float           g_last_valid_temp = 25.0f;
float           g_last_valid_hum  = 50.0f;
unsigned long   g_last_valid_dht_time = 0;

// I2S config
i2s_config_t    g_i2s_config;
i2s_pin_config_t g_i2s_pins;
bool            g_i2s_installed = false;

// =============================================================================
// FILTER UTILITIES
// =============================================================================

void filterInit(FloatBuffer& f, uint8_t capacity) {
    memset(f.data, 0, sizeof(f.data));
    f.head = 0;
    f.count = 0;
    f.capacity = capacity;
    f.sum = 0.0f;
}

float filterPush(FloatBuffer& f, float val) {
    if (f.count == f.capacity) {
        f.sum -= f.data[f.head];
    } else {
        f.count++;
    }
    f.data[f.head] = val;
    f.sum += val;
    f.head = (f.head + 1) % f.capacity;
    return f.sum / f.count;
}

float emaSmooth(float prev, float newVal, float alpha) {
    return alpha * newVal + (1.0f - alpha) * prev;
}

float clampf(float val, float minv, float maxv) {
    if (val < minv) return minv;
    if (val > maxv) return maxv;
    return val;
}

// Median of small array (insertion sort approach for N=5)
float medianFloat(float* arr, uint8_t n) {
    float sorted[DHT_MEDIAN_SIZE];
    memcpy(sorted, arr, n * sizeof(float));
    // Simple insertion sort (n<=5, negligible cost)
    for (uint8_t i = 1; i < n; i++) {
        float key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[n / 2];
}

// =============================================================================
// DSP HELPERS (lightweight, no FFT)
// =============================================================================

// Zero-crossing rate from raw I2S samples (AC-coupled)
float computeZCR(int32_t* samples, int n, double dc_mean) {
    if (n < 2) return 0.0f;
    uint16_t crossings = 0;
    float prev = (float)((samples[0] >> 8) - (int32_t)dc_mean);
    for (int i = 1; i < n; i++) {
        float curr = (float)((samples[i] >> 8) - (int32_t)dc_mean);
        if ((prev >= 0.0f && curr < 0.0f) || (prev < 0.0f && curr >= 0.0f)) {
            crossings++;
        }
        prev = curr;
    }
    return (float)crossings / (float)(n - 1);
}

// Push energy into rolling variance buffer, return variance
float pushEnergyVariance(SoundFeatures& f, float energy) {
    f.energy_hist[f.energy_hist_idx] = energy;
    f.energy_hist_idx = (f.energy_hist_idx + 1) % SOUND_ENERGY_VAR_WINDOW;
    if (f.energy_hist_count < SOUND_ENERGY_VAR_WINDOW) f.energy_hist_count++;

    // Compute variance
    uint8_t n = f.energy_hist_count;
    float mean = 0.0f;
    for (uint8_t i = 0; i < n; i++) mean += f.energy_hist[i];
    mean /= n;
    float var = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float d = f.energy_hist[i] - mean;
        var += d * d;
    }
    return var / n;
}

// Heat index (NWS algorithm: C→F, Rothfusz regression, F→C)
float computeHeatIndex(float temp_c, float hum_pct) {
    // Only meaningful above 27°C (~80°F)
    if (temp_c < 27.0f) return temp_c;
    float tf = temp_c * 1.8f + 32.0f;  // Convert to Fahrenheit
    float rh = hum_pct;

    // Steadman simple formula first
    float hi_f = 0.5f * (tf + 61.0f + (tf - 68.0f) * 1.2f + rh * 0.094f);

    // If above 80°F, use full Rothfusz regression
    if (hi_f > 80.0f) {
        hi_f = -42.379f + 2.04901523f * tf + 10.14333127f * rh
               - 0.22475541f * tf * rh - 0.00683783f * tf * tf
               - 0.05481717f * rh * rh + 0.00122874f * tf * tf * rh
               + 0.00085282f * tf * rh * rh - 0.00000199f * tf * tf * rh * rh;

        // Low humidity adjustment (RH < 13% and T 80-112°F)
        if (rh < 13.0f && tf > 80.0f && tf < 112.0f) {
            hi_f -= ((13.0f - rh) / 4.0f) * sqrtf((17.0f - fabsf(tf - 95.0f)) / 17.0f);
        }
        // High humidity adjustment (RH > 85% and T 80-87°F)
        if (rh > 85.0f && tf > 80.0f && tf < 87.0f) {
            hi_f += ((rh - 85.0f) / 10.0f) * ((87.0f - tf) / 5.0f);
        }
    }

    return (hi_f - 32.0f) * 0.5556f;  // Convert back to Celsius
}

// =============================================================================
// MOTION FEATURE EXTRACTION (v2 upgrade)
// =============================================================================

void updateMotionFeatures(float delta_g_filt, float raw_delta_g, float gyro_mag) {
    // --- Jerk estimation (from RAW delta_g to preserve transients) ---
    float raw_jerk = fabsf(raw_delta_g - g_mot_feat.prev_delta_g);
    g_mot_feat.jerk = emaSmooth(g_mot_feat.jerk, raw_jerk, MOTION_JERK_ALPHA);
    g_mot_feat.prev_delta_g = raw_delta_g;

    // --- Rolling variance of filtered delta_g ---
    g_mot_feat.var_hist[g_mot_feat.var_hist_idx] = delta_g_filt;
    g_mot_feat.var_hist_idx = (g_mot_feat.var_hist_idx + 1) % MOTION_RHYTHMIC_WINDOW;
    if (g_mot_feat.var_hist_count < MOTION_RHYTHMIC_WINDOW) g_mot_feat.var_hist_count++;

    uint8_t n = g_mot_feat.var_hist_count;
    float mean = 0.0f;
    for (uint8_t i = 0; i < n; i++) mean += g_mot_feat.var_hist[i];
    mean /= n;
    float var = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float d = g_mot_feat.var_hist[i] - mean;
        var += d * d;
    }
    g_mot_feat.variance = var / n;

    // --- Rhythmic motion detection (lag-1 autocorrelation) ---
    if (n >= 4) {
        float autocorr = 0.0f, norm = 0.0f;
        for (uint8_t i = 0; i < n - 1; i++) {
            float a = g_mot_feat.var_hist[i] - mean;
            float b = g_mot_feat.var_hist[(i + 1) % MOTION_RHYTHMIC_WINDOW] - mean;
            autocorr += a * b;
            norm += a * a;
        }
        float r = (norm > 0.001f) ? (autocorr / norm) : 0.0f;
        // High positive autocorrelation = rhythmic (fan vibration, rocker)
        g_mot_feat.rhythmic = (r > MOTION_RHYTHMIC_THRESH && g_mot_feat.variance < 0.01f);
    } else {
        g_mot_feat.rhythmic = false;
    }
}

// =============================================================================
// SENSOR CONFIDENCE ESTIMATION 
// =============================================================================

void updateSensorConfidence() {
    // --- Sound confidence ---
    // Low confidence if: sound sensor failed, or constant low-variance noise
    if (g_health_sound == SENSOR_FAILED) {
        g_confidence.sound = 0.0f;
    } else if (g_health_sound == SENSOR_DEGRADED) {
        g_confidence.sound = fmaxf(g_confidence.sound - CONF_DECAY_RATE * 2.0f, 0.2f);
    } else {
        // Reduce confidence if sound energy variance is very low (constant noise)
        float ev = g_snd_feat.energy_variance;
        float norm_var = clampf(ev / 1e10f, 0.0f, 1.0f);  // Normalize
        float target = 0.5f + 0.5f * norm_var;  // 0.5 if flat, 1.0 if varying
        g_confidence.sound = emaSmooth(g_confidence.sound, target, CONF_RECOVERY_RATE);
    }

    // --- Motion confidence ---
    if (g_health_mpu == SENSOR_FAILED) {
        g_confidence.motion = 0.0f;
    } else if (g_mot_feat.rhythmic) {
        // Rhythmic = likely fan/machine vibration, reduce confidence
        g_confidence.motion = fmaxf(g_confidence.motion - CONF_DECAY_RATE * 3.0f, 0.1f);
    } else {
        float target = (g_health_mpu == SENSOR_OK) ? 1.0f : 0.5f;
        g_confidence.motion = emaSmooth(g_confidence.motion, target, CONF_RECOVERY_RATE);
    }

    // --- Environment confidence ---
    if (g_health_dht == SENSOR_FAILED) {
        g_confidence.env = 0.0f;
    } else if (g_health_dht == SENSOR_DEGRADED) {
        g_confidence.env = fmaxf(g_confidence.env - CONF_DECAY_RATE, 0.3f);
    } else {
        g_confidence.env = emaSmooth(g_confidence.env, 1.0f, CONF_RECOVERY_RATE);
    }

    // --- Overall fusion confidence ---
    float total_weight = W_SOUND + W_MOTION + W_ENV;
    g_fusion_confidence = (g_confidence.sound * W_SOUND +
                           g_confidence.motion * W_MOTION +
                           g_confidence.env * W_ENV) / total_weight;
}

// =============================================================================
// EVENT DETECTORS (v2 upgrade)
// =============================================================================

void detectEvents() {
    unsigned long now = millis();
    float snd_eff = g_sound.score * g_confidence.sound;
    float mot_eff = g_motion.score * g_confidence.motion;
    float env_eff = g_env.score * g_confidence.env;

    // ─── CRY EVENT ───
    // Requires: elevated sound + sustained + ZCR in cry range
    bool cry_candidate = (snd_eff > EVT_CRY_SOUND_THRESH) &&
                         g_sound.sustained &&
                         (g_snd_feat.zcr > EVT_CRY_ZCR_LOW) &&
                         (g_snd_feat.zcr < EVT_CRY_ZCR_HIGH);

    if (cry_candidate) {
        if (g_events.cry_onset == 0) g_events.cry_onset = now;
        unsigned long cry_dur = now - g_events.cry_onset;

        if (cry_dur >= EVT_CRY_SUSTAIN_MS) {
            g_events.cry_event = true;
            // Intensity: base sound + motion correlation boost
            float motion_boost = (mot_eff > 0.15f) ? EVT_CRY_MOTION_BOOST : 0.0f;
            g_events.cry_intensity = clampf(snd_eff + motion_boost, 0.0f, 1.0f);
        }
    } else {
        if (g_events.cry_event) {
            // Cry ended — record episode
            g_events.last_cry_end = now;
            if (g_events.cry_episodes < 255) g_events.cry_episodes++;
        }
        g_events.cry_event = false;
        g_events.cry_onset = 0;
        // Decay cry intensity
        g_events.cry_intensity *= 0.8f;
    }

    // Decay cry episode counter over time
    if (now - g_events.last_cry_end > 120000UL && g_events.cry_episodes > 0) {
        g_events.cry_episodes = 0;  // Reset after 2 min quiet
    }

    // ─── AGITATION EVENT ───
    // Requires: motion + jerk (not rhythmic) + sustained
    bool agit_candidate = (mot_eff > EVT_AGITATION_MOTION_THRESH) &&
                          (g_mot_feat.jerk > EVT_AGITATION_JERK_THRESH) &&
                          !g_mot_feat.rhythmic;

    if (agit_candidate) {
        if (g_events.agit_onset == 0) g_events.agit_onset = now;
        unsigned long agit_dur = now - g_events.agit_onset;

        if (agit_dur >= EVT_AGITATION_SUSTAIN_MS) {
            g_events.agitation_event = true;
            g_events.agitation_intensity = clampf(mot_eff * (1.0f + g_mot_feat.jerk), 0.0f, 1.0f);
        }
    } else {
        g_events.agitation_event = false;
        g_events.agit_onset = 0;
        g_events.agitation_intensity *= 0.85f;
    }

    // ─── HEAT STRESS EVENT ───
    float hi_deviation = g_heat_index - TEMP_COMFORT_HIGH;
    g_events.heat_stress_event = (hi_deviation > EVT_HEAT_STRESS_HI);

    // ─── ENVIRONMENTAL STRESS EVENT ───
    g_events.env_stress_event = (env_eff > EVT_ENV_COMPOUND_THRESH) ||
                                 g_events.heat_stress_event;
    g_events.env_stress_intensity = clampf(env_eff + (g_events.heat_stress_event ? 0.2f : 0.0f),
                                           0.0f, 1.0f);

    // ─── RAPID ESCALATION ───
    g_events.rapid_escalation = (g_temporal.di_acceleration > 1.0f) &&
                                 (g_fusion.slope > SLOPE_WARNING_THRESH);
}

// =============================================================================
// TEMPORAL BEHAVIOR MODEL (v2 upgrade)
// =============================================================================

void updateTemporalModel() {
    // --- Episode intensity accumulator ---
    // Grows when events are active, decays otherwise
    float event_input = 0.0f;
    if (g_events.cry_event)       event_input += g_events.cry_intensity * 0.5f;
    if (g_events.agitation_event) event_input += g_events.agitation_intensity * 0.3f;
    if (g_events.env_stress_event) event_input += g_events.env_stress_intensity * 0.2f;

    g_temporal.episode_intensity = g_temporal.episode_intensity * TEMPORAL_EPISODE_DECAY + event_input;
    g_temporal.episode_intensity = clampf(g_temporal.episode_intensity, 0.0f, 5.0f);

    // --- DI acceleration (second derivative) ---
    g_temporal.di_acceleration = g_fusion.slope - g_temporal.prev_slope;
    g_temporal.prev_slope = g_fusion.slope;

    // --- DI instability (variance of recent DI history) ---
    if (g_di_history_count >= 3) {
        uint8_t n = g_di_history_count;
        float mean = 0.0f;
        for (uint8_t i = 0; i < n; i++) mean += g_di_history[i];
        mean /= n;
        float var = 0.0f;
        for (uint8_t i = 0; i < n; i++) {
            float d = g_di_history[i] - mean;
            var += d * d;
        }
        g_temporal.instability = var / n;
    }
}

// =============================================================================
// MPU6050 DRIVER
// =============================================================================

bool mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission(true) == 0);
}

bool mpuReadRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU6050_ADDR, len, (uint8_t)true) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool mpuInit() {
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        Serial.printf("[MPU] Init attempt %u/5\n", attempt + 1);

        uint8_t whoAmI = 0;
        if (!mpuReadRegs(MPU_REG_WHO_AM_I, &whoAmI, 1)) {
            delay(200);
            continue;
        }
        if (whoAmI != 0x68 && whoAmI != 0x70 && whoAmI != 0x72) {
            delay(200);
            continue;
        }

        if (!mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00)) continue;
        delay(100);
        if (!mpuWriteReg(MPU_REG_SMPLRT_DIV, 9)) continue;   // 100 Hz
        if (!mpuWriteReg(MPU_REG_CONFIG, 0x03)) continue;     // DLPF 44Hz
        if (!mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00)) continue; // ±250°/s
        if (!mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00)) continue; // ±2g

        Serial.println("[MPU] Initialized OK");
        return true;
    }
    Serial.println("[MPU] FATAL: Init failed");
    return false;
}

bool mpuCalibrate() {
    Serial.println("[CAL] MPU6050 calibration — hold still 10s");
    double sumAx = 0, sumAy = 0, sumAz = 0;
    double sumGx = 0, sumGy = 0, sumGz = 0;
    uint32_t valid = 0;

    for (uint16_t i = 0; i < 500; i++) {
        uint8_t buf[14];
        if (mpuReadRegs(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
            int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
            int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
            int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
            sumAx += ax; sumAy += ay; sumAz += az;
            sumGx += gx; sumGy += gy; sumGz += gz;
            valid++;
        }
        delay(20);
    }

    if (valid < 250) {
        Serial.println("[CAL] MPU calibration failed — insufficient samples");
        g_calib.valid = false;
        return false;
    }

    g_calib.ax_offset = (sumAx / valid) / ACCEL_SCALE;
    g_calib.ay_offset = (sumAy / valid) / ACCEL_SCALE;
    g_calib.az_offset = ((sumAz / valid) - ACCEL_SCALE) / ACCEL_SCALE;
    g_calib.gx_offset = (sumGx / valid) / GYRO_SCALE;
    g_calib.gy_offset = (sumGy / valid) / GYRO_SCALE;
    g_calib.gz_offset = (sumGz / valid) / GYRO_SCALE;
    g_calib.valid = true;

    Serial.printf("[CAL] Offsets: ax=%.4f ay=%.4f az=%.4f\n",
                  g_calib.ax_offset, g_calib.ay_offset, g_calib.az_offset);
    return true;
}

bool mpuReadProcessed() {
    uint8_t buf[14];
    if (!mpuReadRegs(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
        g_mpu_fail_count++;
        if (g_mpu_fail_count >= 10) g_health_mpu = SENSOR_FAILED;
        else if (g_mpu_fail_count >= 3) g_health_mpu = SENSOR_DEGRADED;
        return false;
    }
    g_mpu_fail_count = 0;
    g_health_mpu = SENSOR_OK;

    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    float ax = (raw_ax / ACCEL_SCALE) - g_calib.ax_offset;
    float ay = (raw_ay / ACCEL_SCALE) - g_calib.ay_offset;
    float az = (raw_az / ACCEL_SCALE) - g_calib.az_offset;
    float gx = (raw_gx / GYRO_SCALE) - g_calib.gx_offset;
    float gy = (raw_gy / GYRO_SCALE) - g_calib.gy_offset;
    float gz = (raw_gz / GYRO_SCALE) - g_calib.gz_offset;

    float fax = filterPush(g_filt_ax, ax);
    float fay = filterPush(g_filt_ay, ay);
    float faz = filterPush(g_filt_az, az);

    float mag = sqrtf(fax * fax + fay * fay + (faz + 1.0f) * (faz + 1.0f));
    float filt_mag = filterPush(g_filt_mag, mag);

    g_motion.delta_g = fabsf(filt_mag - 1.0f);
    g_motion.gyro_mag_dps = sqrtf(gx * gx + gy * gy + gz * gz);
    g_motion.pitch_deg = atan2f(fax, sqrtf(fay * fay + (faz + 1.0f) * (faz + 1.0f))) * 57.2958f;
    g_motion.roll_deg = atan2f(fay, sqrtf(fax * fax + (faz + 1.0f) * (faz + 1.0f))) * 57.2958f;

    // Normalize motion score with gyro gating (rejects fan vibration)
    float accel_score = clampf(g_motion.delta_g / 2.0f, 0.0f, 1.0f);
    float gyro_score = clampf(g_motion.gyro_mag_dps / 30.0f, 0.0f, 1.0f);
    g_motion.score = accel_score * gyro_score;

    // v2: Extract motion DSP features (jerk from raw, variance/rhythmic from filtered)
    float raw_mag = sqrtf(ax * ax + ay * ay + (az + 1.0f) * (az + 1.0f));
    float raw_delta_g = fabsf(raw_mag - 1.0f);
    updateMotionFeatures(g_motion.delta_g, raw_delta_g, g_motion.gyro_mag_dps);

    return true;
}

// =============================================================================
// INMP441 SOUND DRIVER
// =============================================================================

bool soundInit() {
    g_i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    g_i2s_config.sample_rate = SOUND_SAMPLE_RATE;
    g_i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    g_i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    g_i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    g_i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    g_i2s_config.dma_buf_count = 4;
    g_i2s_config.dma_buf_len = SOUND_BUFFER_SIZE;
    g_i2s_config.use_apll = false;
    g_i2s_config.tx_desc_auto_clear = false;
    g_i2s_config.fixed_mclk = 0;

    g_i2s_pins.bck_io_num = PIN_I2S_SCK;
    g_i2s_pins.ws_io_num = PIN_I2S_WS;
    g_i2s_pins.data_out_num = I2S_PIN_NO_CHANGE;
    g_i2s_pins.data_in_num = PIN_I2S_SD;

    if (i2s_driver_install(I2S_NUM_0, &g_i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("[SND] I2S driver install failed");
        return false;
    }
    if (i2s_set_pin(I2S_NUM_0, &g_i2s_pins) != ESP_OK) {
        Serial.println("[SND] I2S pin config failed");
        i2s_driver_uninstall(I2S_NUM_0);
        return false;
    }
    g_i2s_installed = true;
    delay(200);

    // Flush initial DMA buffers (first reads are often garbage)
    int32_t flush_buf[SOUND_BUFFER_SIZE];
    size_t flush_bytes;
    for (int i = 0; i < 8; i++) {
        i2s_read(I2S_NUM_0, (char*)flush_buf, sizeof(flush_buf), &flush_bytes, 100);
    }

    Serial.println("[SND] INMP441 initialized OK");
    return true;
}

void soundCalibrateNoiseFloor() {
    int32_t samples[SOUND_BUFFER_SIZE];
    size_t bytesRead;

    // Phase 1: Flush I2S for 5 seconds (INMP441 settling transients)
    Serial.print("[CAL] Sound sensor settling ");
    unsigned long settle_start = millis();
    int flush_frames = 0;
    while (millis() - settle_start < 5000) {
        if (i2s_read(I2S_NUM_0, (char*)samples, sizeof(int32_t) * SOUND_BUFFER_SIZE,
                     &bytesRead, 100) == ESP_OK) {
            flush_frames++;
        }
        if (flush_frames % 20 == 0) Serial.print(".");
    }
    Serial.printf(" done (%d frames flushed)\n", flush_frames);

    // Phase 2: Calibrate using 3 seconds of post-settle audio
    Serial.println("[CAL] Measuring noise floor (3s silence)...");
    double total_rms = 0;
    int frames = 0;
    float last_rms = 0;

    unsigned long start = millis();
    while (millis() - start < 3000) {
        if (i2s_read(I2S_NUM_0, (char*)samples, sizeof(int32_t) * SOUND_BUFFER_SIZE,
                     &bytesRead, 100) == ESP_OK) {
            int numSamples = bytesRead / sizeof(int32_t);
            // Compute DC mean for removal
            double dc_sum = 0;
            for (int i = 0; i < numSamples; i++) {
                dc_sum += (double)(samples[i] >> 8);
            }
            double dc_mean = dc_sum / numSamples;
            // Compute AC-coupled RMS
            double sum_sq = 0;
            for (int i = 0; i < numSamples; i++) {
                double s = (double)(samples[i] >> 8) - dc_mean;
                sum_sq += s * s;
            }
            last_rms = sqrtf(sum_sq / numSamples);
            total_rms += last_rms;
            frames++;
        }
    }

    if (frames > 0) {
        float avg_rms = (float)(total_rms / frames);
        // Use the lower of average and last frame (I2S may still be settling)
        float base_rms = fminf(avg_rms, last_rms);
        g_calib.sound_noise_floor = base_rms * 1.5f; // 50% margin
        Serial.printf("[CAL] Noise floor: %.0f (avg: %.0f, last: %.0f)\n",
                      g_calib.sound_noise_floor, avg_rms, last_rms);
    } else {
        g_calib.sound_noise_floor = SOUND_NOISE_FLOOR;
        Serial.println("[CAL] Sound calibration failed — using default");
    }
}

bool soundReadRMS() {
    if (!g_i2s_installed) {
        g_health_sound = SENSOR_FAILED;
        g_sound.score = 0.0f;
        return false;
    }

    int32_t samples[SOUND_BUFFER_SIZE];
    size_t bytesRead;

    if (i2s_read(I2S_NUM_0, (char*)samples, sizeof(int32_t) * SOUND_BUFFER_SIZE,
                 &bytesRead, 100) != ESP_OK) {
        g_sound_fail_count++;
        if (g_sound_fail_count >= 10) g_health_sound = SENSOR_FAILED;
        return false;
    }

    int numSamples = bytesRead / sizeof(int32_t);
    if (numSamples == 0) {
        g_sound_fail_count++;
        return false;
    }

    g_sound_fail_count = 0;
    g_health_sound = SENSOR_OK;

    // Compute DC mean for removal
    double dc_sum = 0;
    int zero_count = 0;
    for (int i = 0; i < numSamples; i++) {
        int32_t raw = samples[i] >> 8;
        if (raw == 0) zero_count++;
        dc_sum += (double)raw;
    }
    double dc_mean = dc_sum / numSamples;

    // Compute AC-coupled RMS (removes DC offset from INMP441)
    double sum_sq = 0;
    for (int i = 0; i < numSamples; i++) {
        double s = (double)(samples[i] >> 8) - dc_mean;
        sum_sq += s * s;
    }

    // If all zeros — likely hardware issue
    if (zero_count == numSamples) {
        g_sound_fail_count += 5;
        if (g_sound_fail_count >= 10) g_health_sound = SENSOR_FAILED;
        g_sound.score = 0.0f;
        return false;
    }

    float rms_raw = sqrtf(sum_sq / numSamples);

    // v2: Extract sound DSP features
    g_snd_feat.zcr = computeZCR(samples, numSamples, dc_mean);
    g_snd_feat.energy_variance = pushEnergyVariance(g_snd_feat, rms_raw);
    g_snd_feat.envelope = emaSmooth(g_snd_feat.envelope, rms_raw, 0.3f);
    // Burst detection: rapid energy spike above 2x envelope
    if (rms_raw > g_snd_feat.envelope * 2.0f && rms_raw > g_calib.sound_noise_floor) {
        unsigned long now = millis();
        if (now - g_snd_feat.last_burst > SOUND_BURST_COOLDOWN_MS) {
            if (g_snd_feat.burst_count < 255) g_snd_feat.burst_count++;
            g_snd_feat.last_burst = now;
        }
    }
    // Decay burst count over time
    if (millis() - g_snd_feat.last_burst > 60000UL) g_snd_feat.burst_count = 0;

    // Fast EMA for responsive scoring (alpha=0.5 → ~3 samples to converge)
    static float rms_fast = 0.0f;
    rms_fast = (rms_fast < 1.0f) ? rms_raw : emaSmooth(rms_fast, rms_raw, 0.5f);
    g_sound.rms = rms_fast;

    // Proportional threshold: scales with noise floor level
    float effective_floor = g_calib.sound_noise_floor;
    float cry_delta = fmaxf(SOUND_CRY_THRESHOLD, effective_floor * 0.3f);
    float high_thresh = effective_floor + cry_delta * 0.3f;

    // Adaptive noise floor: track ambient during quiet periods
    if (rms_fast < high_thresh) {
        float target_floor = rms_fast * 1.2f;
        // Faster adaptation during first 2 minutes (I2S still settling)
        float adapt_alpha = (millis() < 120000) ? 0.3f : 0.1f;
        g_calib.sound_noise_floor = emaSmooth(effective_floor, target_floor, adapt_alpha);
        // Clamp: floor should never exceed 3x current RMS during quiet
        float max_floor = rms_fast * 3.0f;
        if (g_calib.sound_noise_floor > max_floor) {
            g_calib.sound_noise_floor = max_floor;
        }
    }

    // Sustained detection
    if (rms_fast > high_thresh) {
        if (!g_sound_is_high) {
            g_sound_is_high = true;
            g_sound_high_start = millis();
        }
        g_sound.sustained = (millis() - g_sound_high_start >= SOUND_SUSTAINED_MS);
    } else {
        g_sound_is_high = false;
        g_sound.sustained = false;
    }

    // Normalize score using proportional cry delta
    g_sound.score = clampf((rms_fast - effective_floor) / cry_delta, 0.0f, 1.0f);

    return true;
}

// =============================================================================
// DHT22 DRIVER
// =============================================================================

void dhtInit() {
    dht.begin();
    delay(2000);  // DHT22 startup time
    Serial.println("[DHT] Initialized");
}

bool dhtReadFiltered() {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    // Validate
    if (isnan(t) || isnan(h) || t < -10.0f || t > 60.0f || h < 0.0f || h > 100.0f) {
        g_dht_fail_count++;
        if (g_dht_fail_count >= 10) g_health_dht = SENSOR_FAILED;
        else if (g_dht_fail_count >= 3) g_health_dht = SENSOR_DEGRADED;

        // Use last valid if within 60s
        if (millis() - g_last_valid_dht_time < 60000) {
            t = g_last_valid_temp;
            h = g_last_valid_hum;
        } else {
            g_env.score = 0.0f;
            return false;
        }
    } else {
        g_dht_fail_count = 0;
        g_health_dht = SENSOR_OK;
        g_last_valid_temp = t;
        g_last_valid_hum = h;
        g_last_valid_dht_time = millis();
    }

    // Median filter
    g_temp_buf[g_dht_buf_idx] = t;
    g_hum_buf[g_dht_buf_idx] = h;
    g_dht_buf_idx = (g_dht_buf_idx + 1) % DHT_MEDIAN_SIZE;
    if (g_dht_buf_count < DHT_MEDIAN_SIZE) g_dht_buf_count++;

    float med_temp = medianFloat(g_temp_buf, g_dht_buf_count);
    float med_hum = medianFloat(g_hum_buf, g_dht_buf_count);

    g_env.temperature = med_temp;
    g_env.humidity = med_hum;

    // Compute deviation from comfort zone
    g_env.temp_deviation = 0.0f;
    if (med_temp < TEMP_COMFORT_LOW) g_env.temp_deviation = TEMP_COMFORT_LOW - med_temp;
    else if (med_temp > TEMP_COMFORT_HIGH) g_env.temp_deviation = med_temp - TEMP_COMFORT_HIGH;

    g_env.hum_deviation = 0.0f;
    if (med_hum < HUM_COMFORT_LOW) g_env.hum_deviation = HUM_COMFORT_LOW - med_hum;
    else if (med_hum > HUM_COMFORT_HIGH) g_env.hum_deviation = med_hum - HUM_COMFORT_HIGH;

    // v2: Compute heat index for thermal stress modeling
    g_heat_index = computeHeatIndex(med_temp, med_hum);
    float hi_deviation = 0.0f;
    if (g_heat_index > TEMP_COMFORT_HIGH) hi_deviation = g_heat_index - TEMP_COMFORT_HIGH;

    // Normalize combined (Euclidean norm, sqrt-compressed)
    // Heat index is NOT blended into base env_score — it is used only for
    // heat_stress_event detection to avoid double-counting humidity (already
    // captured in hum_contrib). Raw temp_deviation alone drives thermal scoring.
    float temp_contrib = clampf(g_env.temp_deviation / TEMP_MAX_DEVIATION, 0.0f, 1.0f);
    float hum_contrib  = clampf(g_env.hum_deviation  / HUM_MAX_DEVIATION,  0.0f, 1.0f);
    g_env.score = clampf(
        sqrtf((temp_contrib * temp_contrib + hum_contrib * hum_contrib) / 2.0f),
        0.0f, 1.0f
    );

    return true;
}

// =============================================================================
// FUSION ENGINE
// =============================================================================

void fusionCompute() {
    // --- Step 1: Confidence-weighted base scores ---
    float snd_eff = g_sound.score * g_confidence.sound;
    float mot_eff = g_motion.score * g_confidence.motion;
    float env_eff = g_env.score * g_confidence.env;

    // --- Step 2: Update confidence estimates ---
    updateSensorConfidence();

    // --- Step 3: Detect behavioral events ---
    detectEvents();

    // --- Step 4: Event-driven severity computation ---
    // Event severity: weighted sum of confirmed events
    float event_severity = (EVT_W_CRY       * g_events.cry_intensity +
                            EVT_W_AGITATION  * g_events.agitation_intensity +
                            EVT_W_ENV_STRESS * g_events.env_stress_intensity) * 100.0f;

    // Temporal escalation bonus: repeated episodes and accumulating intensity
    float temporal_boost = clampf(g_temporal.episode_intensity * 5.0f, 0.0f, 20.0f);
    // Repeated cry episodes amplify severity
    float episode_boost = clampf((float)g_events.cry_episodes * 3.0f, 0.0f, 15.0f);

    // --- Step 5: Blend event severity with legacy weighted score ---
    // Legacy base provides floor; events provide intelligent elevation
    float base_di = (W_SOUND * snd_eff + W_MOTION * mot_eff + W_ENV * env_eff) * 100.0f;
    g_severity = fmaxf(base_di, event_severity) + temporal_boost + episode_boost;
    g_severity = clampf(g_severity, 0.0f, 100.0f);

    // --- Step 6: Final DI = severity (confidence already applied via effective scores) ---
    // Note: State machine gates escalation on g_fusion_confidence separately
    g_fusion.di_raw = g_severity;

    // EMA temporal smoothing (preserved)
    g_fusion.di_smooth = emaSmooth(g_fusion.di_smooth, g_fusion.di_raw, DI_EMA_ALPHA);

    // --- Step 7: Active sensor counting (confidence-aware) ---
    g_fusion.active_sensors = 0;
    if (snd_eff > SENSOR_ACTIVE_THRESH) g_fusion.active_sensors++;
    if (mot_eff > SENSOR_ACTIVE_THRESH) g_fusion.active_sensors++;
    if (env_eff > SENSOR_ACTIVE_THRESH) g_fusion.active_sensors++;
}

// =============================================================================
// PREDICTION ENGINE
// =============================================================================

void predictionUpdate() {
    // Push current DI into history buffer
    g_di_history[g_di_history_idx] = g_fusion.di_smooth;
    g_di_history_idx = (g_di_history_idx + 1) % PREDICTION_WINDOW_SIZE;
    if (g_di_history_count < PREDICTION_WINDOW_SIZE) g_di_history_count++;

    // Compute slope via least-squares linear fit
    if (g_di_history_count < 3) {
        g_fusion.slope = 0.0f;
        return;
    }

    uint8_t n = g_di_history_count;
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

    for (uint8_t i = 0; i < n; i++) {
        // Read from oldest to newest
        uint8_t idx = (g_di_history_idx + PREDICTION_WINDOW_SIZE - n + i) % PREDICTION_WINDOW_SIZE;
        float x = (float)i;
        float y = g_di_history[idx];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    float denom = (float)n * sum_x2 - sum_x * sum_x;
    if (fabsf(denom) < 0.001f) {
        g_fusion.slope = 0.0f;
    } else {
        g_fusion.slope = ((float)n * sum_xy - sum_x * sum_y) / denom;
    }

    // v2: Update temporal behavior model (acceleration, instability, episodes)
    updateTemporalModel();
}

// =============================================================================
// STATE MACHINE
// =============================================================================

void stateMachineUpdate() {
    unsigned long now = millis();
    unsigned long dwell = now - g_state_entered;
    float di = g_fusion.di_smooth;
    uint8_t active = g_fusion.active_sensors;

    // v2: Confidence gate — escalation requires minimum confidence
    bool conf_ok = (g_fusion_confidence >= 0.4f);

    SystemState next = g_state;

    switch (g_state) {
        case STATE_NORMAL:
            if (di >= THRESH_NORMAL_TO_MILD && dwell >= STATE_MIN_DWELL_MS && conf_ok) {
                next = STATE_MILD_DISCOMFORT;
            }
            break;

        case STATE_MILD_DISCOMFORT:
            // De-escalation always allowed (safety)
            if (di < THRESH_MILD_TO_NORMAL && dwell >= STATE_MIN_DWELL_MS) {
                next = STATE_NORMAL;
            } else if (di >= THRESH_MILD_TO_HIGH && active >= MIN_SENSORS_HIGH &&
                       dwell >= STATE_MIN_DWELL_MS && conf_ok) {
                next = STATE_HIGH_DISCOMFORT;
            }
            // v2: Rapid escalation bypass (event-driven)
            else if (g_events.rapid_escalation && g_events.cry_event &&
                     dwell >= STATE_MIN_DWELL_MS && conf_ok) {
                next = STATE_HIGH_DISCOMFORT;
            }
            break;

        case STATE_HIGH_DISCOMFORT:
            if (di < THRESH_HIGH_TO_MILD || active < MIN_SENSORS_HIGH) {
                if (dwell >= STATE_MIN_DWELL_MS) {
                    next = STATE_MILD_DISCOMFORT;
                }
            } else if (dwell >= STATE_MIN_DWELL_MS && conf_ok) {
                // Check CRITICAL transition
                unsigned long time_in_high = now - g_high_entered;
                bool sustained = (time_in_high >= HIGH_SUSTAIN_FOR_CRIT_MS);

                if (di >= THRESH_HIGH_TO_CRITICAL && active >= MIN_SENSORS_HIGH && sustained) {
                    next = STATE_CRITICAL;
                } else if (di >= THRESH_CRITICAL_3SENS && active >= MIN_SENSORS_CRITICAL_3) {
                    next = STATE_CRITICAL;
                }
            }
            break;

        case STATE_CRITICAL:
            if (di < THRESH_CRITICAL_TO_HIGH && dwell >= CRITICAL_MIN_DWELL_MS) {
                next = STATE_HIGH_DISCOMFORT;
            }
            break;
    }

    // Apply transition
    if (next != g_state) {
        g_prev_state = g_state;
        g_state = next;
        g_state_entered = now;
        if (next == STATE_HIGH_DISCOMFORT) {
            g_high_entered = now;
        }
        updateAdaptiveSampling();
        g_state_changed = true;
        Serial.printf("[STATE] %s → %s (DI=%.1f, active=%u)\n",
                      stateToStr(g_prev_state), stateToStr(g_state),
                      di, active);
    }
}

const char* stateToStr(SystemState s) {
    switch (s) {
        case STATE_NORMAL:           return "NORMAL";
        case STATE_MILD_DISCOMFORT:  return "MILD";
        case STATE_HIGH_DISCOMFORT:  return "HIGH";
        case STATE_CRITICAL:         return "CRITICAL";
        default:                     return "UNKNOWN";
    }
}

void updateAdaptiveSampling() {
    switch (g_state) {
        case STATE_NORMAL:
            g_sample_interval = SAMPLE_INTERVAL_NORMAL;
            g_blynk_interval = 30000;
            break;
        case STATE_MILD_DISCOMFORT:
            g_sample_interval = SAMPLE_INTERVAL_MILD;
            g_blynk_interval = 15000;
            break;
        case STATE_HIGH_DISCOMFORT:
            g_sample_interval = SAMPLE_INTERVAL_HIGH;
            g_blynk_interval = 5000;
            break;
        case STATE_CRITICAL:
            g_sample_interval = SAMPLE_INTERVAL_CRITICAL;
            g_blynk_interval = 5000;
            break;
    }
}

// =============================================================================
// RELAY CONTROL
// =============================================================================

void controlFanRelay() {
    // SAFE_HALT wins over everything, including manual override
    if (g_safe_halt) {
        setRelay(PIN_RELAY_FAN, false);
        g_fan_relay_on = false;
        return;
    }

    // Manual override check
    if (g_fan_override == 0) {
        setRelay(PIN_RELAY_FAN, false);
        g_fan_relay_on = false;
        return;
    }
    if (g_fan_override == 1) {
        setRelay(PIN_RELAY_FAN, true);
        g_fan_relay_on = true;
        return;
    }

    // Auto logic
    bool should_be_on = false;

    // Condition 1: HIGH/CRITICAL state with high environmental score
    if ((g_state == STATE_HIGH_DISCOMFORT || g_state == STATE_CRITICAL) &&
        g_env.score > 0.5f) {
        should_be_on = true;
    }

    // Condition 2: CRITICAL state always
    if (g_state == STATE_CRITICAL) {
        should_be_on = true;
    }

    // Condition 3: Absolute temperature override
    if (g_env.temperature > 28.0f) {
        should_be_on = true;
    }

    // Condition 4: Return to NORMAL for >30s → turn off
    if (g_state == STATE_NORMAL &&
        (millis() - g_state_entered) > 30000) {
        should_be_on = false;
    }

    // Debounce: don't toggle relay faster than every 5 seconds
    if (should_be_on != g_fan_relay_on) {
        if (millis() - g_fan_last_toggle >= RELAY_DEBOUNCE_MS) {
            setRelay(PIN_RELAY_FAN, should_be_on);
            g_fan_relay_on = should_be_on;
            g_fan_last_toggle = millis();
            Serial.printf("[RELAY] Fan %s\n", should_be_on ? "ON" : "OFF");
        }
    }
}

void controlMosquitoRelay() {
    // SAFE_HALT wins over everything, including manual override
    if (g_safe_halt) {
        setRelay(PIN_RELAY_MOSQUITO, false);
        g_mosquito_relay_on = false;
        return;
    }

    // Manual override
    if (g_mosq_override == 0) {
        setRelay(PIN_RELAY_MOSQUITO, false);
        g_mosquito_relay_on = false;
        return;
    }
    if (g_mosq_override == 1) {
        setRelay(PIN_RELAY_MOSQUITO, true);
        g_mosquito_relay_on = true;
        return;
    }

    // Auto logic based ONLY on mosquito index (INDEPENDENT of infant state)
    bool should_be_on = g_mosquito_relay_on;  // Hysteresis

    float idx_value = g_mosquito_index * 10.0f;  // Convert back to 0-10 scale

    if (idx_value >= MOSQUITO_ON_THRESHOLD) {
        should_be_on = true;
    } else if (idx_value < MOSQUITO_OFF_THRESHOLD) {
        should_be_on = false;
    }

    // API stale fallback: if data too old, turn ON (safe default)
    if (millis() - g_mosquito_last_valid > API_STALE_MAX_MS) {
        should_be_on = true;
    }

    if (should_be_on != g_mosquito_relay_on) {
        if (millis() - g_mosq_last_toggle >= RELAY_DEBOUNCE_MS) {
            setRelay(PIN_RELAY_MOSQUITO, should_be_on);
            g_mosquito_relay_on = should_be_on;
            g_mosq_last_toggle = millis();
            Serial.printf("[RELAY] Mosquito repellent %s\n", should_be_on ? "ON" : "OFF");
        }
    }
}

void setRelay(uint8_t pin, bool on) {
    // Active LOW relay: LOW = ON, HIGH = OFF
    digitalWrite(pin, on ? LOW : HIGH);
}

// =============================================================================
// ACCUWEATHER MOSQUITO API
// =============================================================================

void fetchMosquitoIndex() {
    if (!g_wifi_connected) return;

    String url = "https://dataservice.accuweather.com/indices/v1/daily/1day/";
    url += ACCUWEATHER_LOCATION;
    url += "/";
    url += MOSQUITO_INDEX_ID;
    url += "?apikey=";
    url += ACCUWEATHER_API_KEY;
    
    HTTPClient http;
    http.setTimeout(10000);
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode == 200) {
        String payload = http.getString();
        Serial.printf("[API] Response: %s\n", payload.substring(0, 200).c_str());
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, payload);

        if (!err && doc.is<JsonArray>() && doc.size() > 0) {
            JsonVariant val = doc[0]["Value"];
            if (!val.isNull()) {
                float value = val.as<float>();
                if (value >= 0.0f && value <= 10.0f) {
                    g_mosquito_index = value / 10.0f;
                    g_mosquito_last_valid = millis();
                    Serial.printf("[API] Mosquito index: %.1f/10\n", value);
                } else {
                    Serial.printf("[API] Value out of range: %.2f\n", value);
                }
            } else {
                Serial.println("[API] Value field is null/missing");
            }
        } else {
            Serial.printf("[API] JSON parse error: %s\n", err.c_str());
        }
    } else {
        Serial.printf("[API] HTTP error: %d\n", httpCode);
        // Keep last valid value — no change to g_mosquito_index
    }

    http.end();

    // If stale beyond max, set moderate default
    if (millis() - g_mosquito_last_valid > API_STALE_MAX_MS) {
        g_mosquito_index = 0.5f;
        Serial.println("[API] Data stale — defaulting to moderate (0.5)");
    }
}

// =============================================================================
// WIFI MANAGEMENT
// =============================================================================

void wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) {
        g_wifi_connected = true;
        g_wifi_retries = 0;
        return;
    }

    // Backoff if too many retries
    if (g_wifi_retries >= WIFI_RETRY_MAX) {
        if (millis() - g_last_wifi_attempt < WIFI_BACKOFF_MS) return;
        g_wifi_retries = 0;
    }

    Serial.println("[WIFI] Connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
        g_wifi_connected = true;
        g_wifi_retries = 0;
        Serial.printf("[WIFI] Connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        g_wifi_connected = false;
        g_wifi_retries++;
        g_last_wifi_attempt = millis();
        Serial.printf("[WIFI] Failed (attempt %u/%u)\n", g_wifi_retries, WIFI_RETRY_MAX);
    }
}

// Duty-cycle the radio in NORMAL state (design §G.2). MILD/HIGH/CRITICAL stay
// connected so alerts and manual overrides are never delayed.
void wifiDutyCycleUpdate(unsigned long now) {
    if (g_state != STATE_NORMAL) {
        if (!g_wifi_window_on) {
            g_wifi_window_on = true;
            g_wifi_window_started = now;
        }
        if (!g_wifi_connected) {
            wifiConnect();
            if (g_wifi_connected) {
                Blynk.config(BLYNK_AUTH_TOKEN);
                Blynk.connect(5000);
                flushAlertQueue();
            }
        }
        return;
    }

    if (g_wifi_window_on) {
        if (now - g_wifi_window_started >= WIFI_WINDOW_ACTIVE_MS) {
            WiFi.mode(WIFI_OFF);
            g_wifi_connected = false;
            g_wifi_window_on = false;
            Serial.println("[WIFI] NORMAL duty cycle — radio off");
        }
    } else if (now - g_wifi_window_started >= WIFI_WINDOW_CYCLE_MS) {
        g_wifi_window_on = true;
        g_wifi_window_started = now;
        wifiConnect();
        if (g_wifi_connected) {
            Blynk.config(BLYNK_AUTH_TOKEN);
            Blynk.connect(5000);
            flushAlertQueue();
        }
    }
}

// =============================================================================
// BLYNK
// =============================================================================

// Store an event alert for later delivery. When the queue is full the oldest
// slot is overwritten so a new alert is never silently dropped.
void queueAlert(const char* code, const char* msg) {
    uint8_t slot = ALERT_QUEUE_SIZE;
    for (uint8_t i = 0; i < ALERT_QUEUE_SIZE; i++) {
        if (!g_alert_queue[i].used) { slot = i; break; }
    }
    if (slot == ALERT_QUEUE_SIZE) {
        slot = g_alert_head;
        g_alert_head = (g_alert_head + 1) % ALERT_QUEUE_SIZE;
    }
    snprintf(g_alert_queue[slot].code, sizeof(g_alert_queue[slot].code),
             "%s", code);
    snprintf(g_alert_queue[slot].msg, sizeof(g_alert_queue[slot].msg),
             "%s", msg);
    g_alert_queue[slot].used = true;
}

void flushAlertQueue() {
    // WiFi up does not mean the Blynk cloud is reachable — keep alerts queued
    // until a real cloud connection exists (design §I.2).
    if (!g_wifi_connected || !Blynk.connected()) return;
    for (uint8_t i = 0; i < ALERT_QUEUE_SIZE; i++) {
        if (!g_alert_queue[i].used) continue;
        Blynk.logEvent(g_alert_queue[i].code, g_alert_queue[i].msg);
        g_alert_queue[i].used = false;
    }
}

void blynkUpdate() {
    if (!g_wifi_connected) return;

    flushAlertQueue();

    Blynk.virtualWrite(VPIN_DI_SCORE, g_fusion.di_smooth);
    Blynk.virtualWrite(VPIN_STATE, (int)g_state);
    Blynk.virtualWrite(VPIN_TEMPERATURE, g_env.temperature);
    Blynk.virtualWrite(VPIN_HUMIDITY, g_env.humidity);
    Blynk.virtualWrite(VPIN_SOUND_LEVEL, g_sound.score * 100.0f);
    Blynk.virtualWrite(VPIN_MOTION_LEVEL, g_motion.score * 100.0f);
    Blynk.virtualWrite(VPIN_MOSQUITO_IDX, g_mosquito_index * 10.0f);
    Blynk.virtualWrite(VPIN_PREDICTION, g_fusion.slope);

    // State transition notification (v2: event-aware messages)
    if (g_state_changed) {
        g_state_changed = false;
        if (g_state == STATE_HIGH_DISCOMFORT) {
            // v2: Include event context in notification
            const char* reason = "sensors elevated";
            if (g_events.cry_event) reason = "crying detected";
            else if (g_events.agitation_event) reason = "agitation detected";
            else if (g_events.heat_stress_event) reason = "heat stress";
            char msg[80];
            snprintf(msg, sizeof(msg), "Baby uncomfortable — %s (conf: %.0f%%)",
                     reason, g_fusion_confidence * 100.0f);
            queueAlert("high_discomfort", msg);
        } else if (g_state == STATE_CRITICAL) {
            char msg[80];
            snprintf(msg, sizeof(msg), "CRITICAL: Immediate attention! Sev:%.0f Ep:%u",
                     g_severity, g_events.cry_episodes);
            queueAlert("critical_alert", msg);
        } else if (g_state == STATE_NORMAL) {
            queueAlert("recovered", "Baby returned to normal state");
        }
    }

    // Prediction alert (rate-limited to once per 5 minutes)
    static unsigned long lastTrendWarning = 0;
    static unsigned long lastRapidEscalation = 0;

    if (g_state == STATE_NORMAL && g_fusion.slope > SLOPE_WARNING_THRESH) {
        if (millis() - lastTrendWarning >= 300000UL) {
            queueAlert("trend_warning", "Rising discomfort trend detected");
            lastTrendWarning = millis();
        }
    } else if (g_state == STATE_MILD_DISCOMFORT && g_fusion.slope > SLOPE_CRITICAL_THRESH) {
        if (millis() - lastRapidEscalation >= 300000UL) {
            queueAlert("rapid_escalation", "Rapid escalation — intervention may be needed");
            lastRapidEscalation = millis();
        }
    }
}

// Blynk virtual pin handlers for manual override
BLYNK_WRITE(VPIN_FAN_OVERRIDE) {
    g_fan_override = param.asInt() - 1;  // 0=auto, 1=off, 2=on → -1, 0, 1
    Serial.printf("[BLYNK] Fan override: %d\n", g_fan_override);
}

BLYNK_WRITE(VPIN_MOSQ_OVERRIDE) {
    g_mosq_override = param.asInt() - 1;
    Serial.printf("[BLYNK] Mosquito override: %d\n", g_mosq_override);
}

// The cloud link can come up after the WiFi link, and controls set while the
// radio was off would otherwise never reach the device.
BLYNK_CONNECTED() {
    Blynk.syncVirtual(VPIN_FAN_OVERRIDE, VPIN_MOSQ_OVERRIDE);
    flushAlertQueue();
}

// =============================================================================
// SYSTEM HEALTH CHECK
// =============================================================================

uint8_t countFailedSensors() {
    uint8_t failed = 0;
    if (g_health_mpu == SENSOR_FAILED) failed++;
    if (g_health_dht == SENSOR_FAILED) failed++;
    if (g_health_sound == SENSOR_FAILED) failed++;
    return failed;
}

void systemHealthCheck() {
    uint8_t failed = countFailedSensors();
    static bool degraded_reported = false;

    if (failed >= 3) {
        // SAFE HALT: latch until at least two sensors recover
        if (!g_safe_halt) {
            g_safe_halt = true;
            queueAlert("system_fault", "SYSTEM FAULT: Multiple sensor failures!");
        }
        Serial.println("[HEALTH] CRITICAL: 3+ sensors failed — SAFE HALT (latched)");
        setRelay(PIN_RELAY_FAN, false);
        setRelay(PIN_RELAY_MOSQUITO, false);
        g_fan_relay_on = false;
        g_mosquito_relay_on = false;
    } else if (failed >= 2) {
        if (!degraded_reported) {
            Serial.println("[HEALTH] WARNING: 2 sensors failed — degraded mode");
            queueAlert("degraded_mode", "System degraded — 2 sensors offline");
            degraded_reported = true;
        }
    } else {
        degraded_reported = false;
        if (g_safe_halt) {
            // Hysteresis: recover only when fewer than two sensors are failed
            g_safe_halt = false;
            queueAlert("system_recovered", "System recovered — SAFE HALT cleared");
            Serial.println("[HEALTH] Sensors recovered — SAFE HALT cleared");
        }
    }
}

// =============================================================================
// SERIAL STATUS OUTPUT
// =============================================================================

void printStatusSerial() {
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("[%8lu ms] State: %s | DI: %.1f (sev: %.1f, conf: %.2f)\n",
                  millis(), stateToStr(g_state), g_fusion.di_smooth,
                  g_severity, g_fusion_confidence);
    Serial.printf("  Scores → Sound: %.2f  Motion: %.2f  Env: %.2f\n",
                  g_sound.score, g_motion.score, g_env.score);
    Serial.printf("  Conf   → Sound: %.2f  Motion: %.2f  Env: %.2f\n",
                  g_confidence.sound, g_confidence.motion, g_confidence.env);
    float snd_delta = fmaxf(SOUND_CRY_THRESHOLD, g_calib.sound_noise_floor * 0.3f);
    float snd_thresh = g_calib.sound_noise_floor + snd_delta * 0.3f;
    Serial.printf("  Sound RMS: %.0f (floor: %.0f, thresh: %.0f)\n",
                  g_sound.rms, g_calib.sound_noise_floor, snd_thresh);
    Serial.printf("  DSP → ZCR: %.2f  EVar: %.0f  Burst: %u  Jerk: %.3f  Rhythm: %s\n",
                  g_snd_feat.zcr, g_snd_feat.energy_variance,
                  g_snd_feat.burst_count, g_mot_feat.jerk,
                  g_mot_feat.rhythmic ? "Y" : "N");
    Serial.printf("  Events → Cry:%s(%.2f,ep%u) Agit:%s(%.2f) Env:%s HI:%.1f°C\n",
                  g_events.cry_event ? "Y" : "N", g_events.cry_intensity,
                  g_events.cry_episodes,
                  g_events.agitation_event ? "Y" : "N", g_events.agitation_intensity,
                  g_events.env_stress_event ? "Y" : "N", g_heat_index);
    Serial.printf("  Temporal → EpInt: %.2f  Accel: %+.2f  Instab: %.1f\n",
                  g_temporal.episode_intensity, g_temporal.di_acceleration,
                  g_temporal.instability);
    Serial.printf("  Active: %u | Slope: %+.2f | Temp: %.1f°C  Hum: %.1f%%\n",
                  g_fusion.active_sensors, g_fusion.slope,
                  g_env.temperature, g_env.humidity);
    Serial.printf("  Relays → Fan: %s  Mosq: %s | Mosquito: %.1f/10\n",
                  g_fan_relay_on ? "ON" : "OFF",
                  g_mosquito_relay_on ? "ON" : "OFF",
                  g_mosquito_index * 10.0f);
    Serial.printf("  Health → MPU:%s DHT:%s SND:%s\n",
                  g_health_mpu == SENSOR_OK ? "OK" : "FAIL",
                  g_health_dht == SENSOR_OK ? "OK" : "FAIL",
                  g_health_sound == SENSOR_OK ? "OK" : "FAIL");
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔══════════════════════════════════════════════╗");
    Serial.println("║   LullabyGuard — Smart Infant Monitor       ║");
    Serial.println("║   ESP32-S3 | Multi-Sensor Fusion System     ║");
    Serial.println("╚══════════════════════════════════════════════╝\n");

    // --- GPIO Init ---
    pinMode(PIN_RELAY_FAN, OUTPUT);
    pinMode(PIN_RELAY_MOSQUITO, OUTPUT);
    setRelay(PIN_RELAY_FAN, false);
    setRelay(PIN_RELAY_MOSQUITO, false);

    // --- I2C Init ---
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    delay(50);

    // --- MPU6050 Init + Calibration ---
    if (!mpuInit()) {
        Serial.println("[SYS] MPU6050 failed — motion monitoring disabled");
        g_health_mpu = SENSOR_FAILED;
    } else {
        mpuCalibrate();
    }

    // --- Filter Init ---
    filterInit(g_filt_ax, MOTION_FILTER_SIZE);
    filterInit(g_filt_ay, MOTION_FILTER_SIZE);
    filterInit(g_filt_az, MOTION_FILTER_SIZE);
    filterInit(g_filt_mag, MOTION_FILTER_SIZE);
    // --- INMP441 Init ---
    if (!soundInit()) {
        g_health_sound = SENSOR_FAILED;
    } else {
        soundCalibrateNoiseFloor();
    }

    // --- DHT22 Init ---
    dhtInit();

    // --- WiFi + Blynk ---
    wifiConnect();
    if (g_wifi_connected) {
        Blynk.config(BLYNK_AUTH_TOKEN);
        Blynk.connect(5000);
        Serial.println("[BLYNK] Connected");
    }
    g_wifi_window_on = true;
    g_wifi_window_started = millis();

    // --- Initialize state ---
    g_state = STATE_NORMAL;
    g_state_entered = millis();
    g_fusion.di_smooth = 0.0f;
    g_mosquito_last_valid = millis();

    // --- v2: Initialize event-driven fusion state ---
    memset(&g_snd_feat, 0, sizeof(g_snd_feat));
    memset(&g_mot_feat, 0, sizeof(g_mot_feat));
    memset(&g_events, 0, sizeof(g_events));
    memset(&g_temporal, 0, sizeof(g_temporal));
    g_confidence.sound = 1.0f;
    g_confidence.motion = 1.0f;
    g_confidence.env = 1.0f;
    g_severity = 0.0f;
    g_fusion_confidence = 1.0f;

    // --- Timing ---
    g_last_sensor_read = millis();
    g_last_blynk_update = millis();

    // First mosquito API poll immediately at startup
    fetchMosquitoIndex();
    controlMosquitoRelay();
    g_last_api_poll = millis();

    Serial.println("\n[SYS] === SYSTEM READY — Monitoring Active ===\n");
}

// Conservative light sleep between sensor reads (design §G.3). Only entered in
// NORMAL state while the WiFi radio is off, and never past a scheduled deadline.
void enterLightSleepIfIdle(unsigned long now) {
#if ENABLE_LIGHT_SLEEP
    if (g_state != STATE_NORMAL || g_wifi_connected) return;
    if (now - g_last_api_poll >= API_POLL_INTERVAL_MS) return;
    if (now - g_last_blynk_update >= g_blynk_interval) return;
    if (now - g_last_sensor_read >= g_sample_interval) return;

    unsigned long sleep_ms = g_sample_interval - (now - g_last_sensor_read);

    if (!g_wifi_window_on) {
        if (now - g_wifi_window_started >= WIFI_WINDOW_CYCLE_MS) return;
        unsigned long until_wifi =
            WIFI_WINDOW_CYCLE_MS - (now - g_wifi_window_started);
        if (until_wifi < sleep_ms) sleep_ms = until_wifi;
    }

    if (sleep_ms < 50) return;  // Not worth the transition
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
    esp_light_sleep_start();
#else
    (void)now;
#endif
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    unsigned long now = millis();

    // ═══ WiFi duty cycle (blocking reconnect may advance the clock) ═══
    wifiDutyCycleUpdate(now);
    now = millis();

    // ═══ TASK 1: Sensor Reading + Processing (adaptive interval) ═══
    if (now - g_last_sensor_read >= g_sample_interval) {
        g_last_sensor_read = now;

        // Read all sensors
        if (g_health_mpu != SENSOR_FAILED) {
            mpuReadProcessed();
        } else {
            g_motion.score = 0.0f;
        }

        soundReadRMS();

        // DHT22: minimum 2s between reads
        static unsigned long lastDHTRead = 0;
        if (now - lastDHTRead >= 2000) {
            dhtReadFiltered();
            lastDHTRead = now;
        }

        // ═══ TASK 2: Fusion Engine ═══
        fusionCompute();

        // ═══ TASK 3: Prediction Engine ═══
        predictionUpdate();

        // ═══ TASK 4: State Machine ═══
        stateMachineUpdate();

        // ═══ TASK 5: Actuator Control ═══
        controlFanRelay();
        controlMosquitoRelay();

        // ═══ TASK 6: Health Check ═══
        systemHealthCheck();

        // ═══ Serial output (every 5s minimum) ═══
        static unsigned long lastPrint = 0;
        if (now - lastPrint >= 5000) {
            printStatusSerial();
            lastPrint = now;
        }
    }

    // ═══ INDEPENDENT: Mosquito API Poll (every 30 min) ═══
    if (now - g_last_api_poll >= API_POLL_INTERVAL_MS) {
        g_last_api_poll = now;
        fetchMosquitoIndex();
        controlMosquitoRelay();
    }

    // ═══ INDEPENDENT: Blynk Update ═══
    if (now - g_last_blynk_update >= g_blynk_interval) {
        g_last_blynk_update = now;
        blynkUpdate();
    }

    // ═══ INDEPENDENT: Blynk Run (process incoming) ═══
    if (g_wifi_connected) {
        Blynk.run();
    }

    // ═══ WiFi reconnection check (only inside an active duty-cycle window) ═══
    if (!g_wifi_connected && g_wifi_window_on && now - g_last_wifi_attempt >= 60000) {
        wifiConnect();
        if (g_wifi_connected) {
            Blynk.config(BLYNK_AUTH_TOKEN);
            Blynk.connect(5000);
            flushAlertQueue();
        }
    }

    // ═══ Power: light sleep in NORMAL while the radio is off ═══
    enterLightSleepIfIdle(now);
}
