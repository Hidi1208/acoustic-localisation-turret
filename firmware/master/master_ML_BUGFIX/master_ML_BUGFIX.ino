/*
 * SIXSEVEN Iron Dome — Master ML FINAL (Bugs Fixed)
 * 
 * CRITICAL FIXES:
 *   - Baseline for Mic3/Mic4 was going NEGATIVE → e3/e4 became 57 million
 *   - This made elevRatio=1.0 always → tilt maxed out → every sound = "Mic4"
 *   - Pan was broken because e1 and e2 were 0 (below negative baseline)
 *   - Now: baselines clamped to ≥0, slave connection verified before calibrating
 */

#include <driver/i2s.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// =============================================
// HARDWARE
// =============================================
uint8_t slaveMAC[] = {0x20, 0xE7, 0xC8, 0x59, 0x6A, 0x84};

#define I2S_WS0    15
#define I2S_SCK0   14
#define I2S_SD0    32
#define PAN_PIN    13
#define TILT_PIN   12
#define SYNC_PIN   27

#define BUFFER_SIZE     256
#define SAMPLE_RATE     44100
#define NOISE_GATE      60000
#define PEAK_THRESHOLD  2.5
#define MIC_SPACING     0.20f
#define SPEED_OF_SOUND  343.0f
#define MAX_LAG         26
#define SERVO_KEEPALIVE_MS 2000

// =============================================
// ML PAN LOOKUP TABLE (from 2,661 samples)
// =============================================
#define PAN_LUT_SIZE 41
const float pan_lut_frac[PAN_LUT_SIZE] = {
  0.000f, 0.025f, 0.050f, 0.075f, 0.100f, 0.125f, 0.150f, 0.175f,
  0.200f, 0.225f, 0.250f, 0.275f, 0.300f, 0.325f, 0.350f, 0.375f,
  0.400f, 0.425f, 0.450f, 0.475f, 0.500f, 0.525f, 0.550f, 0.575f,
  0.600f, 0.625f, 0.650f, 0.675f, 0.700f, 0.725f, 0.750f, 0.775f,
  0.800f, 0.825f, 0.850f, 0.875f, 0.900f, 0.925f, 0.950f, 0.975f,
  1.000f
};
const float pan_lut_angle[PAN_LUT_SIZE] = {
  16.3f, 14.1f, 12.5f, 11.2f, 13.9f, 18.4f, 22.1f, 24.5f,
  27.0f, 32.5f, 38.7f, 43.4f, 48.7f, 52.1f, 56.3f, 60.6f,
  67.2f, 73.5f, 79.1f, 83.4f, 87.5f, 91.8f, 96.2f, 100.3f,
  104.8f, 109.1f, 113.2f, 117.0f, 120.5f, 124.1f, 128.3f, 133.2f,
  138.6f, 143.7f, 148.2f, 152.4f, 156.1f, 160.3f, 165.2f, 170.8f,
  176.4f
};

#define MIC3_GAIN  11.64194639044f
#define MIC4_GAIN  37.524113058793f
#define TDOA_WEIGHT 0.5f

// =============================================
// GLOBALS
// =============================================
Servo panServo;
Servo tiltServo;
int currentPan  = 95;
int currentTilt = 0;
unsigned long lastServoWrite = 0;

// Baselines — initialized to safe positive values
float baseline1 = 50000;
float baseline2 = 50000;
float baseline3 = 1000;
float baseline4 = 1000;

// Track if slave has connected
volatile bool slaveConnected = false;
volatile uint32_t slavePacketCount = 0;

typedef struct {
  int32_t amp3;
  int32_t amp4;
  int32_t smoothed3;
  int32_t smoothed4;
  uint32_t peakTime3;
  uint32_t peakTime4;
  uint32_t syncTime;
  uint8_t  hasPeak;
} SyncMicData;

volatile SyncMicData slaveData;

void onDataReceived(const esp_now_recv_info* info, const uint8_t* data, int len) {
  memcpy((void*)&slaveData, data, sizeof(SyncMicData));
  slaveConnected = true;
  slavePacketCount++;
}

// =============================================
// SERVO MANAGEMENT
// =============================================
void initServos() {
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  
}

void servoKeepalive() {
  unsigned long now = millis();
  if (now - lastServoWrite > SERVO_KEEPALIVE_MS) {
    if (!panServo.attached()) panServo.attach(PAN_PIN, 500, 2400);
    if (!tiltServo.attached()) tiltServo.attach(TILT_PIN, 500, 2400);
    panServo.write(currentPan);
    tiltServo.write(currentTilt);
    lastServoWrite = now;
  }
}

void movePanTo(int angle) {
  angle = constrain(angle, 0, 180);
  if (!panServo.attached()) panServo.attach(PAN_PIN, 500, 2400);
  panServo.write(angle);
  currentPan = angle;
  lastServoWrite = millis();
}

void moveTiltTo(int angle) {
  angle = constrain(angle, 0, 180);
  if (!tiltServo.attached()) tiltServo.attach(TILT_PIN, 500, 2400);
  tiltServo.write(angle);
  currentTilt = angle;
  lastServoWrite = millis();
}

// =============================================
// I2S
// =============================================
void setupI2S() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = true
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK0,
    .ws_io_num  = I2S_WS0,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD0
  };
  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
}

// =============================================
// HELPERS
// =============================================
int32_t getAmplitude(int32_t* buf, int len) {
  int32_t maxVal = 0;
  for (int i = 0; i < len; i++) {
    int32_t val = abs(buf[i] >> 8);
    if (val > maxVal) maxVal = val;
  }
  return maxVal;
}

// SAFE baseline update — never allows negative baselines
void updateBaseline(float &baseline, float value, float alpha) {
  float val = max(0.0f, value);  // clamp input to non-negative
  baseline = max(0.0f, baseline * (1.0f - alpha) + val * alpha);  // clamp result too
}

// SAFE energy calculation — only positive if above positive baseline
float safeEnergy(float raw, float baseline) {
  if (baseline <= 0) return raw;  // no valid baseline yet, use raw
  if (raw > baseline) return raw - baseline;
  return 0;
}

// =============================================
// ML PAN
// =============================================
float predictPanML(float e1, float e2) {
  float frac = e1 / (e1 + e2 + 1.0f);
  if (frac <= pan_lut_frac[0]) return pan_lut_angle[0];
  if (frac >= pan_lut_frac[PAN_LUT_SIZE - 1]) return pan_lut_angle[PAN_LUT_SIZE - 1];
  for (int i = 0; i < PAN_LUT_SIZE - 1; i++) {
    if (frac >= pan_lut_frac[i] && frac <= pan_lut_frac[i + 1]) {
      float t = (frac - pan_lut_frac[i]) / (pan_lut_frac[i + 1] - pan_lut_frac[i] + 0.0001f);
      return pan_lut_angle[i] + t * (pan_lut_angle[i + 1] - pan_lut_angle[i]);
    }
  }
  return 90.0f;
}

// =============================================
// TDOA
// =============================================
float computeTDOA(int32_t* sig1, int32_t* sig2, int len) {
  long bestCorr = LONG_MIN;
  int bestLag = 0;
  static long corrValues[MAX_LAG * 2 + 1];
  for (int lag = -MAX_LAG; lag <= MAX_LAG; lag++) {
    long corr = 0;
    for (int i = MAX_LAG; i < len - MAX_LAG; i++) {
      int j = i + lag;
      corr += (sig1[i] >> 8) * (sig2[j] >> 8);
    }
    corrValues[lag + MAX_LAG] = corr;
    if (corr > bestCorr) {
      bestCorr = corr;
      bestLag = lag;
    }
  }
  float refined = bestLag;
  int idx = bestLag + MAX_LAG;
  if (idx > 0 && idx < 2 * MAX_LAG) {
    long y0 = corrValues[idx - 1];
    long y1 = corrValues[idx];
    long y2 = corrValues[idx + 1];
    float denom = (float)(2 * y1 - y0 - y2);
    if (fabsf(denom) > 0.001f) {
      refined = bestLag + 0.5f * (float)(y0 - y2) / denom;
    }
  }
  return refined;
}

float tdoaToPan(float tdoaSamples) {
  float dt = tdoaSamples / SAMPLE_RATE;
  float distDiff = dt * SPEED_OF_SOUND;
  distDiff = constrain(distDiff, -MIC_SPACING, MIC_SPACING);
  float sinAngle = constrain(distDiff / MIC_SPACING, -1.0f, 1.0f);
  return 90.0f + degrees(asin(sinAngle));
}

// =============================================
// TILT
// =============================================
float predictTilt(float e1, float e2, float e3, float e4) {
  float e3g = e3 * MIC3_GAIN;
  float e4g = e4 * MIC4_GAIN;
  float total = e1 + e2 + e3g + e4g + 1.0f;
  float elevRatio = (e3g + e4g) / total;
  elevRatio = constrain(elevRatio, 0.0f, 0.6f);

  float tilt;
  if (elevRatio < 0.02f) {
    tilt = elevRatio / 0.02f * 15.0f;
  } else if (elevRatio < 0.08f) {
    tilt = 15.0f + (elevRatio - 0.02f) / 0.06f * 45.0f;
  } else if (elevRatio < 0.20f) {
    tilt = 60.0f + (elevRatio - 0.08f) / 0.12f * 50.0f;
  } else if (elevRatio < 0.40f) {
    tilt = 110.0f + (elevRatio - 0.20f) / 0.20f * 45.0f;
  } else {
    tilt = 155.0f + (elevRatio - 0.40f) / 0.20f * 25.0f;
  }

  float e34 = e3g + e4g + 1.0f;
  float e4dom = e4g / e34;
  if (elevRatio > 0.03f) {
    tilt += (e4dom - 0.5f) * 25.0f;
  }

  return constrain(tilt, 0.0f, 180.0f);
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(115200);

  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);

  initServos();
  panServo.write(95);
  tiltServo.write(0);
  currentPan = 95;
  currentTilt = 0;
  lastServoWrite = millis();
  delay(300);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, slaveMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  setupI2S();

  // === WAIT FOR SLAVE CONNECTION ===
  Serial.println("Waiting for slave ESP32...");
  unsigned long waitStart = millis();
  while (!slaveConnected && (millis() - waitStart < 5000)) {
    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SYNC_PIN, LOW);
    delay(50);
  }
  if (slaveConnected) {
    Serial.println("Slave connected!");
  } else {
    Serial.println("WARNING: Slave not detected — Mic3/Mic4 will be 0");
  }

  // === CALIBRATE BASELINES ===
  Serial.println("Calibrating — stay quiet for 3 seconds...");
  for (int i = 0; i < 150; i++) {
    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SYNC_PIN, LOW);

    int32_t buf[BUFFER_SIZE * 2];
    size_t bytesRead;
    i2s_read(I2S_NUM_0, buf, sizeof(buf), &bytesRead, portMAX_DELAY);
    int32_t m1[BUFFER_SIZE], m2[BUFFER_SIZE];
    for (int j = 0; j < BUFFER_SIZE; j++) {
      m1[j] = buf[j * 2];
      m2[j] = buf[j * 2 + 1];
    }

    // SAFE baseline updates — always non-negative
    updateBaseline(baseline1, getAmplitude(m1, BUFFER_SIZE), 0.1);
    updateBaseline(baseline2, getAmplitude(m2, BUFFER_SIZE), 0.1);
    
    // Only calibrate Mic3/Mic4 if slave is connected and sending valid data
    if (slaveConnected && slaveData.amp3 >= 0) {
      updateBaseline(baseline3, slaveData.amp3, 0.1);
    }
    if (slaveConnected && slaveData.amp4 >= 0) {
      updateBaseline(baseline4, slaveData.amp4, 0.1);
    }
    delay(20);
  }

  Serial.println("=== SIXSEVEN Iron Dome — ML FINAL (Bugs Fixed) ===");
  Serial.print("Baselines: B1="); Serial.print(baseline1, 0);
  Serial.print(" B2="); Serial.print(baseline2, 0);
  Serial.print(" B3="); Serial.print(baseline3, 0);
  Serial.print(" B4="); Serial.println(baseline4, 0);
  Serial.print("Slave packets during cal: "); Serial.println(slavePacketCount);
  Serial.println("Ready");
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
  servoKeepalive();

  digitalWrite(SYNC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SYNC_PIN, LOW);

  int32_t buf[BUFFER_SIZE * 2];
  size_t bytesRead;
  i2s_read(I2S_NUM_0, buf, sizeof(buf), &bytesRead, portMAX_DELAY);

  int32_t mic1[BUFFER_SIZE], mic2[BUFFER_SIZE];
  for (int i = 0; i < BUFFER_SIZE; i++) {
    mic1[i] = buf[i * 2];
    mic2[i] = buf[i * 2 + 1];
  }

  float a1 = getAmplitude(mic1, BUFFER_SIZE);
  float a2 = getAmplitude(mic2, BUFFER_SIZE);
  float a3 = max(0.0f, (float)slaveData.amp3);  // clamp to non-negative
  float a4 = max(0.0f, (float)slaveData.amp4);

  float maxAmp = max(a1, max(a2, max(a3 , a4 )));

  // Safe baseline update during silence
  if (maxAmp < NOISE_GATE) {
    updateBaseline(baseline1, a1, 0.03);
    updateBaseline(baseline2, a2, 0.03);
    if (slaveConnected) {
      updateBaseline(baseline3, a3, 0.03);
      updateBaseline(baseline4, a4, 0.03);
    }
    return;
  }

  // Peak check
  float r1 = a1 / max(baseline1, 1.0f);
  float r2 = a2 / max(baseline2, 1.0f);
  float r3 = (baseline3 > 10) ? a3 / baseline3 : 0;
  float r4 = (baseline4 > 10) ? a4 / baseline4 : 0;
  float maxRatio = max(r1, max(r2, max(r3, r4)));

  if (maxRatio > PEAK_THRESHOLD) {
    // SAFE energy — only positive values above valid baselines
    float e1 = safeEnergy(a1, baseline1);
    float e2 = safeEnergy(a2, baseline2);
    float e3 = safeEnergy(a3, baseline3);
    float e4 = safeEnergy(a4, baseline4);

    // === PAN ===
    float mlPan = predictPanML(e1, e2);
    float tdoa = computeTDOA(mic1, mic2, BUFFER_SIZE);
    float tdoaPan = tdoaToPan(tdoa);

    float tdoaConf = min(1.0f, fabsf(tdoa) / (MAX_LAG * 0.5f));
    float weight = TDOA_WEIGHT * tdoaConf;

    float fusedPan;
    if (fabsf(tdoa) < 2.0f) {
      fusedPan = mlPan;
    } else {
      fusedPan = (1.0f - weight) * mlPan + weight * tdoaPan;
    }
    int targetPan = constrain((int)(fusedPan + 0.5f), 0, 180);

    // === TILT ===
    float tilt = predictTilt(e1, e2, e3, e4);
    int targetTilt = constrain((int)(tilt + 0.5f), 0, 180);

    // === DEBUG ===
    Serial.print("PEAK | Pan:"); Serial.print(targetPan);
    Serial.print(" Tilt:"); Serial.print(targetTilt);
    Serial.print(" | ML:"); Serial.print(mlPan, 0);
    Serial.print(" TDOA:"); Serial.print(tdoaPan, 0);
    Serial.print(" lag:"); Serial.print(tdoa, 1);
    Serial.print(" | E:"); Serial.print(e1, 0);
    Serial.print(","); Serial.print(e2, 0);
    Serial.print(","); Serial.print(e3, 0);
    Serial.print(","); Serial.print(e4, 0);
    Serial.print(" | B:"); Serial.print(baseline1, 0);
    Serial.print(","); Serial.print(baseline2, 0);
    Serial.print(","); Serial.print(baseline3, 0);
    Serial.print(","); Serial.println(baseline4, 0);

    movePanTo(targetPan);
    moveTiltTo(targetTilt);
    delay(500);
  }
}
