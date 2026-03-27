/*
 * SIXSEVEN Iron Dome — Slave FINAL
 * ESP32 #2: Mic3 + Mic4, sync wire on GPIO 27, sends via ESP-NOW
 *
 * Wiring:
 *   Mic3+4 SCK → GPIO14, WS → GPIO15, SD → GPIO32
 *   Mic3 L/R → GND, Mic4 L/R → 3.3V
 *   Sync: GPIO27 ← Master GPIO27
 *   GND shared with Master ESP32
 */

#include <driver/i2s.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t masterMAC[] = {0x2C, 0xBC, 0xBB, 0x06, 0x71, 0xE0};

#define SYNC_PIN    27
#define BUFFER_SIZE 256
#define SAMPLE_RATE 44100

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

SyncMicData micData;
int32_t s3 = 0, s4 = 0;

volatile uint32_t lastSyncTime = 0;
volatile bool syncReceived = false;

void IRAM_ATTR onSyncPulse() {
  lastSyncTime = micros();
  syncReceived = true;
}

void setup() {
  Serial.begin(115200);

  // Sync pin
  pinMode(SYNC_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(SYNC_PIN), onSyncPulse, RISING);

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // I2S
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
    .bck_io_num = 14,
    .ws_io_num = 15,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 32
  };
  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);

  Serial.println("Slave FINAL ready");
}

void loop() {
  int32_t buf[BUFFER_SIZE * 2];
  size_t bytesRead;
  i2s_read(I2S_NUM_0, buf, sizeof(buf), &bytesRead, portMAX_DELAY);
  uint32_t readTime = micros();

  // Separate channels
  int32_t mic3[BUFFER_SIZE], mic4[BUFFER_SIZE];
  for (int i = 0; i < BUFFER_SIZE; i++) {
    mic3[i] = buf[i * 2];
    mic4[i] = buf[i * 2 + 1];
  }

  // Peak amplitude and index
  int32_t maxAmp3 = 0, maxAmp4 = 0;
  int peakIdx3 = 0, peakIdx4 = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int32_t v3 = abs(mic3[i] >> 8);
    int32_t v4 = abs(mic4[i] >> 8);
    if (v3 > maxAmp3) { maxAmp3 = v3; peakIdx3 = i; }
    if (v4 > maxAmp4) { maxAmp4 = v4; peakIdx4 = i; }
  }

  // Convert peak index to timestamp
  float usPerSample = 1000000.0f / SAMPLE_RATE;
  uint32_t bufDur = (uint32_t)(BUFFER_SIZE * usPerSample);
  uint32_t bufStart = readTime - bufDur;

  // Smoothed
  s3 = s3 + (((mic3[0] >> 8) - s3) / 8);
  s4 = s4 + (((mic4[0] >> 8) - s4) / 8);

  // Pack
  micData.amp3      = maxAmp3;
  micData.amp4      = maxAmp4;
  micData.smoothed3 = s3;
  micData.smoothed4 = s4;
  micData.peakTime3 = bufStart + (uint32_t)(peakIdx3 * usPerSample);
  micData.peakTime4 = bufStart + (uint32_t)(peakIdx4 * usPerSample);
  micData.syncTime  = lastSyncTime;
  micData.hasPeak   = (maxAmp3 > 5000 || maxAmp4 > 5000) ? 1 : 0;

  esp_now_send(masterMAC, (uint8_t*)&micData, sizeof(micData));

  if (micData.hasPeak) {
    syncReceived = false;
  }
}
