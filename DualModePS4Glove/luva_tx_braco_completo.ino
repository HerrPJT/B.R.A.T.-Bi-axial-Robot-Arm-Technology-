#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>

Adafruit_MPU6050 mpu;

// Ordem: Thumb, Index, Middle, Ring
const int FLEX_PINS[4] = {34, 35, 32, 33};

const int OPEN_V[4]   = {1648, 1333, 1520, 1414};
const int CLOSED_V[4] = {1079, 714, 789, 753};

// MAC Wi-Fi do braco
uint8_t receiverMac[] = {0x08, 0xB6, 0x1F, 0xEF, 0x8D, 0xF8};

struct __attribute__((packed)) GlovePacket {
  uint8_t avgPct;      // 0 aberto, 100 fechado
  int16_t pitchDeg10;  // graus * 10
  int16_t rollDeg10;   // graus * 10
  uint8_t seq;
};

GlovePacket packet;

unsigned long lastSendMs = 0;
unsigned long lastPrintMs = 0;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_SUCCESS;

float filteredAvg = 0.0f;
float filteredPitch = 0.0f;
float filteredRoll = 0.0f;
bool filterInitialized = false;
uint8_t seqCounter = 0;

int clampi(int x, int a, int b) {
  return x < a ? a : (x > b ? b : x);
}

float clampf(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

void readFlexRaw(int out[4]) {
  for (int i = 0; i < 4; i++) {
    out[i] = analogRead(FLEX_PINS[i]);
  }
}

uint8_t flexRawToPct(int rawValue, int openValue, int closedValue) {
  int denom = openValue - closedValue;
  if (abs(denom) <= 10) return 0;
  int pct = (int)(100.0f * (openValue - rawValue) / (float)denom);
  return (uint8_t)clampi(pct, 0, 100);
}

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
#endif
  lastSendStatus = status;
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(false, true);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erro ao adicionar peer ESP-NOW");
    while (true) delay(1000);
  }
}

void setupImu() {
  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND!");
    while (true) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void readPitchRollDeg(float &pitchDeg, float &rollDeg) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  rollDeg = atan2f(ay, az) * 180.0f / PI;
  pitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  pitchDeg = clampf(pitchDeg, -90.0f, 90.0f);
  rollDeg = clampf(rollDeg, -90.0f, 90.0f);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nBOOT OK - LUVA TX BRACO COMPLETO");
  Serial.println("BUILD 2026-06-04 PITCH ROLL GRIPPER");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  setupImu();
  setupEspNow();

  Serial.print("MAC destino do braco: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                receiverMac[0], receiverMac[1], receiverMac[2],
                receiverMac[3], receiverMac[4], receiverMac[5]);
  Serial.println("Luva pronta");
}

void loop() {
  int raw[4];
  uint8_t pct[4];
  int sumPct = 0;

  readFlexRaw(raw);

  for (int i = 0; i < 4; i++) {
    pct[i] = flexRawToPct(raw[i], OPEN_V[i], CLOSED_V[i]);
    sumPct += pct[i];
  }

  int avgPct = sumPct / 4;
  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;
  readPitchRollDeg(pitchDeg, rollDeg);

  if (!filterInitialized) {
    filteredAvg = avgPct;
    filteredPitch = pitchDeg;
    filteredRoll = rollDeg;
    filterInitialized = true;
  } else {
    filteredAvg = 0.45f * filteredAvg + 0.55f * avgPct;
    filteredPitch = 0.88f * filteredPitch + 0.12f * pitchDeg;
    filteredRoll = 0.88f * filteredRoll + 0.12f * rollDeg;
  }

  packet.avgPct = (uint8_t)clampi((int)roundf(filteredAvg), 0, 100);
  packet.pitchDeg10 = (int16_t)roundf(filteredPitch * 10.0f);
  packet.rollDeg10 = (int16_t)roundf(filteredRoll * 10.0f);
  packet.seq = seqCounter++;

  if (millis() - lastSendMs >= 20) {
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&packet, sizeof(packet));
    if (result != ESP_OK) {
      Serial.printf("Erro no envio ESP-NOW: %d\n", (int)result);
    }
    lastSendMs = millis();
  }

  if (millis() - lastPrintMs >= 200) {
    Serial.print("FLEX% ");
    for (int i = 0; i < 4; i++) {
      Serial.print((int)pct[i]);
      if (i < 3) Serial.print(",");
    }
    Serial.print(" | media=");
    Serial.print(avgPct);
    Serial.print(" | filtrada=");
    Serial.print((int)packet.avgPct);
    Serial.print(" | pitch=");
    Serial.print(packet.pitchDeg10 / 10.0f, 1);
    Serial.print(" | roll=");
    Serial.print(packet.rollDeg10 / 10.0f, 1);
    Serial.print(" | envio=");
    Serial.println(lastSendStatus == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    lastPrintMs = millis();
  }

  delay(10);
}
