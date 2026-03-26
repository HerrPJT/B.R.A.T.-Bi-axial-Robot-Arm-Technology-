#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>

Adafruit_MPU6050 mpu;

// Ordem: Thumb, Index, Middle, Ring
const int pins[4] = {34, 35, 32, 33};

// Calibração dos 4 sensores
const int OPEN_V[4]   = {1648, 1333, 1520, 1414};
const int CLOSED_V[4] = {1079,  714,  789,  753};

typedef struct struct_message {
  uint8_t avgPct;   // 0 a 100 = média do fecho da mão
} struct_message;

struct_message msg;

// MAC da garra
uint8_t receiverMac[] = {0x08, 0xB6, 0x1F, 0xEF, 0x8D, 0xF8};

unsigned long lastSend = 0;
float filteredAvg = 0.0;

int clampi(int x, int a, int b) {
  return x < a ? a : (x > b ? b : x);
}

void readOnce(int out[4]) {
  for (int i = 0; i < 4; i++) {
    out[i] = analogRead(pins[i]);
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erro ao adicionar peer");
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nBOOT OK - Luva proporcional rápida");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  setupEspNow();

  Serial.println("Luva pronta");
}

void loop() {
  int raw[4], pct[4];
  readOnce(raw);

  int sumPct = 0;

  for (int i = 0; i < 4; i++) {
    int denom = OPEN_V[i] - CLOSED_V[i];
    int p = 0;

    if (abs(denom) > 10) {
      p = (int)(100.0f * (OPEN_V[i] - raw[i]) / (float)denom);
      p = clampi(p, 0, 100);
    }

    pct[i] = p;
    sumPct += p;
  }

  int avgPct = sumPct / 4;

  // Menos filtragem = mais rápido
  filteredAvg = 0.45 * filteredAvg + 0.55 * avgPct;

  msg.avgPct = (uint8_t)filteredAvg;

  // Envio mais frequente
  if (millis() - lastSend >= 20) {
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&msg, sizeof(msg));

    Serial.print("FLEX% ");
    for (int i = 0; i < 4; i++) {
      Serial.print(pct[i]);
      if (i < 3) Serial.print(",");
    }

    Serial.print(" | media=");
    Serial.print(avgPct);
    Serial.print(" | media_filtrada=");
    Serial.print((int)filteredAvg);
    Serial.print(" | envio=");
    if (result == ESP_OK) Serial.println("OK");
    else Serial.println("ERRO");

    lastSend = millis();
  }

  delay(5);
}