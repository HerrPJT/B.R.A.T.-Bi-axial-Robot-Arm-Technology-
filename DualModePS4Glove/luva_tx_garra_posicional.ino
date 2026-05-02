#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

// Ordem: Thumb, Index, Middle, Ring
const int FLEX_PINS[4] = {34, 35, 32, 33};

// Valores atuais da calibração.
// Se a luva ficar pouco precisa, estes valores têm de ser recalibrados.
const int OPEN_V[4]   = {1648, 1333, 1520, 1414};
const int CLOSED_V[4] = {1079,  714,  789,  753};

struct GlovePacket {
  uint8_t avgPct;  // 0 = aberto, 100 = fechado
};

GlovePacket packet;

// MAC Wi-Fi da garra mostrado na linha "MAC da garra para colocar na luva".
// Nao usar "BTstack up..." nem "BT Addr", porque esses sao Bluetooth.
uint8_t receiverMac[] = {0x08, 0xB6, 0x1F, 0xEF, 0x8D, 0xF8};

// A luva e a garra têm de usar exatamente o mesmo canal ESP-NOW.
static const uint8_t ESPNOW_CHANNEL = 1;

unsigned long lastSendMs = 0;
unsigned long lastPrintMs = 0;

float filteredAvg = 0.0f;
bool filterInitialized = false;

volatile bool sendBusy = false;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_SUCCESS;

int clampi(int x, int minValue, int maxValue) {
  return x < minValue ? minValue : (x > maxValue ? maxValue : x);
}

void readFlexRaw(int out[4]) {
  for (int i = 0; i < 4; i++) {
    out[i] = analogRead(FLEX_PINS[i]);
  }
}

uint8_t flexRawToPct(int rawValue, int openValue, int closedValue) {
  int denom = openValue - closedValue;

  if (abs(denom) <= 10) {
    return 0;
  }

  int pct = (int)(100.0f * (openValue - rawValue) / (float)denom);
  return (uint8_t)clampi(pct, 0, 100);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
#endif
  lastSendStatus = status;
  sendBusy = false;
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erro ao adicionar peer ESP-NOW");
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\nBOOT OK - LUVA TX PARA GARRA POSICIONAL SEM IMU");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  setupEspNow();

  Serial.print("MAC destino da garra: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                receiverMac[0], receiverMac[1], receiverMac[2],
                receiverMac[3], receiverMac[4], receiverMac[5]);
  Serial.printf("Canal ESP-NOW: %u\n", ESPNOW_CHANNEL);
}

void loop() {
  int raw[4];
  uint8_t pct[4];

  readFlexRaw(raw);

  int sumPct = 0;

  for (int i = 0; i < 4; i++) {
    pct[i] = flexRawToPct(raw[i], OPEN_V[i], CLOSED_V[i]);
    sumPct += pct[i];
  }

  int avgPct = sumPct / 4;

  if (!filterInitialized) {
    filteredAvg = avgPct;
    filterInitialized = true;
  } else {
    filteredAvg = 0.65f * filteredAvg + 0.35f * avgPct;
  }

  packet.avgPct = (uint8_t)clampi((int)filteredAvg, 0, 100);

  if (!sendBusy && millis() - lastSendMs >= 50) {
    sendBusy = true;

    esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&packet, sizeof(packet));

    if (result != ESP_OK) {
      Serial.printf("Falha no envio ESP-NOW: %d\n", (int)result);
      sendBusy = false;
    }

    lastSendMs = millis();
  }

  if (millis() - lastPrintMs >= 300) {
    Serial.print("FLEX% ");

    for (int i = 0; i < 4; i++) {
      Serial.print((int)pct[i]);
      if (i < 3) Serial.print(",");
    }

    Serial.print(" | media=");
    Serial.print(avgPct);

    Serial.print(" | filtrada=");
    Serial.println((int)packet.avgPct);

    if (lastSendStatus != ESP_NOW_SEND_SUCCESS) {
      Serial.println("Aviso: ultimo pacote ESP-NOW nao foi confirmado pela garra");
    }

    lastPrintMs = millis();
  }

  delay(2);
}
