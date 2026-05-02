#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

Servo myservo;

const int servoPin = 19;
const int buttonPin = 4;
const int trigPin = 27;
const int echoPin = 26;

// Ajusta se necessário
const int openAngle = 180;
const int closeAngle = 20;

typedef struct struct_message {
  uint8_t avgPct;   // 0 a 100
} struct_message;

struct_message incomingMsg;

volatile uint8_t lastAvgPct = 0;
unsigned long lastPacketTime = 0;

int currentAngle = openAngle;
int targetAngle = openAngle;

float readDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

int mapPctToAngle(int pct) {
  pct = constrain(pct, 0, 100);
  return map(pct, 0, 100, openAngle, closeAngle);
}

void moveServoFast() {
  int diff = targetAngle - currentAngle;

  if (abs(diff) <= 2) {
    currentAngle = targetAngle;
    myservo.write(currentAngle);
    return;
  }

  // Quanto maior a diferença, maior o passo
  int stepSize = abs(diff) / 4;

  if (stepSize < 2) stepSize = 2;
  if (stepSize > 12) stepSize = 12;

  if (diff > 0) currentAngle += stepSize;
  else currentAngle -= stepSize;

  currentAngle = constrain(currentAngle, min(openAngle, closeAngle), max(openAngle, closeAngle));
  myservo.write(currentAngle);
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
    lastAvgPct = incomingMsg.avgPct;
    lastPacketTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400);

  myservo.write(openAngle);
  currentAngle = openAngle;
  targetAngle = openAngle;

  WiFi.mode(WIFI_STA);
  delay(200);

  Serial.print("MAC da garra: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Garra proporcional rápida pronta");
}

void loop() {
  float distance = readDistanceCm();
  bool emergencyButton = (digitalRead(buttonPin) == LOW);
  bool tooClose = (distance < 10.0);

  if (emergencyButton) {
    targetAngle = openAngle;
  }
  else if (millis() - lastPacketTime > 1000) {
    targetAngle = openAngle;
  }
  else if (tooClose) {
    int safePct = min((int)lastAvgPct, 30);
    targetAngle = mapPctToAngle(safePct);
  }
  else {
    targetAngle = mapPctToAngle(lastAvgPct);
  }

  moveServoFast();

  Serial.print("pct=");
  Serial.print(lastAvgPct);
  Serial.print(" | alvo=");
  Serial.print(targetAngle);
  Serial.print(" | atual=");
  Serial.println(currentAngle);

  delay(2);
}
