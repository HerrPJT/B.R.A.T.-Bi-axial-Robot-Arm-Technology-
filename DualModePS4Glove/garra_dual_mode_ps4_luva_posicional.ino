#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18

// Servo posicional da garra
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2700;

static const int SERVO_OPEN_ANGLE = 0;
static const int SERVO_CLOSED_ANGLE = 180;
static const int SERVO_SAFE_ANGLE = SERVO_OPEN_ANGLE;

static const int PAD_DEADZONE = 50;
static const unsigned long GLOVE_TIMEOUT_MS = 1000;
static const unsigned long DEBUG_INTERVAL_MS = 500;

// Tem de ser igual ao canal definido na luva.
static const uint8_t ESPNOW_CHANNEL = 1;

// Se algum sentido estiver invertido, muda para true
static const bool REVERSE_GLOVE = true;
static const bool REVERSE_CONTROLLER = false;

struct GlovePacket {
  uint8_t avgPct;  // 0 = aberto, 100 = fechado
};

Servo myServo;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

volatile uint8_t latestGlovePct = 0;
volatile unsigned long lastGlovePacketMs = 0;
volatile bool gloveUpdated = false;
volatile unsigned long glovePacketCount = 0;

enum ControlMode {
  MODE_GLOVE,
  MODE_CONTROLLER
};

ControlMode controlMode = MODE_GLOVE;

int currentAngle = -1;

bool prevCross = false;
bool prevCircle = false;
bool prevSquare = false;
bool prevTriangle = false;
bool prevPS = false;

unsigned long lastDebugMs = 0;
unsigned long lastServoPrintMs = 0;

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(GlovePacket)) {
    GlovePacket packet;
    memcpy(&packet, incomingData, sizeof(packet));

    latestGlovePct = packet.avgPct;
    lastGlovePacketMs = millis();
    gloveUpdated = true;
    glovePacketCount++;
  }
}

int clampi(int x, int minValue, int maxValue) {
  return x < minValue ? minValue : (x > maxValue ? maxValue : x);
}

void writeServoAngleSafe(int angle) {
  angle = clampi(angle, 0, 180);

  if (angle != currentAngle) {
    currentAngle = angle;
    myServo.write(currentAngle);
    Serial.printf("SERVO <- angle=%d\n", currentAngle);
  }
}

int glovePctToAngle(uint8_t pct) {
  pct = constrain(pct, 0, 100);

  int angle = map((int)pct, 0, 100, SERVO_OPEN_ANGLE, SERVO_CLOSED_ANGLE);

  if (REVERSE_GLOVE) {
    angle = SERVO_OPEN_ANGLE + SERVO_CLOSED_ANGLE - angle;
  }

  return clampi(angle, 0, 180);
}

int controllerToAngle(ControllerPtr ctl) {
  int right = ctl->axisRY();

  if (abs(right) < PAD_DEADZONE) {
    right = 0;
  }

  int angle = map(right, -512, 512, SERVO_OPEN_ANGLE, SERVO_CLOSED_ANGLE);

  if (REVERSE_CONTROLLER) {
    angle = SERVO_OPEN_ANGLE + SERVO_CLOSED_ANGLE - angle;
  }

  return clampi(angle, 0, 180);
}

void setMode(ControlMode newMode) {
  if (controlMode == newMode) {
    return;
  }

  controlMode = newMode;

  if (controlMode == MODE_GLOVE) {
    Serial.println("[MODO] LUVA");
    writeServoAngleSafe(glovePctToAngle(latestGlovePct));
  } else {
    Serial.println("[MODO] COMANDO PS4");
  }
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;

      ControllerProperties properties = ctl->getProperties();

      Serial.printf("Comando ligado no slot %d\n", i);
      Serial.printf("Modelo: %s, VID=0x%04x, PID=0x%04x\n",
                    ctl->getModelName().c_str(),
                    properties.vendor_id,
                    properties.product_id);

      ctl->setColorLED(0, 0, 255);  // azul = modo luva
      return;
    }
  }

  Serial.println("Comando ligado, mas sem slot livre");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Comando desligado do slot %d\n", i);
      break;
    }
  }

  if (controlMode == MODE_CONTROLLER) {
    setMode(MODE_GLOVE);
  }
}

void processGamepad(ControllerPtr ctl) {
  bool crossPressed = ctl->a();       // X / Cross
  bool circlePressed = ctl->b();      // O / Circle
  bool squarePressed = ctl->x();      // Square
  bool trianglePressed = ctl->y();    // Triangle
  bool psPressed = (ctl->miscButtons() & 0x04);

  if (crossPressed && !prevCross) {
    Serial.println("CROSS sem funcao: LED removido");
  }

  if (circlePressed && !prevCircle) {
    writeServoAngleSafe(SERVO_SAFE_ANGLE);
    Serial.println("Garra em posicao segura");
  }

  if (squarePressed && !prevSquare) {
    setMode(MODE_CONTROLLER);
    ctl->setColorLED(0, 255, 0);  // verde = comando
    ctl->playDualRumble(0, 150, 0x40, 0x40);
  }

  if (trianglePressed && !prevTriangle) {
    setMode(MODE_GLOVE);
    ctl->setColorLED(0, 0, 255);  // azul = luva
    ctl->playDualRumble(0, 150, 0x40, 0x40);
  }

  if (psPressed && !prevPS) {
    writeServoAngleSafe(SERVO_SAFE_ANGLE);
    ctl->disconnect();
  }

  prevCross = crossPressed;
  prevCircle = circlePressed;
  prevSquare = squarePressed;
  prevTriangle = trianglePressed;
  prevPS = psPressed;

  if (controlMode == MODE_CONTROLLER) {
    int right = ctl->axisRY();
    int targetAngle = controllerToAngle(ctl);

    writeServoAngleSafe(targetAngle);

    if (millis() - lastServoPrintMs >= 200) {
      lastServoPrintMs = millis();
      Serial.printf("CMD axisRY=%d -> angle=%d\n", right, targetAngle);
    }
  }
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      if (ctl->isGamepad()) {
        processGamepad(ctl);
      } else {
        Serial.println("Comando nao suportado");
      }
    }
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  // Necessario para coexistencia Wi-Fi + Bluetooth no ESP32 com Bluepad32.
  WiFi.setSleep(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.print("MAC da garra para colocar na luva: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Canal ESP-NOW: %u\n", ESPNOW_CHANNEL);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nBOOT OK - GARRA DUAL MODE PS4 + LUVA POSICIONAL");
  Serial.println("BUILD 2026-04-23 FIX WIFI SLEEP");

  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  writeServoAngleSafe(SERVO_SAFE_ANGLE);

  Serial.printf("Servo attach no pino %d, range=%d..%d us\n",
                SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  setupEspNow();

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Descomenta isto so uma vez se precisares de limpar emparelhamentos antigos:
  // BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);

  Serial.printf("Firmware Bluepad32: %s\n", BP32.firmwareVersion());

  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BT Addr da garra: %02X:%02X:%02X:%02X:%02X:%02X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  Serial.println("Arranque em MODO LUVA");
  Serial.println("PS4: TRIANGLE=LUVA | SQUARE=COMANDO | CIRCLE=SEGURA | PS=desligar");
}

void loop() {
  BP32.update();
  processControllers();

  if (controlMode == MODE_GLOVE) {
    if (gloveUpdated) {
      gloveUpdated = false;
      writeServoAngleSafe(glovePctToAngle(latestGlovePct));
    }

    if (glovePacketCount > 0 && millis() - lastGlovePacketMs > GLOVE_TIMEOUT_MS) {
      writeServoAngleSafe(SERVO_SAFE_ANGLE);
    }
  }

  if (millis() - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();

    Serial.printf("DBG mode=%s angle=%d glovePct=%u packets=%lu sinceLast=%lu ms\n",
                  controlMode == MODE_GLOVE ? "LUVA" : "COMANDO",
                  currentAngle,
                  latestGlovePct,
                  glovePacketCount,
                  glovePacketCount == 0 ? 0 : millis() - lastGlovePacketMs);
  }

  delay(5);
}
