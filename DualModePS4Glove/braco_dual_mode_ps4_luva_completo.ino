#include <WiFi.h>
#include <esp_now.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

// =========================
// BRACO DUAL MODE
//   - PS4 ligado ao ESP do braco
//   - Luva envia por ESP-NOW
//
// Mapeamento da luva:
//   - Fechar/abrir mao -> gripper
//   - Inclinar para cima/baixo (pitch) -> servo de baixo + servo de cima
//   - Inclinar esquerda/direita (roll) -> servo terceiro 360
//
// Mapeamento PS4:
//   - Triangle -> modo LUVA
//   - Square   -> modo COMANDO
//   - Circle   -> abre/fecha gripper
//   - Left stick Y  -> servo de baixo
//   - Right stick Y -> servo de cima
//   - D-pad left/right -> servo terceiro 360
//   - PS -> desliga comando e para o servo 360
// =========================

#define LED_PIN 2

const int PIN_SERVO_BAIXO    = 17;
const int PIN_SERVO_CIMA     = 18;
const int PIN_SERVO_TERCEIRO = 19;  // servo 360 continuo
const int PIN_SERVO_GRIPPER  = 21;

const int ANGULO_CENTRO_BAIXO   = 90;
const int ANGULO_CENTRO_CIMA    = 90;
const int ANGULO_ABERTO_GRIPPER = 0;
const int ANGULO_FECHADO_GRIPPER = 180;

const int ANGULO_MIN_BAIXO   = 10;
const int ANGULO_MAX_BAIXO   = 170;
const int ANGULO_MIN_CIMA    = 10;
const int ANGULO_MAX_CIMA    = 170;
const int ANGULO_MIN_GRIPPER = 0;
const int ANGULO_MAX_GRIPPER = 180;

const int PULSE_STOP_TERCEIRO = 1500;
const int PULSE_MIN_TERCEIRO  = 500;
const int PULSE_MAX_TERCEIRO  = 2500;

const float SUAVIZACAO_POS = 0.025f;
const float SUAVIZACAO_GRIPPER = 0.06f;
const float SUAVIZACAO_TERCEIRO = 0.18f;

const int DEADZONE_CMD = 50;
const int RANGE_CMD_TERCEIRO = 240;

const float PITCH_MAX_DEG = 35.0f;
const float ROLL_MAX_DEG  = 35.0f;
const float PITCH_DEADZONE_DEG = 4.0f;
const float ROLL_DEADZONE_DEG  = 5.0f;
const int RANGE_TERCEIRO_GLOVE = 220;
const unsigned long GLOVE_TIMEOUT_MS = 400;

// Se algum eixo estiver invertido, muda aqui para true.
const bool REVERSE_BAIXO_FROM_PITCH = false;
const bool REVERSE_CIMA_FROM_PITCH  = true;
const bool REVERSE_TERCEIRO_FROM_ROLL = false;
const bool REVERSE_GRIPPER_FROM_HAND = false;

Servo servoBaixo;
Servo servoCima;
Servo servoTerceiro;
Servo servoGripper;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

enum ControlMode {
  MODE_GLOVE = 0,
  MODE_CONTROLLER = 1
};

struct __attribute__((packed)) GlovePacket {
  uint8_t avgPct;      // 0 aberto, 100 fechado
  int16_t pitchDeg10;  // graus * 10
  int16_t rollDeg10;   // graus * 10
  uint8_t seq;
};

volatile GlovePacket latestGlove = {0, 0, 0, 0};
volatile bool glovePacketArrived = false;
volatile unsigned long lastGlovePacketMs = 0;
volatile unsigned long glovePacketCount = 0;

ControlMode controlMode = MODE_GLOVE;
bool ledState = false;
bool gripperAberto = true;
bool prevCross = false;
bool prevCircle = false;
bool prevSquare = false;
bool prevTriangle = false;
bool prevPS = false;

float posAtualBaixo = ANGULO_CENTRO_BAIXO;
float posAtualCima = ANGULO_CENTRO_CIMA;
float posAtualGripper = ANGULO_ABERTO_GRIPPER;
float pulseAtualTerceiro = PULSE_STOP_TERCEIRO;

int alvoBaixo = ANGULO_CENTRO_BAIXO;
int alvoCima = ANGULO_CENTRO_CIMA;
int alvoGripper = ANGULO_ABERTO_GRIPPER;
int alvoPulseTerceiro = PULSE_STOP_TERCEIRO;

unsigned long lastDebugMs = 0;

int clampi(int x, int a, int b) {
  return (x < a) ? a : (x > b ? b : x);
}

float clampf(float x, float a, float b) {
  return (x < a) ? a : (x > b ? b : x);
}

float applyDeadzone(float value, float deadzone) {
  return (fabs(value) < deadzone) ? 0.0f : value;
}

int mapPctToAngle(uint8_t pct, int openAngle, int closedAngle, bool reverse) {
  int angle = map((int)pct, 0, 100, openAngle, closedAngle);
  if (reverse) {
    angle = map((int)pct, 0, 100, closedAngle, openAngle);
  }
  return angle;
}

int mapPitchToAngle(float pitchDeg, int centerAngle, int travelAngle, bool reverse, int minAngle, int maxAngle) {
  pitchDeg = applyDeadzone(pitchDeg, PITCH_DEADZONE_DEG);
  float norm = clampf(pitchDeg / PITCH_MAX_DEG, -1.0f, 1.0f);
  if (reverse) norm = -norm;
  int angle = (int)roundf(centerAngle + norm * travelAngle);
  return clampi(angle, minAngle, maxAngle);
}

int mapRollToPulse(float rollDeg, bool reverse) {
  rollDeg = applyDeadzone(rollDeg, ROLL_DEADZONE_DEG);
  float norm = clampf(rollDeg / ROLL_MAX_DEG, -1.0f, 1.0f);
  if (reverse) norm = -norm;
  int pulse = (int)roundf(PULSE_STOP_TERCEIRO + norm * RANGE_TERCEIRO_GLOVE);
  return clampi(pulse, PULSE_MIN_TERCEIRO, PULSE_MAX_TERCEIRO);
}

void applyServoOutputs() {
  posAtualBaixo += (alvoBaixo - posAtualBaixo) * SUAVIZACAO_POS;
  posAtualCima += (alvoCima - posAtualCima) * SUAVIZACAO_POS;
  posAtualGripper += (alvoGripper - posAtualGripper) * SUAVIZACAO_GRIPPER;
  pulseAtualTerceiro += (alvoPulseTerceiro - pulseAtualTerceiro) * SUAVIZACAO_TERCEIRO;

  servoBaixo.write((int)roundf(posAtualBaixo));
  servoCima.write((int)roundf(posAtualCima));
  servoGripper.write((int)roundf(posAtualGripper));
  servoTerceiro.writeMicroseconds((int)roundf(pulseAtualTerceiro));
}

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(GlovePacket)) {
    memcpy((void *)&latestGlove, incomingData, sizeof(latestGlove));
    glovePacketArrived = true;
    lastGlovePacketMs = millis();
    glovePacketCount++;
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  // Necessario quando WiFi + Bluetooth coexistem com Bluepad32.
  WiFi.setSleep(true);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.print("MAC Wi-Fi do braco para colocar na luva: ");
  Serial.println(WiFi.macAddress());
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
      break;
    }
  }
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
    alvoPulseTerceiro = PULSE_STOP_TERCEIRO;
  }
}

void setMode(ControlMode newMode) {
  if (controlMode == newMode) return;
  controlMode = newMode;
  Serial.printf("[MODO] %s\n", controlMode == MODE_GLOVE ? "LUVA" : "COMANDO");
  alvoPulseTerceiro = PULSE_STOP_TERCEIRO;
}

void updateTargetsFromGlove() {
  unsigned long age = millis() - lastGlovePacketMs;
  if (!glovePacketArrived || age > GLOVE_TIMEOUT_MS) {
    alvoPulseTerceiro = PULSE_STOP_TERCEIRO;
    return;
  }

  uint8_t avgPct = latestGlove.avgPct;
  float pitchDeg = latestGlove.pitchDeg10 / 10.0f;
  float rollDeg = latestGlove.rollDeg10 / 10.0f;

  alvoGripper = mapPctToAngle(avgPct, ANGULO_ABERTO_GRIPPER, ANGULO_FECHADO_GRIPPER, REVERSE_GRIPPER_FROM_HAND);
  alvoBaixo = mapPitchToAngle(pitchDeg, ANGULO_CENTRO_BAIXO, 55, REVERSE_BAIXO_FROM_PITCH, ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  alvoCima = mapPitchToAngle(pitchDeg, ANGULO_CENTRO_CIMA, 45, REVERSE_CIMA_FROM_PITCH, ANGULO_MIN_CIMA, ANGULO_MAX_CIMA);
  alvoPulseTerceiro = mapRollToPulse(rollDeg, REVERSE_TERCEIRO_FROM_ROLL);
}

void updateTargetsFromController(ControllerPtr ctl) {
  int leftY = ctl->axisY();
  int rightY = ctl->axisRY();

  if (abs(leftY) < DEADZONE_CMD) leftY = 0;
  if (abs(rightY) < DEADZONE_CMD) rightY = 0;

  alvoBaixo = (leftY == 0) ? ANGULO_CENTRO_BAIXO : map(leftY, -512, 512, ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  alvoCima = (rightY == 0) ? ANGULO_CENTRO_CIMA : map(rightY, -512, 512, ANGULO_MIN_CIMA, ANGULO_MAX_CIMA);

  uint8_t dpad = ctl->dpad();
  if (dpad & 0x02) {
    alvoPulseTerceiro = PULSE_STOP_TERCEIRO - RANGE_CMD_TERCEIRO;
  } else if (dpad & 0x01) {
    alvoPulseTerceiro = PULSE_STOP_TERCEIRO + RANGE_CMD_TERCEIRO;
  } else {
    alvoPulseTerceiro = PULSE_STOP_TERCEIRO;
  }
}

void processGamepad(ControllerPtr ctl) {
  bool crossPressed = ctl->a();
  bool circlePressed = ctl->b();
  bool squarePressed = ctl->x();
  bool trianglePressed = ctl->y();
  bool psPressed = (ctl->miscButtons() & 0x04);

  if (crossPressed && !prevCross) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    Serial.printf("LED %s\n", ledState ? "ON" : "OFF");
  }

  if (circlePressed && !prevCircle) {
    gripperAberto = !gripperAberto;
    alvoGripper = gripperAberto ? ANGULO_ABERTO_GRIPPER : ANGULO_FECHADO_GRIPPER;
    Serial.printf("Gripper %s\n", gripperAberto ? "ABERTO" : "FECHADO");
  }

  if (squarePressed && !prevSquare) {
    setMode(MODE_CONTROLLER);
  }

  if (trianglePressed && !prevTriangle) {
    setMode(MODE_GLOVE);
  }

  if (psPressed && !prevPS) {
    alvoPulseTerceiro = PULSE_STOP_TERCEIRO;
    ctl->disconnect();
  }

  prevCross = crossPressed;
  prevCircle = circlePressed;
  prevSquare = squarePressed;
  prevTriangle = trianglePressed;
  prevPS = psPressed;

  if (controlMode == MODE_CONTROLLER) {
    updateTargetsFromController(ctl);
  }
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
      processGamepad(ctl);
    }
  }
}

ControllerPtr getAnyConnectedGamepad() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->isGamepad()) {
      return ctl;
    }
  }
  return nullptr;
}

void setup() {
  Serial.begin(115200);
  delay(900);
  Serial.println("\nBOOT OK - BRACO DUAL MODE PS4 + LUVA COMPLETO");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBaixo.attach(PIN_SERVO_BAIXO, 500, 2500);
  servoCima.attach(PIN_SERVO_CIMA, 500, 2500);
  servoTerceiro.attach(PIN_SERVO_TERCEIRO, PULSE_MIN_TERCEIRO, PULSE_MAX_TERCEIRO);
  servoGripper.attach(PIN_SERVO_GRIPPER, 400, 2200);

  servoBaixo.write(ANGULO_CENTRO_BAIXO);
  servoCima.write(ANGULO_CENTRO_CIMA);
  servoTerceiro.writeMicroseconds(PULSE_STOP_TERCEIRO);
  servoGripper.write(ANGULO_ABERTO_GRIPPER);
  delay(700);

  setupEspNow();

  Serial.printf("Firmware Bluepad32: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BT Addr do braco: %02X:%02X:%02X:%02X:%02X:%02X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();  // usar so uma vez se precisares de limpar emparelhamentos
  BP32.enableVirtualDevice(false);

  Serial.println("Triangle -> modo LUVA | Square -> modo COMANDO");
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  if (controlMode == MODE_GLOVE) {
    updateTargetsFromGlove();
  } else if (getAnyConnectedGamepad() == nullptr) {
    alvoPulseTerceiro = PULSE_STOP_TERCEIRO;
  }

  applyServoOutputs();

  if (millis() - lastDebugMs >= 300) {
    float pitchDeg = latestGlove.pitchDeg10 / 10.0f;
    float rollDeg = latestGlove.rollDeg10 / 10.0f;
    Serial.printf("DBG mode=%s gripPct=%u pitch=%.1f roll=%.1f packets=%lu | baixo=%d cima=%d terceiro=%d gripper=%d\n",
                  controlMode == MODE_GLOVE ? "LUVA" : "COMANDO",
                  latestGlove.avgPct,
                  pitchDeg,
                  rollDeg,
                  glovePacketCount,
                  alvoBaixo,
                  alvoCima,
                  alvoPulseTerceiro,
                  alvoGripper);
    lastDebugMs = millis();
  }

  delay(10);
}
