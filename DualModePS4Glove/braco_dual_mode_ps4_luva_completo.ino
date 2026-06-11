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
//   - Polegar -> gripper
//   - Indicador + medio + anelar -> servo de cima
//   - Inclinar para cima/baixo (pitch) -> servo de baixo
//   - Inclinar esquerda/direita (roll) -> servo terceiro
//
// Mapeamento PS4:
//   - Triangle -> modo LUVA
//   - Square   -> modo COMANDO
//   - Circle   -> abre/fecha gripper
//   - Cross    -> volta o braco a posicao inicial
//   - Left stick Y  -> servo de baixo
//   - Right stick Y -> servo de cima
//   - D-pad left/right -> servo terceiro
//   - PS -> desliga comando
// =========================

#define LED_PIN 2

const int PIN_SERVO_BAIXO    = 17;
const int PIN_SERVO_CIMA     = 18;
const int PIN_SERVO_TERCEIRO = 19;
const int PIN_SERVO_GRIPPER  = 21;

const int ANGULO_CENTRO_BAIXO   = 90;
const int ANGULO_CENTRO_CIMA    = 90;
const int ANGULO_CENTRO_TERCEIRO = 90;
const int ANGULO_ABERTO_GRIPPER = 0;
const int ANGULO_FECHADO_GRIPPER = 180;

const int ANGULO_MIN_BAIXO   = 10;
const int ANGULO_MAX_BAIXO   = 170;
const int ANGULO_ABERTO_CIMA_GLOVE = 20;
const int ANGULO_FECHADO_CIMA_GLOVE = 160;
const int ANGULO_MIN_CIMA    = 10;
const int ANGULO_MAX_CIMA    = 170;
const int ANGULO_MIN_TERCEIRO = 10;
const int ANGULO_MAX_TERCEIRO = 170;
const int ANGULO_MIN_GRIPPER = 0;
const int ANGULO_MAX_GRIPPER = 180;

const float SUAVIZACAO_POS = 0.025f;
const float SUAVIZACAO_POS_GLOVE = 0.04f;
const float SUAVIZACAO_GRIPPER = 0.20f;
const float SUAVIZACAO_TERCEIRO_CMD = 0.025f;
const float SUAVIZACAO_TERCEIRO_GLOVE = 0.04f;

const int DEADZONE_CMD = 50;
const int PASSO_CMD_MIN = 1;
const int PASSO_CMD_MAX = 3;
const int PASSO_TERCEIRO_CMD = 2;

const float PITCH_MAX_DEG = 35.0f;
const float ROLL_MAX_DEG  = 35.0f;
const float PITCH_DEADZONE_DEG = 4.0f;
const float ROLL_DEADZONE_DEG  = 8.0f;
const unsigned long GLOVE_TIMEOUT_MS = 400;

// Se algum eixo estiver invertido, muda aqui para true.
const bool REVERSE_BAIXO_FROM_PITCH = false;
const bool REVERSE_TERCEIRO_FROM_ROLL = true;
const bool REVERSE_GRIPPER_FROM_HAND = true;
const bool REVERSE_CIMA_FROM_FINGERS = true;

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
  uint8_t gripperPct;  // polegar
  uint8_t cimaPct;     // indicador + medio + anelar
  int16_t pitchDeg10;  // graus * 10
  int16_t rollDeg10;   // graus * 10
  uint8_t seq;
};

volatile GlovePacket latestGlove = {0, 0, 0, 0, 0};
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
float anguloAtualTerceiro = ANGULO_CENTRO_TERCEIRO;

int alvoBaixo = ANGULO_CENTRO_BAIXO;
int alvoCima = ANGULO_CENTRO_CIMA;
int alvoGripper = ANGULO_ABERTO_GRIPPER;
int alvoAnguloTerceiro = ANGULO_CENTRO_TERCEIRO;

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

int mapAxisToAngle(float value, float maxValue, float deadzone,
                   int centerAngle, int travelAngle, bool reverse,
                   int minAngle, int maxAngle) {
  value = applyDeadzone(value, deadzone);
  float norm = clampf(value / maxValue, -1.0f, 1.0f);
  if (reverse) norm = -norm;
  int angle = (int)roundf(centerAngle + norm * travelAngle);
  return clampi(angle, minAngle, maxAngle);
}

void applyServoOutputs() {
  float suavizacaoPosAtual = (controlMode == MODE_GLOVE) ? SUAVIZACAO_POS_GLOVE : SUAVIZACAO_POS;
  float suavizacaoTerceiroAtual = (controlMode == MODE_GLOVE) ? SUAVIZACAO_TERCEIRO_GLOVE : SUAVIZACAO_TERCEIRO_CMD;

  posAtualBaixo += (alvoBaixo - posAtualBaixo) * suavizacaoPosAtual;
  posAtualCima += (alvoCima - posAtualCima) * suavizacaoPosAtual;
  posAtualGripper += (alvoGripper - posAtualGripper) * SUAVIZACAO_GRIPPER;
  anguloAtualTerceiro += (alvoAnguloTerceiro - anguloAtualTerceiro) * suavizacaoTerceiroAtual;

  servoBaixo.write((int)roundf(posAtualBaixo));
  servoCima.write((int)roundf(posAtualCima));
  servoGripper.write((int)roundf(posAtualGripper));
  servoTerceiro.write((int)roundf(anguloAtualTerceiro));
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
}

void setMode(ControlMode newMode) {
  if (controlMode == newMode) return;
  controlMode = newMode;
  Serial.printf("[MODO] %s\n", controlMode == MODE_GLOVE ? "LUVA" : "COMANDO");
}

void updateTargetsFromGlove() {
  unsigned long age = millis() - lastGlovePacketMs;
  if (!glovePacketArrived || age > GLOVE_TIMEOUT_MS) {
    return;
  }

  uint8_t gripperPct = latestGlove.gripperPct;
  uint8_t cimaPct = latestGlove.cimaPct;
  float pitchDeg = latestGlove.pitchDeg10 / 10.0f;
  float rollDeg = latestGlove.rollDeg10 / 10.0f;

  alvoGripper = mapPctToAngle(gripperPct,
                              ANGULO_ABERTO_GRIPPER,
                              ANGULO_FECHADO_GRIPPER,
                              REVERSE_GRIPPER_FROM_HAND);
  alvoCima = mapPctToAngle(cimaPct,
                           ANGULO_ABERTO_CIMA_GLOVE,
                           ANGULO_FECHADO_CIMA_GLOVE,
                           REVERSE_CIMA_FROM_FINGERS);
  alvoBaixo = mapAxisToAngle(pitchDeg, PITCH_MAX_DEG, PITCH_DEADZONE_DEG,
                             ANGULO_CENTRO_BAIXO, 75,
                             REVERSE_BAIXO_FROM_PITCH,
                             ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  alvoAnguloTerceiro = mapAxisToAngle(rollDeg, ROLL_MAX_DEG, ROLL_DEADZONE_DEG,
                                      ANGULO_CENTRO_TERCEIRO, 75,
                                      REVERSE_TERCEIRO_FROM_ROLL,
                                      ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
}

void updateTargetsFromController(ControllerPtr ctl) {
  int leftY = ctl->axisY();
  int rightY = ctl->axisRY();

  if (abs(leftY) < DEADZONE_CMD) leftY = 0;
  if (abs(rightY) < DEADZONE_CMD) rightY = 0;

  // Controlo incremental: ao largar os analogicos conserva a posicao.
  if (leftY != 0) {
    int step = map(abs(leftY), DEADZONE_CMD, 512, PASSO_CMD_MIN, PASSO_CMD_MAX);
    alvoBaixo += (leftY > 0) ? step : -step;
    alvoBaixo = clampi(alvoBaixo, ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  }
  if (rightY != 0) {
    int step = map(abs(rightY), DEADZONE_CMD, 512, PASSO_CMD_MIN, PASSO_CMD_MAX);
    alvoCima += (rightY > 0) ? step : -step;
    alvoCima = clampi(alvoCima, ANGULO_MIN_CIMA, ANGULO_MAX_CIMA);
  }

  uint8_t dpad = ctl->dpad();
  if (dpad & 0x02) {
    alvoAnguloTerceiro -= PASSO_TERCEIRO_CMD;
  } else if (dpad & 0x01) {
    alvoAnguloTerceiro += PASSO_TERCEIRO_CMD;
  }
  alvoAnguloTerceiro = clampi(alvoAnguloTerceiro, ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
}

void resetArmPose() {
  alvoBaixo = ANGULO_CENTRO_BAIXO;
  alvoCima = ANGULO_CENTRO_CIMA;
  alvoGripper = ANGULO_ABERTO_GRIPPER;
  alvoAnguloTerceiro = ANGULO_CENTRO_TERCEIRO;
  gripperAberto = true;
  Serial.println("Braco a voltar a posicao inicial");
}

void processGamepad(ControllerPtr ctl) {
  bool crossPressed = ctl->a();
  bool circlePressed = ctl->b();
  bool squarePressed = ctl->x();
  bool trianglePressed = ctl->y();
  bool psPressed = (ctl->miscButtons() & 0x04);

  if (crossPressed && !prevCross) {
    resetArmPose();
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
  servoTerceiro.attach(PIN_SERVO_TERCEIRO, 500, 2500);
  servoGripper.attach(PIN_SERVO_GRIPPER, 400, 2200);

  servoBaixo.write(ANGULO_CENTRO_BAIXO);
  servoCima.write(ANGULO_CENTRO_CIMA);
  servoTerceiro.write(ANGULO_CENTRO_TERCEIRO);
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
  }

  applyServoOutputs();

  if (millis() - lastDebugMs >= 300) {
    float pitchDeg = latestGlove.pitchDeg10 / 10.0f;
    float rollDeg = latestGlove.rollDeg10 / 10.0f;
    Serial.printf("DBG mode=%s gripPct=%u cimaPct=%u pitch=%.1f roll=%.1f packets=%lu | baixo=%d cima=%d terceiro=%d gripper=%d\n",
                  controlMode == MODE_GLOVE ? "LUVA" : "COMANDO",
                  latestGlove.gripperPct,
                  latestGlove.cimaPct,
                  pitchDeg,
                  rollDeg,
                  glovePacketCount,
                  alvoBaixo,
                  alvoCima,
                  alvoAnguloTerceiro,
                  alvoGripper);
    lastDebugMs = millis();
  }

  delay(10);
}
