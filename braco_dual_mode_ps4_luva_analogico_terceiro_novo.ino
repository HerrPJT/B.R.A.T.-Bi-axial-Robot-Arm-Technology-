#include <WiFi.h>
#include <esp_now.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =========================
// BRACO TRI MODE ESTAVEL
//   - PS4 ligado ao ESP do braco
//   - Luva envia por ESP-NOW
//   - Raspberry Pi envia comandos por Serial USB
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
//   - Circle   -> modo RECONHECIMENTO
//   - R2       -> fecha gripper
//   - L2       -> abre gripper
//   - Cross    -> volta o braco a posicao inicial
//   - Left stick Y  -> servo de baixo
//   - Left stick X  -> servo terceiro
//   - Right stick Y -> servo de cima
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
const int ANGULO_ABERTO_GRIPPER = 180;
const int ANGULO_FECHADO_GRIPPER = 0;

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
const int PASSO_GRIPPER_CMD = 3;
const int PASSO_RECONHECIMENTO = 15;

const float AUTO_L1_CM = 20.0f;
const float AUTO_L2_CM = 28.0f;
const float AUTO_DIST_MIN_CM = 14.0f;
const float AUTO_DIST_MAX_CM = 48.0f;
const unsigned long AUTO_INTERVALO_PASSO_MS = 5000;
const bool AUTO_GARRA_LEGACY_180_ABERTO = true;

const float PITCH_MAX_DEG = 35.0f;
const float ROLL_MAX_DEG  = 35.0f;
const int TERCEIRO_TRAVEL_GLOVE_DEG = 45;
const float PITCH_DEADZONE_DEG = 4.0f;
const float ROLL_DEADZONE_DEG  = 10.0f;
const unsigned long GLOVE_TIMEOUT_MS = 400;
const unsigned long RECOGNITION_TIMEOUT_MS = 1000;

// Se algum eixo estiver invertido, muda aqui para true.
const bool REVERSE_BAIXO_FROM_PITCH = false;
const bool REVERSE_TERCEIRO_FROM_ROLL = true;
const bool REVERSE_GRIPPER_FROM_HAND = false;
const bool REVERSE_CIMA_FROM_FINGERS = true;

Servo servoBaixo;
Servo servoCima;
Servo servoTerceiro;
Servo servoGripper;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

enum ControlMode {
  MODE_GLOVE = 0,
  MODE_CONTROLLER = 1,
  MODE_RECOGNITION = 2
};

enum AutoSequenceState {
  AUTO_PARADO = 0,
  AUTO_EXECUTANDO = 1
};

struct __attribute__((packed)) GlovePacket {
  uint8_t gripperPct;  // polegar
  uint8_t cimaPct;     // indicador + medio + anelar
  int16_t pitchDeg10;  // graus * 10
  int16_t rollDeg10;   // graus * 10
  uint8_t seq;
};

struct AutoCoordinate {
  float x;
  float y;
  float z;
  float garra;
};

const AutoCoordinate AUTO_SEQUENCE[] = {
  {48.0f,   0.0f,  0.0f, 0.0f},
  {48.0f,   0.0f,  0.0f, 20.0f},
  {24.0f,   0.0f, 24.0f, 20.0f},
  {24.0f, -20.0f, 24.0f, 20.0f},
  {24.0f, -20.0f, 10.0f, 20.0f},
  {24.0f, -20.0f, 10.0f, 0.0f},
  { 0.0f,   0.0f, 48.0f,  160.0f}
};

const int AUTO_SEQUENCE_COUNT = sizeof(AUTO_SEQUENCE) / sizeof(AUTO_SEQUENCE[0]);

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
unsigned long lastRecognitionCommandMs = 0;
AutoSequenceState autoSequenceState = AUTO_PARADO;
int autoStepAtual = 0;
int autoStepFinal = 0;
unsigned long autoLastStepMs = 0;

void resetArmPose();
void applyModeColorToController(ControllerPtr ctl);
void stopAutoSequence();

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

int mapAutoGarraToCurrent(float garraLegacy) {
  int angle = (int)roundf(garraLegacy);
  if (AUTO_GARRA_LEGACY_180_ABERTO) {
    angle = 180 - angle;
  }
  return clampi(angle, ANGULO_MIN_GRIPPER, ANGULO_MAX_GRIPPER);
}

bool updateAutoTarget3D(float x, float y, float z, float garraLegacy) {
  float d = sqrtf(x * x + y * y + z * z);
  if (d < AUTO_DIST_MIN_CM || d > AUTO_DIST_MAX_CM) {
    Serial.print("[AUTO] ERRO fora de alcance: ");
    Serial.print(d);
    Serial.println(" cm");
    return false;
  }

  float thetaBaseDeg = atan2f(y, x) * 180.0f / M_PI;
  float r = sqrtf(x * x + y * y);

  float cosTheta2 = (r * r + z * z - AUTO_L1_CM * AUTO_L1_CM - AUTO_L2_CM * AUTO_L2_CM) /
                    (2.0f * AUTO_L1_CM * AUTO_L2_CM);
  cosTheta2 = clampf(cosTheta2, -1.0f, 1.0f);

  float sinTheta2 = -sqrtf(1.0f - cosTheta2 * cosTheta2);
  float theta2 = atan2f(sinTheta2, cosTheta2);

  float k1 = AUTO_L1_CM + AUTO_L2_CM * cosTheta2;
  float k2 = AUTO_L2_CM * sinTheta2;
  float theta1 = atan2f(z, r) - atan2f(k2, k1);

  float theta1Deg = theta1 * 180.0f / M_PI;
  float theta2Deg = theta2 * 180.0f / M_PI;

  alvoBaixo = clampi((int)roundf(180.0f - theta1Deg), ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  alvoCima = clampi((int)roundf(90.0f + theta2Deg), ANGULO_MIN_CIMA, ANGULO_MAX_CIMA);
  alvoAnguloTerceiro = clampi((int)roundf(90.0f + thetaBaseDeg),
                              ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
  alvoGripper = mapAutoGarraToCurrent(garraLegacy);
  gripperAberto = alvoGripper <= ((ANGULO_ABERTO_GRIPPER + ANGULO_FECHADO_GRIPPER) / 2);

  Serial.printf("[AUTO] alvo base=%d baixo=%d cima=%d gripper=%d\n",
                alvoAnguloTerceiro, alvoBaixo, alvoCima, alvoGripper);
  return true;
}

void stopAutoSequence() {
  if (autoSequenceState == AUTO_EXECUTANDO) {
    Serial.println("[AUTO] sequencia parada");
  }
  autoSequenceState = AUTO_PARADO;
}

void executeAutoStep(int step) {
  if (step < 0 || step >= AUTO_SEQUENCE_COUNT) {
    stopAutoSequence();
    return;
  }

  const AutoCoordinate &coord = AUTO_SEQUENCE[step];
  Serial.printf("[AUTO] passo %d/%d -> x=%.1f y=%.1f z=%.1f garra=%.1f\n",
                step + 1, autoStepFinal + 1, coord.x, coord.y, coord.z, coord.garra);
  if (!updateAutoTarget3D(coord.x, coord.y, coord.z, coord.garra)) {
    stopAutoSequence();
  }
}

void startAutoSequence(int firstStep, int finalStep) {
  firstStep = clampi(firstStep, 0, AUTO_SEQUENCE_COUNT - 1);
  finalStep = clampi(finalStep, firstStep, AUTO_SEQUENCE_COUNT - 1);

  autoStepAtual = firstStep;
  autoStepFinal = finalStep;
  autoSequenceState = AUTO_EXECUTANDO;
  executeAutoStep(autoStepAtual);
  autoLastStepMs = millis();
}

void updateAutoSequence() {
  if (controlMode != MODE_RECOGNITION || autoSequenceState != AUTO_EXECUTANDO) return;
  if (millis() - autoLastStepMs < AUTO_INTERVALO_PASSO_MS) return;

  autoStepAtual++;
  if (autoStepAtual <= autoStepFinal) {
    executeAutoStep(autoStepAtual);
    autoLastStepMs = millis();
  } else {
    autoSequenceState = AUTO_PARADO;
    Serial.println("[AUTO] sequencia concluida");
  }
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
      applyModeColorToController(ctl);
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

void applyModeColorToController(ControllerPtr ctl) {
  if (!ctl || !ctl->isConnected()) return;

  if (controlMode == MODE_GLOVE) {
    ctl->setColorLED(0, 0, 255);       // azul = luva
  } else if (controlMode == MODE_CONTROLLER) {
    ctl->setColorLED(0, 255, 0);       // verde = comando
  } else {
    ctl->setColorLED(255, 80, 0);      // laranja = reconhecimento
  }
}

void applyModeColorToAllControllers() {
  for (auto ctl : myControllers) {
    applyModeColorToController(ctl);
  }
}

void sendModeNumberToRaspberry() {
  // Linha simples para o Raspberry filtrar: 0=luva, 1=comando, 2=reconhecimento.
  Serial.println((int)controlMode);
}

void setMode(ControlMode newMode) {
  if (controlMode == newMode) return;
  if (newMode != MODE_RECOGNITION) {
    stopAutoSequence();
  }
  controlMode = newMode;
  if (controlMode == MODE_RECOGNITION) {
    while (Serial.available() > 0) {
      Serial.read();
    }
    lastRecognitionCommandMs = millis();
  }
  const char* modeName =
      controlMode == MODE_GLOVE ? "LUVA" :
      controlMode == MODE_CONTROLLER ? "COMANDO" : "RECONHECIMENTO";
  Serial.printf("[MODO] %s\n", modeName);
  sendModeNumberToRaspberry();
  applyModeColorToAllControllers();
}

void stopArmAtCurrentPosition() {
  alvoBaixo = clampi((int)roundf(posAtualBaixo), ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  alvoCima = clampi((int)roundf(posAtualCima), ANGULO_MIN_CIMA, ANGULO_MAX_CIMA);
  alvoAnguloTerceiro = clampi((int)roundf(anguloAtualTerceiro),
                              ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
  alvoGripper = clampi((int)roundf(posAtualGripper), ANGULO_MIN_GRIPPER, ANGULO_MAX_GRIPPER);
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
                                      ANGULO_CENTRO_TERCEIRO, TERCEIRO_TRAVEL_GLOVE_DEG,
                                      REVERSE_TERCEIRO_FROM_ROLL,
                                      ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
}

void updateTargetsFromController(ControllerPtr ctl) {
  int leftX = ctl->axisX();
  int leftY = ctl->axisY();
  int rightY = ctl->axisRY();

  if (abs(leftX) < DEADZONE_CMD) leftX = 0;
  if (abs(leftY) < DEADZONE_CMD) leftY = 0;
  if (abs(rightY) < DEADZONE_CMD) rightY = 0;

  // Controlo incremental: ao largar os analogicos conserva a posicao.
  if (leftX != 0) {
    int step = map(abs(leftX), DEADZONE_CMD, 512, PASSO_CMD_MIN, PASSO_TERCEIRO_CMD);
    alvoAnguloTerceiro += (leftX > 0) ? -step : step;
    alvoAnguloTerceiro = clampi(alvoAnguloTerceiro, ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
  }
  if (leftY != 0) {
    int step = map(abs(leftY), DEADZONE_CMD, 512, PASSO_CMD_MIN, PASSO_CMD_MAX);
    alvoBaixo += (leftY > 0) ? -step : step;
    alvoBaixo = clampi(alvoBaixo, ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  }
  if (rightY != 0) {
    int step = map(abs(rightY), DEADZONE_CMD, 512, PASSO_CMD_MIN, PASSO_CMD_MAX);
    alvoCima += (rightY > 0) ? step : -step;
    alvoCima = clampi(alvoCima, ANGULO_MIN_CIMA, ANGULO_MAX_CIMA);
  }

  bool openGripper = ctl->brake();
  bool closeGripper = ctl->throttle();
  if (openGripper && !closeGripper) {
    alvoGripper -= PASSO_GRIPPER_CMD;
  } else if (closeGripper && !openGripper) {
    alvoGripper += PASSO_GRIPPER_CMD;
  }
  alvoGripper = clampi(alvoGripper, ANGULO_MIN_GRIPPER, ANGULO_MAX_GRIPPER);
  gripperAberto = alvoGripper <= ((ANGULO_ABERTO_GRIPPER + ANGULO_FECHADO_GRIPPER) / 2);
}

void processRecognitionCommand(String comando) {
  comando.trim();
  if (comando.length() == 0) return;

  lastRecognitionCommandMs = millis();
  Serial.print("[RECONHECIMENTO] ");
  Serial.println(comando);

  if (comando.equalsIgnoreCase("On")) {
    Serial.println("[AUTO] comando ON: sequencia completa");
    startAutoSequence(0, AUTO_SEQUENCE_COUNT - 1);
    return;
  } else if (comando.equalsIgnoreCase("Grab")) {
    Serial.println("[AUTO] comando GRAB: aproximar e agarrar");
    startAutoSequence(0, 3);
    return;
  } else if (comando.equalsIgnoreCase("Go")) {
    Serial.println("[AUTO] comando GO: transportar/largar");
    startAutoSequence(4, AUTO_SEQUENCE_COUNT - 1);
    return;
  } else if (comando.equalsIgnoreCase("Off")) {
    Serial.println("[AUTO] comando OFF: parar e voltar a posicao confortavel");
    stopAutoSequence();
    updateAutoTarget3D(0.0f, 0.0f, 48.0f, 20.0f);
    return;
  }

  stopAutoSequence();

  if (comando.equalsIgnoreCase("Forward") || comando.equalsIgnoreCase("Foward")) {
    alvoBaixo = clampi(alvoBaixo + PASSO_RECONHECIMENTO, ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  } else if (comando.equalsIgnoreCase("Back")) {
    alvoBaixo = clampi(alvoBaixo - PASSO_RECONHECIMENTO, ANGULO_MIN_BAIXO, ANGULO_MAX_BAIXO);
  } else if (comando.equalsIgnoreCase("Left")) {
    alvoAnguloTerceiro = clampi(alvoAnguloTerceiro + PASSO_RECONHECIMENTO,
                                ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
  } else if (comando.equalsIgnoreCase("Right")) {
    alvoAnguloTerceiro = clampi(alvoAnguloTerceiro - PASSO_RECONHECIMENTO,
                                ANGULO_MIN_TERCEIRO, ANGULO_MAX_TERCEIRO);
  } else if (comando.equalsIgnoreCase("Open")) {
    alvoGripper = ANGULO_ABERTO_GRIPPER;
    gripperAberto = true;
  } else if (comando.equalsIgnoreCase("Close")) {
    alvoGripper = ANGULO_FECHADO_GRIPPER;
    gripperAberto = false;
  } else if (comando.equalsIgnoreCase("Stop") || comando.equalsIgnoreCase("None")) {
    stopArmAtCurrentPosition();
  } else if (comando.equalsIgnoreCase("Abort")) {
    resetArmPose();
  } else {
    Serial.println("Comando de reconhecimento desconhecido");
  }
}

void processRecognitionSerial() {
  while (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    if (controlMode == MODE_RECOGNITION) {
      processRecognitionCommand(comando);
    }
  }
}

void applyRecognitionFailsafe() {
  if (controlMode != MODE_RECOGNITION) return;
  if (autoSequenceState == AUTO_EXECUTANDO) return;
  if (millis() - lastRecognitionCommandMs <= RECOGNITION_TIMEOUT_MS) return;
  stopArmAtCurrentPosition();
}

void resetArmPose() {
  stopAutoSequence();
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
    setMode(MODE_RECOGNITION);
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
  Serial.setTimeout(10);
  delay(900);
  Serial.println("\nBOOT OK - BRACO TRI MODE PS4 + LUVA + RECONHECIMENTO SERIAL");

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

  Serial.println("Triangle -> LUVA | Square -> COMANDO | Circle -> RECONHECIMENTO | Cross -> posicao inicial");
  Serial.println("Analogico esquerdo: Y=servo baixo | X=servo terceiro");
  Serial.println("R2 fecha gripper aos poucos | L2 abre gripper aos poucos");
  Serial.println("Serial reconhecimento: Forward/Foward, Back, Left, Right, Open, Close, Stop, Abort, None");
  Serial.println("Serial auto: On, Grab, Go, Off");
  sendModeNumberToRaspberry();
}

void loop() {
  processRecognitionSerial();

  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  if (controlMode == MODE_GLOVE) {
    updateTargetsFromGlove();
  }

  updateAutoSequence();
  applyRecognitionFailsafe();
  applyServoOutputs();

  if (millis() - lastDebugMs >= 300) {
    float pitchDeg = latestGlove.pitchDeg10 / 10.0f;
    float rollDeg = latestGlove.rollDeg10 / 10.0f;
    const char* modeName =
        controlMode == MODE_GLOVE ? "LUVA" :
        controlMode == MODE_CONTROLLER ? "COMANDO" : "RECONHECIMENTO";
    Serial.printf("DBG mode=%s gripPct=%u cimaPct=%u pitch=%.1f roll=%.1f packets=%lu | baixo=%d cima=%d terceiro=%d gripper=%d\n",
                  modeName,
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
