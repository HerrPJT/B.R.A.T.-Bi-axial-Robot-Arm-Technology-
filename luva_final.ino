#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Ordem: Thumb, Index, Middle, Ring, Pinky
const int pins[5] = {34, 35, 32, 33, 25};

// Valores fixos (calibrados)
const int OPEN_V[5]   = {1648, 1333, 1520, 1414, 1344};
const int CLOSED_V[5] = {1079,  714,  789,  753,  714};

int clampi(int x, int a, int b) { return x < a ? a : (x > b ? b : x); }

void readOnce(int out[5]) {
  for (int i = 0; i < 5; i++) out[i] = analogRead(pins[i]);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nBOOT OK (fixed calibration)");

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

  Serial.println("=== LUVA (DEDOS + IMU) ===");
  Serial.println("FLEX% (0=open,100=closed) | pitch/roll | gyro | gesture");
}

void loop() {
  // ---- dedos raw -> % ----
  int raw[5], pct[5];
  readOnce(raw);

  for (int i = 0; i < 5; i++) {
    int denom = (OPEN_V[i] - CLOSED_V[i]);
    int p = 0;
    if (abs(denom) > 10) {
      p = (int)(100.0f * (OPEN_V[i] - raw[i]) / (float)denom);
      p = clampi(p, 0, 100);
    }
    pct[i] = p;
  }

  // ---- IMU ----
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float roll  = atan2(ay, az) * 180.0 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // ---- gestos (aberta/fechada/pinça) ----
  bool allOpen = true, allFist = true;
  for (int i = 0; i < 5; i++) {
    if (pct[i] > 25) allOpen = false;
    if (pct[i] < 75) allFist = false;
  }
  bool pinch = (pct[0] > 60 && pct[1] > 60 && pct[2] < 40 && pct[3] < 40 && pct[4] < 40);

  const char* gesture = "none";
  if (allOpen) gesture = "OPEN_HAND";
  else if (allFist) gesture = "FIST";
  else if (pinch) gesture = "PINCH";

  // ---- print completo ----
  Serial.print("FLEX% ");
  for (int i = 0; i < 5; i++) {
    Serial.print(pct[i]);
    if (i < 4) Serial.print(",");
  }

  Serial.print(" | pitch=");
  Serial.print(pitch, 1);
  Serial.print(" roll=");
  Serial.print(roll, 1);

  Serial.print(" | gyro=");
  Serial.print(g.gyro.x, 2); Serial.print(",");
  Serial.print(g.gyro.y, 2); Serial.print(",");
  Serial.print(g.gyro.z, 2);

  Serial.print(" | gesture=");
  Serial.println(gesture);

  delay(150);
}