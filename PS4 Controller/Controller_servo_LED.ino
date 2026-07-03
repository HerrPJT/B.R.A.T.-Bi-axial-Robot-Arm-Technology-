#include <Bluepad32.h>
#include <ESP32Servo.h>


// NOVO — pinos
#define LED_PIN   21
#define SERVO_PIN 18

// NOVO — objeto servo
Servo myServo;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int myServoStop = 1500;  // ← TEM DE ESTAR AQUI, antes de qualquer função


void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}


void processGamepad(ControllerPtr ctl) {
    if (ctl->a()) {
        static int colorIdx = 0;
        switch (colorIdx % 3) {
            case 0: ctl->setColorLED(255, 0, 0); break;
            case 1: ctl->setColorLED(0, 255, 0); break;
            case 2: ctl->setColorLED(0, 0, 255); break;
        }
        colorIdx++;

        // NOVO — toggle do LED com botão A
        static bool ledState = false;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }

    if (ctl->b()) {
        static int led = 0;
        led++;
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        ctl->playDualRumble(0, 250, 0x80, 0x40);
    }

    // NOVO — botão Home/PS/Xbox desliga o gamepad via Bluetooth
    if (ctl->miscButtons() & 0x04) {
        myServo.writeMicroseconds(1500);   // para o servo antes de desligar
        digitalWrite(LED_PIN, LOW);              // apaga o LED
        ctl->disconnect();                       // desliga só o gamepad, ESP32 continua
    }

        // Joysticks: -512 a 512
    int left = ctl->axisY();   // manípulo esquerdo (vertical)
    int right = ctl->axisRY(); // manípulo direito (vertical)

    // Deadzone (evita drift)
    int deadzone = 50;

    if (abs(left) < deadzone) left = 0;
    if (abs(right) < deadzone) right = 0;

    // Calcula velocidade:
    // esquerdo → negativo (um sentido)
    // direito → positivo (outro sentido)
    int speed = 0;

    if (left != 0) {
        speed = map(left, -512, 512, -500, 500);
    } else if (right != 0) {
        speed = map(right, -512, 512, 500, -500);
    }

    // Servo contínuo:
    // 1500 = parado
    // <1500 = um lado
    // >1500 = outro lado
    int pulseUs = 1500 + speed;

    myServo.writeMicroseconds(pulseUs);

    dumpGamepad(ctl);





}



void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);

    // NOVO — configura LED e servo
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    myServo.attach(SERVO_PIN, 1000, 2000);  // NOVO
    myServo.writeMicroseconds(1500);         // NOVO — parado ao arrancar

    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
