/*
 * PS4 Controller to NEMA 23 Stepper Motor Control
 * Using Bluepad32 library for ESP32
 * Board: DOIT ESP32 DEVKIT V1 (with Bluepad32)
 */

#include <Bluepad32.h>

// ============ PIN DEFINITIONS ============
// X Motor (DM556): PUL+ D15, PUL- D2, DIR+ D16, DIR- D5
#define X_PUL_PLUS    15
#define X_PUL_MINUS   2
#define X_DIR_PLUS    16
#define X_DIR_MINUS   5

// Y Motor (DM556): PUL+ D33, PUL- D14, DIR+ D26, DIR- D13
#define Y_PUL_PLUS    33
#define Y_PUL_MINUS   14
#define Y_DIR_PLUS    26
#define Y_DIR_MINUS   13

// Solenoid relay
#define SOLENOID_PIN  23

// ============ STEPPER SETTINGS ============
#define MAX_SPEED         1500    // Reduced from 4000 for lower sensitivity
#define MIN_SPEED         50
#define DEADZONE          10      // Small deadzone for precise movements

// ============ VARIABLES ============
long xStepInterval = 0;
long yStepInterval = 0;
unsigned long xLastStep = 0;
unsigned long yLastStep = 0;
bool motorsEnabled = true;

// Solenoid control
unsigned long solenoidStartTime = 0;
bool solenoidActive = false;
bool r2WasPressed = false;
#define SOLENOID_PULSE_MS  30

ControllerPtr myController = nullptr;

// ============ CONTROLLER CALLBACKS ============
void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Serial.println("========================================");
        Serial.println("   *** CONTROLLER CONNECTED! ***");
        Serial.println("========================================");
        ControllerProperties properties = ctl->getProperties();
        Serial.printf("Controller: %s, VID=0x%04x, PID=0x%04x\n", 
            ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
        myController = ctl;
        // Set LED to green when connected
        ctl->setColorLED(0, 255, 0);
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Serial.println("Controller disconnected!");
        myController = nullptr;
        // Stop motors when controller disconnects
        xStepInterval = 0;
        yStepInterval = 0;
    }
}

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("========================================");
    Serial.println("   PS4 STEPPER MOTOR CONTROL");
    Serial.println("   Using Bluepad32");
    Serial.println("========================================");
    
    // Setup X motor pins (differential)
    pinMode(X_PUL_PLUS, OUTPUT);
    pinMode(X_PUL_MINUS, OUTPUT);
    pinMode(X_DIR_PLUS, OUTPUT);
    pinMode(X_DIR_MINUS, OUTPUT);
    
    // Setup Y motor pins (differential)
    pinMode(Y_PUL_PLUS, OUTPUT);
    pinMode(Y_PUL_MINUS, OUTPUT);
    pinMode(Y_DIR_PLUS, OUTPUT);
    pinMode(Y_DIR_MINUS, OUTPUT);
    
    // Initialize differential pairs (opposite states)
    digitalWrite(X_PUL_PLUS, LOW);
    digitalWrite(X_PUL_MINUS, HIGH);
    digitalWrite(X_DIR_PLUS, LOW);
    digitalWrite(X_DIR_MINUS, HIGH);
    digitalWrite(Y_PUL_PLUS, LOW);
    digitalWrite(Y_PUL_MINUS, HIGH);
    digitalWrite(Y_DIR_PLUS, LOW);
    digitalWrite(Y_DIR_MINUS, HIGH);
    
    // Setup solenoid relay pin
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW);
    
    // Setup Bluepad32
    Serial.printf("Bluepad32 Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("ESP32 BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", 
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // BP32.forgetBluetoothKeys();  // Commented out to remember pairings
    BP32.enableVirtualDevice(false);
    
    Serial.println();
    Serial.println("Press PS button on controller to connect...");
    Serial.println("Controls:");
    Serial.println("  Left Stick X: Motor X");
    Serial.println("  Left Stick Y: Motor Y");
    Serial.println("  Cross (X): Emergency Stop");
    Serial.println("  Circle (O): Re-enable motors");
    Serial.println("  R2: Fire solenoid (30ms pulse)");
    Serial.println();
}

// ============ STEP MOTORS ============
void stepMotors() {
    unsigned long now = micros();
    
    // X motor step (differential pulse)
    if (xStepInterval > 0 && (now - xLastStep) >= xStepInterval) {
        digitalWrite(X_PUL_PLUS, HIGH);
        digitalWrite(X_PUL_MINUS, LOW);
        delayMicroseconds(2);
        digitalWrite(X_PUL_PLUS, LOW);
        digitalWrite(X_PUL_MINUS, HIGH);
        xLastStep = now;
    }
    
    // Y motor step (differential pulse)
    if (yStepInterval > 0 && (now - yLastStep) >= yStepInterval) {
        digitalWrite(Y_PUL_PLUS, HIGH);
        digitalWrite(Y_PUL_MINUS, LOW);
        delayMicroseconds(2);
        digitalWrite(Y_PUL_PLUS, LOW);
        digitalWrite(Y_PUL_MINUS, HIGH);
        yLastStep = now;
    }
}

// ============ MAP JOYSTICK TO INTERVAL ============
// Bluepad32 joystick range: -511 to 512
long joystickToInterval(int value) {
    if (abs(value) < DEADZONE) return 0;
    long speed = map(abs(value), DEADZONE, 512, MIN_SPEED, MAX_SPEED);
    return 1000000L / speed;
}

// ============ UPDATE FROM CONTROLLER ============
void updateFromController(ControllerPtr ctl) {
    int lx = ctl->axisX();   // Left stick X: -511 to 512
    int ly = ctl->axisY();   // Left stick Y: -511 to 512
    
    // X Motor control (differential direction)
    if (abs(lx) >= DEADZONE) {
        if (lx > 0) {
            digitalWrite(X_DIR_PLUS, HIGH);
            digitalWrite(X_DIR_MINUS, LOW);
        } else {
            digitalWrite(X_DIR_PLUS, LOW);
            digitalWrite(X_DIR_MINUS, HIGH);
        }
        xStepInterval = joystickToInterval(lx);
    } else {
        xStepInterval = 0;
    }
    
    // Y Motor control (differential direction)
    if (abs(ly) >= DEADZONE) {
        if (ly < 0) {
            digitalWrite(Y_DIR_PLUS, HIGH);
            digitalWrite(Y_DIR_MINUS, LOW);
        } else {
            digitalWrite(Y_DIR_PLUS, LOW);
            digitalWrite(Y_DIR_MINUS, HIGH);
        }
        yStepInterval = joystickToInterval(ly);
    } else {
        yStepInterval = 0;
    }
    
    // Emergency Stop (Cross / X button) - just stop stepping, no ENA pins used
    if (ctl->x()) {
        xStepInterval = 0;
        yStepInterval = 0;
        motorsEnabled = false;
        ctl->setColorLED(255, 0, 0);    // Red LED
        Serial.println("!!! EMERGENCY STOP !!!");
    }
    
    // Re-enable motors (Circle / O button)
    if (ctl->b() && !motorsEnabled) {
        motorsEnabled = true;
        ctl->setColorLED(0, 255, 0);    // Green LED
        Serial.println("Motors re-enabled");
    }
    
    // Solenoid trigger (R2) - single press fires 30ms pulse
    bool r2Pressed = ctl->throttle() > 500;  // R2 threshold
    if (r2Pressed && !r2WasPressed && !solenoidActive) {
        // Rising edge of R2 - fire solenoid
        digitalWrite(SOLENOID_PIN, HIGH);
        solenoidStartTime = millis();
        solenoidActive = true;
        Serial.println("SOLENOID FIRED!");
    }
    r2WasPressed = r2Pressed;
}

// ============ UPDATE SOLENOID ============
void updateSolenoid() {
    if (solenoidActive && (millis() - solenoidStartTime >= SOLENOID_PULSE_MS)) {
        digitalWrite(SOLENOID_PIN, LOW);
        solenoidActive = false;
    }
}

// ============ MAIN LOOP ============
void loop() {
    // Update Bluepad32
    BP32.update();
    
    // Always update solenoid timing
    updateSolenoid();
    
    if (myController && myController->isConnected()) {
        // Controller is connected - process if there's new data
        if (myController->hasData()) {
            updateFromController(myController);
        }
        
        // Always step motors while connected
        stepMotors();
        
        // Debug output every 500ms
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug >= 500) {
            lastDebug = millis();
            Serial.printf("LX:%4d LY:%4d X_Int:%6ld Y_Int:%6ld %s\n",
                myController->axisX(), myController->axisY(), 
                xStepInterval, yStepInterval,
                motorsEnabled ? "ENABLED" : "STOPPED");
        }
    } else {
        // No controller - stop motors
        xStepInterval = 0;
        yStepInterval = 0;
        
        static unsigned long lastMsg = 0;
        if (millis() - lastMsg >= 2000) {
            lastMsg = millis();
            Serial.println("Waiting for controller... Press PS button");
        }
    }
    
    // Small delay to prevent watchdog trigger
    delay(1);
}
