#include <Bluepad32.h>

ControllerPtr controller;

// ---------------------------
// L298N Pin Definitions
// ---------------------------
const int PAN_IN1 = 26;
const int PAN_IN2 = 25;
const int PAN_EN  = 27;  // PWM

const int TILT_IN1 = 14;
const int TILT_IN2 = 12;
const int TILT_EN  = 13; // PWM

// PWM Channels
const int PWM_CHANNEL_PAN = 0;
const int PWM_CHANNEL_TILT = 1;

const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

const int DEADZONE = 50;
const int MAX_PWM  = 255;

// ---------------------------
// Smooth Movement Variables
// ---------------------------
int currentPanPWM = 0;
int currentTiltPWM = 0;

int targetPanPWM = 0;
int targetTiltPWM = 0;

const int RAMP_STEP = 5;   // lower values = smoother

// ---------------------------
// Battery Monitoring
// ---------------------------
const int BATTERY_PIN = 36;     // ADC pin
const float R1 = 30000.0;       // Voltage divider resistor 1 (top)
const float R2 = 7500.0;        // Divider resistor 2 (bottom)
                                // Use whatever values YOU choose
const float ADC_MAX = 4095.0;
const float ADC_REF = 3.3;      // ESP32 ADC reference

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float adcVoltage = (raw / ADC_MAX) * ADC_REF;

  // Reverse the voltage divider formula
  float batteryVoltage = adcVoltage * (R1 + R2) / R2;

  return batteryVoltage;
}

// ---------------------------
// Controller Callbacks
// ---------------------------
void onControllerConnected(ControllerPtr ctl) {
  controller = ctl;
}

void onControllerDisconnected(ControllerPtr ctl) {
  controller = nullptr;
}

void setup() {
  Serial.begin(115200);

  BP32.setup(&onControllerConnected, &onControllerDisconnected);

  pinMode(PAN_IN1, OUTPUT);
  pinMode(PAN_IN2, OUTPUT);
  pinMode(TILT_IN1, OUTPUT);
  pinMode(TILT_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL_PAN, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_TILT, PWM_FREQ, PWM_RES);

  ledcAttachPin(PAN_EN, PWM_CHANNEL_PAN);
  ledcAttachPin(TILT_EN, PWM_CHANNEL_TILT);

  analogReadResolution(12);  // ESP32 ADC resolution
}

void applyRamping() {
  // PAN
  if (currentPanPWM < targetPanPWM)
    currentPanPWM += RAMP_STEP;
  else if (currentPanPWM > targetPanPWM)
    currentPanPWM -= RAMP_STEP;

  currentPanPWM = constrain(currentPanPWM, 0, MAX_PWM);

  // TILT
  if (currentTiltPWM < targetTiltPWM)
    currentTiltPWM += RAMP_STEP;
  else if (currentTiltPWM > targetTiltPWM)
    currentTiltPWM -= RAMP_STEP;

  currentTiltPWM = constrain(currentTiltPWM, 0, MAX_PWM);

  // Apply PWM
  ledcWrite(PWM_CHANNEL_PAN, currentPanPWM);
  ledcWrite(PWM_CHANNEL_TILT, currentTiltPWM);
}

void loop() {
  BP32.update();

  if (!controller)
    return;

  // -------------------
  // Center Button (B)
  // -------------------
  if (controller->buttons() & BUTTON_B) {
    targetPanPWM = 0;
    targetTiltPWM = 0;
    applyRamping();
    return;
  }

  // -------------------
  // Joystick Input
  // -------------------
  int lx = controller->axisX();
  int ly = controller->axisY();

  // ----- PAN -----
  if (abs(lx) < DEADZONE) {
    targetPanPWM = 0;
  } else if (lx > 0) {
    digitalWrite(PAN_IN1, HIGH);
    digitalWrite(PAN_IN2, LOW);
    targetPanPWM = map(lx, 0, 512, 0, MAX_PWM);
  } else {
    digitalWrite(PAN_IN1, LOW);
    digitalWrite(PAN_IN2, HIGH);
    targetPanPWM = map(-lx, 0, 512, 0, MAX_PWM);
  }

  // ----- TILT -----
  if (abs(ly) < DEADZONE) {
    targetTiltPWM = 0;
  } else if (ly > 0) {
    digitalWrite(TILT_IN1, HIGH);
    digitalWrite(TILT_IN2, LOW);
    targetTiltPWM = map(ly, 0, 512, 0, MAX_PWM);
  } else {
    digitalWrite(TILT_IN1, LOW);
    digitalWrite(TILT_IN2, HIGH);
    targetTiltPWM = map(-ly, 0, 512, 0, MAX_PWM);
  }

  // Apply smoothing each loop
  applyRamping();

  // -------------------
  // Battery Voltage
  // -------------------
  float volts = readBatteryVoltage();
  Serial.print("Battery: ");
  Serial.print(volts);
  Serial.println("V");
}
