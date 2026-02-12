#include <Bluepad32.h>

ControllerPtr controller;

// ============================================================
// MOTOR DRIVER (L298N) PINS
// ============================================================
const int PAN_IN1 = 26;
const int PAN_IN2 = 25;
const int PAN_EN  = 27;  // PWM output

const int TILT_IN1 = 14;
const int TILT_IN2 = 12;
const int TILT_EN  = 13;  // PWM output

// ============================================================
// LANC INTERFACE PINS
// ============================================================
const int LANC_TX = 18;  // ESP32 -> Camera LANC input
const int LANC_RX = 19;  // Camera LANC output -> ESP32 input

// ============================================================
// PWM CONFIG
// ============================================================
const int PWM_CHANNEL_PAN  = 0;
const int PWM_CHANNEL_TILT = 1;
const int PWM_FREQ = 20000;  // above audible range
const int PWM_RES  = 8;
const int MAX_PWM  = 255;

// ============================================================
// INPUT TUNING
// ============================================================
const int STICK_DEADZONE   = 40;
const int TRIGGER_DEADZONE = 12;

// Exponential curve for very fine control around center
const float STICK_EXPO = 0.55f;

// Ramp limits for smooth motion
const int RAMP_STEP_FAST = 6;
const int RAMP_STEP_SLOW = 2;

// Safety: stop motors if no valid gamepad update for too long
const uint32_t FAILSAFE_MS = 500;

// ============================================================
// LANC COMMAND BYTES
// ============================================================
const uint8_t LANC_REC_TOGGLE = 0x18;
const uint8_t LANC_ZOOM_STOP      = 0x00;
const uint8_t LANC_ZOOM_IN_SLOW   = 0x14;
const uint8_t LANC_ZOOM_IN_MED    = 0x16;
const uint8_t LANC_ZOOM_IN_FAST   = 0x18;
const uint8_t LANC_ZOOM_OUT_SLOW  = 0x24;
const uint8_t LANC_ZOOM_OUT_MED   = 0x26;
const uint8_t LANC_ZOOM_OUT_FAST  = 0x28;

const int LANC_BIT_US   = 104;   // 1/9600
const int LANC_FRAME_US = 1000;  // 1 ms nominal frame spacing

// ============================================================
// RUNTIME STATE
// ============================================================
int panTarget = 0;
int tiltTarget = 0;
int panCurrent = 0;
int tiltCurrent = 0;

uint32_t lastControllerUpdateMs = 0;
uint32_t lastStatusPrintMs = 0;

bool lastXPressed = false;

// ------------------------------------------------------------
// Utility: signed normalized value with deadzone and expo
// ------------------------------------------------------------
float applyDeadzoneExpo(int value, int deadzone, float expo) {
  int absV = abs(value);
  if (absV <= deadzone) {
    return 0.0f;
  }

  float norm = (float)(absV - deadzone) / (512.0f - deadzone);
  norm = constrain(norm, 0.0f, 1.0f);

  // Mix linear + cubic for controllable expo
  float curved = (1.0f - expo) * norm + expo * norm * norm * norm;

  return (value < 0) ? -curved : curved;
}

// ------------------------------------------------------------
// Utility: trigger normalized from 0..1023 to 0..1 with deadzone
// ------------------------------------------------------------
float normalizeTrigger(int triggerValue) {
  int clipped = constrain(triggerValue, 0, 1023);
  if (clipped <= TRIGGER_DEADZONE) {
    return 0.0f;
  }
  float norm = (float)(clipped - TRIGGER_DEADZONE) / (1023.0f - TRIGGER_DEADZONE);
  return constrain(norm, 0.0f, 1.0f);
}

void setMotorDirectionAndTarget(int value, int in1, int in2, int &targetOut) {
  if (value > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    targetOut = value;
  } else if (value < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    targetOut = -value;
  } else {
    // Active braking off (coast)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    targetOut = 0;
  }
}

void rampValue(int target, int &current) {
  int diff = target - current;
  if (diff == 0) return;

  int step = (abs(diff) > 35) ? RAMP_STEP_FAST : RAMP_STEP_SLOW;
  if (diff > 0) {
    current += min(step, diff);
  } else {
    current -= min(step, -diff);
  }
}

void applyMotors() {
  rampValue(panTarget, panCurrent);
  rampValue(tiltTarget, tiltCurrent);

  panCurrent = constrain(panCurrent, 0, MAX_PWM);
  tiltCurrent = constrain(tiltCurrent, 0, MAX_PWM);

  ledcWrite(PWM_CHANNEL_PAN, panCurrent);
  ledcWrite(PWM_CHANNEL_TILT, tiltCurrent);
}

// ------------------------------------------------------------
// LANC low-level send helpers
// ------------------------------------------------------------
void sendLancByte(uint8_t b) {
  pinMode(LANC_TX, OUTPUT);

  // Start bit
  digitalWrite(LANC_TX, LOW);
  delayMicroseconds(LANC_BIT_US);

  // 8 bits LSB first
  for (int i = 0; i < 8; i++) {
    digitalWrite(LANC_TX, (b >> i) & 0x01);
    delayMicroseconds(LANC_BIT_US);
  }

  // Stop bit
  digitalWrite(LANC_TX, HIGH);
  delayMicroseconds(LANC_BIT_US);
}

void sendLanc(uint8_t b) {
  sendLancByte(b);
  sendLancByte(b);
}

uint8_t zoomCommandFromTriggers(float zoomIn, float zoomOut) {
  float net = zoomIn - zoomOut;  // + => zoom in, - => zoom out

  if (abs(net) < 0.05f) return LANC_ZOOM_STOP;

  float mag = abs(net);
  if (net > 0.0f) {
    if (mag < 0.33f) return LANC_ZOOM_IN_SLOW;
    if (mag < 0.66f) return LANC_ZOOM_IN_MED;
    return LANC_ZOOM_IN_FAST;
  }

  if (mag < 0.33f) return LANC_ZOOM_OUT_SLOW;
  if (mag < 0.66f) return LANC_ZOOM_OUT_MED;
  return LANC_ZOOM_OUT_FAST;
}

void onControllerConnected(ControllerPtr ctl) {
  controller = ctl;
  lastControllerUpdateMs = millis();
  Serial.println("Controller connected");
}

void onControllerDisconnected(ControllerPtr ctl) {
  (void)ctl;
  controller = nullptr;
  panTarget = 0;
  tiltTarget = 0;
  Serial.println("Controller disconnected");
}

void setup() {
  Serial.begin(115200);

  pinMode(PAN_IN1, OUTPUT);
  pinMode(PAN_IN2, OUTPUT);
  pinMode(TILT_IN1, OUTPUT);
  pinMode(TILT_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL_PAN, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_TILT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PAN_EN, PWM_CHANNEL_PAN);
  ledcAttachPin(TILT_EN, PWM_CHANNEL_TILT);

  pinMode(LANC_TX, OUTPUT);
  digitalWrite(LANC_TX, HIGH);
  pinMode(LANC_RX, INPUT_PULLUP);

  BP32.setup(&onControllerConnected, &onControllerDisconnected);
  BP32.forgetBluetoothKeys();

  Serial.println("Pan/Tilt + LANC controller ready");
}

void loop() {
  BP32.update();

  uint8_t zoomByte = LANC_ZOOM_STOP;

  if (controller && controller->isConnected()) {
    lastControllerUpdateMs = millis();

    // RIGHT STICK controls pan/tilt with smooth pressure-sensitive speed.
    int rx = controller->axisRX();
    int ry = controller->axisRY();

    float panNorm = applyDeadzoneExpo(rx, STICK_DEADZONE, STICK_EXPO);
    float tiltNorm = applyDeadzoneExpo(-ry, STICK_DEADZONE, STICK_EXPO);  // invert so up = positive tilt

    int panCmd = (int)(panNorm * MAX_PWM);
    int tiltCmd = (int)(tiltNorm * MAX_PWM);

    setMotorDirectionAndTarget(panCmd, PAN_IN1, PAN_IN2, panTarget);
    setMotorDirectionAndTarget(tiltCmd, TILT_IN1, TILT_IN2, tiltTarget);

    // Triggers for smooth zoom pressure mapping
    float zoomOut = normalizeTrigger(controller->brake());     // LT
    float zoomIn = normalizeTrigger(controller->throttle());   // RT
    zoomByte = zoomCommandFromTriggers(zoomIn, zoomOut);

    // X button toggles record start/stop
    bool xPressed = controller->x();
    if (xPressed && !lastXPressed) {
      sendLanc(LANC_REC_TOGGLE);
      Serial.println("LANC record toggle sent");
    }
    lastXPressed = xPressed;

  } else {
    panTarget = 0;
    tiltTarget = 0;
    lastXPressed = false;
  }

  // Failsafe if updates stall
  if (millis() - lastControllerUpdateMs > FAILSAFE_MS) {
    panTarget = 0;
    tiltTarget = 0;
  }

  applyMotors();

  // Send zoom command once per loop frame
  uint32_t frameStart = micros();
  sendLanc(zoomByte);
  while (micros() - frameStart < (uint32_t)LANC_FRAME_US) {
    // maintain nominal LANC frame spacing
  }

  // Status output at 4 Hz
  if (millis() - lastStatusPrintMs > 250) {
    int lancLevel = digitalRead(LANC_RX);

    Serial.print("PAN PWM: ");
    Serial.print(panCurrent);
    Serial.print("  TILT PWM: ");
    Serial.print(tiltCurrent);
    Serial.print("  LANC_RX: ");
    Serial.println(lancLevel);

    lastStatusPrintMs = millis();
  }
}
