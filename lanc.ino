#include <Bluepad32.h>

// -------------------- LANC PINS --------------------
#define LANC_TX 18     // ESP32 → Camera
#define LANC_RX 19     // Camera → ESP32

// -------------------- GAMEPAD ----------------------
ControllerPtr gp;

// -------------------- LANC TIMING ------------------
const int BIT_US   = 104;    // 1/9600
const int FRAME_US = 1000;

// ------------- LANC COMMAND BYTES ------------------
// RECORD
#define REC_TOGGLE   0x18

// ZOOM
#define ZOOM_STOP      0x00
#define ZOOM_IN_SLOW   0x14
#define ZOOM_IN_MED    0x16
#define ZOOM_IN_FAST   0x18
#define ZOOM_OUT_SLOW  0x24
#define ZOOM_OUT_MED   0x26
#define ZOOM_OUT_FAST  0x28

// FOCUS
#define FOCUS_NEAR     0x38
#define FOCUS_FAR      0x48
#define FOCUS_STOP     0x00

// ----------------------------------------------------
// LOW-LEVEL LANC BYTE SEND
// ----------------------------------------------------
void sendLancByte(uint8_t b) {
    pinMode(LANC_TX, OUTPUT);

    // Start bit
    digitalWrite(LANC_TX, LOW);
    delayMicroseconds(BIT_US);

    // 8 bits
    for (int i = 0; i < 8; i++) {
        digitalWrite(LANC_TX, (b >> i) & 1);
        delayMicroseconds(BIT_US);
    }

    // Stop bit
    digitalWrite(LANC_TX, HIGH);
    delayMicroseconds(BIT_US);
}

void sendLanc(uint8_t b) {
    sendLancByte(b);
    sendLancByte(b);
}

// ----------------------------------------------------
// ZOOM SPEED MAP
// ----------------------------------------------------
uint8_t zoomCommand(int axis) {
    // axis is range -512 to +512
    int dz = 80;

    if (axis > dz && axis < 200)  return ZOOM_IN_SLOW;
    if (axis >= 200 && axis < 350) return ZOOM_IN_MED;
    if (axis >= 350)               return ZOOM_IN_FAST;

    if (axis < -dz && axis > -200) return ZOOM_OUT_SLOW;
    if (axis <= -200 && axis > -350) return ZOOM_OUT_MED;
    if (axis <= -350)               return ZOOM_OUT_FAST;

    return ZOOM_STOP;
}

// ----------------------------------------------------
// BLUEPAD32 CALLBACK
// ----------------------------------------------------
void onConnectedController(ControllerPtr ctl) {
    gp = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
    gp = nullptr;
}

void setup() {
    Serial.begin(115200);

    pinMode(LANC_TX, OUTPUT);
    digitalWrite(LANC_TX, HIGH);

    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    BP32.update();

    unsigned long start = micros();

    uint8_t zoomByte = ZOOM_STOP;
    uint8_t focusByte = FOCUS_STOP;
    static bool lastA = false;
    static bool lastB = false;

    if (gp && gp->isConnected()) {

        // ---------------- ZOOM FROM LEFT STICK ----------------
        int ly = gp->axisY();        // -512 .. +512
        zoomByte = zoomCommand(-ly); // invert if needed

        // ---------------- RECORD BUTTON (A) -------------------
        bool A = gp->a();
        if (A && !lastA) {
            sendLanc(REC_TOGGLE);  // send record toggle once
        }
        lastA = A;

        // ---------------- FOCUS BUTTON (B) ---------------------
        bool B = gp->b();

        if (B) {
            // Use stick direction to choose focus direction
            int ly2 = gp->axisY();

            if (ly2 > 150)
                focusByte = FOCUS_FAR;
            else if (ly2 < -150)
                focusByte = FOCUS_NEAR;
            else
                focusByte = FOCUS_FAR; // default
        } else {
            focusByte = FOCUS_STOP;
        }

    } else {
        zoomByte = ZOOM_STOP;
        focusByte = FOCUS_STOP;
    }

    // ---------------- SEND LANC FRAME ----------------
    sendLanc(zoomByte);   // byte 0
    sendLanc(focusByte);  // byte 1

    // maintain exact LANC 1ms frame
    while (micros() - start < FRAME_US) {}
}
