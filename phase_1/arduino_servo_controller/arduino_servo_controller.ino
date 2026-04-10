#include <Servo.h>

// ── Stop values ──
#define STOP_PAN    90

// ── Speed limit ──
#define MAX_SPEED   35

// ── Timeout safety ──
#define TIMEOUT_MS  250

// ── Pins ──
#define PAN_PIN     9
#define TRIGGER_PIN 8
#define LED_PIN     13

// ── MG996R Trigger Settings ──
#define TRIGGER_REST_ANGLE  60
#define TRIGGER_FIRE_ANGLE  150

Servo panServo;
Servo triggerServo;

// ── State ──
bool fireRequested = false;
bool lastFireState = false;

unsigned long lastCmdTime = 0;

// ── Serial buffer ──
char buf[32];
uint8_t bufIdx = 0;

// ── Clamp function ──
int clampSpeed(int v, int center) {
    return constrain(v, center - MAX_SPEED, center + MAX_SPEED);
}

// ── Stop all safely ──
void stopAll() {
    panServo.write(STOP_PAN);

    // Always reset trigger
    triggerServo.write(TRIGGER_REST_ANGLE);

    fireRequested = false;
    lastFireState = false;
}

// ── Setup ──
void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600);

    panServo.attach(PAN_PIN);
    triggerServo.attach(TRIGGER_PIN);

    stopAll();
    delay(500);

    // Startup blink
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);

    Serial.println("READY");
    lastCmdTime = millis();
}

// ── Loop ──
void loop() {

    // 1. Serial read
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            buf[bufIdx] = '\0';
            if (bufIdx > 0) parseCommand(buf);
            bufIdx = 0;
        } 
        else if (c != '\r') {
            if (bufIdx < sizeof(buf) - 1)
                buf[bufIdx++] = c;
            else
                bufIdx = 0;
        }
    }

    // 2. Timeout safety
    if (millis() - lastCmdTime > TIMEOUT_MS) {
        stopAll();
    }

    // 3. Trigger logic (state change only)
    if (fireRequested != lastFireState) {
        if (fireRequested) {
            triggerServo.write(TRIGGER_FIRE_ANGLE);   // HOLD trigger
        } else {
            triggerServo.write(TRIGGER_REST_ANGLE);   // RELEASE trigger
        }
        lastFireState = fireRequested;
    }
}

// ── Parse incoming command: P###F#  (e.g. P090F0) ──
void parseCommand(const char* cmd) {

    if (cmd[0] != 'P') return;

    int p = atoi(cmd + 1);

    panServo.write(clampSpeed(p, STOP_PAN));

    const char* fPtr = strchr(cmd, 'F');
    if (fPtr && (fPtr[1] == '0' || fPtr[1] == '1')) {
        fireRequested = (fPtr[1] == '1');
    } else {
        fireRequested = false;   // FORCE RESET if missing
    }

    lastCmdTime = millis();
}
