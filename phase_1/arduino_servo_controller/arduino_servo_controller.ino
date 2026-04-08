#include <Servo.h>

// ── Stop values (tune if drifting) ──
#define STOP_PAN    90
#define STOP_TILT   90

// ── Speed limit ──
#define MAX_SPEED   35

// ── Timeout safety ──
#define TIMEOUT_MS  250

// ── Pins ──
#define TILT_PIN    9
#define PAN_PIN     10
#define TRIGGER_PIN 11
#define LED_PIN     13

// ── Trigger tuning ──
// If your trigger servo is continuous rotation, 90 is STOP.
// If it is positional, set TRIGGER_STOP_ANGLE to your safe "rest" angle.
#define TRIGGER_STOP_ANGLE  90
#define TRIGGER_FIRE_DELTA  5        // how far from stop/rest to drive when firing
#define TRIGGER_MAX_FIRE_MS 4000UL   // max fire time per burst

Servo panServo;
Servo tiltServo;
Servo triggerServo;

int panSpeed;
int tiltSpeed;

bool fireRequested = false;
bool firingActive = false;
unsigned long firingStartMs = 0;
bool fireLatch = false; // require F0 before next burst

char buf[32];
uint8_t bufIdx = 0;
unsigned long lastCmdTime = 0;

// ── Clamp speed ──
int clampSpeed(int v, int center) {
    return constrain(v, center - MAX_SPEED, center + MAX_SPEED);
}

// ── Stop everything ──
void stopAll() {
    panServo.write(STOP_PAN);
    tiltServo.write(STOP_TILT);
    triggerServo.write(TRIGGER_STOP_ANGLE);
    fireRequested = false;
    firingActive = false;
    fireLatch = false;
}

// ── Setup ──
void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600);

    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    triggerServo.attach(TRIGGER_PIN);

    stopAll();
    delay(500);

    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);

    Serial.println("READY");
    lastCmdTime = millis();
}

// ── Loop ──
void loop() {

    // Read serial
    while (Serial.available()) {
        char c = (char)Serial.read();

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

    // Timeout safety
    if (millis() - lastCmdTime > TIMEOUT_MS) {
        stopAll();
    }

    // ── Trigger logic ──
    // - If no face / off-crosshair (F0): stop trigger immediately and re-arm.
    // - If on-crosshair (F1): fire up to MAX time, then stop until F0 happens.
    if (!fireRequested) {
        firingActive = false;
        fireLatch = false;
        triggerServo.write(TRIGGER_STOP_ANGLE);
    } else {
        if (!firingActive && !fireLatch) {
            firingActive = true;
            firingStartMs = millis();
        }

        if (firingActive) {
            triggerServo.write(TRIGGER_STOP_ANGLE + TRIGGER_FIRE_DELTA);
            if (millis() - firingStartMs >= TRIGGER_MAX_FIRE_MS) {
                firingActive = false;
                fireLatch = true;
                triggerServo.write(TRIGGER_STOP_ANGLE);
            }
        } else {
            triggerServo.write(TRIGGER_STOP_ANGLE);
        }
    }
}

// ── Parse command ──
void parseCommand(const char* cmd) {
    if (cmd[0] != 'P') return;

    const char* tPtr = strchr(cmd, 'T');
    if (!tPtr) return;

    int p = atoi(cmd + 1);
    int t = atoi(tPtr + 1);

    p = clampSpeed(p, STOP_PAN);
    t = clampSpeed(t, STOP_TILT);

    panSpeed = p;
    tiltSpeed = t;

    panServo.write(panSpeed);
    tiltServo.write(tiltSpeed);

    // Fire flag
    const char* fPtr = strchr(cmd, 'F');
    if (fPtr && (fPtr[1] == '0' || fPtr[1] == '1')) {
        fireRequested = (fPtr[1] == '1');
    } else {
        fireRequested = false;
    }

    lastCmdTime = millis();
}