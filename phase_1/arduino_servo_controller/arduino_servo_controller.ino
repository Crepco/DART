/*
 * Face-Tracking Turret – Arduino Controller (STABLE VERSION)
 * ==========================================================
 * Improvements:
 *   • Deadband → ignores tiny jitter
 *   • Smoothing → smooth servo motion
 *   • Stable stop (no micro-movement near 90)
 */

#include <Servo.h>

// ── Neutral / stop ─────────────────
#define STOP_PAN 90
#define STOP_TILT 90

// ── Speed cap ──────────────────────
#define MAX_SPEED 35

// ── Timeout ────────────────────────
#define TIMEOUT_MS 400 // increased for stability

// ── Pins ───────────────────────────
#define TILT_PIN 9
#define PAN_PIN 10
#define LED_PIN 13

Servo panServo;
Servo tiltServo;

// Current values
float lastPan = STOP_PAN;
float lastTilt = STOP_TILT;

int panSpeed;
int tiltSpeed;

char buf[32];
uint8_t bufIdx = 0;
unsigned long lastCmdTime = 0;

// ───────────────────────────────────
int clampSpeed(int v, int center)
{
    int lo = center - MAX_SPEED;
    int hi = center + MAX_SPEED;
    return constrain(v, lo, hi);
}

// ───────────────────────────────────
void stopAll()
{
    lastPan = STOP_PAN;
    lastTilt = STOP_TILT;
    panServo.write(STOP_PAN);
    tiltServo.write(STOP_TILT);
}

// ───────────────────────────────────
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600);

    while (Serial.available())
        Serial.read();

    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);

    stopAll();
    delay(500);

    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);

    Serial.println("READY");
    lastCmdTime = millis();
}

// ───────────────────────────────────
void loop()
{
    // Read serial
    while (Serial.available())
    {
        char c = (char)Serial.read();

        if (c == '\n')
        {
            buf[bufIdx] = '\0';
            if (bufIdx > 0)
                parseCommand(buf);
            bufIdx = 0;
        }
        else if (c != '\r')
        {
            if (bufIdx < sizeof(buf) - 1)
                buf[bufIdx++] = c;
            else
                bufIdx = 0;
        }
    }

    // Timeout safety
    if (millis() - lastCmdTime > TIMEOUT_MS)
    {
        stopAll();
    }
}

// ───────────────────────────────────
void parseCommand(const char *cmd)
{
    if (cmd[0] != 'P')
        return;

    const char *tPtr = strchr(cmd, 'T');
    if (!tPtr)
        return;

    int p = atoi(cmd + 1);
    int t = atoi(tPtr + 1);

    p = clampSpeed(p, STOP_PAN);
    t = clampSpeed(t, STOP_TILT);

    panSpeed = p;
    tiltSpeed = t;

    applySmoothMovement();

    lastCmdTime = millis();
}

// ───────────────────────────────────
void applySmoothMovement()
{
    int targetPan = clampSpeed(panSpeed, STOP_PAN);
    int targetTilt = clampSpeed(tiltSpeed, STOP_TILT);

    // 🔥 EMA smoothing (adjust 0.2 → smoother/slower)
    lastPan = 0.8 * lastPan + 0.2 * targetPan;
    lastTilt = 0.8 * lastTilt + 0.2 * targetTilt;

    // Deadband near stop (VERY IMPORTANT)
    if (abs(targetPan - STOP_PAN) < 3)
        targetPan = STOP_PAN;
    if (abs(targetTilt - STOP_TILT) < 3)
        targetTilt = STOP_TILT;

    // Limit step size (prevents sudden jumps)
    int maxStep = 3;

    if (targetPan > lastPan)
        lastPan += min(maxStep, targetPan - lastPan);
    else
        lastPan -= min(maxStep, lastPan - targetPan);

    if (targetTilt > lastTilt)
        lastTilt += min(maxStep, targetTilt - lastTilt);
    else
        lastTilt -= min(maxStep, lastTilt - targetTilt);

    // Write to servo
    panServo.write(lastPan);
    tiltServo.write(lastTilt);
}