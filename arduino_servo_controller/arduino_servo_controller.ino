/*
 * Face-Tracking Turret – Arduino Controller (ZERO LATENCY)
 * ==========================================================
 * Updated to work with the Python PID controller. 
 * All smoothing, deadbands, and logic are handled by Python.
 * This Arduino simply executes commands as fast as possible.
 * * Hardware Safety: Includes hardware-level bounds checking
 * to protect against corrupted serial packets.
 */

#include <Servo.h>

// ── Neutral / stop ─────────────────
#define STOP_PAN 90
#define STOP_TILT 90

// ── Speed cap ──────────────────────
#define MAX_SPEED 35 // Max offset from 90 (Range: 55 to 125)

// ── Timeout ────────────────────────
#define TIMEOUT_MS 400 // Stops motors if Python crashes or disconnects

// ── Pins ───────────────────────────
#define TILT_PIN 9
#define PAN_PIN 10
#define LED_PIN 13

Servo panServo;
Servo tiltServo;

char buf[32];
uint8_t bufIdx = 0;
unsigned long lastCmdTime = 0;

// ───────────────────────────────────
void stopAll()
{
    panServo.write(STOP_PAN);
    tiltServo.write(STOP_TILT);
}

// ───────────────────────────────────
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    
    // MATCHES PYTHON SCRIPT FOR LOW LATENCY
    Serial.begin(115200); 

    // Clear any junk in the serial buffer
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

    // ── Safety clamp ──────────────────────────────
    // Python already clamps, but this protects the 
    // servos if a corrupted packet slips through.
    p = constrain(p, STOP_PAN - MAX_SPEED, STOP_PAN + MAX_SPEED);
    t = constrain(t, STOP_TILT - MAX_SPEED, STOP_TILT + MAX_SPEED);

    panServo.write(p);
    tiltServo.write(t);

    lastCmdTime = millis();
}