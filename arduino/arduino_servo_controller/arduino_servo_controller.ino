// DART turret servo controller — unified DART + FlowState firmware (Arduino Uno R3).
// Protocol + safety notes: see docs/SERIAL_PROTOCOL.md
//
// One R3 now runs both projects, so this sketch does two jobs:
//   1. Drive pan/tilt/trigger servos from host commands.
//   2. Stream the BioAmp EXG Pill (FlowState EEG, on A0) back to the host on request.
//
// Host -> R3 (one line each):
//   P###T###F#       pan, tilt, fire flag
//   S#               EEG stream: S1 = start, S0 = stop (off by default)
// R3 -> host:
//   READY            one-shot boot handshake
//   E####            one raw 10-bit EEG sample (0..1023), only while streaming
#include <Servo.h>

#define STOP_PAN 90
#define STOP_TILT 90

#define TRIGGER_REST_ANGLE 60
#define TRIGGER_FIRE_ANGLE 150

#define TIMEOUT_MS 500

#define TILT_PIN 9
#define PAN_PIN 10
#define TRIGGER_PIN 8
#define LED_PIN 13
#define EEG_PIN A0             // BioAmp EXG Pill signal out

#define SAMPLE_RATE 250        // Hz — must match SERIAL_FS in scripts/web/focus.py
const unsigned long SAMPLE_US = 1000000UL / SAMPLE_RATE;

Servo panServo;
Servo tiltServo;
Servo triggerServo;

char buf[32];
uint8_t bufIdx = 0;
unsigned long lastCmdTime = 0;
bool fireState = false;
bool streaming = false;
unsigned long nextSample = 0;

void stopAll()
{
    panServo.write(STOP_PAN);
    tiltServo.write(STOP_TILT);
    triggerServo.write(TRIGGER_REST_ANGLE);
    fireState = false;
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);

    while (Serial.available())
        Serial.read();

    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    triggerServo.attach(TRIGGER_PIN);

    stopAll();
    delay(500);

    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);

    Serial.println("READY");
    lastCmdTime = millis();
    nextSample = micros();
}

void loop()
{
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

    // EEG streaming (FlowState). Paced by micros(); independent of command timeout.
    if (streaming)
    {
        unsigned long now = micros();
        if ((long)(now - nextSample) >= 0)
        {
            nextSample += SAMPLE_US;
            int value = analogRead(EEG_PIN);
            Serial.print('E');
            Serial.println(value);
        }
    }

    if (millis() - lastCmdTime > TIMEOUT_MS)
        stopAll();
}

void parseCommand(const char *cmd)
{
    // EEG stream toggle: "S1" / "S0".
    if (cmd[0] == 'S')
    {
        streaming = (cmd[1] == '1');
        if (streaming)
            nextSample = micros();
        lastCmdTime = millis();
        return;
    }

    if (cmd[0] != 'P')
        return;

    const char *tPtr = strchr(cmd, 'T');
    if (!tPtr)
        return;

    int p = atoi(cmd + 1);
    int t = atoi(tPtr + 1);

    panServo.write(p);
    tiltServo.write(t);

    const char *fPtr = strchr(cmd, 'F');
    if (fPtr)
    {
        fireState = (fPtr[1] == '1');
        triggerServo.write(fireState ? TRIGGER_FIRE_ANGLE : TRIGGER_REST_ANGLE);
    }

    lastCmdTime = millis();
}
