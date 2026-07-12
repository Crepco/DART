// DART turret servo controller (Arduino Uno R3).
// Protocol + safety notes: see docs/SERIAL_PROTOCOL.md
//
// Drives pan/tilt/trigger servos from host commands.
//
// Host -> R3:  P###T###F#       pan, tilt, fire flag
// R3 -> host:  READY            one-shot boot handshake
#include <Servo.h>

#define STOP_PAN 90
#define STOP_TILT 90

#define TRIGGER_REST_ANGLE 60
#define TRIGGER_FIRE_ANGLE 150

#define TIMEOUT_MS 500

#define TILT_PIN 11
#define PAN_PIN 10
#define TRIGGER_PIN 8
#define LED_PIN 13

Servo panServo;
Servo tiltServo;
Servo triggerServo;

char buf[32];
uint8_t bufIdx = 0;
unsigned long lastCmdTime = 0;
bool fireState = false;

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

    if (millis() - lastCmdTime > TIMEOUT_MS)
        stopAll();
}

void parseCommand(const char *cmd)
{
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
