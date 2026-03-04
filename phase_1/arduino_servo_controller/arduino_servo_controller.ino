/*
 * Face-Tracking Turret – Arduino Controller
 * ==========================================
 * For CONTINUOUS ROTATION (360°) MG996R servos.
 *
 *   Pin 9  = TILT servo (up / down)
 *   Pin 10 = PAN  servo (left / right)
 *
 * How continuous rotation servos work:
 *   write(90)  → STOP
 *   write(<90) → spin one direction  (lower = faster)
 *   write(>90) → spin other direction (higher = faster)
 *
 * Protocol from Python:  P###T###\n   e.g. P090T090
 *   Values are SPEED commands centered on 90 (not positions).
 *   90 = stop.  Values away from 90 = spin.
 *
 * Safety:
 *   - All received values are clamped to [STOP - MAX_SPEED, STOP + MAX_SPEED]
 *   - If no command received for TIMEOUT_MS, servos stop automatically
 *   - On startup both servos are stopped (90)
 *
 * Upload this sketch BEFORE running face_tracker.py.
 */

#include <Servo.h>   // Use Arduino Servo library to drive the two servos

// ── Neutral / stop value ─────────
//    Tweak STOP_PAN / STOP_TILT by ±1-2 if a servo creeps at "90".
#define STOP_PAN    90   // Servo value that means "don't move" for pan (continuous rotation)
#define STOP_TILT   90   // Servo value that means "don't move" for tilt

// ── Speed cap (max offset from stop) ─────
#define MAX_SPEED   25   // Servo range is [90-25, 90+25] = [65, 115]; keep in sync with Python

// ── Safety timeout ───────────────
#define TIMEOUT_MS  250  // If no serial command for this long, stop both servos

// ── Pins ─────────────────────────
#define TILT_PIN    9    // Tilt servo signal (up/down) on pin 9
#define PAN_PIN     10   // Pan servo signal (left/right) on pin 10
#define LED_PIN     13   // Onboard LED for status blink

Servo panServo;   // Servo object for horizontal (pan) axis
Servo tiltServo;  // Servo object for vertical (tilt) axis

int panSpeed;   // Current pan command value (65..115)
int tiltSpeed;  // Current tilt command value (65..115)

char buf[32];           // Buffer to collect one line: e.g. "P090T085"
uint8_t bufIdx = 0;     // How many chars are in buf so far
unsigned long lastCmdTime = 0;  // When we last received a valid command (for timeout)

int clampSpeed(int v, int center) {
    int lo = center - MAX_SPEED;   // Minimum allowed value (e.g. 65)
    int hi = center + MAX_SPEED;   // Maximum allowed value (e.g. 115)
    return constrain(v, lo, hi);   // Clamp v to [lo, hi] and return
}

void blinkLED(int times, int ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);   // LED on
        delay(ms);                     // Wait
        digitalWrite(LED_PIN, LOW);    // LED off
        delay(ms);                     // Wait
    }
}

void stopAll() {
    panSpeed  = STOP_PAN;              // Store stop value
    tiltSpeed = STOP_TILT;
    panServo.write(STOP_PAN);          // Send stop to pan servo
    tiltServo.write(STOP_TILT);        // Send stop to tilt servo
}

void setup() {
    pinMode(LED_PIN, OUTPUT);          // LED pin is output
    Serial.begin(9600);                // Start serial at 9600 baud (match Python)
    while (Serial.available()) Serial.read();  // Flush any leftover bytes from before reset

    panServo.attach(PAN_PIN);          // Attach pan servo to its pin
    tiltServo.attach(TILT_PIN);        // Attach tilt servo to its pin

    stopAll();                         // Start with both servos stopped
    delay(500);                        // Give servos a moment to settle

    blinkLED(3, 150);                  // Blink 3 times so user knows sketch is running
    Serial.println("READY");           // Tell Python we are ready for commands
    lastCmdTime = millis();            // Reset timeout timer
}

void loop() {
    // Parse incoming serial bytes one character at a time
    while (Serial.available()) {
        char c = (char)Serial.read();  // Read one byte from serial
        if (c == '\n') {               // End of line: we have a full command
            buf[bufIdx] = '\0';        // Null-terminate the string
            if (bufIdx > 0) parseCommand(buf);  // Parse e.g. "P090T085"
            bufIdx = 0;                // Reset buffer for next line
        } else if (c != '\r') {        // Ignore carriage return; accept other chars
            if (bufIdx < sizeof(buf) - 1)
                buf[bufIdx++] = c;     // Add char to buffer if room
            else
                bufIdx = 0;             // Buffer full: discard and start over
        }
    }

    // Safety: if Python crashed or disconnected, stop servos after timeout
    if (millis() - lastCmdTime > TIMEOUT_MS) {
        stopAll();
    }
}

void parseCommand(const char* cmd) {
    if (cmd[0] != 'P') return;         // Command must start with 'P'
    const char* tPtr = strchr(cmd, 'T');  // Find 'T' that separates pan and tilt
    if (!tPtr) return;                  // Invalid format if no 'T'

    int p = atoi(cmd + 1);             // Parse pan number after 'P' (e.g. "090" -> 90)
    int t = atoi(tPtr + 1);            // Parse tilt number after 'T' (e.g. "085" -> 85)

    // Clamp to safe speed range so we never send values outside 65..115
    p = clampSpeed(p, STOP_PAN);
    t = clampSpeed(t, STOP_TILT);

    panSpeed  = p;                      // Store for possible use elsewhere
    tiltSpeed = t;

    // Write to servos; clamp again in case of any mistake
    panServo.write(clampSpeed(panSpeed, STOP_PAN));
    tiltServo.write(clampSpeed(tiltSpeed, STOP_TILT));

    lastCmdTime = millis();             // Update timeout so we know we got a command
}
