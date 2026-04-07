/*
 * Face-Tracking Turret – Arduino Controller
 * ==========================================
 * For CONTINUOUS ROTATION (360°) MG996R servos.
 *
 *   Pin 8  = PAN  servo (left / right)
 *   Pin 9  = TILT 1 (same command as tilt 2 — move together)
 *   Pin 10 = TILT 2 (same command as tilt 1 — move together)
 *   Pin 11 = SHOOT mechanism servo (positional 0-180)
 *
 * How continuous rotation servos work:
 *   write(90)  → STOP
 *   write(<90) → spin one direction  (lower = faster)
 *   write(>90) → spin other direction (higher = faster)
 *
 * Protocol from Python:  P###T###S###\n   e.g. P090T090S000
 *   Values are SPEED commands centered on 90 (not positions).
 *   90 = stop.  Values away from 90 = spin.
 *   S000 = shoot idle, S001 = shoot active (angle step)
 *
 * Safety:
 *   - All received values are clamped to [STOP - MAX_SPEED, STOP + MAX_SPEED]
 *   - If no command received for TIMEOUT_MS, servos stop automatically
 *   - On startup pan + both tilts are stopped (90)
 *
 * Upload this sketch BEFORE running face_tracker.py.
 */

#include <Servo.h>   // Arduino Servo library: pan, dual tilt (same PWM), shoot

// ── Neutral / stop value ─────────
//    Tweak STOP_PAN / STOP_TILT by ±1-2 if a servo creeps at "90".
#define STOP_PAN    90   // Servo value that means "don't move" for pan (continuous rotation)
#define STOP_TILT   90   // Servo value that means "don't move" for tilt

// ── Speed cap (max offset from stop) ─────
#define MAX_SPEED   35   // Servo range is [90-25, 90+25] = [65, 115]; keep in sync with Python

// ── Safety timeout ───────────────
#define TIMEOUT_MS  250  // If no serial command for this long, stop both servos

// ── Pins ─────────────────────────
#define PAN_PIN      8   // Pan servo (left/right)
#define TILT1_PIN    9   // Tilt motor 1 — same speed as TILT2 every update
#define TILT2_PIN   10   // Tilt motor 2 — same speed as TILT1 every update
#define SHOOT_PIN   11   // Shoot servo (positional)
#define LED_PIN     13   // Onboard LED for status blink

// ── Shoot servo angles (positional servo, degrees) ────────────────
// Shoot behavior requested:
// - When target is in frame: move 10 degrees clockwise
// - Hold for max 3 seconds, then return even if target stays in frame
// Note: "clockwise" depends on mounting; if it's reversed, change +10 to -10.
#define SHOOT_REST_ANGLE  90
#define SHOOT_FIRE_ANGLE  (SHOOT_REST_ANGLE + 10)

// Keep the shoot servo fired for at least this long (ms) after a trigger.
// Requested: 3 seconds maximum.
#define SHOOT_HOLD_MS     3000

Servo panServo;
Servo tiltServo1;
Servo tiltServo2;
Servo shootServo;

int panSpeed;   // Current pan command value (65..115)
int tiltSpeed;  // Current tilt command value (65..115)
int shootActive = 0; // 0 = rest, 1 = fire position
unsigned long shootHoldUntil = 0; // millis() timestamp until which we must stay in FIRE
int lastShootSignal = 0; // Previous S value (0/1). Used to detect rising edge.

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
    panServo.write(STOP_PAN);
    tiltServo1.write(STOP_TILT);
    tiltServo2.write(STOP_TILT);
    shootActive = 0;
    shootHoldUntil = 0;
    lastShootSignal = 0;
    shootServo.write(SHOOT_REST_ANGLE);
}

void setup() {
    pinMode(LED_PIN, OUTPUT);          // LED pin is output
    Serial.begin(9600);                // Start serial at 9600 baud (match Python)
    while (Serial.available()) Serial.read();  // Flush any leftover bytes from before reset

    panServo.attach(PAN_PIN);
    tiltServo1.attach(TILT1_PIN);
    tiltServo2.attach(TILT2_PIN);
    shootServo.attach(SHOOT_PIN);

    stopAll();                         // Pan + both tilts stopped, shoot at rest
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
    const char* sPtr = strchr(cmd, 'S');  // Optional shoot flag (S000 / S001)

    int p = atoi(cmd + 1);             // Parse pan number after 'P' (e.g. "090" -> 90)
    int t = atoi(tPtr + 1);            // Parse tilt number after 'T' (e.g. "085" -> 85)
    int s = sPtr ? atoi(sPtr + 1) : 0;
    int shootSignal = (s != 0) ? 1 : 0;

    // Clamp to safe speed range so we never send values outside 65..115
    p = clampSpeed(p, STOP_PAN);
    t = clampSpeed(t, STOP_TILT);

    panSpeed  = p;                      // Store for possible use elsewhere
    tiltSpeed = t;

    // Write to servos; clamp again in case of any mistake
    int tiltCmd = clampSpeed(tiltSpeed, STOP_TILT);
    panServo.write(clampSpeed(panSpeed, STOP_PAN));
    tiltServo1.write(tiltCmd);
    tiltServo2.write(tiltCmd);

    // Shoot: trigger on rising edge only, then hold for max SHOOT_HOLD_MS.
    if (shootSignal && !lastShootSignal) {
        shootActive = 1;
        shootHoldUntil = millis() + SHOOT_HOLD_MS;
    }
    if (shootActive && (long)(millis() - shootHoldUntil) >= 0) {
        shootActive = 0;
    }
    lastShootSignal = shootSignal;

    shootServo.write(shootActive ? SHOOT_FIRE_ANGLE : SHOOT_REST_ANGLE);

    lastCmdTime = millis();             // Update timeout so we know we got a command
}
