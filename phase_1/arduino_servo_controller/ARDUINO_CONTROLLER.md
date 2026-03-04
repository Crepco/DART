# Phase 1 — Arduino Sketch: `arduino_servo_controller.ino`

Arduino firmware for the face-tracking turret. It receives pan/tilt **speed** commands from the Python script over USB serial and drives two **continuous-rotation** MG996R servos (pan and tilt). No position feedback — the servos spin at a speed proportional to how far the command is from 90.

---

## Overview

| Item | Detail |
|------|--------|
| **Sketch** | `arduino_servo_controller.ino` |
| **Role** | Parse serial commands and drive pan/tilt servos; safety timeout and clamping |
| **Servo type** | Continuous rotation MG996R |
| **Library** | Arduino `Servo.h` |

**Upload this sketch before** running `face_tracker.py`. Pin numbers and constants must match the Python script.

---

## Pin assignments

| Pin | Function | Notes |
|-----|----------|--------|
| **9** | TILT servo (up/down) | Signal wire only; power from external supply |
| **10** | PAN servo (left/right) | Signal wire only |
| **13** | Onboard LED | Status blink on startup |

Power the MG996R servos from a 6 V supply (e.g. 6 V 2 A); do not power them from the Arduino 5 V rail for sustained use.

---

## Constants (must match Python)

| Constant | Value | Meaning |
|----------|--------|---------|
| `STOP_PAN` | 90 | Servo value for “stop” on pan axis |
| `STOP_TILT` | 90 | Servo value for “stop” on tilt axis |
| `MAX_SPEED` | 25 | Max offset from 90 → allowed range [65, 115] |
| `TIMEOUT_MS` | 250 | If no serial command for this long, both servos stop |
| `TILT_PIN` | 9 | Tilt servo signal pin |
| `PAN_PIN` | 10 | Pan servo signal pin |
| `LED_PIN` | 13 | LED for blink |

If a continuous-rotation servo creeps at 90, adjust `STOP_PAN` or `STOP_TILT` by ±1–2 in the Arduino sketch (and optionally in Python so the “stop” command stays consistent).

---

## Continuous-rotation servo behavior

- **`write(90)`** → stop (neutral).
- **`write(< 90)`** → spin one direction; lower value = faster.
- **`write(> 90)`** → spin the other direction; higher value = faster.

So the values are **speed** commands centered at 90, not target angles. The Python script sends values in the range 065–115; the Arduino clamps received values to the same range.

---

## Serial protocol (Python → Arduino)

- **Baud rate:** 9600 (must match Python `BAUD_RATE`).
- **Message format:** one line per command: `P###T###` followed by newline (e.g. `P090T085\n`).
  - `P` + 3-digit pan value (065–115).
  - `T` + 3-digit tilt value (065–115).
- **Arduino → Python:** Arduino sends one line `READY\n` from `setup()` so the script knows the board has reset and is ready for commands.

---

## Parsing (line-based)

- Incoming bytes are read one at a time in `loop()`.
- Characters are stored in a buffer `buf[32]` until a newline `\n` is received.
- Carriage return `\r` is ignored (so `\r\n` from some terminals is fine).
- When `\n` is seen, the buffer is null-terminated and `parseCommand(buf)` is called; then the buffer index is reset.
- If the buffer would overflow, it is cleared and parsing restarts (no partial commands).

---

## parseCommand() — logic

1. **Format check:** command must start with `'P'`; there must be a `'T'` somewhere (e.g. `P090T085`).
2. **Numbers:** pan = `atoi(cmd + 1)`, tilt = `atoi(pointer after 'T')`.
3. **Clamp:** pan and tilt are clamped with `clampSpeed(v, STOP_*)` to the range `[STOP - MAX_SPEED, STOP + MAX_SPEED]` (65..115 for default 90 and 25).
4. **Apply:** store in `panSpeed` / `tiltSpeed`, then `panServo.write(clampSpeed(panSpeed, STOP_PAN))` and `tiltServo.write(clampSpeed(tiltSpeed, STOP_TILT))`.
5. **Timeout:** set `lastCmdTime = millis()` so the safety timeout is reset.

---

## clampSpeed(v, center)

- `lo = center - MAX_SPEED`, `hi = center + MAX_SPEED`.
- Returns `constrain(v, lo, hi)` so invalid or corrupted serial data never drives the servos out of the safe range.

---

## Safety timeout

- In `loop()`, if `millis() - lastCmdTime > TIMEOUT_MS` (250 ms by default), `stopAll()` is called.
- So if the laptop disconnects, Python crashes, or serial stops, the turret stops after ~250 ms instead of holding the last command indefinitely.

---

## stopAll()

- Sets `panSpeed = STOP_PAN`, `tiltSpeed = STOP_TILT`.
- Calls `panServo.write(STOP_PAN)` and `tiltServo.write(STOP_TILT)`.
- Used at startup and whenever the timeout fires.

---

## setup() — sequence

1. Set `LED_PIN` as output.
2. `Serial.begin(9600)` and flush any bytes already in the buffer (e.g. from before reset).
3. `panServo.attach(PAN_PIN)`, `tiltServo.attach(TILT_PIN)`.
4. `stopAll()` so both servos are stopped from the first moment.
5. `delay(500)` to let servos settle.
6. Blink LED 3 times (150 ms on/off) so you can see the board is running.
7. `Serial.println("READY")` so the Python script can synchronize.
8. Set `lastCmdTime = millis()` to start the timeout timer.

---

## loop() — structure

1. **Read serial:** while bytes are available, read into `buf` until `\n`; on newline, call `parseCommand(buf)` and reset the buffer.
2. **Timeout:** if no valid command has been received for `TIMEOUT_MS`, call `stopAll()`.

No other blocking delays in `loop()`, so the board stays responsive to serial and timeout.

---

## Helper: blinkLED(times, ms)

- Blinks the onboard LED `times` times, with `ms` on and `ms` off.
- Used in `setup()` for a simple “sketch is running” indication.

---

## Wiring summary

- **Arduino** — USB to laptop for serial; 5 V and GND only for the board (and LED).
- **Servos** — signal to pins 9 and 10; VCC and GND to an external 6 V supply (common GND with Arduino recommended).
- **LED** — onboard LED on pin 13 (no extra wiring).

Keep servo power supply separate from Arduino’s 5 V to avoid brownouts and resets when the servos move.

---

## Matching Python and Arduino

Keep these in sync between `face_tracker.py` and this sketch:

- **Baud rate:** 9600.
- **Protocol:** `P###T###\n`, 3 digits each, range 065–115.
- **STOP value:** 90 (or the same tuned value on both sides).
- **MAX_SPEED:** 25 (so range 65..115).

Then the Python script’s “stop” and “max speed” behavior will match what the Arduino applies and clamps.
