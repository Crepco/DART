# DART — Serial Protocol

Wire format between the Python host (`scripts/yolo.py`) and the Arduino controller
(`arduino_servo_controller/arduino_servo_controller.ino`).

## Link

| Setting   | Value     |
|-----------|-----------|
| Transport | USB serial |
| Baud      | `115200`  |
| Framing   | One ASCII line per command, `\n`-terminated (`\r` ignored) |
| Direction | Host → Arduino (commands); Arduino → Host emits `READY` once after reset |

## Command format

```
P###T###F#[Z#]\n
```

| Field | Meaning      | Range / values                                   |
|-------|--------------|--------------------------------------------------|
| `P###`| Pan speed    | `065`–`115`, `090` = stop (centre ± `MAX_SPEED`) |
| `T###`| Tilt speed   | `065`–`115`, `090` = stop                        |
| `F#`  | Fire flag    | `0` = trigger rest, `1` = trigger fire           |
| `Z#`  | Zone-out flag (optional) | `0`/`1` → drives status **pin 12** (FlowState "not focusing" indicator). Independent of `F`. |

The pan/tilt servos are **continuous-rotation**: the value is a *speed*, where `090` is
neutral/stop and values above/below drive in either direction. The host clamps both to
`090 ± MAX_SPEED` (25) before sending; the Arduino also bounds-checks.

Examples:

| Packet         | Effect                                       |
|----------------|----------------------------------------------|
| `P090T090F0`   | Both motors stopped, trigger at rest.        |
| `P105T080F0`   | Pan + tilt moving, not firing.               |
| `P090T090F1`   | Motors stopped, trigger fires.               |

### Backward compatibility

If the `F#` token is absent (`P###T###\n`), the Arduino updates pan/tilt and leaves the
trigger untouched. If `Z#` is absent, pin 12 is left unchanged. New host code (the web UI)
always sends `F`; it sends `Z` only in FlowState mode.

## FlowState EEG (shared link)

Because one Uno R3 now runs **both** DART and FlowState, the BioAmp EXG Pill streams over
the *same* serial port, multiplexed with the servo commands.

| Direction | Line     | Meaning                                                       |
|-----------|----------|--------------------------------------------------------------|
| Host → R3 | `S1` / `S0` | Start / stop the EEG sample stream (off after reset).      |
| R3 → Host | `E####`  | One raw 10-bit ADC sample from `A0` (`0`–`1023`).            |

- Sample rate: `SAMPLE_RATE` = **250 Hz** on the R3 (must match `SERIAL_FS` in
  `scripts/web/focus.py`). At ~6 bytes/sample that is ~1.5 kB/s — well within 115200 baud
  alongside the 30 Hz command traffic.
- The host's `SerialLink` reader thread routes `E###` lines into the focus pipeline and
  `READY` to the handshake; servo commands flow the other way on the same port.
- Mode 2 ("Just DART") never sends `S1`, so the link stays command-only and identical to
  the original behaviour.

## Wiring (BioAmp EXG Pill → Uno R3)

| BioAmp pin        | R3 pin |
|-------------------|--------|
| Yellow (signal)   | `A0`   |
| Red (VCC)         | `5V` (or `3.3V`) |
| Black (GND)       | `GND`  |

Electrodes: two active on the forehead (above the eyebrows), one reference on an earlobe.
Status **pin 12** can drive an LED/buzzer that lights when the wearer zones out.

## Trigger (MG90)

| Constant             | Angle |
|----------------------|-------|
| `TRIGGER_REST_ANGLE` | `60`  |
| `TRIGGER_FIRE_ANGLE` | `150` |

`F1` drives the trigger servo to `TRIGGER_FIRE_ANGLE`; `F0` returns it to
`TRIGGER_REST_ANGLE`. The angles are defined on the Arduino; the host only sends the flag.

## Timing on the host

- **Send rate**: pan/tilt updates are rate-limited to `SEND_HZ` (30 Hz) and only sent when
  the command actually changes, decoupling serial traffic from YOLO inference frequency.
- **Heartbeat**: a packet is sent at least every `HEARTBEAT_INTERVAL` (0.4 s) to keep the
  watchdog satisfied even when idle.
- **Fire transitions**: when the fire flag flips `0↔1`, the packet bypasses the rate gate
  and the host calls `flush()` immediately so the OS does not buffer a fire/cease command.

## Safety watchdog (Arduino)

- `TIMEOUT_MS` = 500 ms. If no command arrives within this window, `stopAll()` runs:
  pan/tilt → neutral (`90`), trigger → `TRIGGER_REST_ANGLE`, and the internal `fireState`
  flag is cleared.
- Clearing `fireState` ensures a stray packet arriving on the same loop as a watchdog
  recovery cannot re-fire from stale state — firing requires a fresh explicit `F1`.

## Startup handshake

1. Arduino resets on serial open, initializes servos to neutral / trigger rest.
2. Arduino prints `READY`.
3. Host waits for `READY` (up to ~2 s), then sends an initial `P090T090F0` stop.
