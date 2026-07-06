# DART — Serial Protocol

Wire format between the Python host (`scripts/web/serial_link.py` for the web UI,
`scripts/core/serial_handler.py` for the desktop window) and the Arduino controller
(`arduino/arduino_servo_controller/arduino_servo_controller.ino`).

## Link

| Setting   | Value     |
|-----------|-----------|
| Transport | USB serial |
| Baud      | `115200`  |
| Framing   | One ASCII line per command, `\n`-terminated (`\r` ignored) |
| Direction | Host → Arduino (commands); Arduino → Host emits `READY` once after reset |

## Command format

```
P###T###F#\n
```

| Field | Meaning      | Range / values                                   |
|-------|--------------|--------------------------------------------------|
| `P###`| Pan speed    | `065`–`115`, `090` = stop (centre ± `MAX_SPEED`) |
| `T###`| Tilt speed   | `065`–`115`, `090` = stop                        |
| `F#`  | Fire flag    | `0` = trigger rest, `1` = trigger fire           |

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
trigger untouched. New host code (the web UI) always sends `F`.

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
3. Host waits for `READY` (up to 5 s), then sends an initial `P090T090F0` stop. If no
   `READY` arrives, the host closes the port and runs preview-only (no servo output).
