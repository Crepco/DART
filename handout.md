# DART — Continuity Handout

> Handoff document for a fresh working session (any model) with zero prior
> context. Updated 2026-07-15 against the actual repo state.
> Branch `main`, tip `234b939`, **in sync with origin**. Test suite:
> **95 passed**. Rounds 3b + 4–4f (pursuit continuity) are landed and pushed.
> **IN FLIGHT: round 5 — ByteTrack `match_thresh 0.8→0.7` for motion ID-churn
> (§3.14) is in the tree UNCOMMITTED with TEMP debug flags ON, awaiting a
> hardware run.** `scripts/embeddings/authorized.pkl` is RESTORED
> (identities: Lateef, me) — enrolled people are never targeted; tracking
> tests need an unenrolled subject or the pkl moved aside (user's call).
>
> §1–7 = current state + map. §8–10 = how to debug NEW problems here: the
> log-forensics methods that solved the hardware defects, the invariants
> that bite, and the working agreements. **Read §8–10 before touching
> control, serial, or tracking code.**

---

## 1. What DART is

DART (Autonomous Detection, Aiming, and Real-Time Tracking) is a pan-tilt
tracking turret: YOLOv8n + ByteTrack track persons, YOLOv8n-face +
InsightFace `buffalo_sc` (ArcFace embeddings) classify each track
AUTHORIZED/UNAUTHORIZED against enrolled identities, and a PID control loop
streams servo commands over serial to an Arduino driving MG996R
continuous-rotation servos. Two equivalent frontends share the same control
math: a Flask web dashboard (`scripts/web/`) and a desktop cv2 window
(`scripts/main.py`).

**Standing constraint for ALL future work: fire/trigger logic is deliberately
deferred and untouched.** `update_fire_state` (scripts/main.py), the serial
`F` flag (`scripts/web/serial_link.py`, `scripts/core/serial_handler.py`),
the fire constants in config.py, and the Arduino sketch's trigger handling
are read-only telemetry sources. Do not add, wire, or modify trigger
behavior.

## 2. Current state of the working tree

**Committed & pushed** (`main` == `origin/main`):

- `234b939` chore: revert TEMP debug flags shipped in e9c5527.
- `e9c5527` "fixed tracking issue" — the USER's commit of the entire
  round-4–4f pursuit-continuity stack (§3.13: core/targeting.py, detector
  lock-holds, both loops, 24 new tests, docs). Committed by the user
  mid-session WITH the TEMP flags on (fixed by 234b939) and with their
  `STOP_TILT = 90` change included — 90 is now the committed tilt neutral
  even though the config comment still documents the calibrated 87 (user
  says tilt behaves; recalibrate/edit comment when convenient).
- `f58cbd8` fix(control): derivative kick on deadband crossings — per-axis
  PID resync (§3.12, hardware round 3b PASSED 2026-07-14; TEMP flags
  reverted).
- `6c41136` frontend redesign — the USER's commit: viewport-locked aspect-fit
  dashboard tweaks + the `TILT_PIN 9 → 11` .ino change (firmware must be
  flashed to match).
- `36f3a65` fix(hw): servo neutral calibration — `STOP_PAN/STOP_TILT = 87` +
  `scripts/calibrate_stop.py` (hardware-verified: 373 s dead-still).
- `32ccf72` fix(control): derivative-kick fix — `PID.resync()` +
  `D_KICK_LIMIT`, both loops + tests + docs (hardware-verified round 2).
- `56122f9` — the full dashboard rebuild. Its message ("relayout
  header/footer controls, color person brackets by auth verdict") understates
  it: that one commit contains the entire rebuild (16 files, +2103/−631,
  eventlog, enroll wizard backend, dashboard.html/css/js, wizard.js, tests).

**Uncommitted (round 5 IN FLIGHT — commit only after its hardware run passes):**

- `scripts/bytetrack_dart.yaml` — `match_thresh 0.8 → 0.7` (§3.14)
- `scripts/config.py` — `CONTROL_DEBUG/TRACK_DEBUG = True  # TEMP` for the
  round-5 capture — revert before committing
- `?? handout.md` — this file (untracked on purpose)

**Other state:** `scripts/embeddings/authorized.pkl` RESTORED by the user
(identities: Lateef, me). `run3b/run4/run4c/run4e/run4f.log` at repo root are
the round captures (gitignored via `*.log`); `run.log` is the live capture.

**`scripts/embeddings/authorized.pkl`: ABSENT** (deliberately deleted for
UNAUTHORIZED hardware testing; a backup copy sits at repo root as
`authorized.pkl`). Until restored (copy back) or re-enrolled via the wizard,
**everyone reads UNAUTHORIZED** — the classifier's verified safe default
(`ref_mat is None → best = -1.0` in `auth/classifier.py:_classify`).

## 3. What's done (chronological)

1. **`56e65d8` fix: runner killed at startup** — a second POST `/start`
   (double-click/stale tab) replaced the still-loading runner. `/start` is
   now idempotent under `_runner_lock`; only ERROR/STOPPED runners are
   replaced. *Fully verified (tests + hardware sessions since).*
2. **`ab745ab` feat: web Authorize Person flow** — enrollment from the web UI
   (live burst + photo upload) with the quality gate (face ≥80px,
   det_score ≥0.5, sharpness ≥60 — `config.py` ENROLL_*). *Fully verified.*
3. **`620bd57` perf: face-auth latency** — provider signal (`auth_provider`/
   `auth_ms` in `/state`), InsightFace warm-up, small-crop upscale
   (`AUTH_MIN_CROP`/`AUTH_UPSCALE`). Note: this dev laptop is **CPU-only**
   (torch `+cpu`, no CUDA onnxruntime) — ~20-30ms warm classify here is
   normal; a GPU deploy machine would want `onnxruntime-gpu`. *Verified.*
4. **`38b35e7` tune: overshoot tail** — settle-phase overshoot: centroid EMA
   `SMOOTH 0.90→0.70`, asymmetric command EMA (fast release), brake-aware
   slew (`SLEW_BRAKE_MULT 3.0`), frame gating via `read_seq()`. *Verified in
   software; the hardware checklist for it (§4) was never formally run.*
5. **`be5aa08` test:** pure-logic suite (start race, control shaping,
   enrollment gate).
6. **`72c94dd` fix: track identity lost on conf dips** — ultralytics filters
   dets at `conf` BEFORE ByteTrack, starving its low-conf second stage; now
   `TRACK_CONF=0.1` feeds the tracker (`scripts/bytetrack_dart.yaml`) while
   display/targeting stay at `PERSON_CONF=0.45`, plus time-based verdict
   grace (`TRACK_STATE_GRACE_S=8`). *Verified live on webcam; not yet in the
   original failure conditions.*
7. **`f4961d3` feat: Authorize anywhere** — enrollment works with DART
   running, starting (409), or stopped (`_standalone_frames` opens the camera
   directly). *Fully verified.*
8. **`9575ff5` polish, `950686f` comment cleanup, `e4675c6` docs rewrite**
   (README + docs/ to then-current state).
9. **`56122f9` feat(web): FULL dashboard rebuild** — replaced the two-page UI
   with a single-page terminal/HUD dashboard: MJPEG feed with restyled
   server-side HUD (corner brackets + verdict chips, **brackets colored by
   verdict**: red UNAUTHORIZED / gray UNKNOWN / green AUTHORIZED), telemetry
   panel (fps, target ID, error vector, pan/tilt + FPS sparkline), real event
   log (`web/eventlog.py` ring buffer + `GET /logs`, diff-based emitters in
   the runner loop), Enrolled Database panel, guided **10-shot enrollment
   wizard** (`/enroll/begin|capture|ping|finish|cancel`, 2 sets × 5 poses,
   glasses interstitial, same quality gate, **directional floor**: ≥1
   accepted front/left/right required, down/up skippable after 3 retries;
   300s idle timeout + 45s keep-alive ping; `begin` auto-cancels a prior
   session and reports `cancelled_previous`), turret controls in the header
   bar (Start/Stop toggle + CAM/PORT selects), status chips + port readout in
   the footer. New `/state` fields: `error_x/error_y/port/baud`. Old
   `POST /authorize` burst endpoint kept intact for tests/back-compat.
   *UI verified live (screenshots, real runner); wizard backend fully tested;
   wizard's interactive modal flow not yet human-walked end-to-end.*
10. **`32ccf72` fix(control): derivative kick across detection gaps.**
    First closed-loop hardware run (2026-07-11) showed full-speed "lurches".
    Log forensics (branch-aware PID replay matched every logged output to
    ≤0.05, proving shipped code == designed tune): coast/lost frames freeze
    the PID, `prev_error` goes stale, re-acquisition computes a fictitious
    multi-thousand-px/s derivative → command pinned at ±MAX_SPEED for 3–8
    frames. Round-1 baseline: **pan 20 bursts / 63 frames; tilt 6 / 12** in
    ~2.5 min. Fix: `PID.resync()` on every tracking resume (both loops) +
    `D_KICK_LIMIT=400 px/s` raw-derivative clamp. *FULLY hardware-verified
    (round 2, 15 fps closed loop): tilt 0 bursts, pan stale-gap kicks gone,
    every tracking resume exactly P-only; 5 unit tests pin the defect + fix.
    Note: at 15 fps genuine error rates reach p99≈560/max≈794 px/s, so the
    400 clamp trims some real transients (softer D at extremes — deliberate,
    revisit only if pursuit feels underdamped).*
11. **`36f3a65` fix(hw): servo neutral calibration.** At-rest drift: standing
    still, the turret never settled — tilt showed 23/23 one-signed deadband
    escapes with zero commanded output (plant creeping at `write(90)`).
    Ruled out KI=0 (deadband hides sub-30px error from the integral by
    construction), jitter (slow + one-signed), and D-kick/brake tails (drift
    during cmd=90). Fix: interactive `scripts/calibrate_stop.py` found true
    neutrals **87/87**; config STOP_PAN/STOP_TILT updated. *FULLY
    hardware-verified: 373 s dead-still on both axes (tilt previously could
    not rest 30 s).*
12. **`f58cbd8` fix(control): derivative kick on deadband crossings.**
    Round-3 forensics (2026-07-14, 79 s log, camera 1/COM6):
    both prior fixes verified holding (quiet stretches parked 87/87; 7/7 gap
    resumes P-only; 0 overshoot rebounds in 15 episodes), but the user's
    "pan still moving" isolated to a **deadband-crossing derivative kick** —
    err collapsing into INPUT_DEADBAND in one frame (−36.9 → 0.0) fed a fake
    −586 px/s derivative: pid pinned at −25 **at zero error**, command swept
    87→75→87 over ~0.9 s right after settling. Exit side = same mechanism
    (the one non-P-only resume, dev 20.6). Tilt immune in practice: its kick
    peaked at pid 10.8, EMA 2.9 < OUTPUT_DEADBAND 4 → cmd never moved. Fix:
    per-axis `PID.resync()` on any deadband boundary crossing (both loops,
    right after the `pid_live` block); `D_KICK_LIMIT` retained but now guards
    mid-track box jumps only. 3 new unit tests; **71 green**. *Hardware round
    3b PASSED (2026-07-14 evening, cam 1/COM6, 2816 CTRL frames of real
    movement): max |pid| over all err==0 frames = 0.00 both axes (was
    −25.00); 61/61 err 0→nonzero transitions P-only ≤ 0.05 (was 1/8
    failing); cmd back to STOP ≤3 frames after deadband entry (was ~10);
    0 stale-gap bursts (6 A-criterion flags all genuine dynamics: err moving
    400+ px/s in-track, P+D same direction); parked 87/87 for 287 s
    continuous; 0 rebounds in 61 episodes.*
13. **`e9c5527` rounds 4–4f: pursuit continuity** (user's commit message
    "fixed tracking issue"; flags cleanup in `234b939`). User complaint:
    turret loses target mid-pursuit, re-locks, sails past, reverses. Five
    instrumented hardware rounds, each mechanism measured before fixing —
    new module `scripts/core/targeting.py` (pure logic, no model imports):
    - **Conf-dip lock hold** (`resolve_lock`): a dip below PERSON_CONF
      unlocked a target ByteTrack never dropped ("box disappears, ID stays").
      Held while conf ≥ `TARGET_HOLD_CONF` 0.2, verdict UNAUTHORIZED, hold
      < `TARGET_HOLD_MAX_S` 2 s; held boxes re-surface in all_persons (HUD);
      `[TRACK] hold start/end` markers under TRACK_DEBUG.
    - **Faceless aim** (`AimSelector`): aiming required a face EVERY frame;
      now falls back to the person box's head proxy (`HEAD_PROXY_FRAC`)
      corrected by the offset learned while the face was visible.
    - **Handoff anchor** (4b): a held blur-distorted box teleported the aim
      112 px at a face→proxy switch (measured; the turret chased it at the
      rail ~1.5 s = the perceived "overshoot"). The handoff frame re-bases
      the offset — zero-jump by construction.
    - **`PAN_MAX_SPEED = 18`** (4b): track-loss rate 6–8× parked baseline
      while slewing (blur); pan PID capped below the 25 serial clamp.
    - **`AIM_STEP_LIMIT = 45` px/frame** (4c): after long gaps the drifted
      proxy→face correction measured 100–375 px in ONE frame; now ramped.
    - **Box-gated, nearest-first face pick** (4d): the old area-dominant
      global `select_face` walked the aim 100–270 px toward false-positive
      "faces" (the visible "circle shifts rapidly"); only faces inside the
      locked box (inflated ×1.5) count, nearest-to-anchor wins.
    - **`AIM_REACQUIRE_FRAMES = 3`** (4f): the self-sustaining loop — pan
      blur drops the face, a 1-frame flicker re-detection fired a full-speed
      correction, whose motion caused the next loss (0.46–0.92 s cycles,
      pid ±18 within 2–3 frames of regain). After a gap the anchored proxy
      holds until the face survives 3 consecutive frames.
    *Validated across runs 4→4f: every round strictly better; control-law
    criteria stayed green throughout (the 5 "non-P-only resumes" in 4f are
    a measurement artifact — at |err| > 240 px pure P exceeds the new 18
    cap and clamps; analysis B's expected value must be
    min(|KP·err|, PAN_MAX_SPEED) from now on). 95 tests.*
14. **UNCOMMITTED / IN FLIGHT: round 5 — ByteTrack ID churn under motion.**
    Round-4f log: 156 box losses (ALL while slewing, median gap 6 frames),
    **75/155 re-acquisitions returned a NEW ID** — each new ID bypasses the
    whole continuity stack (new lock, cold verdict). The yaml's documented
    escalation applied: `match_thresh 0.8 → 0.7`. *Pass bar (stated before
    the run): same-ID re-acquisition fraction ≥ 80% (was 52%); no
    cross-person ID merges (verdict flips in the event log would show it);
    §3.13 wins stay (few AIM clamps, no sub-second lose→correct cycles);
    control criteria green.*

## 4. What's left / not yet done

- ~~Hardware round 3b~~ — **PASSED 2026-07-14 evening** (§3.12), fix landed
  as `f58cbd8` and pushed. Detection also spot-verified live: person
  detected+tracked when present, UNAUTHORIZED verdict correct (pkl absent),
  no tracking without a face, parks 87/87 when frame empty.
- **Hardware round 5 — validate §3.14** (NEXT ACTION): movement script with
  TEMP flags on, capture `run.log`, measure same-ID re-acquisition fraction
  (analysis in §8.3-F). CAVEAT: the restored pkl authorizes the user — an
  AUTHORIZED person is never locked/chased, so the run needs an unenrolled
  subject OR the pkl temporarily moved aside (user's call). On pass: revert
  TEMP flags, pytest, commit the yaml change, push, update this file.
- ~~Dropout follow-up / overshoot re-judge~~ — **RESOLVED by rounds 4–4f**
  (§3.13): the perceived overshoot was, in order of discovery: the
  deadband-entry kick (round 3), the conf-dip unlock + face-gap stalls
  (round 4), aim teleports at face↔proxy handoffs (4b/4c), false-positive
  face yank (4d), and the blur→flicker→correction feedback loop (4f). The
  turret-side settle was clean all along once §3.12 landed. `SLEW_BRAKE_MULT`
  sweep stays off the list (rebound analysis keeps showing target-crossings,
  not turret swings).
- **Lit-room comparison run** (unchanged, still worth one session): the
  dim-light flicker floor (~4–12 losses/1000 frames parked) is environmental;
  a bright-light run with the same script would quantify how much of the
  remaining churn light alone removes.
- **STOP_TILT = 90 is now committed** (user's e9c5527) while the config
  comment still documents the calibrated 87 — reconcile when convenient
  (re-run `calibrate_stop.py` for tilt, or fix the comment).
- **`JTe5` serial handshake glitch** — once, after an Arduino replug, connect
  read garbage `JTe5` instead of `READY`; a stop/start retry cleared it.
  Workaround only, not root-caused. If it recurs, investigate
  `SerialLink.connect()`'s reset-wait/handshake window
  (`scripts/web/serial_link.py`). Do not normalize it.
- **Detection flicker in dim light is the dominant tracking-quality limiter**
  (not control): in evening room light tracks flap in/out ~1/s at conf
  0.27–0.90, so the loop only gets 1–4-frame tracking windows. Control
  degrades gracefully (small P nudges), but pursuit is slow. First lever:
  more light / brighter exposure. Code levers if needed: detector conf
  thresholds, ByteTrack thresholds.
- **Wizard interactive walkthrough** — the pkl now holds "Lateef" and "me",
  so the user may have already driven the wizard end-to-end; confirm with
  them before keeping this item.

## 5. Known hardware quirks (physical setup)

- **Servo neutrals are 87/87, not 90/90** — measured with
  `scripts/calibrate_stop.py` (2026-07-12), set in config `STOP_PAN/STOP_TILT`.
  `write(90)` creeps on both CR servos. The Arduino failsafe still parks at
  its own hardcoded 90/90, so slow idle creep while DART is *stopped* is
  expected and harmless; the servo trim pots are the hardware fix if that
  ever matters. Re-run the calibration after any servo swap.
- **Tilt servo wiring**: `TILT_PIN 11` (committed in `6c41136`; flashed and
  confirmed working by the user 2026-07-15). Tilt neutral is the user-set 90
  (committed), pan stays at the calibrated 87.
- **Turret camera is index 1** — `CAM_INDEX` defaults to 0 (laptop webcam).
  Always start with camera=1 (header CAM select, or `/start` form field).
  Starting on camera 0 with hardware attached runs the loop OPEN-loop and
  looks like wild overshoot (this happened; cost a session).
- **Arduino is on COM6** — config default `PORT` is "COM3". Pick COM6 in the
  header PORT select (it enumerates as "USB Serial Port (COM6)").
- **USB replug settle**: after (re)plugging camera/Arduino, wait ~10s before
  starting. Camera 1 can fail all 5 open retries (3s window) right after USB
  churn or a `/cameras` probe; a retry after a pause works. Flapping
  availability = check the physical cable first.
- **`JTe5` handshake**: if serial comes up in `preview` with a garbage
  `[ARDUINO]` line in the log, stop and start once — it cleared the one time
  it happened.
- **COM port can vanish outright** (seen 2026-07-14 after a hard kill of the
  server process): `Could not open COM6 ... FileNotFoundError` with ZERO
  ports enumerated = the board dropped off USB — replug + ~10 s settle.
  Distinct from JTe5 (port present, handshake garbage).
- Baud 115200 both sides. `serial: "preview"` in `/state` means no servo
  output (port missing or handshake failed) — the loop still runs.

## 6. How to proceed (in order)

1. **Hardware round 5** (§4 first bullet): validate `match_thresh 0.7`,
   then revert TEMP flags, pytest, commit, push.
2. Normal start: hardware connected → wait ~10 s → `python scripts/run_web.py`
   → http://127.0.0.1:5000 → header CAM **1**, PORT **COM6** → Start Turret →
   confirm SYSTEM ENGAGED chip + `[ARDUINO] READY` in the log.
3. Lit-room comparison run when convenient (§4).
4. Resume normal use; reconcile the STOP_TILT comment (§4 last bullet).

## 7. Where to find things

| area | path |
|---|---|
| All constants (camera/servo/PID/serial/enroll/auth) | `scripts/config.py` |
| Web app (Flask routes, enrollment endpoints, security note) | `scripts/web/app.py` |
| Web runner (loop, state dict, HUD drawing, event emitters) | `scripts/web/runner.py` |
| Event ring buffer / enrollment sessions | `scripts/web/eventlog.py`, `scripts/web/enroll_session.py` |
| Serial link (READY handshake, command bytes) | `scripts/web/serial_link.py` |
| Dashboard frontend | `scripts/web/templates/dashboard.html`, `scripts/web/static/dashboard.{css,js}`, `scripts/web/static/wizard.js` |
| Desktop twin (shared control math, fire gate — do not touch fire) | `scripts/main.py` |
| PID + shaping helpers (resync/kick clamp live here) | `scripts/core/pid.py` |
| Camera / detector / tracking | `scripts/core/camera.py`, `scripts/core/detector.py`, `scripts/bytetrack_dart.yaml` |
| Auth (classifier, enrollment, per-track verdict state) | `scripts/auth/classifier.py`, `scripts/auth/enroll.py`, `scripts/auth/track_manager.py` |
| Identities db | `scripts/embeddings/authorized.pkl` |
| Arduino sketch (READ-ONLY per fire constraint) | `arduino/arduino_servo_controller/arduino_servo_controller.ino` |
| Docs | `README.md`, `docs/ARCHITECTURE.md` (control law incl. resync/kick clamp, tuning guide), `docs/SERIAL_PROTOCOL.md`, `docs/FACE_TRACKER.md` |
| Tests (68) | `tests/` — run `python -m pytest tests/` from repo root |
| Launchers | `scripts/run_web.py`, `run_dart_web.bat` (web); `python scripts/main.py` (desktop) |
| Servo neutral calibration | `scripts/calibrate_stop.py --port COM6` (turret on, DART stopped) |

Security note (also in `scripts/web/app.py`): the web server binds
127.0.0.1 by design; `/authorize` and `/enroll/*` ARE the turret's access
control and carry no auth of their own — never bind 0.0.0.0.

## 8. Debugging playbook — how the solved problems were actually solved

All three hardware defects (§3 items 10–12) were diagnosed offline from
CONTROL_DEBUG log forensics, not by watching the turret or tuning by feel.
Every "obvious" first guess (bad tune, needs KI, more smoothing) was wrong,
and the measurements said so before any constant changed. Reuse the method.
Note: three control-law layers now touch the same frames (gap resync,
deadband-crossing resync, D_KICK_LIMIT) — if a new symptom appears, first
check whether it's an interaction between these before hypothesizing a
fourth mechanism.

### 8.1 Standard forensics session

1. Confirm the setup first: camera **1**, port **COM6** (§5). An open-loop
   run on camera 0 perfectly mimics "wild overshoot" and invalidated a whole
   round of data once.
2. Flip `CONTROL_DEBUG = True` in `scripts/config.py` (and `TRACK_DEBUG =
   True` if track-ID behavior is in question). Mark the flips `# TEMP`.
3. Capture: `python scripts/run_web.py *> run.log` (PowerShell; redirects all
   streams), reproduce the symptom for ~2 min, stop the runner.
4. Parse and measure (§8.3) BEFORE changing any constant.
5. Revert the TEMP flags before committing anything.

Healthy baseline: `dt ≈ 66 ms` (15 fps) in the `[CTRL]` lines; serial sends
at 30 Hz independently of the frame rate.

### 8.2 Log formats (exact)

`[CTRL]` — one line per control frame (grep `[CTRL]` in
`scripts/web/runner.py` / `scripts/main.py` — line numbers drift):

```
[CTRL] dt=  66ms pan(err= +42.0 pid= +3.15 ema= +2.80 slew= +2.80 cmd= 92) tilt(err=  +0.0 pid= +0.00 ema= +0.00 slew= +0.00 cmd= 87)
```

- `err` — post-INPUT_DEADBAND pixel error. `0.0` means "inside the 30 px
  deadband OR no target this frame" — both zero it.
- `pid` — raw PID output before EMA/slew; sign already includes `INVERT_*`.
- `ema` / `slew` — after command smoothing / after slew limiting.
- `cmd` — final servo value (`STOP_* +` rounded, output-deadbanded offset).

Working parse regex (from the round-2 analysis):

```python
pat = re.compile(
    r"\[CTRL\] dt=\s*(\d+)ms "
    r"pan\(err=\s*([+-]?\d+\.?\d*) pid=\s*([+-]?\d+\.?\d*).*?"
    r"tilt\(err=\s*([+-]?\d+\.?\d*) pid=\s*([+-]?\d+\.?\d*)")
```

`[TRACK]` — printed only on ID-set changes:
`[TRACK] +3(0.87) -2 active=[1, 3]` (`+id(conf)` appeared, `-id` dropped).

`[ARDUINO] READY` — handshake OK. Any other `[ARDUINO] <garbage>` line is
the §5 handshake glitch.

### 8.3 The five analyses (exact criteria)

The analysis scripts live in session-temp scratchpads and don't survive; each
is ~40 lines to rewrite from the criteria below (parse regex in §8.2).

**A. Saturation-burst — full-speed lurch class.** A frame is a D-spike if
`|pid| ≥ 24.9` (MAX_SPEED 25 minus rounding) while `|KP·err| < 8`
(pan KP 0.075, tilt KP 0.04) — command pinned but P alone can't explain it.
Count bursts as starts of consecutive runs. History: round 1 pan 20 bursts /
tilt 6 in ~2.5 min; post-fix round 2: ZERO stale-gap bursts (every remaining
saturation had large `KP·err` or braking context — genuine dynamics).

**B. Resume P-only proof — regression check for BOTH resync mechanisms. Run
this after ANY edit to `pid.py` or the loop's branch structure.** Every frame
where an axis err goes 0 → nonzero must be P-only:
`|pid − KP·err·INVERT| ≤ 0.05` (INVERT_PAN = −1, INVERT_TILT = +1). Since
§3.12 this covers ALL such transitions — gap resumes AND deadband exits
(round 3 pre-fix: 7/8 passed, the failure was a deadband exit at dev 20.6).
Companion check for the entry side: max |pid| over all err==0 frames must be
0.00 — any nonzero pid at zero error is a derivative firing with no target
offset (round 3 pre-fix: pinned at 25.00).

**C. Escape analysis — at-rest drift / won't settle.** Count err 0→nonzero
transitions per axis; record each escape's preceding quiet duration and sign.
Interpretation:
- One-signed escapes with `cmd == STOP_*` through the whole preceding second
  → the PLANT moves on its own (servo neutral bias / mechanics) → re-run
  `calibrate_stop.py`; not a control-law problem.
- Bidirectional, fast, after short quiets → measurement jitter → `SMOOTH`
  0.70→0.80 first (per config comment), or deadband.
History: tilt was 23/23 positive (incl. after 18–30 s of quiet) → 87/87 trim
→ **373 s dead-still on both axes**.

**D. Legit-derivative distribution — required BEFORE touching
`D_KICK_LIMIT`.** Compute `|Δerr/dt|` over consecutive in-track frames (both
errs nonzero). Measured at 15 fps: p99 ≈ 560, max ≈ 794 px/s — the 400 clamp
already trims the top ~1% of genuine transients (deliberate softer-D trade).
If pursuit ever feels underdamped, re-measure this first; raise the clamp
only if the distribution says so.

**E. Rebound episodes — the overshoot detector.** An episode = a contiguous
nonzero-err run per axis; a rebound = the err sign flipping *within* one
episode (the turret drove past center and came back). Report episodes,
sign-cross count, and the post-flip peak (px past center). Round 3: 0
rebounds in 15 episodes on both axes — proof there is no settle overshoot
and `SLEW_BRAKE_MULT` needs no sweep. CAVEAT (learned round 4): with a
moving target, sign flips are usually the TARGET crossing center, not the
turret — check cmd at the flip (cmd ≈ STOP means no turret momentum, so it
wasn't overshoot) and correlate with `[AIM]` events before believing E.

**F. ID-churn / feedback-loop metrics (round 4+; needs TRACK_DEBUG +
CONTROL_DEBUG).** From `[TRACK] +id/-id` pairs: per re-acquisition, did the
SAME id return (fraction; round-4f baseline 80/155 = 52%) and the gap length;
from `[AIM]` lines: face->proxy losses binned by |pan cmd−87| at the loss
(loss rate per 1000 frames per speed bin; parked baseline ~4–12), full
lose→regain→next-loss cycle times (sub-second cycles = the feedback loop),
and step-clamp count. Round-4f references: 6 sub-second cycles pre-window;
face-loss ~6× baseline at ANY nonzero pan speed in dim light. NOTE for
analysis B here: expected P-only is min(|KP·err|, PAN_MAX_SPEED) — pure P
clamps at the 18 cap for |err| ≳ 240 px.

### 8.4 Symptom → first check

| symptom | check first | then |
|---|---|---|
| wild overshoot / spins past target | camera index (0 = open loop!) | §8.3-A |
| full-speed lurch at re-acquisition | §8.3-B (resync regression) | `D_KICK_LIMIT` still 400? |
| twitch/sweep right AFTER settling on target | §8.3-B entry check: any pid ≠ 0 at err==0 → deadband-crossing resync regressed (§3.12) | `D_SMOOTH` tail |
| never settles; slow one-way micro-corrections | §8.3-C → recalibrate neutrals | servo trim pots |
| clean overshoot-and-return on big moves | §8.3-E rebounds (round 3: zero) → only then `SLEW_BRAKE_MULT` sweep 2/3/4 | `CMD_SMOOTH` |
| track IDs flap ~1/s | light level (dominant known limiter) | `bytetrack_dart.yaml` thresholds (§4) |
| verdict slow to flip (~3 s) | expected: DEBOUNCE_COUNT=5 × classify-every-10-frames | — |
| everyone UNAUTHORIZED | pkl present at `scripts/embeddings/`? (§2) | SIMILARITY_THRESHOLD last |
| `serial: "preview"` in /state | COM6 selected? board powered? | JTe5 → stop/start once (§5) |
| camera won't open / flaps | ~10 s USB settle; no `/cameras` probe right before start | physical cable |
| feed freezes after ~60 s | `CAM_BACKEND` still DSHOW? (MSMF stall) | — |
| corrupt tiled/gray frames | `CAM_FOURCC` forced to MJPG? keep `None` | USB bandwidth |
| auth slow on this laptop | CPU-only machine — expected (§3.3) | GPU deploy: onnxruntime-gpu |

### 8.5 Frontend verification without a human browser

Headless Chrome renders the live dashboard, including real MJPEG frames:

```
chrome --headless=new --screenshot=out.png --window-size=1920,1080 --virtual-time-budget=4000 http://127.0.0.1:5000
```

Used to verify the viewport-locked layout at 1920×1080 / 1536×864 / 1366×768.

## 9. Invariants & gotchas (non-obvious; each bit us or nearly did)

1. **Servos are velocity devices.** MG996R continuous-rotation:
   `write(90±x)` is a speed, not an angle. Every command is `STOP_* +
   offset`; there is no position feedback anywhere in the system.
2. **INPUT_DEADBAND zeroes error BEFORE the PID**, so the integral never
   sees sub-30 px offsets — KI cannot correct small steady-state error *by
   construction*. Restoring KI is only meaningful for a persistent >30 px
   offset (see the KI=0 rationale comment in config.py).
3. **The PID must be re-anchored across ANY error discontinuity.** Two
   guards exist (grep `resync` in `runner.py`/`main.py`): the `pid_live`
   flag (set False in coast/lost branches — `update()` isn't called there,
   `prev_error` goes stale — resync-then-True on tracking resume), and the
   per-axis deadband-crossing resync (§3.12: entering/exiting INPUT_DEADBAND
   steps the error by up to 30 px in one frame while the PID stays live).
   **Any new loop branch that skips `update()`, or any new place the error
   is zeroed/stepped, must resync or the lurch/twitch comes back.**
4. **ultralytics filters detections at `conf` BEFORE ByteTrack sees them.**
   The tracker must be fed `TRACK_CONF=0.1`; display/targeting re-filter at
   `PERSON_CONF=0.45` afterwards. "Tidying" the track() conf up to 0.45
   reintroduces the 1-second ID-flap bug (§3.6).
5. **Firmware failsafe:** 500 ms of host silence → `stopAll()` at the .ino's
   OWN hardcoded 90/90 + trigger rest. Consequences: any host-side tool that
   holds a pose must resend at ≥2 Hz (`calibrate_stop.py` uses 5 Hz), and
   slow idle creep while DART is STOPPED is expected and harmless (87 ≠ 90).
6. **Trigger-angle mirror MISMATCH — do not "fix" blindly.** The .ino has
   `REST=60 / FIRE=150`; config.py's parity copies say `REST=150 / FIRE=60`.
   Runtime-harmless: trigger angles live ONLY on the Arduino — the host
   sends just the F0/F1 flag — so config's copies are documentation, not
   behavior. If fire work is ever sanctioned, verify against the physical
   trigger geometry before trusting either file. Until then the fire path
   stays untouched (§1).
7. **`SerialLink.close()` sends the config STOP values first** — right for
   the runner, wrong for calibration (it would stomp the candidate neutral);
   `calibrate_stop.py` deliberately closes `link.ser` raw instead.
8. **Opening the COM port asserts DTR → resets the Uno** (~700 ms boot →
   one-shot `READY`; the link is write-only afterwards). `build_command`
   clamps to STOP ± MAX_SPEED, so no host bug can command past ±25.
9. **Missing/empty pkl → everyone UNAUTHORIZED** (`ref_mat is None → best =
   −1.0` in `auth/classifier.py`). Safe default, not a bug.
10. **Per-track identity NAMES do not exist** — the classifier discards the
    matched name at verdict time; only verdict + track ID survive. Don't
    promise names in the UI without an `auth/` change.
11. **Camera-stack landmines:** MSMF stalls USB cams after ~60 s (hence
    `CAM_BACKEND=DSHOW` on Windows); forcing MJPEG fourcc decodes USB
    glitches into corrupt-but-"valid" frames (keep `CAM_FOURCC=None`); the
    control loop gates on `read_seq()` so a stale frame can't double-step
    the PID.
12. **`/start` is idempotent** under `_runner_lock` — only ERROR/STOPPED
    runners are replaced (§3.1). Don't "fix" double-POST symptoms anywhere
    else.
13. **Event log** is a `deque(maxlen=200)` with a monotonic `seq`; poll
    `GET /logs?since=N`. **Enrollment** allows ONE active session: `begin`
    auto-cancels a prior one and reports `cancelled_previous`; 300 s idle
    timeout; the wizard pings every 45 s to stay alive.
14. **Every aim-point change must flow through `AimSelector`**
    (`core/targeting.py`): box-gate → reacquire window → offset/anchor →
    step clamp, in that order. Rounds 4b–4f exist because aim discontinuities
    reach the PID as fake velocity; any new aim source (new detector, manual
    override, multi-target switch) that bypasses the selector reintroduces
    the teleport/feedback-loop class. Same spirit as invariant 3 (PID
    resync): targets may not TELEPORT, errors may not STEP.
15. **An AUTHORIZED verdict makes a person invisible to targeting** (only
    confirmed-UNAUTHORIZED persons are candidates). With the pkl restored,
    enrolled people (currently: Lateef, me) are never locked — pursuit tests
    need an unenrolled subject or the pkl moved aside. Not a bug.

## 10. Working agreements (how this project is actually run)

- **The user drives all physical steps** (plug/unplug, power, calibration
  keys, standing in frame). Ask, then WAIT for their "done" — never start
  the server assuming hardware state, and never pick camera/port for a
  hardware run without confirming camera 1 + COM6.
- **The blaster stays unloaded** for all test sessions.
- **Never restore or recreate `authorized.pkl` unprompted** — its absence is
  a deliberate test condition; the user decides when identities come back.
- **TEMP debug flags:** mark flips with `# TEMP`; always revert before
  committing.
- **Measure, then tune.** State numeric pass criteria BEFORE a hardware test
  (e.g. "0 long-quiet escapes in 60 s") and land fixes only after they pass
  on hardware. Both landed fixes followed this; every skipped-measurement
  instinct in these sessions turned out wrong.
- **Commits are separately-revertable logical units** — the control fix and
  the neutral trim were split even though both touch config.py (stage one by
  temporarily reverting the other's hunk).
- **Keep THIS file current.** Update it after every landed round, and verify
  its claims against the repo (git log/status, config.py) before trusting or
  extending it — a stale handout is worse than no handout.
- **The fire/trigger constraint (§1) is standing** and survives everything
  above.
