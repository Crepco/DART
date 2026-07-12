"""Headless DART tracking runner for the web UI.

This is the same camera -> YOLO -> PID -> serial pipeline as scripts/main.py, but with
no cv2 window. Instead it runs in a background thread and publishes:

  * the latest annotated frame as JPEG bytes  (-> MJPEG video feed)
  * a state dict                              (-> /state JSON the page polls)

Behaviour matches the desktop turret: it locks UNAUTHORIZED persons and fires on a
dwelled lock (auth gate). The lock/dwell/PID/serial-cadence logic is shared with main.py.
"""

from __future__ import annotations

import os
import threading
import time

import cv2

from config import *  # noqa: F401,F403  (camera/servo/PID/serial/colour constants)
from core import CameraStream, YOLODetector, PID
from core.detector import select_face
from core.pid import asym_ema, slew_step
from main import update_fire_state, apply_output_deadband, AUTH_COLS

from .eventlog import log as event_log
from .serial_link import SerialLink


def _fire_label_dart(fire, locked, locked_face_auth):
    if fire:
        return "FIRING", COL_FIRING
    if locked and locked_face_auth == "UNAUTHORIZED":
        return "ARMED", COL_ARMED
    if locked and locked_face_auth == "AUTHORIZED":
        return "SAFE/AUTH", COL_AUTHORIZED
    return "SAFE", (120, 120, 120)


# Web-HUD palette (BGR) — local to this module so the desktop HUD (main.py)
# keeps the shared config.py COL_* colours untouched. Person brackets + chips
# use AUTH_COLS per verdict: red UNAUTHORIZED / gray UNKNOWN / green AUTHORIZED
# must survive the green theme.
HUD_GREEN = (122, 227, 78)    # primary lines/text (theme green, ~#4ee37a)
HUD_DIM   = (96, 132, 92)     # secondary lines / non-tracking text
HUD_CHIP  = (12, 22, 10)      # label-chip fill (near-black green)
_FONT     = cv2.FONT_HERSHEY_SIMPLEX


def _corner_brackets(frame, x1, y1, x2, y2, col, thickness=1):
    """Mockup-style box: 8 short lines at the corners instead of a rectangle."""
    w, h = x2 - x1, y2 - y1
    arm = max(4, min(28, int(0.18 * min(w, h)), w // 2, h // 2))
    for (cx, cy, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1),
                             (x1, y2, 1, -1), (x2, y2, -1, -1)):
        cv2.line(frame, (cx, cy), (cx + dx * arm, cy), col, thickness, cv2.LINE_AA)
        cv2.line(frame, (cx, cy), (cx, cy + dy * arm), col, thickness, cv2.LINE_AA)


def _label_chip(frame, x, y_baseline, text, col, w_f):
    """Filled dark chip with a 1px border; anchored above y_baseline, clamped
    on-screen and below the 32px status bar (a box touching the frame top
    otherwise pushes its chip into the bar text)."""
    (tw, th), _ = cv2.getTextSize(text, _FONT, 0.45, 1)
    pad = 5
    x = max(2, min(x, w_f - tw - 2 * pad - 2))
    yb = max(th + 2 * pad + 36, y_baseline)
    cv2.rectangle(frame, (x, yb - th - 2 * pad), (x + tw + 2 * pad, yb),
                  HUD_CHIP, -1)
    cv2.rectangle(frame, (x, yb - th - 2 * pad), (x + tw + 2 * pad, yb),
                  col, 1, cv2.LINE_AA)
    cv2.putText(frame, text, (x + pad, yb - pad), _FONT, 0.45, col, 1,
                cv2.LINE_AA)


def _draw_hud(frame, status, pan_cmd, tilt_cmd, smooth_cx, smooth_cy,
              all_persons, faces, track_id, w_f, h_f, error_x, error_y,
              fire_label, fire_col, auth_statuses):
    """Annotate the frame: person/face brackets, verdict chips, centroid, top
    status bar, reticle, and lock ring. Same inputs and information as before —
    only the drawing style changed (dashboard terminal/HUD look). Kept
    self-contained so the proven desktop path (main.py) is untouched."""
    tracking   = (status == "TRACKING")
    bar_col    = HUD_GREEN if tracking else HUD_DIM
    cx_f, cy_f = w_f // 2, h_f // 2

    for (x1, y1, x2, y2, tid) in all_persons:
        auth_status = auth_statuses.get(tid, "UNKNOWN")
        col         = AUTH_COLS[auth_status]   # brackets + chip share the verdict colour
        is_target   = (tid == track_id)
        _corner_brackets(frame, x1, y1, x2, y2, col, 2 if is_target else 1)
        _label_chip(frame, x1, y1 - 8, f"ID:{tid} {auth_status}", col, w_f)

    for (x1, y1, x2, y2) in faces:
        _corner_brackets(frame, x1, y1, x2, y2, HUD_GREEN, 1)

    if smooth_cx is not None and tracking:
        cv2.line(frame, (int(smooth_cx), int(smooth_cy)), (cx_f, cy_f),
                 HUD_DIM, 1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 4, HUD_GREEN, -1,
                   cv2.LINE_AA)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_f, 30), (8, 12, 8), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    cv2.line(frame, (0, 31), (w_f, 31), HUD_DIM, 1, cv2.LINE_AA)

    tid_label = f"  ID={track_id}" if track_id is not None else ""
    bar_text  = (f"[{status}]{tid_label}   PAN {pan_cmd:3d}  TILT {tilt_cmd:3d}"
                 f"   ERR ({int(error_x):+d},{int(error_y):+d})")
    cv2.putText(frame, bar_text, (10, 20), _FONT, 0.5, bar_col, 1, cv2.LINE_AA)
    (fw, _), _ = cv2.getTextSize(fire_label, _FONT, 0.5, 2)
    cv2.putText(frame, fire_label, (w_f - fw - 10, 20), _FONT, 0.5, fire_col, 2,
                cv2.LINE_AA)

    arm = 10 if tracking else 18
    cv2.line(frame, (cx_f - arm, cy_f), (cx_f + arm, cy_f), HUD_GREEN, 1, cv2.LINE_AA)
    cv2.line(frame, (cx_f, cy_f - arm), (cx_f, cy_f + arm), HUD_GREEN, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx_f, cy_f), arm + 8, HUD_GREEN, 1, cv2.LINE_AA)
    # the lock ring stays fire-coloured — ARMED/FIRING must remain distinct
    cv2.circle(frame, (cx_f, cy_f), LOCK_ON_RADIUS, fire_col, 1, cv2.LINE_AA)


class DartRunner:
    """Owns the camera, detector, serial link, and loop thread."""

    def __init__(self, camera: int | None = None, port: str | None = None):
        self.camera = CAM_INDEX if camera is None else camera
        self.port   = PORT if port is None else port

        self._t0         = time.monotonic()   # for the loading-progress banner
        self._lock       = threading.Lock()
        self._cam_lock   = threading.Lock()   # guards hot-swapping self.cam
        self._frame_jpeg: bytes | None = None
        self._frame_raw  = None               # latest un-annotated frame (enrollment)
        # port present from the start so the footer readout works while loading
        self._state: dict = {"running": False, "status": "STARTING",
                             "port": self.port, "baud": BAUD_RATE}
        self._stopped    = False
        self._thread: threading.Thread | None = None

        self.link: SerialLink | None = None
        self.cam = None
        self.detector = None
        self.error: str | None = None

    # --------------------------------------------------------------- lifecycle
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self, reason: str = "", join_timeout: float = 10.0):
        # Every stop is attributable: a runner exiting with no [ERROR] and no
        # stop-request log line means something is wrong with THIS code path.
        print(f"[INFO] Runner stop requested{f' ({reason})' if reason else ''}.")
        self._stopped = True
        if self._thread:
            self._thread.join(timeout=join_timeout)
            if self._thread.is_alive():
                print("[WARN] Runner thread still alive after "
                      f"{join_timeout:.0f}s join (mid model-load?).")

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------ access
    def get_frame_jpeg(self) -> bytes | None:
        with self._lock:
            return self._frame_jpeg

    def get_raw_frame(self):
        """Latest camera frame without HUD annotations (None until the loop runs).
        The 'Authorize Person' capture reads faces from here."""
        with self._lock:
            return None if self._frame_raw is None else self._frame_raw.copy()

    @property
    def classifier(self):
        det = self.detector
        return det.classifier if det is not None else None

    def get_state(self) -> dict:
        with self._lock:
            state = dict(self._state)
        if state.get("status") in ("STARTING", "LOADING MODELS"):
            state["elapsed"] = round(time.monotonic() - self._t0, 1)
        return state

    def set_camera(self, index: int):
        """Hot-swap the capture device without reloading models/serial.

        Opens the new camera first (slow part, off the lock); only swaps it in if it
        actually delivers a frame, so a bad index leaves the current feed untouched.
        Returns (ok, error)."""
        if index == self.camera:
            return True, None
        try:
            newcam = CameraStream(src=index)
            grabbed, test = newcam.read()
            if not grabbed or test is None:
                newcam.stop()
                return False, f"camera {index} opened but returned no frame"
        except Exception as exc:
            return False, str(exc)
        with self._cam_lock:
            old, self.cam = self.cam, newcam
            self.camera = index
        if old is not None:
            old.stop()
        print(f"[INFO] Switched to camera {index}")
        return True, None

    def _set_state(self, **kw):
        with self._lock:
            self._state.update(kw)

    # --------------------------------------------------------------------- run
    def _run(self):
        try:
            self._loop()
        except Exception as exc:           # surface the failure to the UI, don't crash silently
            self.error = str(exc)
            self._set_state(running=False, status="ERROR", error=str(exc))
            print(f"[ERROR][runner] {exc}")
            event_log.emit("error", "SYS", f"runner error: {exc}", echo=False)
        finally:
            self._teardown()

    def _camera_error(self) -> str:
        """Actionable message for the UI banner: name the indices that DO work."""
        from core.camera import probe_cameras
        working = [i for i in probe_cameras() if i != self.camera]
        if working:
            return (f"Camera {self.camera} not available. "
                    f"Working cameras: {working}")
        return (f"Camera {self.camera} not available and no other working "
                f"camera found — is the webcam in use by another app?")

    def _setup_camera(self):
        try:
            cam = CameraStream(src=self.camera)
        except RuntimeError:
            raise RuntimeError(self._camera_error()) from None
        grabbed, test = cam.read()
        if not grabbed or test is None:
            cam.stop()
            raise RuntimeError(f"Camera {self.camera} opened but returned no frame.")
        print(f"[INFO] Camera OK — {test.shape[1]}x{test.shape[0]}")
        return cam

    def _loop(self):
        self._set_state(running=True, status="LOADING MODELS")
        cam      = self._setup_camera()
        self.cam = cam
        face_path = FACE_MODEL if os.path.exists(FACE_MODEL) else None
        detector = YOLODetector(PERSON_MODEL, face_path)
        self.detector = detector

        pid_pan  = PID(PAN_KP,  PAN_KI,  PAN_KD,  MAX_SPEED)
        pid_tilt = PID(TILT_KP, TILT_KI, TILT_KD, MAX_SPEED)

        self.link = SerialLink(self.port, BAUD_RATE)
        connected = self.link.connect()
        if not connected:
            self.link = None
        # echo=False: SerialLink.connect() already prints the same fact
        if connected:
            event_log.emit("info", "SERIAL",
                           f"connected on {self.port} @ {BAUD_RATE}", echo=False)
        else:
            event_log.emit("warn", "SERIAL",
                           f"no Arduino on {self.port} — preview mode "
                           f"(servos inactive)", echo=False)

        # ── loop state (mirrors main.py) ──
        smooth_cx = smooth_cy = None
        no_body_count = 0
        smooth_pan = smooth_tilt = 0.0
        slew_pan = slew_tilt = 0.0
        last_send = 0.0
        last_time = time.monotonic()
        last_seq = -1
        error_x = error_y = 0.0
        last_sent_pan, last_sent_tilt = STOP_PAN, STOP_TILT
        last_sent_fire = False
        fire = locked = False
        lock_count = 0
        fps = 0.0
        pid_live = False   # False across coast/lost frames -> resync on resume

        # ── event-capture state (logging only — diff-based so core/auth stay
        # untouched; the loop already receives every per-frame value) ──
        prev_status     = "RUNNING"
        prev_fire_label = "SAFE"
        prev_auth: dict = {}    # tid -> last verdict emitted to the log
        track_seen: dict = {}   # tid -> last monotonic time seen in all_persons

        self._set_state(status="RUNNING",
                        serial=("connected" if self.link else "preview"))
        print("[INFO] Runner loop started")
        event_log.emit("info", "SYS", "models loaded — tracking loop started",
                       echo=False)

        while not self._stopped:
            with self._cam_lock:
                cam = self.cam
            grabbed, frame, seq = (cam.read_seq() if cam is not None
                                   else (False, None, -1))
            # Gate on NEW frames: spinning on the latest frame at CPU speed made
            # the loop run at thousands of Hz — the PID derivative collapses
            # (error unchanged between duplicates) and the per-frame EMAs decay
            # 20x faster than their constants suggest.
            if not grabbed or frame is None or seq == last_seq:
                time.sleep(0.002)
                continue
            last_seq = seq

            with self._lock:                  # clean copy before the HUD draws on it
                self._frame_raw = frame.copy()

            now = time.monotonic()
            dt  = max(now - last_time, 1e-6)
            last_time = now
            # Seed the EMA on the first sane interval — the very first frame sees
            # dt~0 (last_time was set moments earlier) and 1/dt would poison the
            # average with a ~1e6 spike that takes minutes to decay.
            if dt > 1e-3:
                fps = (1.0 / dt) if fps == 0.0 else (0.9 * fps + 0.1 * (1.0 / dt))

            h_f, w_f = frame.shape[:2]
            cx_frame, cy_frame = w_f // 2, h_f // 2

            detector.submit(frame)
            det = detector.get_result()
            track_id      = det["track_id"]
            all_persons   = det["all_persons"]
            faces         = det["faces"]
            auth_statuses = det["auth_statuses"]
            locked_face_auth = det.get("locked_face_auth", "UNKNOWN")

            def fire_gate(ex, ey, fire_in, lc_in):
                return update_fire_state(ex, ey, locked_face_auth, fire_in, lc_in)

            pan_raw = tilt_raw = 0.0
            pan_pid = tilt_pid = 0.0
            status  = "NO BODY"
            target_box = select_face(faces, smooth_cx, smooth_cy)

            if target_box is not None and det["person_box"] is not None:
                no_body_count = 0
                x1, y1, x2, y2 = target_box
                raw_cx, raw_cy = (x1 + x2) // 2, (y1 + y2) // 2
                if smooth_cx is None:
                    smooth_cx, smooth_cy = float(raw_cx), float(raw_cy)
                else:
                    smooth_cx = SMOOTH * smooth_cx + (1.0 - SMOOTH) * raw_cx
                    smooth_cy = SMOOTH * smooth_cy + (1.0 - SMOOTH) * raw_cy

                error_x = smooth_cx - cx_frame
                error_y = smooth_cy - cy_frame
                fire, lock_count, locked = fire_gate(error_x, error_y, fire, lock_count)

                if abs(error_x) < INPUT_DEADBAND:
                    error_x = 0.0
                if abs(error_y) < INPUT_DEADBAND:
                    error_y = 0.0

                # Re-anchor the derivative after any gap: coast/lost frames
                # freeze the PID (no update), so prev_error is stale — a
                # derivative computed across the gap is fictitious and
                # saturates the command (the hardware "lurch").
                if not pid_live:
                    pid_pan.resync(error_x * INVERT_PAN)
                    pid_tilt.resync(error_y * INVERT_TILT)
                    pid_live = True

                pan_pid  = pid_pan.update(error_x  * INVERT_PAN,  dt, INTEGRAL_CLAMP)
                tilt_pid = pid_tilt.update(error_y * INVERT_TILT, dt, INTEGRAL_CLAMP)
                pan_raw  = apply_output_deadband(pan_pid,  OUTPUT_DEADBAND)
                tilt_raw = apply_output_deadband(tilt_pid, OUTPUT_DEADBAND)
                status   = "TRACKING"

            elif smooth_cx is not None and no_body_count < FIRE_COAST_FRAMES:
                no_body_count += 1
                pid_live = False
                error_x = smooth_cx - cx_frame
                error_y = smooth_cy - cy_frame
                fire, lock_count, locked = fire_gate(error_x, error_y, fire, lock_count)
                error_x = error_y = 0.0
                status = "TRACKING"

            else:
                no_body_count += 1
                pid_live = False
                error_x = error_y = 0.0
                fire = locked = False
                lock_count = 0
                if no_body_count > 15:
                    smooth_cx = smooth_cy = None
                    smooth_pan = smooth_tilt = 0.0
                    slew_pan = slew_tilt = 0.0
                    pid_pan.reset()
                    pid_tilt.reset()

            smooth_pan  = asym_ema(smooth_pan,  pan_raw,  CMD_SMOOTH, CMD_SMOOTH_DECAY)
            smooth_tilt = asym_ema(smooth_tilt, tilt_raw, CMD_SMOOTH, CMD_SMOOTH_DECAY)
            slew_pan  = slew_step(slew_pan,  smooth_pan,
                                  MAX_CMD_CHANGE_PER_SEC_PAN  * dt, SLEW_BRAKE_MULT)
            slew_tilt = slew_step(slew_tilt, smooth_tilt,
                                  MAX_CMD_CHANGE_PER_SEC_TILT * dt, SLEW_BRAKE_MULT)
            # Deadband the FINAL offset too: the EMA/slew decay tail otherwise
            # dribbles 1-3 unit commands the CR servos act on (creep past center).
            pan_out  = apply_output_deadband(slew_pan,  OUTPUT_DEADBAND)
            tilt_out = apply_output_deadband(slew_tilt, OUTPUT_DEADBAND)
            pan_cmd  = STOP_PAN  + int(round(pan_out))
            tilt_cmd = STOP_TILT + int(round(tilt_out))

            if CONTROL_DEBUG:
                print(f"[CTRL] dt={dt*1000:4.0f}ms "
                      f"pan(err={error_x:+6.1f} pid={pan_pid:+6.2f} ema={smooth_pan:+6.2f} "
                      f"slew={slew_pan:+6.2f} cmd={pan_cmd:3d}) "
                      f"tilt(err={error_y:+6.1f} pid={tilt_pid:+6.2f} ema={smooth_tilt:+6.2f} "
                      f"slew={slew_tilt:+6.2f} cmd={tilt_cmd:3d})")

            fire_label, fire_col = _fire_label_dart(fire, locked, locked_face_auth)

            # ── event capture (logging only, no behavior change): diff this
            # frame's already-computed values against the last emitted ones ──
            if status != prev_status:
                event_log.emit("info", "SYS", f"state: {prev_status} -> {status}",
                               echo=False)
                prev_status = status
            for (_bx1, _by1, _bx2, _by2, tid) in all_persons:
                cur = auth_statuses.get(tid, "UNKNOWN")
                if tid not in track_seen:
                    event_log.emit("info", "TRACK", f"Track {tid} acquired",
                                   echo=False)
                    prev_auth[tid] = "UNKNOWN"
                if prev_auth.get(tid) != cur:
                    event_log.emit("warn" if cur == "UNAUTHORIZED" else "info",
                                   "AUTH",
                                   f"Track {tid}: {prev_auth.get(tid, 'UNKNOWN')}"
                                   f" -> {cur}", echo=False)
                    prev_auth[tid] = cur
                track_seen[tid] = now
            # "lost" only after 2s absent so ByteTrack conf dips don't spam
            for tid in [t for t, ts in track_seen.items() if now - ts > 2.0]:
                event_log.emit("info", "TRACK", f"Track {tid} lost", echo=False)
                track_seen.pop(tid, None)
                prev_auth.pop(tid, None)
            if fire_label != prev_fire_label:
                lvl = ("error" if fire_label == "FIRING"
                       else "warn" if fire_label == "ARMED" else "info")
                tgt = f" (target ID {track_id})" if track_id is not None else ""
                event_log.emit(lvl, "FIRE",
                               f"{prev_fire_label} -> {fire_label}{tgt}",
                               echo=False)
                prev_fire_label = fire_label

            _draw_hud(frame, status, pan_cmd, tilt_cmd, smooth_cx, smooth_cy,
                      all_persons, faces, track_id, w_f, h_f, error_x, error_y,
                      fire_label, fire_col, auth_statuses)

            # ── serial send (same cadence as main.py) ──
            if self.link:
                time_since_send = now - last_send
                changed = (fire != last_sent_fire)
                moved   = (pan_cmd != last_sent_pan or tilt_cmd != last_sent_tilt)
                if (changed
                        or (moved and time_since_send >= SEND_INTERVAL)
                        or (time_since_send >= HEARTBEAT_INTERVAL)):
                    self.link.send_command(pan_cmd, tilt_cmd, fire, flush=changed)
                    last_sent_pan, last_sent_tilt = pan_cmd, tilt_cmd
                    last_sent_fire = fire
                    last_send = now

            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            clf = detector.classifier
            state = {
                "running": True, "status": status,
                "track_id": track_id, "n_persons": len(all_persons),
                "pan": pan_cmd, "tilt": tilt_cmd, "fire": bool(fire),
                "locked": bool(locked), "fire_label": fire_label,
                # post-deadband pixel error — the same numbers the HUD bar shows
                "error_x": int(round(error_x)), "error_y": int(round(error_y)),
                "serial": "connected" if self.link else "preview",
                "camera": self.camera,
                "port": self.port, "baud": BAUD_RATE,
                "fps": round(fps, 1),
                "auth_provider": clf.provider,
                "auth_ms": (round(clf.classify_ms) if clf.classify_ms is not None
                            else None),
            }
            with self._lock:
                if ok:
                    self._frame_jpeg = buf.tobytes()
                self._state = state

        self._set_state(running=False, status="STOPPED")
        event_log.emit("info", "SYS", "runner stopped", echo=False)

    def _teardown(self):
        for closer in (
            lambda: self.link and self.link.close(),
            lambda: self.detector and self.detector.stop(),
            lambda: self.cam and self.cam.stop(),
        ):
            try:
                closer()
            except Exception:
                pass
        print("[INFO] Runner stopped.")
