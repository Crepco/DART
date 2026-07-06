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
from core import CameraStream, YOLODetector, PID, clamp
from core.detector import select_face
from main import update_fire_state, apply_output_deadband, AUTH_COLS

from .serial_link import SerialLink


def _fire_label_dart(fire, locked, locked_face_auth):
    if fire:
        return "FIRING", COL_FIRING
    if locked and locked_face_auth == "UNAUTHORIZED":
        return "ARMED", COL_ARMED
    if locked and locked_face_auth == "AUTHORIZED":
        return "SAFE/AUTH", COL_AUTHORIZED
    return "SAFE", (120, 120, 120)


def _draw_hud(frame, status, pan_cmd, tilt_cmd, smooth_cx, smooth_cy,
              all_persons, faces, track_id, w_f, h_f, error_x, error_y,
              fire_label, fire_col, auth_statuses):
    """Annotate the frame: person/face boxes, centroid, top status bar, reticle, and lock
    ring. Adapted from main.draw_hud, kept self-contained so the proven desktop path
    (main.py) is untouched."""
    tracking   = (status == "TRACKING")
    bar_col    = COL_TRACKING if tracking else COL_NO_BODY
    cx_f, cy_f = w_f // 2, h_f // 2

    for (x1, y1, x2, y2, tid) in all_persons:
        auth_status = auth_statuses.get(tid, "UNKNOWN")
        col         = AUTH_COLS[auth_status]
        is_target   = (tid == track_id)
        thickness   = 3 if is_target else 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"ID{tid} {auth_status}", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

    for (x1, y1, x2, y2) in faces:
        cv2.rectangle(frame, (x1, y1), (x2, y2), COL_FACE_BOX, 2, cv2.LINE_AA)

    if smooth_cx is not None and tracking:
        cv2.line(frame, (int(smooth_cx), int(smooth_cy)), (cx_f, cy_f),
                 (80, 80, 80), 1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 8, COL_CENTROID, -1, cv2.LINE_AA)
        cv2.circle(frame, (int(smooth_cx), int(smooth_cy)), 8, (0, 0, 0), 1, cv2.LINE_AA)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_f, 40), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    tid_label = f"  ID={track_id}" if track_id is not None else ""
    bar_text  = (f"[{status}]{tid_label}   Pan {pan_cmd:3d}  Tilt {tilt_cmd:3d}"
                 f"   Err ({int(error_x):+d}, {int(error_y):+d})")
    cv2.putText(frame, bar_text, (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_col, 2, cv2.LINE_AA)
    cv2.putText(frame, fire_label, (w_f - 150, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, fire_col, 2, cv2.LINE_AA)

    arm = 10 if tracking else 18
    cv2.line(frame, (cx_f - arm, cy_f), (cx_f + arm, cy_f), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.line(frame, (cx_f, cy_f - arm), (cx_f, cy_f + arm), COL_RETICLE, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx_f, cy_f), arm + 8, COL_RETICLE, 1, cv2.LINE_AA)
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
        self._state: dict = {"running": False, "status": "STARTING"}
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

        # ── loop state (mirrors main.py) ──
        smooth_cx = smooth_cy = None
        no_body_count = 0
        smooth_pan = smooth_tilt = 0.0
        slew_pan = slew_tilt = 0.0
        last_send = 0.0
        last_time = time.monotonic()
        error_x = error_y = 0.0
        last_sent_pan, last_sent_tilt = STOP_PAN, STOP_TILT
        last_sent_fire = False
        fire = locked = False
        lock_count = 0
        fps = 0.0

        self._set_state(status="RUNNING",
                        serial=("connected" if self.link else "preview"))
        print("[INFO] Runner loop started")

        while not self._stopped:
            with self._cam_lock:
                cam = self.cam
            grabbed, frame = cam.read() if cam is not None else (False, None)
            if not grabbed or frame is None:
                time.sleep(0.01)
                continue

            with self._lock:                  # clean copy before the HUD draws on it
                self._frame_raw = frame.copy()

            now = time.monotonic()
            dt  = max(now - last_time, 1e-6)
            last_time = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

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

                pan_pid  = pid_pan.update(error_x  * INVERT_PAN,  dt, INTEGRAL_CLAMP)
                tilt_pid = pid_tilt.update(error_y * INVERT_TILT, dt, INTEGRAL_CLAMP)
                pan_raw  = apply_output_deadband(pan_pid,  OUTPUT_DEADBAND)
                tilt_raw = apply_output_deadband(tilt_pid, OUTPUT_DEADBAND)
                status   = "TRACKING"

            elif smooth_cx is not None and no_body_count < FIRE_COAST_FRAMES:
                no_body_count += 1
                error_x = smooth_cx - cx_frame
                error_y = smooth_cy - cy_frame
                fire, lock_count, locked = fire_gate(error_x, error_y, fire, lock_count)
                error_x = error_y = 0.0
                status = "TRACKING"

            else:
                no_body_count += 1
                error_x = error_y = 0.0
                fire = locked = False
                lock_count = 0
                if no_body_count > 15:
                    smooth_cx = smooth_cy = None
                    smooth_pan = smooth_tilt = 0.0
                    slew_pan = slew_tilt = 0.0
                    pid_pan.reset()
                    pid_tilt.reset()

            smooth_pan  = CMD_SMOOTH * smooth_pan  + (1.0 - CMD_SMOOTH) * pan_raw
            smooth_tilt = CMD_SMOOTH * smooth_tilt + (1.0 - CMD_SMOOTH) * tilt_raw
            max_step_pan  = MAX_CMD_CHANGE_PER_SEC_PAN  * dt
            max_step_tilt = MAX_CMD_CHANGE_PER_SEC_TILT * dt
            slew_pan  += clamp(smooth_pan  - slew_pan,  -max_step_pan,  max_step_pan)
            slew_tilt += clamp(smooth_tilt - slew_tilt, -max_step_tilt, max_step_tilt)
            pan_cmd  = STOP_PAN  + int(round(slew_pan))
            tilt_cmd = STOP_TILT + int(round(slew_tilt))

            fire_label, fire_col = _fire_label_dart(fire, locked, locked_face_auth)

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
            state = {
                "running": True, "status": status,
                "track_id": track_id, "n_persons": len(all_persons),
                "pan": pan_cmd, "tilt": tilt_cmd, "fire": bool(fire),
                "locked": bool(locked), "fire_label": fire_label,
                "serial": "connected" if self.link else "preview",
                "camera": self.camera,
                "fps": round(fps, 1),
            }
            with self._lock:
                if ok:
                    self._frame_jpeg = buf.tobytes()
                self._state = state

        self._set_state(running=False, status="STOPPED")

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
