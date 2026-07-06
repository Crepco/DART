import threading
import time

import cv2

from config import (CAM_WIDTH, CAM_HEIGHT, CAM_FPS, CAM_BACKEND, CAM_FOURCC,
                    CAM_MAX_READ_FAILURES, CAM_RECONNECT_DELAY,
                    CAM_RECONNECT_MAX_DELAY, CAM_READ_FAIL_SLEEP,
                    CAM_OPEN_RETRIES, CAM_OPEN_RETRY_DELAY)


PROBE_MAX_INDEX = 6   # scan indices 0..5 when looking for connected cameras


def probe_cameras(max_index: int = PROBE_MAX_INDEX, skip: int | None = None):
    """Best-effort list of camera indices that deliver a frame. Opens each index
    briefly on the configured backend. `skip` (an index the caller already holds
    open) is reported as available without re-opening it, since the OS won't let
    us open a camera that's already in use."""
    found = []
    for i in range(max_index):
        if skip is not None and i == skip:
            found.append(i)
            continue
        cap = None
        try:
            cap = cv2.VideoCapture(i, CAM_BACKEND)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    found.append(i)
        except Exception:
            pass
        finally:
            if cap is not None:
                cap.release()
    return sorted(set(found))


class CameraStream:
    """Threaded webcam capture that self-heals after read failures.

    OpenCV's default Windows backend (MSMF) stalls USB webcams after ~60s
    (grabFrame Error -1072873822). We open on DirectShow (see config.CAM_BACKEND)
    and the background thread releases + reopens the device on repeated failures,
    so a transient glitch — or even an unplug/replug — can't permanently kill the
    stream.
    """

    def __init__(self, src: int = 0):
        self._src     = src
        self._lock    = threading.Lock()
        self._stopped = False
        # DSHOW may refuse an index for a second or two after another process
        # releases the device, so the startup open gets several logged attempts
        # (mid-stream failures already have their own reconnect loop below).
        self.cap = None
        for attempt in range(1, CAM_OPEN_RETRIES + 1):
            self.cap = self._open()
            if self.cap is not None:
                break
            print(f"[CAM] Open attempt {attempt}/{CAM_OPEN_RETRIES} failed for "
                  f"camera {src} (backend={CAM_BACKEND}).")
            if attempt < CAM_OPEN_RETRIES:
                time.sleep(CAM_OPEN_RETRY_DELAY)
        if self.cap is None:
            raise RuntimeError(f"Camera {src} not available "
                               f"(after {CAM_OPEN_RETRIES} attempts)")
        self.grabbed, self.frame = self.cap.read()
        self.seq = 1 if self.grabbed else 0   # bumps on every fresh frame
        self._thread  = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

    def _open(self):
        """Open the capture on the configured backend and apply all props.

        Returns an opened VideoCapture, or None on failure (the caller decides
        whether that's fatal or a reconnect retry).
        """
        cap = cv2.VideoCapture(self._src, CAM_BACKEND)
        if not cap.isOpened():
            cap.release()
            return None
        # FOURCC must be set BEFORE width/height/fps — the codec gates which
        # (w, h, fps) modes the driver will accept.
        if CAM_FOURCC:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*CAM_FOURCC))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
        # Low latency: keep only the newest frame. Often ignored by DSHOW, but
        # the threaded latest-frame-wins loop below keeps latency low regardless.
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        # Note: DSHOW may silently substitute the nearest supported resolution;
        # the pipeline reads frame.shape dynamically, so that's tolerated. Log the
        # mode it actually negotiated so a mismatch is visible.
        w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"[CAM] Camera {self._src} opened: {w}x{h} @ {fps:.0f}fps "
              f"(backend={CAM_BACKEND}, fourcc={CAM_FOURCC or 'default'})")
        return cap

    def _sleep_interruptible(self, seconds):
        """Sleep in short slices so a stop() request is honoured within ~50ms."""
        end = time.monotonic() + seconds
        while not self._stopped and time.monotonic() < end:
            time.sleep(0.05)

    def _update(self):
        fails   = 0
        backoff = CAM_RECONNECT_DELAY
        while not self._stopped:
            cap  = self.cap
            g, f = (False, None) if cap is None else cap.read()

            if g and f is not None:
                with self._lock:
                    self.grabbed, self.frame = True, f
                    self.seq += 1
                if fails:
                    print("[CAM] Recovered — stream healthy again.")
                fails   = 0
                backoff = CAM_RECONNECT_DELAY
                continue

            # ── failure path ──
            fails += 1
            with self._lock:
                self.grabbed = False          # read() now returns (False, None)
            if fails < CAM_MAX_READ_FAILURES:
                time.sleep(CAM_READ_FAIL_SLEEP)   # throttle the spin (~5ms)
                continue

            # Threshold crossed → release the dead session and reopen.
            print(f"[WARN][CAM] {fails} consecutive read failures — "
                  f"reopening camera {self._src} (backend={CAM_BACKEND}).")
            if cap is not None:
                cap.release()
            with self._lock:
                self.cap = None

            new_cap = None
            while not self._stopped:
                new_cap = self._open()
                if new_cap is not None:
                    break
                print(f"[WARN][CAM] Reopen failed; retrying in {backoff:.1f}s "
                      f"(camera unplugged?).")
                self._sleep_interruptible(backoff)
                backoff = min(backoff * 2, CAM_RECONNECT_MAX_DELAY)

            if self._stopped:
                if new_cap is not None:
                    new_cap.release()
                break

            with self._lock:
                self.cap = new_cap
            fails   = 0
            backoff = CAM_RECONNECT_DELAY
            print(f"[CAM] Reconnected to camera {self._src}.")

    def read(self):
        with self._lock:
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def read_seq(self):
        """(grabbed, frame, seq) — seq identifies the frame, so a control loop can
        wait for a NEW frame instead of re-processing the latest one at CPU speed
        (which collapses the PID derivative and distorts the per-frame EMAs)."""
        with self._lock:
            return (self.grabbed,
                    self.frame.copy() if self.frame is not None else None,
                    self.seq)

    def stop(self):
        self._stopped = True
        self._thread.join()
        if self.cap is not None:
            self.cap.release()
