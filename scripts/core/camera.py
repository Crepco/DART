import threading
import time

import cv2

from config import (CAM_WIDTH, CAM_HEIGHT, CAM_FPS, CAM_BACKEND, CAM_FOURCC,
                    CAM_MAX_READ_FAILURES, CAM_RECONNECT_DELAY,
                    CAM_RECONNECT_MAX_DELAY, CAM_READ_FAIL_SLEEP)


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
        self.cap      = self._open()
        if self.cap is None:
            raise RuntimeError(f"Camera {src} not available. Try --camera 0 or 1")
        self.grabbed, self.frame = self.cap.read()
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

    def stop(self):
        self._stopped = True
        self._thread.join()
        if self.cap is not None:
            self.cap.release()
