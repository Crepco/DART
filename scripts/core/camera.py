import threading

import cv2

from config import CAM_WIDTH, CAM_HEIGHT, CAM_FPS


class CameraStream:
    def __init__(self, src: int = 0):
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera {src} not available. Try --camera 0 or 1")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        self.grabbed, self.frame = self.cap.read()
        self._lock    = threading.Lock()
        self._stopped = False
        self._thread  = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

    def _update(self):
        while not self._stopped:
            g, f = self.cap.read()
            with self._lock:
                self.grabbed, self.frame = g, f

    def read(self):
        with self._lock:
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self._stopped = True
        self._thread.join()
        self.cap.release()
