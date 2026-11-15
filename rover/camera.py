"""USB webcam in a background thread. Only JPEG-encodes while someone is watching."""
import threading
import time


class Camera:
    def __init__(self, cfg):
        c = cfg.get("camera", {})
        self.enabled = bool(c.get("enabled", False))
        self.jpeg, self.viewers, self.fps = None, 0, 0.0
        self.error = ""
        if not self.enabled:
            return
        try:
            import cv2
            self.cv2 = cv2
            self.cap = cv2.VideoCapture(int(c.get("device", 0)), cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.get("width", 640))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.get("height", 480))
            self.cap.set(cv2.CAP_PROP_FPS, c.get("fps", 15))
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open /dev/video{c.get('device', 0)}")
        except Exception as e:
            self.enabled, self.error = False, str(e)
            return
        self.quality = int(c.get("quality", 70))
        self._cond = threading.Condition()
        threading.Thread(target=self._loop, name="camera", daemon=True).start()

    def _loop(self):
        last = time.monotonic()
        while True:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.5)
                continue
            if self.viewers > 0:
                ok, buf = self.cv2.imencode(".jpg", frame, [self.cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if ok:
                    with self._cond:
                        self.jpeg = buf.tobytes()
                        self._cond.notify_all()
            now = time.monotonic()
            self.fps = 0.9 * self.fps + 0.1 / max(now - last, 1e-3)
            last = now

    def wait_jpeg(self, timeout=1.0):
        with self._cond:
            self._cond.wait(timeout)
            return self.jpeg
