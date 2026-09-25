"""USB scan camera: photos, and focus on the barcode by sweeping manual focus for the sharpest ROI.

python station/camera.py [out.jpg]  -> focus (if the camera allows it) and save one photo.
python station/camera.py --live      -> preview window (q quit, s save, f focus).
Env overrides: KAUCJO_CAM (OpenCV index, default 1; 0 is the laptop's own camera).
"""
import os
import sys
import time

import cv2

FOCUS_VALUES = range(0, 256, 8)  # ponytail: 0-255 is the UVC norm; some cameras use 0-1023, widen if best hits an end
SETTLE_FRAMES = 6  # frames to throw away after a focus move: lens travel + stale buffer


class Camera:
    def __init__(self, index=None, size=(1920, 1080)):
        index = int(os.environ.get("KAUCJO_CAM", 1)) if index is None else index
        # DSHOW is the backend where OpenCV's CAP_PROP_FOCUS works; MJPG is needed for 1080p over USB2
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        if not self.cap.isOpened() or not self.cap.read()[0]:
            raise RuntimeError(f"camera {index}: no frames. Wrong KAUCJO_CAM index or another app has it open?")

    def frame(self, settle=SETTLE_FRAMES):
        for _ in range(settle):  # the driver queues old frames; only the last read is current
            self.cap.grab()
        ok, img = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed (unplugged?)")
        return img

    def photo(self, path=None):
        img = self.frame()
        if path:
            cv2.imwrite(path, img)
        return img

    def focus_on(self, roi=None, values=FOCUS_VALUES):
        """Sweep manual focus, keep the value where roi = (x, y, w, h) is sharpest. Default roi: middle half."""
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        if not self.cap.set(cv2.CAP_PROP_FOCUS, values[0]):
            raise RuntimeError("camera does not expose focus control to OpenCV (DirectShow)")
        best, best_score = values[0], -1.0
        for v in values:
            self.cap.set(cv2.CAP_PROP_FOCUS, v)
            img = self.frame()
            if roi is None:
                h, w = img.shape[:2]
                roi = (w // 4, h // 4, w // 2, h // 2)
            x, y, rw, rh = roi
            score = cv2.Laplacian(cv2.cvtColor(img[y:y + rh, x:x + rw], cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
            if score > best_score:
                best, best_score = v, score
        self.cap.set(cv2.CAP_PROP_FOCUS, best)
        time.sleep(0.3)
        return best

    def close(self):
        self.cap.release()


def live(cam):
    """Preview window: q quit, s save photo.jpg, f run focus_on (the box is the focus area)."""
    while True:
        img = cam.frame(settle=0)
        h, w = img.shape[:2]
        view = img.copy()
        cv2.rectangle(view, (w // 4, h // 4), (3 * w // 4, 3 * h // 4), (0, 255, 0), 2)
        cv2.imshow("camera (q quit, s save, f focus)", cv2.resize(view, (w // 2, h // 2)))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("s"):
            cv2.imwrite("photo.jpg", img)
            print("saved photo.jpg")
        if k == ord("f"):
            try:
                print("focus:", cam.focus_on())
            except RuntimeError as e:
                print("focus skipped:", e)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    cam = Camera()
    if "--live" in sys.argv:
        live(cam)
        cam.close()
        sys.exit()
    try:
        print("focus:", cam.focus_on())
    except RuntimeError as e:
        print("focus skipped:", e)
    out = sys.argv[1] if len(sys.argv) > 1 else "photo.jpg"
    print(cam.photo(out).shape, "->", out)
    cam.close()
