"""Driving camera: newest-frame capture, AprilTag 36h11 detection, floor-colour obstacle check.

    python vision.py floor             learn the floor colour now (clear floor in front!) and save it to floor.npy
    python vision.py snap [out.jpg]    one frame with tags, the obstacle corridor and the not-floor mask (red) drawn
"""
import os
import sys
import threading

import cv2
import numpy as np

FLOOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "floor.npy")
_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))


class Camera:
    """A thread keeps reading, so frame() returns the newest image, never one the driver buffered a
    second ago (stale frames make the steering oscillate). index None = mock grey frames."""

    def __init__(self, index=0, size=(640, 480)):
        self.mock, self.size = index is None, size
        if self.mock:
            return
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2 if sys.platform == "linux" else cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        if not self.cap.isOpened() or not self.cap.read()[0]:
            raise RuntimeError(f"camera {index}: no frames (wrong index? `v4l2-ctl --list-devices`)")
        self.img, self.seq, self.last = None, 0, 0
        self.cond = threading.Condition()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            ok, img = self.cap.read()
            if ok:
                with self.cond:
                    self.img, self.seq = img, self.seq + 1
                    self.cond.notify_all()

    def frame(self):
        if self.mock:
            return np.full((self.size[1], self.size[0], 3), 128, np.uint8)
        with self.cond:
            if not self.cond.wait_for(lambda: self.seq > self.last, timeout=1.0):
                raise RuntimeError("camera stalled (unplugged?)")
            self.last = self.seq
            return self.img


def find_tags(img):
    """{tag id: (centre x, centre y, side length px, (x0, y0, x1, y1) box)}."""
    corners, ids, _ = _detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    out = {}
    for c, i in zip(corners, [] if ids is None else ids.flatten()):
        c = c.reshape(4, 2)
        (x0, y0), (x1, y1) = c.min(axis=0), c.max(axis=0)
        out[int(i)] = (*c.mean(axis=0), float(np.linalg.norm(c[0] - c[1])), (x0, y0, x1, y1))
    return out


class Obstacles:
    """Is the corridor right in front of the car still floor?

    learn() remembers the floor's colour (hue + saturation histogram) from the corridor while it is clear:
    floor.npy if `vision.py floor` saved one, else the first frame of the run. blocked() then counts how much of the corridor does not look like that floor.
    ponytail: colour only; a box the same colour as the floor is invisible. Add a depth/ToF sensor if that bites."""

    def __init__(self, cfg):
        self.roi, self.frac = cfg["obstacle_roi"], cfg["obstacle_frac"]
        self.hist = np.load(FLOOR) if os.path.exists(FLOOR) else None

    def _crop(self, img):
        h, w = img.shape[:2]
        x0, y0, x1, y1 = self.roi
        return (int(x0 * w), int(y0 * h)), cv2.cvtColor(img[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)],
                                                        cv2.COLOR_BGR2HSV)

    def learn(self, img):
        _, hsv = self._crop(img)
        self.hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX)

    def mask(self, img, ignore=()):
        """Not-floor mask of the corridor (uint8 0/255), with the ignore boxes (tags) cleared."""
        (ox, oy), hsv = self._crop(img)
        bp = cv2.calcBackProject([hsv], [0, 1], self.hist, [0, 180, 0, 256], 1)
        m = np.where(cv2.GaussianBlur(bp, (9, 9), 0) < 20, 255, 0).astype(np.uint8)
        for x0, y0, x1, y1 in ignore:
            m[max(0, int(y0) - oy):max(0, int(y1) - oy), max(0, int(x0) - ox):max(0, int(x1) - ox)] = 0
        return m

    def blocked(self, img, ignore=()):
        """(blocked?, fraction of the corridor that is not floor)."""
        f = float(self.mask(img, ignore).mean() / 255)
        return f > self.frac, f


def draw(img, tags, obstacles=None):
    out = img.copy()
    for i, (cx, cy, side, (x0, y0, x1, y1)) in tags.items():
        cv2.rectangle(out, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
        cv2.putText(out, f"{i}: {side:.0f}px", (int(x0), int(y0) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if obstacles is not None and obstacles.hist is not None:
        h, w = img.shape[:2]
        x0, y0, x1, y1 = obstacles.roi
        p0, p1 = (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h))
        m = obstacles.mask(img, [t[3] for t in tags.values()])
        out[p0[1]:p0[1] + m.shape[0], p0[0]:p0[0] + m.shape[1]][m > 0] = (0, 0, 255)
        cv2.rectangle(out, p0, p1, (255, 255, 0), 2)
    return out


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd not in ("floor", "snap"):
        sys.exit(__doc__)
    from motors import load_config
    cfg = load_config()
    img = Camera(cfg["camera"], tuple(cfg["camera_size"])).frame()
    obs = Obstacles(cfg)
    if cmd == "floor":
        obs.learn(img)
        np.save(FLOOR, obs.hist)
        sys.exit(f"floor saved to {FLOOR}")
    if obs.hist is None:
        sys.exit("no floor.npy yet: run `python vision.py floor` first")
    out = sys.argv[2] if len(sys.argv) > 2 else "snap.jpg"
    cv2.imwrite(out, draw(img, find_tags(img), obs))
    print("tags:", {i: f"x={t[0]:.0f} side={t[2]:.0f}px" for i, t in find_tags(img).items()}, "->", out)
