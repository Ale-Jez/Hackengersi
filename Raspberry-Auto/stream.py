"""Live camera view in a browser: tags (id, px, estimated distance), the obstacle corridor, detection rate.

    python stream.py [port]     then open http://<pi ip>:8000 on a phone or laptop in the same network

Distance = tag_focal_px * tag_size_cm / side px. Calibrate tag_focal_px once: hold a tag at a known
distance D cm, read its side px here, set tag_focal_px = px * D / tag_size_cm.
"""
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

import vision
from motors import load_config

PAGE = b"""<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>Robot camera</title><body style="margin:0;background:#111">
<img src="/stream" style="width:100%;max-width:960px;display:block;margin:auto"></body>"""


def annotate(img, cfg, obs, seen):
    """Tags (id, px, distance), the obstacle corridor and the detection rate over `seen` (a deque)."""
    f, size = cfg.get("tag_focal_px"), cfg.get("tag_size_cm", 6.0)
    tags = vision.find_tags(img)
    seen.append(1 if tags else 0)
    out = vision.draw(img, tags, obs if cfg.get("obstacle_check", True) else None)
    y = 30
    for i, (cx, cy, side, _) in sorted(tags.items()):
        dist = f" ~{f * size / side:.0f} cm" if f else ""
        cv2.putText(out, f"tag {i}: {side:.0f}px{dist}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        y += 30
    if not tags:
        cv2.putText(out, "no tag", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(out, f"seen {sum(seen) / len(seen):.0%} of last {len(seen)} frames", (10, out.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


class Feed:
    """Camera -> annotated JPEG, one worker thread; every browser gets the newest JPEG."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.cam = vision.Camera(cfg["camera"], tuple(cfg["camera_size"]), cfg.get("camera_flip", False))
        self.obs = vision.Obstacles(cfg)
        self.seen = deque(maxlen=30)  # 1 per frame with at least one tag, for the detection rate
        self.jpg, self.cond = None, threading.Condition()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            ok, buf = cv2.imencode(".jpg", annotate(self.cam.frame(), self.cfg, self.obs, self.seen),
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with self.cond:
                    self.jpg = buf.tobytes()
                    self.cond.notify_all()

    def next(self, last):
        with self.cond:
            self.cond.wait_for(lambda: self.jpg is not last, timeout=2.0)
            return self.jpg


def serve(port):
    feed = Feed(load_config())

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path != "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(PAGE)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            jpg = None
            try:
                while True:
                    jpg = feed.next(jpg)
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                pass

    print(f"open http://<pi ip>:{port}  (Ctrl-C to stop)")
    ThreadingHTTPServer(("", port), Handler).serve_forever()


if __name__ == "__main__":
    serve(int(sys.argv[1]) if len(sys.argv) > 1 else 8000)
