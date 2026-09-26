"""YOLO for the Raspberry Pi: a laptop or Brev (GPU; bash Brev/setup.sh starts it) searches for bottles for the Pi
(both YOLO scales in ~25 ms), so the Pi saves CPU.

The camera and the SO-101 stay on the Pi (so101_station.py runs there as a service). This program pulls downscaled
frames from the Pi, runs YOLO11n and sends back the bottle boxes. The worker connects to the Pi, so a Windows firewall
does not get in the way. While the arm TRACKS a bottle, the Pi runs YOLO itself (fast 320 scale, ~18 fps): the
network delay (60-400 ms, variable) would make the arm swing. So this worker only helps while searching (distant
bottles, less heat on a fanless Pi). Close it (Ctrl+C) - within half a second the Pi does everything itself.

    python yolo_laptop.py                        Pi at http://malina:8765 (Tailscale)
    python yolo_laptop.py http://malina.local:8765
"""
import os
import sys
import threading
import time

import cv2
import numpy as np
import requests

import so101_station

PI = (sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PI_URL", "http://malina:8765")).rstrip("/")
THREADS = 2  # this many frames in flight: while one travels over the network, the other is computed (~60 ms each)
FIELDS = ("cx", "cy", "w", "h", "conf", "cls", "box")

_stats = {"n": 0, "yolo": 0.0, "seen": [], "connected": None}
_lock = threading.Lock()


def worker(detector):
    session = requests.Session()
    nr, result = 0, None
    while True:
        try:
            # result of the previous frame -> the next frame comes back in the answer (one request per frame)
            r = session.post(f"{PI}/yolo", params={"nr": nr}, json=result, timeout=3)
        except requests.RequestException as e:
            with _lock:
                if _stats["connected"] is not False:
                    print(f"\nPi not answering ({type(e).__name__}) - is so101_station.py running on the Pi? "
                          "Retrying...")
                _stats["connected"] = False
            nr, result = 0, None
            time.sleep(1)
            continue
        if r.status_code == 404:
            print("\nThe Pi answers but has an old program without /yolo - deploy: python deploy_to_pi.py")
            os._exit(1)
        with _lock:
            if not _stats["connected"]:
                print("Connected - searching for bottles for the Pi (while the arm tracks, the Pi computes itself). "
                      "Ctrl+C ends.")
                _stats["connected"] = True
        nr, result = 0, None
        if r.status_code != 200:  # 204: the Pi had no new frame for 1 s
            continue
        frame = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        t = time.perf_counter()
        result = [{k: b[k] for k in FIELDS} for b in detector.detect(frame)]
        nr = int(r.headers.get("X-Nr", 0))
        with _lock:
            _stats["n"] += 1
            _stats["yolo"] += time.perf_counter() - t
            _stats["seen"] = result


def main():
    so101_station.YOLO_INTERLEAVE = False  # the worker has time for both scales (320 + 640) every frame - surer
    detector = so101_station.BottleDetector()  # one network for both threads (onnxruntime allows it)
    print(f"Connecting to {PI} ...")
    for _ in range(THREADS):
        threading.Thread(target=worker, args=(detector,), daemon=True).start()
    t_stats = time.time()
    while True:
        time.sleep(2.0)
        with _lock:
            n, elapsed, seen = _stats["n"], _stats["yolo"], _stats["seen"]
            _stats["n"], _stats["yolo"] = 0, 0.0
        if n:
            text = ", ".join("%s %.0f%%" % (b["cls"], b["conf"] * 100) for b in seen) or "-"
            print(f"\r{n / (time.time() - t_stats):5.1f} fps | YOLO {elapsed / n * 1000:4.0f} ms | seen: {text:40s}",
                  end="", flush=True)
        t_stats = time.time()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped - the Pi runs YOLO itself.")
