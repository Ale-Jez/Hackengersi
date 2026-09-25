"""Camera, AprilTags, pixel -> RoArm mapping, and the Brev LLM that finds the trash.

The Brev box runs any OpenAI-compatible vision server, e.g.
    vllm serve Qwen/Qwen2.5-VL-7B-Instruct --port 8000
so there is no custom server to write. BREV_KEY (env) is sent as the bearer token if set.

    python vision.py photo [out.jpg]     save one frame
    python vision.py ask photo.jpg       send a photo to Brev and print the items
    python vision.py                     self-check (no hardware, no network)
"""
import base64
import json
import os
import re
import sys

import cv2
import numpy as np
import requests

BINS = ("cans_bottles", "paper", "plastic")
LLM_SIZE = (896, 672)  # multiple of 28 (Qwen-VL patch grid), so its pixel answers need no rescaling on its side
PROMPT = """This photo looks down into a trash can. List every loose piece of trash a robot gripper should pick up.
Sort each into exactly one bin:
- "cans_bottles": metal cans, drink bottles (plastic or glass)
- "paper": paper, cardboard, cartons, napkins
- "plastic": any other plastic (bags, wrappers, cups, foil packs)
Skip anything that fits none of these.
The image is {w}x{h} pixels. For each item give the pixel point where a gripper coming from straight above
should grab it (its centre of mass, on the item itself).
Answer with only a JSON array, largest items first, like:
[{{"label": "coke can", "bin": "cans_bottles", "x": 412, "y": 300}}]
Empty can: []"""


class Camera:
    def __init__(self, index=0, size=(1280, 720)):
        self.mock = index is None
        if self.mock:
            return
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2 if sys.platform == "linux" else cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        if not self.cap.isOpened() or not self.cap.read()[0]:
            raise RuntimeError(f"camera {index}: no frames (wrong index? `v4l2-ctl --list-devices`)")

    def frame(self, settle=5):
        if self.mock:
            return np.full((720, 1280, 3), 128, np.uint8)
        for _ in range(settle):  # the driver buffers old frames; only the last one is current
            self.cap.grab()
        ok, img = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed (unplugged?)")
        return img


_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))


def find_tags(img):
    """{tag id: (centre x, centre y, side length px)} for AprilTag 36h11."""
    corners, ids, _ = _detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    out = {}
    for c, i in zip(corners, [] if ids is None else ids.flatten()):
        c = c.reshape(4, 2)
        out[int(i)] = (*c.mean(axis=0), float(np.linalg.norm(c[0] - c[1])))
    return out


def fit_homography(pixels, arm_xy):
    """Image pixels -> RoArm x,y (mm) on the pick plane. Needs >= 4 points, not all on one line."""
    H, _ = cv2.findHomography(np.float32(pixels), np.float32(arm_xy))
    if H is None:
        raise RuntimeError("homography fit failed: points collinear or too few")
    return H.tolist()


def to_arm(H, px, py):
    # ponytail: one plane (the can's floor); tall items get grabbed a bit off, add depth when that bites
    x, y = cv2.perspectiveTransform(np.float32([[[px, py]]]), np.float32(H))[0, 0]
    return float(x), float(y)


def parse_items(text, w, h):
    """LLM reply -> validated [{label, bin, x, y}] (x, y in the w x h image). Junk entries are dropped."""
    m = re.search(r"\[.*\]", text, re.S)  # models like to wrap JSON in ```json fences or prose
    if not m:
        return []
    try:
        raw = json.loads(m.group(0))
    except ValueError:
        return []
    items = []
    for it in raw if isinstance(raw, list) else []:
        try:
            x, y = float(it["x"]), float(it["y"])
        except (TypeError, KeyError, ValueError):
            continue
        if it.get("bin") in BINS and 0 <= x < w and 0 <= y < h:
            items.append({"label": str(it.get("label", "?")), "bin": it["bin"], "x": x, "y": y})
    return items


def ask_llm(img, url, model):
    """Send the photo to the Brev model; returns items with x, y in the ORIGINAL image's pixels."""
    w, h = LLM_SIZE
    jpg = cv2.imencode(".jpg", cv2.resize(img, LLM_SIZE), [cv2.IMWRITE_JPEG_QUALITY, 85])[1]
    key = os.environ.get("BREV_KEY")
    r = requests.post(
        f"{url.rstrip('/')}/v1/chat/completions",
        headers={"Authorization": f"Bearer {key}"} if key else {},
        json={"model": model, "temperature": 0, "max_tokens": 1024, "messages": [{"role": "user", "content": [
            {"type": "text", "text": PROMPT.format(w=w, h=h)},
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(jpg).decode()}},
        ]}]},
        timeout=60,
    )
    r.raise_for_status()
    items = parse_items(r.json()["choices"][0]["message"]["content"], w, h)
    sx, sy = img.shape[1] / w, img.shape[0] / h
    for it in items:
        it["x"], it["y"] = it["x"] * sx, it["y"] * sy
    return items


def draw(img, items):
    out = img.copy()
    for it in items:
        p = (int(it["x"]), int(it["y"]))
        cv2.circle(out, p, 8, (0, 0, 255), -1)
        cv2.putText(out, f'{it["label"]} -> {it["bin"]}', (p[0] + 10, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 255), 2)
    return out


if __name__ == "__main__":
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")))
    if sys.argv[1:2] == ["photo"]:
        out = sys.argv[2] if len(sys.argv) > 2 else "photo.jpg"
        cv2.imwrite(out, Camera(cfg["camera"]).frame())
        print("->", out)
    elif sys.argv[1:2] == ["ask"]:
        img = cv2.imread(sys.argv[2])
        items = ask_llm(img, cfg["brev_url"], cfg["brev_model"])
        print(json.dumps(items, indent=1))
        cv2.imwrite("answer.jpg", draw(img, items))
        print("-> answer.jpg")
    else:
        reply = 'Sure!\n```json\n[{"label":"can","bin":"cans_bottles","x":10,"y":20},' \
                '{"label":"apple","bin":"compost","x":1,"y":1},{"label":"far","bin":"paper","x":5000,"y":1},' \
                '{"label":"bag","bin":"plastic","x":"7","y":8}]\n```'
        assert [i["label"] for i in parse_items(reply, 896, 672)] == ["can", "bag"]
        assert parse_items("no idea", 896, 672) == []
        H = fit_homography([(0, 0), (100, 0), (100, 100), (0, 100)], [(300, 100), (300, -100), (200, -100), (200, 100)])
        x, y = to_arm(H, 50, 50)
        assert abs(x - 250) < 1e-3 and abs(y) < 1e-3, (x, y)
        assert find_tags(Camera(None).frame()) == {}
        print("self-check ok")
