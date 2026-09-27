"""RoArm picks what the SO-101 camera sees: camera pixel -> RoArm x, y (homography from calibration) -> grab.

Setup (once):
  - SO-101 panel (http://<pi>:8765/): aim the camera at the pick area -> key M (look pose)
  - python roarm_pick.py calibrate tag: AprilTag 0 (AprilTags/) on the gripper, the RoArm placed (panel) so the camera
    sees the tag, 3-5 cm above the floor. The RoArm finds the floor, then puts the gripper at a small grid of points
    around that spot; the camera finds the tag -> pixel -> RoArm x, y. Measure tag_mm, tag_above_tip_mm and
    tag_offset_mm (roarm_calibration.json) if the tag is not at the gripper tip.
  - or python roarm_pick.py calibrate brev: same, but the VLM on Brev (Qwen2.5-VL) finds the gripper tip (no tag).

Commands:
    python roarm_pick.py where     where the SO-101 sees the bottle and where the RoArm would grab (RoArm does not move)
    python roarm_pick.py aim       RoArm stops above the bottle (no grab)
    python roarm_pick.py grab      RoArm grabs the bottle and lifts it
    python roarm_pick.py info      calibration summary
    python roarm_pick.py save-rest save where the RoArm is now as its rest pose (waits there for the vehicle)
    python roarm_pick.py poses     list the saved poses (RoArm panel: "Save pose")
    python roarm_pick.py save-pose NAME   save where the RoArm is now as pose NAME
    python roarm_pick.py goto NAME        move to pose NAME by joint angles
    python roarm_pick.py play [--restart] the saved poses in order; after a failure it continues at the failed step
    python roarm_pick.py rest      RoArm lifts straight up, then goes to its rest pose
    python roarm_pick.py --test    logic without hardware (fake RoArm and SO-101)

On the laptop, while so101_station.py runs on the Pi: SO101_URL=http://192.168.32.114:8765.
The RoArm never drives to a fixed pose: every move starts from where it is.
"""
import json
import math
import os
import re
import statistics
import sys
import time

import requests

_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(_dir, "..", "Raspberry"))  # on the laptop: roarm_wifi.py from Raspberry/
from roarm_wifi import Overheat, RoArm  # noqa: E402

CONFIG_FILE = os.path.join(_dir, "roarm_calibration.json")
TEAM_CONFIG = [os.path.expanduser("~/Hackengersi/Raspberry/config.json"),  # repo on the Pi (git pull = current IP)
               os.path.join(_dir, "..", "Raspberry", "config.json")]
DEFAULTS = {
    "roarm_ip": None,             # None = from Raspberry/config.json
    "so101_url": "http://localhost:8765",
    "speed": 0.2,                 # RoArm speed (firmware fraction, max 0.25: the shoulder heats up)
    "approach_mm": 80,            # above a grab point: first this much higher
    "grip_open": 1.57,
    "grip_closed": 3.14,
    "reach_mm": [120, 380],       # RoArm reach from its base (outside: do not try)
    "still_s": 1.0,               # a bottle must stay put this long before the RoArm reaches for it
    "retry_dz_mm": [0, -15, 15],  # grab attempts (bottle still on the table -> lower, then higher)
    "cal_step_mm": 70,            # calibration grid step around the centre
    "calibration": None,          # SO-101 image (look pose) -> RoArm x, y: python roarm_pick.py calibrate
    "rest": None,                 # joint angles where the RoArm waits for the vehicle: python roarm_pick.py save-rest
    "poses": {},                  # named joint-angle poses (RoArm panel "Save pose" / save-pose NAME)
    # --- calibration (tag / brev) ---
    "brev_url": None, "brev_model": None,  # None = from Raspberry/config.json (key: $BREV_KEY or ~/.bashrc)
    "cal_center": [250, 0],       # x, y (mm): grid centre - in RoArm reach and in the SO-101 camera view
    "cal_above_floor_mm": 5,      # gripper tip this far above the floor while the camera looks at it
    "cal_grip": 3.0,              # gripper during calibration: almost closed - 3.14 squeezes the jaws (servo at 57 C)
    "cal_tilt": 1.57, "cal_roll": 0.0,  # gripper tilt/roll during calibration (1.57 = down); "calibrate tag" takes
                                  # them, and the grid centre, from where the RoArm stands (placed so the tag is seen)
    "cal_vlm_agree_px": 30,       # full-frame px: two VLM answers (two frames) must agree, else the point is skipped
    "cal_ransac_mm": 20,          # a point further than this from the fitted homography is a bad answer, dropped
    # --- calibrate tag: AprilTag 36h11 on the gripper, facing the SO-101 camera ---
    "tag_id": 0,                  # AprilTags/tag36h11_00.png
    "tag_mm": 30,                 # side of the black square (measure after printing)
    "tag_above_tip_mm": 25,       # tag centre this far above the jaw tips (vertical tag) - converted to the tip
    # tag lying FLAT, face up, on a card flag next to the tip (gripper down): tag centre offset from the tip by
    # [away from base, to the left] mm - the calibration pair is then the tag centre. Then tag_above_tip_mm = 0.
    "tag_offset_mm": [0, 0],
    "grip_height_mm": 180,        # grab height above the floor: neck of a 0.5 l bottle (~20 cm tall, under the cap)
    # lying bottle (e.g. 1.5 l in the vehicle basket, ~9 cm thick - wider than the gripper): grab it by the neck
    "bottle_pose": "standing",    # "standing" (grab by the neck from above) or "lying" (neck found in the image)
    "cap_mm": 30,                 # lying: cap diameter (scale for the step below)
    "neck_from_cap_mm": 35,       # lying: neck centre this far from the cap centre, toward the bottle
    "neck_frac": 0.38,            # lying, no coloured cap found: neck this far from the middle, fraction of length
    "roll_offset": 0.0,           # rad: gripper roll that makes the jaws close ACROSS the bottle (tune with "aim")
    "floor_start_z": 0,           # mm: the floor search starts here (must be above the floor!)
    "floor_search_mm": 250,       # mm: at most this far down from floor_start_z (hard limit) - no floor = error
    "upside_down": False,         # True = RoArm hangs upside down: world "up" is -z of the RoArm
    "floor_step_mm": 5,
    "floor_speed": 0.1,
    "floor_pause_s": 0.5,         # after each step: servos settle, load settles
    "floor_load_delta": 60,       # shoulder/elbow load change (firmware units) vs. in the air = touching
    "floor_lag_mm": 6,            # or: the arm stays this far above the target (floor holds it), vs. the air error
    "floor_max_load": 350,        # |shoulder/elbow load| above this = STOP (like LOAD_STOP in roarm_panel.py)
}

IK_ON_PI = False  # True (cfg "ik_on_pi", default when upside_down): the Pi computes joint angles - see ik()
UP = 1  # sign of RoArm z for world "up": -1 when the RoArm hangs upside down (cfg "upside_down")


def load_config():
    cfg = dict(DEFAULTS)
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, encoding="utf-8") as f:
            cfg.update(json.load(f))
    if os.environ.get("SO101_URL"):  # roarm_pick.py on the laptop, so101_station.py on the Pi
        cfg["so101_url"] = os.environ["SO101_URL"]
    for p in TEAM_CONFIG:
        if os.path.exists(p):
            with open(p, encoding="utf-8") as f:
                team = json.load(f)
            for k in ("roarm_ip", "brev_url", "brev_model"):
                cfg[k] = cfg[k] or team.get(k)
            break
    globals()["UP"] = -1 if cfg["upside_down"] else 1
    globals()["IK_ON_PI"] = cfg.get("ik_on_pi", cfg["upside_down"])
    return cfg


def save_config(cfg):
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump({k: v for k, v in cfg.items() if k != "roarm_ip" or v}, f, indent=2, ensure_ascii=False)


# ----------------------------------------------------------------------------- RoArm kinematics (as in the firmware)
# The firmware (T:104) computes joints from x y z itself and always picks the same elbow branch; a RoArm hanging upside
# down in the other branch flips its elbow on the first move (measured 2026-09-27: elbow 0.64 -> 0.17 rad, 7 cm off).
# Same maths as RoArm-M3_module.h, but with the solution nearest the current joints, sent as T:102 (joint angles).
L2 = math.hypot(236.82, 30.0)
T2 = math.atan2(30.0, 236.82)
L3 = 144.49
LE = math.hypot(171.67, 13.69)
TE = math.atan2(13.69, 171.67)


def fk(b, s, e, t):
    """Joint angles (rad) -> (x, y, z, tit) of the tip, like the firmware feedback (RoArmM3_computePosbyJointRad)."""
    a1, a2, a3 = math.pi / 2 - (s + T2), math.pi / 2 - (e + s), math.pi / 2 - (e + s + t + TE)
    r = L2 * math.cos(a1) + L3 * math.cos(a2) + LE * math.cos(a3)
    z = L2 * math.sin(a1) + L3 * math.sin(a2) + LE * math.sin(a3)
    return r * math.cos(b), r * math.sin(b), z, e + s + t - math.pi / 2


def ik(x, y, z, tit, now):
    """(x, y, z, tit) -> (b, s, e, t) nearest the joints `now` (dict b s e t from feedback). None = out of reach
    or outside the firmware limits (shoulder +-pi/2, elbow 0..pi, wrist +-pi/2)."""
    b = math.atan2(y, x)
    r = math.hypot(x, y)
    phi = -(tit + TE)  # gripper direction (from horizontal, up +)
    wr, wz = r - LE * math.cos(phi), z - LE * math.sin(phi)
    d2 = wr * wr + wz * wz
    c = (d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(c) > 1:
        return None
    solutions = []
    for delta in (math.acos(c), -math.acos(c)):  # link 3 angle relative to link 2 - the two elbow branches
        a = math.atan2(wz, wr) - math.atan2(L3 * math.sin(delta), L2 + L3 * math.cos(delta))
        s = math.pi / 2 - T2 - a
        e = math.pi / 2 - s - (a + delta)
        t = tit + math.pi / 2 - e - s
        if -math.pi / 2 <= s <= math.pi / 2 and 0 <= e <= math.pi and -math.pi / 2 <= t <= math.pi / 2:
            solutions.append((b, s, e, t))
    return min(solutions, key=lambda q: abs(q[1] - now["s"]) + abs(q[2] - now["e"]), default=None)


def command(arm, x, y, z, t, r, g, spd):
    """Move the tip to x y z (tilt t, roll r, gripper g). IK_ON_PI: angles from ik() -> T:102, else T:104."""
    if not IK_ON_PI:
        arm.send({"T": 104, "x": round(x, 1), "y": round(y, 1), "z": round(z, 1), "t": round(t, 3), "r": round(r, 3),
                  "g": round(g, 3), "spd": spd})
        return
    now = arm.where()
    # wrist at its limit (+-pi/2): the nearest tilt that works (the tip still reaches x y z; the tag stays visible)
    q = next((q for dt in (0.0, *(k * 0.02 * sg for k in range(1, 26) for sg in (-1, 1)))
              for q in [ik(x, y, z, t + dt, now)] if q), None)
    if q is None:
        raise RuntimeError(f"RoArm cannot reach x={x:.0f} y={y:.0f} z={z:.0f} tilt {t:.2f}")
    b, s, e, tt = q
    arm.send({"T": 102, "base": round(b, 4), "shoulder": round(s, 4), "elbow": round(e, 4), "wrist": round(tt, 4),
              "roll": round(r, 3), "hand": round(g, 3), "spd": int(spd * 1000), "acc": 10})


def go_to(arm, p, g=None, dz=0.0, droll=0.0, spd=0.2, tol=30.0, timeout=20.0):
    """To point p (+dz mm up in the world, +droll rad roll). Waits until the measured x y z is within tol.
    tol 30 mm: the Feetech shoulder hangs up to ~25 mm off target with the arm stretched (measured 2026-09-26)."""
    r = max(-3.14, min(3.14, p["r"] + droll))
    target = {"x": p["x"], "y": p["y"], "z": p["z"] + UP * dz}
    try:
        command(arm, target["x"], target["y"], target["z"], p["t"], r, DEFAULTS["grip_open"] if g is None else g, spd)
    except RuntimeError as e:
        if "not answering" not in str(e):
            raise  # e.g. "Queue full"
        # the RoArm does not answer while it moves (longer move) - the command arrived: watch the position below
    if arm.mock:
        arm.fb.update(target, r=r)
        return
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.3)
        try:
            w = arm.where()
        except RuntimeError:
            continue  # busy moving
        if max(abs(w[k] - target[k]) for k in target) < tol:
            return
    raise TimeoutError(f"RoArm did not reach {target} (it is at {arm.where()})")


class SO101:
    """so101_station.py (SO-101 with the camera) over HTTP."""

    def __init__(self, url):
        self.url = url.rstrip("/")

    def _get(self, path, **params):
        return requests.get(self.url + path, params=params, timeout=3).json()

    def bottles(self):
        return self._get("/bottles")

    def tracking(self, on):
        self._get("/tracking", on=int(bool(on)))

    def look(self):
        answer = self._get("/look")
        if answer.get("error"):
            raise RuntimeError(answer["error"])

    def frame(self):
        """Fresh full-resolution frame from /frame (1920x1080 - a small tag comes out sharper) -> (jpg, width, height).
        Waits 0.3 s so the frame is from after the last move, not during it."""
        import cv2
        import numpy as np

        time.sleep(0.3)
        r = requests.get(self.url + "/frame", timeout=5)
        r.raise_for_status()
        h, w = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_GRAYSCALE).shape
        return r.content, w, h

    def status(self, text):
        """RoArm status line in the /demo view (network errors do not matter)."""
        try:
            self._get("/roarm_status", text=text)
        except requests.RequestException:
            pass


# ----------------------------------------------------------------------------- SO-101 image -> RoArm table
def bottle_base(b):
    """Where the bottle stands on the table: middle of the bottom edge of its box (table plane)."""
    return b["cx"], b["box"][3]


def to_calibration_pose(u, v, view, cal_joints):
    """The camera returns to the look pose within ~1-3 deg - shift the pixel as if it stood exactly as during
    calibration (px_per_deg: how many px the image moves per 1 deg of the joint)."""
    for joint, px in (view.get("px_per_deg") or {}).items():
        if joint in cal_joints and joint in view.get("joints", {}):
            d = view["joints"][joint] - cal_joints[joint]
            if joint == "pan":
                u -= px * d
            else:
                v -= px * d
    return u, v


def to_table(cal, u, v):
    """Pixel (look pose) -> RoArm x, y in mm (homography from calibration)."""
    H = cal["H"]
    w = H[2][0] * u + H[2][1] * v + H[2][2]
    return (H[0][0] * u + H[0][1] * v + H[0][2]) / w, (H[1][0] * u + H[1][1] * v + H[1][2]) / w


def wait_for_look(so, timeout=40.0):
    """SO-101 in its look pose and still (only then is the image a map of the table)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        w = so.bottles()
        if w.get("at_look") and w.get("still"):
            return w
        time.sleep(0.25)
    raise TimeoutError("SO-101 did not return to its look pose (saved with key M?)")


def bottle_point(b, lying):
    """Pixel that marks where a bottle is: base of a standing bottle, middle of a lying one."""
    return (b["cx"], b["cy"]) if lying else bottle_base(b)


def standing_bottle(so, still_s=1.0, timeout=None, skip=(), lying=False):
    """Wait until a bottle stays still on the table (same place for `still_s` s). Returns (u, v, view, b) -
    bottle_point - or None after timeout. skip = pixels not to take (e.g. a bottle that cannot be grabbed)."""
    deadline = None if timeout is None else time.monotonic() + timeout
    history = []
    while deadline is None or time.monotonic() < deadline:
        w = so.bottles()
        good = [b for b in w.get("bottles", []) if b["cls"] == "bottle" and (lying or b["box"][3] < w["height"] - 4)
                and all(math.hypot(bottle_point(b, lying)[0] - pu, bottle_point(b, lying)[1] - pv) > 40
                        for pu, pv in skip)]
        if not (w.get("at_look") and w.get("still")) or not good:
            history = []
        else:
            b = max(good, key=lambda b: (b["box"][2] - b["box"][0]) * (b["box"][3] - b["box"][1]))
            u, v = bottle_point(b, lying)
            now = time.monotonic()
            history = [h for h in history if math.hypot(h[1] - u, h[2] - v) < 15] + [(now, u, v)]
            if now - history[0][0] >= still_s and len(history) >= 4:
                return statistics.median(h[1] for h in history), statistics.median(h[2] for h in history), w, b
        time.sleep(0.15)
    return None


def in_reach(cfg, x, y):
    lo, hi = cfg["reach_mm"]
    return lo <= math.hypot(x, y) <= hi


# ----------------------------------------------------------------------------- calibration: find the gripper tip
VLM_SIZE = (896, 504)  # 16:9 like the camera, multiple of 28 (Qwen-VL grid) -> the VLM answers in these pixels
TIP_PROMPT = """This {w}x{h} image is from a camera looking at a table. A black robot arm (RoArm) may reach into
view with its gripper pointing straight down at the table. Give the pixel point of the very tip of the gripper jaws:
the lowest point of the gripper, just above the table surface.
If no robot gripper is visible, answer {{"tip": null}}.
Answer with only JSON: {{"tip": [x, y]}}"""


def brev_key():
    """BREV_KEY from the environment, or on the Pi (the service does not read ~/.bashrc) from export BREV_KEY=..."""
    if os.environ.get("BREV_KEY"):
        return os.environ["BREV_KEY"]
    try:
        with open(os.path.expanduser("~/.bashrc"), encoding="utf-8") as f:
            m = re.search(r"^\s*export\s+BREV_KEY=['\"]?([\w-]+)", f.read(), re.M)
        return m.group(1) if m else None
    except OSError:
        return None


def tip_from_answer(text, w, h):
    """VLM answer -> (x, y) in a w x h image, or None (no gripper, garbage, point outside the image)."""
    m = re.search(r"\{.*\}", text, re.S)  # models like ```json and prose around it
    try:
        tip = json.loads(m.group(0))["tip"] if m else None
        x, y = float(tip[0]), float(tip[1])
    except (ValueError, KeyError, TypeError, IndexError):
        return None
    return (x, y) if 0 <= x < w and 0 <= y < h else None


def ask_brev(jpg, width, height, cfg):
    """Where in the frame (width x height) is the RoArm gripper tip - asks Qwen2.5-VL on Brev. (u, v) or None."""
    import base64

    import cv2
    import numpy as np

    w, h = VLM_SIZE
    img = cv2.resize(cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR), VLM_SIZE)
    small = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1]
    key = brev_key()
    r = requests.post(f"{cfg['brev_url'].rstrip('/')}/v1/chat/completions", timeout=60,
                      headers={"Authorization": f"Bearer {key}"} if key else {},
                      json={"model": cfg["brev_model"], "temperature": 0, "max_tokens": 60, "messages": [
                          {"role": "user", "content": [
                              {"type": "text", "text": TIP_PROMPT.format(w=w, h=h)},
                              {"type": "image_url", "image_url": {
                                  "url": "data:image/jpeg;base64," + base64.b64encode(small).decode()}}]}]})
    r.raise_for_status()
    xy = tip_from_answer(r.json()["choices"][0]["message"]["content"], w, h)
    return None if xy is None else (xy[0] * width / w, xy[1] * height / h)


def find_tag(jpg, width, height, cfg):
    """Gripper tip from the AprilTag on the jaw: tag centre shifted down by tag_above_tip_mm.
    Scale from the tag's own vertical edge (the tag stands vertical like that distance, so foreshortening matches).
    (u, v) in frame pixels, or None when the tag is not visible."""
    import cv2
    import numpy as np

    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    corners, ids, _ = det.detectMarkers(img)
    for c, i in zip(corners, [] if ids is None else ids.ravel()):
        if i != cfg["tag_id"]:
            continue
        c = c.reshape(4, 2)  # top-left, top-right, bottom-right, bottom-left
        vertical = (np.linalg.norm(c[3] - c[0]) + np.linalg.norm(c[2] - c[1])) / 2  # px per tag_mm vertically
        u, v = c.mean(axis=0)
        v += cfg["tag_above_tip_mm"] * vertical / cfg["tag_mm"]
        return (float(u), float(v)) if 0 <= v < height else None
    return None


def tag_xy(cfg, x, y):
    """Tag centre on a flag: tip (x, y) + offset [away from base, left], rotated with the RoArm base."""
    d_r, d_t = cfg.get("tag_offset_mm") or (0, 0)
    b = math.atan2(y, x)
    return x + d_r * math.cos(b) - d_t * math.sin(b), y + d_r * math.sin(b) + d_t * math.cos(b)


def temps_text(arm):
    return "temp " + ("/".join(f"{v:.0f}" for v in arm.temps.values()) + " C" if arm.temps else "?")


def position(arm):
    """arm.where() with one retry (the RoArm is sometimes busy right after a command)."""
    try:
        return arm.where()
    except RuntimeError:
        time.sleep(0.3)
        return arm.where()


def hold_here(arm, w):
    """STOP = command the measured joint angles (never T:0: it freezes the firmware and drops torque)."""
    arm.send({"T": 102, "base": w["b"], "shoulder": w["s"], "elbow": w["e"], "wrist": w["t"], "roll": w["r"],
              "hand": w["g"], "spd": 50, "acc": 10})


def find_floor(arm, cfg, x, y, log=print):
    """The gripper tip (almost closed) steps down above (x, y) until it touches the floor -> floor z (mm).

    Touch = shoulder/elbow load differs from the load in the air, or the arm stays above the target (floor holds it).
    The z error in the air is subtracted: the Feetech shoulder hangs a little off target anyway."""
    grip, spd, pause = cfg["cal_grip"], cfg["floor_speed"], cfg["floor_pause_s"]
    p = {"x": x, "y": y, "z": cfg["floor_start_z"], "t": cfg["cal_tilt"], "r": cfg["cal_roll"]}
    go_to(arm, p, grip, spd=spd)
    time.sleep(pause)
    samples = [position(arm) for _ in range(3)]
    base = {k: statistics.median(w[k] for w in samples) for k in ("tS", "tE")}
    z_err = statistics.median(w["z"] for w in samples) - p["z"]
    z = p["z"]
    for _ in range(int(cfg["floor_search_mm"] // cfg["floor_step_mm"])):
        z -= UP * cfg["floor_step_mm"]
        command(arm, x, y, z, p["t"], p["r"], grip, spd)
        time.sleep(pause)
        w = position(arm)
        d_s, d_e, above = w["tS"] - base["tS"], w["tE"] - base["tE"], UP * (w["z"] - z - z_err)
        log(f"  floor? z={w['z']:.0f} (target {z:.0f}) shoulder {d_s:+.0f} elbow {d_e:+.0f} above target "
            f"{above:+.0f} mm | {temps_text(arm)}")
        if max(abs(w["tS"]), abs(w["tE"])) > cfg["floor_max_load"]:
            hold_here(arm, w)
            raise RuntimeError(f"load too high at the floor (shoulder {w['tS']}, elbow {w['tE']}) - STOP")
        if max(abs(d_s), abs(d_e)) > cfg["floor_load_delta"] or above > cfg["floor_lag_mm"]:
            go_to(arm, dict(p, z=w["z"] + UP * 10), grip, spd=spd)  # back off the floor
            return w["z"]
    raise RuntimeError(f"no floor within {cfg['floor_search_mm']} mm below z={p['z']:.0f} (floor_search_mm; "
                       f"RoArm upside down but upside_down = false?)")


def calibrate(arm, so, cfg, locate=ask_brev, log=print, source="brev"):
    """Calibration without a bottle: the RoArm puts its gripper tip just above the floor at a grid of points, and
    `locate` (VLM on Brev, or the AprilTag) says where the tip is in the SO-101 image -> homography image -> table."""
    import cv2
    import numpy as np

    grip, spd = cfg["cal_grip"], cfg["floor_speed"]
    log(f"CALIBRATION ({source}). SO-101 looks at the table...")
    so.tracking(False)
    so.look()
    joints = dict(wait_for_look(so)["joints"])
    x0, y0 = cfg["cal_center"]
    if not in_reach(cfg, x0, y0):
        raise RuntimeError(f"cal_center {x0}, {y0} outside RoArm reach {cfg['reach_mm']}")
    so.status("calibration: looking for the floor")
    floor_z = find_floor(arm, cfg, x0, y0, log=log)
    log(f"Floor: z={floor_z:.0f} mm | {temps_text(arm)}")
    so.status("calibration: finding the gripper")
    k = cfg["cal_step_mm"]
    pairs = []
    for dx, dy in ((0, 0), (k, 0), (0, k), (-k, 0), (0, -k), (k, k), (-k, -k), (k, -k), (-k, k)):
        x, y = x0 + dx, y0 + dy
        if not in_reach(cfg, x, y):
            continue
        up = {"x": x, "y": y, "z": floor_z + UP * cfg["approach_mm"], "t": cfg["cal_tilt"], "r": cfg["cal_roll"]}
        go_to(arm, up, grip, spd=spd)
        low = floor_z + UP * cfg["cal_above_floor_mm"]
        go_to(arm, dict(up, z=low), grip, spd=spd)
        hang = UP * (position(arm)["z"] - low)  # tip higher than commanded: correct once by the measured difference
        if hang > 5:
            go_to(arm, dict(up, z=low - UP * hang), grip, spd=spd)
        here = position(arm)  # pair from the MEASURED position (x, y), not the command
        view = wait_for_look(so)
        answers = []
        for _ in range(2):  # two frames, two answers: a random answer does not repeat
            jpg, w_img, h_img = so.frame()
            uv = locate(jpg, w_img, h_img, cfg)
            answers.append(None if uv is None else (uv[0] * view["width"] / w_img, uv[1] * view["height"] / h_img))
        go_to(arm, up, grip, spd=spd)  # up before moving on (no dragging on the floor)
        if None in answers:
            log(f"  x={x:.0f} y={y:.0f}: gripper not found (out of view?) - skipped | {temps_text(arm)}")
            continue
        spread = math.hypot(answers[0][0] - answers[1][0], answers[0][1] - answers[1][1])
        if spread > cfg["cal_vlm_agree_px"]:
            log(f"  x={x:.0f} y={y:.0f}: the two answers differ by {spread:.0f} px - skipped")
            continue
        u, v = (answers[0][0] + answers[1][0]) / 2, (answers[0][1] + answers[1][1]) / 2
        tx, ty = tag_xy(cfg, here["x"], here["y"])
        pairs.append({"px": list(to_calibration_pose(u, v, view, joints)), "x": tx, "y": ty})
        log(f"  point {len(pairs)}: ({pairs[-1]['px'][0]:.0f}, {pairs[-1]['px'][1]:.0f}) px -> x={here['x']:.0f} "
            f"y={here['y']:.0f} mm, tip {UP * (here['z'] - floor_z):+.0f} mm above the floor | {temps_text(arm)}")
        if len(pairs) >= 8:
            break
    if len(pairs) < 4:
        raise RuntimeError(f"only {len(pairs)} points with the gripper in view - change cal_center or the camera pose")

    src = np.float32([p["px"] for p in pairs])
    dst = np.float32([[p["x"], p["y"]] for p in pairs])
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, cfg["cal_ransac_mm"])
    if H is None:
        raise RuntimeError("calibration failed - repeat it")
    good = [p for p, m in zip(pairs, mask.ravel()) if m]
    if len(good) < 4:
        raise RuntimeError(f"only {len(good)} consistent points (the rest: bad answers) - repeat it")
    # grab: bottle from above by the neck. The camera saw the tip at floor height, and picking uses the bottle base
    # in the image - both on the floor plane, so the homography holds; grab height = floor + grip_height_mm.
    grip_pose = {"z": floor_z + UP * cfg["grip_height_mm"], "t": UP * 1.57, "r": 0.0}  # t: gripper pointing down
    cal = {"H": H.tolist(), "joints": joints, "width": view["width"], "height": view["height"], "pairs": good,
           "grip": grip_pose, "center": [statistics.mean(p["x"] for p in good), statistics.mean(p["y"] for p in good)],
           "floor_z": floor_z, "source": source}
    errors = [math.hypot(*(a - b for a, b in zip(to_table(cal, *p["px"]), (p["x"], p["y"])))) for p in good]
    cfg["calibration"] = cal
    save_config(cfg)
    so.status("calibration done")
    log(f"\nDone: {len(good)}/{len(pairs)} points, mean error {statistics.mean(errors):.0f} mm, max "
        f"{max(errors):.0f} mm" + (" - large: repeat" if max(errors) > 25 else " - OK") + f" | {temps_text(arm)}")
    return cal


# ----------------------------------------------------------------------------- picking
def bottle_axis(img, box, neck_frac=0.38):
    """Lying bottle inside `box` (grey or BGR image) -> (neck_u, neck_v, axis_angle) in image pixels / rad, or None.
    Axis = main direction of the edge pixels (PCA); the neck end is the narrower one (its edge spread is smaller)."""
    import cv2
    import numpy as np

    x0, y0, x1, y1 = (max(0, int(v)) for v in box)
    crop = img[y0:y1, x0:x1]
    if crop.ndim == 3:
        crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    ys, xs = np.nonzero(cv2.Canny(cv2.GaussianBlur(crop, (5, 5), 0), 40, 120))
    if len(xs) < 50:
        return None
    pts = np.column_stack([xs, ys]).astype(np.float64)
    c = pts.mean(axis=0)
    d = np.linalg.svd(pts - c, full_matrices=False)[2][0]  # axis direction (unit)
    along, across = (pts - c) @ d, (pts - c) @ np.array([-d[1], d[0]])
    lo, hi = np.percentile(along, 2), np.percentile(along, 98)
    length = hi - lo

    def width(sel):  # spread of the edges across the axis in the outer 20% of one end
        a = across[sel]
        return np.percentile(a, 95) - np.percentile(a, 5) if a.size > 10 else float("inf")
    sign = 1 if width(along > hi - 0.2 * length) < width(along < lo + 0.2 * length) else -1
    neck = c + d * ((lo + hi) / 2 + sign * neck_frac * length)
    return float(neck[0] + x0), float(neck[1] + y0), math.atan2(d[1], d[0])


def cap_neck(img_bgr, box, cap_mm=30.0, neck_from_cap_mm=35.0):
    """Lying bottle: its coloured cap -> (neck_u, neck_v, axis_angle) or None (white/clear cap, not found).
    YOLO boxes of a lying clear bottle often cut off the neck, and the cap is the clearest part: a small, roughly
    round, saturated blob near the bottle, the one furthest from the bottle middle (the label is near the middle).
    Scale from the cap itself (cap_mm across); the neck is neck_from_cap_mm from the cap toward the bottle."""
    import cv2
    import numpy as np

    x0, y0, x1, y1 = box
    bw, bh = x1 - x0, y1 - y0
    X0, Y0 = max(0, int(x0 - 0.3 * bw)), max(0, int(y0 - 0.3 * bh))  # the cap may lie outside the box
    X1, Y1 = min(img_bgr.shape[1], int(x1 + 0.3 * bw)), min(img_bgr.shape[0], int(y1 + 0.3 * bh))
    hsv = cv2.cvtColor(img_bgr[Y0:Y1, X0:X1], cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 1] > 120) & (hsv[..., 2] > 60)).astype(np.uint8)
    n, _, stats, centres = cv2.connectedComponentsWithStats(mask)
    middle = np.array([(x0 + x1) / 2 - X0, (y0 + y1) / 2 - Y0])
    size = max(bw, bh)
    caps = [(np.linalg.norm(centres[i] - middle), i) for i in range(1, n)
            if 0.04 * size < max(stats[i, 2], stats[i, 3]) < 0.25 * size   # cap-sized relative to the bottle
            and 0.6 < stats[i, 2] / stats[i, 3] < 1.6                         # roughly round
            and stats[i, 4] > 0.5 * stats[i, 2] * stats[i, 3]]                # solid, not a thin line
    if not caps:
        return None
    dist, i = max(caps)
    if dist < 0.25 * size:  # a coloured blob near the middle is the label, not a cap
        return None
    cap = centres[i]
    d = (middle - cap) / dist  # from the cap toward the bottle
    px_per_mm = (stats[i, 2] + stats[i, 3]) / 2 / cap_mm
    neck = cap + d * neck_from_cap_mm * px_per_mm
    return float(neck[0] + X0), float(neck[1] + Y0), math.atan2(-d[1], -d[0])


def jaw_roll(cfg, x, y, axis):
    """Gripper roll so the jaws close across a bottle whose axis points at `axis` (rad, RoArm frame) at (x, y).
    Jaws are symmetric, so the roll is taken within +-pi/2; roll_offset calibrates the jaw direction at roll 0."""
    r = axis + math.pi / 2 - math.atan2(y, x) + cfg["roll_offset"]
    return (r + math.pi / 2) % math.pi - math.pi / 2


def target_on_table(so, cfg, timeout=None, skip=()):
    """Bottle on the table -> ((x, y) of the RoArm, (u, v) pixel, axis angle in the RoArm frame or None);
    None after timeout. Lying bottle: the target is its neck, found in a full-resolution frame."""
    cal = cfg["calibration"]
    lying = cfg["bottle_pose"] == "lying"
    found = standing_bottle(so, still_s=cfg["still_s"], timeout=timeout, skip=skip, lying=lying)
    if found is None:
        return None
    u, v, w, b = found
    axis_px = None
    if lying:
        import cv2
        import numpy as np

        jpg, fw, _ = so.frame()
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        k = fw / w["width"]  # /frame and /bottles pixels may differ in scale
        box = [c * k for c in b["box"]]
        found_axis = cap_neck(img, box, cfg["cap_mm"], cfg["neck_from_cap_mm"]) or \
            bottle_axis(img, box, cfg["neck_frac"])
        if found_axis:
            u, v, axis_px = found_axis[0] / k, found_axis[1] / k, found_axis[2]

    def table(pu, pv):
        if w["width"] != cal["width"]:  # another camera resolution than during calibration
            pu, pv = pu * cal["width"] / w["width"], pv * cal["height"] / w["height"]
        return to_table(cal, *to_calibration_pose(pu, pv, w, cal["joints"]))
    x, y = table(u, v)
    axis = None
    if axis_px is not None:  # axis direction on the table: a second point 40 px along the axis in the image
        x2, y2 = table(u + 40 * math.cos(axis_px), v + 40 * math.sin(axis_px))
        axis = math.atan2(y2 - y, x2 - x)
    return (x, y), (u, v), axis


def still_on_table(so, u, v, seconds=1.2, lying=False):
    """After a grab: is the bottle still where it was (the grab failed)? Lying: any bottle box around (u, v)."""
    deadline, hits, tries = time.monotonic() + seconds, 0, 0
    while time.monotonic() < deadline:
        w = so.bottles()
        tries += 1
        if any((b["box"][0] <= u <= b["box"][2] and b["box"][1] <= v <= b["box"][3]) if lying
               else math.hypot(bottle_base(b)[0] - u, bottle_base(b)[1] - v) < 30 for b in w.get("bottles", [])):
            hits += 1
        time.sleep(0.2)
    return tries > 0 and hits >= max(2, tries // 2)


def grab(arm, so, cfg, pick, px=None, log=print):
    """Grab the bottle at point `pick` and lift it. px = where it is seen - then the camera checks, and the grab is
    retried lower/higher. True = holding it (or it cannot be checked)."""
    opened, closed, spd, dz = cfg["grip_open"], cfg["grip_closed"], cfg["speed"], cfg["approach_mm"]
    for i, retry in enumerate(cfg["retry_dz_mm"] if px else [0]):
        p = dict(pick, z=pick["z"] + UP * retry)
        if i:
            log(f"   the bottle is still on the table - attempt {i + 1} ({retry:+d} mm)")
            so.status(f"grabbing again ({i + 1})")
        go_to(arm, p, opened, dz=dz, spd=spd)
        go_to(arm, p, opened, spd=spd)
        arm.gripper(closed)
        go_to(arm, p, closed, dz=dz, spd=spd)
        if not px or not still_on_table(so, *px, lying=cfg["bottle_pose"] == "lying"):
            return True
        arm.gripper(opened)
    return False


def go_rest(arm, cfg, timeout=20.0):
    """To the saved rest pose: first straight up by approach_mm (out of the basket), then joint angles (T:102).
    Joint angles, not x y z: the pose is reached exactly as it was saved, whichever elbow branch that was."""
    rest = (cfg.get("poses") or {}).get("rest") or cfg["rest"]  # the panel's "rest" pose wins over save-rest
    if not rest:
        raise RuntimeError("no rest pose: place the RoArm and run python roarm_pick.py save-rest")
    w = arm.where()
    go_to(arm, {"x": w["x"], "y": w["y"], "z": w["z"], "t": w["tit"], "r": w["r"]}, w["g"], dz=cfg["approach_mm"],
          spd=cfg["speed"])
    arm.send({"T": 102, "base": rest["b"], "shoulder": rest["s"], "elbow": rest["e"], "wrist": rest["t"],
              "roll": rest["r"], "hand": rest["g"], "spd": int(cfg["speed"] * 1000), "acc": 10})
    if arm.mock:
        return
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.3)
        try:
            w = arm.where()
        except RuntimeError:
            continue
        if max(abs(w[k] - rest[k]) for k in "bset") < 0.08:
            return
    raise TimeoutError(f"RoArm did not reach its rest pose (it is at {arm.where()})")


def go_pose(arm, cfg, name, timeout=25.0):
    """To saved pose `name` by joint angles (T:102): every joint straight to its angle; waits until there."""
    pose = cfg["poses"][name]
    arm.send({"T": 102, "base": pose["b"], "shoulder": pose["s"], "elbow": pose["e"], "wrist": pose["t"],
              "roll": pose["r"], "hand": pose["g"], "spd": int(cfg["speed"] * 1000), "acc": 10})
    if arm.mock:
        return
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.3)
        try:
            w = arm.where()
        except RuntimeError:
            continue
        if max(abs(w[k] - pose[k]) for k in "bset") < 0.08:
            return
    raise TimeoutError(f"RoArm did not reach pose '{name}' (it is at {arm.where()})")


PROGRESS_FILE = os.path.join(_dir, "pick_progress.json")  # which step of the sequence is next (survives a crash)


def play(arm, cfg, restart=False, log=print, progress_file=None, retries=2, cool_wait_s=600.0):
    """Taught pick: the saved poses one after another (cfg "sequence", else the order they were saved in, starting
    and ending at "rest" - the arm finishes where it started).

    Progress is written after every reached pose, so a failed run continues AT the failed step next time (or right
    away: a timed-out move is retried `retries` times; overheat / RoArm not answering -> wait until it cools down /
    answers again, then the same step). restart=True starts from the first pose. Returns True when all are done."""
    progress_file = progress_file or PROGRESS_FILE
    poses = cfg.get("poses") or {}
    sequence = cfg.get("sequence") or list(poses)
    if not cfg.get("sequence") and "rest" in poses:
        sequence = ["rest"] + [n for n in sequence if n != "rest"] + ["rest"]
    missing = [n for n in sequence if n not in poses]
    if not sequence or missing:
        raise RuntimeError(f"sequence {sequence}: missing poses {missing}" if missing else "no poses saved")
    step = 0
    if not restart and os.path.exists(progress_file):
        with open(progress_file, encoding="utf-8") as f:
            saved = json.load(f)
        if saved.get("sequence") == sequence and 0 <= saved.get("next", 0) < len(sequence):
            step = saved["next"]
            log(f"continuing from step {step + 1}/{len(sequence)} ({sequence[step]})")

    def save(nxt):
        with open(progress_file, "w", encoding="utf-8") as f:
            json.dump({"sequence": sequence, "next": nxt, "t": time.time()}, f)

    while step < len(sequence):
        name = sequence[step]
        save(step)  # a crash from here on resumes at this step
        failures = 0
        while True:
            try:
                log(f"step {step + 1}/{len(sequence)}: {name}")
                go_pose(arm, cfg, name)
                break
            except TimeoutError as e:
                failures += 1
                if failures > retries:
                    log(f"step {step + 1} ({name}) failed {failures}x - stopped; run play again to continue here")
                    raise
                log(f"  did not reach {name} ({e}) - retry {failures}/{retries}")
            except Overheat as e:
                log(f"  {e} - waiting for it to cool down, then {name} again")
                deadline = time.monotonic() + cool_wait_s
                while True:
                    time.sleep(10)
                    try:
                        arm.check_temp(max_age=0)
                        break
                    except Overheat:
                        if time.monotonic() > deadline:
                            raise
            except (RuntimeError, requests.RequestException) as e:  # RoArm not answering (power, USB bridge)
                failures += 1
                if failures > retries + 3:
                    raise
                log(f"  RoArm not answering ({e}) - waiting 5 s, then {name} again")
                time.sleep(5)
        step += 1
    save(len(sequence))
    log("sequence done")
    return True


# ----------------------------------------------------------------------------- test without hardware
def test():
    """Fake RoArm + fake camera: the table seen through the homography px -> mm = (u/2, v/2 + 100)."""
    opened, closed = DEFAULTS["grip_open"], DEFAULTS["grip_closed"]
    world = {"bottle": (230.0, 240.0), "held": False}  # where the bottle stands (RoArm mm) / is it in the gripper

    class FakeSO:
        def __init__(self):
            self.tracks, self.looks, self.texts = None, 0, []

        def tracking(self, on):
            self.tracks = on

        def look(self):
            self.looks += 1

        def status(self, t):
            self.texts.append(t)

        def frame(self):
            return b"", 640, 360

        def bottles(self):  # camera 3 deg off the calibration pose (pan 13 instead of 10) -> image shifted
            bottles = []
            if not world["held"]:
                x, y = world["bottle"]
                u, v = 2 * x - 19.2 * 3, 2 * (y - 100)
                bottles = [{"cls": "bottle", "conf": 0.9, "cx": u, "cy": v - 100, "box": [u - 40, v - 200, u + 40, v]}]
            return {"width": 1920, "height": 1080, "at_look": True, "still": True, "joints": {"pan": 13.0, "wflex": 0.0},
                    "px_per_deg": {"pan": -19.2, "wflex": -16.2}, "bottles": bottles}

    orig_save = globals()["save_config"]
    globals()["save_config"] = lambda c: None  # the test does not overwrite the real file
    try:
        cfg = dict(DEFAULTS, still_s=0.3, calibration=None)

        # calibration: floor at z=-100, the fake VLM sees the gripper tip (640x360 frame);
        # of the 9 grid points: 3 out of reach (>380 mm), 1 out of view -> 5 pairs
        floor = -100.0
        arm = RoArm("mock", mock=True)
        arm.fb.update(x=230.0, y=240.0, z=50.0, tit=1.57, r=0.0, tS=-150, tE=150)

        def send(c):
            if c.get("T") == 104:  # the floor stops the arm and pushes on the shoulder
                arm.fb.update(x=c["x"], y=c["y"], z=max(c["z"], floor), tS=-150 + (90 if c["z"] < floor else 0))
        arm.send = send

        def fake_vlm(jpg, w, h, c):
            x, y = arm.fb["x"], arm.fb["y"]
            if (x, y) == (160.0, 170.0):
                return None  # gripper out of view here
            return (2 * x - 19.2 * 3) * w / 1920, 2 * (y - 100) * h / 1080

        world["held"] = True  # no bottle on the table
        cfg2 = dict(cfg, cal_center=[230, 240], floor_pause_s=0)
        so = FakeSO()
        cal = calibrate(arm, so, cfg2, locate=fake_vlm, log=lambda *_: None)
        assert abs(cal["floor_z"] - floor) < 1 and cal["grip"]["z"] == floor + cfg2["grip_height_mm"], cal
        assert len(cal["pairs"]) == 5 and all((p["x"], p["y"]) != (160.0, 170.0) for p in cal["pairs"]), cal["pairs"]
        x, y = to_table(cal, *to_calibration_pose(2 * 260 - 19.2 * 3, 2 * (210 - 100), so.bottles(), cal["joints"]))
        assert abs(x - 260) < 2 and abs(y - 210) < 2, (x, y)
        assert so.tracks is False and so.looks == 1 and "calibration done" in so.texts
        assert tip_from_answer('```json\n{"tip": [100, 50]}\n```', 896, 504) == (100.0, 50.0)
        assert tip_from_answer('{"tip": null}', 896, 504) is None
        assert tip_from_answer('{"tip": [900, 50]}', 896, 504) is None and tip_from_answer("no", 9, 9) is None

        # picking: the bottle stands at (260, 180) -> seen -> RoArm x, y; the grab lifts it
        cfg["calibration"] = cal
        world.update(bottle=(260.0, 180.0), held=False)
        (x, y), px, _ = target_on_table(so, cfg, timeout=5)
        assert abs(x - 260) < 2 and abs(y - 180) < 2, (x, y)
        arm2 = RoArm("mock", mock=True)
        arm2.fb.update(x=200.0, y=0.0, z=50.0, tit=1.57, r=0.0)
        arm2.send = lambda c: None

        def gripper(g):
            near = math.hypot(arm2.fb["x"] - world["bottle"][0], arm2.fb["y"] - world["bottle"][1]) < 5
            world["held"] = g == closed and near or (world["held"] and g != opened)
        arm2.gripper = gripper
        assert grab(arm2, so, cfg, dict(cal["grip"], x=x, y=y), px=px, log=lambda *_: None) and world["held"]
        # the grab misses (bottle stays on the table) -> 3 attempts, then False
        world.update(bottle=(240.0, 200.0), held=False)
        arm2.gripper = lambda g: None
        assert not grab(arm2, so, cfg, dict(cal["grip"], x=200.0, y=100.0), px=(2 * 240.0 - 57.6, 200.0),
                        log=lambda *_: None)
        assert not in_reach(cfg, 50, 0) and in_reach(cfg, 200, 100)
        # lying bottle: a synthetic outline (body 60 px wide, neck 20 px) along the image x axis, neck on the right
        import cv2
        import numpy as np

        img = np.full((300, 500), 230, np.uint8)
        cv2.rectangle(img, (60, 120), (340, 180), 90, 3)   # body
        cv2.rectangle(img, (340, 140), (430, 160), 90, 3)  # neck
        found_axis = bottle_axis(img, [40, 90, 460, 210], neck_frac=0.38)
        assert found_axis and found_axis[0] > 330 and abs(found_axis[1] - 150) < 6, found_axis
        assert min(abs(found_axis[2]), abs(abs(found_axis[2]) - math.pi)) < 0.05, found_axis
        # coloured cap: blue 30x30 px cap at the right end of a grey bottle (box cuts the neck off) -> neck 35 px left
        img = np.full((300, 500, 3), 230, np.uint8)
        cv2.rectangle(img, (60, 120), (330, 180), (200, 200, 200), -1)
        cv2.rectangle(img, (400, 135), (430, 165), (200, 60, 0), -1)
        u, v, a = cap_neck(img, [60, 115, 340, 185])
        assert abs(u - (414.5 - 35)) < 4 and abs(v - 149.5) < 4 and abs(a) < 0.1, (u, v, a)
        assert cap_neck(np.full((300, 500, 3), 230, np.uint8), [60, 115, 340, 185]) is None
        # jaws across a bottle lying along the RoArm x axis straight ahead: roll pi/2; along y: roll 0 (offset 0)
        assert abs(abs(jaw_roll(dict(DEFAULTS), 250, 0, 0.0)) - math.pi / 2) < 1e-9
        assert abs(jaw_roll(dict(DEFAULTS), 250, 0, math.pi / 2)) < 1e-9
        # taught sequence: step 3 fails 3x -> stopped, progress says step 3; the next play continues there
        import tempfile

        prog = os.path.join(tempfile.mkdtemp(), "progress.json")
        pcfg = dict(cfg, sequence=None, poses={n: {"b": i, "s": 0, "e": 1, "t": 0, "r": 0, "g": 3}
                                               for i, n in enumerate(("rest", "aim", "grip", "lift"))})
        reached, fail = [], {"grip": 3}
        orig_go_pose = globals()["go_pose"]

        def fake_go_pose(a, c, name, timeout=25.0):
            if fail.get(name):
                fail[name] -= 1
                raise TimeoutError(name)
            reached.append(name)
        globals()["go_pose"] = fake_go_pose
        try:
            try:
                play(arm2, pcfg, log=lambda *_: None, progress_file=prog)
                raise AssertionError("step grip should have failed")
            except TimeoutError:
                pass
            assert reached == ["rest", "aim"] and json.load(open(prog))["next"] == 2, reached
            assert play(arm2, pcfg, log=lambda *_: None, progress_file=prog)
            assert reached == ["rest", "aim", "grip", "lift", "rest"], reached  # ends where it started
            fail["aim"] = 1  # one timeout is retried on the spot
            assert play(arm2, pcfg, log=lambda *_: None, progress_file=prog)
            assert reached[-5:] == ["rest", "aim", "grip", "lift", "rest"], reached
        finally:
            globals()["go_pose"] = orig_go_pose
        # rest pose: lift first (world up), then the saved joint angles
        sent = []
        arm2.send = sent.append
        arm2.fb.update(x=200.0, y=0.0, z=50.0, tit=1.57, r=0.0, g=3.0)
        go_rest(arm2, dict(cfg, rest={"b": 0.79, "s": 0.0, "e": 2.06, "t": 1.46, "r": -3.1, "g": 3.1}))
        assert sent[-1]["T"] == 102 and sent[-1]["elbow"] == 2.06 and arm2.fb["z"] == 50.0 + cfg["approach_mm"], sent

        # kinematics: RoArm feedback 2026-09-27 (b s e t) -> z 270 mm; ik(fk(q)) returns to the same elbow branch
        x, y, z, tit = fk(2.37, 0.26, 0.17, 1.56)
        assert abs(z - 270) < 2 and abs(math.hypot(x, y) - 301) < 2, (x, y, z)
        for q in ((2.37, 0.25, 0.64, 1.2), (2.37, 0.26, 0.17, 1.4), (0.3, -0.4, 1.9, 0.2)):
            x, y, z, tit = fk(*q)
            q2 = ik(x, y, z, tit, {"s": q[1], "e": q[2]})
            assert q2 and max(abs(a - b) for a, b in zip(q, q2)) < 1e-6, (q, q2)
        assert ik(900, 0, 0, 0, {"s": 0, "e": 1}) is None
        # wrist at its limit: this tilt is impossible lower down, command() takes the nearest possible one
        sent = []
        mock = RoArm("mock", mock=True)
        mock.fb.update(b=2.37, s=0.25, e=0.64, t=1.565)
        mock.send = sent.append
        globals()["IK_ON_PI"] = True
        try:
            assert ik(-214.8, 208.3, 179.4, 0.87, {"s": 0.25, "e": 0.64}) is None
            command(mock, -214.8, 208.3, 179.4, 0.87, -1.64, 3.0, 0.1)
            assert sent[-1]["T"] == 102 and abs(sent[-1]["wrist"]) <= math.pi / 2, sent
        finally:
            globals()["IK_ON_PI"] = False

        # RoArm upside down: the floor "below" it is larger z (here z=+150); it steps in +z
        globals()["UP"] = -1
        try:
            arm3 = RoArm("mock", mock=True)
            arm3.fb.update(x=230.0, y=240.0, z=50.0, tit=-1.57, r=0.0, tS=-150, tE=150)

            def send3(c):
                if c.get("T") == 104:
                    arm3.fb.update(x=c["x"], y=c["y"], z=min(c["z"], 150.0), tS=-150 + (90 if c["z"] > 150 else 0))
            arm3.send = send3
            cfg3 = dict(cfg, cal_center=[230, 240], floor_pause_s=0, floor_start_z=50, cal_tilt=-1.57, upside_down=True)
            z3 = find_floor(arm3, cfg3, 230, 240, log=lambda *_: None)
            assert abs(z3 - 150) < 1 and arm3.fb["z"] < 150, (z3, arm3.fb["z"])
        finally:
            globals()["UP"] = 1

        # AprilTag on the gripper: 60x60 px tag at (300..360, 100..160), side 30 mm, centre 25 mm above the tip ->
        # tip 50 px below the tag centre; another tag id = None
        import cv2
        import numpy as np

        img = np.full((360, 640), 255, np.uint8)
        tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 60)
        img[100:160, 300:360] = tag
        jpg = cv2.imencode(".jpg", img)[1].tobytes()
        u, v = find_tag(jpg, 640, 360, DEFAULTS)
        assert abs(u - 329.5) < 2 and abs(v - (129.5 + 50)) < 3, (u, v)
        assert find_tag(jpg, 640, 360, dict(DEFAULTS, tag_id=1)) is None
        # flag 50 mm away from the base and 20 mm left; the RoArm faces +y (base turned 90 deg)
        tx, ty = tag_xy(dict(DEFAULTS, tag_offset_mm=[50, 20]), 0.0, 200.0)
        assert abs(tx + 20) < 1e-6 and abs(ty - 250) < 1e-6, (tx, ty)
    finally:
        globals()["save_config"] = orig_save
    print("test ok")


def main():
    if "--test" in sys.argv:
        return test()
    cfg = load_config()
    cmd = sys.argv[1:2] or [""]
    arm = RoArm(cfg["roarm_ip"])
    so = SO101(cfg["so101_url"])
    if cmd == ["info"]:
        cal = cfg["calibration"]
        print(f"RoArm {cfg['roarm_ip']}, SO-101 {cfg['so101_url']}, upside down: {cfg['upside_down']}")
        print("calibration:", f"{len(cal['pairs'])} points ({cal.get('source', '?')}), floor z={cal.get('floor_z', 0):.0f}"
              f" mm, grab z={cal['grip']['z']:.0f} mm" if cal else "- none -")
    elif cmd == ["save-rest"]:
        w = arm.where()
        cfg["rest"] = {k: round(w[k], 4) for k in ("b", "s", "e", "t", "r", "g", "x", "y", "z", "tit")}
        save_config(cfg)
        print("rest pose saved:", cfg["rest"])
    elif cmd == ["rest"]:
        go_rest(arm, cfg)
        print("RoArm at its rest pose")
    elif cmd == ["poses"]:
        for name, v in (cfg.get("poses") or {}).items():
            print(f"  {name:16s} b={v['b']:+.2f} s={v['s']:+.2f} e={v['e']:+.2f} t={v['t']:+.2f} r={v['r']:+.2f} "
                  f"g={v['g']:.2f} | x={v['x']:.0f} y={v['y']:.0f} z={v['z']:.0f}")
        print("" if cfg.get("poses") else "no poses saved")
    elif cmd == ["save-pose"] and sys.argv[2:3]:
        w = arm.where()
        cfg.setdefault("poses", {})[sys.argv[2]] = {k: round(w[k], 4)
                                                    for k in ("b", "s", "e", "t", "r", "g", "x", "y", "z", "tit")}
        save_config(cfg)
        print(f"pose '{sys.argv[2]}' saved")
    elif cmd == ["play"]:
        play(arm, cfg, restart="--restart" in sys.argv)
    elif cmd == ["goto"] and sys.argv[2:3] and sys.argv[2] in (cfg.get("poses") or {}):
        go_pose(arm, cfg, sys.argv[2])
        print(f"RoArm at pose '{sys.argv[2]}'")
    elif cmd == ["calibrate"] and sys.argv[2:3] == ["tag"]:  # like brev, but the tip from the AprilTag on the gripper
        w = arm.where()  # RoArm placed so the camera sees the tag: grid centre, tilt and roll from here
        if not any(w.get(k) for k in ("torswitchB", "torswitchS", "torswitchE")):  # motors off (moved by hand):
            # servo goals = measured angles, then torque on - the arm stays where it is (like the RoArm panel)
            hold_here(arm, w)
            arm.send({"T": 210, "cmd": 1})
            time.sleep(1.0)
            w = arm.where()
        cfg.update(cal_center=[w["x"], w["y"]], cal_tilt=w["tit"], cal_roll=w.get("r", 0.0), floor_start_z=w["z"])
        print(f"centre x={w['x']:.0f} y={w['y']:.0f} mm, tilt {w['tit']:.2f} rad")
        calibrate(arm, so, cfg, locate=find_tag, source="tag")
    elif cmd == ["calibrate"] and sys.argv[2:3] == ["brev"]:  # no bottle, no person: VLM on Brev
        calibrate(arm, so, cfg)
    elif cmd in (["where"], ["aim"], ["grab"]):
        if not cfg["calibration"]:
            raise SystemExit("no calibration: python roarm_pick.py calibrate tag")
        so.tracking(False)
        so.look()
        wait_for_look(so)
        target = target_on_table(so, cfg, timeout=15)
        if target is None:
            raise SystemExit("no bottle seen on the table")
        (x, y), (u, v), axis = target
        pick = dict(cfg["calibration"]["grip"], x=x, y=y)
        if axis is not None:
            pick["r"] = jaw_roll(cfg, x, y, axis)
        print(f"bottle{' neck' if axis is not None else ''}: ({u:.0f}, {v:.0f}) px -> RoArm x={x:.0f} y={y:.0f} mm, "
              f"{math.hypot(x, y):.0f} mm from the base"
              + (f", axis {math.degrees(axis):.0f} deg, gripper roll {pick['r']:.2f} rad" if axis is not None else "")
              + ("" if in_reach(cfg, x, y) else "  OUT OF REACH"))
        if cmd == ["grab"] and in_reach(cfg, x, y):
            ok = grab(arm, so, cfg, pick, px=(u, v))
            print("HOLDING it - the RoArm waits above the table" if ok
                  else "missed (3 attempts) - change grip_height_mm or repeat the calibration")
        elif cmd == ["aim"] and in_reach(cfg, x, y):
            go_to(arm, pick, cfg["grip_open"], dz=cfg["approach_mm"], spd=cfg["speed"])
            print(f"RoArm is {cfg['approach_mm']} mm above the grab point - the gripper should be above the bottle")
    else:
        print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (RuntimeError, TimeoutError, Overheat, requests.RequestException) as e:
        sys.exit(f"ERROR: {e}")
