"""Driving trash can: follow the route in config.json; at every "sort" stop the SO-101 camera looks into
the can, the Brev LLM says what is where, and the RoArm moves each item into one of three bins.

    python main.py run [--loop]   drive the route (once, or forever)
    python main.py sort           one sort cycle where the robot stands
    python main.py calibrate      fit camera pixels -> RoArm x,y (AprilTag taped on the RoArm gripper)
    python main.py selftest       logic check with mock arms, camera and LLM

Route steps:  {"tag": 3, "stop_px": 150}  drive at AprilTag 3 until it looks 150 px wide
              {"drive": [left, right, seconds]}  blind timed move, wheel speeds -1..1
              {"sort": true}
"""
import json
import math
import os
import sys
import time

import cv2

from roarm_wifi import RoArm
from so101 import CONFIG, SO101
from vision import Camera, ask_llm, draw, find_tags, fit_homography, to_arm

MOCK = os.environ.get("MOCK") == "1"


def set_wheels(left, right):
    """Drive base, -1..1 per side (+ = forward)."""
    if MOCK:
        return
    # ponytail: description doesn't name the drive hardware yet; send left/right to it here
    raise NotImplementedError("set_wheels(): wire up the drive base (motor HAT / ESP32 / CubeBot)")


def approach(cfg, cam, tag_id, stop_px, timeout=60):
    """Steer at the tag until it looks stop_px wide; spin on the spot while it is not in view."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        img = cam.frame(settle=1)
        tag = find_tags(img).get(tag_id)
        if tag is None:
            set_wheels(-cfg["search_speed"], cfg["search_speed"])
            continue
        cx, _, side = tag
        if side >= stop_px:
            set_wheels(0, 0)
            return
        err = (cx - img.shape[1] / 2) / (img.shape[1] / 2)  # -1 tag at left edge .. +1 right edge
        v, k = cfg["drive_speed"], cfg["steer_gain"]
        set_wheels(v + k * err, v - k * err)
    set_wheels(0, 0)
    raise TimeoutError(f"tag {tag_id} not reached in {timeout}s")


def park(roarm, cfg):
    roarm.goto(*cfg["park"], cfg["pick_t"], cfg["grip_open"], cfg["roarm_spd"])


def pick(roarm, cfg, x, y, bin_name):
    lo, hi = cfg["reach_mm"]
    if not lo <= math.hypot(x, y) <= hi:
        print(f"  skip: {x:.0f},{y:.0f} mm is outside the reach {lo}-{hi} mm")
        return False
    t, op, cl, spd = cfg["pick_t"], cfg["grip_open"], cfg["grip_closed"], cfg["roarm_spd"]
    z, up = cfg["pick_z"], cfg["pick_z"] + cfg["approach_dz"]
    roarm.goto(x, y, up, t, op, spd)
    roarm.goto(x, y, z, t, op, spd)
    roarm.gripper(cl)
    roarm.goto(x, y, up, t, cl, spd)
    roarm.goto(*cfg["bins"][bin_name], t, cl, spd)
    roarm.gripper(op)
    return True


def sort_can(cfg, roarm, so, cam, ask=ask_llm):
    if not cfg["homography"]:
        raise SystemExit("no homography: run `python main.py calibrate` first")
    for _ in range(cfg["max_rounds"]):  # re-look after each round: items shift when others are pulled out
        park(roarm, cfg)  # out of the picture
        so.go("look")
        img = cam.frame()
        so.go("stow")
        items = ask(img, cfg["brev_url"], cfg["brev_model"])
        cv2.imwrite("last_look.jpg", draw(img, items))
        print(f"LLM sees {len(items)} item(s) (last_look.jpg)")
        picked = 0
        for it in items:
            x, y = to_arm(cfg["homography"], it["x"], it["y"])
            print(f"  {it['label']} -> {it['bin']} at {x:.0f},{y:.0f} mm")
            picked += pick(roarm, cfg, x, y, it["bin"])
        if not picked:  # empty, or nothing reachable: don't loop on it
            break
    park(roarm, cfg)


def calibrate(cfg, roarm, so, cam):
    """Tape AprilTag 36h11 id cfg["calib_tag"] on the RoArm gripper where the look camera sees it,
    as close to the fingertips as possible. The RoArm visits cfg["calib_points"] at pick_z, the camera
    finds the tag, and the pixel -> x,y homography is saved. Pick a look pose high enough to stay clear."""
    so.go("look")
    px, xy = [], []
    for x, y in cfg["calib_points"]:
        roarm.goto(x, y, cfg["pick_z"], cfg["pick_t"], cfg["grip_closed"], cfg["roarm_spd"])
        time.sleep(1)
        tag = find_tags(cam.frame()).get(cfg["calib_tag"])
        print(f"  {x},{y} mm -> {'tag not seen' if tag is None else f'{tag[0]:.0f},{tag[1]:.0f} px'}")
        if tag is not None:
            px.append(tag[:2])
            xy.append((x, y))
    park(roarm, cfg)
    so.go("stow")
    if len(px) < 4:
        raise SystemExit(f"only {len(px)} points seen, need 4+: move the tag or the look pose")
    cfg["homography"] = fit_homography(px, xy)
    json.dump(cfg, open(CONFIG, "w"), indent=2)
    print("homography saved to config.json")


def run(cfg, roarm, so, cam, loop=False):
    try:
        while True:
            for step in cfg["route"]:
                print("step", step)
                if "tag" in step:
                    so.go("drive")
                    approach(cfg, cam, step["tag"], step["stop_px"])
                elif "drive" in step:
                    left, right, secs = step["drive"]
                    set_wheels(left, right)
                    time.sleep(secs)
                    set_wheels(0, 0)
                elif step.get("sort"):
                    sort_can(cfg, roarm, so, cam)
            if not loop:
                return
    finally:
        set_wheels(0, 0)


def selftest():
    global MOCK
    MOCK = True
    cfg = json.load(open(CONFIG))
    cfg["homography"] = fit_homography([(0, 0), (100, 0), (100, 100), (0, 100)],
                                       [(300, 100), (300, -100), (200, -100), (200, 100)])
    roarm, so = RoArm("mock", mock=True), SO101(mock=True)
    for name in ("look", "stow", "drive"):
        cfg["so101_poses"].setdefault(name, {"pan": 0})
    so.go = lambda name: so.move(cfg["so101_poses"][name])
    answers = [[{"label": "can", "bin": "cans_bottles", "x": 50, "y": 50},
                {"label": "far away", "bin": "paper", "x": 5000, "y": 50}], []]
    sort_can(cfg, roarm, so, Camera(None), ask=lambda *_: answers.pop(0))
    assert answers == [], "second look must happen, then stop on the empty answer"
    assert roarm.where()["x"] == cfg["park"][0]

    # steering: tag right of centre -> left wheel faster; big tag -> stop
    tag = cv2.cvtColor(cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 5, 100), cv2.COLOR_GRAY2BGR)
    small, big = Camera(None).frame(), Camera(None).frame()
    small[300:400, 1000:1100] = 255  # white quiet zone
    small[310:390, 1010:1090] = cv2.resize(tag, (80, 80))
    big[100:600, 400:900] = cv2.resize(tag, (500, 500))
    frames, calls = [small, big], []

    class FakeCam:
        def frame(self, settle=0):
            return frames.pop(0)

    global set_wheels
    set_wheels = lambda l, r: calls.append((l, r))
    approach(cfg, FakeCam(), 5, stop_px=200)
    assert calls[0][0] > calls[0][1] and calls[-1] == (0, 0), calls
    print("selftest ok")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "selftest":
        selftest()
        sys.exit()
    if cmd not in ("run", "sort", "calibrate"):
        sys.exit(__doc__)
    cfg = json.load(open(CONFIG))
    roarm = RoArm(cfg["roarm_ip"], mock=MOCK)
    so = SO101(cfg["so101_port"], mock=MOCK)
    cam = Camera(None if MOCK else cfg["camera"])
    if cmd == "run":
        run(cfg, roarm, so, cam, loop="--loop" in sys.argv)
    elif cmd == "sort":
        sort_can(cfg, roarm, so, cam)
    else:
        calibrate(cfg, roarm, so, cam)
