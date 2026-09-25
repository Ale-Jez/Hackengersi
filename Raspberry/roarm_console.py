"""Keyboard jog for the RoArm-M3 over WiFi (Pi terminal or Windows console).

    python roarm_console.py             real arm, IP from config.json
    python roarm_console.py --selftest  key logic against a mock arm

 W/S  x forward / back (mm)        A/D  y left / right       R/F  z up / down
 T/G  wrist tilt up / down         Y/H  roll                 Z/X  gripper open / close
 [ ]  smaller / bigger step        - =  slower / faster      P    print measured pose
 SPACE stop: hold the measured pose, sync the target        Q or Ctrl+C  quit

Hold a key to repeat. At most one step per axis per 0.15 s goes out, so the arm stops within about a
step of you releasing the key. Feedback only arrives ~1 Hz, so P and SPACE can be up to a second stale.
Start with a small step and clear space: the arm moves as soon as you press a key.
"""
import json
import math
import os
import re
import sys
import time

import requests

from roarm_wifi import RoArm

TICK = 0.15  # s between sends
JOG = {"w": ("x", 1), "s": ("x", -1), "a": ("y", 1), "d": ("y", -1), "r": ("z", 1), "f": ("z", -1),
       "t": ("t", 1), "g": ("t", -1), "y": ("r", 1), "h": ("r", -1), "z": ("g", -1), "x": ("g", 1)}
Z_LIMITS = (-150.0, 450.0)  # mm; the firmware's IK limits the rest, lower it if the table is higher
ANGLE_LIMITS = {"t": (-3.14, 3.14), "r": (-3.14, 3.14), "g": (1.0, 3.5)}  # docs: 1.57 opens, 3.14 grabs
SCALE = {"x": 1, "y": 1, "z": 1, "t": 0.005, "r": 0.005, "g": 0.02}  # per mm of step: 10 mm = 0.05 rad tilt/roll, 0.2 rad grip
ESCAPES = re.compile(r"\x1b(\[[0-9;]*[A-Za-z~]|O.)")  # arrow / function keys, so their '[' isn't a step key

try:
    import select
    import termios
    import tty

    def pending():
        out = ""
        while select.select([sys.stdin], [], [], 0)[0]:
            out += os.read(sys.stdin.fileno(), 64).decode(errors="ignore")
        return out

    class raw_keys:
        def __enter__(self):
            self.old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)  # keys arrive unbuffered, Ctrl+C still works

        def __exit__(self, *_):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old)
except ImportError:  # Windows
    import msvcrt

    def pending():
        out = ""
        while msvcrt.kbhit():
            c = msvcrt.getwch()
            out += msvcrt.getwch() and "" if c in ("\x00", "\xe0") else c  # swallow the arrow-key second code
        return out

    class raw_keys:
        def __enter__(self): pass
        def __exit__(self, *_): pass


def clamp(v, lo, hi):
    return min(max(v, lo), hi)


def clamp_xy(x, y, lo, hi):
    """Keep the tool point between lo and hi mm from the base axis, same direction."""
    r = math.hypot(x, y)
    if r == 0:
        return lo, 0.0
    k = clamp(r, lo, hi) / r
    return x * k, y * k


def main(arm, cfg, pending=pending):
    lo, hi = cfg["reach_mm"]
    p = arm.where()  # raises if there is no feedback: better than jogging from a made-up pose
    g0 = p.get("g", 0)
    tgt = {"x": p["x"], "y": p["y"], "z": p["z"], "t": p.get("tit", cfg["pick_t"]), "r": p.get("r", 0.0),
           "g": g0 if ANGLE_LIMITS["g"][0] <= g0 <= ANGLE_LIMITS["g"][1] else cfg["grip_open"]}  # g reads 0 on some firmware
    step, spd = 10.0, cfg["roarm_spd"]
    delta, last, quit_ = {}, 0.0, False
    print(__doc__)
    while True:
        for c in ESCAPES.sub("", pending()).lower():
            if c in ("q", "\x03"):
                quit_ = True
            elif c in JOG:
                j, sign = JOG[c]
                delta[j] = sign  # held-key repeats collapse into one step per tick
            elif c == "[":
                step = max(step / 2, 1.0)
            elif c == "]":
                step = min(step * 2, 40.0)
            elif c == "-":
                spd = max(round(spd - 0.05, 2), 0.05)
            elif c == "=":
                spd = min(round(spd + 0.05, 2), 1.0)
            elif c in ("p", " "):
                try:
                    m = arm.where()
                    print("\nmeasured", {k: round(v, 2) for k, v in m.items() if k in ("x", "y", "z", "tit", "r", "g")})
                    if c == " ":
                        delta = {}
                        tgt.update(x=m["x"], y=m["y"], z=m["z"], t=m.get("tit", tgt["t"]))
                        arm.send({"T": 104, **{k: round(v, 3) for k, v in tgt.items()}, "spd": spd})
                except (RuntimeError, requests.RequestException) as e:
                    print("\n", e)
        now = time.monotonic()
        if delta and (quit_ or now - last >= TICK):
            for j, sign in delta.items():
                tgt[j] += sign * step * SCALE[j]
            tgt["x"], tgt["y"] = clamp_xy(tgt["x"], tgt["y"], lo, hi)
            tgt["z"] = clamp(tgt["z"], *Z_LIMITS)
            for j, (a, b) in ANGLE_LIMITS.items():
                tgt[j] = clamp(tgt[j], a, b)
            delta, last = {}, now
            try:
                arm.send({"T": 104, **{k: round(v, 3) for k, v in tgt.items()}, "spd": spd})
                print("\r" + "  ".join(f"{k}={v:+.2f}" for k, v in tgt.items()) + f"  step={step:g} spd={spd}   ",
                      end="", flush=True)
            except (RuntimeError, requests.RequestException) as e:  # keep the session, drop this step
                print("\n", e)
        if quit_:
            print()
            return
        time.sleep(0.01)


def selftest():
    sent = []
    arm = RoArm("mock", mock=True)  # mock feedback: x=200 y=0 z=200
    arm.send = sent.append
    cfg = {"reach_mm": [120, 450], "pick_t": 1.57, "grip_open": 1.57, "roarm_spd": 0.25}
    groups = ["wwww" + "\x1b[A" + "zz", "d", "q"]  # repeats collapse; the arrow must not act as '['
    main(arm, cfg, lambda: groups.pop(0) if groups else "")
    assert len(sent) == 2, sent
    assert sent[0]["x"] == 210 and abs(sent[0]["g"] - 1.37) < 1e-9, sent[0]  # one step: 10 mm, grip 1.57 - 0.2
    assert sent[1]["y"] == -10 and sent[1]["x"] == 210, sent[1]  # flushed on quit, step size unchanged
    assert clamp_xy(500, 0, 120, 450) == (450, 0) and clamp_xy(0, 0, 120, 450) == (120, 0)
    print("selftest ok")


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        selftest()
        sys.exit()
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")))
    arm = RoArm(cfg["roarm_ip"])
    time.sleep(2.5)  # first websocket feedback
    with raw_keys():
        try:
            main(arm, cfg)
        except KeyboardInterrupt:
            print("\nbye")
