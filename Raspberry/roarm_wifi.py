"""RoArm-M3 over WiFi: HTTP GET http://<ip>/js?json=...
Position: {"T":105} returns {"T":1051, x y z (mm), tit, b s e t r g (rad), ...} in the HTTP reply
(seen on this arm's firmware; the vendor source's ws://<ip>/ws feed did not connect, so it is not used).
Gripper feedback g reads 0 on some firmware whatever the fingers do.

Never send T:0: over USB it froze this arm's firmware ~10 s and the arm could drop.
T:104 with spd 0 is silently ignored by the firmware, so spd must be > 0.

    python roarm_wifi.py where           measured pose (jog the arm with its own web page http://<ip>/)
    python roarm_wifi.py send '{"T":105}'
    python roarm_wifi.py wiggle          up 40 mm, back, gripper open/close: proves the link works
    MOCK=1 python roarm_wifi.py          self-check without hardware
"""
import json
import os
import sys
import time

import requests


class RoArm:
    def __init__(self, ip, mock=False):
        self.ip, self.mock = ip, mock
        self.fb = {"x": 200.0, "y": 0.0, "z": 200.0}  # mock pose

    def _get(self, cmd):
        try:
            r = requests.get(f"http://{self.ip}/js", params={"json": json.dumps(cmd, separators=(",", ":"))},
                             timeout=2)  # a reply takes 0.4-0.6 s over the phone hotspot
            r.raise_for_status()
        except requests.RequestException as e:
            raise RuntimeError(f"arm {self.ip} not answering ({type(e).__name__}): rebooting, busy or off WiFi?") from None
        if "error" in r.text:  # {"error":"Queue full"}
            raise RuntimeError(f"roarm: {r.text}")
        return r.text

    def send(self, cmd):
        if cmd.get("T") == 0:
            raise ValueError("T:0 freezes this arm ~10 s; stop by commanding the measured pose")
        if self.mock:
            print("  [roarm]", cmd)
            return
        self._get(cmd)

    def where(self):
        """Measured pose: dict with x y z (mm), tit, b s e t r g (rad). Raises if the arm gives none."""
        if self.mock:
            return dict(self.fb)
        try:
            d = json.loads(self._get({"T": 105}))
        except ValueError:
            d = {}
        if d.get("T") != 1051 or d.get("x") is None:  # nulls: servos unpowered / feedback invalid
            raise RuntimeError(f"arm at {self.ip} gave no valid position (servo power off?): {d}")
        return d

    def goto(self, x, y, z, t, g, spd=0.25, tol=15.0, timeout=12.0):
        """Tool point to x y z (mm, base frame: x forward, y left, z up), wrist pitch t and gripper g (rad).
        Blocks until the measured xyz is within tol mm."""
        if spd <= 0:
            raise ValueError("T:104 ignores spd 0")
        self.send({"T": 104, "x": round(x, 1), "y": round(y, 1), "z": round(z, 1),
                   "t": t, "r": 0, "g": g, "spd": spd})
        if self.mock:
            self.fb.update(x=x, y=y, z=z, g=g)
            return
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            time.sleep(0.3)
            p = self.where()
            if max(abs(p["x"] - x), abs(p["y"] - y), abs(p["z"] - z)) < tol:
                return
        raise TimeoutError(f"roarm did not reach {x:.0f},{y:.0f},{z:.0f} (at {self.where()})")

    def gripper(self, g):
        self.send({"T": 106, "cmd": g, "spd": 0, "acc": 0})
        time.sleep(0.8)  # no feedback on the fingers: give them time to close


if __name__ == "__main__":
    cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")))
    arm = RoArm(cfg["roarm_ip"], mock=os.environ.get("MOCK") == "1")
    if sys.argv[1:2] == ["where"]:
        print(arm.where())
    elif sys.argv[1:2] == ["send"]:
        arm.send(json.loads(sys.argv[2]))
    elif sys.argv[1:2] == ["wiggle"]:  # first-contact test: small moves around wherever it stands now
        p = arm.where()
        print("at", {k: round(p[k], 2) for k in ("x", "y", "z", "tit", "g")})
        g = p["g"] if p["g"] > 1 else cfg["grip_open"]  # g reads 0 on some firmware: don't command that
        arm.goto(p["x"], p["y"], p["z"] + 40, p["tit"], g, spd=0.2)
        arm.goto(p["x"], p["y"], p["z"], p["tit"], g, spd=0.2)
        arm.gripper(cfg["grip_closed"])
        arm.gripper(cfg["grip_open"])
        print("wiggle ok")
    else:
        arm = RoArm("mock", mock=True)
        arm.goto(250, 50, 0, 1.57, 1.57)
        assert arm.where()["x"] == 250
        try:
            arm.send({"T": 0})
            raise SystemExit("T:0 must be refused")
        except ValueError:
            pass
        print("mock ok")
