"""RoArm-M3 over WiFi: HTTP GET http://<ip>/js?json=...
<ip> can also be "localhost:8766": roarm_usb.py serves the same API over the arm's USB cable (config.json default).
Position: {"T":105} returns {"T":1051, x y z (mm), tit, b s e t r g (rad), ...} in the HTTP reply
(seen on this arm's firmware; the vendor source's ws://<ip>/ws feed did not connect, so it is not used).
Gripper feedback g reads 0 on some firmware whatever the fingers do.

Never send T:0: over USB it froze this arm's firmware ~10 s and the arm could drop.
T:104 with spd 0 is silently ignored by the firmware, so spd must be > 0.

    python roarm_wifi.py where           measured pose (jog the arm with its own web page http://<ip>/)
    python roarm_wifi.py send '{"T":105}'
    python roarm_wifi.py wiggle          up 40 mm, back, gripper open/close: proves the link works
    python roarm_wifi.py temp            servo temperatures + loads every second (Ctrl+C ends)

Overheat guard: every motion command first checks the servo temperatures (T:1051 "temp", needs the
firmware with temps, Docs/RoArm-M3/.../firmware/0.84-temp) and raises Overheat at ROARM_TEMP_MAX (default 55 C).
    MOCK=1 python roarm_wifi.py          self-check without hardware
"""
import json
import os
import sys
import time

import requests

TEMP_MAX_C = float(os.environ.get("ROARM_TEMP_MAX", 55))  # stock firmware's own (disabled) limit is 50 C
# The firmware reports the supply as "v" (0.01 V). The shoulder pair are Feetech 7.4 V servos, but the team runs the
# arm at 12 V (decided 2026-09-26): the temperature limit is what protects them. 12.6 V = full 3S LiPo.
VOLT_RANGE = tuple(float(v) for v in os.environ.get("ROARM_VOLT_RANGE", "7.0,12.6").split(","))
SERVOS = ("base", "shoulder", "shoulder2", "elbow", "wrist", "roll", "gripper")  # IDs 11-17, order of "temp"


class Overheat(Exception):
    """A servo is at TEMP_MAX_C or hotter, or the supply is outside VOLT_RANGE: motion refused until it cools. Not a RuntimeError on purpose:
    callers treat RuntimeError from where() as 'arm busy' and retry."""


class RoArm:
    def __init__(self, ip, mock=False):
        self.ip, self.mock = ip, mock
        self.fb = {"x": 200.0, "y": 0.0, "z": 200.0, "temp": [30] * 7}  # mock pose
        self.temps, self.volts, self._t_temps, self._no_temps_warned = None, None, 0.0, False

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
        if cmd.get("T") != 105:
            self.check_temp()
        if self.mock:
            print("  [roarm]", cmd)
            return
        self._get(cmd)

    def check_temp(self, max_age=2.0):
        """Raise Overheat if any servo is at TEMP_MAX_C. Reads fresh feedback when the last one is older than max_age s."""
        if time.monotonic() - self._t_temps > max_age:
            try:
                self.where()
            except RuntimeError:
                pass  # busy moving or feedback invalid: judge by the last temps we have
        if self.temps is None:
            if not self._no_temps_warned:
                print(f"WARNING: arm {self.ip} reports no servo temperatures (stock firmware?) - overheat guard OFF")
                self._no_temps_warned = True
            return
        if self.volts is not None and not VOLT_RANGE[0] <= self.volts <= VOLT_RANGE[1]:
            raise Overheat(f"supply {self.volts:.2f} V outside {VOLT_RANGE[0]}-{VOLT_RANGE[1]} V: "
                           "check the power supply")
        hot = {k: v for k, v in self.temps.items() if v >= TEMP_MAX_C}
        if hot:
            raise Overheat(f"servo too hot {hot} (limit {TEMP_MAX_C:.0f} C): let it cool, all temps {self.temps}")

    def where(self):
        """Measured pose: dict with x y z (mm), tit, b s e t r g (rad). Raises if the arm gives none."""
        if self.mock:
            d = dict(self.fb)
        else:
            try:
                d = json.loads(self._get({"T": 105}))
            except ValueError:
                d = {}
        if d.get("temp"):
            self.temps, self._t_temps = dict(zip(SERVOS, d["temp"])), time.monotonic()
            self.volts = d["v"] / 100 if d.get("v") else None
        if self.mock:
            return d
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
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
    cfg = json.load(open(here if os.path.exists(here) else os.path.expanduser("~/Hackengersi/Raspberry/config.json")))
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
    elif sys.argv[1:2] == ["temp"]:
        while True:
            try:
                d = arm.where()
                t = arm.temps or {}
                print(time.strftime("%H:%M:%S"), f"{arm.volts}V", " ".join(f"{k}={v:.0f}C" for k, v in t.items()) or "no temps",
                      "| load", " ".join(f"{k}={d.get(k)}" for k in ("tB", "tS", "tE", "tT", "tR", "tG")),
                      " HOT!" if any(v >= TEMP_MAX_C for v in t.values()) else "", flush=True)
            except RuntimeError as e:
                print(time.strftime("%H:%M:%S"), e, flush=True)
            time.sleep(1)
    else:
        arm = RoArm("mock", mock=True)
        arm.goto(250, 50, 0, 1.57, 1.57)
        assert arm.where()["x"] == 250
        try:
            arm.send({"T": 0})
            raise SystemExit("T:0 must be refused")
        except ValueError:
            pass
        arm.fb.update(temp=[30] * 7, v=1350)  # supply above VOLT_RANGE must refuse motion
        arm._t_temps = 0.0
        try:
            arm.goto(250, 0, 0, 1.57, 1.57)
            raise SystemExit("13.5 V supply must refuse motion")
        except Overheat:
            pass
        arm.fb.update(v=1204)
        arm.fb["temp"] = [30, 30, TEMP_MAX_C + 1, 30, 30, 30, 30]
        arm._t_temps = 0.0
        try:
            arm.goto(250, 0, 0, 1.57, 1.57)
            raise SystemExit("hot servo must refuse motion")
        except Overheat:
            pass
        print("mock ok")
