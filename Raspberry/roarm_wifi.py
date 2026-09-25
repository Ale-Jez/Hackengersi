"""RoArm-M3 over WiFi. Commands: HTTP GET http://<ip>/js?json=...  (the reply is only {"ok":1}).
Feedback: the ws://<ip>/ws websocket pushes {"T":-15, x y z (mm), b s e t r g (rad)} about once a
second, measured from the servos (vendor/RoArm-M3_example/http_server.h + .ino loop).

Never send T:0: over USB it froze this arm's firmware ~10 s and the arm could drop.
T:104 with spd 0 is silently ignored by the firmware, so spd must be > 0.

    python roarm_wifi.py where           measured pose (jog the arm with its own web page http://<ip>/)
    python roarm_wifi.py send '{"T":105}'
    MOCK=1 python roarm_wifi.py          self-check without hardware
"""
import json
import os
import sys
import threading
import time

import requests


class RoArm:
    def __init__(self, ip, mock=False):
        self.ip, self.mock = ip, mock
        self.fb, self._fb_t = {"x": 200.0, "y": 0.0, "z": 200.0}, 0.0
        if mock:
            return
        import websocket  # websocket-client

        ws = websocket.WebSocketApp(f"ws://{ip}/ws", on_message=self._on_msg)
        threading.Thread(target=ws.run_forever, kwargs={"reconnect": 2}, daemon=True).start()

    def _on_msg(self, _ws, msg):
        try:
            d = json.loads(msg)
        except ValueError:
            return
        if d.get("T") == -15:
            self.fb, self._fb_t = d, time.monotonic()

    def send(self, cmd):
        if cmd.get("T") == 0:
            raise ValueError("T:0 freezes this arm ~10 s; stop by commanding the measured pose")
        if self.mock:
            print("  [roarm]", cmd)
            return
        r = requests.get(f"http://{self.ip}/js", params={"json": json.dumps(cmd, separators=(",", ":"))},
                         timeout=3)
        r.raise_for_status()
        if "error" in r.text:  # {"error":"Queue full"}
            raise RuntimeError(f"roarm: {r.text}")

    def where(self, max_age=3.0):
        if not self.mock and time.monotonic() - self._fb_t > max_age:
            raise RuntimeError(f"no feedback from ws://{self.ip}/ws for {max_age}s (arm off / wrong IP?)")
        return dict(self.fb)

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
            time.sleep(0.5)  # feedback is ~1 Hz, faster polling buys nothing
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
        time.sleep(2.5)
        print(arm.where())
    elif sys.argv[1:2] == ["send"]:
        arm.send(json.loads(sys.argv[2]))
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
