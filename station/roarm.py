"""RoArm-M3 over USB serial (JSON lines, 115200). KAUCJO_MOCK=1 (or mock=True) = no hardware.

A pose is a dict of radians keyed like the T:1051 feedback: b s e t r g.
arm.move(arm.pose()) is a no-op, so taught poses round-trip.

Command formats come from json_cmd.h in RoArm-M3/deployments/.../src, not the docs.
Env overrides: ROARM_PORT (e.g. COM8), ROARM_USB_SERIAL.
"""
import collections
import json
import os
import threading
import time

from serial import Serial
from serial.tools import list_ports

USB_SERIAL = os.environ.get("ROARM_USB_SERIAL", "58EEF970F200F011B172C7295C2A50C9")
KEYS = {"b": "base", "s": "shoulder", "e": "elbow", "t": "wrist", "r": "roll", "g": "hand"}
ARM_JOINTS = "bsetr"  # what move() waits on; the gripper stalls on the bottle, so it's excluded
GRIP_OPEN = 1.57  # assumed gripper angle when unknown: the measured g reads 0 whatever the gripper does


def unread(d):
    """True for the stock firmware's placeholder frame: a servo that doesn't answer keeps pos 0,
    which shows up as angles no joint can reach (unpowered servos, wrong/missing servo ID).
    base/roll/gripper read plausible values at pos 0, so a dead one of those isn't caught."""
    return abs(d["s"]) > 3 or abs(d["t"]) > 3 or d["e"] < -0.5


class RoArm:
    def __init__(self, mock=None, port=None):
        self.mock = os.environ.get("KAUCJO_MOCK") == "1" if mock is None else mock
        self.replies = collections.deque(maxlen=50)  # non-feedback lines, e.g. the T:302 MAC
        self._pose = {k: 0.0 for k in KEYS}
        self._grip = GRIP_OPEN  # last commanded gripper rad, see move()
        self._seen = 0.0
        self._bad = 0  # placeholder frames rejected by unread()
        if self.mock:
            return
        port = port or os.environ.get("ROARM_PORT") or self._find_port()
        self.ser = Serial(None, 115200, timeout=0.1)
        self.ser.dtr = self.ser.rts = False  # keep the ESP32 out of reset/download mode
        self.ser.port = port
        self.ser.open()
        threading.Thread(target=self._read, daemon=True).start()
        deadline = time.monotonic() + 25  # covers a ~20 s boot after a reset or brown-out
        while not self._seen:
            if time.monotonic() > deadline:
                if self._bad:
                    raise TimeoutError(
                        f"{port}: the ESP32 answers but the servos don't ({self._bad} placeholder "
                        "frames). Servo power supply off/unplugged, servo bus cable loose, or a "
                        "replaced servo still has the wrong ID (expected 11-17)?")
                raise TimeoutError(
                    f"{port}: no T:1051 feedback. Wrong Type-C port (use the one labelled USB, "
                    "not LIDAR), arm unpowered, or still booting (~20 s)?")
            time.sleep(0.05)
        self._grip = self._pose["g"] or GRIP_OPEN

    @staticmethod
    def _find_port():
        ports = [p.device for p in list_ports.comports() if p.serial_number == USB_SERIAL]
        if len(ports) != 1:
            raise RuntimeError(f"expected one USB device with serial {USB_SERIAL}, got {ports}")
        return ports[0]

    def _read(self):
        while True:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except Exception:  # unplugged/closed; pose() goes stale and raises
                return
            if line.startswith('{"T":1051'):
                try:
                    d = json.loads(line)
                except ValueError:  # partial line right after connecting
                    continue
                if all(d.get(k) is not None for k in KEYS):  # 0.84-s1 sends nulls when invalid
                    if unread(d):  # stock firmware: zeros instead of nulls, so don't refresh _seen
                        self._bad += 1
                        continue
                    self._pose = {k: d[k] for k in KEYS}
                    self._seen = time.monotonic()
            elif line:
                self.replies.append(line)

    def _send(self, cmd):
        data = (json.dumps(cmd, separators=(",", ":")) + "\n").encode()
        if len(data) > 255:
            raise ValueError("command exceeds the firmware's 256-byte input buffer")
        if not self.mock:
            self.ser.write(data)

    def pose(self):
        """Measured joint angles in rad. Raises if feedback is older than 1 s."""
        if not self.mock and time.monotonic() - self._seen > 1:
            raise RuntimeError("stale feedback: arm unplugged, unpowered or resetting")
        return dict(self._pose)

    def move(self, pose, spd=300, acc=10, wait=True, tol=0.03, timeout=10):
        """Move to `pose` (partial ok: missing joints keep their current angle).
        spd is servo steps/s (~26 deg/s at 300); 0 would mean full speed, so it's refused.
        Gripper: uses the last commanded value unless 'g' is in `pose`, so moving never
        loosens a grip (the measured g is where the fingers stopped, not what was commanded)."""
        if spd <= 0:
            raise ValueError("spd=0 is full speed; pass a positive value")
        target = {**self.pose(), **pose}
        self._grip = target["g"] if "g" in pose else self._grip
        cmd = {KEYS[k]: round(target[k], 4) for k in ARM_JOINTS}
        self._send({"T": 102, **cmd, "hand": round(self._grip, 4), "spd": spd, "acc": acc})
        if self.mock:
            self._pose = {**target, "g": self._grip}
        if wait:
            self.wait(target, tol, timeout)
        return target

    def wait(self, target, tol=0.05, timeout=10):
        """Block until the arm joints are within tol rad of target (servos settle ~0.02 short)."""
        end = time.monotonic() + timeout
        while max(abs(self.pose()[k] - target[k]) for k in ARM_JOINTS if k in target) > tol:
            if time.monotonic() > end:
                raise TimeoutError(f"did not reach {target} within {timeout}s (at {self.pose()})")
            time.sleep(0.05)

    def gripper(self, rad, spd=0, acc=0):
        """Docs: 1.57 releases, 3.14 grabs, up to 4.0. Doesn't wait (it stalls on the bottle)."""
        self._grip = rad
        self._send({"T": 106, "cmd": round(rad, 4), "spd": spd, "acc": acc})
        if self.mock:
            self._pose["g"] = rad

    def torque(self, on):
        """T:210. Off first drives the arm to a fixed pose (blocking, several seconds) and then
        releases: clear the workspace and support the arm."""
        self._send({"T": 210, "cmd": int(on)})

    def stop(self):
        """Halt where it is: command the measured pose as the new goal. Deliberately NOT T:0,
        which on this arm freezes the firmware (no feedback, no commands) for ~10 s and appears to
        release torque (the arm can drop). Raises if feedback is stale."""
        self.move({}, wait=False)

    def resume(self):
        """Clears the firmware's stop flag, only needed if T:0 was sent by hand."""
        self._send({"T": 999})

    def close(self):
        if not self.mock:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


if __name__ == "__main__":
    import sys
    if sys.argv[1:] == ["live"]:
        with RoArm() as arm:
            print(arm.pose())
    elif sys.argv[1:] == ["console"]:  # raw JSON in, replies + pose out; T:1051 stream hidden
        with RoArm() as arm:
            print('JSON command, blank = show pose, q = quit. Stop: {"T":0}, resume: {"T":999}')
            try:
                while (line := input("> ").strip()) != "q":
                    arm.replies.clear()
                    try:
                        if line:
                            arm._send(json.loads(line))
                            time.sleep(0.7)
                        for reply in list(arm.replies):
                            print(reply)
                        print({k: round(v, 3) for k, v in arm.pose().items()})
                    except (ValueError, RuntimeError) as e:  # bad JSON, >255 bytes, stale feedback
                        print("error:", e)
            except (EOFError, KeyboardInterrupt):
                pass
    else:  # self-check, no hardware
        with RoArm(mock=True) as arm:
            arm.move({"b": 0.3})
            assert abs(arm.pose()["b"] - 0.3) < 1e-9
            arm.gripper(3.14)
            arm.move({"s": 0.5})  # must not touch the gripper
            assert arm.pose()["g"] == 3.14 and arm._grip == 3.14
            try:
                arm.move({"b": 0}, spd=0)
                raise SystemExit("spd=0 should be refused")
            except ValueError:
                pass
        assert unread({"b": 3.14, "s": -3.14, "e": -1.57, "t": -3.14, "r": 3.14, "g": 0})
        assert not unread({"b": 0, "s": -1.687, "e": 3.12, "t": 0.2, "r": 0, "g": 0})  # real rest
        assert RoArm(mock=True)._grip == GRIP_OPEN
        print("mock ok")
