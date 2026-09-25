"""Manual control of the SO-101 from the keyboard or an Xbox-style gamepad.

Run:   python teleop.py            (or the Run button in VS Code)
       python teleop.py COM5       (explicit port)

KEYBOARD (click into the terminal first; hold a key to keep moving)
  A / D     base rotate               W / S     shoulder forward / back
  I / K     elbow up / down           J / L     wrist bend
  U / O     wrist roll                Z / X     gripper open / close (small steps)
  SPACE     CLAMP (closes until it feels the box, then holds) / RELEASE
  1 2 3     speed: slow / medium / fast
  H         go home (pose "spoczynek" from poses_so101.json)
  ENTER     freeze - stop right here
  ESC       quit (arm keeps holding its position)

GAMEPAD (Xbox / XInput, plug in before starting)
  left stick    base (X) + shoulder (Y)       right stick   wrist roll (X) + elbow (Y)
  LB / RB       wrist bend                    A  CLAMP      B  RELEASE
  Y  home       X  freeze                     D-pad up/down  speed      BACK  quit

Emergency: pull the power plug of the servo adapter.
"""

import ctypes
import json
import os
import sys
import time

from so101 import SO101, SO101Error, TICKS_PER_DEG, find_port, _to_deg, _to_ticks

# ---------------- SETTINGS ----------------
SPEEDS = [10.0, 25.0, 50.0]      # deg/s for speed levels 1 / 2 / 3
KEY_STEP_TIME = 1 / 30           # a held key repeats ~30x per second
MAX_LEAD = 12.0                  # target may run at most this far ahead of the real arm (deg)
LIMIT_MARGIN = 3.0               # stay this far from the servo's own angle limits (deg)

CLAMP_SPEED = 30.0               # deg/s while closing onto the box
CLAMP_LOAD = 20.0                # % load that means "touching the box"
CLAMP_SQUEEZE = 4.0              # deg of extra squeeze after contact (more = firmer grip)
RELEASE_OPEN = 35.0              # deg the gripper opens on RELEASE

BOOST_P = True                   # P=32 on shoulder/elbow until power-off (they sag with 16)
HOME_POSE = "spoczynek"

# Flip a direction here if a key moves the wrong way (1 or -1).
DIRECTION = {"pan": 1, "lift": -1, "elbow": -1, "wflex": 1, "wroll": 1}
# ------------------------------------------

KEYS = {  # key -> (joint, sign)
    "a": ("pan", 1), "d": ("pan", -1),
    "w": ("lift", 1), "s": ("lift", -1),
    "i": ("elbow", 1), "k": ("elbow", -1),
    "j": ("wflex", 1), "l": ("wflex", -1),
    "u": ("wroll", 1), "o": ("wroll", -1),
}
MOVE_JOINTS = ["pan", "lift", "elbow", "wflex", "wroll"]


class Gamepad:
    """Minimal XInput reader (Xbox-compatible pads) - no extra packages."""

    class _State(ctypes.Structure):
        _fields_ = [("packet", ctypes.c_uint32), ("buttons", ctypes.c_uint16),
                    ("lt", ctypes.c_uint8), ("rt", ctypes.c_uint8),
                    ("lx", ctypes.c_int16), ("ly", ctypes.c_int16),
                    ("rx", ctypes.c_int16), ("ry", ctypes.c_int16)]

    BTN = {"up": 0x0001, "down": 0x0002, "back": 0x0020, "lb": 0x0100, "rb": 0x0200,
           "a": 0x1000, "b": 0x2000, "x": 0x4000, "y": 0x8000}

    def __init__(self):
        self.dll = None
        for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                self.dll = getattr(ctypes.windll, name)
                break
            except (OSError, AttributeError):
                continue
        self.prev = 0

    def read(self):
        """Returns (sticks dict in -1..1, set of newly pressed buttons, set of held buttons) or None."""
        if self.dll is None:
            return None
        st = self._State()
        if self.dll.XInputGetState(0, ctypes.byref(st)) != 0:
            return None

        def axis(v):
            v = max(-1.0, v / 32767)
            return 0.0 if abs(v) < 0.2 else (v - 0.2 * (1 if v > 0 else -1)) / 0.8

        held = {n for n, m in self.BTN.items() if st.buttons & m}
        pressed = {n for n, m in self.BTN.items() if st.buttons & m and not self.prev & m}
        self.prev = st.buttons
        return {"lx": axis(st.lx), "ly": axis(st.ly), "rx": axis(st.rx), "ry": axis(st.ry)}, pressed, held


def read_keys():
    import msvcrt

    keys = []
    while msvcrt.kbhit():
        ch = msvcrt.getwch()
        if ch in ("\x00", "\xe0"):  # arrow/function keys - ignore
            msvcrt.getwch()
            continue
        keys.append(ch.lower())
    return keys


def servo_limits(arm):
    lim = {}
    for j in MOVE_JOINTS + ["grip"]:
        sid = arm.ids[j]
        lo = arm.ph.read2ByteTxRx(arm.port, sid, 9)[0]
        hi = arm.ph.read2ByteTxRx(arm.port, sid, 11)[0]
        lim[j] = (_to_deg(lo) + LIMIT_MARGIN, _to_deg(hi) - LIMIT_MARGIN)
    return lim


def boost_p(arm):
    for j in ("lift", "elbow"):
        sid = arm.ids[j]
        if arm.ph.read1ByteTxRx(arm.port, sid, 55)[0] != 1:
            arm.ph.write1ByteTxRx(arm.port, sid, 55, 1)  # EEPROM locked -> change lasts until power-off
        arm.ph.write1ByteTxRx(arm.port, sid, 21, 32)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    arm = SO101(port)
    limits = servo_limits(arm)
    if BOOST_P:
        boost_p(arm)
    arm.torque(True)
    pad = Gamepad()
    pad_ok = pad.read() is not None

    here = arm.joints()
    target = dict(here)
    sent = dict(here)       # last goal sent to each servo
    level = 1
    clamp = "open"          # open | closing | holding
    last_status = 0.0
    poses_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses_so101.json")

    print("SO-101 manual control")
    print(f"Port {port}.  Gamepad: {'CONNECTED' if pad_ok else 'not found (keyboard only)'}")
    print("Keys: WASD IJKL UO move, SPACE clamp, Z/X grip, 1/2/3 speed, H home, ENTER freeze, ESC quit\n")

    def set_grip(deg):
        target["grip"] = min(max(deg, limits["grip"][0]), limits["grip"][1])

    try:
        prev_t = time.time()
        while True:
            now_t = time.time()
            dt = min(now_t - prev_t, 0.1)
            prev_t = now_t
            speed = SPEEDS[level]
            here = arm.joints()

            # ---- keyboard ----
            for key in read_keys():
                if key == "\x1b":
                    raise KeyboardInterrupt
                if key in KEYS:
                    joint, sign = KEYS[key]
                    target[joint] += sign * DIRECTION[joint] * speed * KEY_STEP_TIME
                elif key in "123":
                    level = int(key) - 1
                elif key == "z":
                    clamp = "open"
                    set_grip(target["grip"] + 3)
                elif key == "x":
                    clamp = "open"
                    set_grip(target["grip"] - 3)
                elif key == " ":
                    clamp = "closing" if clamp == "open" else "release"
                elif key == "\r":
                    target = dict(here)
                    clamp = "open" if clamp == "closing" else clamp
                elif key == "h":
                    try:
                        with open(poses_path, encoding="utf-8") as f:
                            home = json.load(f)[HOME_POSE]
                        print(f"\nGoing home ({HOME_POSE})...")
                        arm.move_slow(target={j: home[j] for j in MOVE_JOINTS}, verbose=False)
                        here = arm.joints()
                        target.update({j: here[j] for j in MOVE_JOINTS})
                    except (OSError, KeyError):
                        print(f"\nNo pose '{HOME_POSE}' saved - teach it in teach.py")

            # ---- gamepad ----
            state = pad.read()
            if state:
                sticks, pressed, held = state
                target["pan"] += -sticks["lx"] * DIRECTION["pan"] * speed * dt
                target["lift"] += sticks["ly"] * DIRECTION["lift"] * speed * dt
                target["elbow"] += sticks["ry"] * DIRECTION["elbow"] * speed * dt
                target["wroll"] += sticks["rx"] * DIRECTION["wroll"] * speed * dt
                if "lb" in held:
                    target["wflex"] -= DIRECTION["wflex"] * speed * dt
                if "rb" in held:
                    target["wflex"] += DIRECTION["wflex"] * speed * dt
                if "up" in pressed:
                    level = min(level + 1, 2)
                if "down" in pressed:
                    level = max(level - 1, 0)
                if "a" in pressed and clamp == "open":
                    clamp = "closing"
                if "b" in pressed:
                    clamp = "release"
                if "x" in pressed:
                    target = dict(here)
                if "y" in pressed:
                    read_keys()  # drop stale keys
                    target["_home"] = True
                if "back" in pressed:
                    raise KeyboardInterrupt
            if target.pop("_home", False):
                try:
                    with open(poses_path, encoding="utf-8") as f:
                        home = json.load(f)[HOME_POSE]
                    arm.move_slow(target={j: home[j] for j in MOVE_JOINTS}, verbose=False)
                    here = arm.joints()
                    target.update({j: here[j] for j in MOVE_JOINTS})
                except (OSError, KeyError):
                    print(f"\nNo pose '{HOME_POSE}' saved - teach it in teach.py")

            # ---- clamp logic ----
            grip_load = 0.0
            if clamp == "closing":
                grip_load = arm.loads()["grip"]
                if abs(grip_load) > CLAMP_LOAD or here["grip"] <= limits["grip"][0] + 1:
                    set_grip(here["grip"] - CLAMP_SQUEEZE)
                    clamp = "holding"
                    print(f"\nCLAMPED (load {abs(grip_load):.0f}%) - SPACE / B to release")
                else:
                    set_grip(min(target["grip"], here["grip"] + 2) - CLAMP_SPEED * dt)
            elif clamp == "release":
                set_grip(here["grip"] + RELEASE_OPEN)
                clamp = "open"
                print("\nRELEASED")

            # ---- safety: stay inside limits and close to the real arm ----
            for j in MOVE_JOINTS:
                lo, hi = limits[j]
                target[j] = min(max(target[j], lo, here[j] - MAX_LEAD), hi, here[j] + MAX_LEAD)

            servo_speed = int(max(speed, CLAMP_SPEED) * 2 * TICKS_PER_DEG)
            for j in MOVE_JOINTS + ["grip"]:
                if abs(target[j] - sent.get(j, 1e9)) > 0.05:
                    arm._write_goal(arm.ids[j], _to_ticks(target[j]), servo_speed)
                    sent[j] = target[j]

            if now_t - last_status > 0.25:
                last_status = now_t
                pos = "  ".join(f"{j}={here[j]:6.1f}" for j in MOVE_JOINTS + ["grip"])
                print(f"\r[speed {level + 1}] [{clamp:8s}] {pos}   ", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopping - arm holds its position.")
    except SO101Error as e:
        print("\nERROR:", e)
    finally:
        try:
            arm.hold()
        except SO101Error:
            pass
        arm.close()


if __name__ == "__main__":
    main()
