"""SO-101 arm - EVERYTHING IN ONE FILE.

  python so101_station.py         (or the Run button in VS Code) camera window + BOTTLE TRACKING:
                             a neural network detects bottles/cans, the arm keeps the nearest one in the middle
                             of the image, lets go when it moves away or is lost, switches to a closer one (T on/off).
                             The barcode is read along the way: green box = deposit bottle.
  python so101_station.py --no-camera   control from the terminal + can inspection on ENTER, no camera
  python so101_station.py --mock        no arm (camera only, the arm stays still)
  python so101_station.py --web         no window: preview, buttons and sliders in the browser http://<IP>:8765/
                                        (turns on by itself on Linux without a monitor, e.g. Raspberry over SSH)
  (the adapter port is found automatically; you can pass e.g. COM5)

Keys (click the camera window):
  W/S UP/DOWN     R/F FARTHER/CLOSER     A/D LEFT/RIGHT   (the gripper moves like a crane hook)
  J/L tilt gripper   U/O roll gripper   Z/X open/close the gripper a little
  SPACE grip/release (closes until it feels resistance)    1/2/3 speed    B stop
  ENTER  INSPECT CAN: the arm circles the can with the camera until it sees the barcode
  P save the current pose as scan1..scan4 (circuit)   C delete the scan poses
  M save the home pose   H go home   Q/ESC quit (the arm holds its position)
Xbox pad: left stick left/right + up/down, right stick farther/closer + roll, LB/RB tilt,
  A grip, B release, X stop, Y inspect can, BACK quit.
Signal from the RoArm: http://<laptop-IP>:8765/scan  (JSON answer with the code)
Bottle inspection (state machine): SEARCHING -> CENTERING -> READING -> RESULT. Result: http://<IP>:8765/result
  (JSON: result DEPOSIT/NO_DEPOSIT/NO_CODE, "rotate_bottle": true/false). After rotating: /rotated
Emergency: pull the servo power supply plug.

Angles in degrees from the middle of the encoder range: (ticks - 2048) * 360 / 4096.
"""

import ctypes
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

WINDOWS = sys.platform.startswith("win")
if not WINDOWS:
    # OpenCV with Qt on Linux (Wayland) needs X11, otherwise the window won't open
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
# without a monitor (Raspberry over SSH / as a service) an OpenCV window would kill the program - then the browser panel
HEADLESS = "--web" in sys.argv or (not WINDOWS and not os.environ.get("DISPLAY")
                                   and not os.environ.get("WAYLAND_DISPLAY"))

# STS3215 registers (SRAM)
ADDR_TORQUE_ENABLE = 40
ADDR_ACC = 41
ADDR_GOAL_POSITION = 42  # 42-43 position, 44-45 time, 46-47 speed
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60  # 0-1000 (0.1 %), bit 10 = direction

TICKS_PER_DEG = 4096 / 360
MAX_TICK = 4095

# Standard SO-101 servo IDs (after lerobot setup-motors)
SO101_IDS = {"pan": 1, "lift": 2, "elbow": 3, "wflex": 4, "wroll": 5, "grip": 6}


class SO101Error(Exception):
    pass


def _to_deg(ticks):
    return round((ticks - 2048) / TICKS_PER_DEG, 1)


def _to_ticks(deg):
    return max(0, min(MAX_TICK, int(round(deg * TICKS_PER_DEG + 2048))))


class SO101:
    JOINTS = list(SO101_IDS)
    ARM_JOINTS = JOINTS[:5]

    def __init__(self, port, baud=1_000_000, speed=30, acc=50, ids=None):
        from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

        self._ok = COMM_SUCCESS
        self.ids = dict(ids or SO101_IDS)
        self.speed = speed
        self.acc = acc
        # No defaults - they depend on the assembly. Set via "cal open/closed".
        self.gripper_open = None
        self.gripper_closed = None
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise SO101Error(f"cannot open {port}")
        if not self.port.setBaudRate(baud):
            raise SO101Error(f"cannot set {baud} bps")
        self.ph = PacketHandler(0)  # 0 = STS/SMS (little-endian)
        missing = [name for name, sid in self.ids.items() if not self.ping(sid)]
        if missing:
            raise SO101Error(
                f"no answer from servos: {', '.join(missing)} "
                f"(ID {[self.ids[m] for m in missing]}). Power? Bus cable? Use --scan."
            )

    def close(self):
        self.port.closePort()

    def ping(self, sid):
        _, res, _ = self.ph.ping(self.port, sid)
        return res == self._ok

    def scan(self, max_id=20):
        return [sid for sid in range(1, max_id + 1) if self.ping(sid)]

    def _read_pos(self, sid):
        for _ in range(3):
            val, res, _ = self.ph.read2ByteTxRx(self.port, sid, ADDR_PRESENT_POSITION)
            if res == self._ok:
                return -(val & 0x7FFF) if val & 0x8000 else val
        raise SO101Error(f"cannot read the position of servo ID {sid}")

    def _write_goal(self, sid, ticks, speed_ticks):
        if sid in {self.ids[n] for n in LOCKED_JOINTS if n in self.ids}:
            return  # locked joint (e.g. camera screwed onto the gripper) - we never move it
        # acc, position L/H, time L/H (0), speed L/H - like WritePosEx from the Feetech SDK
        data = [
            self.acc,
            ticks & 0xFF, (ticks >> 8) & 0xFF,
            0, 0,
            speed_ticks & 0xFF, (speed_ticks >> 8) & 0xFF,
        ]
        res, _ = self.ph.writeTxRx(self.port, sid, ADDR_ACC, len(data), data)
        if res != self._ok:
            raise SO101Error(f"cannot send the goal to servo ID {sid}")

    def joints(self):
        return {name: _to_deg(self._read_pos(sid)) for name, sid in self.ids.items()}

    def loads(self):
        """Servo load in % (sign = pushing direction)."""
        out = {}
        for name, sid in self.ids.items():
            val, res, _ = self.ph.read2ByteTxRx(self.port, sid, ADDR_PRESENT_LOAD)
            if res != self._ok:
                raise SO101Error(f"cannot read the load of servo ID {sid}")
            load = (val & 0x3FF) / 10
            out[name] = -load if val & 0x400 else load
        return out

    def move(self, pose, speed=None, wait=True, tol=3.0, timeout=15.0):
        # Speed 0 = maximum for the STS3215 - we never send that.
        if not getattr(self, "_torque_on", False):
            self.torque(True)
        spd = max(1, int((speed or self.speed) * TICKS_PER_DEG))
        for name, deg in pose.items():
            if name in self.ids:
                self._write_goal(self.ids[name], _to_ticks(deg), spd)
        if wait:
            self.wait_reached(pose, tol, timeout)
        current = self.joints()
        current.update({k: v for k, v in pose.items() if k in self.ids})
        return current

    def wait_reached(self, target, tol=2.0, timeout=15.0, settle_tol=10.0):
        """Wait until the joints arrive. SO-101 servos have a low P (16), so under load
        they stop a few degrees short of the goal - if a joint stopped closer than settle_tol,
        we count it as arrived. Farther = blocked/obstacle -> error."""
        check = [j for j in self.ARM_JOINTS if j in target]
        deadline = time.time() + timeout
        last, still_since = None, None
        while time.time() < deadline:
            now = {j: _to_deg(self._read_pos(self.ids[j])) for j in check}
            err = max((abs(now[j] - target[j]) for j in check), default=0.0)
            if err <= tol:
                return True
            if last and all(abs(now[j] - last[j]) < 0.3 for j in check):
                still_since = still_since or time.time()
                if time.time() - still_since > 0.5:
                    if err <= settle_tol:
                        return True
                    self.hold()
                    raise SO101Error(f"joint stopped {err:.0f} deg from the goal (obstacle?)")
            else:
                still_since = None
            last = now
            time.sleep(0.05)
        raise SO101Error(f"did not reach the pose in {timeout}s (obstacle or out of reach?)")

    def move_slow(self, delta=None, target=None, step=2.0, speed=10.0, max_load=60.0, verbose=True):
        """Safe movement in small steps with load monitoring.

        delta={"lift": 20}      -> by 20 deg from the current position
        target={"lift": -30}    -> to a specific angle
        Stops (and holds position) when the load > max_load %% or a joint gets stuck.
        """
        start = self.joints()
        goal = dict(target or {})
        for j, d in (delta or {}).items():
            goal[j] = start[j] + d
        goal = {j: v for j, v in goal.items() if j in self.ids}
        if not goal:
            return start
        steps = max(1, math.ceil(max(abs(goal[j] - start[j]) for j in goal) / step))
        for i in range(1, steps + 1):
            waypoint = {j: start[j] + (goal[j] - start[j]) * i / steps for j in goal}
            self.move(waypoint, speed=speed, wait=False)
            try:
                self.wait_reached(waypoint, tol=1.0, timeout=step / speed + 3.0)
            except SO101Error:
                self.hold()
                raise
            now, loads = self.joints(), self.loads()
            if verbose:
                info = "  ".join(f"{j}={now[j]:6.1f} load={loads[j]:5.1f}%" for j in goal)
                print(f"  [{i:2d}/{steps}] {info}")
            worst = max(abs(loads[j]) for j in self.ARM_JOINTS)
            if worst > max_load:
                self.hold()
                raise SO101Error(f"STOP: load {worst:.0f}% > {max_load:.0f}% - the arm holds its position")
        return self.joints()

    def _need_gripper_cal(self, value, which):
        if value is None:
            raise SO101Error(f"gripper not calibrated - set it by hand and type 'cal {which}'")
        return value

    def grip(self, angle=None, settle=0.6):
        angle = angle if angle is not None else self._need_gripper_cal(self.gripper_closed, "closed")
        self.move({"grip": angle}, wait=False)
        time.sleep(settle)

    def release(self, angle=None, settle=0.6):
        angle = angle if angle is not None else self._need_gripper_cal(self.gripper_open, "open")
        self.move({"grip": angle}, wait=False)
        time.sleep(settle)

    def hold(self):
        self.move(self.joints(), wait=False)

    def torque(self, on):
        for name, sid in self.ids.items():
            if name in LOCKED_JOINTS:  # no torque = the servo doesn't push, can't burn out
                self.ph.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, 0)
                continue
            if on:
                # Goal = current position, otherwise the servo jerks to the old goal.
                self._write_goal(sid, max(0, self._read_pos(sid)), int(20 * TICKS_PER_DEG))
            self.ph.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, 1 if on else 0)
        self._torque_on = on


class MockSO101(SO101):
    """Pretends to be an SO-101 without hardware."""

    def __init__(self, speed=30, acc=50):
        self.ids = dict(SO101_IDS)
        self.speed = speed
        self.acc = acc
        self.gripper_open = None
        self.gripper_closed = None
        self._pose = {name: 0.0 for name in self.ids}

    def close(self):
        pass

    def joints(self):
        return dict(self._pose)

    def loads(self):
        return {name: 0.0 for name in self.ids}

    def move(self, pose, speed=None, wait=True, tol=3.0, timeout=15.0):
        for k, v in pose.items():
            if k in self._pose:
                self._pose[k] = float(v)
        print(f"  [mock] -> {pose} spd={speed or self.speed}")
        return dict(self._pose)

    def wait_reached(self, target, tol=2.0, timeout=15.0, settle_tol=10.0):
        return True

    def torque(self, on):
        print(f"  [mock] torque {'on' if on else 'off'}")

    def scan(self, max_id=20):
        return list(self.ids.values())


def _mock_write_goal(self, sid, ticks, speed_ticks):
    name = next(n for n, i in self.ids.items() if i == sid)
    if name in LOCKED_JOINTS:
        return
    self._pose[name] = _to_deg(ticks)


MockSO101._write_goal = _mock_write_goal


# =============================================================================
#  SETTINGS - you can change these
# =============================================================================
SPEEDS = [10.0, 25.0, 50.0]      # deg/s for speed 1 / 2 / 3 (rotations: base, wrist)
SPEEDS_MM = [20.0, 50.0, 100.0]  # mm/s for speed 1 / 2 / 3 (up/down, farther/closer)
ARM_L1 = 116.0                   # mm: shoulder -> elbow (SO-101)
ARM_L2 = 135.0                   # mm: elbow -> wrist (SO-101)
HOLD_GRIPPER_ANGLE = True        # on up/down/farther/closer the gripper (camera) keeps its tilt
COMPENSATION_SIGN = 1            # flip to -1 if the gripper tilts instead of holding its angle
KEY_STEP_TIME = 1 / 30           # a held key repeats ~30x/s
MAX_LEAD = 12.0                  # the goal may lead the real arm by at most this many deg
LIMIT_MARGIN = 3.0               # distance from the servo angle limits (deg)
CLAMP_SPEED = 30.0               # deg/s when closing on an object
CLAMP_LOAD = 20.0                # % load = "touched the object"
CLAMP_SQUEEZE = 4.0              # deg of extra squeeze after touching
RELEASE_OPEN = 35.0              # how many deg the gripper opens on release
LOCKED_JOINTS = {"grip"}         # these joints: torque off and zero movement (camera screwed onto the gripper!)
BOOST_P = False                  # P=32 on lift/elbow until power-off (with 16 they sag)

CAMERA = None                    # None = picks one itself (external, and if there is none - the laptop's)
SCAN_POSES = ["scan1", "scan2", "scan3", "scan4"]
AUTO_MOVES = [                   # looking around the start position when there are no scan1..4 poses
    {"pan": -20}, {"pan": -10}, {"pan": 0}, {"pan": 10}, {"pan": 20},
    {"pan": 0, "wflex": -15}, {"pan": 0, "wflex": 15},
]
SCAN_PAUSE = 0.8                 # how many s it looks in each position
SCAN_SPEED = 25                  # deg/s
# --- bottle inspection: SEARCHING -> CENTERING -> READING -> RESULT ---
LOOK_AROUND = False              # False = the camera waits in the waiting position (the other arm brings the bottle)
RETURN_AFTER = 3.0               # s without a bottle -> the camera calmly returns to the waiting position
SEARCH_AFTER = 2.0               # s without a bottle -> the camera starts looking around (only when LOOK_AROUND = True)
SEARCH_RANGE_H = 30.0            # deg: looking around left/right of the start position
SEARCH_RANGE_V = 15.0            # deg: looking around up/down of the start position
SEARCH_SPEED = 12.0              # deg/s: how fast it looks around (slow = the camera sees sharply)
CENTERED_TIME = 0.3              # s calmly in the middle -> starts reading the code
CODE_READ_TIME = 4.0             # s of reading the code -> if nothing, "NO CODE - rotate the bottle"
DECISION_URL = ""                # e.g. "http://192.168.1.30:8000/decision" - the result is sent there automatically (POST JSON)
HTTP_PORT = 8765                 # signal from the RoArm: http://<IP>:8765/scan
WEB_FPS = 20                     # at most this many preview frames/s in the browser (WiFi limits the rest)
WEB_WIDTH = 640                  # width of preview frames: smaller = more frames/s over weak WiFi
HOME_POSE = "look"
POSES_FILE = "poses_so101.json"
DEPOSIT_FILE = "deposit_list.json"  # own list of deposit EANs (besides api.kaucja.pl)

# --- deposit bottle tracking (camera mode, key T turns it on/off) ---
TRACK = True                     # starts enabled
# SMOOTH tracking: arm speed proportional to the target's distance from the image center.
TRACK_SMOOTH = True              # False = the old "correct and wait" mode (move - pause - move)
TRACK_LOOP_GAIN = 2.0            # reaction speed (1/s): more = catches up faster, too much = overshoots
                                 # (was 1.2 with YOLO on Brev; 2.0 needs YOLO on the Pi while tracking - simulation:
                                 # 12 deg in 0.6 s instead of ~1 s, no swaying; with Brev latency 2.0 would sway)
TRACK_MAX_SPEED = 45.0           # deg/s: fastest move while tracking (faster = blurred image, unreadable code)
TRACK_FILTER = 0.5               # smoothing of the target position (0..1, less = smoother, but reacts slower)
SENSITIVITY_RANGE = (0.6, 2.0)   # sensitivity learning only within this range x the (measured) start value - smaller =
                                 # too strong a reaction = waving, larger = the arm lazily catches up with the bottle
TRACK_LEARN_SENSITIVITY = True   # measure live how far the image shifts per 1 deg of movement (depends on distance)
TRACK_START_SMOOTH = 0.10        # a stationary arm starts moving when the target drifts farther than this (detection jitter = no move)
TRACK_BRAKE = 0.25               # s: how fast it brakes after a momentary loss of the target
TRACK_FF = 0.5                   # prediction: move with the bottle's speed (0 = off, 1 = full)
TRACK_DELAY = 0.3                # s: camera delay (the image shows the arm from this many seconds ago) - when the frame age is unknown
CAMERA_DELAY = 0.17              # s: from a servo command to the frame showing that move; measured on the SO-101 +
                                 # this camera: the servo starts after ~140 ms, the image shows the move ~45 ms later.
                                 # The program adds the frame age to this (YOLO time etc.)
TRACK_FF_THRESHOLD = 1.0         # deg/s: slower bottle movement = noise, ignore
TRACK_FF_SMOOTHING = 0.3         # 0..1: how fast the prediction reacts to a change in the bottle's movement
# "Correct and wait" mode (TRACK_SMOOTH = False): one move towards the target, pause, new frame, next move.
TRACK_STEP = 0.5                 # what part of the distance to the center one move covers (0.3 calm, 0.8 fast)
TRACK_SENSITIVITY_H = 0.020      # start: how far the image shifts (fraction of half the width) per 1 deg of base -
                                 # measured: 3.2 px/deg at 320 px (then the program keeps learning - depends on distance)
TRACK_SENSITIVITY_V = 0.030      # start: the same for the wrist (measured: 2.7 px/deg at 180 px height)
TRACK_TARGET_ZONE = 0.04         # this close to the center = centered, the arm stops
TRACK_START = 0.16               # a centered arm moves only when the code runs farther than this (no jitter)
TRACK_MIN_STEP = 0.5             # doesn't make smaller moves (deg)
TRACK_MAX_STEP = 8.0             # largest single move (deg)
TRACK_PAUSE = 0.7                # s after a move before looking again (camera delay + the arm stops swaying)
TRACK_SPEED = 15                 # deg/s of tracking moves - slow = the camera doesn't sway
TRACK_ACC = 15                   # acceleration of tracking moves (less = gentler start and braking)
TRACK_LEAD = 0.3                 # s: the servo gets its goal this far ahead -> moves continuously, doesn't stop every frame
TRACK_AIM_AT_CODE = False        # True = after reading, aim at the code itself (lower; the bottle top may leave the frame)
TRACK_AIM_HEIGHT = 0.45          # where to aim at the bottle: 0 = top, 0.5 = middle, 1 = bottom (label with the deposit)
TRACK_ASPECT = {"bottle": 2.8, "can": 1.8, "basket": 1.0}  # height/width - to estimate a cut-off bottle
CUT_MAX = 1.3                    # cut-off bottle: guessed size at most this many times the visible part
CUT_MAX_SPEED = 12.0             # deg/s: when the bottle sticks out of the frame, the target is only an estimate - move carefully
CLOSE_FROM = 0.72                # bottle taller than 72% of the frame = right in front of the camera: don't aim vertically (label visible)
TRACK_ACCEL = 120.0              # deg/s^2: the tracking speed changes at most this fast (smooth start and braking)
TRACK_SAMPLES = 3                # median code position over this many frames
# Bottle detection with a neural network. YOLO11n (onnxruntime) also sees a bottle partly outside the frame;
# SSD MobileNet v2 (OpenCV only) - fallback when there's no onnxruntime.
DETECTOR = "yolo"                # "yolo" or "ssd"
YOLO_SIZES = (320, 640)          # YOLO at two scales: 320 sees close bottles, 640 farther ones
YOLO_INTERLEAVE = True           # a different scale every frame (results of the other one from the previous frame) = 2x faster
YOLO_TRACKING_SIZE = 320         # tracked bottle large in the frame: only this scale, fresh every frame (Pi: 32 ms)
YOLO_LARGE_FROM = 0.22           # "large" = taller than 22% of the image (smaller: both scales, because at 320 it would vanish)
YOLO_CLASSES = {39: "bottle", 41: "can"}   # COCO classes in YOLO: 39 bottle, 41 cup (that's how it sees cans)
SSD_CLASSES = {44: "bottle", 47: "can"}    # the same in SSD numbering: 44 bottle, 47 cup
BOTTLE_THRESHOLD = 0.25          # confidence (0..1) to CATCH a new bottle
CAN_THRESHOLD = 0.50             # the same for "can" - higher, because the network also calls cups that
BOTTLE_HOLD_THRESHOLD = 0.15     # confidence to KEEP an already tracked one (a momentary confidence dip doesn't lose the target)
MIN_BOTTLE = 0.12                # bottle lower than 12% of the image height = too far -> lets go
TRACK_CONFIRM = 3                # in how many consecutive frames the bottle must be visible to catch it
TRACK_SWITCH_FRAMES = 8          # ... and another bottle must be clearly closer for this many frames (~0.4 s) to switch
TRACK_DEPOSIT_ONLY = False       # True = track only bottles with a read deposit code
TRACK_OBJECT = "bottle"          # what the camera keeps centred: "bottle" (YOLO) or "basket" (key G, /track_object)
# the vehicle's basket: light grey rim + white inside on an orange wooden floor = a large, solid, low-saturation blob
BASKET_MAX_SAT = 45              # HSV saturation 0..255: at most this (grey/white)
BASKET_MIN_VAL = 150             # HSV value 0..255: at least this (light)
BASKET_MIN_AREA = 0.04           # the blob covers at least this part of the image
READS_PER_SECOND = 5             # how many times per second to read the code (more = faster, but loads the CPU)
CODE_CONFIRM = 2                 # this many matching EAN reads to accept it (1 wrong read doesn't spoil the result)
CODE_MISSING_AFTER = 3.0         # s without reading the tracked bottle's code -> "rotate the bottle code to the camera"
TRACK_SEARCH_AFTER = 0.5         # s: search only after this many seconds without the code (ignore single lost frames)
TRACK_SEARCH_TIME = 1.5          # s: after losing the code keep moving in the direction the code was escaping
TRACK_SEARCH_MAX = 8.0           # deg: at most this far "blind" after losing the code
SERVO_ACC = 20                   # servo acceleration (less = gentler; was 50)
SERVO_DEADBAND = 1               # encoder steps (1 = factory); more = slow moves go in jumps
TRACK_LOG = "tracking.log"       # run log (for diagnosis); "" = no log
TRACK_LOG_EVERY = float(os.environ.get("DK_LOG_EVERY", "0.3"))  # s between control entries (DK_LOG_EVERY=0 = every frame)
TRACK_SIGN_H = -1                # flip to -1 if the arm RUNS AWAY from the code horizontally
TRACK_SIGN_V = -1                # flip to -1 if it runs away vertically
TRACK_JOINT_H = "pan"            # which joint aims horizontally
TRACK_JOINT_V = "wflex"          # which joint aims vertically
MIN_CODE_WIDTH = 0.08            # code narrower than 8% of the image = too far -> lets go of the target
SWITCH_WHEN = 1.3                # another deposit bottle with a 1.3x bigger code (closer) -> switch
LOST_AFTER = 2.0                 # s without the code -> target lost
TRACK_RADIUS = 0.25              # unread (blurred) code closer than 25% of the image to the last position = the same one
CAMERA_RESOLUTION = (2560, 1440)  # this camera then gives 1920x1080 at 30 fps = sharp barcode
#                                   (asking for 1920x1080 directly = only 5 fps, 1280x720 = 10 fps)
CAMERA_EXPOSURE = None           # None = auto (brightest). Fixed: -4 bright but blurs, -5/-6 less blur, needs a lamp
ON_LOSS = "stay"                 # "stay" or "home" (go back to the home pose)

# Flip 1 / -1 if a key moves the wrong way.
DIRECTION = {"pan": 1, "lift": -1, "elbow": -1, "wflex": 1, "wroll": 1}
# =============================================================================

KEYS = {  # key -> (joint, sign) - rotations
    "a": ("pan", 1), "d": ("pan", -1),
    "j": ("wflex", 1), "l": ("wflex", -1),
    "u": ("wroll", 1), "o": ("wroll", -1),
}
KEYS_CRANE = {  # key -> (farther/closer, up/down) - the gripper moves like a crane hook
    "w": (0, 1), "s": (0, -1),
    "r": (1, 0), "f": (-1, 0),
}
MOVE_JOINTS = ["pan", "lift", "elbow", "wflex", "wroll"]

HELP = (
    "W/S up/down | R/F farther/closer | A/D left/right | J/L tilt | U/O roll | SPACE grip | 1 2 3 speed | B stop\n"
    "ENTER inspect can | T tracking | P save circuit pose | C delete circuit poses | M save home | H go home | Q quit"
)


def fk(lift, elbow):
    """Arm kinematics in its plane: (reach mm, height mm, forearm angle deg).
    Angles as in the program: lift 0 = upper arm vertical (+ backwards), elbow -90 = forearm straight up."""
    a = math.radians(-lift)
    b = a + math.radians(elbow + 90)
    return (ARM_L1 * math.sin(a) + ARM_L2 * math.sin(b),
            ARM_L1 * math.cos(a) + ARM_L2 * math.cos(b), math.degrees(b))


def ik(x, z):
    """The inverse: (lift, elbow) for the wrist at point (x, z), elbow bent forward. None = out of reach."""
    c = (x * x + z * z - ARM_L1 ** 2 - ARM_L2 ** 2) / (2 * ARM_L1 * ARM_L2)
    if not -1.0 <= c <= 0.999:  # too far or the arm almost straight (kinematics goes crazy there)
        return None
    phi = math.acos(c)
    a = math.atan2(x, z) - math.atan2(ARM_L2 * math.sin(phi), ARM_L1 + ARM_L2 * math.cos(phi))
    a = (a + math.pi) % (2 * math.pi) - math.pi  # into the -180..180 range (otherwise 281 instead of 79 deg)
    return -math.degrees(a), math.degrees(phi) - 90

_dir = os.path.dirname(os.path.abspath(__file__))


def load_json(name, default):
    try:
        with open(os.path.join(_dir, name), encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return default


def save_json(name, data):
    with open(os.path.join(_dir, name), "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def find_port():
    """Find the SO-101 servo adapter (CH343/CH340 chip, VID 1A86)."""
    from serial.tools import list_ports

    ports = [p.device for p in list_ports.comports() if p.vid == 0x1A86]
    if not ports:
        raise SO101Error("can't see the servo adapter (CH343) - is the USB cable plugged in?")
    return ports[0]


def connect(port=None, mock=False):
    return MockSO101() if mock else SO101(port or find_port())


def my_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


# ----------------------------------------------------------------------------- pad
class Gamepad:
    """Xbox pad / XInput via ctypes - no extra libraries."""

    class _State(ctypes.Structure):
        _fields_ = [("packet", ctypes.c_uint32), ("buttons", ctypes.c_uint16),
                    ("lt", ctypes.c_uint8), ("rt", ctypes.c_uint8),
                    ("lx", ctypes.c_int16), ("ly", ctypes.c_int16),
                    ("rx", ctypes.c_int16), ("ry", ctypes.c_int16)]

    BTN = {"up": 0x0001, "down": 0x0002, "start": 0x0010, "back": 0x0020, "lb": 0x0100,
           "rb": 0x0200, "a": 0x1000, "b": 0x2000, "x": 0x4000, "y": 0x8000}

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


# ----------------------------------------------------------------------------- manual control
class Controller:
    """Manual control (keys + pad) with safety limits. Shared by both modes."""

    def __init__(self, arm):
        self.arm = arm
        self.mock = isinstance(arm, MockSO101)
        self.limits = self._limits()
        if BOOST_P and not self.mock:
            for j in ("lift", "elbow"):
                sid = arm.ids[j]
                if arm.ph.read1ByteTxRx(arm.port, sid, 55)[0] != 1:
                    arm.ph.write1ByteTxRx(arm.port, sid, 55, 1)  # EEPROM locked -> until power-off
                arm.ph.write1ByteTxRx(arm.port, sid, 21, 32)
        arm.acc = SERVO_ACC  # gentle acceleration and braking = no jerks
        if not self.mock:
            for j in arm.ARM_JOINTS:  # deadband (until power-off - EEPROM locked)
                sid = arm.ids[j]
                if arm.ph.read1ByteTxRx(arm.port, sid, 55)[0] != 1:
                    arm.ph.write1ByteTxRx(arm.port, sid, 55, 1)
                arm.ph.write1ByteTxRx(arm.port, sid, 26, SERVO_DEADBAND)
                arm.ph.write1ByteTxRx(arm.port, sid, 27, SERVO_DEADBAND)
        self.slow = set()        # joints that tracking wants to move slowly and gently
        self.track_speed = {}    # tracking speed (deg/s, signed) - the servo moves continuously instead of in jumps
        arm.torque(True)
        self.pad = Gamepad()
        self.pad_ok = self.pad.read() is not None
        self.level = 1
        self.clamp = "open"  # open | closing | holding
        self.message = ""
        self.here = arm.joints()
        self.target = dict(self.here)
        self.sent = dict(self.here)
        self._t = time.time()

    def _limits(self):
        if self.mock:
            return {j: (-150.0, 150.0) for j in self.arm.JOINTS}
        lim = {}
        for j in self.arm.JOINTS:
            sid = self.arm.ids[j]
            lo = self.arm.ph.read2ByteTxRx(self.arm.port, sid, 9)[0]
            hi = self.arm.ph.read2ByteTxRx(self.arm.port, sid, 11)[0]
            lim[j] = (_to_deg(lo) + LIMIT_MARGIN, _to_deg(hi) - LIMIT_MARGIN)
        return lim

    def after_task(self):
        """After an automatic move: take over the joint positions, the gripper keeps holding."""
        self.here = self.arm.joints()
        for j in MOVE_JOINTS:
            self.target[j] = self.sent[j] = self.here[j]
        self._t = time.time()

    def _grip(self, deg):
        lo, hi = self.limits["grip"]
        self.target["grip"] = min(max(deg, lo), hi)

    def shift(self, farther_mm, up_mm):
        """Move the gripper up/down and farther/closer (lift + elbow together), keeping the gripper angle."""
        x, z, beta = fk(self.target["lift"], self.target["elbow"])
        result = ik(x + farther_mm, z + up_mm)
        if result is None:
            if not self.message:
                self.message = "out of the arm's reach"
            return
        lift, elbow = result
        if not (self.limits["lift"][0] <= lift <= self.limits["lift"][1]
                and self.limits["elbow"][0] <= elbow <= self.limits["elbow"][1]):
            if not self.message:
                self.message = "end of the arm's range - move the other way"
            return
        # with a folded arm a small gripper move = a huge joint move -> block instead of jerking
        jump = max(abs(lift - self.target["lift"]), abs(elbow - self.target["elbow"]))
        if jump > 3.0 * max(1.0, (abs(farther_mm) + abs(up_mm)) / 2.0):
            if not self.message:
                self.message = "arm folded too much - first R (farther) to unfold it"
            return
        self.target["lift"], self.target["elbow"] = lift, elbow
        if HOLD_GRIPPER_ANGLE:
            self.target["wflex"] -= COMPENSATION_SIGN * (fk(lift, elbow)[2] - beta)

    def key(self, k):
        """Handle a movement key. Returns True if the key was a movement key."""
        v = SPEEDS[self.level]
        if k in KEYS_CRANE:
            farther, up = KEYS_CRANE[k]
            step = SPEEDS_MM[self.level] * KEY_STEP_TIME
            self.shift(farther * step, up * step)
        elif k in KEYS:
            joint, sign = KEYS[k]
            self.target[joint] += sign * DIRECTION[joint] * v * KEY_STEP_TIME
        elif k in ("1", "2", "3"):
            self.level = int(k) - 1
        elif k in ("z", "x", " ") and "grip" in LOCKED_JOINTS:
            self.message = "gripper LOCKED (camera) - change LOCKED_JOINTS in so101_station.py"
        elif k == "z":
            self.clamp = "open"
            self._grip(self.target["grip"] + 3)
        elif k == "x":
            self.clamp = "open"
            self._grip(self.target["grip"] - 3)
        elif k == " ":
            self.clamp = "closing" if self.clamp == "open" else "release"
        elif k == "b":
            self.target = dict(self.here)
            if self.clamp == "closing":
                self.clamp = "open"
        else:
            return False
        return True

    def step(self):
        """One cycle: pad, gripper, safety limits, sending goals. Returns newly pressed pad buttons."""
        now_t = time.time()
        dt = min(now_t - self._t, 0.1)
        self._t = now_t
        v = SPEEDS[self.level]
        self.here = self.arm.joints()
        pressed = set()

        state = self.pad.read()
        if state:
            sticks, pressed, held = state
            self.target["pan"] += -sticks["lx"] * DIRECTION["pan"] * v * dt
            vmm = SPEEDS_MM[self.level] * dt
            if sticks["ly"] or sticks["ry"]:  # left stick up/down, right farther/closer
                self.shift(sticks["ry"] * vmm, sticks["ly"] * vmm)
            self.target["wroll"] += sticks["rx"] * DIRECTION["wroll"] * v * dt
            if "lb" in held:
                self.target["wflex"] -= DIRECTION["wflex"] * v * dt
            if "rb" in held:
                self.target["wflex"] += DIRECTION["wflex"] * v * dt
            if "up" in pressed:
                self.level = min(self.level + 1, 2)
            if "down" in pressed:
                self.level = max(self.level - 1, 0)
            if "grip" not in LOCKED_JOINTS:
                if "a" in pressed and self.clamp == "open":
                    self.clamp = "closing"
                if "b" in pressed:
                    self.clamp = "release"
            if "x" in pressed:
                self.target = dict(self.here)

        if self.clamp == "closing":
            load = self.arm.loads()["grip"]
            if abs(load) > CLAMP_LOAD or self.here["grip"] <= self.limits["grip"][0] + 1:
                self._grip(self.here["grip"] - CLAMP_SQUEEZE)
                self.clamp = "holding"
                self.message = f"GRIPPED (resistance {abs(load):.0f}%)"
                print(f"\n{self.message}")
            else:
                self._grip(min(self.target["grip"], self.here["grip"] + 2) - CLAMP_SPEED * dt)
        elif self.clamp == "release":
            self._grip(self.here["grip"] + RELEASE_OPEN)
            self.clamp = "open"
            self.message = "RELEASED"
            print(f"\n{self.message}")

        for j in MOVE_JOINTS:
            lo, hi = self.limits[j]
            self.target[j] = min(max(self.target[j], lo, self.here[j] - MAX_LEAD), hi, self.here[j] + MAX_LEAD)

        spd = int(max(v, CLAMP_SPEED) * 2 * TICKS_PER_DEG)
        for j in self.arm.JOINTS:
            if j in self.track_speed:
                # smooth tracking: goal pushed TRACK_LEAD seconds of motion ahead, speed = tracking
                # speed -> the servo moves continuously and doesn't manage to brake before the next frame
                w = self.track_speed[j]
                lo, hi = self.limits[j]
                goal = min(max(self.target[j] + w * TRACK_LEAD, lo), hi)
                self.arm.acc = TRACK_ACC
                self.arm._write_goal(self.arm.ids[j], _to_ticks(goal), max(1, int(max(abs(w), 3.0) * TICKS_PER_DEG)))
                self.arm.acc = SERVO_ACC
                self.sent[j] = self.target[j]
                continue
            threshold = 0.05 if j in self.slow else 0.2  # step tracking: small steps
            if abs(self.target[j] - self.sent.get(j, 1e9)) > threshold:
                if j in self.slow:  # step tracking move: gently
                    self.arm.acc = TRACK_ACC
                    self.arm._write_goal(self.arm.ids[j], _to_ticks(self.target[j]), max(1, int(TRACK_SPEED * TICKS_PER_DEG)))
                    self.arm.acc = SERVO_ACC
                else:
                    self.arm._write_goal(self.arm.ids[j], _to_ticks(self.target[j]), spd)
                self.sent[j] = self.target[j]
        self.slow.clear()
        self.track_speed.clear()
        return pressed

    def describe(self):
        pos = "  ".join(f"{j}={self.here[j]:6.1f}" for j in self.arm.JOINTS)
        return f"[speed {self.level + 1}] [{self.clamp:7s}] {pos}"


def go_home(arm):
    poses = load_json(POSES_FILE, {})
    if HOME_POSE not in poses:
        raise SO101Error(f"no '{HOME_POSE}' pose - position the arm and press M")
    arm.move_slow(target={j: poses[HOME_POSE][j] for j in MOVE_JOINTS}, verbose=False)


def save_pose(arm, name):
    poses = load_json(POSES_FILE, {})
    poses[name] = arm.joints()
    save_json(POSES_FILE, poses)
    return f"saved pose '{name}'"


_terminal = {"old": None}


def _linux_raw_terminal():
    """Linux/macOS: terminal without waiting for Enter (like msvcrt on Windows). Restored on exit."""
    import atexit
    import termios
    import tty

    if _terminal["old"] is not None or not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    _terminal["old"] = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    atexit.register(lambda: termios.tcsetattr(fd, termios.TCSADRAIN, _terminal["old"]))


def read_keys():
    """Pressed keys without waiting (Windows and Linux). Enter always as "\\r"."""
    keys = []
    if WINDOWS:
        import msvcrt

        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):  # arrows/F-keys - skipped
                msvcrt.getwch()
                continue
            keys.append(ch.lower())
        return keys

    import select

    if not sys.stdin.isatty():
        return keys
    _linux_raw_terminal()
    text = ""
    while select.select([sys.stdin], [], [], 0)[0]:
        data = os.read(sys.stdin.fileno(), 64).decode(errors="ignore")
        if not data:
            break
        text += data
    i = 0
    while i < len(text):
        ch = text[i]
        if ch == "\x1b" and text[i + 1:i + 2] in ("[", "O"):  # arrows: ESC [ A - skipped
            i += 3
            continue
        keys.append("\r" if ch == "\n" else ch.lower())
        i += 1
    return keys


def pose_key(arm, c):
    """P/C/M pose saving. Returns a message or None if it's not one of these keys."""
    if c == "m":
        return save_pose(arm, HOME_POSE)
    if c == "p":
        poses = load_json(POSES_FILE, {})
        free = [p for p in SCAN_POSES if p not in poses]
        return save_pose(arm, free[0]) if free else "already 4 scan poses - C deletes them"
    if c == "c":
        poses = load_json(POSES_FILE, {})
        for p in SCAN_POSES:
            poses.pop(p, None)
        save_json(POSES_FILE, poses)
        return "scan poses deleted - ENTER will look around"
    return None


def manual_control(extra_keys=None, on_start=None, port=None, mock=False, http=False):
    """Terminal mode (no camera). Also used by pizza.py.

    extra_keys = {"p": function(arm)} - own keys (take precedence);
    on_start(arm) - action at start;  http=True - listen for the RoArm signal.
    """
    extra_keys = extra_keys or {}
    arm = connect(port, mock)
    if on_start:
        on_start(arm)
    st = Controller(arm)
    srv = http_server(arm) if http else None
    print(f"Pad: {'CONNECTED' if st.pad_ok else 'none (keyboard only)'}")
    print("Keys (the terminal must be active):\n" + HELP)
    if extra_keys:
        print("Extra keys:", ", ".join(k.upper() for k in extra_keys))
    if srv:
        print(f"Signal from the RoArm: http://{my_ip()}:{HTTP_PORT}/scan")
    print()
    last, was_busy = 0.0, False
    try:
        while True:
            for k in read_keys():
                if k in ("\x1b", "q"):
                    raise KeyboardInterrupt
                if k in extra_keys:
                    if _busy.is_set():
                        continue
                    print()
                    try:
                        extra_keys[k](arm)
                    except SO101Error as e:
                        print("ERROR:", e)
                    st.after_task()
                elif k == "\r":
                    run_task(arm, inspect_can, "scan") or print("\narm busy")
                elif _busy.is_set():
                    continue
                elif k == "h":
                    run_task(arm, go_home, "home")
                elif (msg := pose_key(arm, k)) is not None:
                    print(f"\n{msg}")
                else:
                    st.key(k)

            if _busy.is_set():
                was_busy = True
            else:
                if was_busy:
                    st.after_task()
                    was_busy = False
                pressed = st.step()
                if "y" in pressed or "start" in pressed:
                    run_task(arm, inspect_can, "scan")
                if "back" in pressed:
                    raise KeyboardInterrupt
                if time.time() - last > 0.25:
                    last = time.time()
                    print(f"\r{st.describe()}   ", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nDone - the arm holds its position.")
    except SO101Error as e:
        print("\nERROR:", e)
    finally:
        if srv:
            srv.shutdown()
        if _busy.is_set():
            _scan_done.wait(timeout=30)
        try:
            arm.hold()
        except SO101Error:
            pass
        arm.close()


# ----------------------------------------------------------------------------- can inspection
state = {"state": "waiting", "code": None, "result": None}
_busy = threading.Event()        # an automatic move is running (scan / going home)
_scan_done = threading.Event()


def open_camera():
    import cv2

    # CAMERA in the settings or an environment variable, e.g. on Linux:  CAMERA=4 python3 so101_station.py
    chosen = os.environ.get("CAMERA", CAMERA)
    if chosen is not None:
        order = [int(chosen)]
    else:  # external ones first, the laptop camera (0) last
        order = [1, 2, 3, 0] if WINDOWS else [2, 4, 6, 1, 3, 5, 0]
    backend = cv2.CAP_DSHOW if WINDOWS else cv2.CAP_V4L2
    for i in order:
        cam = cv2.VideoCapture(i, backend)
        # MJPG + 1280x720: sharper codes than the default 640x480, and still smooth
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION[0])
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION[1])
        # We always set the exposure explicitly (Windows remembers it in the driver).
        # Note: AUTO_EXPOSURE has different values - Windows 1 = auto, Linux (V4L2) 3 = auto, 1 = manual.
        if CAMERA_EXPOSURE is None:
            cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1 if WINDOWS else 3)
        else:
            if not WINDOWS:
                cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            cam.set(cv2.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE)
        if cam.isOpened() and cam.read()[0]:
            print(f"Camera {i}" + ((" (laptop)" if i == 0 else " (external)") if WINDOWS else f" (/dev/video{i})"))
            return cam
        cam.release()
    raise SO101Error("no camera found")


class CameraThread:
    """Camera read in a separate thread: always the NEWEST frame + the time it arrived.

    Without this the camera driver keeps a queue of old frames (a loop slower than 30 fps gets an image from
    100+ ms ago), and decoding 1080p MJPEG (~25 ms) blocks the loop. Here decoding runs in parallel with YOLO.
    """

    def __init__(self, cam):
        self.cam = cam
        self._new = threading.Condition()
        self._frame, self._t, self._nr, self._given = None, 0.0, 0, 0
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop:
            if not self.cam.grab():  # just grabbing the frame (no decoding) - cheap
                time.sleep(0.01)
                continue
            t = time.time()  # the frame just came from the camera
            if self._nr != self._given and t - self._t < 0.04:
                continue  # the previous fresh frame is waiting for the loop - don't decode ahead (CPU = heat)
            ok, frame = self.cam.retrieve()
            if ok:
                with self._new:
                    self._frame, self._t, self._nr = frame, t, self._nr + 1
                    self._new.notify_all()

    def read(self, timeout=1.0):
        """(ok, frame, frame_time) - waits for a frame it hasn't handed out yet."""
        with self._new:
            if not self._new.wait_for(lambda: self._nr != self._given, timeout=timeout):
                return False, None, 0.0
            self._given = self._nr
            return True, self._frame, self._t

    def release(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        self.cam.release()


def inspect_can(arm):
    """Circuit with the camera, ends when the camera sees a code. Returns the result."""
    t0 = time.time()
    state.update(state="scanning", code=None, result=None)
    start = arm.joints()
    poses = load_json(POSES_FILE, {})
    if all(p in poses for p in SCAN_POSES):
        moves = [(p, {j: poses[p][j] for j in MOVE_JOINTS}) for p in SCAN_POSES]
        print("\nINSPECTING CAN (taught poses)")
    else:
        moves = [(f"auto{i}", {j: start[j] + d for j, d in r.items()}) for i, r in enumerate(AUTO_MOVES, 1)]
        print("\nINSPECTING CAN (looking around - no scan1..4 poses)")
    code = None
    for name, pose in moves:
        state["state"] = f"scanning: {name}"
        arm.move(pose, speed=SCAN_SPEED)
        end = time.time() + SCAN_PAUSE
        while time.time() < end and not state["code"]:
            time.sleep(0.05)
        if state["code"]:
            code = state["code"]
            break
    state["state"] = "returning"
    arm.move({j: start[j] for j in MOVE_JOINTS}, speed=SCAN_SPEED)
    deposit = (code in load_json(DEPOSIT_FILE, [])) if code else None
    return {"code": code, "deposit": deposit, "time": round(time.time() - t0, 1)}


def run_task(arm, function, name):
    """Run an automatic move in the background (manual control paused). False if something is already running."""
    if _busy.is_set():
        return False
    _busy.set()
    _scan_done.clear()

    def run():
        try:
            result = function(arm)
        except SO101Error as e:
            result = {"error": str(e)}
        if name == "scan":
            state["result"] = result
            print("RESULT:", result)
        state["state"] = "waiting"
        _busy.clear()
        _scan_done.set()

    threading.Thread(target=run, daemon=True).start()
    return True


# ----------------------------------------------------------------------------- browser panel (no window)
# jpg = preview with overlays (panel), clean = only the bottle boxes and the crosshair (the /demo presentation view)
_web = {"jpg": None, "clean": None, "nr_jpg": 0, "nr_clean": 0, "viewers": 0, "viewers_clean": 0, "tracking": False,
        "detections": [],
        "yolo": "local"}
_web_new = threading.Condition()  # new preview frame
_web_keys = queue.SimpleQueue()  # keys from the browser -> main loop

PANEL_HTML = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>SO-101 Arm</title><style>
body{margin:0;background:#111;color:#eee;font:15px system-ui,sans-serif}
main{display:grid;grid-template-columns:minmax(0,3fr) minmax(260px,1fr);gap:12px;padding:12px}
@media(max-width:800px){main{grid-template-columns:1fr}}
img{width:100%;background:#000;border-radius:6px}
#result{font-size:22px;font-weight:700;padding:10px;border-radius:6px;background:#333;margin:0 0 10px}
.k{background:#1e6b1e}.b{background:#8a1c1c}.o{background:#a35c00}
.p{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:0 0 10px}
button{padding:10px 4px;font:inherit;border:0;border-radius:6px;background:#2d2d2d;color:#eee;touch-action:none}
button:active,button.on{background:#0a6ebd}#stop{background:#8a1c1c}
h3{margin:12px 0 6px;font-size:13px;color:#aaa;text-transform:uppercase}
label{display:grid;grid-template-columns:1fr 48px;font-size:13px;margin:2px 0}input{grid-column:1/3}
small{color:#888}</style></head><body><main><div><img src="/preview" alt="camera preview">
<small>The keyboard works like in the window: W/S R/F A/D J/L U/O, 1/2/3, B stop, T tracking, G basket/bottle, M, H, Y, Enter</small>
<p><a href="/roarm_panel" style="color:#4cc2ff">RoArm control (grip teaching, bottle collecting) &rarr;</a></p></div>
<div><div id="result">...</div><div class="p"><button id="rot">Rotated</button><button data-k="t" id="tr">Tracking</button>
<button data-k="g" id="bk">G follow basket</button>
<button data-k="b" id="stop">STOP</button></div>
<h3>Move (hold)</h3><div class="p">
<button data-h="j">J tilt</button><button data-h="w">W up</button><button data-h="l">L tilt</button>
<button data-h="a">A left</button><button data-h="s">S down</button><button data-h="d">D right</button>
<button data-h="r">R farther</button><button data-h="f">F closer</button><span></span>
<button data-h="u">U roll</button><span></span><button data-h="o">O roll</button></div>
<div class="p"><button data-k="1">slow</button><button data-k="2">medium</button><button data-k="3">fast</button></div>
<h3>Poses</h3><div class="p"><button data-k="m">M look here (table)</button><button data-k="h">H home</button>
<button data-k="y">Y history</button></div><h3>Settings</h3><div id="sliders"></div></div></main><script>
const k=c=>fetch('/key?k='+encodeURIComponent(c));let held={};
function press(c){if(held[c])return;k(c);held[c]=setInterval(()=>k(c),1000/30)}
function release(c){clearInterval(held[c]);delete held[c]}
function releaseAll(){Object.keys(held).forEach(release)}
document.querySelectorAll('[data-h]').forEach(b=>{const c=b.dataset.h;
 b.onpointerdown=e=>{b.setPointerCapture(e.pointerId);press(c)};b.onpointerup=b.onpointercancel=()=>release(c)});
document.querySelectorAll('[data-k]').forEach(b=>b.onclick=()=>k(b.dataset.k));
document.getElementById('rot').onclick=()=>fetch('/rotated');
const MOVE='wsrfadjluo',ACTIONS='123btmhycpg';
onkeydown=e=>{if(e.target.tagName=='INPUT')return;const c=e.key.toLowerCase();
 if(MOVE.includes(c)&&c.length==1){press(c);e.preventDefault()}
 else if(ACTIONS.includes(c)&&c.length==1&&!e.repeat)k(c);else if(e.key=='Enter'&&!e.repeat)k('\\r')};
onkeyup=e=>release(e.key.toLowerCase());onblur=releaseAll;
fetch('/settings').then(r=>r.json()).then(s=>{const d=document.getElementById('sliders');s.forEach((u,i)=>{
 const l=document.createElement('label');l.innerHTML=`<span>${u.label}</span><b>${u.v}</b>
 <input type=range min=${u.lo} max=${u.hi} value=${u.v}>`;const r=l.querySelector('input');
 r.oninput=()=>l.querySelector('b').textContent=r.value;r.onchange=()=>fetch(`/set?i=${i}&v=${r.value}`);d.append(l)})});
async function refresh(){try{const w=await(await fetch('/result')).json(),e=document.getElementById('result');
 const o=w.state=='RESULT'?w:(w.last_result||{});const t={DEPOSIT:'k',NO_DEPOSIT:'b',NO_CODE:'o'};
 e.className=w.state=='RESULT'?(t[w.result]||''):'';
 e.textContent=w.state=='RESULT'?`${w.result}${w.amount!=null?' '+w.amount.toFixed(2)+' '+w.currency:''} ${w.name||w.code||''}`
  :`${w.state}${o.result?' | last: '+o.result+' '+(o.code||''):''}`;
 document.getElementById('tr').classList.toggle('on',!!w.tracking);
 document.getElementById('bk').classList.toggle('on',w.track_object==='basket')}catch(e){}setTimeout(refresh,700)}refresh();
</script></body></html>"""


def _set_slider(i, v):
    """Slider no. i (from the window or the browser) -> setting in the program."""
    label, variable, lo, hi, mul = SLIDERS[i]
    v = min(max(int(v), lo), hi)
    if variable is None:
        _web_keys.put("track1" if v else "track0")
    else:
        globals()[variable] = bool(v) if variable in ("TRACK_DEPOSIT_ONLY", "TRACK_SMOOTH") else v * mul


def _slider_state():
    out = []
    for label, variable, lo, hi, mul in SLIDERS:
        v = int(_web["tracking"]) if variable is None else int(round(float(globals()[variable]) / mul))
        out.append({"label": label, "lo": lo, "hi": hi, "v": min(max(v, lo), hi)})
    return out


def web_frame(screen, clean=False):
    """Hand a frame to the browser preview (only when someone is watching - JPEG costs CPU)."""
    import cv2

    if screen.shape[1] > WEB_WIDTH:  # less data over WiFi = smoother preview
        screen = cv2.resize(screen, (WEB_WIDTH, screen.shape[0] * WEB_WIDTH // screen.shape[1]), interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode(".jpg", screen, [cv2.IMWRITE_JPEG_QUALITY, 65])
    if ok:
        key = "clean" if clean else "jpg"
        with _web_new:  # a separate counter per stream - otherwise each would also send the other's frames (2x WiFi)
            _web[key], _web["nr_" + key] = jpg.tobytes(), _web.get("nr_" + key, 0) + 1
            _web_new.notify_all()


_sys = {"cpu": None, "throttle": None, "throttle_t": 0.0}


def _system_status():
    """The computer running so101_station.py (Raspberry): CPU %, RAM, temperature, clock, throttling + SO-101 servos."""
    import subprocess

    out = {"so101": _web.get("so101")}
    try:
        with open("/proc/stat") as f:
            v = [int(x) for x in f.readline().split()[1:]]
        idle, total = v[3] + v[4], sum(v)
        prev, _sys["cpu"] = _sys["cpu"], (idle, total)
        if prev and total > prev[1]:
            out["cpu"] = round(100 * (1 - (idle - prev[0]) / (total - prev[1])), 1)
        mem = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, w = line.split(":", 1)
                mem[k] = int(w.split()[0])
        out["ram"] = round(100 * (1 - mem["MemAvailable"] / mem["MemTotal"]), 1)
        out["ram_mb"], out["ram_total_mb"] = round((mem["MemTotal"] - mem["MemAvailable"]) / 1024), round(mem["MemTotal"] / 1024)
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            out["temp"] = round(int(f.read()) / 1000, 1)
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            out["clock_mhz"] = round(int(f.read()) / 1000)
        if time.time() - _sys["throttle_t"] > 5:  # vcgencmd only every 5 s
            _sys["throttle_t"] = time.time()
            r = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2)
            _sys["throttle"] = int(r.stdout.strip().split("=")[1], 16)
    except (OSError, ValueError, IndexError, KeyError, subprocess.SubprocessError):
        try:  # not a Raspberry (e.g. a Windows laptop)
            import psutil

            out.update(cpu=psutil.cpu_percent(None), ram=psutil.virtual_memory().percent)
        except ImportError:
            pass
    if _sys["throttle"] is not None:
        out["throttled"] = bool(_sys["throttle"] & 0x6)          # now: clock capped / throttled
        out["undervoltage"] = bool(_sys["throttle"] & 0x1)       # the Pi's power supply is too weak
    return out


def _bottle_data():
    """For the RoArm (roarm_pick.py): bottles in the frame (full-frame pixels) + whether the camera is in the look pose.

    Look pose = the pose saved with key M (look). Only from it can the image be mapped onto the table.
    """
    view = dict(_web.get("view") or {"bottles": [], "width": 0, "height": 0})
    joints = _web.get("joints") or {}
    pose = load_json(POSES_FILE, {}).get(HOME_POSE)
    view["joints"] = joints
    view["look_saved"] = bool(pose)
    view["at_look"] = bool(pose and joints and all(abs(joints[j] - pose[j]) < 4.0 for j in MOVE_JOINTS))
    view["still"] = bool(joints) and time.time() - _web.get("moved_t", 0.0) > 0.5 and not _busy.is_set()
    view["tracking"] = _web["tracking"]
    # how many px the image shifts per 1 deg of a joint - roarm_pick.py uses it to fix small errors in returning to the pose
    view["px_per_deg"] = {TRACK_JOINT_H: TRACK_SIGN_H * TRACK_SENSITIVITY_H * view.get("width", 0) / 2,
                          TRACK_JOINT_V: TRACK_SIGN_V * TRACK_SENSITIVITY_V * view.get("height", 0) / 2}
    return view


def _demo_data():
    """Everything for the /demo view: stage, result, what the network sees, history and bottle counters."""
    results = list(state.get("results", {}).values())  # the last result of each bottle
    deposit = [r for r in results if r["result"] == "DEPOSIT"]
    return {
        "inspection": state.get("inspection") or {"state": "SEARCHING"},
        "last_result": state.get("last_result"),
        "history": state.get("history", [])[-12:][::-1],
        "counter": {"bottles": len(results), "deposit": len(deposit),
                    "no_deposit": sum(r["result"] == "NO_DEPOSIT" for r in results),
                    "total": round(sum(r["amount"] or 0 for r in deposit), 2)},
        "detections": _web["detections"], "tracking": _web["tracking"], "fps": state.get("fps", 0),
        "yolo": _web.get("yolo", "local"), "yolo_fps": _remote["fps"], "laptop": yolo_laptop_active(),
        "roarm": (_web.get("roarm") or {}).get("text") if time.time() - (_web.get("roarm") or {}).get("t", 0) < 90
        else None,
    }


def http_server(arm):
    class Handler(BaseHTTPRequestHandler):
        # the connection stays open between requests (YOLO laptop: no new TCP per frame),
        # and small packets go out immediately (Nagle + Windows delayed ACK = up to 200 ms per answer)
        protocol_version = "HTTP/1.1"
        disable_nagle_algorithm = True

        def _json(self, data, code=200):
            body = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _preview(self, clean=False, fps=None):
            """MJPEG: the browser shows it like a video (<img src="/preview">). fps = at most this many frames/s to
            this viewer (the RoArm panel: fewer frames leave WiFi room for its move requests)."""
            self.close_connection = True  # endless stream - the connection closes after it
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            counter = "viewers_clean" if clean else "viewers"
            key = "clean" if clean else "jpg"
            _web[counter] += 1
            nr, t_sent = -1, 0.0
            try:
                while True:
                    if fps:
                        time.sleep(max(0.0, t_sent + 1.0 / fps - time.time()))
                        t_sent = time.time()
                    with _web_new:
                        _web_new.wait_for(lambda: _web.get("nr_" + key, 0) != nr, timeout=2)
                        jpg, nr = _web[key], _web.get("nr_" + key, 0)
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                         % len(jpg) + jpg + b"\r\n")
            except OSError:  # browser closed
                pass
            finally:
                _web[counter] -= 1

        def do_GET(self):
            url = urlsplit(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            # always read the body - on a kept-alive connection it would otherwise be taken as "the next request"
            length = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(length) if length > 0 else b""
            if url.path == "/":
                page = PANEL_HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif url.path == "/preview":
                try:
                    fps = min(max(float(q.get("fps", 0)), 1.0), 30.0) if q.get("fps") else None
                except ValueError:
                    fps = None
                self._preview(clean="clean" in q, fps=fps)
            elif url.path == "/frame":  # one fresh frame at full resolution (calibration: small tag)
                import cv2

                raw = _web.get("raw")
                if raw is None:
                    self.send_error(503)
                    return
                jpg = cv2.imencode(".jpg", raw, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
            elif url.path == "/demo":  # presentation view (the file is read every time - it can be edited live)
                try:
                    with open(os.path.join(_dir, "demo.html"), "rb") as f:
                        page = f.read()
                except FileNotFoundError:
                    return self._json({"error": "demo.html missing next to so101_station.py"}, 404)
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif url.path == "/data":
                self._json(_demo_data())
            elif url.path == "/system":  # CPU, RAM, Pi temperature + SO-101 servo temperatures
                self._json(_system_status())
            elif url.path == "/bottles":  # RoArm: where the bottles stand (pixels) + whether the camera is in the look pose
                self._json(_bottle_data())
            elif url.path == "/tracking" and q.get("on") in ("0", "1"):  # RoArm turns tracking on/off
                _web_keys.put("track" + q["on"])
                self._json({"ok": True})
            elif url.path == "/track_object" and q.get("name") in ("bottle", "basket"):  # what the camera follows
                _web_keys.put("object:" + q["name"])
                self._json({"ok": True})
            elif url.path == "/look":  # SO-101 goes back to the pose looking at the table (pose from key M)
                pose, joints = load_json(POSES_FILE, {}).get(HOME_POSE), _web.get("joints") or {}
                if not pose:
                    return self._json({"error": "no look pose - aim the camera at the table and press M"}, 409)
                # tracking moves only the base and the wrist - a big difference in lift/elbow = an old pose from another
                # arm setup; a big move could hit the RoArm
                if not joints:
                    return self._json({"error": "don't know yet where the arm is - try again in a moment"}, 409)
                far = [j for j in ("lift", "elbow") if abs(joints[j] - pose[j]) > 15]
                if far and q.get("force") != "1":
                    return self._json({"error": f"look pose far from the current one ({', '.join(far)}) - "
                                                "aim the camera at the table and press M in the panel"}, 409)
                _web_keys.put("h")
                self._json({"ok": True})
            elif url.path == "/roarm_panel" or url.path.startswith("/roarm/"):  # RoArm control
                import roarm_panel

                roarm_panel.handle(self, url.path, q)
            elif url.path == "/roarm_status":  # RoArm caption in the /demo view
                _web["roarm"] = {"text": q.get("text", "")[:120], "t": time.time()}
                self._json({"ok": True})
            elif url.path == "/yolo":  # laptop: YOLO result of the previous frame -> the next frame in the answer
                try:
                    result_nr = int(q.get("nr", 0))
                except ValueError:
                    result_nr = 0
                _yolo_from_laptop(body, result_nr)
                jpg, nr = _yolo_frame()
                if jpg is None:
                    self.send_response(204)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.send_header("X-Nr", str(nr))
                self.end_headers()
                self.wfile.write(jpg)
            elif url.path == "/key" and q.get("k"):
                _web_keys.put(q["k"][:1].lower())
                self._json({"ok": True})
            elif url.path == "/settings":
                self._json(_slider_state())
            elif url.path == "/set" and "i" in q and "v" in q:
                try:
                    _set_slider(int(q["i"]), q["v"])
                except (ValueError, IndexError):
                    return self._json({"error": "bad slider"}, 400)
                self._json({"ok": True})
            elif self.path.startswith("/scan"):
                print("\nSIGNAL from the network")
                if not run_task(arm, inspect_can, "scan"):
                    return self._json({"error": "arm busy"}, 409)
                _scan_done.wait(timeout=120)
                self._json(state["result"] or {"error": "timeout"})
            elif self.path.startswith("/status"):
                self._json(state)
            elif self.path.startswith("/result"):  # bottle inspection result (deposit / none / rotate)
                self._json(dict(state.get("inspection") or {"state": "no inspection"},
                                last_result=state.get("last_result"), tracking=_web["tracking"],
                                track_object=TRACK_OBJECT))
            elif self.path.startswith("/rotated"):  # bottle rotated - read the code again
                if _inspection:
                    _inspection.after_rotation()
                self._json({"ok": True})
            else:
                self._json({"error": "use /scan, /status, /result or /rotated"}, 404)

        do_POST = do_GET

        def log_message(self, *a):
            pass

    class Server(ThreadingHTTPServer):
        daemon_threads = True

        def handle_error(self, request, client_address):
            if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
                return  # the browser or laptop disconnected midway - normal, no log spam
            super().handle_error(request, client_address)

    srv = Server(("0.0.0.0", HTTP_PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def _ean_ok(ean):
    """EAN-8/EAN-13 checksum - rejects wrong reads."""
    if not ean.isdigit() or len(ean) not in (8, 13):
        return False
    d = [int(c) for c in ean]
    weights = [3, 1] * (len(d) // 2) if len(d) == 8 else [1, 3] * 6
    return (10 - sum(w * x for w, x in zip(weights, d[:-1])) % 10) % 10 == d[-1]


def find_codes(frame):
    """All EAN codes in the frame: [{"ean", "cx", "cy", "size", "corners"}] (size = fraction of the image width)."""
    import cv2
    import numpy as np

    width = frame.shape[1]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = []
    try:
        import zxingcpp

        raw = zxingcpp.read_barcodes(gray)
        if not raw:  # cans with glare: more contrast
            raw = zxingcpp.read_barcodes(cv2.createCLAHE(2.0, (8, 8)).apply(gray))
        for r in raw:
            p = r.position
            corners = np.array([[p.top_left.x, p.top_left.y], [p.top_right.x, p.top_right.y],
                                [p.bottom_right.x, p.bottom_right.y], [p.bottom_left.x, p.bottom_left.y]])
            found.append((r.text, corners))
    except ImportError:  # fallback: the reader built into OpenCV
        ok, texts, _t, pts = cv2.barcode.BarcodeDetector().detectAndDecodeWithType(frame)
        if ok and pts is not None:
            found = [(t, r) for t, r in zip(texts, pts) if t]
    codes = []

    def add(ean, corners):
        xs, ys = corners[:, 0], corners[:, 1]
        codes.append({"ean": ean, "cx": float(xs.mean()), "cy": float(ys.mean()),
                      "size": float(max(xs.max() - xs.min(), ys.max() - ys.min())) / width,
                      "corners": corners.astype(int).reshape(-1, 1, 2)})

    for ean, corners in found:
        if _ean_ok(ean):
            add(ean, corners)
    # Blurred code: the digits can't be read, but the code's rectangle is visible -> ean=None (for tracking)
    global _barcode_detector
    if _barcode_detector is None:
        _barcode_detector = cv2.barcode.BarcodeDetector()
    ok, pts = _barcode_detector.detect(frame)
    if ok and pts is not None:
        for corners in pts:
            cx, cy = corners[:, 0].mean(), corners[:, 1].mean()
            if all(abs(cx - k["cx"]) + abs(cy - k["cy"]) > 0.05 * width for k in codes):
                add(None, corners)
    return codes


_barcode_detector = None


def sharpness(frame, whole=False):
    """Sharpness of the image center (or the whole crop) - variance of the Laplacian. A code is usually readable from ~100."""
    import cv2

    h, w = frame.shape[:2]
    piece = frame if whole else frame[h // 4:3 * h // 4, w // 4:3 * w // 4]
    center = cv2.cvtColor(piece, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(center, cv2.CV_64F).var()


def read_skewed(gray, corners):
    """Code visible but not read: straighten it (bars vertical), enlarge, sharpen, try several thresholds."""
    import cv2
    import numpy as np

    try:
        import zxingcpp
    except ImportError:
        return None
    (cx, cy), (rw, rh), angle = cv2.minAreaRect(corners.astype(np.float32))
    M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
    rot = cv2.warpAffine(gray, M, (gray.shape[1], gray.shape[0]), flags=cv2.INTER_CUBIC)
    w2, h2 = rw * 1.3, rh * 1.3
    crop = rot[max(0, int(cy - h2 / 2)):int(cy + h2 / 2), max(0, int(cx - w2 / 2)):int(cx + w2 / 2)]
    if crop.size == 0:
        return None
    formats = [zxingcpp.BarcodeFormat.EAN13, zxingcpp.BarcodeFormat.EAN8]
    for scale in (2, 3):
        big = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        for image in (big, cv2.addWeighted(big, 1.8, cv2.GaussianBlur(big, (0, 0), 2), -0.8, 0)):
            for binarizer in (zxingcpp.Binarizer.LocalAverage, zxingcpp.Binarizer.GlobalHistogram):
                for w in zxingcpp.read_barcodes(image, formats=formats, try_rotate=True, binarizer=binarizer):
                    if _ean_ok(w.text):
                        return w.text
    return None


class CodeReader:
    """Reads barcodes in a separate thread - tracking doesn't wait for the slow reading (steady rhythm).

    Each frame: the whole image + an enlarged crop of the tracked bottle (a small code is easier to read).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._target = None
        self.codes = []          # last codes from the whole image
        self.target_code = {}    # tracked bottle id -> read EAN
        self._votes = {}         # bottle id -> {EAN: how many times read}
        self.confirmed = set()   # confirmed EANs (CODE_CONFIRM matching reads) - these need to be read only once
        self._stop = False
        self.paused = False      # True = read nothing (the result is already decided)
        self._last = 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def submit(self, frame, target_id=None, box=None):
        with self._lock:
            self._frame = frame
            self._target = (target_id, box) if target_id and box else None

    def wants_frame(self):
        """Whether the reader waits for a new frame (we copy it only then - saves loop time)."""
        return self._frame is None and not self.paused and time.time() - self._last >= 1.0 / READS_PER_SECOND

    def stop(self):
        self._stop = True

    def _loop(self):
        import cv2

        while not self._stop:
            with self._lock:
                frame, target = self._frame, self._target
                self._frame = None
            if frame is None:
                time.sleep(0.01)
                continue
            self._last = time.time()
            try:
                if not target:  # no target: the whole image (e.g. the can circuit on ENTER)
                    self.codes = find_codes(frame)
                    continue
                self.codes = []
                tid, (x0, y0, x1, y1) = target
                ean = None
                if ean is None:  # just the bottle crop (enlarged when small) - much faster than the whole 1080p image
                    h, w = frame.shape[:2]
                    mx, my = (x1 - x0) * 0.15, (y1 - y0) * 0.15
                    crop = frame[max(0, int(y0 - my)):min(h, int(y1 + my)), max(0, int(x0 - mx)):min(w, int(x1 + mx))]
                    if crop.size:
                        if max(crop.shape[:2]) < 700:
                            crop = cv2.resize(crop, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
                        crop_codes = find_codes(crop)
                        ean = next((k["ean"] for k in crop_codes if k["ean"]), None)
                        if ean is None:  # skewed code in the crop: straighten it and read harder
                            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                            for k in crop_codes:
                                ean = read_skewed(gray, k["corners"].reshape(-1, 2))
                                if ean:
                                    break
                if ean:
                    # a single read can be wrong (swapped digits and the checksum happens to match -
                    # that gave "0415191856075" instead of 8445291856035): accept only matching reads
                    if len(self._votes) > 200:
                        self._votes.clear()
                    votes = self._votes.setdefault(tid, {})
                    votes[ean] = votes.get(ean, 0) + 1
                    if votes[ean] >= CODE_CONFIRM or ean in self.confirmed:
                        self.confirmed.add(ean)
                        self.target_code[tid] = ean
            except Exception as e:  # the reader must not crash the whole program
                print("\ncode reader:", e)


class Deposits:
    """Whether an EAN has a deposit: deposit_list.json or api.kaucja.pl (in the background, cached)."""

    def __init__(self):
        self.status = {}   # ean -> True / False / None (checking)
        self.names = {}
        self.amounts = {}  # ean -> (amount, currency)

    def check(self, ean):
        if ean in self.status:
            return self.status[ean]
        if ean in load_json(DEPOSIT_FILE, []):
            self.status[ean] = True
            return True
        self.status[ean] = None
        threading.Thread(target=self._api, args=(ean,), daemon=True).start()
        return None

    def _api(self, ean):
        try:
            import requests

            r = requests.get(f"https://api.kaucja.pl/buf/pos/product/{ean}", timeout=5)
            data = r.json() if r.status_code == 200 else {}
            self.status[ean] = data.get("deposit") is not None
            self.names[ean] = data.get("publishedName") or data.get("name") or ""
            deposit = data.get("deposit") or {}
            if deposit.get("amount") is not None:
                self.amounts[ean] = (deposit.get("amount"), deposit.get("currency") or "PLN")
        except Exception:
            self.status[ean] = False  # no network / error -> treat as no deposit
        print(f"\n{ean}: {'DEPOSIT' if self.status[ean] else 'no deposit'} {self.names.get(ean, '')}")


_log_queue = None


def _write_log(q):
    with open(os.path.join(_dir, TRACK_LOG), "a", encoding="utf-8") as f:
        while True:
            f.write(q.get())
            if q.empty():
                f.flush()


def _log(text):
    """Write to the log in the background (opening the file on every entry took ~200 ms and stalled the program)."""
    global _log_queue
    if not TRACK_LOG:
        return
    if _log_queue is None:
        import queue

        _log_queue = queue.Queue()
        threading.Thread(target=_write_log, args=(_log_queue,), daemon=True).start()
    now = time.time()
    _log_queue.put(f"{time.strftime('%H:%M:%S', time.localtime(now))}.{int(now * 1000) % 1000:03d} {text}\n")


class Tracker:
    """Keeps a deposit code in the middle of the image. Lets go when too far or lost;
    switches to another deposit bottle if it is clearly closer."""

    def __init__(self):
        self.target = None       # ean of the tracked bottle
        self.size = 0.0
        self.seen = 0.0
        self.xy = None           # last code position (px)
        self._reset()

    def step(self, codes, deposits, width, height, st, dt):
        """Updates the target and moves st.target. Returns a message or None."""
        now = time.time()
        deposit = [k for k in codes if k["ean"] and deposits.check(k["ean"]) and k["size"] >= MIN_CODE_WIDTH]
        nearest = max(deposit, key=lambda k: k["size"], default=None)
        msg = None
        current = next((k for k in codes if self.target and k["ean"] == self.target), None)
        if current is None and self.target and self.xy:
            # the digits aren't visible (blur) - take the nearest unread code rectangle
            near = [k for k in codes if k["ean"] is None and
                    abs(k["cx"] - self.xy[0]) + abs(k["cy"] - self.xy[1]) < TRACK_RADIUS * width]
            current = min(near, key=lambda k: abs(k["cx"] - self.xy[0]) + abs(k["cy"] - self.xy[1]),
                          default=None)

        if self.target is None:
            if nearest:
                self.target = nearest["ean"]
                current = nearest
                msg = f"TRACKING {self.target}"
        elif current is not None:
            if current["size"] < MIN_CODE_WIDTH:
                msg, self.target, current = f"{self.target} MOVED AWAY - letting go", None, None
            elif nearest and nearest["ean"] != self.target and \
                    nearest["size"] > current["size"] * SWITCH_WHEN:
                msg = f"ANOTHER CLOSER: {self.target} -> {nearest['ean']}"
                self.target, current = nearest["ean"], nearest
        elif nearest:  # the target vanished, but there's another deposit bottle
            msg = f"ANOTHER BOTTLE: {self.target} -> {nearest['ean']}"
            self.target, current = nearest["ean"], nearest
        elif now - self.seen > LOST_AFTER:
            msg, self.target = f"LOST {self.target} - letting go", None

        if current is None:
            if self.target and not msg:
                self._search(st, now)
            return msg
        self.seen, self.size = now, current["size"]
        self.source = "code" if current["ean"] else ("bottle" if current.get("bottle") else "rectangle")
        self.xy = (current["cx"], current["cy"])
        self.error = ((current["cx"] - width / 2) / (width / 2), (current["cy"] - height / 2) / (height / 2),
                      current["ean"] is not None)
        if msg:  # new target - start over
            self._reset()
        # code position history (last 0.6 s) - to predict where it's escaping
        self._hist = [h for h in self._hist if now - h[0] < 0.6] + [(now, self.error[0], self.error[1])]
        self._searched = 0.0
        return self._steer(st, msg, now)

    def _search(self, st, now):
        """Code momentarily invisible: keep moving where it was escaping (limited time and angle)."""
        if TRACK_SMOOTH:
            if hasattr(self, "g"):
                self._search_smooth(st, now)
            return
        if not self._hist or now < self._next or not hasattr(self, "g"):
            return
        since_loss = now - self.seen
        if since_loss < TRACK_SEARCH_AFTER:  # momentary missing read - stand still, don't wave
            return
        if since_loss > TRACK_SEARCH_TIME or self._searched >= TRACK_SEARCH_MAX:
            return
        (t0, x0, y0), (t1, x1, y1) = self._hist[0], self._hist[-1]
        # speed only from a trustworthy history - otherwise detection noise pretends the bottle moves
        if len(self._hist) >= 3 and t1 - t0 >= 0.3:
            vx, vy = (x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0)
        else:
            vx = vy = 0.0
        horizon = min(since_loss + TRACK_PAUSE, 1.0)
        pred = {"x": max(-1.5, min(1.5, x1 + vx * horizon)), "y": max(-1.5, min(1.5, y1 + vy * horizon))}
        if max(abs(pred["x"]), abs(pred["y"])) < TRACK_START:
            return  # vanished near the center = a reading problem, not an escape - stand still
        joints = {"x": TRACK_JOINT_H, "y": TRACK_JOINT_V}
        left = TRACK_SEARCH_MAX - self._searched
        move = {}
        for o, joint in joints.items():
            if abs(pred[o]) > TRACK_TARGET_ZONE:
                d = -TRACK_STEP * pred[o] / self.g[o]
                d = max(-min(TRACK_MAX_STEP, left), min(min(TRACK_MAX_STEP, left), d))
                if abs(d) >= TRACK_MIN_STEP:
                    st.target[joint] += d
                    st.slow.add(joint)
                    move[joint] = round(d, 1)
        if move:
            self._searched += max(abs(d) for d in move.values())
            self._next = now + TRACK_PAUSE
            self._on_target = False
            self._last_move = None  # don't learn sensitivity from blind moves
            self._samples = []
            _log(f"SEARCHING target={self.target} predicted x={pred['x']:+.2f} y={pred['y']:+.2f} move={move} "
                 f"total {self._searched:.0f} deg")

    def _reset(self):
        self._samples = []         # measurements from a still image after the last move
        self._last_move = None     # (error_x, error_y, move_x, move_y) - for sensitivity learning
        self._on_target = False    # centered -> stands still until the code clearly escapes
        self._next = 0.0
        self._prev = None
        self._hist = []            # (time, error_x, error_y) - to predict the code's movement
        self._filt = None          # smoothed error (x, y) - smooth mode
        self.code_rel = None       # where the code is on the bottle (relative to the box) - we aim there
        self._moving = {"x": False, "y": False}  # smooth mode hysteresis
        self._cut = {"top": False, "bottom": False, "left": False, "right": False}
        self._v = {"x": 0.0, "y": 0.0}  # current joint speed (deg/s) - smooth mode
        self._t_last = None
        self._searched = 0.0       # how many deg it moved "blind" since losing the code

    def _dt(self, now):
        dt = 0.05 if self._t_last is None else min(max(now - self._t_last, 0.0), 0.2)
        self._t_last = now
        return dt

    def _latency(self):
        """How many seconds old is the arm movement visible in the image (known frame age -> exact)."""
        return getattr(self, "_lat", None) or TRACK_DELAY

    def measure_latency(self, frame_t, now):
        """Age of the current frame + the fixed servo and camera delay (smoothed - no jumps)."""
        o = CAMERA_DELAY + max(0.0, now - frame_t)
        old = getattr(self, "_lat", None)
        self._lat = o if old is None else old + 0.2 * (o - old)

    def _drive(self, st, dt):
        """Move the joint goals by speed * dt; the servo gets a speed matching the movement."""
        joints = {"x": TRACK_JOINT_H, "y": TRACK_JOINT_V}
        move = 0.0
        for o, joint in joints.items():
            w = self._v[o]
            if abs(w) < 0.05:
                if abs(getattr(self, "_v_prev", {}).get(o, 0.0)) >= 0.05:
                    st.slow.add(joint)
                    st.track_speed[joint] = 0.0  # just stopped: send the goal without lead
                continue
            st.target[joint] += w * dt
            st.slow.add(joint)
            st.track_speed[joint] = w
            move = max(move, abs(w * dt))
        self._v_prev = dict(self._v)
        return move

    def _steer_smooth(self, st, msg, now):
        """Smooth tracking: speed proportional to the (smoothed) distance of the target from the center."""
        dt = self._dt(now)
        ex, ey = self.error[:2]
        if msg or self._filt is None:
            self._filt = (ex, ey)
            self._ff = {"x": 0.0, "y": 0.0}
            self._hist_v = []
            self._prev_f = None
        a = TRACK_FILTER
        self._filt = (self._filt[0] + a * (ex - self._filt[0]), self._filt[1] + a * (ey - self._filt[1]))
        # speed of the bottle itself in the image = image change minus what our own movement did
        # (the camera shows our movement with a TRACK_DELAY delay)
        if self._prev_f is not None and dt > 0:
            v_then = next((w for t, w in reversed(self._hist_v) if t <= now - self._latency()),
                          {"x": 0.0, "y": 0.0})
            for i, o in enumerate(("x", "y")):
                v_bottle = (self._filt[i] - self._prev_f[i]) / dt - self.g[o] * v_then[o]
                self._ff[o] += TRACK_FF_SMOOTHING * (-v_bottle / self.g[o] - self._ff[o])
        self._prev_f = self._filt
        for i, o in enumerate(("x", "y")):
            e = self._filt[i]
            bottle_moves = abs(self._ff[o]) > TRACK_FF_THRESHOLD  # the bottle is really moving
            # hysteresis: a stationary arm starts only on a clear drift, a moving one goes all the way to the center
            threshold = TRACK_TARGET_ZONE if (self._moving[o] or bottle_moves) else TRACK_START_SMOOTH
            excess = max(0.0, abs(e) - TRACK_TARGET_ZONE) / (1.0 - TRACK_TARGET_ZONE) if abs(e) > threshold else 0.0
            w = -math.copysign(excess, e) * TRACK_LOOP_GAIN / self.g[o]
            if bottle_moves:  # the bottle is moving -> move along with it
                w += TRACK_FF * self._ff[o]
            self._moving[o] = abs(w) > 0.05
            cut = getattr(self, "_cut", {})
            is_cut = (cut.get("top") or cut.get("bottom")) if o == "y" else (cut.get("left") or cut.get("right"))
            v_max = CUT_MAX_SPEED if is_cut else TRACK_MAX_SPEED  # the target is only an estimate -> more carefully
            w = max(-v_max, min(v_max, w))
            dw = TRACK_ACCEL * dt  # speed changes gradually: smooth start and braking, no jerks
            self._v[o] += max(-dw, min(dw, w - self._v[o]))
        self._hist_v = [h for h in self._hist_v if now - h[0] < 1.0] + [(now, dict(self._v))]
        if TRACK_LEARN_SENSITIVITY:
            self._learn_sensitivity(st, now)
        self._drive(st, dt)
        if now - getattr(self, "_log_t", 0.0) > TRACK_LOG_EVERY:
            self._log_t = now
            _log(f"target={self.target} x={self._filt[0]:+.2f} y={self._filt[1]:+.2f} {getattr(self, 'source', '?')} "
                 f"pan={st.here['pan']:.1f} wflex={st.here['wflex']:.1f} "
                 f"speed pan={self._v['x']:+.1f} wflex={self._v['y']:+.1f} deg/s "
                 f"sensitivity x={self.g['x']:+.3f} y={self.g['y']:+.3f} latency={self._latency():.2f}")
        return msg

    def _learn_sensitivity(self, st, now):
        """Sensitivity = change of the target's position in the image / the joint move that caused it (the camera sees it late).

        It depends on the bottle's distance (close = small move, big image shift), so we learn it live.
        Measurements that don't fit (e.g. the bottle moved by itself) are rejected.
        """
        joints = {"x": TRACK_JOINT_H, "y": TRACK_JOINT_V}
        h = getattr(self, "_hist_g", [])
        cut = getattr(self, "_cut", {})
        is_cut = {"x": bool(cut.get("left") or cut.get("right")), "y": bool(cut.get("top") or cut.get("bottom"))}
        h = [x for x in h if now - x[0] < 2.0] + [(now, self._filt, {o: st.here[j] for o, j in joints.items()},
                                                   is_cut)]
        self._hist_g = h
        if now - getattr(self, "_g_t", 0.0) < 0.2:
            return
        self._g_t = now

        def entry(t):
            return next((x for x in reversed(h) if x[0] <= t), None)

        window, lat = 0.4, self._latency()
        now_e, old_e = h[-1], entry(now - window)
        now_j, old_j = entry(now - lat), entry(now - window - lat)
        if not (old_e and now_j and old_j):
            return
        for i, o in enumerate(("x", "y")):
            if any(x[3][o] for x in h if now - x[0] <= window + lat):
                continue  # the bottle was cut off during the measurement window - don't learn from it
            dth = now_j[2][o] - old_j[2][o]
            if abs(dth) < 0.8:
                continue
            g_obs = (now_e[1][i] - old_e[1][i]) / dth
            if g_obs * self.g[o] > 0 and 0.25 <= g_obs / self.g[o] <= 4.0:
                g = self.g[o] + 0.3 * (g_obs - self.g[o])
                start = TRACK_SENSITIVITY_H if o == "x" else TRACK_SENSITIVITY_V
                self.g[o] = math.copysign(min(max(abs(g), SENSITIVITY_RANGE[0] * start), SENSITIVITY_RANGE[1] * start), g)

    def _search_smooth(self, st, now):
        """Target momentarily invisible: if it was escaping - keep following it, otherwise brake smoothly."""
        dt = self._dt(now)
        since_loss = now - self.seen
        was_escaping = self._filt is not None and max(abs(self._filt[0]), abs(self._filt[1])) > TRACK_START
        if not (was_escaping and since_loss < TRACK_SEARCH_TIME and self._searched < TRACK_SEARCH_MAX):
            decay = math.exp(-dt / TRACK_BRAKE)
            self._v = {o: w * decay for o, w in self._v.items()}
        self._searched += self._drive(st, dt)

    def _steer(self, st, msg, now):
        """Correct and wait: median of several frames from a still image -> one move -> pause."""
        if not hasattr(self, "_samples"):
            self._reset()
        if not hasattr(self, "g"):  # sensitivity: how far the code shifts (fraction of half the image) per 1 deg of a joint
            self.g = {"x": TRACK_SIGN_H * TRACK_SENSITIVITY_H, "y": TRACK_SIGN_V * TRACK_SENSITIVITY_V}
        if TRACK_SMOOTH:
            return self._steer_smooth(st, msg, now)
        joints = {"x": TRACK_JOINT_H, "y": TRACK_JOINT_V}
        now_pos = {o: st.here[j] for o, j in joints.items()}
        arm_moves = self._prev and any(abs(now_pos[o] - self._prev[o]) > 0.3 for o in joints)
        self._prev = now_pos
        # wait until the pause passes and the servos stop - only then the image is current and sharp
        if now < self._next or arm_moves:
            self._samples = []
            return msg
        self._samples.append(self.error[:2])
        if len(self._samples) < TRACK_SAMPLES:
            return msg
        bx = sorted(p[0] for p in self._samples)[len(self._samples) // 2]
        by = sorted(p[1] for p in self._samples)[len(self._samples) // 2]
        self._samples = []
        error = {"x": bx, "y": by}

        # sensitivity learning from the previous move (rejects measurements where the bottle moved by itself)
        if self._last_move:
            e0, d0 = self._last_move
            for o in joints:
                if abs(d0[o]) >= 1.0:
                    g_obs = (error[o] - e0[o]) / d0[o]
                    if g_obs * self.g[o] > 0 and 0.3 <= g_obs / self.g[o] <= 3.0:
                        self.g[o] += 0.4 * (g_obs - self.g[o])
            self._last_move = None

        move = {}
        threshold = TRACK_START if self._on_target else TRACK_TARGET_ZONE
        for o, joint in joints.items():
            if abs(error[o]) > threshold:
                d = -TRACK_STEP * error[o] / self.g[o]
                d = max(-TRACK_MAX_STEP, min(TRACK_MAX_STEP, d))
                if abs(d) >= TRACK_MIN_STEP:
                    st.target[joint] += d
                    st.slow.add(joint)
                    move[o] = d
        self._on_target = not move
        if move:
            self._last_move = (error, {o: move.get(o, 0.0) for o in joints})
            self._next = now + TRACK_PAUSE
        _log(f"target={self.target} x={bx:+.2f} y={by:+.2f} {getattr(self, 'source', '?')} "
             f"pan={st.here['pan']:.1f} wflex={st.here['wflex']:.1f} "
             f"move={ {joints[o]: round(d, 1) for o, d in move.items()} } "
             f"sensitivity x={self.g['x']:+.3f} y={self.g['y']:+.3f}{' ON TARGET' if self._on_target else ''}")
        return msg


# ---- tracking history (author: Kakukoro) - bottle positions over time, saved to tracking_history.json
class TrackedItem:
    """Represents a single tracked can/bottle with its history."""

    def __init__(self, ean, first_seen=None, name=""):
        self.ean = ean
        self.name = name
        self.first_seen = first_seen or time.time()
        self.last_seen = time.time()
        self.positions = []
        self.status = "active"
        self.total_tracking_time = 0.0
        self.pickup_count = 0
        self.last_pickup_time = None
        self.average_position = None

    def add_position(self, timestamp, cx, cy, size, pan, wflex):
        """Add a new position observation."""
        self.positions.append((timestamp, cx, cy, size, pan, wflex))
        self.last_seen = timestamp
        cutoff = timestamp - 300
        self.positions = [p for p in self.positions if p[0] >= cutoff]

    def get_velocity(self):
        """Calculate current velocity based on recent positions."""
        if len(self.positions) < 2:
            return (0.0, 0.0)
        recent = self.positions[-5:]
        if len(recent) < 2:
            return (0.0, 0.0)
        t0, x0, y0, _, _, _ = recent[0]
        t1, x1, y1, _, _, _ = recent[-1]
        dt = t1 - t0
        if dt < 0.1:
            return (0.0, 0.0)
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        return (vx, vy)

    def predict_position(self, future_time):
        """Predict position at a future time based on velocity."""
        if not self.positions:
            return None
        last_time, last_x, last_y, _, _, _ = self.positions[-1]
        dt = future_time - last_time
        vx, vy = self.get_velocity()
        pred_x = last_x + vx * dt
        pred_y = last_y + vy * dt
        return (pred_x, pred_y)

    def to_dict(self):
        """Convert to dictionary for serialization."""
        return {
            "ean": self.ean,
            "name": self.name,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
            "status": self.status,
            "total_tracking_time": self.total_tracking_time,
            "pickup_count": self.pickup_count,
            "last_pickup_time": self.last_pickup_time,
            "positions": self.positions,
            "average_position": self.average_position
        }

    @classmethod
    def from_dict(cls, data):
        """Create from dictionary."""
        item = cls(data["ean"], data.get("first_seen"), data.get("name", ""))
        item.last_seen = data.get("last_seen", time.time())
        item.status = data.get("status", "active")
        item.total_tracking_time = data.get("total_tracking_time", 0.0)
        item.pickup_count = data.get("pickup_count", 0)
        item.last_pickup_time = data.get("last_pickup_time")
        item.positions = data.get("positions", [])
        item.average_position = data.get("average_position")
        return item


class TrackingHistory:
    """Manages tracking history for all detected cans/bottles."""

    def __init__(self, max_age=3600):
        self.items = {}
        self.max_age = max_age
        self.last_cleanup = time.time()

    def update(self, ean, name, timestamp, cx, cy, size, pan, wflex):
        """Update tracking data for a specific EAN."""
        if ean not in self.items:
            self.items[ean] = TrackedItem(ean, timestamp, name)
        else:
            if name and not self.items[ean].name:
                self.items[ean].name = name
        self.items[ean].add_position(timestamp, cx, cy, size, pan, wflex)
        return self.items[ean]

    def mark_pickup(self, ean):
        """Mark an item as picked up."""
        if ean in self.items:
            self.items[ean].pickup_count += 1
            self.items[ean].last_pickup_time = time.time()
            self.items[ean].status = "picked_up"

    def mark_lost(self, ean):
        """Mark an item as lost."""
        if ean in self.items:
            self.items[ean].status = "lost"

    def cleanup(self, current_time=None):
        """Remove old items that haven't been seen in a while."""
        if current_time is None:
            current_time = time.time()
        if current_time - self.last_cleanup < 60:
            return
        self.last_cleanup = current_time
        to_remove = [
            ean for ean, item in self.items.items()
            if current_time - item.last_seen > self.max_age and item.status != "picked_up"
        ]
        for ean in to_remove:
            del self.items[ean]

    def get_active_items(self):
        """Get all currently active (recently seen) items."""
        self.cleanup()
        return {ean: item for ean, item in self.items.items()
                if item.status == "active"}

    def get_item(self, ean):
        """Get a specific tracked item."""
        return self.items.get(ean)

    def predict_position(self, ean, future_time):
        """Predict where an item will be at a future time."""
        item = self.items.get(ean)
        if item:
            return item.predict_position(future_time)
        return None

    def save(self, filepath=None):
        """Save tracking history to a file."""
        filepath = filepath or os.path.join(_dir, "tracking_history.json")
        data = {
            "items": {ean: item.to_dict() for ean, item in self.items.items()},
            "last_cleanup": self.last_cleanup,
            "saved_at": time.time()
        }
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"Failed to save tracking history: {e}")
            return False

    def load(self, filepath=None):
        """Load tracking history from a file."""
        filepath = filepath or os.path.join(_dir, "tracking_history.json")
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.items = {}
            for ean, item_data in data.get("items", {}).items():
                self.items[ean] = TrackedItem.from_dict(item_data)
            self.last_cleanup = data.get("last_cleanup", time.time())
            return True
        except FileNotFoundError:
            return False
        except Exception as e:
            print(f"Failed to load tracking history: {e}")
            return False


MODEL_URL = "http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_coco_2018_03_29.tar.gz"
MODEL_PBTXT_URL = ("https://raw.githubusercontent.com/opencv/opencv_extra/4.x/testdata/dnn/"
                   "ssd_mobilenet_v2_coco_2018_03_29.pbtxt")


def download_model():
    """Download the bottle detection model to dorm_keeper/models (once, ~190 MB archive)."""
    import tarfile
    import urllib.request

    folder = os.path.join(_dir, "models")
    pb = os.path.join(folder, "ssd_mobilenet_v2_coco.pb")
    pbtxt = os.path.join(folder, "ssd_mobilenet_v2_coco.pbtxt")
    os.makedirs(folder, exist_ok=True)
    if not os.path.exists(pb):
        print("Downloading the bottle detection model (once, a few minutes)...")
        archive = os.path.join(folder, "ssd.tar.gz")
        urllib.request.urlretrieve(MODEL_URL, archive)
        with tarfile.open(archive) as tar:
            member = next(m for m in tar.getmembers() if m.name.endswith("frozen_inference_graph.pb"))
            with tar.extractfile(member) as src, open(pb, "wb") as dst:
                dst.write(src.read())
        os.remove(archive)
    if not os.path.exists(pbtxt):
        urllib.request.urlretrieve(MODEL_PBTXT_URL, pbtxt)
    return pb, pbtxt


YOLO_URL = "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"


def download_yolo():
    import urllib.request

    path = os.path.join(_dir, "models", "yolo11n.onnx")
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        print("Downloading the YOLO11n model (once, 11 MB)...")
        urllib.request.urlretrieve(YOLO_URL, path)
    return path


def yolo_opset21(path):
    """yolo11n.onnx has opset 22, and CUDA in onnxruntime has no Conv for it - the whole network would then run on the CPU
    (A100: 40 ms instead of 7 ms). One-time conversion to opset 21 (pip install onnx, only on the GPU machine)."""
    new = path.replace(".onnx", "_op21.onnx")
    if not os.path.exists(new):
        import onnx
        from onnx import version_converter

        onnx.save(version_converter.convert_version(onnx.load(path), 21), new)
    return new


# ----------------------------------------------------------------------------- YOLO computed on the laptop
# On the Raspberry YOLO gives ~10 fps. The laptop (yolo_laptop.py) connects here by itself: POST /yolo with the result of
# the previous frame, and gets the next (downscaled) one in the answer. The laptop listens on nothing = the Windows firewall
# doesn't get in the way. The laptop is silent for longer than REMOTE_YOLO_SILENCE -> YOLO is computed here again.
REMOTE_YOLO_SIZE = 640           # longer frame side for the laptop (= the largest YOLO scale, so the result is the same)
REMOTE_YOLO_SILENCE = 0.5        # s
REMOTE_YOLO_WAIT = 0.15          # s: the loop waits this long for a fresh result (1 cycle = 1 new detection, like locally)
# nr = last frame put out, issued = last frame taken by the laptop (two laptop threads = different frames),
# result_nr = frame the result comes from, given_nr = result already handed to the loop
_remote = {"jpg": None, "nr": 0, "issued": 0, "scale": 1.0, "result": [], "result_nr": 0, "given_nr": 0,
           "asked": 0.0, "t": 0.0, "fps": 0.0}
_remote_new = threading.Condition()
# the old worker still sends Polish keys/classes - normalised to the new ones on arrival
_OLD_CLASSES = {"butelka": "bottle", "puszka": "can"}


def _yolo_for_laptop(frame):
    """Put out a frame for the laptop and return its freshest detections (from a frame 1-2 cycles old)."""
    import cv2

    h, w = frame.shape[:2]
    s = min(1.0, REMOTE_YOLO_SIZE / max(h, w))
    small = cv2.resize(frame, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA) if s < 1 else frame
    ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 75])
    with _remote_new:
        if ok:
            _remote.update(jpg=jpg.tobytes(), nr=_remote["nr"] + 1, scale=s)
            _remote_new.notify_all()
        # tracking computes speed from consecutive detections - the same detection several times in a row would fool it
        _remote_new.wait_for(lambda: _remote["result_nr"] > _remote["given_nr"], timeout=REMOTE_YOLO_WAIT)
        _remote["given_nr"] = _remote["result_nr"]
        if time.time() - _remote["t"] > 0.5:
            return []  # a result from before a break (e.g. from before tracking) - the bottles may be elsewhere now
        return [dict(b) for b in _remote["result"]]  # copies: the loop adds "ean" to them


def _yolo_from_laptop(body, result_nr):
    """Detections from the laptop (in downscaled frame coordinates) -> full-frame pixels."""
    _remote["asked"] = time.time()
    if not body or result_nr <= _remote["result_nr"]:  # empty or late (the other thread was faster)
        return
    try:
        bottles = json.loads(body)
        k = 1.0 / _remote["scale"]
        result = []
        for b in bottles:
            cls = b.get("cls", b.get("klasa"))
            cls = _OLD_CLASSES.get(cls, cls)
            if cls not in ("bottle", "can"):
                continue
            result.append({"cx": float(b["cx"]) * k, "cy": float(b["cy"]) * k, "w": float(b["w"]) * k,
                           "h": float(b["h"]) * k, "conf": float(b.get("conf", b.get("pewnosc"))), "cls": cls,
                           "ean": None, "box": tuple(int(v * k) for v in b["box"])})
    except (ValueError, KeyError, TypeError):
        return
    now = time.time()
    with _remote_new:
        if result_nr <= _remote["result_nr"]:
            return
        if _remote["t"]:
            _remote["fps"] = round(0.8 * _remote["fps"] + 0.2 / max(now - _remote["t"], 1e-3), 1)
        _remote.update(result=result, result_nr=result_nr, t=now)
        _remote_new.notify_all()


def _yolo_frame():
    """Wait (max 1 s) for a frame no laptop thread has taken yet. (jpg, nr) or (None, 0)."""
    with _remote_new:
        for _ in range(5):  # a waiting laptop is "present" too (while tracking the Pi doesn't put out frames)
            _remote["asked"] = time.time()
            if _remote_new.wait_for(lambda: _remote["nr"] > _remote["issued"] and _remote["jpg"], timeout=0.2):
                _remote["issued"] = _remote["nr"]
                return _remote["jpg"], _remote["nr"]
        return None, 0


def yolo_laptop_active():
    return time.time() - _remote["asked"] < REMOTE_YOLO_SILENCE


class BottleDetector:
    """Bottles (and cans) in the frame: [{"cx","cy","w","h","conf","cls","box"}] in pixels.

    YOLO11n via onnxruntime by default; when it's missing or fails - SSD MobileNet via OpenCV.
    """

    def __init__(self):
        self.yolo = None
        if DETECTOR == "yolo":
            try:
                import onnxruntime as ort

                opts = ort.SessionOptions()
                opts.intra_op_num_threads = max(1, min(4, (os.cpu_count() or 4) // 2))
                # onnxruntime threads spin between frames by default (100% of a core for nothing) -
                # on a Raspberry without a fan that's overheating and the clock throttled from 2.4 to 1.5 GHz
                opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
                opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
                # GPU (Brev, onnxruntime-gpu) if there is one, otherwise CPU (Raspberry)
                path = download_yolo()
                if "CUDAExecutionProvider" in ort.get_available_providers():
                    ort.preload_dlls()  # CUDA/cuDNN libraries from pip (onnxruntime-gpu[cuda,cudnn])
                    path = yolo_opset21(path)
                providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider")
                             if p in ort.get_available_providers()]
                self.yolo = ort.InferenceSession(path, opts, providers=providers)
                # GPU: a separate session per scale - changing the input shape in one CUDA session costs ~150 ms, fixed ~7 ms
                self._sessions = {} if "CUDAExecutionProvider" in providers else None
                self._new_session = lambda: ort.InferenceSession(path, opts, providers=providers)
                self.input = self.yolo.get_inputs()[0].name
                print(f"Bottle detection: YOLO11n ({self.yolo.get_providers()[0]})")
            except Exception as e:
                print(f"YOLO unavailable ({e}) - using SSD. Install: pip install onnxruntime")
        if self.yolo is None:
            import cv2

            try:
                pb, pbtxt = download_model()
            except Exception as e:
                raise SO101Error(f"cannot download the bottle model ({e}) - check the internet") from e
            self.net = cv2.dnn.readNetFromTensorflow(pb, pbtxt)
            print("Bottle detection: SSD MobileNet")

    @staticmethod
    def _entry(x0, y0, x1, y1, conf, cls, w, h):
        x0, y0, x1, y1 = max(0.0, x0), max(0.0, y0), min(float(w), x1), min(float(h), y1)
        if x1 - x0 < 4 or y1 - y0 < 4:
            return None
        return {"cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2, "w": x1 - x0, "h": y1 - y0,
                "conf": float(conf), "cls": cls, "box": (int(x0), int(y0), int(x1), int(y1)), "ean": None}

    def _detect_yolo(self, frame, sizes=None):
        """YOLO at several scales (a close bottle comes out better in the small one, a far one in the large one) + joint NMS.

        sizes = only these scales, all fresh in this frame (tracking a large bottle - no results from memory).
        """
        import cv2
        import numpy as np

        h, w = frame.shape[:2]
        threshold = min(BOTTLE_THRESHOLD, BOTTLE_HOLD_THRESHOLD)
        boxes, scores, classes = [], [], []
        if sizes:
            scales = list(sizes)
            self._memory = {}  # after returning to interleaving don't mix with old boxes
        elif YOLO_INTERLEAVE and len(YOLO_SIZES) > 1:
            # one scale in this frame, the others from memory (from previous frames - negligible shift)
            self._nr = (getattr(self, "_nr", -1) + 1) % len(YOLO_SIZES)
            scales = [YOLO_SIZES[self._nr]]
            if not hasattr(self, "_memory"):
                self._memory = {}
            for size, (b_, s_, c_) in self._memory.items():
                if size != scales[0]:
                    boxes += b_
                    scores += s_
                    classes += c_
        else:
            scales = YOLO_SIZES
        for size in scales:
            new_b, new_s, new_c = [], [], []
            s = size / max(h, w)
            img = cv2.resize(frame, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
            ph, pw = (img.shape[0] + 31) // 32 * 32, (img.shape[1] + 31) // 32 * 32  # multiples of 32
            pad = np.full((ph, pw, 3), 114, np.uint8)
            pad[:img.shape[0], :img.shape[1]] = img
            x = cv2.cvtColor(pad, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None].astype(np.float32) / 255.0
            if self._sessions is not None and size not in self._sessions:
                self._sessions[size] = self._new_session()
            session = self.yolo if self._sessions is None else self._sessions[size]
            out = session.run(None, {self.input: x})[0][0].T  # N x (4 + 80 classes)
            for nr, name in YOLO_CLASSES.items():
                sc = out[:, 4 + nr]
                for i in np.where(sc >= threshold)[0]:
                    cx, cy, bw, bh = (float(v) / s for v in out[i, :4])
                    new_b.append([int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh)])
                    new_s.append(float(sc[i]))
                    new_c.append(name)
            boxes += new_b
            scores += new_s
            classes += new_c
            if YOLO_INTERLEAVE:
                self._memory[size] = (new_b, new_s, new_c)
        results = []
        if boxes:
            for i in np.array(cv2.dnn.NMSBoxes(boxes, scores, threshold, 0.5)).flatten():
                bx, by, bw, bh = boxes[i]
                entry = self._entry(bx, by, bx + bw, by + bh, scores[i], classes[i], w, h)
                if entry:
                    results.append(entry)
        return results

    def detect(self, frame, tracking=False, large=False):
        """tracking = the arm is holding a bottle (every frame counts: only here, no WiFi);
        large = the tracked bottle is large in the frame -> only the small scale, fresh every frame (fast and no jumps)."""
        import cv2

        # Brev only for spotting bottles. While tracking the Pi computes YOLO (320, ~32 ms): the Pi->Brev ping is ~160 ms,
        # and the Brev result is from the previous frame - a detection 0.3-0.5 s old swayed the arm and forced low
        # gains (measured 2026-09-26). The Pi also computes everything when Brev is silent > REMOTE_YOLO_SILENCE
        remote = yolo_laptop_active() and not tracking
        _web["yolo"] = "laptop" if remote else "local"  # caption on the image and in /demo
        if remote != getattr(self, "_remote", False):
            self._remote = remote
            if yolo_laptop_active():
                print(f"\nYOLO: {'laptop (searching for a bottle)' if remote else 'here (tracking a bottle - no WiFi delays)'}")
        if remote:
            return _yolo_for_laptop(frame)
        if self.yolo is not None:
            return self._detect_yolo(frame, (YOLO_TRACKING_SIZE,) if tracking and large else None)
        h, w = frame.shape[:2]
        self.net.setInput(cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True))
        results = []
        for d in self.net.forward()[0, 0]:
            cls, conf = int(d[1]), float(d[2])
            if cls not in SSD_CLASSES or conf < min(BOTTLE_THRESHOLD, BOTTLE_HOLD_THRESHOLD):
                continue
            # float(): plain numbers instead of numpy.float32 (otherwise saving the history to JSON crashes)
            entry = self._entry(float(d[3]) * w, float(d[4]) * h, float(d[5]) * w, float(d[6]) * h, conf,
                                SSD_CLASSES[cls], w, h)
            if entry:
                results.append(entry)
        return results


def full_box(bt, cut):
    """Estimated whole bottle when part of it sticks out of the frame (cut = {"top","bottom","left","right"}: bool).

    Without this the center of a cut-off box "runs" towards the edge and the arm slides to the bottom or neck of the bottle.
    """
    x0, y0, x1, y1 = bt["box"]
    w, h = max(x1 - x0, 1), max(y1 - y0, 1)
    aspect = TRACK_ASPECT.get(bt["cls"], 2.5)
    if w > h * 1.2:  # the bottle is lying down - the longer side horizontal
        length = min(max(w, h * aspect), w * CUT_MAX)
        if cut["left"] and not cut["right"]:
            x0 = x1 - length
        elif cut["right"] and not cut["left"]:
            x1 = x0 + length
        return x0, y0, x1, y1
    est_h = min(max(h, w * aspect), h * CUT_MAX)
    if cut["top"] and not cut["bottom"]:
        y0 = y1 - est_h
    elif cut["bottom"] and not cut["top"]:
        y1 = y0 + est_h
    est_w = min(max(w, est_h / aspect), w * CUT_MAX)
    if cut["left"] and not cut["right"]:
        x0 = x1 - est_w
    elif cut["right"] and not cut["left"]:
        x1 = x0 + est_w
    return x0, y0, x1, y1


def find_basket(frame):
    """The vehicle's basket -> a detection like BottleDetector's (cls "basket") or None.

    No YOLO class for it and the VLM is too slow to steer by, but it is easy to see: the largest solid light-grey/white
    blob (checked on real frames: found with and without a bottle, the RoArm or the floor in view). ~3 ms at 320 px."""
    import cv2
    import numpy as np

    h, w = frame.shape[:2]
    k = 320 / w
    small = cv2.resize(frame, (320, max(1, int(h * k))), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 1] <= BASKET_MAX_SAT) & (hsv[..., 2] >= BASKET_MIN_VAL)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))    # cables and specks away
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))   # bottle / shadows inside filled
    best = None
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        area = cv2.contourArea(c)
        # solid: a white cable loop has a big hull but a small area
        if area >= BASKET_MIN_AREA * mask.size and area >= 0.6 * cv2.contourArea(cv2.convexHull(c)) \
                and (best is None or area > best[0]):
            best = (area, c)
    if best is None:
        return None
    x, y, bw, bh = cv2.boundingRect(best[1])
    x0, y0, x1, y1 = x / k, y / k, (x + bw) / k, (y + bh) / k
    return {"cls": "basket", "conf": 1.0, "box": (x0, y0, x1, y1), "cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2,
            "w": x1 - x0, "h": y1 - y0, "ean": None, "code_xy": None}


def aim_point(bt, width, height, cut=None, rel=None):
    """The point on the bottle the arm aims at: the remembered code location (rel) or the middle of the label."""
    if cut is None:
        m = 3
        x0, y0, x1, y1 = bt["box"]
        cut = {"top": y0 <= m, "bottom": y1 >= height - m, "left": x0 <= m, "right": x1 >= width - m}
    x0, y0, x1, y1 = full_box(bt, cut)
    rx, ry = rel or (0.5, TRACK_AIM_HEIGHT)
    px, py = x0 + rx * (x1 - x0), y0 + ry * (y1 - y0)
    if cut["top"] and cut["bottom"]:
        py = height / 2  # bottle taller than the frame - stand still vertically
    if cut["left"] and cut["right"]:
        px = width / 2
    return px, py


class Inspection:
    """Inspection state machine: SEARCHING -> CENTERING -> READING -> RESULT (DEPOSIT / NO_DEPOSIT / NO_CODE).

    Result: screen + log + http://<IP>:8765/result (+ optionally POST to DECISION_URL).
    NO_CODE -> "rotate_bottle": true. After rotating: /rotated -> reads the code again.
    """

    # look-around points relative to the start position (x = left/right, y = up/down, within the ranges from the settings)
    POINTS = [(0, 0), (-1, 0), (1, 0), (1, 1), (-1, 1), (-1, -1), (1, -1), (0, 0)]

    def __init__(self, st):
        now = time.time()
        self.center = {TRACK_JOINT_H: st.here[TRACK_JOINT_H], TRACK_JOINT_V: st.here[TRACK_JOINT_V]}
        saved = load_json(POSES_FILE, {}).get(HOME_POSE)
        if saved and all(abs(saved[j] - st.here[j]) < 10 for j in ("lift", "elbow")):
            # after a restart wait where M was set (arm in the same posture), not where it happens to stand
            self.center = {j: saved[j] for j in (TRACK_JOINT_H, TRACK_JOINT_V)}
        self.state, self.since, self.no_target_since = "SEARCHING", now, now
        self.target, self.result, self.centered_since = None, None, None
        self.point, self._t = 0, None
        self.manual = 0.0  # when manual control was last used (then we don't look around)

    def after_rotation(self):
        """The bottle was rotated - read the code again."""
        if self.target:
            self.state, self.since, self.result = "READING", time.time(), None
            state["inspection"] = {"state": "READING", "bottle": self.target}

    def _result(self, result, deposit, trk, deposits):
        am = deposits.amounts.get(trk.ean) if trk.ean else None
        self.result = {
            "result": result, "deposit": deposit, "rotate_bottle": result == "NO_CODE",
            "bottle": self.target, "code": trk.ean, "name": deposits.names.get(trk.ean or "", ""),
            "amount": float(am[0]) if am else None, "currency": am[1] if am else None,
            "time": time.strftime("%H:%M:%S"),
        }
        self.state = "RESULT"
        state["inspection"] = dict(self.result, state="RESULT")
        state["last_result"] = dict(self.result)  # stays even when the bottle leaves the frame
        # /demo view: event history + last result of each bottle (after rotating NO_CODE -> DEPOSIT = 1 bottle)
        state["history"] = (state.get("history", []) + [dict(self.result)])[-50:]
        results = state.setdefault("results", {})
        results.pop(self.target, None)
        results[self.target] = dict(self.result)
        while len(results) > 200:
            results.pop(next(iter(results)))
        print(f"\n=== RESULT: {result} {self.result['code'] or ''} {self.result['name']} ===")
        _log(f"RESULT {self.result}")
        if DECISION_URL:  # in the background - doesn't block tracking
            def send(data=dict(self.result)):
                try:
                    import requests

                    requests.post(DECISION_URL, json=data, timeout=3)
                except Exception as e:
                    print("\ncould not send the result to", DECISION_URL, e)

            threading.Thread(target=send, daemon=True).start()
        return f"RESULT: {result}"

    def set_waiting(self, st):
        """Current camera position = the waiting position (the other arm brings the bottle here)."""
        self.center = {TRACK_JOINT_H: st.here[TRACK_JOINT_H], TRACK_JOINT_V: st.here[TRACK_JOINT_V]}

    def _look_around(self, st, now, return_only=False):
        """Slow search of the surroundings with the camera or a return to the waiting position (smoothly)."""
        dt = 0.05 if self._t is None else min(max(now - self._t, 0.0), 0.2)
        self._t = now
        px, py = self.POINTS[self.point]
        goal = {TRACK_JOINT_H: self.center[TRACK_JOINT_H] + px * SEARCH_RANGE_H,
                TRACK_JOINT_V: self.center[TRACK_JOINT_V] + py * SEARCH_RANGE_V}
        arrived = True
        for j, c in goal.items():
            lo, hi = st.limits[j]
            c = min(max(c, lo), hi)
            d = c - st.target[j]
            if abs(d) > 0.5:
                arrived = False
                st.target[j] += math.copysign(min(abs(d), SEARCH_SPEED * dt), d)
                st.slow.add(j)
                st.track_speed[j] = math.copysign(SEARCH_SPEED, d)
        if arrived and not return_only:
            self.point = (self.point + 1) % len(self.POINTS)

    def step(self, trk, deposits, st, now, active):
        """One step of the state machine. Returns a message or None."""
        msg = None
        if trk.target != self.target:  # new bottle or lost
            self.target, self.result, self.centered_since = trk.target, None, None
            self.state, self.since = ("CENTERING" if trk.target else "SEARCHING"), now
            if not trk.target:
                self.no_target_since = now
            self._t = None
            state["inspection"] = {"state": self.state, "bottle": self.target}
        if not active:
            return None

        if self.state == "SEARCHING":
            if now - self.manual < 3.0:
                pass  # you're controlling manually - don't interfere
            elif LOOK_AROUND and now - self.no_target_since > SEARCH_AFTER:
                self._look_around(st, now)
            elif not LOOK_AROUND and now - self.no_target_since > RETURN_AFTER:
                self.point = 0  # point (0, 0) = the waiting position
                self._look_around(st, now, return_only=True)
        elif self.state == "CENTERING":
            f = trk._filt or trk.error[:2] if hasattr(trk, "error") else None
            calm = f is not None and max(abs(f[0]), abs(f[1])) < 0.15 and \
                max(abs(w) for w in trk._v.values()) < 5.0
            if trk.ean and deposits.check(trk.ean) is not None:  # code already read while centering
                msg = self._result("DEPOSIT" if deposits.check(trk.ean) else "NO_DEPOSIT",
                                   deposits.check(trk.ean), trk, deposits)
            elif calm:
                self.centered_since = self.centered_since or now
                if now - self.centered_since > CENTERED_TIME:
                    self.state, self.since = "READING", now
                    state["inspection"] = {"state": "READING", "bottle": self.target}
                    msg = "centered - reading the code"
            else:
                self.centered_since = None
        elif self.state == "READING":
            if trk.ean:
                deposit = deposits.check(trk.ean)
                if deposit is not None:
                    msg = self._result("DEPOSIT" if deposit else "NO_DEPOSIT", deposit, trk, deposits)
            elif now - self.since > CODE_READ_TIME:
                msg = self._result("NO_CODE", None, trk, deposits)
        elif self.state == "RESULT" and self.result and self.result["result"] == "NO_CODE" and trk.ean:
            self.state, self.since = "READING", now  # the code showed up later (e.g. after rotating) - decide
        return msg

    def label(self, now):
        """(text, color) for the big caption on the screen."""
        if self.state == "SEARCHING":
            if LOOK_AROUND and now - self.no_target_since > SEARCH_AFTER:
                return "SEARCHING FOR A BOTTLE - looking around...", (200, 200, 200)
            return "WAITING FOR A BOTTLE...", (200, 200, 200)
        if self.state == "CENTERING":
            return "CENTERING THE BOTTLE...", (0, 220, 255)
        if self.state == "READING":
            return f"READING THE CODE... {max(0.0, CODE_READ_TIME - (now - self.since)):.1f} s", (0, 220, 255)
        r = self.result or {}
        if r.get("result") == "DEPOSIT":
            am = f" {r['amount']:.2f} {r['currency']}" if r.get("amount") is not None else ""
            return f"DEPOSIT{am}  {r.get('name', '')}", (0, 200, 0)
        if r.get("result") == "NO_DEPOSIT":
            return f"NO DEPOSIT ({r.get('code')})", (0, 0, 230)
        return "NO CODE - ROTATE THE BOTTLE", (0, 140, 255)


_inspection = None


class BottleTracker(Tracker):
    """Keeps the nearest (largest) bottle in the middle of the image.

    Catches it only after TRACK_CONFIRM frames, lets go when too far or lost,
    switches to another one only when it's clearly closer for several frames.
    The arm movement itself (correct-and-wait, sensitivity learning, searching) is inherited from Tracker.
    """

    def __init__(self):
        super().__init__()
        self.box = None
        self.ean = None
        self._nr = 0
        self._cand, self._cand_n = None, 0
        self._other_n = 0

    @staticmethod
    def _rank(b):
        # a bottle takes precedence over a "can" (the network also calls cups that); then larger = closer
        return (b["cls"] == "bottle", b["w"] * b["h"])

    def _near(self, b, cx, cy, width):
        return abs(b["cx"] - cx) + abs(b["cy"] - cy) < TRACK_RADIUS * width

    @staticmethod
    def _overlap(a, b):
        """Two boxes of one bottle (e.g. whole + just a piece): the center of one inside the other or mostly shared."""
        ax0, ay0, ax1, ay1 = a["box"]
        bx0, by0, bx1, by1 = b["box"]
        if (ax0 <= b["cx"] <= ax1 and ay0 <= b["cy"] <= ay1) or (bx0 <= a["cx"] <= bx1 and by0 <= a["cy"] <= by1):
            return True
        shared = max(0, min(ax1, bx1) - max(ax0, bx0)) * max(0, min(ay1, by1) - max(ay0, by0))
        return shared > 0.5 * min((ax1 - ax0) * (ay1 - ay0), (bx1 - bx0) * (by1 - by0))

    def step(self, bottles, deposits, width, height, st, frame_t=None):
        """frame_t = when the camera took this frame (exact latency for movement prediction)."""
        now = time.time()
        if frame_t:
            self.measure_latency(frame_t, now)
        msg = None

        def fits(b):
            threshold = BOTTLE_THRESHOLD if b["cls"] == "bottle" else CAN_THRESHOLD
            if b["conf"] < threshold or b["h"] < MIN_BOTTLE * height:
                return False
            return not TRACK_DEPOSIT_ONLY or (b["ean"] and deposits.check(b["ean"]))

        good = [b for b in bottles if fits(b)]
        largest = max(good, key=self._rank, default=None)
        current = None
        if self.target is not None and self.xy:
            near = [b for b in bottles if self._near(b, self.xy[0], self.xy[1], width)]
            current = min(near, key=lambda b: abs(b["cx"] - self.xy[0]) + abs(b["cy"] - self.xy[1]),
                          default=None)
            if current is not None:  # the same bottle detected twice (whole + piece) -> always the whole one, no jumping
                current = max((b for b in bottles if self._overlap(b, current)), key=lambda b: b["w"] * b["h"])

        if self.target is None:
            if largest:  # confirm over several frames - one false detection doesn't move the arm
                if self._cand and self._near(largest, self._cand[0], self._cand[1], width):
                    self._cand_n += 1
                else:
                    self._cand_n = 1
                self._cand = (largest["cx"], largest["cy"])
                if self._cand_n >= TRACK_CONFIRM:
                    self._nr += 1
                    self.target, current, self.ean = f"{largest['cls']}{self._nr}", largest, None
                    self._other_n = 0
                    msg = f"TRACKING {self.target}"
            else:
                self._cand, self._cand_n = None, 0
        elif current is not None:
            if current["h"] < MIN_BOTTLE * height:
                msg, self.target, current = f"{self.target} MOVED AWAY - letting go", None, None
            elif largest is not None and largest is not current and not self._overlap(largest, current) and (
                    (largest["cls"] == "bottle") > (current["cls"] == "bottle")
                    or (largest["cls"] == current["cls"]
                        and self._rank(largest)[1] > self._rank(current)[1] * SWITCH_WHEN)):
                self._other_n += 1
                if self._other_n >= TRACK_SWITCH_FRAMES:
                    self._nr += 1
                    old = self.target
                    self.target, current, self.ean = f"bottle{self._nr}", largest, None
                    self._other_n = 0  # without this it switched again already in the next frame (series of ANOTHER CLOSER)
                    msg = f"ANOTHER CLOSER: {old} -> {self.target}"
            else:
                self._other_n = 0
        elif now - self.seen > LOST_AFTER:
            msg, self.target = f"LOST {self.target} - letting go", None

        if self.target is None:
            self.box = None
            return msg
        if current is None:
            if not msg:
                self._search(st, now)
            return msg

        self.seen, self.size = now, current["h"] / height
        self.xy = (current["cx"], current["cy"])
        self.box = current["box"]
        if current["ean"]:
            self.ean = current["ean"]
        self.source = current["cls"]
        # frame edges with hysteresis (turns on at 3 px, off from 25 px) - no flicker
        x0, y0, x1, y1 = current["box"]
        dist = {"top": y0, "bottom": height - y1, "left": x0, "right": width - x1}
        for edge, d in dist.items():
            # the basket is usually bigger than the frame: its box touching the edges is not a cut-off bottle (that
            # would cap the speed at CUT_MAX_SPEED and stop vertical centring) - aim at the visible middle instead
            self._cut[edge] = False if current["cls"] == "basket" else (d <= 3 if not self._cut[edge] else d < 25)
        # where the code is on the bottle: remember it (relative to the full box) and keep aiming there - no jumping
        # between "code read" and "middle of the label"
        if current.get("code_xy"):
            f0, g0, f1, g1 = full_box(current, self._cut)
            rel = ((current["code_xy"][0] - f0) / max(f1 - f0, 1), (current["code_xy"][1] - g0) / max(g1 - g0, 1))
            if 0.0 <= rel[0] <= 1.0 and 0.0 <= rel[1] <= 1.0:
                k = self.code_rel
                self.code_rel = rel if k is None else (k[0] + 0.3 * (rel[0] - k[0]), k[1] + 0.3 * (rel[1] - k[1]))
        basket = current["cls"] == "basket"  # the basket: its middle, not a label height or a code
        px, py = aim_point(current, width, height, self._cut,
                           (0.5, 0.5) if basket else (self.code_rel if TRACK_AIM_AT_CODE else None))
        # bottle right in front of the camera (most of the frame height): the label is in the frame anyway, and the guessed
        # center of a cut-off bottle jumps (was: wrist -12 -> +12 deg/s) - stand still vertically, center only horizontally
        h_rel = current["h"] / height
        self._close = not basket and h_rel >= (CLOSE_FROM - 0.12 if getattr(self, "_close", False) else CLOSE_FROM)
        if self._close:
            py = height / 2
        self.point = (px, py)
        self.error = ((px - width / 2) / (width / 2), (py - height / 2) / (height / 2), True)
        if msg:
            self._reset()
        self._hist = [h for h in self._hist if now - h[0] < 0.6] + [(now, self.error[0], self.error[1])]
        self._searched = 0.0
        return self._steer(st, msg, now)


KEY_PANEL = [
    ("MOVE", None),
    ("W / S", "UP / DOWN"), ("R / F", "FARTHER / CLOSER"), ("A / D", "LEFT / RIGHT"),
    ("J / L", "tilt gripper"), ("U / O", "roll gripper"), ("1 2 3", "speed"), ("B", "STOP"),
    ("GRIPPER", None),
    ("SPACE", "grip / release"), ("Z / X", "open / close a little"),
    ("CAMERA", None),
    ("T", "tracking on / off"), ("ENTER", "inspect can (circuit)"),
    ("POSES", None),
    ("P", "save circuit pose"), ("C", "delete circuit poses"), ("M", "wait here for a bottle"), ("H", "go home"),
    ("", None),
    ("Y", "save bottle history"), ("TAB", "hide / show help"), ("Q / ESC", "quit"),
]

# sliders in the "Settings" window: (label, variable, min, max, multiplier)
SLIDERS = [
    ("tracking 0/1", None, 0, 1, 1),
    ("smooth 0/1", "TRACK_SMOOTH", 0, 1, 1),
    ("reaction x10", "TRACK_LOOP_GAIN", 3, 40, 0.1),
    ("max speed", "TRACK_MAX_SPEED", 3, 60, 1),
    ("prediction %", "TRACK_FF", 0, 100, 0.01),
    ("step % (stepwise)", "TRACK_STEP", 10, 100, 0.01),
    ("pause ms", "TRACK_PAUSE", 200, 2000, 0.001),
    ("speed deg/s", "TRACK_SPEED", 5, 60, 1),
    ("target zone %", "TRACK_TARGET_ZONE", 3, 30, 0.01),
    ("move start %", "TRACK_START", 5, 40, 0.01),
    ("min bottle %", "MIN_BOTTLE", 3, 60, 0.01),
    ("confidence %", "BOTTLE_THRESHOLD", 20, 90, 0.01),
    ("search max deg", "TRACK_SEARCH_MAX", 0, 40, 1),
    ("deposit only 0/1", "TRACK_DEPOSIT_ONLY", 0, 1, 1),
]
SETTINGS_WINDOW = "Settings (sliders work live)"


def draw_panel(frame):
    """Semi-transparent key panel on the right side of the image."""
    import cv2

    h, w = frame.shape[:2]
    x0 = w - 400
    piece = frame[36:h - 50, x0:w]
    cv2.addWeighted(piece, 0.35, piece * 0, 0.65, 0, dst=piece)
    y = 64
    for key, desc in KEY_PANEL:
        if desc is None:
            if key:
                cv2.putText(frame, key, (x0 + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
            y += 26 if key else 10
            continue
        cv2.putText(frame, key, (x0 + 18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        cv2.putText(frame, desc, (x0 + 125, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (210, 210, 210), 1)
        y += 25


def create_sliders(tracking):
    import cv2

    cv2.namedWindow(SETTINGS_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(SETTINGS_WINDOW, 460, 420)
    for label, variable, lo, hi, mul in SLIDERS:
        value = int(tracking) if variable is None else int(round(float(globals()[variable]) / mul))
        cv2.createTrackbar(label, SETTINGS_WINDOW, min(max(value, lo), hi), hi, lambda _v: None)
        cv2.setTrackbarMin(label, SETTINGS_WINDOW, lo)


def read_sliders(tracking):
    """Copy the sliders into the settings. Returns the tracking switch state (or tracking, when the window is closed)."""
    import cv2

    try:
        for label, variable, lo, hi, mul in SLIDERS:
            v = cv2.getTrackbarPos(label, SETTINGS_WINDOW)
            if v < 0:
                return tracking
            if variable is None:
                tracking = bool(v)
            else:
                globals()[variable] = bool(v) if variable in ("TRACK_DEPOSIT_ONLY", "TRACK_SMOOTH") else v * mul
    except cv2.error:  # the settings window is closed - the last values stay
        pass
    return tracking


def camera_mode(port=None, mock=False):
    """Main program: camera window + manual control + can inspection on a signal."""
    import cv2

    arm = connect(port, mock)
    st = Controller(arm)
    cam = CameraThread(open_camera())
    deposits, trk = Deposits(), BottleTracker()
    history = TrackingHistory()
    history.load()
    inspection = Inspection(st)
    globals()["_inspection"] = inspection
    detector = BottleDetector()
    reader = CodeReader()
    codes, target_since, prev_target = [], 0.0, None
    tracking = TRACK and "--no-tracking" not in sys.argv  # --no-tracking: the arm moves only from keys
    loop_t = time.time()
    srv = http_server(arm)
    print(f"Pad: {'CONNECTED' if st.pad_ok else 'none (keyboard only)'}")
    print(f"READY. Signal from the RoArm: http://{my_ip()}:{HTTP_PORT}/scan")
    title = "SO-101 Arm"
    panel = f"http://{my_ip()}:{HTTP_PORT}/"
    print(f"Presentation view: {panel}demo")
    if HEADLESS:
        print(f"No window - panel in the browser: {panel}")
        show_help = False  # the keys are on the page
    else:
        print("Click the camera window and steer:\n" + HELP)
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)  # the window can be shrunk/enlarged
        cv2.resizeWindow(title, 960, 540)
        show_help = True
        create_sliders(tracking)
    message, message_until = "", 0.0
    was_busy = False
    t_web = t_clean = 0.0
    frame_nr, sharp = 0, 0.0
    t_servo, servo_nr = 0.0, -1  # SO-101 servo temperatures read one by one
    screen_width = 960 if HEADLESS else 1280  # without a window the image goes only to the browser (960 px anyway)

    def show(text):
        nonlocal message, message_until
        message, message_until = text, time.time() + 3
        print(f"\n{text}")

    try:
        while True:
            ok, frame, t_shot = cam.read()  # the newest frame + when it came from the camera
            if not ok:
                continue
            _web["raw"] = frame  # /frame (reference only - encoded only on request)
            now = time.time()
            if now > loop_t:  # loop fps (smoothed) - on the image and in /status
                state["fps"] = round(0.9 * state.get("fps", 0.0) + 0.1 / (now - loop_t), 1)
            dt, loop_t = min(now - loop_t, 0.2), now
            height, width = frame.shape[:2]
            frame_nr += 1
            if frame_nr % 5 == 1:  # sharpness only for the on-screen caption - no need every frame
                sharp = sharpness(frame)
            bottles = detector.detect(frame, tracking=bool(trk.target), large=trk.size >= YOLO_LARGE_FROM)
            basket = find_basket(frame) if TRACK_OBJECT == "basket" else None
            # for the RoArm (roarm_pick.py): where the bottles stand in the frame - /bottles
            _web["view"] = {"width": width, "height": height, "t": t_shot,
                            "basket": [int(v) for v in basket["box"]] if basket else None, "bottles": [
                {"cls": b["cls"], "conf": round(b["conf"], 3), "cx": round(b["cx"], 1),
                 "cy": round(b["cy"], 1), "box": [int(v) for v in b["box"]]}
                for b in bottles if b["conf"] >= (BOTTLE_THRESHOLD if b["cls"] == "bottle" else CAN_THRESHOLD)]}
            reader.paused = inspection.state == "RESULT" and bool(inspection.result) and inspection.result["code"] is not None
            if reader.wants_frame():
                reader.submit(frame.copy(), trk.target, trk.box)  # codes read in the background, on a copy of the frame
            codes = reader.codes
            if trk.target and trk.target in reader.target_code:
                trk.ean = reader.target_code[trk.target]
            # processing at full resolution, displaying on a smaller image
            sk = screen_width / width
            screen = cv2.resize(frame, (screen_width, int(round(height * sk))), interpolation=cv2.INTER_AREA) \
                if sk < 1 else frame.copy()
            sk = min(sk, 1.0)
            ew, eh = screen.shape[1], screen.shape[0]
            # a code inside a bottle = that bottle's code (deposit)
            for bt in bottles:  # (only confirmed codes - a single read from the whole image can be wrong)
                x0, y0, x1, y1 = bt["box"]
                code = next((k for k in codes if k["ean"] in reader.confirmed and x0 <= k["cx"] <= x1 and y0 <= k["cy"] <= y1),
                            None)
                bt["ean"] = code["ean"] if code else None
                bt["code_xy"] = (code["cx"], code["cy"]) if code else None
            for k in codes:
                if not k["ean"]:
                    continue
                deposit = deposits.check(k["ean"])
                color = (0, 200, 0) if deposit else ((0, 0, 230) if deposit is False else (0, 220, 255))
                corners = (k["corners"] * sk).astype(int)
                cv2.polylines(screen, [corners], True, color, 2)
                cv2.putText(screen, k["ean"], tuple(corners[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                if _busy.is_set() and not state["code"]:
                    state["code"] = k["ean"]
            for bt in bottles:
                x0, y0, x1, y1 = (int(v * sk) for v in bt["box"])
                is_target = bool(trk.target and trk.xy and abs(bt["cx"] - trk.xy[0]) + abs(bt["cy"] - trk.xy[1]) < 0.05 * width)
                if bt["conf"] < (BOTTLE_THRESHOLD if bt["cls"] == "bottle" else CAN_THRESHOLD) and not is_target:
                    continue  # a weak detection that isn't the target - don't clutter the image
                ean = bt["ean"] or (trk.ean if is_target else None)
                deposit = deposits.check(ean) if ean else None
                # green = deposit, red = no deposit, yellow = unknown (code not read)
                color = (0, 200, 0) if deposit else ((0, 0, 230) if deposit is False else (0, 220, 255))
                cv2.rectangle(screen, (x0, y0), (x1, y1), color, 6 if is_target else 2)
                if is_target and getattr(trk, "point", None):  # the arm aims here
                    cv2.drawMarker(screen, (int(trk.point[0] * sk), int(trk.point[1] * sk)), color,
                                   cv2.MARKER_TILTED_CROSS, 30, 3)
                desc = f"{bt['cls']} {bt['conf']:.0%}" + (" DEPOSIT" if deposit else "") + ("  <- TARGET" if is_target else "")
                cv2.putText(screen, desc, (x0 + 4, max(y0 + 22, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if basket:  # cyan: the basket the camera keeps centred
                x0, y0, x1, y1 = (int(v * sk) for v in basket["box"])
                cv2.rectangle(screen, (x0, y0), (x1, y1), (255, 255, 0), 6 if trk.target else 2)
                if trk.target and getattr(trk, "point", None):
                    cv2.drawMarker(screen, (int(trk.point[0] * sk), int(trk.point[1] * sk)), (255, 255, 0),
                                   cv2.MARKER_TILTED_CROSS, 30, 3)
                cv2.putText(screen, "basket" + ("  <- TARGET" if trk.target else ""), (x0 + 4, max(y0 + 22, 40)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            # /demo view: the image with only the boxes (no bars with numbers) + list of detections
            _web["detections"] = [{"cls": b["cls"], "conf": round(b["conf"], 2)} for b in bottles
                                  if b["conf"] >= (BOTTLE_THRESHOLD if b["cls"] == "bottle" else CAN_THRESHOLD)]
            if _web["viewers_clean"] > 0 and now - t_clean >= 1 / WEB_FPS:
                t_clean = now
                web_frame(screen, clean=True)

            # big caption: inspection stage and result (deposit / none / rotate the bottle)
            if tracking:
                caption, color = inspection.label(now)
                (tw, th), _ = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                cv2.rectangle(screen, (0, 32), (tw + 20, 32 + th + 20), (0, 0, 0), -1)
                cv2.putText(screen, caption, (10, 32 + th + 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

            # manual control only when no automatic move is running
            if _busy.is_set():
                was_busy = True
            else:
                if was_busy:
                    st.after_task()
                    was_busy = False
                if tracking:
                    had_target = trk.target is not None
                    key_before = trk.ean or trk.target
                    targets = bottles if TRACK_OBJECT == "bottle" else ([basket] if basket else [])
                    msg = trk.step(targets, deposits, width, height, st, frame_t=t_shot)
                    # history: a fresh measurement of the tracked bottle (key = EAN, and without a code - the bottle number)
                    if trk.target and trk.xy and now - trk.seen < 0.05:
                        history.update(trk.ean or trk.target, deposits.names.get(trk.ean or "", ""), now,
                                       trk.xy[0], trk.xy[1], trk.size, st.here["pan"], st.here["wflex"])
                    if had_target and trk.target is None and key_before:
                        history.mark_lost(key_before)
                    history.cleanup(now)
                    if msg:
                        show(msg)
                        _log(msg)
                    if trk.target and now - getattr(trk, "_print_t", 0) > 0.3 and hasattr(trk, "error"):
                        trk._print_t = now
                        ex, ey, was_read = trk.error
                        print(f"  {trk.target} x={ex:+.2f} y={ey:+.2f}"
                              f" | pan={st.here['pan']:6.1f} wflex={st.here['wflex']:6.1f}"
                              f" | goal pan={st.target['pan']:6.1f} wflex={st.target['wflex']:6.1f}")
                    if had_target and trk.target is None and ON_LOSS == "home":
                        run_task(arm, go_home, "home")
                # bottle inspection (read the code, deposit) only while following bottles
                msg_i = inspection.step(trk, deposits, st, now,
                                        tracking and not _busy.is_set() and TRACK_OBJECT == "bottle")
                if msg_i:
                    show(msg_i)
                pressed = st.step()
                # for the RoArm: whether the camera stands still (image = table map only in the still look pose)
                prev = _web.get("joints")
                _web["joints"] = dict(st.here)
                _web["moved_t"] = now if not prev or any(abs(st.here[j] - prev[j]) > 0.4 for j in MOVE_JOINTS) \
                    else _web.get("moved_t", 0.0)
                # SO-101 servo temperature and voltage: one servo every 0.5 s (registers 62 voltage, 63 temperature)
                if not st.mock and now - t_servo > 0.5:
                    t_servo, servo_nr = now, (servo_nr + 1) % len(arm.JOINTS)
                    j = arm.JOINTS[servo_nr]
                    val, res, _ = arm.ph.read2ByteTxRx(arm.port, arm.ids[j], 62)
                    if res == arm._ok:
                        s = _web.setdefault("so101", {"temp": {}, "voltage": {}})
                        s["temp"][j], s["voltage"][j] = val >> 8, (val & 0xFF) / 10
                if "y" in pressed or "start" in pressed:
                    run_task(arm, inspect_can, "scan")
                if "back" in pressed:
                    break

            inputs = []
            new = tracking
            if not HEADLESS:
                k = cv2.waitKey(1) & 0xFF
                new = read_sliders(tracking)
                if k != 255:
                    inputs.append("\r" if k in (10, 13) else ("\x1b" if k == 27 else chr(k).lower()))
            while not _web_keys.empty():  # keys and the tracking switch from the browser panel
                c = _web_keys.get()
                if c.startswith("track"):
                    new = c == "track1"
                elif c.startswith("object:"):  # /track_object: follow bottles or the basket
                    globals()["TRACK_OBJECT"], trk.target = c[7:], None
                    show(f"camera follows: {TRACK_OBJECT}")
                elif c not in ("q", "\x1b"):  # the browser doesn't quit the program
                    inputs.append(c)
            if new != tracking:
                tracking, trk.target = new, None
                if not HEADLESS:  # switch from the browser -> slider in the window (otherwise the slider switches it back)
                    try:
                        cv2.setTrackbarPos("tracking 0/1", SETTINGS_WINDOW, int(tracking))
                    except cv2.error:
                        pass
                show(f"tracking {'ON' if tracking else 'OFF'}")
            inputs += [c for c in read_keys() if c == "\r"]  # Enter from the terminal too
            for c in inputs:
                if c in ("q", "\x1b"):
                    raise KeyboardInterrupt
                if c == "\r":
                    run_task(arm, inspect_can, "scan") or show("arm busy")
                elif _busy.is_set():
                    continue
                elif c == "h":
                    run_task(arm, go_home, "home") or show("arm busy")
                elif c == "g":  # G: follow the (grey) basket <-> bottles
                    globals()["TRACK_OBJECT"] = "bottle" if TRACK_OBJECT == "basket" else "basket"
                    trk.target = None
                    show(f"camera follows: {TRACK_OBJECT}")
                elif c == "t":
                    tracking, trk.target = not tracking, None
                    if not HEADLESS:
                        try:
                            cv2.setTrackbarPos("tracking 0/1", SETTINGS_WINDOW, int(tracking))
                        except cv2.error:
                            pass
                    show(f"tracking {'ON' if tracking else 'OFF'}")
                elif c == "\t":
                    show_help = not show_help
                elif c == "m":
                    inspection.set_waiting(st)
                    show(save_pose(arm, HOME_POSE) + " + the camera waits here for a bottle")
                elif c == "p":
                    poses = load_json(POSES_FILE, {})
                    free = [p for p in SCAN_POSES if p not in poses]
                    show(save_pose(arm, free[0]) if free else "already 4 scan poses - C deletes them")
                elif c == "c":
                    poses = load_json(POSES_FILE, {})
                    for p in SCAN_POSES:
                        poses.pop(p, None)
                    save_json(POSES_FILE, poses)
                    show("scan poses deleted - it will look around")
                elif c == "y":
                    show(f"history saved ({len(history.items)} bottles)" if history.save()
                         else "error saving the history")
                elif st.key(c):
                    inspection.manual = time.time()  # you're controlling manually - don't look around for 3 s

            # captions on the image
            if st.message:
                show(st.message)
                st.message = ""
            top = state["state"] + (f" | TRACKING {trk.target}" if trk.target else (" | tracking on" if tracking else ""))
            top += f" | {state.get('fps', 0):.0f} fps"
            top += f" | YOLO laptop {_remote['fps']:.0f}/s" if _web.get("yolo") == "laptop" else ""
            top += f" | sharpness {sharp:.0f}" + (" (BLURRY - turn the lens)" if sharp < 60 else "")
            if state["result"] and not _busy.is_set():
                r = state["result"]
                top += f" | last: {r.get('code') or r.get('error') or 'no code'}"
                top += " (DEPOSIT)" if r.get("deposit") else ""
            h, w_ = eh, ew
            cv2.drawMarker(screen, (w_ // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
            cv2.rectangle(screen, (0, 0), (w_, 28), (0, 0, 0), -1)
            cv2.putText(screen, top, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.rectangle(screen, (0, h - 46), (w_, h), (0, 0, 0), -1)
            cv2.putText(screen, st.describe()[:90], (6, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 255, 200), 1)
            footer = (f"panel: {panel}" if HEADLESS
                      else "TAB = key help | sliders in the 'Settings' window | Q = quit")
            cv2.putText(screen, footer, (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
            if show_help:
                draw_panel(screen)
            if message and time.time() < message_until:
                # below the big deposit caption (that one takes the top of the image), so they don't overlap
                cv2.putText(screen, message, (10, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            _web["tracking"] = tracking
            if not HEADLESS:
                cv2.imshow(title, screen)
            if _web["viewers"] > 0 and now - t_web >= 1 / WEB_FPS:  # browser panel (also next to the window)
                t_web = now
                web_frame(screen)
    except KeyboardInterrupt:
        pass
    except SO101Error as e:
        print("\nERROR:", e)
    finally:
        reader.stop()
        history.save()
        print("\nDone - the arm holds its position.")
        srv.shutdown()
        cam.release()
        if not HEADLESS:
            cv2.destroyAllWindows()
        if _busy.is_set():
            _scan_done.wait(timeout=30)
        try:
            arm.hold()
        except SO101Error:
            pass
        arm.close()


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else None
    mock = "--mock" in sys.argv
    try:
        if "--no-camera" in sys.argv:
            manual_control(port=port, mock=mock, http=True)
        else:
            camera_mode(port=port, mock=mock)
    except SO101Error as e:
        print("ERROR:", e)
