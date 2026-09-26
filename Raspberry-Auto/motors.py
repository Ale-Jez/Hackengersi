"""Two MAB MA-D-GL40 KV70 wheel actuators (built-in MD driver) on a CANdle USB-FDCAN dongle, velocity mode.

    python motors.py ping      list the drive ids on the bus and blink them
    python motors.py test      left wheel, right wheel, both: slowly forward 1 s each (wheels OFF the ground!)
    python motors.py jog       keyboard drive: w/s forward/back, a/d spin, space stop, q quit

Needs `pip install candlesdk` (imports as pyCandle). MOCK=1 runs without hardware.
"""
import json
import os
import sys
import threading
import time

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
MOCK = os.environ.get("MOCK") == "1"
DT = 0.02  # 50 Hz: well inside the MD watchdog, so the drives stay enabled only while this program runs


def load_config():
    with open(CONFIG) as f:
        return json.load(f)


def _candle(cfg):
    import pyCandle as pc
    return pc, pc.attachCandle(pc.CANdleDatarate_E.CAN_DATARATE_1M, getattr(pc.busTypes_t, cfg["candle_bus"]))


class Wheels:
    """set(left, right) in -1..1 of max_wheel_rad_s (+ = forward for the car).

    A 50 Hz thread ramps the actual speed toward the target (accel_rad_s2) and keeps sending it: the MD
    drives disable themselves when nobody talks to them, so the car stops if this program dies.
    halt() skips the ramp (obstacle stop)."""

    def __init__(self, cfg, mock=MOCK):
        self.max, self.acc = cfg["max_wheel_rad_s"], cfg["accel_rad_s2"]
        self.sign = (cfg["left_sign"], cfg["right_sign"])  # one motor is mirrored, so one sign is -1
        self.target, self.now = [0.0, 0.0], [0.0, 0.0]
        self.mock, self.log = mock, []  # log: every set() call, for the selftest
        self.lock, self.done = threading.Lock(), threading.Event()
        self.mds = []
        if not mock:
            if cfg["left_id"] is None or cfg["right_id"] is None:
                raise SystemExit("set left_id / right_id in config.json (python motors.py ping)")
            pc, self.candle = _candle(cfg)
            for can_id in (cfg["left_id"], cfg["right_id"]):
                md = pc.MD(can_id, self.candle)
                if md.init() != pc.MD_Error_t.OK:
                    raise RuntimeError(f"drive {can_id} not answering (power? CAN cable? id?)")
                md.clearErrors()
                md.setMotionMode(pc.MotionMode_t.VELOCITY_PID)
                md.enable()
                self.mds.append(md)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def set(self, left, right):
        left, right = (max(-1.0, min(1.0, float(s))) for s in (left, right))
        self.log.append((round(left, 3), round(right, 3)))
        with self.lock:
            self.target = [left * self.max, right * self.max]
            if self.mock:
                self.now = list(self.target)  # no ramp in mock, so tests don't depend on timing

    def still(self):
        with self.lock:
            return max(abs(n) for n in self.now) < 0.05 * self.max

    def halt(self):
        self.log.append((0.0, 0.0))
        with self.lock:
            self.target, self.now = [0.0, 0.0], [0.0, 0.0]

    def _loop(self):
        while not self.done.is_set():
            with self.lock:
                step = self.acc * DT
                self.now = [n + max(-step, min(step, t - n)) for n, t in zip(self.now, self.target)]
                now = list(self.now)
            for md, s, v in zip(self.mds, self.sign, now):
                md.setTargetVelocity(s * v)  # rad/s at the rotor = at the wheel (direct drive)
            time.sleep(DT)

    def close(self):
        self.halt()
        time.sleep(3 * DT)  # let the loop send the zero
        self.done.set()
        self.thread.join()
        for md in self.mds:
            md.disable()


def ping(cfg):
    pc, candle = _candle(cfg)
    ids = pc.discoverMDs(candle)
    print("drives:", list(ids) or "none (power on? CANdle plugged in? udev rule?)")
    for i in ids:
        pc.MD(i, candle).blink()


def test(cfg):
    w = Wheels(cfg)
    try:
        for name, l, r in (("left", 0.2, 0), ("right", 0, 0.2), ("both", 0.2, 0.2)):
            print(f"{name} forward")
            w.set(l, r)
            time.sleep(1.0)
            w.set(0, 0)
            time.sleep(0.8)
        print("wrong wheel moved -> swap left_id/right_id; wheel went backwards -> flip its *_sign")
    finally:
        w.close()


def jog(cfg, speed=0.3):
    import termios
    import tty
    keys = {"w": (speed, speed), "s": (-speed, -speed), "a": (-speed, speed), "d": (speed, -speed), " ": (0, 0)}
    w, fd = Wheels(cfg), sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(__doc__.splitlines()[4].strip())
    try:
        tty.setcbreak(fd)
        while (k := sys.stdin.read(1)) != "q":
            if k in keys:
                w.set(*keys[k])
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        w.close()


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd not in ("ping", "test", "jog"):
        sys.exit(__doc__)
    {"ping": ping, "test": test, "jog": jog}[cmd](load_config())
