"""Two MAB MA-D-GL40 KV70 wheel actuators (built-in MD driver) on a CANdle USB-FDCAN dongle, velocity mode.

    python motors.py ping      list the drive ids on the bus and blink them
    python motors.py test      left wheel, right wheel, both: slowly forward 1 s each (wheels OFF the ground!)
    python motors.py jog       keyboard drive: w/s forward/back, a/d spin, z/c arc left/right, space stop, q quit

Needs `pip install candlesdk` (imports as pyCandle). MOCK=1 runs without hardware.
"""
import gc
import json
import os
import sys
import threading
import time

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
MOCK = os.environ.get("MOCK") == "1"
DT = 0.02  # 50 Hz: well inside the MD watchdog, so the drives stay enabled only while this program runs
LINK_FAILS = 10  # failed ticks in a row (0.2 s) before the link counts as lost and gets re-attached


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
    halt() skips the ramp (obstacle stop). power(False) disables the drives while parked (no holding
    current); volts is the battery voltage, read about once a second; pos is how far each wheel has turned
    (rad, + = car forward) from the encoders, integrated from the speed in mock. All CAN traffic stays in
    that thread.

    A lost link (the CANdle's USB dropped for a moment, a drive browned out): after LINK_FAILS failed ticks
    in a row the wheels go to zero, link_ok turns False and the thread re-attaches the CANdle every second,
    setting the drives up again. keep_trying=True also starts without the drives (the panel); the command line
    tools keep failing at once."""

    def __init__(self, cfg, mock=MOCK, keep_trying=False):
        self.cfg = cfg
        self.max, self.acc = cfg["max_wheel_rad_s"], cfg["accel_rad_s2"]
        self.sign = (cfg["left_sign"], cfg["right_sign"])  # one motor is mirrored, so one sign is -1
        self.target, self.now = [0.0, 0.0], [0.0, 0.0]
        self.mock, self.log = mock, []  # log: the last set() calls, for the selftest
        self.lock, self.done = threading.Lock(), threading.Event()
        self.enabled = self.want_enabled = True
        self.volts, self.pos = None, [0.0, 0.0]
        self.mds, self.candle, self.link_ok, self.fails = [], None, mock, 0
        if not mock:
            if cfg["left_id"] is None or cfg["right_id"] is None:
                raise SystemExit("set left_id / right_id in config.json (python motors.py ping)")
            try:
                self._connect()
            except RuntimeError as e:
                if not keep_trying:
                    raise
                print(f"drives not ready ({e}): retrying every second")
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _connect(self):
        """Attach the CANdle and set both drives up (gains, velocity mode, enabled if wanted). RuntimeError if not."""
        gc.collect()  # a dropped CANdle still holding its USB interface would make this attach fail
        pc, candle = _candle(self.cfg)  # RuntimeError "Could not connect USB device!" while it is unplugged
        mds = []
        try:
            for can_id in (self.cfg["left_id"], self.cfg["right_id"]):
                md = pc.MD(can_id, candle)
                if md.init() != pc.MD_Error_t.OK:
                    raise RuntimeError(f"drive {can_id} not answering (power? CAN cable? id?)")
                md.clearErrors()
                md.setMaxTorque(self.cfg["max_torque"])
                # the factory gains are too soft to turn a loaded car. Written one register at a time:
                # md.setVelocityPIDparam() returns OK but leaves the values unchanged (candlesdk 1.5.0)
                for reg, val in zip(("motorVelPidKp", "motorVelPidKi", "motorVelPidKd", "motorVelPidWindup"), self.cfg["vel_pid"]):
                    pc.writeRegisterFloat(md, reg, val)
                md.setMotionMode(pc.MotionMode_t.VELOCITY_PID)
                md.enable() if self.want_enabled else md.disable()
                mds.append(md)
        except Exception:
            mds.clear()  # the drive objects point at the CANdle: let them go first
            del candle
            raise
        self.candle, self.mds = candle, mds
        self.enabled, self.fails, self.link_ok = self.want_enabled, 0, True

    def _drop(self, why):
        print(f"drives lost ({why}): wheels stopped, reconnecting every second")
        self.halt()
        self.mds = []  # the drive objects point at the CANdle: let them go first
        self.candle = None
        self.link_ok, self.volts = False, None

    def set(self, left, right):
        left, right = (max(-1.0, min(1.0, float(s))) for s in (left, right))
        self.log.append((round(left, 3), round(right, 3)))
        if len(self.log) > 2000:  # the panel runs for hours: keep only the recent calls
            del self.log[:1000]
        with self.lock:
            self.target = [left * self.max, right * self.max]
            if self.mock:
                self.now = list(self.target)  # no ramp in mock, so tests don't depend on timing

    def still(self):
        with self.lock:
            return max(abs(n) for n in self.now) < 0.05 * self.max

    def speed(self):
        """The ramped speed now being sent, rad/s per wheel (+ = car forward)."""
        with self.lock:
            return list(self.now)

    def halt(self):
        self.log.append((0.0, 0.0))
        with self.lock:
            self.target, self.now = [0.0, 0.0], [0.0, 0.0]

    def power(self, on):
        """Enable (drive) or disable (parked) the drives; returns once the loop has done it."""
        if not on:
            self.halt()
        self.want_enabled = on
        while self.enabled != on and self.thread.is_alive():
            time.sleep(DT)

    def _loop(self):
        pc = None if self.mock else __import__("pyCandle")
        tick, retry_at, told = 0, 0.0, False
        while not self.done.is_set():
            if not self.mock and not self.mds:  # no link: keep the wheels at zero and try again every second
                self.enabled = self.want_enabled  # nothing to switch, and power() must not wait for it
                with self.lock:
                    self.target, self.now = [0.0, 0.0], [0.0, 0.0]
                if time.monotonic() >= retry_at:
                    retry_at = time.monotonic() + 1.0
                    try:
                        self._connect()
                        print("drives connected")
                        told = False
                    except Exception as e:
                        if not told:
                            print(f"drives not reachable yet: {e}")
                            told = True
                time.sleep(DT)
                continue
            try:
                ok = self._tick(pc, tick)
            except Exception as e:  # pyCandle raising instead of returning an error: the same, a broken link
                print(f"drive call failed: {e!r}")
                ok = False
            if not self.mock:
                self.fails = 0 if ok else self.fails + 1
                if self.fails >= LINK_FAILS:
                    self._drop(f"{self.fails} failed calls in a row")
            tick += 1
            time.sleep(DT)

    def _tick(self, pc, tick):
        """One 50 Hz step: switch enabled/disabled, ramp, send the speeds, read the encoders (and the battery
        now and then). False if a drive call failed. A method of its own, so no drive object outlives it in the
        loop's variables: a dropped CANdle must really be freed, or its USB interface stays claimed."""
        ok = True
        if self.enabled != self.want_enabled:
            for md in self.mds:
                if self.want_enabled:
                    md.clearErrors()
                    md.setMotionMode(pc.MotionMode_t.VELOCITY_PID)  # disable() clears it: "Motion mode not set"
                    md.enable()
                else:
                    md.setTargetVelocity(0.0)
                    md.disable()
            self.enabled = self.want_enabled
        with self.lock:
            step = self.acc * DT
            self.now = [n + max(-step, min(step, t - n)) for n, t in zip(self.now, self.target)]
            now = list(self.now)
        if self.enabled:
            for md, s, v in zip(self.mds, self.sign, now):
                ok &= md.setTargetVelocity(s * v) == pc.MD_Error_t.OK  # rad/s at the rotor = at the wheel
        if self.mds:
            for k, (md, s) in enumerate(zip(self.mds, self.sign)):
                p, err = md.getPosition()
                if err == pc.MD_Error_t.OK:
                    self.pos[k] = s * p
                else:
                    ok = False
        else:
            self.pos = [p + v * DT for p, v in zip(self.pos, now)]
        if self.mds and tick % 50 == 0:
            v, err = pc.readRegisterFloat(self.mds[0], "dcBusVoltage")
            if err == pc.MD_Error_t.OK:
                self.volts = v
        return ok

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
    keys = {"w": (speed, speed), "s": (-speed, -speed), "a": (-speed, speed), "d": (speed, -speed),
            "z": (0, speed), "c": (speed, 0), " ": (0, 0)}  # z/c: pivot on the stopped wheel, less floor scrub
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
