"""SO-101 (camera arm) over its USB servo adapter, trimmed from dorm_keeper/ramie.py.

Angles are degrees from the encoder centre: (ticks - 2048) * 360 / 4096.

    python so101.py save look     torque OFF (hold the arm!), place it by hand, Enter -> saved in config.json
    python so101.py go look       move to a saved pose (names used: look, stow, drive)
    python so101.py where
"""
import json
import os
import sys
import time

ADDR_TORQUE_ENABLE, ADDR_ACC, ADDR_PRESENT_POSITION = 40, 41, 56
TICKS_PER_DEG = 4096 / 360
IDS = {"pan": 1, "lift": 2, "elbow": 3, "wflex": 4, "wroll": 5, "grip": 6}
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")


def to_deg(t):
    return round((t - 2048) / TICKS_PER_DEG, 1)


def to_ticks(d):
    return max(0, min(4095, int(round(d * TICKS_PER_DEG + 2048))))


class SO101:
    def __init__(self, port=None, mock=False, speed=30, acc=20):
        self.mock, self.speed, self.acc = mock, speed, acc
        self._pose = {j: 0.0 for j in IDS}
        if mock:
            return
        from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

        self._ok = COMM_SUCCESS
        self.port = PortHandler(port or self._find_port())
        if not self.port.openPort() or not self.port.setBaudRate(1_000_000):
            raise RuntimeError(f"cannot open {self.port.port_name} at 1 Mbps")
        self.ph = PacketHandler(0)
        missing = [j for j, i in IDS.items() if self.ph.ping(self.port, i)[1] != self._ok]
        if missing:
            raise RuntimeError(f"SO-101 servos not answering: {missing} (servo power? bus cable?)")

    @staticmethod
    def _find_port():
        from serial.tools import list_ports

        ports = [p.device for p in list_ports.comports() if p.vid == 0x1A86]  # CH343/CH340 adapter
        if not ports:
            raise RuntimeError("SO-101 servo adapter (VID 1A86) not found - USB cable?")
        return ports[0]

    def joints(self):
        if self.mock:
            return dict(self._pose)
        out = {}
        for j, i in IDS.items():
            val, res, _ = self.ph.read2ByteTxRx(self.port, i, ADDR_PRESENT_POSITION)
            if res != self._ok:
                raise RuntimeError(f"cannot read servo {i}")
            out[j] = to_deg(-(val & 0x7FFF) if val & 0x8000 else val)
        return out

    def move(self, pose, speed=None, tol=3.0, settle_tol=10.0, timeout=15.0):
        """Move and wait. The SO-101 servos stop a few degrees short under load (P=16), so a joint
        that stopped within settle_tol counts as arrived; further away = blocked -> error."""
        if self.mock:
            self._pose.update(pose)
            print("  [so101]", pose)
            return
        self.torque(True)
        for j, deg in pose.items():
            self._goal(IDS[j], deg, speed or self.speed)
        end, last, still = time.time() + timeout, None, None
        arm = [j for j in pose if j != "grip"]
        while time.time() < end:
            now = self.joints()
            err = max((abs(now[j] - pose[j]) for j in arm), default=0)
            if err <= tol:
                return
            if last and all(abs(now[j] - last[j]) < 0.3 for j in arm):
                still = still or time.time()
                if time.time() - still > 0.5:
                    if err <= settle_tol:
                        return
                    raise RuntimeError(f"SO-101 stopped {err:.0f} deg short (obstacle?)")
            else:
                still = None
            last = now
            time.sleep(0.05)
        raise TimeoutError("SO-101 did not reach the pose")

    def _goal(self, sid, deg, speed):
        t, spd = to_ticks(deg), max(1, int(speed * TICKS_PER_DEG))  # speed 0 would be full speed
        data = [self.acc, t & 0xFF, t >> 8, 0, 0, spd & 0xFF, spd >> 8]
        if self.ph.writeTxRx(self.port, sid, ADDR_ACC, len(data), data)[0] != self._ok:
            raise RuntimeError(f"cannot command servo {sid}")

    def torque(self, on):
        if self.mock or getattr(self, "_torque", None) == on:
            return
        if on:  # goal = where it is now, or the servos jerk back to their old goal
            for j, deg in self.joints().items():
                self._goal(IDS[j], deg, 20)
        for i in IDS.values():
            self.ph.write1ByteTxRx(self.port, i, ADDR_TORQUE_ENABLE, int(on))
        self._torque = on

    def go(self, name):
        self.move(json.load(open(CONFIG))["so101_poses"][name])


if __name__ == "__main__":
    cmd, name = (sys.argv[1:] + ["", ""])[:2]
    arm = SO101(json.load(open(CONFIG)).get("so101_port"), mock=os.environ.get("MOCK") == "1")
    if cmd == "where":
        print(arm.joints())
    elif cmd == "go" and name:
        arm.go(name)
    elif cmd == "save" and name:
        # torque off first so the arm can be posed by hand; the current reading becomes the goal on torque on
        arm.torque(False)
        input(f"torque OFF - hold the arm, place it in '{name}', press Enter")
        cfg = json.load(open(CONFIG))
        cfg["so101_poses"][name] = arm.joints()
        json.dump(cfg, open(CONFIG, "w"), indent=2)
        arm.torque(True)  # hold it there
        print("saved", name, cfg["so101_poses"][name])
    else:
        print(__doc__)
