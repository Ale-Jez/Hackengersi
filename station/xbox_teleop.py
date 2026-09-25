"""Xbox controller jog for the RoArm (Windows, stdlib + inputs library).

    python station/xbox_teleop.py             # real arm
    python station/xbox_teleop.py --selftest  # test controller logic

Analog sticks send events only on change, so the controller is polled for their values.
Button/trigger events are delivered as they happen.
"""
import sys
import threading
import time
from collections import defaultdict

from inputs import get_gamepad
from roarm import RoArm

LIMITS = {"b": (-3.14, 3.14), "s": (-1.57, 1.57), "e": (0.0, 3.0), "t": (-1.57, 1.57), "r": (-3.14, 3.14)}
GRIP_LIMITS = (1.0, 3.5)
SPD_MIN, SPD_MAX = 100, 900
LEAD_STEPS = 3

HELP = """\
Left stick:  base (X), shoulder (Y)
Right stick: roll (X), elbow (Y)
LT / RT:     wrist left / right
A:           gripper close
B:           gripper open
X:           stop and hold
Y:           print pose
D-pad:       speed (up/down), step size (left/right)
Start:       quit

Analog sticks smoothly control the arm. Buttons are discrete.
"""


def clamp(v, lo_hi):
    return min(max(v, lo_hi[0]), lo_hi[1])


class XboxTeleop:
    def __init__(self, arm):
        self.arm = arm
        self.step, self.spd = 0.05, 500
        self.stopped = False
        self.quit_ = False

        p = arm.pose()
        self.tgt = {k: p[k] for k in LIMITS}
        self.grip = arm._grip or 1.57

        # Analog stick values (0.0-1.0, deadzone applied)
        self.sticks = defaultdict(float)  # "LX", "LY", "RX", "RY"
        self.last_move = 0.0

        print(HELP)

    def update_sticks(self):
        """Read analog sticks once (called before each move)."""
        try:
            events_processed = 0
            for event in get_gamepad():
                # Analog stick/trigger events
                if event.ev_type == "Absolute":
                    # Xbox controller sends values 0-255 for sticks/triggers
                    val = event.state / 32768.0 - 1.0  # Convert to -1.0 to 1.0
                    # Apply deadzone
                    if abs(val) < 0.1:
                        val = 0.0
                    self.sticks[event.code] = val
                    events_processed += 1
                # Button events
                elif event.ev_type == "Key":
                    self.handle_button(event.code, event.state)
                # Don't process more than 10 events per frame to avoid buffering
                if events_processed > 10:
                    break
        except StopIteration:
            pass

    def handle_button(self, btn, pressed):
        """Handle button press/release."""
        if not pressed:
            return  # Only handle press, not release

        if btn == "BTN_START":
            self.quit_ = True
        elif btn == "BTN_X":
            self.stopped = not self.stopped
            if not self.stopped:
                try:
                    meas = self.arm.pose()
                    self.tgt = {k: meas[k] for k in LIMITS}
                except RuntimeError:
                    pass
            else:
                try:
                    self.arm.stop()
                except RuntimeError as e:
                    print(f"could not send hold: {e}")
        elif btn == "BTN_Y":
            try:
                print("\n", {k: round(v, 3) for k, v in self.arm.pose().items()})
            except RuntimeError as e:
                print(f"\n{e}")
        elif btn == "BTN_A":  # Gripper close
            self.grip = clamp(self.grip + 0.2, GRIP_LIMITS)
            self.arm.gripper(self.grip)
            print(f"\ngripper close -> {self.grip:.2f}")
        elif btn == "BTN_B":  # Gripper open
            self.grip = clamp(self.grip - 0.2, GRIP_LIMITS)
            self.arm.gripper(self.grip)
            print(f"\ngripper open -> {self.grip:.2f}")
        elif btn == "BTN_DPAD_UP":
            self.spd = min(self.spd + 100, SPD_MAX)
            print(f"\nspeed -> {self.spd}")
        elif btn == "BTN_DPAD_DOWN":
            self.spd = max(self.spd - 100, SPD_MIN)
            print(f"\nspeed -> {self.spd}")
        elif btn == "BTN_DPAD_LEFT":
            self.step = max(self.step / 2, 0.005)
            print(f"\nstep -> {self.step:.3f}")
        elif btn == "BTN_DPAD_RIGHT":
            self.step = min(self.step * 2, 0.2)
            print(f"\nstep -> {self.step:.3f}")

    def run(self):
        """Main loop."""
        while True:
            self.update_sticks()

            if self.quit_:
                print("bye")
                return

            if self.stopped:
                time.sleep(0.02)
                continue

            try:
                meas = self.arm.pose()
            except RuntimeError:
                meas = None
                time.sleep(0.02)
                continue

            # Map sticks to joints with deadzone
            # Left stick: base (X), shoulder (Y)
            # Right stick: roll (X), elbow (Y)
            lx = self.sticks.get("ABS_X", 0.0)  # -1 to 1
            ly = self.sticks.get("ABS_Y", 0.0)
            rx = self.sticks.get("ABS_RX", 0.0)
            ry = self.sticks.get("ABS_RY", 0.0)
            # Triggers (0 to 1)
            lt = self.sticks.get("ABS_Z", 0.0) / 255.0
            rt = self.sticks.get("ABS_RZ", 0.0) / 255.0

            lead = LEAD_STEPS * self.step
            dirty = False

            # Base (left stick X, left is -)
            if abs(lx) > 0.1:
                target = clamp(self.tgt["b"] + lx * self.step, (meas["b"] - lead, meas["b"] + lead))
                target = clamp(target, LIMITS["b"])
                if target != self.tgt["b"]:
                    self.tgt["b"] = target
                    dirty = True

            # Shoulder (left stick Y, up is -)
            if abs(ly) > 0.1:
                target = clamp(self.tgt["s"] - ly * self.step, (meas["s"] - lead, meas["s"] + lead))
                target = clamp(target, LIMITS["s"])
                if target != self.tgt["s"]:
                    self.tgt["s"] = target
                    dirty = True

            # Roll (right stick X)
            if abs(rx) > 0.1:
                target = clamp(self.tgt["r"] + rx * self.step, (meas["r"] - lead, meas["r"] + lead))
                target = clamp(target, LIMITS["r"])
                if target != self.tgt["r"]:
                    self.tgt["r"] = target
                    dirty = True

            # Elbow (right stick Y, up is -)
            if abs(ry) > 0.1:
                target = clamp(self.tgt["e"] - ry * self.step, (meas["e"] - lead, meas["e"] + lead))
                target = clamp(target, LIMITS["e"])
                if target != self.tgt["e"]:
                    self.tgt["e"] = target
                    dirty = True

            # Wrist (triggers, LT is -, RT is +)
            if abs(lt - rt) > 0.05:
                wrist_input = (rt - lt) * self.step
                target = clamp(self.tgt["t"] + wrist_input, (meas["t"] - lead, meas["t"] + lead))
                target = clamp(target, LIMITS["t"])
                if target != self.tgt["t"]:
                    self.tgt["t"] = target
                    dirty = True

            now = time.monotonic()
            if dirty and (now - self.last_move >= 0.1):
                try:
                    self.arm.move(self.tgt, spd=self.spd, acc=30, wait=False)
                    self.last_move = now
                    print("\r " + "  ".join(f"{k}={v:+.2f}" for k, v in self.tgt.items())
                          + f"  g={self.grip:.2f}  ", end="", flush=True)
                except RuntimeError as e:
                    print(f"\n{e}; retrying")

            time.sleep(0.01)


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        print("selftest: connect an Xbox controller and move sticks around")
        print("or press --selftest with a mock arm (not implemented yet)")
        sys.exit(0)
    else:
        with RoArm() as arm:
            try:
                teleop = XboxTeleop(arm)
                teleop.run()
            except KeyboardInterrupt:
                print("\nbye")
