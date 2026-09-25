"""Keyboard jog for the RoArm, one key pair per joint (Windows console, stdlib only).

    python station/keyteleop.py             # real arm
    python station/keyteleop.py --selftest  # key logic against the mock arm

Hold a key for repeat. The target may lead the measured pose by at most LEAD_STEPS steps, so
the arm stops within ~0.2 s of releasing the key instead of chasing a runaway target. Sends are
capped at 10/s (the arm's command queue is small).
"""
import msvcrt
import sys
import time

from roarm import RoArm

HELP = """\
 A/D  base      (+ = left)         W/S  shoulder (+ = forward/down)
 R/F  elbow     (+ = down)         T/G  wrist
 Y/H  roll                         Z/X  gripper open / close
 [ ]  smaller / bigger step        +/-  slower / faster
 P    print measured pose          SPACE stop and hold the pose
 C    resume jogging               Q or Ctrl+C  quit
"""
JOG = {"a": ("b", 1), "d": ("b", -1), "w": ("s", 1), "s": ("s", -1), "r": ("e", 1),
       "f": ("e", -1), "t": ("t", 1), "g": ("t", -1), "y": ("r", 1), "h": ("r", -1)}
# rad, from the firmware's constrain() calls; elbow kept off its far end (the arm slid there)
LIMITS = {"b": (-3.14, 3.14), "s": (-1.57, 1.57), "e": (0.0, 3.0), "t": (-1.57, 1.57), "r": (-3.14, 3.14)}
GRIP_LIMITS = (1.0, 3.5)  # docs: 1.57 releases, 3.14 grabs; unverified on this arm
SPD_MIN, SPD_MAX = 100, 900  # steps/s
LEAD_STEPS = 3  # target may lead the measured joint by this many steps


def clamp(v, lo_hi):
    return min(max(v, lo_hi[0]), lo_hi[1])


def main(arm, kbhit=msvcrt.kbhit, getwch=msvcrt.getwch):
    step, spd, stopped, quit_ = 0.05, 500, False, False
    p = arm.pose()
    tgt = {k: p[k] for k in LIMITS}
    grip = arm._grip or 1.57  # measured g has read 0 regardless of the gripper, so don't trust it
    dirty = gdirty = False
    last = 0.0
    print(HELP)
    while True:
        try:
            meas = arm.pose()
        except RuntimeError:  # feedback paused: ignore jog keys rather than move blind
            meas = None
        while kbhit():
            c = getwch().lower()
            if c in ("\x00", "\xe0"):  # arrow/function key: swallow the second code
                getwch()
            elif c in ("q", "\x03"):
                quit_ = True
                break
            elif c == " ":
                stopped, dirty, gdirty = True, False, False  # drop anything not yet sent
                try:
                    arm.stop()  # hold the measured pose; not T:0, which freezes the arm for ~10 s
                except RuntimeError as e:
                    print(f"\n{e}; could not send the hold")
            elif c == "c":
                stopped = False
                if meas:
                    tgt = {k: meas[k] for k in LIMITS}
            elif c == "[":
                step = max(step / 2, 0.005)
                print(f"\nstep={step:.3f}  spd={spd}  ", end="", flush=True)
            elif c == "]":
                step = min(step * 2, 0.2)
                print(f"\nstep={step:.3f}  spd={spd}  ", end="", flush=True)
            elif c == "+":
                spd = min(spd + 100, SPD_MAX)
                print(f"\nstep={step:.3f}  spd={spd}  ", end="", flush=True)
            elif c == "-":
                spd = max(spd - 100, SPD_MIN)
                print(f"\nstep={step:.3f}  spd={spd}  ", end="", flush=True)
            elif c == "p":
                print("\n", {k: round(v, 3) for k, v in meas.items()} if meas else "no feedback")
            elif stopped:
                continue
            elif c in JOG:
                if not meas:
                    print(f"\nno feedback for jog key '{c}'")
                    continue
                j, d = JOG[c]
                lead = LEAD_STEPS * step
                v = clamp(tgt[j] + d * step, (meas[j] - lead, meas[j] + lead))
                v2 = clamp(v, LIMITS[j])
                if v != v2:
                    print(f"\n'{c}': {tgt[j]:.3f} -> {v:.3f} (clamped to {v2:.3f})")
                tgt[j] = v2
                dirty = True
            elif c in ("z", "x"):
                grip = clamp(grip + (4 * step if c == "x" else -4 * step), GRIP_LIMITS)
                gdirty = True
        now = time.monotonic()
        if (dirty or gdirty) and (quit_ or now - last >= 0.1):
            try:
                if dirty:
                    arm.move(tgt, spd=spd, acc=30, wait=False)
                if gdirty:
                    arm.gripper(grip)
                dirty = gdirty = False
                last = now
                print("\r " + "  ".join(f"{k}={v:+.2f}" for k, v in tgt.items())
                      + f"  g={grip:.2f}  step={step:.3f}   ", end="", flush=True)
            except RuntimeError as e:  # stale feedback: don't move blind, keep the keys, retry in 1 s
                print(f"\n{e}; retrying")
                last = now + 0.9
        if quit_:
            print()
            return
        time.sleep(0.01)


def selftest():
    # each group is one drain pass; the 20 'a's must be capped by the lead limit (the mock arm's
    # measured pose only updates on send) and ' ' must block the later 'a'; +- and [] are also tested
    groups = ["a" * 20 + "xxwfff+[-]", " a", "q"]
    cur = []

    def kbhit():
        if not cur and groups:
            cur.extend(groups.pop(0))
            return False  # nothing pending this pass
        return bool(cur)

    with RoArm(mock=True) as arm:
        main(arm, kbhit, lambda: cur.pop(0))
        p = arm.pose()
        assert abs(p["b"] - 0.15) < 1e-9, p   # capped at LEAD_STEPS x 0.05, not 20 x 0.05
        assert abs(p["s"] - 0.05) < 1e-9, p
        assert p["e"] == 0.0, p                # clamped at the lower limit
        assert abs(p["g"] - (1.57 + 0.4)) < 1e-9, p  # 2 x (4 x 0.05) closing
    print("selftest ok")


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        selftest()
    else:
        with RoArm() as arm:
            try:
                main(arm)
            except KeyboardInterrupt:  # Windows raises this instead of delivering '\x03'
                print("\nbye")
