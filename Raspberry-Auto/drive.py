"""Driving base: follow the AprilTag route in config.json with the front camera.

    python drive.py run [--loop]     drive the route (once, or forever)
    python drive.py tag ID [STOP_PX] drive to one tag
    python drive.py selftest         logic check with mock wheels and synthetic frames (no hardware)

Behaviour
  steering   hybrid: small heading error -> arc, both wheels forward and the inner one slowed;
             big error -> spin in place (wheels opposite). Hysteresis between the two.
  approach   drive at the tag, slow down as it grows, stop when it looks stop_px wide.
  lost tag   gone for less than lost_grace_s: keep going. Longer: stop, then turn toward the side it was
             last seen in steps (search_step_s turning, search_look_s still), give up after one full turn
             (wheel encoders, track_m / wheel_radius_m) or search_timeout.
  obstacle   the corridor in front stops looking like floor for obstacle_frames frames -> halt
             (no ramp) and wait until it is clear again; give up after obstacle_timeout.
             Off during the final slow_px of a tag approach (the tag's station fills the corridor).

Route steps:  {"tag": 1, "stop_px": 150}       drive to AprilTag 1 until it looks 150 px wide
              {"drive": [left, right, seconds]} blind timed move, wheel speeds -1..1 (obstacle check if forward)
              {"wait": seconds}
              {"cmd": "shell command"}         run it and wait, e.g. the arm Pi's sort cycle over ssh
"""
import math
import subprocess
import sys
import time

from motors import MOCK, Wheels, load_config
from vision import Camera, Obstacles, find_tags


def mix(cfg, v, err, pivoting):
    """(left, right), pivoting. err: -1 target at the left image edge .. +1 right edge."""
    pivoting = abs(err) > (cfg["pivot_exit"] if pivoting else cfg["pivot_enter"])
    if pivoting:
        s = cfg["spin_speed"] if err > 0 else -cfg["spin_speed"]
        return (s, -s), True
    inner = v * max(0.0, 1 - cfg["steer_gain"] * abs(err))
    return ((v, inner) if err > 0 else (inner, v)), False


class Guard:
    """Obstacle stop with debouncing: blocked for n frames -> halt; clear for n frames -> go on."""

    def __init__(self, cfg, wheels):
        self.cfg, self.wheels, self.obs = cfg, wheels, Obstacles(cfg)
        self.n, self.bad, self.good, self.stopped_at = cfg["obstacle_frames"], 0, 0, None

    def check(self, img, ignore=()):
        """True while the car must stand still."""
        if not self.cfg["obstacle_check"]:
            return False
        if self.obs.hist is None:
            self.obs.learn(img)  # no floor.npy: the first frame of the run is taken as clear floor
        blocked, frac = self.obs.blocked(img, ignore)
        self.bad, self.good = (self.bad + 1, 0) if blocked else (0, self.good + 1)
        if self.stopped_at is None and self.bad >= self.n:
            self.wheels.halt()
            self.stopped_at = time.monotonic()
            print(f"  obstacle ({frac:.0%} of the corridor is not floor): stopped")
        elif self.stopped_at is not None and self.good >= self.n:
            print("  path clear")
            self.stopped_at = None
        if self.stopped_at is not None and time.monotonic() - self.stopped_at > self.cfg["obstacle_timeout"]:
            raise TimeoutError(f"blocked for {self.cfg['obstacle_timeout']} s")
        return self.stopped_at is not None


class Aborted(Exception):
    """The STOP button (abort event) was pressed."""


def _check_abort(abort, wheels):
    if abort is not None and abort.is_set():
        wheels.halt()
        raise Aborted()


def approach(cfg, cam, wheels, guard, tag_id, stop_px, timeout=90, abort=None, search_side=-1):
    """Drive to tag_id, or to any one of a list of ids (a station with several tags): the first one
    seen (the biggest if several) is kept for the whole approach, so the car never swaps targets midway.
    search_side: which way to turn if no tag is in view before one was ever seen (-1 left, +1 right)."""
    ids = [tag_id] if isinstance(tag_id, int) else list(tag_id)
    locked = None
    name = lambda: str(locked) if locked is not None else "/".join(map(str, ids))
    end = time.monotonic() + timeout
    # slow-down zone: slow_frac of stop_px when set (scales with the stop distance), else slow_px
    slow_px = cfg["slow_frac"] * stop_px if "slow_frac" in cfg else cfg["slow_px"]
    pivoting, last_side, lost_at, searching, search_t0 = False, search_side, None, False, 0.0
    step, look = cfg.get("search_step_s", 0), cfg.get("search_look_s", 0)
    # one full turn in place = each wheel rolls a whole circle of radius track/2: searching longer only repeats it
    full_turn = 2 * math.pi * cfg["track_m"] / 2 / cfg["wheel_radius_m"] if cfg.get("track_m") and cfg.get("wheel_radius_m") else None
    search_pos = None
    last_tag, logged = None, 0.0
    while time.monotonic() < end:
        _check_abort(abort, wheels)
        img = cam.frame()
        tags = find_tags(img)
        if locked is None:
            seen = [i for i in ids if i in tags]
            locked = max(seen, key=lambda i: tags[i][2]) if seen else None
        tag = tags.get(locked)
        # final slow approach: whatever the tag hangs on fills the corridor, that's the goal, not an obstacle
        docking = tag is not None and tag[2] >= stop_px - slow_px
        if not docking and guard.check(img, [t[3] for t in tags.values()]):
            continue
        if tag is None:
            now = time.monotonic()
            if lost_at is None:
                lost_at = now
            if now - lost_at < cfg.get("lost_grace_s", 0) and not pivoting:
                continue  # a few missed frames (motion blur, a far tag): keep driving as before, but never keep spinning
            if now - lost_at > cfg["search_timeout"]:
                break
            if not searching:
                if not wheels.still():
                    wheels.set(0, 0)  # brake to a standstill first, then spin
                    continue
                if last_tag is not None:
                    x0, y0, x1, y1 = last_tag[3]
                    h, margin = img.shape[0], 0.06 * img.shape[0]
                    if y0 < margin or y1 > h - margin:
                        # it left through the top or bottom: turning cannot bring it back
                        where = "top" if y0 < margin else "bottom"
                        raise TimeoutError(f"tag {name()} left the {where} of the image at {last_tag[2]:.0f} px: "
                                           f"hang it at the camera's height")
                print(f"  tag {name()} not in view: search {'right' if last_side > 0 else 'left'}")
                searching, search_t0, search_pos = True, now, list(wheels.pos)
            turned = abs((wheels.pos[0] - search_pos[0]) - (wheels.pos[1] - search_pos[1])) / 2
            if full_turn and turned >= full_turn:
                wheels.set(0, 0)
                raise TimeoutError(f"tag {name()} not in view after a full turn")
            s = cfg["search_speed"] * last_side
            # in steps: turn a little, then stand still and look. A spinning camera blurs a small tag away.
            turning = not step or (now - search_t0) % (step + look) < step
            wheels.set(s, -s) if turning else wheels.set(0, 0)
            continue
        lost_at, searching, last_tag = None, False, tag
        cx, cy, side, _ = tag
        if side >= stop_px:
            wheels.set(0, 0)
            print(f"  tag {name()}: {side:.0f} px >= {stop_px:.0f}: arrived")
            return
        err = (cx - img.shape[1] / 2) / (img.shape[1] / 2)
        last_side = 1 if err > 0 else -1
        # full speed far away, down to min_speed over the last slow_px of growth
        v = cfg["drive_speed"] * max(cfg["min_speed"], min(1.0, (stop_px - side) / slow_px))
        (left, right), pivoting = mix(cfg, v, err, pivoting)
        wheels.set(left, right)
        if time.monotonic() - logged > 0.5:  # a trace in the log, to see afterwards what the car saw and did
            logged = time.monotonic()
            print(f"  tag {name()}: {side:.0f} px at x={cx:.0f} y={cy:.0f} err={err:+.2f} -> "
                  f"{'spin' if pivoting else 'arc'} L={left:+.2f} R={right:+.2f}")
    wheels.set(0, 0)
    raise TimeoutError(f"tag {name()} not reached")


def timed(cam, wheels, guard, left, right, secs, abort=None):
    """Blind move; forward moves still stop for obstacles (the paused time doesn't count)."""
    left_s = secs
    while left_s > 0:
        _check_abort(abort, wheels)
        t = time.monotonic()
        if left > 0 and right > 0 and guard.check(cam.frame()):
            continue
        wheels.set(left, right)
        time.sleep(0.05)
        left_s -= time.monotonic() - t
    wheels.set(0, 0)


def run(cfg, cam, wheels, loop=False):
    guard = Guard(cfg, wheels)
    while True:
        for step in cfg["route"]:
            print("step", step)
            if "tag" in step:
                approach(cfg, cam, wheels, guard, step["tag"], step["stop_px"])
            elif "drive" in step:
                timed(cam, wheels, guard, *step["drive"])
            elif "wait" in step:
                time.sleep(step["wait"])
            elif "cmd" in step:
                subprocess.run(step["cmd"], shell=True, check=True)
        if not loop:
            return


def selftest():
    import cv2
    import numpy as np
    cfg = load_config()

    # steering: small error arcs (both forward, inner slower), big error spins, hysteresis in between
    (l, r), p = mix(cfg, 0.5, 0.1, False)
    assert l == 0.5 and 0 < r < 0.5 and not p, (l, r)
    (l, r), p = mix(cfg, 0.5, -0.1, False)
    assert r == 0.5 and 0 < l < 0.5
    (l, r), p = mix(cfg, 0.5, 0.9, False)
    assert l > 0 > r and p
    mid = (cfg["pivot_enter"] + cfg["pivot_exit"]) / 2
    assert mix(cfg, 0.5, mid, True)[1] and not mix(cfg, 0.5, mid, False)[1], "hysteresis"

    w, h = cfg["camera_size"]
    grey = lambda: np.full((h, w, 3), 128, np.uint8)
    tag = cv2.cvtColor(cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 1, 100), cv2.COLOR_GRAY2BGR)
    small, big, box = grey(), grey(), grey()
    small[100:180, 360:440] = 255  # white quiet zone, tag a bit right of centre (err 0.25: arc, not spin)
    small[108:172, 368:432] = cv2.resize(tag, (64, 64))
    big[40:440, 120:520] = 255
    big[60:420, 140:500] = cv2.resize(tag, (360, 360))
    box[int(0.75 * h):, int(0.35 * w):int(0.65 * w)] = (0, 0, 255)  # red box in the corridor

    class FakeCam:
        def __init__(self, frames):
            self.frames = frames

        def frame(self):
            return self.frames.pop(0)

    # pinned: the 64 px tag must be outside the docking zone, and the red box (~60% of the corridor) must stop the car
    cfg.update(obstacle_frames=2, search_timeout=5, slow_px=80, obstacle_frac=0.35, obstacle_check=True, lost_grace_s=0)
    cfg.pop("slow_frac", None)
    wheels = Wheels(cfg, mock=True)
    guard = Guard(cfg, wheels)
    guard.obs.hist = None  # learn from the first (clear) test frame, not a floor.npy lying around
    frames = [small, grey(), box, box, small, small, big]
    approach(cfg, FakeCam(frames), wheels, guard, 1, stop_px=200)
    log = wheels.log
    assert len(log) == 6 and frames == [], log
    assert log[0][0] > log[0][1] > 0, f"tag right -> arc right (inner wheel slowed): {log}"
    assert log[1] == (0.0, 0.0), f"tag lost -> stop first: {log}"
    assert log[2][0] > 0 > log[2][1], f"then spin toward where it was last seen (right): {log}"
    assert log[3] == (0.0, 0.0), f"obstacle for 2 frames -> halt: {log}"
    assert log[4][0] > log[4][1] > 0, f"clear for 2 frames + tag back -> arc again: {log}"
    assert log[5] == (0.0, 0.0), f"tag big enough -> stop: {log}"
    wheels.close()
    print("selftest ok")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "selftest":
        selftest()
        sys.exit()
    if cmd not in ("run", "tag"):
        sys.exit(__doc__)
    cfg = load_config()
    cam = Camera(None if MOCK else cfg["camera"], tuple(cfg["camera_size"]), cfg.get("camera_flip", False))
    wheels = Wheels(cfg)
    try:
        if cmd == "run":
            run(cfg, cam, wheels, loop="--loop" in sys.argv)
        else:
            stop = int(sys.argv[3]) if len(sys.argv) > 3 else cfg["stop_px"]
            approach(cfg, cam, wheels, Guard(cfg, wheels), int(sys.argv[2]), stop)
    finally:
        wheels.close()  # also on Ctrl-C: zero speed, then the drives are disabled
