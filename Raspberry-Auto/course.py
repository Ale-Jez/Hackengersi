"""Fixed course: 1 m forward, right 90 deg, 0.5 m forward, left 90 deg, then forward to AprilTag 1, stop 15 cm from it.

    python course.py          drive it (MOCK=1: no hardware)
    python course.py check    maths self-check

The blind legs are dead reckoning from time x wheel speed, so they drift. Calibrate in config.json:
  wheel_radius_m   drive a "1 m" leg, measure the real distance D, set wheel_radius_m *= D / 1.0
  track_m          wheel-to-wheel distance; if a "90 deg" turn does A deg, set track_m *= 90 / A
The last leg steers at the tag (drive.approach), so the 15 cm stop comes from the camera, not the clock.
"""
import math
import sys

from drive import Guard, approach, timed
from motors import MOCK, Wheels, load_config
from vision import Camera

STOP_CM = 15


def secs(cfg, metres, speed):
    """Time for a wheel at `speed` (fraction of max_wheel_rad_s) to roll `metres`. The ramp up and the ramp
    down after the stop roughly cancel out, so plain v*t is close enough."""
    return metres / (speed * cfg["max_wheel_rad_s"] * cfg["wheel_radius_m"])


def turn(cfg, cam, wheels, guard, degrees):
    """Spin in place, + = right. Each wheel rolls along an arc of radius track/2."""
    s = cfg["spin_speed"] if degrees > 0 else -cfg["spin_speed"]
    timed(cam, wheels, guard, s, -s, secs(cfg, math.radians(abs(degrees)) * cfg["track_m"] / 2, abs(s)))


def course(cfg, cam, wheels):
    guard, v = Guard(cfg, wheels), cfg["drive_speed"]
    print("forward 1 m")
    timed(cam, wheels, guard, v, v, secs(cfg, 1.0, v))
    print("right 90")
    turn(cfg, cam, wheels, guard, 90)
    print("forward 0.5 m")
    timed(cam, wheels, guard, v, v, secs(cfg, 0.5, v))
    print("left 90")
    turn(cfg, cam, wheels, guard, -90)
    stop_px = cfg["tag_focal_px"] * cfg["tag_size_cm"] / STOP_CM
    print(f"to tag 1, stop at {STOP_CM} cm ({stop_px:.0f} px)")
    approach(cfg, cam, wheels, guard, 1, stop_px)


if __name__ == "__main__":
    cfg = load_config()
    if sys.argv[1:] == ["check"]:
        c = dict(cfg, max_wheel_rad_s=10, wheel_radius_m=0.05, track_m=0.3)
        assert abs(secs(c, 1.0, 0.5) - 4.0) < 1e-9  # 0.5*10 rad/s * 5 cm = 0.25 m/s
        assert abs(secs(c, math.pi / 2 * 0.15, 0.5) - 0.3 * math.pi) < 1e-9  # quarter turn: 23.6 cm per wheel
        print("check ok")
        sys.exit()
    cam = Camera(None if MOCK else cfg["camera"], tuple(cfg["camera_size"]), cfg.get("camera_flip", False))
    wheels = Wheels(cfg)
    try:
        course(cfg, cam, wheels)
    finally:
        wheels.close()
