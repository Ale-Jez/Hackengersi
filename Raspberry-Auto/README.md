# Raspberry Pi 5: driving base

Two MAB **MA-D-GL40 KV70** direct-drive actuators (one per wheel, each with its own MD driver) on a **CANdle** USB-to-CAN FD dongle, and one forward-looking USB camera. The Pi steers at AprilTags (36h11, see `../AprilTags`) and stops for obstacles.

| File | Does |
|---|---|
| `drive.py` | route driving: hybrid steering, tag approach, search, obstacle stop, `selftest` |
| `motors.py` | the two wheels over CANdle (velocity mode, speed ramp, watchdog feed), `ping` / `test` / `jog` |
| `vision.py` | newest-frame camera, AprilTag detection, floor-colour obstacle check, `floor` / `snap` |
| `config.json` | CAN ids, wheel signs, speeds, steering and obstacle tuning, the route |

## Behaviour

| Situation | What the car does |
|---|---|
| Tag slightly off centre (\|err\| < `pivot_enter`) | **Arc:** both wheels forward, the inner one slowed by `steer_gain × err` |
| Tag far off centre | **Spin in place** at `spin_speed` (wheels opposite) until \|err\| < `pivot_exit` |
| Getting close | Speed drops linearly over the last `slow_px` of tag growth (never below `min_speed`), stop at `stop_px` |
| Tag not in view | Brake to a standstill, then spin toward the side it was last seen; error after `search_timeout` s |
| Something in the corridor | After `obstacle_frames` frames: **halt at once** (no ramp), wait until clear for as many frames; error after `obstacle_timeout` s. Off during the final `slow_px` of an approach, where the tag's own station fills the corridor |
| Program dies / Ctrl-C | Speed goes to 0 and the drives are disabled; if the Pi hangs, the MD watchdog stops the motors |

`err` runs from -1 (tag at the left image edge) to +1 (right edge). Every speed in the config is a fraction (-1..1) of `max_wheel_rad_s`, and the ramp limits changes to `accel_rad_s2`.

## Setup

```sh
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python drive.py selftest
```

- **CANdle:** plug it into USB, power the actuators, and follow MAB's CANdle setup (the udev rule, or run as root). The Python package `candlesdk` imports as `pyCandle`. The CANdle HAT (SPI) is not supported by its Python bindings yet, so use the USB dongle.
- **Camera:** find the index with `v4l2-ctl --list-devices`, then set `camera`.

## Bring-up (wheels off the ground first)

1. `python motors.py ping` prints the drive ids. Put them in `left_id` / `right_id`.
2. `python motors.py test` runs left, then right, then both, slowly forward. If the wrong wheel moves, swap the ids. If a wheel turns backwards, flip its `*_sign`.
3. Set the current / torque limit and the CAN watchdog on each drive with MAB's `candletool` (or MD tool). The GL40 is direct drive (about 0.25 Nm rated), so check that it can push the loaded can on your floor before tuning speed.
4. Put it on the floor: `python motors.py jog` (w/s/a/d, space to stop, q to quit).
5. Obstacles: point the car at clear floor, run `python vision.py floor`, then `python vision.py snap` with a box in front. Red pixels in the cyan corridor are "not floor". Tune `obstacle_roi` (fractions of the frame) and `obstacle_frac`.
6. Tags: `python drive.py tag 1 150`. If it zig-zags, lower `steer_gain`. If it spins too often, raise `pivot_enter`.
7. `python drive.py run`.

**Limits:** the obstacle check only looks at colour, so an obstacle the same colour as the floor is invisible, and so is anything outside the corridor. Strong light changes after `vision.py floor` need a re-learn. `max_wheel_rad_s` is 12 rad/s by default. The KV70 could go more than 10× faster at 24 V, so raise it carefully.
