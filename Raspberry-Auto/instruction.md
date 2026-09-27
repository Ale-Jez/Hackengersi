# Instructions for whoever continues this code (human or LLM)

Read this first, then `README.md` (hardware, bring-up, tuning) and the docstrings at the top of each `.py`.

## What this is

A two-wheel car (Raspberry Pi 5, two MAB GL40 wheel drives on a CANdle USB dongle, CSI camera) that drives to AprilTags (36h11).

| File | Role |
|---|---|
| `motors.py` | `Wheels`: velocity control of both wheels, 50 Hz loop, speed ramp, watchdog. `ping` / `test` / `jog` |
| `vision.py` | `Camera` (newest frame), `find_tags`, `Obstacles` (floor-colour corridor check) |
| `drive.py` | Building blocks: `approach()` (steer to a tag, stop at `stop_px`), `timed()` (blind timed move with obstacle stop), `Guard`, `run()` for the `route` in config |
| `course.py` | Fixed course: 1 m forward, right 90°, 0.5 m forward, left 90°, then drive at tag 1 and stop 15 cm from it |
| `stream.py` | Live camera view at `http://<pi>:8000` |
| `app.py` | Station panel at `http://<pi>:8000` (buttons per station, manual driving, taught routes, 90° turns, CANdle auto-reconnect). Owns the camera and the CANdle while it runs |
| `config.json` | Every tunable number. Code never hard-codes a calibration value |

## The Pi

- Hostname `malina-auto`, user `hackengersi`, Debian 13 (trixie), aarch64.
- Network: on WiFi `4G-Gateway-7417` the Pi has a **static 192.168.32.144** (set 2026-09-27 on the Pi's NetworkManager profile, gateway/DNS 192.168.32.1). On the other WiFi networks (phone hotspots) it still uses DHCP; find it with "Finding the Pi" in `README.md`. `malina-auto.local` usually does not resolve from Windows.
- The phone hotspots `POCO X8 PRO` / `POCO X8 Pro` have a higher priority (10) than the gateway (5). If one is on when the Pi boots, the Pi joins it instead of the gateway, and .144 no longer applies.
- SSH uses key auth from the dev laptop (`ssh hackengersi@192.168.32.144`). **Never write passwords (Pi login or WiFi) into any file in this repo.** Ask the user for them.
- Not the same machine as `malina` (192.168.32.114), the other Pi that runs the arm / SO-101 station. Don't deploy there.
- Code lives in `~/Raspberry-Auto` with a venv in `~/Raspberry-Auto/.venv` (system site packages: apt OpenCV + picamera2; `candlesdk`/`pyCandle` built from `~/CANdle-SDK`). Always run with `.venv/bin/python`.
- Pi-only files that are not in git and must not be overwritten or deleted: `.venv/`, `floor.npy` (learned floor colour), `routes.json` (taught routes), `where.json` (station the car stands at), `calib.json` (measured 90° turn), `*.jpg`, `*.log`.
- Only one process can open the camera (and the CANdle). Stop `app.py` / `stream.py` before running `drive.py` / `course.py` / `motors.py`, and check the panel is not driving or teaching (`/status`) before restarting it.
- Shut down with `sudo poweroff` before cutting power (a hard cut once corrupted an SD card).

## Deploy

From `Raspberry-Auto/` on the laptop, copy only the tracked source files:

```sh
scp *.py config.json *.md requirements.txt hackengersi@192.168.32.144:~/Raspberry-Auto/
ssh hackengersi@192.168.32.144 'cd ~/Raspberry-Auto && .venv/bin/python drive.py selftest && .venv/bin/python course.py check'
```

Before overwriting, compare `config.json` with the Pi's copy. Values tuned on the Pi must be brought into git, not clobbered.

## Running

```sh
.venv/bin/python motors.py ping      # CANdle + drives visible? (lsusb should show 0069:1000)
.venv/bin/python course.py           # the fixed course
.venv/bin/python drive.py tag 1      # just drive to tag 1
.venv/bin/python app.py              # the station panel (keep it running: nohup / setsid)
MOCK=1 python drive.py selftest      # no hardware, runs anywhere
```

Wheels off the ground for the first test after any change to motors or signs.

## Calibration (config.json)

Physical values are **measured by the user with a ruler**, not found by trial runs. Ask them to measure; don't guess.

| Key | Value | Source |
|---|---|---|
| `wheel_radius_m` | 0.0414 | Car rolled 26 cm per wheel turn, so r = 0.26 / 2π |
| `track_m` | 0.14 | Measured centre-of-tread to centre-of-tread |
| `tag_size_cm` | 6.0 | Black square of the printed tag, without the white border |
| `tag_focal_px` | 947 | Tag held at a known distance, px read in `stream.py` |

- Distance to a tag = `tag_focal_px × tag_size_cm / side_px`. A stop at D cm is `stop_px = 947 × 6 / D` (15 cm → 379 px, 20 cm → 284 px).
- Blind moves are dead reckoning: time = distance / (speed × `max_wheel_rad_s` × `wheel_radius_m`). A spin of θ rad rolls each wheel θ × `track_m` / 2. Tyre skid in spins can make 90° turns short or long; the fix is nudging `track_m`, not changing code.
- Speeds in config are fractions (-1..1) of `max_wheel_rad_s`.

## Rules the user has set

1. **Minimal code.** Reuse what's in `drive.py` / `motors.py` / `vision.py` before writing anything new. No new dependencies, abstractions or config for values that never change. Keep calibration knobs for anything physical.
2. **New behaviours go in a new small script** (like `course.py`) that imports the building blocks. Don't fork or duplicate `approach` / `timed`.
3. Leave one runnable check for non-trivial logic (`selftest` / `check` sub-command with `assert`s), not a test framework.
4. Code, comments and docs in **English**.
5. **Git:** never add AI co-author lines, "generated with" footers or any other AI tool attribution to commits, PRs or any git text. Commit in batches, not one PR per small change. Branch + PR + merge and deploying to the Pi are allowed without asking.
6. Safety stays in place: `wheels.close()` in `finally`, obstacle stop on forward moves, the MD watchdog. Never remove them to "simplify".

## Open points (as of 2026-09-27)

- `course.py` hasn't been driven on the real car yet. Expect to tune `track_m` (turns) and maybe `wheel_radius_m` after the first run.
- On 2026-09-27 the CANdle dongle wasn't plugged in (`lsusb` showed no `0069:1000`). Plug it in and power the drives before running.
- If the tag isn't in view after the last turn, `approach()` spins to search and gives up after `search_timeout` s.
