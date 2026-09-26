# Plan: programming the arms and the Raspberry Pi

**Goal:** the SO-101 camera looks into the trash bin, the vision model (running on Brev) names and sorts every item, and the RoArm-M3 picks each item and drops it into one of three containers:

| Bin key (`config.json`) | What goes in |
|---|---|
| `cans_bottles` | metal cans, drink bottles |
| `paper` | paper, cardboard, napkins |
| `plastic` | bags, wrappers, cups |

## Architecture

```
            USB (servo bus)              WiFi (HTTP /js)
 SO-101  <------------------  Pi 5  ------------------>  RoArm-M3
 + USB camera <-- USB ------   |                         (no camera)
                               | HTTPS
                               v
                        Brev: Qwen2.5-VL-7B
```

- **The RoArm gets no camera.** It computes its own joint angles, so all it needs is a target `x, y, z` in mm in its base frame. The Pi converts camera pixels to RoArm millimetres with a one-time calibration (step 05).
- **AprilTags are only used for calibration** (one tag on the RoArm gripper) and, optionally, as a drift check (one tag on the bin rim). The vision model finds the trash itself.
- **No new cables.** The Pi uses USB for the SO-101 and its camera, and WiFi for the RoArm.

## The code already exists

It lives in `Raspberry/`. The work is mostly teaching poses, calibrating and tuning, not writing new code.

| File | Role |
|---|---|
| `main.py` | `sort` loop, `calibrate`, `run` (route driving), `selftest` |
| `roarm_wifi.py` | RoArm over HTTP: `where`, `goto`, `gripper`, `wiggle` |
| `roarm_console.py` | keyboard jog for the RoArm (to teach positions) |
| `so101.py` | SO-101 over USB: `save`/`go`/`where` named poses |
| `vision.py` | camera, AprilTags, homography, Brev client, `photo`/`ask` |
| `config.json` | every IP, pose, height, bin position and tuning value |

## Steps

| # | File | Output | Depends on |
|---|---|---|---|
| 01 | [01-pi-setup.md](01-pi-setup.md) | the Pi runs every self-test, and can reach the camera, the SO-101 and Brev | nothing |
| 02 | [02-roarm.md](02-roarm.md) | RoArm on safe power, on WiFi, with `park`, `bins`, `pick_z` and `pick_t` taught | 01 |
| 03 | [03-so101.md](03-so101.md) | SO-101 `look` and `stow` poses taught, and shown to be repeatable | 01 |
| 04 | [04-vision.md](04-vision.md) | Brev returns correct labels, bins and grab points on real photos | 01, 03 |
| 05 | [05-calibration.md](05-calibration.md) | `homography` saved and pick error measured below ~15 mm | 02, 03 |
| 06 | [06-pick-and-sort.md](06-pick-and-sort.md) | `python main.py sort` empties a bin of mixed trash | 04, 05 |
| 07 | [07-demo.md](07-demo.md) | a demo run you can repeat, a reset procedure, and a list of fallbacks | 06 |

## Parallel tracks (team of 4)

- **A: Pi and vision.** Steps 01 and 04, then prompt tuning in 06.
- **B: RoArm.** Step 02, then pick tuning in 06.
- **C: SO-101 and calibration.** Step 03, then 05 together with B.
- **D: mechanics and demo.** Mount the bin and containers, print the tags, then 07.

Steps 05 and 06 need everyone's hardware in one place. Fix the layout (step 02, "Fix the layout") early: moving anything afterwards means redoing step 05.

## Hard rules (these have already cost us hardware)

- **RoArm power: 7.4–8.4 V only.** The shoulder now uses Feetech STS3215 7.4 V servos. Never connect the 12 V adapter again.
- **Never send `T:0`** (it freezes the firmware for about 10 s, and the arm can drop). `roarm_wifi.py` refuses it. To stop, command the measured pose.
- **Keep the RoArm slow** (`roarm_spd` ≤ 0.25) and only handle light, empty items. Check the shoulder temperature by hand after each test session.
- Don't use `T:210` (torque off) near people or objects: it moves the arm to a fixed pose first.
