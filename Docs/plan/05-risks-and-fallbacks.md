# 05: Risks and Fallbacks

Rated **L**ikelihood and **I**mpact (H/M/L). When a trigger fires, switch at once; do not "try a bit longer".

## Top risks

| ID | Risk | L | I | Mitigation | Trigger → fallback |
|---|---|---|---|---|---|
| R1 | Organizers insist on the original hexapod + TriArm brief | M | H | Ask in the first 30 min | Keep dock + arms + station; the pusher becomes whatever legged base they require |
| R2 | **Bottle rolls away from the bumper** while pushing (quadruped steps are jerky) | H | H | V-scoop bumper, slow gait, short steps, smooth floor, tape strip adds friction | Under 6/10 teleop pushes by H+9 → bottles start **standing** (slide, don't roll) or start closer to the funnel |
| R3 | Bottle enters the funnel but doesn't settle in the pocket (bounces, rolls back, ends diagonal) | M | H | Pocket only 3 cm longer than the bottle, 1 cm lip, V-groove floor, test 10 hand-rolls | Diagonal → steeper funnel walls; bounce → cloth lining |
| R4 | **No Kaucja.pl API access** (it's for registered shops; the quick start has no URL, auth or schemas) | H | L | Planned for: mock with the same endpoints, honest framing (`../drs-api.md`) | Already the default. A sandbox is a Tier 3 bonus |
| R5 | **Barcode not readable** by webcam (curved, glossy label, glare, focus) | M | M | Manual focus, lamp at an angle, 5-frame vote, zxing-cpp + pyzbar, test before kickoff | Under 7/10 with the barcode facing up → USB barcode scanner mounted over the pocket (reads curved labels well) |
| R6 | SO-101 roll pushes the bottle out of the pocket or doesn't turn it | M | M | V-groove, padded tip, light pressure, short drag | Under 7/10 by T-6h → no roll: scan once; unreadable goes to reject. Or place demo bottles barcode-up in Tier 1 and say so |
| R7 | RoArm gripper crushes or dents the bottle (deposit void) | M | M | Teach `close` on a real bottle; cap on (a capped bottle is stiffer) | Dents → open the close angle 0.05 rad at a time |
| R8 | RoArm gripper drops the bottle | M | M | Grip the middle, foam or rubber on the fingers | 2 drops in 10 → grip tape, slow lift |
| R9 | **Only one of each arm, no spares.** An arm dies (servo, board, cable) | L | H | Handle gently, correct supply voltage, spare USB cables | RoArm dead → SO-101 does both jobs (picks and bags; roll dropped). SO-101 dead → no roll, scan once |
| R10 | RoArm still in ESP-NOW follower mode, ignores serial or moves unexpectedly | M | M | Test on day 1, turn follower mode off, clear zone around it | Unexpected move → power off, check `../arm.md`, re-home |
| R11 | Arms collide with each other or with CubeBot | L | H | One arm at a time (lock), `clear` + arm-zone ROI, test each full sequence against the other's home pose | Any contact → move `home` poses apart, bigger arm zone |
| R12 | SO-101 wrong supply voltage or lost calibration | L | H | Check the servo label; calibration file committed | Lost calibration → recalibrate with the web UI (20 min) |
| R13 | COCO misses a **lying** bottle | M | H | Test before kickoff; tape strip for HSV | Under 90% of frames → HSV in `demo.yaml` |
| R14 | CubeBot turn-left / walk-back don't exist yet (only `walk_points` and `rotate_right_points` in the repo) | M | M | Build and test them on blocks in the first hours | Turn left fails → turn right 3x; back fails → the robot turns away and the arm-zone ROI confirms it's gone |
| R15 | Stale CubeBot snapshot (imports, `/home/vladimir` paths, import-time serial) | H | M | 1-2 h budgeted, mocks first | Over 2 h → rewrite the 100 lines needed instead of debugging |
| R16 | Brownouts on CubeBot | M | H | Separate 5 V buck for the Pi, bench PSU during development | Any unexplained reboot → fix power before any code change |
| R17 | Network between Pi and laptop drops | M | M | Laptop hotspot / own router | Network dead → station runs from the dock camera alone; counter still works |
| R18 | Integration slips (three robots, two computers) | H | H | Contracts in hour 1, Integration #1 with a human pressing Enter | G2 missed → Tier 2 cancelled |
| R19 | Scope creep (bag seal, voice, real API before Tier 2) | H | M | Tier order in `01`, parking-lot list | After freeze → revert |
| R20 | Fatigue | H | M | Sleep rule, checklists | Two mistakes in a row on one task → swap owner for 30 min |

## Fallback ladders

Each step still gives a coherent demo.

### Pushing (CubeBot)

1. **Autonomous:** search → approach → push → back off. (target)
2. **Semi:** operator places CubeBot behind the bottle, the robot pushes to the dock on its own (marker steering only).
3. **Teleop:** operator drives CubeBot with the gamepad; the station stays autonomous.
4. **CubeBot as a show piece:** a human rolls the bottle into the funnel; the station does the rest. Be open about it.

### Scanning

A. Overhead camera + **SO-101 roll** until the EAN is read. (target)
B. Overhead camera, no roll: one try, unreadable goes to reject.
C. USB barcode scanner over the pocket instead of the webcam.
D. No scan: every docked bottle counts as accepted (Tier 1 behaviour).

### Picking

A. **RoArm** taught-pose pick after scan → bag or reject. (target)
B. Operator-triggered: the station waits for a key press instead of the dock camera.
C. **SO-101 picks** (if the RoArm fails): its own taught poses into one bin.
D. Human bags the bottle; counter by key press.

### Deposit system

A. Real Kaucja.pl sandbox (only if granted). B. **Mock DRS with the same endpoints** (default). C. Voucher generated locally, no HTTP.

### Perception (CubeBot)

A. COCO `bottle` on Hailo. B. HSV on the tape strip. C. Operator clicks the bottle on the dashboard.

## Kill criteria (stop working on it)

- A part exceeds **2x its budget** and has a fallback: switch (owner + lead, 5 min).
- Arm on CubeBot, arm vision, arm-to-arm handoff, SLAM, custom detector training: the answer is no, at any time.
- Real API integration before Tier 2 works on the mock: no.
- A new failure after the freeze: revert to the last committed `demo.yaml` and code, not a fix.

## Pre-mortem: "We lost because..."

| Cause | Prevention |
|---|---|
| The bottle rolled away in front of the judges | Bumper + dock tested 10x per change; standing-bottle fallback ready |
| The scanner stared at a blank side of the label for 30 s | Max 4 rolls, then reject with a clear message; ladder B/C ready |
| The demo didn't run | Backup video, semi/teleop ladders, 5 rehearsals with a pass/fail log |
| The arm crushed the bottle and the joke fell flat | `close` taught on a real bottle, checked in every rehearsal |
| We said "connected to the deposit system" and a judge asked how | Real/taught/mocked statement in the pitch and README |
| Someone edited code at T-30 min | Freeze rule |
