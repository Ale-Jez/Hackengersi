# 03: Timeline and Gates

Clock times are placeholders. Day 1 is counted in hours from kickoff (`H+`). Day 2 is counted back from the demo/submission time (`T-`). If the event runs 48 h non-stop, keep the same order and add sleep blocks (see the sleep rule below).

## Before kickoff (if there is any time)

Most of the software can be built and tested without a robot:

- [ ] Create repo, layout from `02`, mock devices, interface stubs.
- [ ] Port `Arm`/`Body` to a 6-leg config; write `gait.py` + `gait_preview.py` and check IK reachability offline.
- [ ] Dashboard skeleton (MJPEG from a webcam, state text, E-STOP).
- [ ] State machine running against mocks with fake targets.
- [ ] Read SO-100 / Feetech / LeRobot docs; message Guardians of the Hardware (who, what interface, when).
- [ ] Prepare debris objects, a rug/towel, a board for the threshold, boxes, a bin, tape, cable ties, spare USB cables.
- [ ] Copy needed models to a USB stick. Install list of Pi packages (`picamera2`, `hailo` runtime, `pyserial`, `smbus2`, `gpiod`, `evdev`, `opencv`, `flask`).

Check the rules first: pre-written code may be restricted (A5).

## Day 1: get every subsystem alive, then meet

| Time | R1 Legs | R2 Eyes | R3 Hands | R4 Brain |
|---|---|---|---|---|
| H0-0:45 | **Kickoff together:** inventory, ask organizers (`00`), meet partner team, agree mount/power/protocol, assign roles | | | |
| H0:45-3 | Power on, servo channel map, `servo_zero.py`, save `calib.json`, stand pose | Camera + Hailo alive, ball HEF running, overlay on laptop | TriArm alive from Pi, joint sweep, torque-off pose reading | Repo, mocks, contracts, dashboard skeleton |
| H3-4 | **Gate G1** (below) | | | |
| H4-7 | Tripod gait straight (robot on blocks, then on floor) | Target output `cx,cy,w,h`; HSV fallback; pick-zone capture | Mount + wiring + power on chassis; stow pose | State machine on mocks; logging |
| H7-9 | Turning gait; leveling loop | Depth strip obstacle signal + thresholds | Record poses; pick/drop sequence on a static table | Wire real perception -> brain (dry-run: prints commands) |
| H9-11 | **Integration #1** (all on one robot) | | | Fix list |
| H11+ | Stop. Eat. **Sleep at least 5 h.** | | | |

**Integration #1 target (end of Day 1):** robot walks and turns from the gamepad with the arm stowed; the dashboard shows detections; `arm.go("pick")` picks a ball placed at the pick zone; brain in dry-run outputs sensible commands from live detections. If this holds, Day 2 is closing the loop. If not, Day 2 morning is triage, not new features.

## Day 2 (T = demo/submission time)

| Time | Work |
|---|---|
| T-9h to T-7h | Fix list from Integration #1. **Close the loop:** APPROACH -> ALIGN -> PICK -> DROP live |
| **T-6h: Gate G3** | MVP passes acceptance (`01`, Tier 1)? Yes -> Tier 2. No -> cut Tier 2 entirely, spend all time on reliability |
| T-6h to T-4h | Tier 2, only what fits: obstacle avoidance, rug + threshold, multi-item, leveling wow. R4 builds the demo course, slides, README |
| **T-4h: FEATURE FREEZE (G4)** | No new features. Lock `demo.yaml`. Only bug fixes with a re-test |
| T-4h to T-2.5h | 5 full-course rehearsals with a pass/fail log. Tune only thresholds |
| T-2.5h | **Record backup video** (2 good takes) + photos. Charge batteries, swap to fresh |
| T-2h to T-1h | Pitch rehearsal x3 with the robot. Submission materials done (`06`) |
| T-1h | Lock. Robot on the table. Nobody edits code. Submit anything due early |
| T-0 | Demo |

## Gates

| Gate | When | Question | If NO |
|---|---|---|---|
| **G0** | H+1 | Do we have the hexapod, 18 working servos, Pi/Hailo/camera/IMU, and TriArm access? | Adapt (`05` risk R1/R2). Decide chassis and arm plan within 30 min |
| **G1** | H+4 | Legs stand and hold pose? Camera+detector run on the Pi? Arm moves from the Pi? Brain runs on mocks? | Apply fallback per stream (`05`): HSV blob, arm static on table, teleop gait |
| **G2** | End Day 1 | Integration #1 target met? | Day 2 morning is triage; Tier 2 shrinks |
| **G3** | T-6h | MVP >= 3/5 trials? | Cut Tier 2, harden Tier 1 |
| **G4** | T-4h | Freeze | Non-negotiable |

Gate decisions are made by the lead (R4) in at most 5 minutes with input from the owner. Nobody reopens a gate.

## Bench schedule (one robot, four people)

- Robot has one **bench owner** at a time in 60-90 min slots, written on a whiteboard.
- Off-robot work: R1 gait preview on laptop; R2 recorded video + laptop webcam; R3 arm on the table alone; R4 mocks. Everyone keeps something useful to do without the robot.
- Robot handover: the previous owner leaves it in **sit pose, servos off, battery unplugged**.
- Robot on blocks whenever gait code is new. **Hands away from legs** while powered.

## Sync and comms

- 10-minute stand-up at H+0, H+4, H+9 (Day 1) and at T-9h, T-6h, T-4h (Day 2): blocked? done? next? Nothing longer.
- One shared channel for blockers. A blocker older than 30 min goes to the lead.
- Timebox rule: **any task over 2x its estimate** gets escalated, and the owner either gets help or the fallback.

## Git rules

- One repo, `main` always runnable. Short branches, merge daily at least at each sync point, small commits.
- Ownership by directory (`04`). Interface changes (`02`) need an announcement.
- Config in files: `calib.json`, `pickzone.json`, `poses.json`, `demo.yaml`. **Commit them**, since the robot's tuning is as valuable as the code.
- Before every robot session: `git pull` on the Pi. After: commit whatever was tuned.

## Sleep rule

At least 5 continuous hours for everyone between Day 1 and Day 2 (staggered if one person must babysit charging). Overnight debugging usually loses more than it gains.
