# 03: Timeline and Gates

Clock times are placeholders. Day 1 is counted in hours from kickoff (`H+`). Day 2 is counted back from the demo/submission time (`T-`).

## Before kickoff (if there is any time)

- [ ] Create the repo with the layout from `02`, mocks, interface stubs.
- [ ] `station/roarm.py` + `teach.py` working against the real RoArm on a desk. Confirm it takes serial commands with no leader.
- [ ] SO-101: calibrate (`../arm.md`), commit the calibration file, move it from Python.
- [ ] Scan test: webcam 30 cm above a lying 0.5 L bottle, barcode up → `zxing-cpp` decodes it? Try 3 different bottles and a can. **This decides A6 early.**
- [ ] Build `deposit_eans.json`: scan every demo container (EAN, name, type, 0.50) + the reject item (not in the list).
- [ ] `tools/mock_drs.py` with the endpoints from `../drs-api.md`.
- [ ] CubeBot: confirm walk, turn right, and try walk-back and turn-left on blocks.
- [ ] Run the COCO HEF on the Pi and check that a **lying** 0.5 L bottle is detected as `bottle` from 30-150 cm.
- [ ] Collect: 10 empty 0.5 L bottles (same product is fine), 3 cans, 1 reject item, coloured tape, foamboard or cardboard, 2 strips for the V-groove, hot glue, gaffer tape, printed ArUco markers, bag + reject bin, USB webcam + stand + lamp, foam/rubber tape for the SO-101 tip, gamepad, spare servo.

Check the rules first: pre-written code may be restricted (A5).

## Day 1: every part alive, then meet

| Time | R1 Legs | R2 Eyes (Pi + scanner) | R3 Arms + Dock | R4 Brain + Station |
|---|---|---|---|---|
| H0-0:45 | **Kickoff together:** inventory, ask organizers (`00`), assign roles, agree the interfaces in `02` | | | |
| H0:45-3 | CubeBot stand + walk on the floor, walk_back, turn_left | COCO bottle detector on the Pi, overlay in a browser | Dock v1 (cardboard, V-groove pocket), camera stand + lamp. RoArm + SO-101 from the laptop, torque off/on, readback | Repo, mocks, `/robot` + `/status` server, `mock_drs.py`, dashboard skeleton |
| H3-4 | **Gate G1** (below) | | | |
| H4-7 | Bumper v1, push a bottle by teleop 1 m straight, tune step size | ArUco dock marker, bumper-zone calibration, HSV fallback | Teach RoArm poses, pick + bag a hand-placed bottle 10 times, tune gripper angle | Pusher state machine on mocks with a webcam |
| H7-9 | Teleop push into the funnel from 1 m, 10 tries, fix the funnel with R3 | `scan.py` on the overhead camera, glare and focus tuning; `Seen` live to the pusher | SO-101 roll: teach, tune pressure, 10 quarter turns; `dock.py` ROIs | Station brain (lock, scan loop, decide, pick) + counter + sound |
| H9-11 | **Integration #1**: full loop with a human pressing Enter between states | | | Fix list |
| H11+ | Stop. Eat. **Sleep at least 5 h.** | | | |

**Integration #1 target (end of Day 1):** teleop pushes a bottle into the dock at least 7 of 10 times. The RoArm picks a docked bottle and bags it at least 8 of 10 times. SO-101 roll + scan finds the EAN of a docked bottle at least 7 of 10 times. The pusher, running live, prints sensible step decisions from the camera. If this holds, Day 2 is closing the loop. If not, Day 2 morning is triage.

## Day 2 (T = demo/submission time)

| Time | Work |
|---|---|
| T-9h to T-7h | Fix list. **Close the loop:** pusher in `auto`, station reacts to `clear` |
| **T-6h: Gate G3** | MVP acceptance (`01`, Tier 1)? Yes: Tier 2. No: cut Tier 2, all hands on reliability |
| T-6h to T-4h | Tier 2 in its order (`01`): scan + decide → roll → voucher → 3 containers incl. reject → can → rug. R4 builds the final arena, slides, README |
| **T-4h: FEATURE FREEZE (G4)** | No new features. Lock `demo.yaml` and `poses.json` |
| T-4h to T-2.5h | 5 full rehearsals with a pass/fail log. Tune only thresholds and cardboard |
| T-2.5h | **Record the backup video** (2 good takes) + photos. Fresh batteries |
| T-2h to T-1h | Pitch rehearsal x3 with the robots. Submission materials (`06`) |
| T-1h | Lock. Nobody edits code |
| T-0 | Demo |

## Gates

| Gate | When | Question | If NO |
|---|---|---|---|
| **G0** | H+1 | Organizers OK with the concept? API sandbox yes/no? CubeBot powers up and stands? Both arms answer from the laptop? | Concept: adapt the pitch (`00` A1). No sandbox: mock (already planned). Hardware: `05` ladders |
| **G1** | H+4 | CubeBot walks forward and back and turns both ways? Bottle detected on the Pi? Both arms replay a pose? Dock v1 built? A barcode decoded from the overhead camera? | Per-part fallback (`05`): teleop pushing, HSV, one arm does both jobs, USB barcode scanner |
| **G2** | End Day 1 | Integration #1 target met? | Day 2 morning is triage; Tier 2 shrinks |
| **G3** | T-6h | MVP at least 3 of 5? | Cut Tier 2, harden Tier 1 |
| **G4** | T-4h | Freeze | Non-negotiable |

Gate decisions: lead (R4), at most 5 minutes, input from the owner. Nobody reopens a gate.

## Sharing hardware

- CubeBot has one **bench owner** at a time (whiteboard, 60-90 min slots). The dock station (arms + camera) belongs to R3; R2 and R4 book it for scan and station-brain tests.
- Off-robot work: R2 with recorded video from the Pi camera; R4 with mocks; R1 tunes the gait on blocks.
- Handover: CubeBot in sit pose, servos off, battery unplugged.

## Sync and comms

- 10-minute stand-ups at H+0, H+4, H+9 and T-9h, T-6h, T-4h: done? blocked? next?
- A blocker older than 30 min goes to the lead. A task at 2x its estimate gets help or its fallback.

## Git rules

- One repo, `main` always runnable. Small commits, merge at each sync.
- Ownership by directory (`04`). Interface changes (`02`) must be announced.
- **Commit tuning:** `poses.json`, `dock_roi.json`, `deposit_eans.json`, `so101_calibration.json`, `demo.yaml`. The tuning is worth as much as the code.
- `git pull` on the Pi before every session; commit whatever was tuned after.

## Sleep rule

At least 5 continuous hours for everyone between Day 1 and Day 2.
