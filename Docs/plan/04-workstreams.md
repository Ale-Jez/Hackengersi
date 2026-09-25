# 04: Workstreams and Roles

Four roles, one owner each. Names go in the table at kickoff. Role fit hint: **R1** the person most comfortable with servos/hardware bring-up; **R2** the CV/AI person; **R3** the mechanical/electronics person who can talk to the partner team; **R4** the strongest generalist/communicator (also the gate lead).

| Role | Owner | Directory | Buddy (cross-help) |
|---|---|---|---|
| **R1 Legs** (locomotion + power) | ______ | `hw/`, `legs/` | R3 (servo/power) |
| **R2 Eyes** (perception) | ______ | `perception/` | R4 (interfaces, UI overlay) |
| **R3 Hands** (TriArm + mechanical integration + partner liaison) | ______ | `arm/` + mount hardware | R1 |
| **R4 Brain** (state machine, UI, demo, docs, lead) | ______ | `brain/`, `ui/`, `tools/`, `docs/`, submission | R2 |

---

## R1: Legs

**Mission:** the robot stands, walks straight, turns, stays level, and never browns out.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 1.1 | Servo map + zero calibration (`servo_zero.py`), `calib.json` committed | All 18 legs hit jig angles within about 2 deg; stand pose symmetric | 2 h |
| 1.2 | `config_hex.py` (6 legs: mounts, mirrored frames, link lengths measured) | `gait_preview.py` shows every foot target reachable | 1 h |
| 1.3 | Tripod gait, straight, time-based `set_cmd(vx,0,0)` | Walks 1 m within 10 cm of a straight line, 3 times, on the floor | 3 h |
| 1.4 | Turning + `wz` and small lateral `vy` | Rotates 90 deg +-10 deg in place; `set_cmd` accepts all three | 2 h |
| 1.5 | Body leveling (IMU -> `body.set_rotation`) | On a board tilted about 10 deg the body stays within +-3 deg of level | 2 h |
| 1.6 | Rug + threshold tuning (lift height, stride, cycle time, foot grip) | Crosses the rug and the threshold in 4 of 5 tries with the arm stowed | 3 h (Day 2) |
| 1.7 | Power: rails, fuse, measure stall, battery swap plan | No brownout in 10 min of continuous walking | 1 h (shared with R3) |
| 1.8 | `legs.stop/stand/sit`, hardware **E-STOP** path (gamepad button -> all legs hold and servos relaxed) | Pressing E-STOP halts motion in under 0.3 s | 1 h |

**Rules:** gait on blocks first. Slower and stable beats faster. Never change `Arm` IK without telling R4 (used by the leveling and mocks). If a leg servo overheats or buzzes, stop and tell R3.

---

## R2: Eyes

**Mission:** tell the brain where the debris and obstacles are, reliably under venue lighting.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 2.1 | Camera + Hailo running on the Pi (fix the stale imports in the snapshot) | 15+ FPS overlay visible on a laptop browser | 2 h |
| 2.2 | Ball detector output as `Perception.targets` (normalised coords) | Detects the demo debris at 30-120 cm, more than 90% of frames on a still scene | 2 h |
| 2.3 | HSV colour-blob fallback (same output format), selectable by config | Works with detector off; tested under a different light | 1.5 h |
| 2.4 | Pick-zone capture tool, writes `pickzone.json` | Re-capture in under 2 min after moving the camera | 1 h |
| 2.5 | Depth strip -> `obstacle{clear,left,right,min_depth}` with measured thresholds | A box at 25 cm sets `clear=false`; open floor `clear=true` in 20 of 20 frames | 2 h |
| 2.6 | Lock exposure/focus; lighting robustness (matte debris, floor colour choice, shade) | Same behaviour at the venue as on the bench, verified on Day 2 morning | 2 h |
| 2.7 | Stretch: `hand_tracking` "come here" / "stop" gesture -> a command | Only after Tier 2 is stable | 2 h |

**Rules:** detections must degrade gracefully (return an empty list, never crash the brain). Log raw frames for 30 s on request so a bad run can be replayed offline. Camera mount is a shared decision with R3: it must see the pick zone and not be blocked by the arm.

---

## R3: Hands

**Mission:** the TriArm is bolted on, powered, controlled by one call, and picks the demo debris from the pick zone at least 70% of the time.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 3.0 | **Partner liaison (first hour):** interface agreed with Guardians of the Hardware: mount, power, protocol, who brings what, availability | Written 5-line agreement in chat/repo | 1 h |
| 3.1 | Arm + hand controlled from the Pi (their driver or LeRobot/Feetech SDK) | Every joint moves on command; torque-off pose readback works | 2 h |
| 3.2 | Mount to hexapod: bracket/plate, harness, low centre of gravity, cable strain relief | Robot stands with the arm stowed and does not lean; arm moves at rest | 3-4 h |
| 3.3 | Arm power rail (separate, common ground) | No servo dropouts during a full pick cycle | 1 h |
| 3.4 | `record_pose.py` + `poses.json` (stow, ready, pick, drop, waypoints) | Re-record any pose in under 1 min | 1.5 h |
| 3.5 | `arm.go(name)` with speed limit and interpolation; `arm.hand("open"/"close")` | 10 back-to-back pick+drop cycles at the pick zone without a fault | 2 h |
| 3.6 | Grasp verification + one retry | A deliberate miss triggers exactly one retry, then reports failure | 1.5 h |
| 3.7 | Choose and finalise the **demo debris** (shape/weight/friction) with R2 | 8 of 10 grasps succeed | 1 h |

**Rules:** the arm never moves while the hexapod is walking. Everything the brain needs is `go()` and `hand()`. Torque limits and the E-STOP must relax the arm safely (holding a pose against gravity is fine, slamming is not).

---

## R4: Brain (lead)

**Mission:** the pieces connect into a loop, the demo is rehearsed, and the submission is complete on time. Also gate lead and the person who talks to organizers.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 4.1 | Repo, layout, mocks (`HEXA_MOCK=1`), interface stubs; announce contracts | All four can run their module on a laptop by H+2 | 2 h |
| 4.2 | Dashboard: MJPEG + state + counter + tilt + FPS + **E-STOP** + mode switch (`auto/semi/teleop`) | Opens from a phone on the robot's hotspot | 2 h |
| 4.3 | State machine with per-state timeouts, retry, ESTOP, LED status | Runs the full loop against mocks and fake targets | 3 h |
| 4.4 | Real integration (perception + legs + arm) and visual servoing gains | Tier 1 acceptance (`01`) | 4 h |
| 4.5 | JSONL run logging + `log_replay.py` | A failed run can be explained from the log in under 2 min | 1 h |
| 4.6 | Demo course + `demo.yaml` (locked config) | Course rebuilds to the same layout in under 10 min | 1 h |
| 4.7 | Slides (5), README, video, tnkr project page / BOM if required | Checklist in `06` all ticked | 3 h |
| 4.8 | Gate keeping: run stand-ups, call go/no-go, enforce freeze and sleep | Gates decided in 5 min each | ongoing |

**Rules:** R4 does not become the hardware bottleneck. If the brain is on track, R4 helps whoever is behind (mostly R2 or R1). R4 writes down every fallback decision in `docs/plan/decisions.md` so the pitch stays honest.

---

## Cross-training (so one blocked person doesn't stop the team)

- R1 <-> R3: both talk servo buses, power, and mechanical mounting. Either can babysit the other's rig.
- R2 <-> R4: both touch the perception -> brain data path and the dashboard overlay.
- Everyone can run the `demo.yaml` and the E-STOP procedure by end of Day 1.

## Definition of done (any deliverable)

1. Works on the **real robot**, not only on mocks.
2. Config committed (`calib.json`, `poses.json`, ...).
3. Someone else ran it once from a clean `git pull`.
4. Failure case is safe (E-STOP works, timeouts fire, no crash).
