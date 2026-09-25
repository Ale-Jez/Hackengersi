# 04: Workstreams and Roles

Four roles, one owner each. Names go in the table at kickoff. Role fit hint: **R1** knows CubeBot's servos and gait best; **R2** is the CV person (robot vision and the barcode scanner); **R3** is hands-on with mechanics and both arms; **R4** is the strongest generalist/communicator and gate lead.

| Role | Owner | Directory | Buddy |
|---|---|---|---|
| **R1 Legs** (CubeBot gait, bumper, pushing) | ______ | `robot/hw.py`, `robot/gait.py` | R3 |
| **R2 Eyes** (container + dock detection on the Pi, EAN scanning at the dock) | ______ | `robot/vision.py`, `station/scan.py`, `station/dock.py` | R4 |
| **R3 Arms + Dock** (RoArm, SO-101, poses, dock mechanics, camera stand) | ______ | `station/roarm.py`, `so101.py`, `teach.py`, cardboard | R1 |
| **R4 Brain + Station** (both state machines, DRS client + mock, dashboard, demo, lead) | ______ | `robot/pusher.py`, `station/server.py`, `drs.py`, `tools/`, `docs/` | R2 |

---

## R1: Legs

**Mission:** CubeBot walks forward and back, turns both ways, and pushes a container in a straight line without losing it.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 1.1 | Lazy servo link + mock (`hw.py`) | Pusher runs on a laptop with `KAUCJO_MOCK=1` | 0.5 h |
| 1.2 | `gait.py`: `walk_fwd/back`, `turn_left/right`, `stand/sit`, one cycle per call | Walks 1 m within 15 cm of straight; turns 90 deg +-15 deg in a known number of steps | 3 h |
| 1.3 | Bumper v1 → v2 (V-scoop, light, mounted low, doesn't block the camera) | Teleop pushes a lying bottle 1 m without losing it 8 of 10 times | 2 h |
| 1.4 | Push into the funnel by teleop, tune with R3 | 7 of 10 bottles end in the pocket from 1 m | 2 h |
| 1.5 | Gamepad teleop + **E-STOP** button (all legs hold) | E-STOP stops within one step | 1 h |
| 1.6 | Tier 2: push the reject item and a can; rug patch; `crawl.onnx` if the waypoint gait can't push | Each pushed into the pocket 3 of 5 | 2 h (Day 2) |

**Rules:** new gait code on blocks first. Slow and straight beats fast. Measure and write down: cm per step, degrees per turn step.

---

## R2: Eyes

**Mission:** tell the pusher where the container and the dock are, and tell the station which EAN is in the pocket.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 2.1 | Camera + Hailo COCO HEF running on the Pi (fix stale imports from the CubeBot snapshot) | 10+ FPS overlay in a laptop browser | 2 h |
| 2.2 | `bottle` output as `Seen.item`, closest one | Lying 0.5 L bottle detected at 30-150 cm in >90% of frames on a still scene | 1.5 h |
| 2.3 | HSV fallback on the tape strip, same output | Works with Hailo off, under two different lights | 1 h |
| 2.4 | ArUco dock marker → `Seen.dock`; bumper-zone and "at funnel mouth" thresholds in `demo.yaml` | Marker found from 2 m and at 20 cm | 1.5 h |
| 2.5 | `scan.py`: `zxing-cpp` (+ `pyzbar`) on the overhead camera, 5-frame vote, check digit verified | Barcode facing up: decoded 10 of 10. No false EANs in 50 frames of an empty pocket | 2 h |
| 2.6 | Overhead camera setup with R3: height, manual focus, lamp angle against glare | Scan works at the venue as on the bench | 1 h |
| 2.7 | `dock.py`: pocket-full ROI + arm-zone ROI | Empty/full correct 20 of 20; robot in the arm zone detected | 1 h |

**Rules:** vision never crashes a brain: no detection is `None`, no EAN is `None`. Record 30 s of raw video on request for offline tuning.

---

## R3: Arms + Dock

**Mission:** a container that enters the funnel always ends in the same spot. The SO-101 can turn it, and the RoArm bags it without crushing it.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 3.1 | Dock v1: funnel + pocket with V-groove floor + back-wall marker + camera stand, taped down | A bottle rolled in by hand from any angle in the mouth ends in the pocket 9 of 10 | 2 h |
| 3.2 | `roarm.py` over serial (JSON), mock mode; confirm it works without a leader | Every joint and the gripper move from Python | 1 h |
| 3.3 | `so101.py` on the Feetech bus, calibration committed, mock mode | Every joint moves from Python; torque off/on works | 1.5 h |
| 3.4 | `teach.py` + `poses.json` for both arms | Re-teach any pose in under 1 min | 1 h |
| 3.5 | RoArm gripper `close` angle that holds but doesn't dent | 10 of 10 lifts, bottle undamaged | 1 h |
| 3.6 | RoArm pick → bag / reject sequences, speed-limited | 10 back-to-back cycles, at least 9 in the right bin | 1.5 h |
| 3.7 | SO-101 `roll()`: padded tip, taught drag move | 4 rolls turn the bottle roughly a full circle, bottle stays in the pocket 10 of 10 | 2 h |
| 3.8 | Arm layout: both reach the pocket, never collide from their `home` poses | Each arm moves through its full sequence while the other sits at `home` without contact | 0.5 h |
| 3.9 | Tier 2: can poses (standing can: RoArm side grip; roll not needed if the barcode faces the camera, else skip) | 4 of 5 cans bagged, undamaged | 1.5 h |

**Rules:** only one arm moves at a time. Support an arm by hand before turning its torque off. Check the SO-101 supply voltage against its servo label before the first power-up. The dock is taped down and its outline marked, so it can be rebuilt in the same place.

---

## R4: Brain + Station (lead)

**Mission:** the three robots become one loop, the deposit flow looks like the real thing, the demo is rehearsed, and the submission is on time.

| # | Deliverable | Acceptance | Budget |
|---|---|---|---|
| 4.1 | Repo, layout, mocks, interface stubs; announce contracts | Everyone runs their part on a laptop by H+2 | 1.5 h |
| 4.2 | `tools/mock_drs.py` + `station/drs.py` (transaction, get_voucher, bag_replacement), station ID in `demo.yaml` | `drs.transaction(...)` returns a voucher from the mock; switching the URL is config only | 1.5 h |
| 4.3 | `station/server.py` + dashboard: counter in PLN, last EAN + product name + ACCEPT/REJECT, both states, overhead video, E-STOP, "ding"/"buzz" sounds, voucher QR | Opens on a phone and on the big screen | 2.5 h |
| 4.4 | `pusher.py` state machine (step-and-look, timeouts, ESTOP) | Full loop on mocks with a webcam and a bottle on the desk | 3 h |
| 4.5 | Station state machine: clear + full → scan/roll loop → decide → pick → bag/reject → count, one-arm-at-a-time lock | Loop with both real arms and a hand-pushed bottle | 2 h |
| 4.6 | Live integration, steering gains, thresholds | Tier 1 acceptance (`01`) | 4 h |
| 4.7 | JSONL logs on both machines + `log_replay.py` | A failed run explained from the logs in under 2 min | 1 h |
| 4.8 | Arena build + `demo.yaml` | Arena rebuilt the same way in under 10 min (tape marks) | 1 h |
| 4.9 | Slides (5), README, video | `06` checklist all ticked | 3 h |
| 4.10 | Gate keeping, stand-ups, freeze, sleep | Gates decided in 5 min each | ongoing |

**Rules:** R4 doesn't become the hardware bottleneck. When the brain is on track, R4 helps whoever is behind. Every fallback decision goes into `docs/plan/decisions.md`.

---

## Cross-training

- R1 ↔ R3: the push-into-funnel tuning is joint work (bumper vs funnel shape).
- R2 ↔ R3: scan quality depends on the pocket, camera stand and lamp. Tune them together.
- R2 ↔ R4: both touch the `Seen` → pusher path, the scan → decide path, and the dashboard overlay.
- Everyone can run `demo.yaml`, power up both arms safely, and use both E-STOPs by the end of Day 1.

## Definition of done (any deliverable)

1. Works on the **real hardware**, not only on mocks.
2. Tuning committed (`poses.json`, `dock_roi.json`, `deposit_eans.json`, `demo.yaml`).
3. Someone else ran it once from a clean `git pull`.
4. The failure case is safe (E-STOP works, timeouts fire, no crash).
