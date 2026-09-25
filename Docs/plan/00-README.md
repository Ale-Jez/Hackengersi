# Kaucjobot: Alien Bazaar Hackathon Plan

Team of 4, about 2 days, one walking robot plus two arms. This folder is the whole plan. Read this file first (5 min), then your role in `04`.

## TL;DR

**Build:** a home **return machine (RVM) on legs**. CubeBot (our quadruped) finds an empty bottle on the floor and pushes it into a **docking bay**. The **SO-101** rolls the bottle in place until the overhead camera reads its **EAN**. The station checks the EAN against the deposit list, like a shop till does. The **RoArm-M3 Pro** drops deposit containers into the bag and everything else into the reject bin. The big screen goes *ding* and adds **0.50 PLN**. At the end a **voucher** comes out through the Kaucja.pl OpenAPI flow (`POST /transaction`).

**Why it wins:** a real, current problem (Poland's deposit system, running since October 2025), a physical loop anyone understands in five seconds, three robots handing off to each other, a real barcode check, and money at the end.

**How we stay doable:** the dock is a **mechanical funnel**. Every bottle ends up in the same place, so both arms replay **taught poses**, with no arm vision and no grasp planning. CubeBot already walks and turns and already runs Hailo detection. Barcode reading is an off-the-shelf library on a fixed webcam. The deposit API is a mock with the real endpoint shapes (`../drs-api.md`). We write glue, not research.

## Files

| File | What's in it |
|---|---|
| `00-README.md` | This page: TL;DR, hardware, assumptions, questions for organizers, rules |
| `01-concept-and-scope.md` | Demo story, dock design, autonomy loop, scope tiers, cut list |
| `02-architecture.md` | System design, interfaces, reuse map, power |
| `03-timeline-and-gates.md` | Hour-by-hour plan, go/no-go gates, git rules |
| `04-workstreams.md` | 4 roles, tasks, acceptance criteria, time budgets |
| `05-risks-and-fallbacks.md` | Risk register, fallback ladders, kill criteria |
| `06-demo-and-submission.md` | 3-min demo script, pitch, Q&A, submission checklist |
| `../arm.md` | RoArm-M3 Pro and SO-101: jobs, commands, teaching |
| `../drs-api.md` | Kaucja.pl OpenAPI: what it offers, what it doesn't, how we use it |

## Hardware we have

- **CubeBot** (`CubeBot-Code/`): quadruped, 12 PWM servos through a Pico, Raspberry Pi + Hailo-8L, Pi camera, WT901 IMU, 3 status LEDs. It has a waypoint walk gait (`walk_points`), a turn-right gait (`rotate_right_points`), an RL crawl policy (`RL/walk/models/crawl.onnx`) and a bottom-strip depth obstacle check.
- **1x RoArm-M3 Pro** (picker) and **1x SO-101 follower** (scanner/roller). No leader arms (`../arm.md`).
- Models on disk: `yolo11n_coco` / `yolo26n_coco` HEF (COCO includes `bottle` and `cup`, **not** `can`), `best_ball_v8n`, `scdepthv3` depth.
- To buy or bring: 1080p USB webcam with manual focus, desk lamp, cardboard/foamboard, 2 wooden or aluminium strips for the V-groove.

## Assumptions (verify in the first hour)

| # | Assumption | If wrong |
|---|---|---|
| A1 | Organizers accept this concept even though the original brief was hexapod + TriArm (older version in git) | Ask first. If a hexapod is mandatory, the same dock + arms + station work with a hexapod pusher; only the gait changes |
| A2 | CubeBot walks forward and turns on the demo floor with a 100-200 g bumper attached | Gate G1. Fallback: teleop pushing (`05`) |
| A3 | The RoArm takes serial commands on its own (no leader), and the SO-101 is calibrated and moves under LeRobot/Feetech | Gate G1. Fallback: the other arm does both jobs (`05`) |
| A4 | Demo containers are **empty 0.5 L PET bottles** (Tier 1); cans are Tier 2 | Bigger bottles need a bigger dock and new poses, about 1 h |
| A5 | There is **no real Kaucja.pl API access** during the event; we use a mock with the same endpoints | If a sandbox exists, switch the URL + auth in `demo.yaml` (1-2 h) |
| A6 | A webcam can read the EAN-13 on a curved 0.5 L label at about 30 cm under a lamp | Test before kickoff. Fallback: USB barcode scanner over the pocket |
| A7 | Pre-existing code (CubeBot, RoArm firmware, LeRobot) is allowed if declared | Check the rules |
| A8 | We control a floor area of about 2 m x 1.5 m for the demo | Shrink the arena, dock stays the same |

## Ask organizers in the first 30 minutes

1. Is the concept change acceptable (A1)? What are the judging criteria and weights?
2. Any contact at Kaucja.pl or another deposit operator for an API sandbox or an EAN list (A5)?
3. Demo slot length, floor space, floor type (smooth is easier for pushing than carpet).
4. Submission format (repo, video, slides, tnkr page?) and deadline.
5. Are pre-existing repos allowed? Is there a workshop (cardboard, hot glue, 3D printer)?
6. Venue WiFi. We plan to run fully local anyway.

## Rules of engagement

1. **MVP loop first.** One bottle: push → dock → pick → bag → +0.50 PLN. Scanning, sorting and vouchers are layered on top, never in the way.
2. **Mechanics before code.** If a cardboard wall solves it, don't write software for it.
3. **Gates are binary.** At each gate (`03`) the lead decides in 5 minutes: go, or fall back. No debate after that.
4. **One owner per directory.** Trunk-based git, small commits, only `main` runs on the hardware.
5. **Freeze and sleep.** Feature freeze 4 h before the demo. Everyone sleeps at least 5 h.
6. **Everything runs local.** No cloud or internet dependency in the demo (the mock deposit API runs on the laptop).
