# HexaSweep: Alien Bazaar Hackathon Plan

Team of 4, about 2 days, one robot. This folder is the whole plan. Read this file first (5 min), then your role in `04`.

## TL;DR

**Build:** a hexapod that walks a small "living room" course, spots debris with its camera, walks up to it, and hands off to the partner team's **TriArm** to pick it up and drop it in a bin. The counter goes up on a live dashboard.

**Why it wins:** the brief is a two-team hardware collaboration (our mobility base + Guardians of the Hardware's TriArm). At a "bazaar" the **integration is the product**. We show one clean, repeatable end-to-end loop, not five half-working features.

**How we stay doable:** we reuse the team's existing CubeBot stack (leg IK, body pose, Pico servo link, Hailo vision, IMU) instead of writing from scratch. We do **not** use RL, SLAM, stairs, custom-trained models or real grasp planning. The robot aligns itself to a fixed "pick zone" using vision and the arm replays taught poses.

## Files

| File | What's in it |
|---|---|
| `00-README.md` | This page: TL;DR, assumptions, questions for organizers, rules |
| `01-concept-and-scope.md` | Demo story, autonomy loop, scope tiers, explicit cut list |
| `02-architecture.md` | System design, module layout, interfaces, CubeBot reuse map, power |
| `03-timeline-and-gates.md` | Hour-by-hour plan, go/no-go gates, bench schedule, git rules |
| `04-workstreams.md` | 4 roles, tasks, acceptance criteria, time budgets |
| `05-risks-and-fallbacks.md` | Risk register, fallback ladders, kill criteria |
| `06-demo-and-submission.md` | 3-min demo script, pitch, Q&A, submission checklist |

## What we found in the material

- `docs/description.md`: **hexapod** chassis + navigation + debris finding (us), partnered with **Guardians of the Hardware** whose **TriArm** does gripping. Domestic terrain: stairs, thick carpet, thresholds, clutter.
- TriArm (tnkr.ai, `theaiwhisperers-workspace/tri-arm`): 3D-printed **6-axis SO-100 arm + 8-DOF 4-finger hand**, $300 kit, originally a wearable limb.
- `CubeBot-Code/` is a **quadruped**, not a hexapod. It is our code base to reuse, and its RL gaits do **not** transfer to six legs. Details in `02`.

## Assumptions (verify in the first hour)

| # | Assumption | If wrong |
|---|---|---|
| A1 | We get or build a **6-leg, 3-DOF/leg (18 servo)** hexapod with a **Raspberry Pi + Hailo-8L + camera + IMU**, like CubeBot | Gate G0 in `03`. Adapt gait/config only; the architecture stays the same |
| A2 | The **TriArm** is physically provided by the partner team and can be mounted on the hexapod | Manipulation fallback ladder in `05` |
| A3 | "Alien Bazaar" rewards **cross-team integration + a working live demo**, and probably a **tnkr.ai project page** (docs, BOM, code) | Ask organizers (below). Cheap to satisfy either way |
| A4 | The CubeBot repo (github.com/VGlukhov-git/CubeBot) is ours to reuse | Confirm with its author. Credit it in the README either way |
| A5 | Pre-existing code is allowed if declared | Check the rules. If not allowed, use it as reference and rewrite the ~500 lines that matter |
| A6 | Demo is a live table/floor course, about 3 minutes, and we control the space | Backup video covers us (`06`) |

## Ask organizers in the first 30 minutes

1. Judging criteria and weights. Is there a rubric?
2. Exact hardware we receive: hexapod frame, servos count/type, battery, Pi/Hailo/camera, power supplies. Who supplies TriArm and when?
3. Is there a workshop (3D printer, drill, cable, soldering) for the arm-to-hexapod mount?
4. Submission format: repo, video, tnkr project, slides? Deadline time and demo slot length.
5. Are pre-existing repos allowed? Any code-freeze rule?
6. Venue network (WiFi reliability) and demo-space size/floor type.
7. Rules on the partner interface: who on Guardians of the Hardware do we talk to, and do they build the mount or we do?

## Rules of engagement

1. **MVP loop first.** Nothing from Tier 2 starts until Tier 1 works end to end on the real robot.
2. **Integration over features.** One integration test on hardware beats a great module that has never met the robot.
3. **Gates are binary.** At each gate (`03`) the lead decides in 5 minutes: go, or cut and fall back. No debate after that.
4. **One owner per directory.** No cross-edits without telling the owner. Trunk-based git, small commits, only `main` goes on the robot.
5. **One robot, four people.** Use the bench schedule and `--mock` mode (`02`) so nobody idles.
6. **Freeze and sleep.** Feature freeze is 4 h before the demo. Everyone sleeps at least 5 h. A tired team breaks a working robot.
7. **Everything runs local.** No cloud, no internet dependency in the demo.
