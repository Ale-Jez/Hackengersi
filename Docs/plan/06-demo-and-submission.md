# 06: Demo and Submission

## Demo run (3 minutes, adjust to the slot)

Two people: **Narrator** (R4) and **Operator** (R1 or R2, hands on the gamepad E-STOP). R3 stands by with the spare batteries and the reset kit.

| Time | Robot / screen | Narrator says |
|---|---|---|
| 0:00-0:20 | Robot standing on the course; dashboard on a laptop | Problem: robot vacuums stop at thresholds, carpets, clutter. Homes need something that can walk over them |
| 0:20-0:40 | Point to the arm and hexapod | Two teams, two hardware projects: our hexapod base + Guardians of the Hardware's TriArm. Today we show them working as one machine |
| 0:40-2:00 | Press START. SEARCH -> APPROACH -> obstacle turn -> rug/threshold -> ALIGN -> PICK -> DROP; the counter goes up | Narrate live what the dashboard shows: sees debris, steers, avoids the box, crosses the rug, aligns, arm picks, drops. Say "this is autonomous" only in `auto` mode |
| 2:00-2:20 | Second item (if Tier 2 is stable) | Same loop repeats, counter = 2 |
| 2:20-2:40 | Tilted board: body-leveling | Same legs, IMU feedback keeps the body level: what a wheeled base cannot do |
| 2:40-3:00 | Slide: what's next | Next: stairs, real household debris, on-board sensing... (only claims we can back) |

**Pre-run checklist (60 s):** batteries fresh, E-STOP tested, `demo.yaml` loaded, dashboard reachable, camera exposure locked, course set (same as the rehearsals), debris placed with marks on the floor.

**If it breaks live:** do not apologise or improvise. Say "switching to operator-assist", flip `semi` (or `teleop`) and continue; if the robot is down, play the backup video and walk through the dashboard log. Prepared, calm failure handling scores better than a silent crash.

## Honest framing (use in pitch and README)

- The **perception, navigation, gait and leveling are live.**
- The **arm's pick/drop poses are taught**, and the robot's job is to bring the debris to the pick zone using vision.
- Prior art declared: CubeBot open-source repo (leg IK, servo link, vision workers) and the TriArm (partner team). New during the event: hexapod gait, integration, state machine, dashboard, demo course.

## Slides (5 only)

1. Title + team + partner + one-line pitch.
2. Problem: domestic terrain vs wheeled robots. **Add a sourced number only if we have one; do not invent statistics.**
3. Solution diagram (from `02`): hexapod + TriArm, sensors, brain.
4. What we built in 2 days: photos of the robot, dashboard screenshot, the Tier that works.
5. What's next + credits (CubeBot, TriArm/Guardians of the Hardware, open-source libraries).

## Q&A prep

| Likely question | Short answer |
|---|---|
| Why a hexapod, not wheels or tracks? | Thresholds/carpet/stairs and clutter. Legs step over, and a tripod gait is statically stable |
| Is the grasp autonomous? | The robot autonomously finds and aligns; the arm replays taught poses. Grasp planning is next |
| How does it avoid obstacles? | Hailo-accelerated monocular depth, bottom strip, reactive stop/turn |
| What did you build vs reuse? | CubeBot code as base (credited). New: 6-leg config, tripod gait, leveling, brain, dashboard, integration with TriArm |
| How would it work on real household debris? | Swap the detector (train on household classes), same pipeline. Not done here because of the Hailo compile and dataset time |
| How does the interface between the two teams work? | Mount + separate power rail + `arm.go(name)` API; the arm team's driver is called through one function |
| What was the hardest problem? | Whichever real one it was (power, gait on carpet, grasp reliability). Answer with data from the logs |
| Cost? | Add BOM total on the slide (from the parts list). Do not guess |

## Submission checklist (R4 owns; complete by T-1h)

- [ ] Repo public/shared with a clear `README.md`: what it is, how to run (`HEXA_MOCK=1` and real), architecture picture, credits, licence.
- [ ] `docs/plan/` stays in the repo (shows planning discipline); add `decisions.md` with the real gates and fallbacks taken.
- [ ] 60-90 s **video**: robot doing the loop, dashboard, one leveling clip. Two takes, best one exported.
- [ ] 5 slides exported to PDF as a backup.
- [ ] tnkr.ai project page (docs, BOM, photos, repo link, hexapod <-> TriArm mount notes) **if the event uses tnkr** (A3, verify).
- [ ] Photos: chassis, arm mounted, wiring, close-ups of the interface.
- [ ] BOM and rough cost.
- [ ] Team members and roles; partner team credited.
- [ ] Rehearsal log (pass/fail table) saved in `runs/`.
- [ ] Everything copied to a USB stick and to a laptop that does not need the internet.

## Rehearsal log template

| # | Time | Mode | Result | Time to finish | Failure cause | Fix |
|---|---|---|---|---|---|---|
| 1 | | auto | | | | |

Need at least 5 consecutive runs logged before T-1h. Pick the demo mode from the data: full `auto` only if at least 4 of the last 5 passed.
