# 05: Risks and Fallbacks

Rated **L**ikelihood and **I**mpact (H/M/L). Each risk has a trigger for switching to the fallback. If the trigger fires, switch at once; do not "try a bit longer".

## Top risks

| ID | Risk | L | I | Mitigation | Trigger -> fallback |
|---|---|---|---|---|---|
| R1 | Hexapod hardware is missing, different, or servos are DOA / not enough channels (Pico firmware maps fewer than 18) | M | H | G0 check at H+1; bring spare servos and Pico; firmware channel count read in first 30 min | Not standing by H+4 -> fall back to the closest working chassis (see chassis ladder) |
| R2 | TriArm cannot be mounted or powered on the chassis, or its driver is unavailable | M | H | Liaison in hour 1; separate power rail; ask for their code/driver | No arm control by H+4 -> manipulation ladder step B/C |
| R3 | **Power brownouts** reset the Pi or twitch servos | H | H | Separate rails, common ground, bench PSU for dev, measure stall current | Any unexplained reboot -> stop, fix power before any code change |
| R4 | Gait unreliable on rug/threshold; robot drifts or stumbles | M | M | Tripod (static stability), slow cycle, rubber feet, short stride, tune on flat first | Rug fails after 2 h tuning -> drop rug or use a thinner mat; threshold -> lower it |
| R5 | Arm + hand weight destabilises gait or overloads leg servos | M | H | Stow low over body centre; ballast opposite; arm only moves when stopped | Legs overheat/sag -> reduce hand, lighter debris, arm on a rail/fixture (ladder C) |
| R6 | Grasp is unreliable | H | H | Forgiving debris, taught poses tuned by R3, verify + one retry | Below 60% after Day 2 T-6h -> debris swap (bigger/softer/tray) or `semi` mode |
| R7 | Vision fails under venue light | M | H | Lock exposure, HSV fallback, matte objects, shade the course | Detector below 90% -> switch to HSV in `demo.yaml` |
| R8 | Depth thresholds wrong (repo snapshot values were for another mount) | H | M | Re-measure with a box at 15/30/60 cm | Unreliable -> use bump-avoid by operator (`semi`) and drop autonomous obstacle avoidance |
| R9 | Stale/broken repo snapshot (imports, hardcoded paths, `/home/vladimir`, import-time serial open) | H | M | 1-2 h budgeted (R2/R4); mocks first | Over 2 h -> rewrite the 100-line piece, don't debug the old one |
| R10 | Single robot bottleneck; idle teammates | H | M | Bench schedule + mock mode + off-robot tasks | Person idle over 20 min -> lead reassigns |
| R11 | Venue WiFi/network flaky, dashboard drops | M | M | Pi hotspot/Ethernet, everything local | Dashboard dead -> robot still runs; narrate from the terminal + backup video |
| R12 | Integration slips: modules work alone but not together | H | H | Contracts on day 1 hour 1, Integration #1, dry-run mode | G2 missed -> Tier 2 cancelled, all-hands on loop |
| R13 | Scope creep / "one more feature" | H | H | Freeze at T-4h, parking lot list, tier gates | Any feature added after freeze -> revert |
| R14 | Team fatigue / errors | H | M | Sleep rule, hard stops, checklists | Two mistakes in a row on the same task -> swap owner for 30 min |
| R15 | Safety: pinch points, LiPo, hot servos | M | H | E-STOP, robot on blocks for new code, LiPo bag, no hands near legs when powered | Any smell/heat -> power off |
| R16 | Rules/attribution: reused code and partner IP | L | M | Declare CubeBot as prior art with credit; partner credited; check rules (A5) | Rules forbid -> rewrite the key 500 lines using it as reference |

## Fallback ladders

Descend one step when the trigger for that ladder fires. Each step still yields a coherent demo.

### Chassis (mobility)

1. **Hexapod, tripod gait, all six legs, carries the arm.** (target)
2. Hexapod with reduced stride/speed and shorter course, arm carried but the rug/threshold removed.
3. Whatever legged chassis we get working (CubeBot **quadruped**, same code base, static-crawl gait), arm stationary near the course; robot delivers or approaches the debris zone. Narrate the hexapod as the design and show the working platform honestly.
4. Robot on blocks: gait shown as a "walking in place" demo plus the perception and arm halves working separately, shown as a scripted chain.

### Manipulation

A. **Arm on the robot picks from the floor at the pick zone.** (target)
B. Arm on the robot picks from a **raised tray/platform** at the pick zone (shorter reach, easier).
C. Arm on a **fixed stand beside the course**; robot walks debris into (or up to) the arm's reach zone (push or align), arm picks and drops.
D. Arm only **points/places pre-positioned items**: shows detect -> locate -> arm reacts. No real grasp.

### Perception

A. Hailo ball HEF. B. HSV colour blob. C. COCO YOLO on household objects. D. Operator marks the target from the dashboard (click), and the robot approaches. (`semi`)

### Autonomy

`auto` (full loop) -> `semi` (operator presses OK before PICK and can nudge) -> `teleop` (gamepad only, perception and arm still visible on the dashboard).

## Kill criteria (stop working on it)

- A subsystem exceeds **2x its budget** and has a fallback: switch (owner + lead, 5 min).
- Stairs, RL, SLAM, custom detector, digital twin: **any** time they are mentioned before Tier 2 is stable, the answer is no.
- A new failure mode appears after the freeze: revert to the last committed `demo.yaml` and code, not a fix.

## Pre-mortem: "We lost because..."

| Cause | Prevention |
|---|---|
| The demo did not run when the judges came | Backup video, `semi`/`teleop` ladder, 5 rehearsals with a pass/fail log |
| The robot browned out at the worst time | Fresh battery for the demo, separate rails, do not reboot during the run |
| We had many features but no loop | Tier discipline and G3 |
| The pitch didn't explain the collaboration | Script in `06`: name the partner, show the interface |
| We overclaimed | Real-vs-taught statement in the pitch and README |
| Someone edited code at T-30 min | Freeze rule |
