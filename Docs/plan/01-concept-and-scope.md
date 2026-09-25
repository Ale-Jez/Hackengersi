# 01: Concept and Scope

## The pitch in three sentences

Wheeled robot vacuums stop at thresholds, thick carpet and clutter. HexaSweep is a hexapod that walks over them, finds debris by camera, and, paired with the TriArm from Guardians of the Hardware, picks it up and removes it. Two teams' hardware, one working loop.

## Demo course (built by us, about 2 m x 1.5 m)

```
 START                                            BIN
   |                                               |
   v   [clutter box]   [threshold 1-2 cm]   [rug patch]      [debris x3]
```

- Flat floor or table top with a **rug/towel patch** (carpet) and a **low threshold** (a board or book, height at most 50% of the robot's measured max foot lift).
- 1-2 **clutter obstacles** (boxes) placed so a straight line hits one.
- **2-3 debris items**: forgiving shapes, high contrast on the floor (foam balls, crumpled coloured paper, small blocks, about 3-5 cm). Ball-shaped is best because we already have a trained ball detector.
- A **drop bin** (on the robot's back tray or a fixed box next to the arm's reach).

Why not stairs: too risky for 2 days. We show a tilted board with body-leveling instead (wow beat, cheap).

## Autonomy loop

```
STAND -> SEARCH -> APPROACH -> ALIGN -> PICK -> DROP -> (count+1) -> SEARCH ... -> DONE
           ^           |          |        |
           +-----------+ lost / timeout    +-- retry once, then skip item
   ESTOP reachable from every state (gamepad button + dashboard + keyboard)
```

| State | What happens | Signals |
|---|---|---|
| STAND | Reach stance pose, arm in **stow** pose (low centre of gravity) | IMU level |
| SEARCH | Rotate in place in small steps; sway body to scan; stop when a target is seen for N frames | Detector |
| APPROACH | Walk toward target; steer to centre it; depth strip stops or turns for obstacles | Detector + depth |
| ALIGN | Small steps until the target sits inside the **pick zone** box in the image for 5 frames | Detector |
| PICK | Stop walking. Arm: stow -> ready -> pick pose -> hand close -> lift | Arm API |
| DROP | Arm to bin pose -> hand open -> stow. Counter +1 | Arm API |

### What is real vs taught (say this openly in the pitch)

- **Real, live:** detection, target tracking, steering, obstacle stop, gait, body leveling, state machine.
- **Taught:** the arm's pick/drop poses are recorded once by hand-guiding the arm and replayed. The robot's job is to **put the debris in the same place relative to the arm every time** (the pick zone), which removes hand-eye calibration and grasp planning.

## Scope tiers

| Tier | Goal | Acceptance test | Ships by |
|---|---|---|---|
| **0: Alive** | Hexapod stands, walks straight, turns; arm moves to named poses; camera + detection overlay in a browser | Keyboard/gamepad drives the robot 1 m in a straight line, turns 90 deg; `arm.go("pick")` works; dashboard shows boxes | End of Day 1 |
| **1: MVP (must ship)** | Detect -> approach -> align -> pick -> drop, one item, flat floor | 5 trials, ball 60-100 cm ahead +-20 cm sideways: at least 3/5 land in bin, each under 90 s | Day 2, T-6h |
| **2: Target** | 2-3 items in a row, obstacle avoidance, rug + threshold crossing, body-leveling demo, status LEDs | Full course run at least 3/5 times or at least 2 items removed | Day 2, T-4h (freeze) |
| **3: Stretch (only if Tier 2 is boringly reliable)** | Hand-gesture "come here" (existing `hand_tracking.py`), Unity/3D digital twin, auto-return-to-start, small step, voice line | Any one of these works twice in a row | Never blocks the demo |

## Explicit cut list (do NOT do)

| Cut | Why |
|---|---|
| RL walking / stabilisation | Existing ONNX policies are quadruped-specific (18-dim obs, 12-dim action). A tripod gait needs no training |
| SLAM / mapping / path planning | Reactive steering + depth stop is enough for a 2 m course |
| Stairs | Chassis and time risk. Level board demo instead |
| Custom-trained detector for real household debris | Hailo compile chain plus data collection is a day on its own. Use the existing ball HEF or COCO YOLO, plus an HSV colour-blob fallback |
| Hand-eye calibration, grasp planning, IK for the arm | Taught poses plus fixed pick zone |
| Full 8-DOF finger control | Use 2 hand presets: open and close (pinch or power grasp, whichever holds) |
| New mechanical design from scratch | Use the supplied frame; only a mounting plate or bracket for TriArm |
| Fancy UI | One page: video, state, counter, E-STOP, mode switch |

## Why this should score

(Criteria unknown, so we hedge across the usual ones.)

| Typical criterion | How we address it |
|---|---|
| Working demo | Small closed loop, rehearsed 5+ times, backup video |
| Technical depth | Multi-leg IK + tripod gait, IMU leveling, Hailo-accelerated vision, autonomy state machine |
| Integration / collaboration | Hexapod + TriArm is literally the brief: mount, power, protocol, pick zone |
| Real-world usefulness | Domestic terrain the wheeled vacuum can't cover, clear next steps |
| Communication | 3-minute script, live dashboard, honest real-vs-taught framing |
