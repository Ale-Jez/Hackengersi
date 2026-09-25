# 02: Architecture

Two computers, one local network, no cloud.

```
 +------------- CubeBot (Raspberry Pi + Hailo-8L) --------------+
 | Pi camera -> vision: container (COCO/HSV) + ArUco dock       |
 |                 |                                            |
 |                 v                                            |
 |           pusher brain (SEARCH/APPROACH/PUSH/BACK_OFF)       |
 |                 | walk / turn / back                         |
 |                 v                                            |
 |           gait (walk_points, rotate_*) -> UART -> Pico -> 12 servos
 +-----------------|--------------------------------------------+
                   | HTTP  POST /robot {state, clear}
                   v
 +------------- Station (laptop, next to the dock) ------------------------+
 | overhead USB webcam -> dock.py (pocket full? robot in arm zone?)        |
 |                     -> scan.py (EAN via zxing-cpp)                      |
 | station brain: clear + full -> scan (+ SO-101 roll) -> decide -> pick   |
 |    +-- USB serial (JSON) ------> RoArm-M3 Pro  (picker)                 |
 |    +-- USB serial (Feetech) ---> SO-101        (roller)                 |
 |    +-- deposit_eans.json (local accept/reject, like a shop till)        |
 |    +-- drs.py --HTTP--> mock_drs.py (localhost) | real Kaucja.pl if granted
 | dashboard: counter in PLN, last EAN, both states, video, voucher QR, E-STOP
 +-------------------------------------------------------------------------+
 Network: laptop hotspot or a travel router. Both arms are on USB.
```

Why split it this way: the arms, the camera, the EAN list and the counter never move, so they live on the laptop. The Pi only does what has to ride on the robot. The two halves talk through **one HTTP endpoint**, so they can be built and tested separately.

## Repo layout (new repo, `git init` at hour 0)

```
kaucjobot/
  robot/      (runs on the Pi)
    hw.py         servo link + LEDs + mock (from CubeBot devices/servo_uart.py)
    gait.py       walk_fwd / walk_back / turn_left / turn_right, one gait cycle per call
    vision.py     Hailo COCO bottle detector + HSV fallback + ArUco dock marker
    pusher.py     state machine, talks to the station
  station/    (runs on the laptop)
    roarm.py      RoArm JSON over serial: go(pose), grip(open|close), torque(on|off)
    so101.py      SO-101 over the Feetech bus: go(pose), roll(), torque(on|off)
    teach.py      teach.py <arm> <pose>: torque off, move by hand, Enter saves -> poses.json
    dock.py       overhead camera ROIs: pocket full? robot in arm zone?
    scan.py       EAN decode from the overhead frame
    drs.py        Kaucja.pl client: transaction, get_voucher, redeem_voucher, bag_replacement
    server.py     HTTP: /robot, /status, /estop, /voucher + dashboard page
    static/index.html
  tools/      mock_drs.py, log_replay.py
  config/     poses.json, dock_roi.json, deposit_eans.json, demo.yaml, so101_calibration.json
```

Models (`*.hef`) stay **out of git**; copy only `yolo11n_coco...hef` to the Pi.

## Interfaces (agree at hour 1, then everyone codes against mocks)

```python
# robot/vision.py -> robot/pusher.py   (latest value, ~15 Hz)
Seen = {
  "t": float,
  "item": {"cx": 0-1, "cy": 0-1, "w": 0-1, "h": 0-1, "conf": 0-1} | None,   # closest container
  "dock": {"cx": 0-1, "size": 0-1} | None,                                    # ArUco marker
}

# robot/gait.py  (blocking, one gait cycle per call, so the brain re-checks vision between steps)
walk_fwd(n=1); walk_back(n=1); turn_left(n=1); turn_right(n=1); stand(); sit()

# robot -> station  (HTTP, JSON)
POST /robot  {"state": "PUSH", "clear": false}      # "clear": true once BACK_OFF is finished
GET  /status -> {"station": "IDLE|SCANNING|PICKING|DONE", "count": 3, "pln": 1.50, "estop": false}

# station arms (same shape for both)
roarm.go("home" | "above_pocket" | "pick" | "lift" | "above_bag" | "above_reject")
roarm.grip("open" | "close")        # "close" = taught angle that holds without crushing
so101.go("home" | "above_pocket"); so101.roll()     # one quarter turn of the bottle
arm.torque(False)                   # for teaching

# station/scan.py
scan.read(frame) -> "5901234123457" | None

# station/drs.py  (mirrors ../drs-api.md; base URL + auth from demo.yaml)
drs.transaction(station_id, eans: list[str]) -> {"voucher_id": str, "amount": float}
drs.get_voucher(voucher_id) -> {"status": ..., "amount": ...}
drs.bag_replacement(station_id, seal_code, eans) -> ok
```

Request and response bodies for the real API are unknown (the quick start has no schemas). `mock_drs.py` defines our own. If we get real access, only `drs.py` changes.

**Safety rules:**
- Arms move only after the robot reports `clear: true` **and** `dock.py` sees no robot in the arm zone.
- **Only one arm moves at a time.** The SO-101 goes back to `home` before the RoArm leaves `home`, and the other way round. The station brain holds one lock.
- The pusher does not walk forward while the station is not `IDLE`.

## Mock mode (`KAUCJO_MOCK=1`)

CubeBot opens the serial port **at import time** (`devices/servo_uart.py`), so nothing runs on a laptop today. First task (30 min): make the servo link lazy and add a mock that logs angles. Both arm drivers get a mock that prints commands. `mock_drs.py` runs always. With mocks, the pusher and the station run on one laptop with a webcam and a bottle on the desk.

## Reuse map

| Source | Use | Change needed |
|---|---|---|
| CubeBot `models/arm.py`, `models/body.py`, `config/robot_setup.py` | Leg IK and geometry | None. It is already this robot |
| CubeBot `points/animation.py` `walk_points` | Forward walk | Wrap as `walk_fwd(n)`. Backward = same waypoints in reverse order |
| CubeBot `points/animation.py` `rotate_right_points` | Turn right | Turn left = mirrored (`dYaw` sign flipped). Test on blocks first |
| CubeBot `robot_controller.py` | Reference for the stand-up + walk loop, LEDs on GPIO 17/27/22 | Lift the loop, drop the hardcoded paths and globals |
| CubeBot `devices/servo_uart.py` | Pi → Pico `"ch;angle\n"` at 230400 | Lazy open + mock |
| CubeBot `ai/fast_depth.py`, `ai/ai_camera.py` | picamera2 + Hailo inference and YOLO decode | Swap in the `yolo11n_coco` HEF, keep class `bottle`. Stale imports: budget 1-2 h |
| CubeBot `control/joystick.py` | Gamepad teleop + E-STOP button | Hardcoded `/dev/input/event5`: find the device by name |
| CubeBot `RL/walk/models/crawl.onnx` | Backup gait | Only if the waypoint gait can't push |
| `RoArm-M3/python_demo/` | Serial/HTTP JSON examples | Base for `roarm.py` |
| `SO-Arm-101/Software/WEBUI_CALIBRATION.md`, LeRobot `so101_follower` | Calibration, bus access | Calibrate once; `so101.py` uses LeRobot's bus class or the Feetech SDK |
| `zxing-cpp` (pip) | EAN-13 decode, handles rotation and moderate curvature | None. `pyzbar` as a second decoder |

## Pusher design (R1 + R2)

- Step-and-look: **one gait cycle, then look again.** Slow, but no timing bugs.
- APPROACH steering: `err = item.cx - 0.5`. If `|err| > 0.15`, turn one step toward it, else walk one step forward. Stop when the box bottom is in the bumper zone (calibrate with the bottle touching the bumper).
- PUSH steering: same rule on `dock.cx`. The container may be hidden in the bumper, so don't depend on seeing it.
- BACK_OFF trigger: `dock.size` above a threshold measured with the bumper at the funnel mouth, then N back steps, then `POST clear`.
- Lost container during PUSH: back 2 steps, return to SEARCH. Per-state timeouts (APPROACH 45 s, PUSH 45 s). ESTOP checked before every step.

## Station design (R3 + R4)

- **Teaching:** `teach.py roarm pick` turns torque off, you move the arm by hand, Enter saves the joint angles. Same for the SO-101. Support the arm when torque goes off.
- **RoArm gripper:** teach `close` on a real bottle: it holds the bottle, but the bottle doesn't dent (a crushed bottle loses its deposit).
- **SO-101 roll:** `above_pocket` → lower the padded tip onto the top of the bottle → drag about 5 cm sideways along the groove's cross direction → lift → back. Each drag turns a 0.5 L bottle (about 6.5 cm across) roughly a quarter turn. Tune the tip height so it presses lightly: too hard and the bottle skids, too light and it slips.
- **Scan:** grab 5 frames after each roll, decode each, take the first valid EAN-13 (check digit verified). Lamp at an angle to avoid glare on the shiny label.
- **Pick sequence:** `home → above_pocket → pick → close → lift → above_bag|above_reject → open → home`. Speed-limited, 0.3 s pause at `pick`.
- **Missed grasp:** after `lift`, if `dock.py` still sees a bottle in the pocket, retry once, then report failure.
- **Dock check:** compare the pocket ROI with an empty-pocket reference image (mean abs diff > threshold), plus a second ROI for the arm zone. No detector needed.
- **Session:** accepted EANs are kept in memory and in `runs/`. The "Voucher" button (dashboard or keyboard) calls `drs.transaction`, then shows the voucher ID as a QR code with the amount.

## Power and wiring

| Item | Supply | Note |
|---|---|---|
| CubeBot Pi + Hailo | Own 5 V 5 A buck | Never from the servo rail |
| CubeBot leg servos | Battery or bench PSU | Bench PSU during development, robot on blocks for new gait code |
| RoArm-M3 Pro | 12 V 5 A adapter | |
| SO-101 | Its own adapter (check the kit: 5 V or 12 V STS3215 version) | Wrong voltage burns the servos: check the label before plugging in |
| Laptop, webcam, lamp | Mains | |

## Ops

- Laptop runs a hotspot (or bring a travel router). The Pi joins it. Venue WiFi doesn't matter.
- Deploy: `git pull` on the Pi. Only `main`. `demo.yaml` holds the locked thresholds, the DRS URL and the station ID.
- Every state change is logged to `runs/<timestamp>.jsonl` on both machines, including each EAN and decision.
