# 02: Architecture

Single Raspberry Pi runs everything. Laptop/phone only opens the dashboard. No cloud.

```
                       +------------------ Raspberry Pi (Hailo-8L) -------------------+
 Camera --picamera2--> | perception  --targets, obstacle-->  brain (state machine)     |
                       |  (Hailo: ball/YOLO, depth; HSV fallback)   |     |            |
 IMU (WT901, I2C) ---> | legs: gait + IK + body leveling  <--vx,vy,wz,body pose--+    |
                       |   |                                                  |       |
                       |   +--UART 230400--> Pico (PWM) --> 18 leg servos     |       |
                       | arm: TriArm driver <--go("pick"|"drop"|"stow")-------+       |
                       |   +--USB/serial--> SO-100 bus servos + hand              |   |
                       | ui: HTTP page (MJPEG + status + E-STOP)  <----- status ------+
                       +---------------------------------------------------------------+
 Gamepad (evdev) -> teleop + E-STOP          LEDs (GPIO 17/27/22) -> state colours
```

## Repo layout (new repo, `git init` at hour 0)

```
hexasweep/
  hw/         servo_uart.py (from CubeBot), imu.py (wt901), leds.py, mock.py
  legs/       kinematics (Arm, Body reused), gait.py (tripod), level.py, config_hex.py, calib.json
  perception/ camera.py, detect.py (Hailo ball/YOLO), blob.py (HSV fallback), depth.py, pickzone.json
  arm/        triarm.py (driver wrapper), poses.json, record_pose.py
  brain/      states.py, main.py
  ui/         server.py, static/index.html
  tools/      servo_zero.py, gait_preview.py, log_replay.py
  docs/       (this plan lives at repo root docs/plan)
```

One owner per top-level dir (see `04`). Models (`*.hef`, about 300 MB) stay **out of git** (shared folder or `git lfs`). Copy only `best_ball_v8n.hef`, `scdepthv3...hef`, `yolov11n.hef` to the Pi.

## Interfaces (agree at hour 1, then everyone codes to mocks)

```python
# perception -> brain  (latest value, thread-safe, ~15-30 Hz)
Perception = {
  "t": float,
  "targets": [{"cls": "ball", "conf": 0.0-1.0,
               "cx": 0-1, "cy": 0-1, "w": 0-1, "h": 0-1}],   # normalised image coords
  "obstacle": {"clear": bool, "min_depth": float, "left": float, "right": float},
}

# brain -> legs
legs.set_cmd(vx: float, vy: float, wz: float, body=None)   # body = {"roll","pitch","yaw","z"}
legs.stop(); legs.stand(); legs.sit()

# brain -> arm  (blocking calls with timeout, return True/False)
arm.go(name)      # "stow" | "ready" | "pick" | "drop"
arm.hand("open" | "close")

# everything -> ui
status = {"state": str, "counter": int, "tilt": [roll, pitch], "fps": {...}, "mode": "auto|semi|teleop", "log": [...]}
```

Rules: perception never moves anything; legs never read the camera; only the brain talks to both. This is what lets four people work in parallel.

## Mock mode (`HEXA_MOCK=1`)

The CubeBot code opens the serial port at **import time** (`devices/servo_uart.py`), so nothing runs on a laptop today. Fix first (30 min, R4): `hw/mock.py` provides `MockServo` (prints or logs angles), `MockIMU`, `MockCamera` (webcam or a recorded video). `Arm` already takes a `device` in its props, so it's a one-line injection. This lets legs/gait/brain/UI be developed without the robot.

## Reuse map (CubeBot -> HexaSweep)

| CubeBot file | Use | Change needed |
|---|---|---|
| `models/arm.py` | 3-DOF leg IK (`set_coord`, offsets `delta_a/b/c`, `invert`) | Add reachability assert (IK currently clamps `acos` silently). Re-measure link lengths `A,B,C` on the hexapod |
| `models/body.py` | `Body` with `set_rotation/position/move`, leg-count agnostic | Feed it 6 legs. Use for leveling and body sway |
| `config/robot_setup.py` | Leg geometry, mounts, channel map | New `config_hex.py`: 6 legs, mirrored `local_to_body`, 18 channels, `calib.json` offsets |
| `devices/servo_uart.py` | Pi->Pico link `"ch;angle\n"` @ 230400 | Import-time serial open -> lazy/mocked. Verify Pico firmware maps 18 channels |
| `pwm_servo/pwm_servo.cpp` | Pico PWM firmware (per-GPIO channels) | Only if channel count/pins differ. Reflash is about 10 min |
| `devices/wt901.py` | IMU roll/pitch over I2C `0x50` | Verify sign and zero on a level table (the `-180` offset is suspicious) |
| `ai/depth_worker.py`, `ai/ai_camera.py` | Hailo `scdepthv3` 320x256 depth + picamera2 lores stream | Repo snapshot has stale imports (`from depth_worker import ...`, `robot_controller` expects `ai_camera.shared_depth_value` that no longer exists). Budget 1-2 h. Recalibrate the depth threshold (`-5.6`) for our camera mount |
| `ai/hand_tracking.py` | Hailo hand detector (`get_last_hands()`) | Stretch only. Pattern for running any detection HEF |
| `ai/fast_depth.py` | YOLO decode and drawing code | Lift the detection decode for the ball/COCO HEF |
| `ai/models/best_ball_v8n.hef` | 1-class ball detector, 224x224, Hailo-8L | Primary detector if the debris are balls |
| `ai/models/yolov11n*.hef` | COCO detector (cup, bottle, sports ball...) | Second option for household objects |
| `control/joystick.py` | evdev gamepad | Hardcoded `/dev/input/event5`. Use for teleop plus **E-STOP button** |
| `pre_settings/servo_zero.py`, `gyro_test.py`, `camera_test.py`, `led_test.py` | Bring-up scripts | Use as-is on day 1 |
| `points/animation.py` | Quadruped waypoint gait | **Replace** with parametric tripod (`legs/gait.py`) |
| `RL/**` | ONNX policies for 4 legs | **Do not use** |
| Everything with `/home/vladimir/...` paths | | Move to a `paths.py`/env var |

## Hexapod locomotion design (R1)

- **Gait: alternating tripod.** Group A = {L1, R2, L3}, group B = {R1, L2, R3}. Three feet always on ground, so it is statically stable, so **no RL/balance loop needed**.
- Foot path in body frame, cycle time `T` (start 1.6 s), duty 50%:
  - stance: foot slides linearly `+s/2 -> -s/2` at ground height,
  - swing: `-s/2 -> +s/2` with a half-sine lift `h` (start 25-30% of leg height).
- Per-leg stride vector for turning: `stride = (vx, vy) + wz x r_leg` using the leg mount position `r_leg` (rotates in place when `vx=vy=0`).
- Time-based (not step-count-based like `Arm.set_position`), so speed does not depend on loop rate. Update at 30-50 Hz.
- **Body leveling:** IMU roll/pitch -> PI -> `body.set_rotation(roll, pitch)`, clamped to +-12 deg, 20 Hz. Independent of the gait, so it also works standing (tilted-board demo).
- **Stow pose for the arm** during walking: compact, low, over the body centre. Only deploy when stopped. Lower centre of gravity matters more than speed.
- Preview the gait offline first (`tools/gait_preview.py`, matplotlib + IK reachability check) before touching servos.

## Perception design (R2)

- Camera: picamera2 `lores` 320x256 stream (already working in CubeBot) for depth; a second stream or crop for the detector. Keep autofocus continuous or **lock exposure/focus** for the demo.
- Detector: `best_ball_v8n.hef` (one class). Fallback in parallel: **HSV blob** on the debris colour, about 30 lines of OpenCV, no Hailo needed, very stable indoors.
- Pick zone: place debris at the arm's sweet spot, read the detection box, store `pickzone.json` = `{cx, cy, w, tol}`. ALIGN just drives the error to zero.
- Obstacle: bottom strip of the depth map, split left/centre/right, thresholds measured empirically (log values with a box at 15/30/60 cm).

## Brain design (R4)

- Plain enum + one function per state, 20-30 Hz tick, per-state **timeouts**, single **ESTOP** flag checked first every tick. No framework.
- Modes on the dashboard: `auto` (full loop), `semi` (operator presses OK before PICK), `teleop` (gamepad only). This is the demo's graceful-degradation ladder.
- Log every tick to `runs/<timestamp>.jsonl` (state, targets, tilt) so failed runs can be diagnosed in 2 minutes.
- Status LEDs: solid = ready, blink = moving, all on = ESTOP (reuse GPIO 17/27/22).

## Arm design (R3)

- Driver depends on the partner's stack (SO-100 normally uses Feetech bus servos over a USB serial board; check at hour 0). Wrap whatever works into `arm.go(name)`.
- **Record & replay:** torque off, move the arm by hand, save joint positions to `poses.json` (`stow`, `ready`, `pick`, `drop`, plus 1-2 intermediate waypoints). Replay with interpolation and speed limit.
- Hand: `open` and `close` presets. Verify closure (position reached vs blocked) to detect a missed grasp, then retry once.

## Power and wiring (biggest silent killer)

| Rail | Feeds | Note |
|---|---|---|
| 5 V / 5 A clean | Pi 5 + Hailo | Own buck converter; never from servo rail |
| Servo rail (leg PWM servos) | 18 servos | Peak current can exceed 10 A. Big BEC/battery or bench PSU while developing |
| Arm rail | SO-100 servos (7.4-12 V typ.) + hand | Separate supply, common ground |

Common ground everywhere. Fuse or inline current-limit. Measure stall current before the first walking test. **Develop on a bench PSU with the robot on blocks**, battery only for final runs.

UART capacity check: 18 servos x 50 Hz x about 9 bytes = about 8 KB/s vs about 23 KB/s at 230400 baud. OK, but keep the update at 30-50 Hz and only send changed channels.

## Ops

- Dashboard: Flask (or stdlib `http.server`) MJPEG at about 10 FPS, low res. Pi runs its **own WiFi hotspot or Ethernet** so venue WiFi is irrelevant.
- Deploy: `git pull` on the Pi. Only `main`. Keep a `demo.yaml` (locked thresholds) separate from dev config.
