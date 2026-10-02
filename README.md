<p align="center">
  <h1 align="center">Bin There, Sort That — Autonomous Trash Sorting Robot</h1>
  <p align="center">
    <em>A self-driving trash can that navigates indoor spaces, identifies waste with AI vision, and sorts recyclables with robotic arms.</em>
  </p>
</p>

---

## What It Does

A **CubeBot** (motorized mobile platform) drives around an indoor space following a pre-programmed route using **AprilTag** navigation. When it arrives at a sorting station, two robotic arms work together:

1. **SO-101 arm** — positions a camera above the trash can, detects bottles/cans via YOLO, reads barcodes (EAN-13) to identify deposit-eligible containers, and tracks objects in real-time
2. **RoArm-M3 Pro** — picks up each item and drops it into one of the sorting bins based on AI classification

A cloud GPU (**Nvidia Brev**) runs a **Qwen2.5-VL-7B** vision-language model that analyzes photos of the trash can contents and tells the robot what each item is and where to put it.

### Sorting Bins

| Bin | Contents |
|-----|----------|
| **Deposit (kaucja)** | Bottles and cans eligible for Poland's DRS deposit refund |
| **Paper** | Paper, cardboard, napkins |
| **Plastic** | Bags, wrappers, cups, other plastic |

---

## System Architecture

```
                                    ┌─────────────────────────┐
                                    │   Nvidia Brev (Cloud)   │
                                    │  Qwen2.5-VL-7B (vLLM)  │
                                    │  YOLO bottle detector   │
                                    └────────┬────────────────┘
                                             │ HTTPS (Tailscale)
                                             │
┌──────────────┐   USB servo bus   ┌─────────┴─────────┐  USB serial (CP2102) ┌──────────────┐
│   SO-101     │◄─────────────────►│   Raspberry Pi 5  │◄───────────────────►│  RoArm-M3    │
│  (scanner)   │                   │   8GB (malina)    │  roarm_usb.py →     │  (picker)    │
│  + USB cam   │◄── USB ──────────►│                   │  localhost:8766     │  5-DOF +     │
│  6-DOF       │                   │   main.py         │  (same HTTP API)    │  gripper     │
└──────────────┘                   │   vision.py       │                     └──────────────┘
                                   │   roarm_wifi.py   │
                                   │   roarm_usb.py    │
                                   │   so101.py        │
                                   └───────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│                     CubeBot (Driving Base)                                  │
│  Raspberry Pi 5 8GB (malina-auto)                                          │
│  2x MAB MA-D-GL40 KV70 direct-drive actuators                             │
│  CANdle USB-to-CAN FD dongle                                              │
│  Forward USB / CSI camera for AprilTag navigation + obstacle detection     │
│  drive.py · motors.py · vision.py · stream.py                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
.
├── Raspberry/            # Sorting station controller (Pi #1: arms + vision)
│   ├── main.py           # Route driving, sort cycle, calibration, selftest
│   ├── roarm_wifi.py     # RoArm-M3 HTTP client (/js commands, /ws feedback)
│   ├── roarm_usb.py      # RoArm USB-serial bridge → localhost:8766 (same HTTP API)
│   ├── roarm_console.py  # Keyboard jog for teaching RoArm positions
│   ├── so101.py          # SO-101 arm over USB (save/go named poses)
│   ├── vision.py         # Camera, AprilTags, homography, Brev LLM client
│   └── config.json       # All IPs, poses, heights, bin positions, tuning
│
├── Raspberry-Auto/       # Driving base controller (Pi #2: wheels + navigation)
│   ├── drive.py          # Route driving: tag approach, search, obstacle stop
│   ├── motors.py         # Wheel control via CANdle (velocity mode, ramp, watchdog)
│   ├── vision.py         # Camera, AprilTag detection, floor-colour obstacle check
│   ├── stream.py         # Live annotated camera view at http://<pi>:8000
│   └── config.json       # CAN IDs, wheel signs, speeds, steering tuning, route
│
├── dorm_keeper/          # SO-101 camera station + RoArm picking
│   ├── so101_station.py  # SO-101 all-in-one: camera, YOLO detection, barcode reading,
│   │                     #   arm tracking, web UI (http://<IP>:8765/so101), Xbox controller
│   ├── roarm_pick.py     # Camera → RoArm calibration (AprilTag / VLM) and grab
│   ├── roarm_panel.py    # RoArm control page (http://<IP>:8765/roarm_panel):
│   │                     #   per-joint jog, hold-to-move, servo temperature & load,
│   │                     #   upside-down support
│   ├── demo.html         # Presentation view (http://<IP>:8765/demo)
│   ├── yolo_laptop.py    # GPU-accelerated YOLO worker (runs on Brev)
│   └── deploy_to_pi.py   # Deploy code to the Pi + manage dorm-keeper systemd service
│
├── Station/              # Early-stage station code (RoArm via USB serial)
│   ├── roarm.py          # RoArm-M3 over USB serial (JSON lines, 115200 baud)
│   ├── keyteleop.py      # Keyboard jog (Windows console)
│   ├── camera.py         # Camera + barcode scanning
│   └── xbox_teleop.py    # Xbox controller teleoperation
│
├── AprilTags/            # Printable AprilTag markers (36h11 family)
│   ├── tag36h11_00-05    # Tag images (ID 0 = calibration, 1-2 = route, 3-5 = spare)
│   └── print.tex         # LaTeX for printing (A4, 6 cm black square)
│
├── Brev/                 # Cloud GPU setup (Nvidia Brev instance)
│   ├── setup.sh          # Installs vLLM + Qwen2.5-VL, starts server, configures Tailscale
│   └── setup.md          # Quick clone + run instructions
│
├── Docs/                 # Technical documentation
│   ├── description.md    # Project concept overview
│   ├── arm.md            # RoArm-M3 Pro & SO-101 hardware specs and usage
│   ├── bom.md            # Bill of materials
│   ├── drs-api.md        # Kaucja.pl deposit system API documentation
│   ├── plan/             # Step-by-step build & calibration plan (7 phases)
│   └── RoArm-M3/         # RoArm hardware docs + custom firmware (0.84-temp)
│
├── requirements.txt      # Python dependencies
└── .gitignore
```

---

## Hardware

| Component | Model | Qty | Role |
|-----------|-------|:---:|------|
| Single-board computer | Raspberry Pi 5 (8 GB) | 2 | Arm controller + driving base |
| Picker arm | RoArm-M3 Pro (5-DOF + gripper) | 1 | Picks trash, drops into bins |
| Scanner arm | SO-101 (6x Feetech STS3215) | 1 | Holds camera, tracks bottles, reads barcodes |
| Mobile base | CubeBot | 1 | Drives the trash can around |
| Drive motors | MAB MA-D-GL40 KV70 | 2 | Direct-drive wheel actuators |
| CAN adapter | CANdle USB-to-CAN FD | 1 | Motor bus communication |
| Cameras | USB + CSI (Camera Module 3) | 2 | Overhead view + forward navigation |
| Cloud GPU | Nvidia Brev instance | 1 | Runs Qwen2.5-VL-7B + YOLO |

---

## Quick Start

### Prerequisites

- Python 3.10+
- OpenCV, NumPy, requests, pyserial, zxing-cpp, feetech-servo-sdk
- candlesdk (for CubeBot wheels — build from git, see `Raspberry-Auto/README.md`)
- Tailscale (for Brev ↔ Pi networking)

### 1. Clone

```bash
git clone https://github.com/Ale-Jez/Hackengersi && cd Hackengersi
```

### 2. Set up the cloud GPU (Brev)

```bash
bash Brev/setup.sh && source ~/.bashrc
# Installs vLLM, downloads Qwen2.5-VL-7B (~16 GB), starts the server
# Prints the API key and URL to use on the Pi
```

### 3. Set up the sorting station (Pi #1 — `malina`)

```bash
cd Raspberry
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt

# Set the Brev API key
export BREV_KEY=<key from Brev setup>

# Start the RoArm USB bridge (if using USB instead of WiFi)
python roarm_usb.py &   # serves on localhost:8766

# Self-test (no hardware needed)
python main.py selftest
```

### 4. Set up the driving base (Pi #2 — `malina-auto`)

```bash
cd Raspberry-Auto
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt

# Self-test (no hardware needed)
python drive.py selftest
```

### 5. Teach & calibrate (once per physical build)

See the full calibration guide in [`Docs/plan/`](Docs/plan/00-overview.md).

**SO-101 poses:**
```bash
python so101.py save look    # camera above the can, looking down
python so101.py save stow    # out of the RoArm's way
python so101.py save drive   # camera forward for AprilTag navigation
```

**RoArm positions:**
```bash
# Jog via web UI: http://<roarm_ip>/
python roarm_wifi.py where   # read current x, y, z → copy into config.json
```

**Camera-to-arm calibration:**
```bash
# Tape AprilTag 36h11 ID 0 on the RoArm gripper
python main.py calibrate
```

### 6. Run

```bash
# Sort one cycle (arms only)
python main.py sort

# Full autonomous run (drive + sort)
python main.py run --loop
```

---

## AI Pipeline

```
USB Camera → SO-101 positions camera → Photo captured
                                            │
                                            ▼
                              ┌──────────────────────────┐
                              │    Brev GPU Server        │
                              │                          │
                              │  1. Qwen2.5-VL-7B        │
                              │     "What items are in   │
                              │      the trash can?"     │
                              │     → label, bin, x, y   │
                              │                          │
                              │  2. YOLO (bottle detect) │
                              │     → bounding boxes     │
                              │                          │
                              │  3. zxing-cpp (barcode)  │
                              │     → EAN-13 code        │
                              │     → Kaucja.pl lookup   │
                              └──────────────────────────┘
                                            │
                                            ▼
                              Homography: pixel → arm mm
                                            │
                                            ▼
                              RoArm picks item → drops in correct bin
```

---

## Navigation

The CubeBot follows a route defined in `config.json`:

```json
[
  {"tag": 1, "stop_px": 150},
  {"drive": [0.5, 0.5, 3.0]},
  {"sort": true},
  {"tag": 2, "stop_px": 150}
]
```

| Step type | Meaning |
|-----------|---------|
| `tag` | Drive toward AprilTag until it appears `stop_px` pixels wide |
| `drive` | Blind timed move: `[left_speed, right_speed, seconds]` (obstacle check still active for forward moves) |
| `sort` | Run the full sorting cycle at current position |
| `wait` | Pause for N seconds |
| `cmd` | Execute a shell command and wait (e.g., `ssh malina 'python main.py sort'`) |

**Live camera view:** `python stream.py` on Pi #2 serves an annotated MJPEG stream at `http://<pi>:8000` — shows tag IDs, pixel size, estimated distance, the obstacle corridor, and detection rate. Useful for tuning `stop_px` and `tag_focal_px`.

**Steering:** hybrid arc/spin. Small heading error → both wheels forward, inner wheel slowed. Large error → spin in place. Hysteresis prevents oscillation.

**Obstacle detection:** floor-colour learning. If the corridor ahead doesn't look like floor for N frames → emergency stop. Resumes when clear.

---

## Safety Rules

> **These rules exist because hardware has already been damaged. Follow them.**

| Rule | Reason |
|------|--------|
| **RoArm: 7.4–8.4 V ONLY** | Shoulder uses Feetech STS3215 (7.4 V rated). The 12 V adapter has already damaged servos. |
| **Never send `T:0`** | Freezes firmware for ~10 s; arm can drop. `roarm_wifi.py` blocks this command. |
| **Keep RoArm slow** (`spd ≤ 0.25`) | Only handle light, empty items. The panel shows servo temperatures and loads; motion is blocked above 65 °C. |
| **`T:210` (torque off):** hold the arm | It moves to a fixed pose first. Support by hand to prevent drops. |
| **CANdle:** wheels off the ground first | Test wheel direction before putting CubeBot on the floor. |
| **Shut Pi down cleanly** | `sudo poweroff` — a hard power cut has already corrupted an SD card. |

---

## Deposit System (Kaucja.pl)

The project integrates with Poland's Deposit Return System (DRS) via the [Kaucja.pl OpenAPI](https://cdn.kaucja.pl/gcdeposits/media/2025OpenAPIKaucjaplszybkistartdlasklepwiproducentwRVM.pdf):

- **Barcode scanning** → EAN-13 read by zxing-cpp
- **Product lookup** → `api.kaucja.pl/buf/pos/product/{ean}`
- **Transaction flow** → `POST /transaction` (mock server during hackathon)
- **Voucher** → generated for deposit-eligible items

Currently uses a local mock server. Real API access requires shop registration.

---

## Testing

Every module has a `selftest` command that runs without hardware:

```bash
python main.py selftest       # Sorting logic (mock arms, camera, LLM)
python drive.py selftest      # Steering, obstacle, approach logic
python roarm_pick.py --test   # Calibration, kinematics and grab logic (mock arms and camera)
python roarm_usb.py --test    # USB-serial bridge HTTP layer (no arm needed)
```

---

## Development

This project evolved from a hackathon prototype (Alien Bazaar, Hackengersi team) into a full autonomous sorting system. Active development areas include:

- **Vision & AI**: Qwen2.5-VL-7B analysis, YOLO detection, barcode reading
- **Arm control**: RoArm-M3 picking optimization, SO-101 positioning & tracking
- **Navigation**: AprilTag-based route following, obstacle detection, steering tuning
- **Integration**: Multi-arm coordination, deposit system API, web UI for calibration & control

---

## License

Hackathon project — see individual component licenses for hardware SDKs.

---

<p align="center">
  <em>Bin There, Sort That — Autonomous Sorting in Action</em>
</p>
