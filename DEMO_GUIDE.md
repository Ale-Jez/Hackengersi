# 🎬 Demo Day Guide

## Pre-Demo Checklist (30 minutes before)

### Power & Connections
- [ ] RoArm powered at **7.4–8.4 V** (check with multimeter!)
- [ ] RoArm USB cable in Pi #1 (CP2102 port on the arm)
- [ ] `roarm_usb.py` running (or roarm-usb systemd service enabled)
- [ ] SO-101 USB connected to Pi #1
- [ ] Camera USB connected to Pi #1
- [ ] CubeBot motors powered, CANdle USB in Pi #2
- [ ] Both Pis powered and booted
- [ ] Brev instance running (`bash Brev/setup.sh`)

### Software
- [ ] `python main.py selftest` passes on Pi #1
- [ ] `python drive.py selftest` passes on Pi #2
- [ ] `python vision.py ping` returns OK (Brev connection)
- [ ] `so101_station.py` web UI accessible at `http://malina:8765/`
- [ ] `stream.py` live camera at `http://malina-auto:8000/` (useful for tuning)

### Calibration
- [ ] Camera-to-arm calibration is fresh (recalibrate if anything moved)
- [ ] SO-101 "look", "stow", "drive" poses verified
- [ ] RoArm "park", "bins" positions verified
- [ ] Floor colour learned for obstacle detection

### Demo Materials
- [ ] 3–5 demo bottles/cans (mix of deposit and non-deposit)
- [ ] AprilTags posted at route stops (flat, well-lit)
- [ ] Three sorting bins in position
- [ ] One non-deposit item (glass jar or juice carton) for reject demo

---

## 🎙️ Demo Script

### Act 1: The Problem (30 seconds)
> "Poland's new deposit return system (Kaucja) requires sorting billions of containers. Current RVM machines are expensive and stationary. What if the sorting came to you?"

### Act 2: Meet the Robot (1 minute)
> "This is our autonomous trash sorter. It drives itself using AprilTag navigation, sees inside the bin with AI vision, identifies each item with a Qwen2.5-VL model, reads barcodes to check deposit eligibility, and sorts with two robotic arms."

**Demo: start `python main.py run`**

### Act 3: The Sort (2 minutes)
> "Watch — the camera arm looks inside, the AI identifies a Żywiec can as deposit-eligible, and the picker arm grabs it and drops it in the deposit bin. Non-deposit items go to recycling."

**Let the robot do 2–3 items.**

### Act 4: The Tech (1 minute)
> "Two Raspberry Pi 5s. Two robotic arms. Computer vision running on a cloud GPU via Tailscale. Self-calibrating — the robot teaches itself where things are by moving a bottle around and watching with its camera. Built against the Kaucja.pl OpenAPI flow."

### Act 5: Q&A

---

## 🔄 Reset Between Demos

```bash
# On Pi #1:
python main.py sort    # Clear any remaining items

# Refill the bin with demo items
# Verify AprilTags are in position
# Check RoArm temperature on the panel (http://malina:8765/roarm_panel)
```

---

## 🚨 If Things Go Wrong

| Failure | Recovery | Time |
|---------|----------|------|
| RoArm stops responding | Check USB cable. `systemctl --user restart roarm-usb`. Restart `main.py` | 30 sec |
| LLM returns wrong labels | `python vision.py photo p.jpg && python vision.py ask p.jpg` to debug. Check lighting. | 1 min |
| CubeBot overshoots tag | `python drive.py tag <id> <stop_px>` to manually reposition. Check live view at `http://malina-auto:8000/` | 30 sec |
| Camera image is dark | `v4l2-ctl --set-ctrl brightness=128` or move to better lighting | 15 sec |
| Calibration is off | `python main.py calibrate` (takes ~2 min with tag on gripper) | 2 min |
| Nothing works | Run the sort cycle only (skip driving): `python main.py sort` | Instant |
| *Total disaster* | Show the selftest + explain architecture on whiteboard. The code runs. | — |

---

## 💬 Key Talking Points

- **Self-calibrating:** the robot teaches itself pixel-to-millimeter mapping (AprilTag or VLM-based)
- **Real DRS integration:** follows Kaucja.pl API flow (mock server, ready for real credentials)
- **No ROS:** pure Python, runs on stock Raspberry Pi OS
- **Obstacle-aware:** floor-colour learning for safety stops
- **Cloud-edge split:** heavy AI on GPU, real-time control on Pi
- **Every module has tests:** `selftest` commands work without any hardware
- **USB bridge:** RoArm connected over USB serial — no WiFi needed, same HTTP API
- **Live camera stream:** `stream.py` gives a browser view with tags and obstacles
