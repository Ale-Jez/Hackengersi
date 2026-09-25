# Raspberry Pi 5: trash can controller

| File | Does |
|---|---|
| `main.py` | route driving (AprilTags + timed moves), sort cycle, calibration |
| `roarm_wifi.py` | RoArm-M3 over WiFi (HTTP `/js` commands, `/ws` feedback) |
| `so101.py` | SO-101 camera arm over USB, save/go named poses |
| `vision.py` | USB camera, AprilTag detection, pixel to arm mapping, Brev LLM client |
| `config.json` | every IP, pose, height and bin position |

## Setup

```sh
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python main.py selftest && python vision.py && MOCK=1 python roarm_wifi.py
```

- **Brev:** on the Brev box run `bash Brev/setup.sh` (installs and serves `Qwen/Qwen2.5-VL-7B-Instruct` on port 8000, prints the URL and token). Put the URL in `brev_url` and `export BREV_KEY=...` on the Pi.
- **RoArm WiFi:** it starts as an access point at `192.168.4.1`. Either connect the Pi to it, or join the arm to the Pi's network from the arm's web page and set `roarm_ip`.
- **Camera:** find the index with `v4l2-ctl --list-devices`, then set `camera`.

## Teach and calibrate (once per build)

1. SO-101 poses: `python so101.py save look` (camera above the can, looking down), `save stow` (out of the RoArm's way), `save drive` (camera looking forward for the tags).
2. RoArm positions: jog the arm on its web page `http://<roarm_ip>/`, then `python roarm_wifi.py where`. Copy x, y, z into `park` (outside the camera view), `bins.*` (above each container) and `pick_z` (the can's floor). Check the sign of `pick_t` (wrist pitch) so the gripper points down.
3. Tape AprilTag 36h11 id `calib_tag` on the RoArm gripper, then run `python main.py calibrate`.
4. Check the vision alone: `python vision.py photo p.jpg && python vision.py ask p.jpg` gives `answer.jpg`.
5. Run it: `python main.py sort`, then `python main.py run`.

**Open item:** `set_wheels()` in `main.py` is not wired up yet. The description doesn't say what drives the can.
