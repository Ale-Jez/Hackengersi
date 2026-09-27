# Troubleshooting

## 🦾 RoArm-M3 Pro

| Problem | Solution |
|---------|----------|
| Arm doesn't respond (USB) | Is `roarm_usb.py` running? Check `systemctl --user status roarm-usb`. USB cable in the port labelled USB (CP2102, not the one labelled GROVE). `ls /dev/ttyUSB*`. |
| Arm doesn't respond (WiFi) | Check WiFi connection. Try `http://<roarm_ip>/` in browser. Restart arm power. |
| `roarm_usb.py` shows "503 no feedback" | Arm is powered off or rebooting (firmware boot takes ~5 s). Check power supply voltage. |
| Arm freezes for ~10 seconds | You sent `T:0`. **Never do this.** `roarm_wifi.py` blocks it, but direct HTTP won't. |
| Shoulder overheating | Stop immediately. The panel (`/roarm_panel`) shows temps live; motion is blocked above 65 °C. Let it cool 15+ min. Reduce `spd` to ≤ 0.2. |
| Arm drops suddenly | `T:210` (torque off) was sent, or power supply voltage too low. Always support by hand when toggling torque. |
| Gripper doesn't close fully | The `grip_closed` value in config might need tuning. Check for mechanical obstruction. |
| "not answering" errors | RoArm is busy executing a long move. The command still went through — wait for position feedback. |
| Wrong IP after power cycle | RoArm starts as AP at `192.168.4.1`. If using WiFi: re-join your network via its web page. If using USB: IP is always `localhost:8766`. |
| Upside-down mounting is wrong | Check the "odwrocony" (upside-down) setting in roarm_calibration.json and the panel. |

## 🔧 SO-101

| Problem | Solution |
|---------|----------|
| USB serial not found | `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`. Try unplugging/replugging. Check `dmesg`. |
| Servo jitters or doesn't move | Calibrate with `lerobot-calibrate` or `pi_servo_studio.py`. Check 30% torque cap. |
| Camera image is black | Run `v4l2-ctl --list-devices` to find the right index. Set it in `config.json → camera`. |
| "pozycja M nie ustawiona" | Press `M` in the web UI (`http://<pi>:8765/`) to save the home/look pose. |
| Barcode not reading | Ensure good lighting. Clean camera lens. Try rotating the bottle (the arm does this automatically up to 3 times). |

## 🚗 CubeBot / Driving Base

| Problem | Solution |
|---------|----------|
| Wheels don't spin | `python motors.py ping` — check CAN IDs match config. Is power on? Is CANdle plugged in? |
| Wrong wheel direction | Flip `left_sign` or `right_sign` in `config.json`. |
| Robot zig-zags at tag | Lower `steer_gain`. If it spins too often, raise `pivot_enter`. Use `stream.py` (`http://<pi>:8000`) to watch live. |
| Obstacle false positives | Re-learn floor colour: `python vision.py floor`. Avoid strong lighting changes. |
| "blocked for N s" timeout | Real obstacle, or floor colour changed. Run `vision.py floor` again. |
| Tag not detected | Check tag is flat, matte, well-lit. Print size should be ≥ 6 cm. Verify `DICT_APRILTAG_36h11`. |
| Camera busy / can't open | Only one process can use the camera. Stop `stream.py` before running `drive.py` or `vision.py snap`. |
| CSI camera not working | Check `"camera": "csi"` in config. Is picamera2 installed? (`sudo apt install python3-picamera2`). Check ribbon cable. |

## ☁️ Brev (Cloud GPU)

| Problem | Solution |
|---------|----------|
| `setup.sh` fails at nvidia-smi | Wrong instance type — you need a GPU instance (≥ 24 GB VRAM for 7B model). |
| Server not ready after 15 min | Check `~/vllm.log` for the first ERROR line. Usually: OOM, wrong CUDA version, or network issue downloading the model. |
| Pi can't reach Brev | Both must be on the same Tailscale network. Run `tailscale status` on both. Check `brev_url` in config. |
| "Unauthorized" from vLLM | `$BREV_KEY` on the Pi must match the key in `~/.brev_key` on Brev. |
| LLM returns garbage | Check prompt in `vision.py`. The model expects a specific JSON format. Try `python vision.py photo p.jpg && python vision.py ask p.jpg`. |
| YOLO worker not running | `pgrep -f yolo_laptop.py`. Check `~/yolo.log`. Re-run `bash Brev/setup.sh`. |

## 📐 Calibration

| Problem | Solution |
|---------|----------|
| "only N points seen, need 4+" | Tag not visible from enough positions. Move the `look` pose higher, or check tag placement on gripper. |
| High calibration error (> 25 mm) | Re-run `python roarm_pick.py calibrate tag` with the tag clean (no glare) and fully in view. |
| Calibration off by ~5 mm on one side | Feetech shoulder sag — the arm droops under its own weight. Re-calibrate with the arm in its working orientation. |
| Pick misses by 2+ cm | Recalibrate. The layout may have shifted since last calibration. |
| Homography not saved | Check file permissions on `config.json` / `roarm_calibration.json`. |
| VLM calibration fails | Check Brev connection (`python vision.py ping`). Good lighting helps the VLM find the gripper tip. |

## 🌐 Network

| Problem | Solution |
|---------|----------|
| Can't SSH to Pi | Check hostname: `malina` (arm Pi) or `malina-auto` (driving Pi). Verify same WiFi/Tailscale. |
| Pi lost WiFi | Connect via Ethernet or direct console. Check `/etc/wpa_supplicant/` or NetworkManager. |
| Tailscale not connected | `sudo tailscale up --hostname <name>`. Log in via the printed URL. |

## 🆘 Emergency Procedures

1. **Arm moving dangerously:** Pull the power cable. Support the arm if torque cuts.
2. **CubeBot driving into things:** Press Ctrl+C on the driving Pi, or pull CANdle USB. The MD watchdog will stop motors within 100 ms.
3. **Brev costs running up:** `bash Brev/setup.sh stop` then shut down the instance.
4. **Deploy new code fast:** `python dorm_keeper/deploy_to_pi.py --autostart --log` pushes and restarts the service.
5. **Everything broken:** Deep breath. Run selftests. Check one component at a time. Shut the Pi down cleanly with `sudo poweroff`.
