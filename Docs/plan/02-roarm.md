# 02: RoArm-M3 (the picker)

**Done when:** `python roarm_wifi.py wiggle` passes from the Pi, and `park`, `bins.*`, `pick_z`, `pick_t`, `grip_open` and `grip_closed` in `config.json` are real measured values.

## 1. Power: check this first, every time

- [ ] Supply is 2S LiPo or a buck/bench supply set to **7.4–8.4 V**. Measure it with a multimeter before connecting. **Never the 12 V adapter** (the shoulder has Feetech 7.4 V servos).
- [ ] After power-up, `python roarm_wifi.py where` should show real values (resting pose roughly `b≈0 s≈0 e≈1.6`). If it shows placeholders (`b=π s=-π e=-1.57 ...`), the servo bus is not answering. Stop and debug before sending any motion.

## 2. WiFi

- [ ] The arm should join `4G-Gateway-7417` by itself. If it's stuck retrying an old SSID, `T:404`/`T:407` fail. What worked: use `T:206` to rewrite line 1 of `wifiConfig.json`, then reboot the arm.
- [ ] Put the IP in `roarm_ip` and run `python roarm_wifi.py wiggle` (up 40 mm, back, then gripper open and close).

## 3. Fix the layout (do this once, then don't move anything)

- [ ] Put the RoArm base, the trash bin and the three containers in their final places. **Tape or screw them down.** Every taught value below, and the calibration in step 05, is only valid for this exact layout.
- [ ] The whole bin floor must be inside `reach_mm` (120–380 mm from the base axis). Check that the far corners are reachable before taping.

## 4. Teach positions

Jog the arm with `python roarm_console.py` (W/S/A/D/R/F = x/y/z, T/G = tilt, Z/X = gripper, P = print the pose), or with its web page at `http://<roarm_ip>/`. Copy the `P` readout into `config.json`:

- [ ] `pick_t`: the wrist pitch at which the gripper points straight down. Check the sign.
- [ ] `grip_open` / `grip_closed`: open wide enough for a can lying on its side, and closed enough to hold a crumpled paper ball.
- [ ] `pick_z`: fingertips about 5 mm above the bin floor.
- [ ] `approach_dz`: height above `pick_z` for moving in and out. It must clear the bin wall (default 100).
- [ ] `park`: a pose outside the camera view and outside the SO-101's path.
- [ ] `bins.cans_bottles`, `bins.paper`, `bins.plastic`: above each container's opening, high enough to clear the rims.
- [ ] Walk the path by hand once: park, above the bin, down to `pick_z`, up, to each container, back to park. Nothing should collide.

## Pitfalls

- `goto()` waits until the measured xyz is within 15 mm of the target. The Feetech shoulder sags about 0.05 rad under load, so if `goto` times out near a target, raise `tol` rather than the speed.
- `T:104` with `spd: 0` is silently ignored. The code refuses it.
- Feedback over HTTP arrives about once a second, so a jog's readout can be a second stale.
