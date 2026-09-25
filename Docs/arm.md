# Arms: one RoArm-M3 Pro + one SO-101

We have **one RoArm-M3 Pro** and **one SO-101 follower**. No leader arms, so poses are taught by turning torque off and moving the arm by hand. The TriArm (tnkr.ai) is no longer part of the plan.

Both arms are fixed at the dock and never move at the same time.

| Arm | Job | Why this arm |
|---|---|---|
| **RoArm-M3 Pro** | **Picker:** dock pocket → bag (deposit) or reject bin | Longer reach (0.5 m), metal servos, can reach both bins |
| **SO-101** | **Scanner:** rolls the bottle in the pocket so the overhead camera can see the barcode | Short, precise moves close to the pocket. Backup picker if the RoArm fails |

## RoArm-M3 Pro

| Fact | Value | Source |
|---|---|---|
| DOF | 5 + gripper | `RoArm-M3/README.md` |
| Payload | 0.2 kg at 0.5 m reach (an empty 0.5 L PET bottle is about 25 g, a can about 15 g) | same |
| Workspace | up to 1 m diameter, 360 deg base | same |
| Power | 7-12.6 V, use the 12 V 5 A supply | same |
| Control | JSON commands over USB serial (or HTTP `http://<ip>/js?json=...`) | `RoArm-M3/python_demo/` |
| Joints | `{"T":102,"joints":[...],"rads":[...],"spd":50,"acc":10}` | JSON command table |
| Gripper | `{"T":106,"cmd":<rad>,"spd":0,"acc":0}` | same |
| Torque off/on (for teaching by hand) | `{"T":210,"cmd":0}` / `{"T":210,"cmd":1}` | same |

- It runs guarded firmware 0.84-s1 with ESP-NOW pairing from the earlier leader-follower setup (`RoArm-M3/deployments/2026-09-08-m3-pro/`). **Verify on day 1** that it accepts serial joint commands with no leader present, and turn off follower mode if it doesn't.
- **Teaching:** torque off, move by hand, read the joint angles back, save as a named pose. Check which command returns joint feedback (T:105 in the Waveshare firmware, verify in the JSON table), then torque on.
- Support the arm by hand when turning torque off, so it doesn't drop.

## SO-101 follower

| Fact | Value | Source |
|---|---|---|
| Servos | 6x Feetech STS3215 serial bus servos (5 joints + gripper) | `SO-Arm-101/README.md` |
| Control | USB serial bus adapter, LeRobot (`so101_follower`) or the Feetech SDK directly | same |
| Calibration | `lerobot-calibrate --robot.type=so101_follower --webui` or `pi_servo_studio.py` (3-point per joint, 30% torque cap) | `SO-Arm-101/Software/WEBUI_CALIBRATION.md` |

- **Teaching:** torque off on the bus, move by hand, read positions, save. Same `poses.json` format as the RoArm.
- **Roll move:** gripper closed as a "finger" with foam or rubber tape on its tip. Put it on the top of the lying bottle and drag sideways about 5 cm, so the bottle rolls in place in the V-groove of the pocket. Repeat 3-4 times to turn it a full circle.
- Calibrate once and commit the calibration file. Recalibrating on the demo day costs 20 minutes.
