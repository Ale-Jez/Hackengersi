# 01: Raspberry Pi setup

**Done when:** all three self-tests pass on the Pi, `python vision.py photo` saves a sharp frame, `python so101.py where` prints six joint angles, and a `curl` to Brev answers.

## Tasks

- [ ] Flash Raspberry Pi OS (64-bit) and enable SSH. `rpi5-setup-guide.html` has the full procedure.
- [ ] Join the Pi to the same WiFi as the RoArm (`4G-Gateway-7417`, `192.168.32.x`). The credentials are in `malina.txt`. Find the SSID by its key, not by line number: the file has two Pi blocks.
- [ ] Install the Python environment:
  ```sh
  cd Raspberry
  python3 -m venv .venv && . .venv/bin/activate
  pip install -r ../requirements.txt
  ```
- [ ] Run the self-tests without any hardware:
  ```sh
  python main.py selftest && python vision.py && MOCK=1 python roarm_wifi.py && python roarm_console.py --selftest
  ```
- [ ] **Camera:** run `v4l2-ctl --list-devices`, put the index in `config.json` as `camera`, then run `python vision.py photo p.jpg`. Check the photo is sharp at the working distance. If it isn't, fix the focus now, because calibration depends on it.
- [ ] **SO-101:** plug in the USB servo adapter (VID 1A86) and power the servos. Run `python so101.py where`. If port auto-detection picks the wrong device, set `so101_port` (for example `/dev/ttyACM0`).
  - Add the user to the `dialout` group (`sudo usermod -aG dialout $USER`, then log in again).
- [ ] **Brev:** on the Brev box, run `bash Brev/setup.sh` (see `Brev/setup.md`). Put the printed URL in `brev_url`, and on the Pi add `export BREV_KEY=...` to `~/.bashrc`. Test it:
  ```sh
  curl -H "Authorization: Bearer $BREV_KEY" $BREV_URL/v1/models
  ```
- [ ] **RoArm link:** run `python roarm_wifi.py where`. If you get no answer, see step 02 (WiFi section).

## Pitfalls

- The RoArm gets its IP by DHCP, so the IP can change after a reboot. Reserve it on the router, or find it again by its MAC (`F0:24:F9:10:E2:88`) with `arp -a` after a ping sweep. The ESP32 answers pings slowly, so use a timeout of 1 s or more.
- USB camera and SO-101 on the same Pi: use a powered hub if the camera drops frames while the servos move.
