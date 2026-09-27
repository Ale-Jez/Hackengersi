# Raspberry Pi 5: driving base

Two MAB **MA-D-GL40 KV70** direct-drive actuators (one per wheel, each with its own MD driver) on a **CANdle** USB-to-CAN FD dongle, and one forward-looking USB camera. The Pi steers at AprilTags (36h11, see `../AprilTags`) and stops for obstacles.

| File | Does |
|---|---|
| `drive.py` | route driving: hybrid steering, tag approach, search, obstacle stop, `selftest` |
| `motors.py` | the two wheels over CANdle (velocity mode, speed ramp, watchdog feed), `ping` / `test` / `jog` |
| `vision.py` | newest-frame camera, AprilTag detection, floor-colour obstacle check, `floor` / `snap` |
| `stream.py` | live annotated camera view at `http://<pi ip>:8000` (tag id, px, distance, corridor, detection rate) |
| `app.py` | station panel at `http://<pi ip>:8000`: one button per station, manual driving, taught routes, 90° turns, STOP, battery and drive link status |
| `config.json` | CAN ids, wheel signs, speeds, steering and obstacle tuning, the route |

## Station panel (`app.py`)

Stations are in `config.json` `stations` (dok = tag 0, biurko = tag 1, sortownia = tags 2, 3, 4). A trip from a station to another replays the route taught for that pair (`routes.json`): every move drives until the wheel encoders turned as far as when taught. With `use_tags` on, the car then docks `stop_cm` in front of the station tag if that tag was in view when the route was saved; with it off (now), routes alone drive the car.

Teaching: stand the car at the start station, "Ucz trasy", drive with the arrows and the 90° buttons to the target, "Zapisz trasę". Moves made after saving (e.g. turning round to park) are offered to be appended to that route. A 90° turn is `π/2 × (track_m / 2) / wheel_radius_m` of wheel rotation, unless "Kalibruj obrót" measured it (`calib.json`).

For a presentation, `http://<pi ip>:8000/demo` shows only one big button per leg (`demo_legs`, default dok → biurko → sortownia → dok), live only for the leg that starts where the car stands, and a STOP that also resets the car to the first station (`home`) so the run can start over.

If the CANdle drops off USB, the wheels stop, a running trip is aborted and the link is re-attached every second; the panel shows the link state. Only one process can own the camera and the CANdle: stop `app.py` before `stream.py`, `motors.py`, `drive.py` or `course.py`.

## Behaviour

| Situation | What the car does |
|---|---|
| Tag slightly off centre (\|err\| < `pivot_enter`) | **Arc:** both wheels forward, the inner one slowed by `steer_gain × err` |
| Tag far off centre | **Spin in place** at `spin_speed` (wheels opposite) until \|err\| < `pivot_exit` |
| Getting close | Speed drops linearly over the last `slow_px` of tag growth (never below `min_speed`), stop at `stop_px` |
| Tag not in view | Brake to a standstill, then spin toward the side it was last seen; error after `search_timeout` s |
| Something in the corridor | After `obstacle_frames` frames: **halt at once** (no ramp), wait until clear for as many frames; error after `obstacle_timeout` s. Off during the final `slow_px` of an approach, where the tag's own station fills the corridor |
| Program dies / Ctrl-C | Speed goes to 0 and the drives are disabled; if the Pi hangs, the MD watchdog stops the motors |

`err` runs from -1 (tag at the left image edge) to +1 (right edge). Every speed in the config is a fraction (-1..1) of `max_wheel_rad_s`, and the ramp limits changes to `accel_rad_s2`.

## Setup

On the Pi (Raspberry Pi OS 13), OpenCV and picamera2 come from apt, and `candlesdk` has no ARM wheel on PyPI and its sdist lacks submodules, so build it from git:

```sh
sudo apt install -y python3-venv python3-opencv python3-picamera2 libusb-1.0-0-dev build-essential git
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
git clone --recursive --depth 1 https://github.com/mabrobotics/CANdle-SDK.git ~/CANdle-SDK
CMAKE_POLICY_VERSION_MINIMUM=3.5 pip install ~/CANdle-SDK
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0069", ATTR{idProduct}=="1000", MODE="0666"' | sudo tee /etc/udev/rules.d/99-candle.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
python drive.py selftest
```

- **CANdle:** plug it into USB and power the actuators. The Python package `candlesdk` imports as `pyCandle`. The CANdle HAT (SPI) is not supported by its Python bindings yet, so use the USB dongle.
- **Drive gains:** the factory velocity PID is too soft to turn the loaded car, so `motors.py` writes `vel_pid` (kp, ki, kd, windup) and `max_torque` on every start. It writes the registers one by one, because `MD.setVelocityPIDparam()` returns OK but changes nothing in candlesdk 1.5.0. Nothing is saved to the drives.
- **Camera:** `"camera": "csi"` is the ribbon camera (Camera Module 3, through picamera2, continuous autofocus). A number is a USB camera index (`v4l2-ctl --list-devices`). `camera_flip` turns the image 180° for a camera mounted upside down.

## Networks

The Pi (`malina-auto`, user `hackengersi`) joins WiFi on its own. The saved networks live in NetworkManager on the Pi, not in this repo, and the passwords stay off git.

| SSID | Priority | Pi's address range |
|---|---|---|
| `POCO X8 PRO` / `POCO X8 Pro` (phone hotspot) | 10 | `10.192.231.x` |
| `4G-Gateway-7417` | 5 | `192.168.32.x` |
| `iPhone (Jan)` (phone hotspot) | 0 | `172.20.10.x` |

When several networks are in range, the Pi prefers the higher priority when it connects, but it does not leave a working connection. SSIDs are case-sensitive, so copy the exact name from a laptop connected to that network (`netsh wlan show interfaces` on Windows).

Add a network (the WiFi country is PL):

```sh
sudo nmcli connection add type wifi ifname wlan0 con-name "<ssid>" ssid "<ssid>" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<password>" connection.autoconnect-priority 5
nmcli -f NAME,AUTOCONNECT-PRIORITY,DEVICE connection show   # list; `sudo nmcli connection up "<ssid>"` switches now
```

**Finding the Pi:** `malina-auto.local` often does not resolve from Windows, so put the laptop on the same network and look for an open SSH port:

```powershell
$pre = (Get-NetIPAddress -AddressFamily IPv4 -InterfaceAlias Wi-Fi).IPAddress -replace '\.\d+$','.'
1..254 | % { $c = New-Object Net.Sockets.TcpClient; if ($c.ConnectAsync("$pre$_",22).Wait(150) -and $c.Connected) { "$pre$_" }; $c.Close() }
```

Before cutting the power, shut down with `sudo poweroff` (or hold the power button about 2 s). A hard power cut once corrupted an SD card.

## Bring-up (wheels off the ground first)

1. `python motors.py ping` prints the drive ids. Put them in `left_id` / `right_id`.
2. `python motors.py test` runs left, then right, then both, slowly forward. If the wrong wheel moves, swap the ids. If a wheel turns backwards, flip its `*_sign`.
3. Set the current / torque limit and the CAN watchdog on each drive with MAB's `candletool` (or MD tool). The GL40 is direct drive (about 0.25 Nm rated), so check that it can push the loaded can on your floor before tuning speed.
4. Put it on the floor: `python motors.py jog` (w/s, a/d spin, z/c arc turn around the stopped wheel, space to stop, q to quit).
5. Obstacles: point the car at clear floor, run `python vision.py floor`, then `python vision.py snap` with a box in front. Red pixels in the cyan corridor are "not floor". Tune `obstacle_roi` (fractions of the frame) and `obstacle_frac`.
6. Tags: hold a tag at a known distance D cm in front of the camera, read its side px in `stream.py`, and set `tag_focal_px = px × D / tag_size_cm`. The stop is `stop_px = tag_focal_px × tag_size_cm / distance` (6 cm tag: 284 px at 20 cm). Stop `stream.py` before other camera scripts, because only one process can open the camera. Then run `python drive.py tag 1`. If it zig-zags, lower `steer_gain`. If it spins too often, raise `pivot_enter`.
7. `python drive.py run`.

**Limits:** the obstacle check only looks at colour, so an obstacle the same colour as the floor is invisible, and so is anything outside the corridor. Strong light changes after `vision.py floor` need a re-learn. `max_wheel_rad_s` is 12 rad/s by default. The KV70 could go more than 10× faster at 24 V, so raise it carefully.
