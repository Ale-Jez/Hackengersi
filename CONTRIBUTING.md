# Contributing to Alien Bazaar

## 🛠️ Development Setup

```bash
git clone https://github.com/Ale-Jez/Hackengersi && cd Hackengersi
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
```

## 📋 Code Conventions

- **Language:** Python 3.10+
- **Style:** PEP 8 (except line length: 120 chars)
- **Comments:** Polish is OK for internal notes; public-facing docs in English
- **Config:** all hardware parameters go in `config.json`, not hardcoded
- **Testing:** every module has a `selftest` or `--test` command — run it before pushing

## 🧪 Before Every Push

```bash
python Raspberry/main.py selftest
python Raspberry-Auto/drive.py selftest
python dorm_keeper/roarm_pick.py --test
python Raspberry/roarm_usb.py --test
```

## ⚠️ Hardware Safety Checklist

Before working with the robot:

- [ ] RoArm power supply is **7.4–8.4 V** (NEVER 12 V)
- [ ] RoArm speed ≤ 0.25 in config
- [ ] CubeBot wheels are **off the ground** for initial testing
- [ ] Someone is ready to **pull the power plug** if anything goes wrong
- [ ] Shoulder temperature checked after each test session (panel shows it live)
- [ ] Shut the Pi down with `sudo poweroff` before pulling power

## 🔀 Git Workflow

1. Work on your track's files (see README for parallel tracks A–D)
2. Test with `selftest` before committing
3. Commit with clear messages: `[track] what changed`
4. Pull before pushing — coordinate in the team chat

## 📂 Where Things Go

| What | Where |
|------|-------|
| Arm + vision code | `Raspberry/` |
| Driving code | `Raspberry-Auto/` |
| Bottle inspection / deposit | `dorm_keeper/` |
| RoArm USB bridge | `Raspberry/roarm_usb.py` |
| Hardware docs | `Docs/` |
| Config values | `config.json` in each module |
| AprilTag images | `AprilTags/` |
| Deployment to Pi | `dorm_keeper/deploy_to_pi.py` |
