"""Deploy dorm_keeper to the Raspberry Pi (F5 in VS Code or: python deploy_to_pi.py).

The code lands in ~/dorm_keeper on the Pi - separate from the team repo (~/Hackengersi), so its git pull has no
conflicts. Poses, the deposit list, history and calibration on the Pi are NOT overwritten (they may be newer there).

The SO-101 with the camera runs on the Pi as the "dorm-keeper" service (so101_station.py without a window,
panel http://malina.local:8765/roarm_panel, SO-101 page /so101). YOLO on the Pi gives ~10 fps - faster when yolo_laptop.py runs elsewhere.

    python deploy_to_pi.py              deploy; if the service is enabled - restart it
    python deploy_to_pi.py --autostart  enable the service (also after a Pi reboot) and start it
    python deploy_to_pi.py --log        ... and follow its log (Ctrl+C ends the view, the program keeps running)
    python deploy_to_pi.py --stop       disable the service (e.g. to use the SO-101 elsewhere)

Pi under another name/address:  PI=hackengersi@192.168.32.114 python deploy_to_pi.py
"""
import io
import os
import socket
import subprocess
import sys
import tarfile

PI = os.environ.get("PI", "hackengersi@malina.local")
PYTHON_PI = "~/Hackengersi/.venv/bin/python"  # venv with opencv, onnxruntime, zxing-cpp, feetech-servo-sdk
CODE = ["so101_station.py", "roarm_pick.py", "roarm_panel.py", "demo.html",
        "../Raspberry/roarm_wifi.py",  # RoArm HTTP client (WiFi or the USB bridge below)
        "../Raspberry/roarm_usb.py"]   # RoArm over USB: localhost:8766 imitates the RoArm web API (roarm-usb service)
DATA = ["poses_so101.json", "deposit_list.json", "tracking_history.json", "roarm_calibration.json"]  # only if missing
MODEL = os.path.join("models", "yolo11n.onnx")

SERVICE = """[Unit]
Description=Dorm-Keeper: SO-101 + camera, panel http://%H.local:8765/
After=network-online.target

[Service]
WorkingDirectory=%h/dorm_keeper
Environment=OPENCV_LOG_LEVEL=ERROR
ExecStart={python} -u so101_station.py --web
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
"""

SERVICE_USB = """[Unit]
Description=RoArm over USB: http://localhost:8766/js (same API as the RoArm over WiFi)

[Service]
WorkingDirectory=%h/dorm_keeper
ExecStart={python} -u roarm_usb.py
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
"""

_dir = os.path.dirname(os.path.abspath(__file__))


def address():
    """malina.local sometimes does not resolve on Windows - then use the last known IP."""
    user, _, host = PI.rpartition("@")
    try:
        socket.getaddrinfo(host, 22)
        return PI
    except OSError:
        print(f"{host} does not resolve - trying 192.168.32.114")
        return f"{user}@192.168.32.114"


def ssh(target, command, data=None):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", target, command],
                          input=data, check=False).returncode


def bundle(with_model):
    """In-memory tar: code/, data/, models/ + the service files. Windows line endings -> Linux."""
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tar:
        def add(name, content):
            info = tarfile.TarInfo(name)
            info.size = len(content)
            tar.addfile(info, io.BytesIO(content))

        for group, files in (("code", CODE), ("data", DATA)):
            for p in files:
                path = os.path.join(_dir, p)
                if os.path.exists(path):
                    with open(path, "rb") as f:
                        add(f"{group}/{os.path.basename(p)}", f.read().replace(b"\r\n", b"\n"))
        if with_model:
            with open(os.path.join(_dir, MODEL), "rb") as f:
                add("models/yolo11n.onnx", f.read())
        add("dorm-keeper.service", SERVICE.format(python=PYTHON_PI.replace("~", "%h")).encode())
        add("roarm-usb.service", SERVICE_USB.format(python=PYTHON_PI.replace("~", "%h")).encode())
    return buf.getvalue()


def main():
    target = address()
    if "--stop" in sys.argv:
        sys.exit(ssh(target, "systemctl --user disable --now dorm-keeper 2>/dev/null; echo 'stopped and disabled'"))

    no_model = ssh(target, "test -f ~/dorm_keeper/models/yolo11n.onnx") != 0
    if no_model and not os.path.exists(os.path.join(_dir, MODEL)):
        no_model = False  # nothing to send - so101_station.py on the Pi downloads the model itself
    script = """set -e
D=~/dorm_keeper; T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
tar -x -C "$T"
mkdir -p "$D/models" ~/.config/systemd/user
cp "$T"/code/* "$D"/
for f in "$T"/data/*; do [ -e "$f" ] && { [ -e "$D/$(basename "$f")" ] || cp "$f" "$D"/; }; done
[ -d "$T/models" ] && cp "$T"/models/* "$D/models/"
cp "$T/dorm-keeper.service" "$T/roarm-usb.service" ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable -q roarm-usb; loginctl enable-linger 2>/dev/null || true
# NEVER restart: opening the port on Linux raises DTR/RTS -> the ESP32 resets -> the RoArm drives to its firmware
# start pose by itself (that hit the vehicle on 2026-09-26). Only start it when not running; new roarm_usb.py:
# restart it by hand, with nothing near the arm.
systemctl --user start roarm-usb
""" + ("""systemctl --user enable -q dorm-keeper; loginctl enable-linger 2>/dev/null || true
echo "autostart enabled"
""" if "--autostart" in sys.argv else "") + """if ! systemctl --user is-enabled -q dorm-keeper; then
  echo "deployed to ~/dorm_keeper (service disabled - enable it: python deploy_to_pi.py --autostart)"
  exit 0
fi
START=$(date +%s)
systemctl --user restart dorm-keeper
for i in $(seq 40); do  # wait until the arm, camera and YOLO are up
  sleep 1
  if curl -sf -m 2 localhost:8765/status >/dev/null; then
    echo "RUNNING: panel http://$(hostname).local:8765/roarm_panel  (or http://$(hostname -I | cut -d' ' -f1):8765/roarm_panel)"
    exit 0
  fi
done
echo "DID NOT START - log:"; journalctl --user-unit dorm-keeper --since "@$START" -o cat --no-pager | tail -30
exit 1
"""
    print(f"Deploying to {target}{' (+ YOLO model 11 MB)' if no_model else ''}...", flush=True)
    code = ssh(target, f"bash -c {sh_quote(script)}", bundle(no_model))
    if code == 0 and "--log" in sys.argv:
        try:
            ssh(target, "journalctl --user-unit dorm-keeper -f -n 20 --no-pager")
        except KeyboardInterrupt:
            pass
    sys.exit(code)


def sh_quote(s):
    return "'" + s.replace("'", "'\"'\"'") + "'"


if __name__ == "__main__":
    main()
