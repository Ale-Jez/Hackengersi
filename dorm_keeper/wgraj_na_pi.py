"""Wgraj dorm_keeper na Raspberry (F5 w VS Code albo: python wgraj_na_pi.py).

Kod laduje w ~/dorm_keeper na Pi - osobno od repo kolegi (~/Hackengersi), zeby jego git pull nie mial konfliktow.
Pozy, lista kaucji i historia na Pi NIE sa nadpisywane (tam moga byc nowsze).

SO-101 z kamera dziala na Pi jako usluga "dorm-keeper" (ramie.py bez okna, panel http://malina.local:8765/).
YOLO na Pi daje ~10 kl/s - szybciej, gdy na laptopie dziala yolo_laptop.py (liczy YOLO za Pi).

    python wgraj_na_pi.py            wgraj; jesli usluga jest wlaczona - uruchom ja od nowa
    python wgraj_na_pi.py --autostart    wlacz usluge (tez po restarcie Pi) i uruchom
    python wgraj_na_pi.py --log      ... i pokaz log na zywo (Ctrl+C konczy podglad, program na Pi dziala dalej)
    python wgraj_na_pi.py --stop     wylacz usluge (np. SO-101 dla main.py kolegi albo na laptopie)

Pi pod inna nazwa/adresem:  PI=hackengersi@192.168.32.114 python wgraj_na_pi.py
"""
import io
import os
import socket
import subprocess
import sys
import tarfile

PI = os.environ.get("PI", "hackengersi@malina.local")
PYTHON_PI = "~/Hackengersi/.venv/bin/python"  # venv z opencv, onnxruntime, zxing-cpp, feetech-servo-sdk
KOD = ["ramie.py", "camera.py", "pizza.py", "teach.py", "roarm.py", "butelki.py", "pokaz.html",
       "../Raspberry/roarm_wifi.py", "../Raspberry/roarm_console.py"]  # RoArm przez WiFi (kod kolegi)
DANE = ["poses_so101.json", "kaucja.json", "tracking_history.json", "roarm_kaucja.json"]  # tylko gdy na Pi ich brak
MODEL = os.path.join("modele", "yolo11n.onnx")

USLUGA = """[Unit]
Description=Dorm-Keeper: SO-101 + kamera, panel http://%H.local:8765/
After=network-online.target

[Service]
WorkingDirectory=%h/dorm_keeper
Environment=OPENCV_LOG_LEVEL=ERROR
ExecStart={python} -u ramie.py --web
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
"""

_katalog = os.path.dirname(os.path.abspath(__file__))


def adres():
    """malina.local czasem sie nie rozwiazuje na Windows - wtedy uzyj ostatniego znanego IP."""
    user, _, host = PI.rpartition("@")
    try:
        socket.getaddrinfo(host, 22)
        return PI
    except OSError:
        print(f"{host} nie odpowiada po nazwie - probuje 192.168.32.114")
        return f"{user}@192.168.32.114"


def ssh(cel, polecenie, dane=None):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", cel, polecenie],
                          input=dane, check=False).returncode


def paczka(z_modelem):
    """tar w pamieci: kod/, dane/, modele/ + plik uslugi. Konce linii z Windows -> Linux."""
    bufor = io.BytesIO()
    with tarfile.open(fileobj=bufor, mode="w") as tar:
        def dodaj(nazwa, tresc):
            info = tarfile.TarInfo(nazwa)
            info.size = len(tresc)
            tar.addfile(info, io.BytesIO(tresc))

        for grupa, pliki in (("kod", KOD), ("dane", DANE)):
            for p in pliki:
                sciezka = os.path.join(_katalog, p)
                if os.path.exists(sciezka):
                    with open(sciezka, "rb") as f:
                        dodaj(f"{grupa}/{os.path.basename(p)}", f.read().replace(b"\r\n", b"\n"))
        if z_modelem:
            with open(os.path.join(_katalog, MODEL), "rb") as f:
                dodaj("modele/yolo11n.onnx", f.read())
        dodaj("dorm-keeper.service", USLUGA.format(python=PYTHON_PI.replace("~", "%h")).encode())
    return bufor.getvalue()


def main():
    cel = adres()
    if "--stop" in sys.argv:
        sys.exit(ssh(cel, "systemctl --user disable --now dorm-keeper 2>/dev/null; echo 'zatrzymany i wylaczony'"))

    brak_modelu = ssh(cel, "test -f ~/dorm_keeper/modele/yolo11n.onnx") != 0
    if brak_modelu and not os.path.exists(os.path.join(_katalog, MODEL)):
        brak_modelu = False  # nie ma czego wyslac - ramie.py na Pi sam pobierze model
    skrypt = """set -e
D=~/dorm_keeper; T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
tar -x -C "$T"
mkdir -p "$D/modele" ~/.config/systemd/user
cp "$T"/kod/* "$D"/
for f in "$T"/dane/*; do [ -e "$f" ] && { [ -e "$D/$(basename "$f")" ] || cp "$f" "$D"/; }; done
[ -d "$T/modele" ] && cp "$T"/modele/* "$D/modele/"
cp "$T/dorm-keeper.service" ~/.config/systemd/user/
systemctl --user daemon-reload
""" + ("""systemctl --user enable -q dorm-keeper; loginctl enable-linger 2>/dev/null || true
echo "autostart wlaczony"
""" if "--autostart" in sys.argv else "") + """if ! systemctl --user is-enabled -q dorm-keeper; then
  echo "wgrane do ~/dorm_keeper (usluga wylaczona - wlaczysz ja: python wgraj_na_pi.py --autostart)"
  exit 0
fi
START=$(date +%s)
systemctl --user restart dorm-keeper
for i in $(seq 40); do  # czekaj, az ramie, kamera i YOLO wystartuja
  sleep 1
  if journalctl --user-unit dorm-keeper --since "@$START" -o cat --no-pager | grep -q "^GOTOWY"; then
    echo "DZIALA: panel http://$(hostname).local:8765/  (albo http://$(hostname -I | cut -d' ' -f1):8765/)"
    exit 0
  fi
done
echo "NIE WYSTARTOWAL - log:"; journalctl --user-unit dorm-keeper --since "@$START" -o cat --no-pager | tail -30
exit 1
"""
    print(f"Wgrywam na {cel}{' (+ model YOLO 11 MB)' if brak_modelu else ''}...", flush=True)
    kod = ssh(cel, f"bash -c {sh_quote(skrypt)}", paczka(brak_modelu))
    if kod == 0 and "--log" in sys.argv:
        try:
            ssh(cel, "journalctl --user-unit dorm-keeper -f -n 20 --no-pager")
        except KeyboardInterrupt:
            pass
    sys.exit(kod)


def sh_quote(s):
    return "'" + s.replace("'", "'\"'\"'") + "'"


if __name__ == "__main__":
    main()
