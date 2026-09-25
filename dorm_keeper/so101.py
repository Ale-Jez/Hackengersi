"""SO-Arm-101: sterownik + TWOJ PROGRAM (na dole pliku).

Uruchom:   py so101.py COM5
Edytujesz tylko funkcje moj_program() na samym dole.

Ten sam interfejs co roarm.RoArm: joints(), move(), grip(), release(), hold(), torque().
Katy w stopniach liczonych od srodka zakresu enkodera: (ticks - 2048) * 360 / 4096.
Nie wymaga kalibracji LeRobot - pozy uczymy i odtwarzamy w tych samych jednostkach.
"""

import math
import sys
import time

# Rejestry STS3215 (pamiec SRAM)
ADDR_TORQUE_ENABLE = 40
ADDR_ACC = 41
ADDR_GOAL_POSITION = 42  # 42-43 pozycja, 44-45 czas, 46-47 predkosc
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60  # 0-1000 (0.1 %), bit 10 = kierunek

TICKS_PER_DEG = 4096 / 360
MAX_TICK = 4095

# Standardowe ID serw SO-101 (po lerobot setup-motors)
SO101_IDS = {"pan": 1, "lift": 2, "elbow": 3, "wflex": 4, "wroll": 5, "grip": 6}


class SO101Error(Exception):
    pass


def _to_deg(ticks):
    return round((ticks - 2048) / TICKS_PER_DEG, 1)


def _to_ticks(deg):
    return max(0, min(MAX_TICK, int(round(deg * TICKS_PER_DEG + 2048))))


class SO101:
    JOINTS = list(SO101_IDS)
    ARM_JOINTS = JOINTS[:5]

    def __init__(self, port, baud=1_000_000, speed=30, acc=50, ids=None):
        from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

        self._ok = COMM_SUCCESS
        self.ids = dict(ids or SO101_IDS)
        self.speed = speed
        self.acc = acc
        # Brak domyslnych wartosci - zalezne od montazu. Ustaw przez "cal open/closed".
        self.gripper_open = None
        self.gripper_closed = None
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise SO101Error(f"nie moge otworzyc {port}")
        if not self.port.setBaudRate(baud):
            raise SO101Error(f"nie moge ustawic {baud} bps")
        self.ph = PacketHandler(0)  # 0 = STS/SMS (little-endian)
        missing = [name for name, sid in self.ids.items() if not self.ping(sid)]
        if missing:
            raise SO101Error(
                f"brak odpowiedzi serw: {', '.join(missing)} "
                f"(ID {[self.ids[m] for m in missing]}). Zasilanie? Kabel magistrali? Uzyj --scan."
            )

    def close(self):
        self.port.closePort()

    def ping(self, sid):
        _, res, _ = self.ph.ping(self.port, sid)
        return res == self._ok

    def scan(self, max_id=20):
        return [sid for sid in range(1, max_id + 1) if self.ping(sid)]

    def _read_pos(self, sid):
        for _ in range(3):
            val, res, _ = self.ph.read2ByteTxRx(self.port, sid, ADDR_PRESENT_POSITION)
            if res == self._ok:
                return -(val & 0x7FFF) if val & 0x8000 else val
        raise SO101Error(f"nie moge odczytac pozycji serwa ID {sid}")

    def _write_goal(self, sid, ticks, speed_ticks):
        # acc, pozycja L/H, czas L/H (0), predkosc L/H - jak WritePosEx z SDK Feetech
        data = [
            self.acc,
            ticks & 0xFF, (ticks >> 8) & 0xFF,
            0, 0,
            speed_ticks & 0xFF, (speed_ticks >> 8) & 0xFF,
        ]
        res, _ = self.ph.writeTxRx(self.port, sid, ADDR_ACC, len(data), data)
        if res != self._ok:
            raise SO101Error(f"nie moge wyslac celu do serwa ID {sid}")

    def joints(self):
        return {name: _to_deg(self._read_pos(sid)) for name, sid in self.ids.items()}

    def loads(self):
        """Obciazenie serw w % (znak = kierunek pchania)."""
        out = {}
        for name, sid in self.ids.items():
            val, res, _ = self.ph.read2ByteTxRx(self.port, sid, ADDR_PRESENT_LOAD)
            if res != self._ok:
                raise SO101Error(f"nie moge odczytac obciazenia serwa ID {sid}")
            load = (val & 0x3FF) / 10
            out[name] = -load if val & 0x400 else load
        return out

    def move(self, pose, speed=None, wait=True, tol=3.0, timeout=15.0):
        # Predkosc 0 = maksymalna dla STS3215 - nigdy tego nie wysylamy.
        if not getattr(self, "_torque_on", False):
            self.torque(True)
        spd = max(1, int((speed or self.speed) * TICKS_PER_DEG))
        for name, deg in pose.items():
            if name in self.ids:
                self._write_goal(self.ids[name], _to_ticks(deg), spd)
        if wait:
            self.wait_reached(pose, tol, timeout)
        current = self.joints()
        current.update({k: v for k, v in pose.items() if k in self.ids})
        return current

    def wait_reached(self, target, tol=2.0, timeout=15.0, settle_tol=10.0):
        """Czekaj az stawy dojada. Serwa SO-101 maja niskie P (16), wiec pod ciezarem
        staja kilka stopni przed celem - jesli staw sie zatrzymal blizej niz settle_tol,
        to uznajemy to za dojazd. Dalej = blokada/przeszkoda -> blad."""
        check = [j for j in self.ARM_JOINTS if j in target]
        deadline = time.time() + timeout
        last, still_since = None, None
        while time.time() < deadline:
            now = {j: _to_deg(self._read_pos(self.ids[j])) for j in check}
            err = max((abs(now[j] - target[j]) for j in check), default=0.0)
            if err <= tol:
                return True
            if last and all(abs(now[j] - last[j]) < 0.3 for j in check):
                still_since = still_since or time.time()
                if time.time() - still_since > 0.5:
                    if err <= settle_tol:
                        return True
                    self.hold()
                    raise SO101Error(f"staw stanal {err:.0f} st. od celu (przeszkoda?)")
            else:
                still_since = None
            last = now
            time.sleep(0.05)
        raise SO101Error(f"nie dojechal do pozy w {timeout}s (przeszkoda albo poza zasiegiem?)")

    def move_slow(self, delta=None, target=None, step=2.0, speed=10.0, max_load=60.0, verbose=True):
        """Bezpieczny ruch malymi krokami z kontrola obciazenia.

        delta={"lift": 20}      -> o 20 st. od obecnej pozycji
        target={"lift": -30}    -> do konkretnego kata
        Zatrzymuje sie (i trzyma pozycje), gdy obciazenie > max_load %% albo staw utknie.
        """
        start = self.joints()
        goal = dict(target or {})
        for j, d in (delta or {}).items():
            goal[j] = start[j] + d
        goal = {j: v for j, v in goal.items() if j in self.ids}
        if not goal:
            return start
        steps = max(1, math.ceil(max(abs(goal[j] - start[j]) for j in goal) / step))
        for i in range(1, steps + 1):
            waypoint = {j: start[j] + (goal[j] - start[j]) * i / steps for j in goal}
            self.move(waypoint, speed=speed, wait=False)
            try:
                self.wait_reached(waypoint, tol=1.0, timeout=step / speed + 3.0)
            except SO101Error:
                self.hold()
                raise
            now, loads = self.joints(), self.loads()
            if verbose:
                info = "  ".join(f"{j}={now[j]:6.1f} obc={loads[j]:5.1f}%" for j in goal)
                print(f"  [{i:2d}/{steps}] {info}")
            worst = max(abs(loads[j]) for j in self.ARM_JOINTS)
            if worst > max_load:
                self.hold()
                raise SO101Error(f"STOP: obciazenie {worst:.0f}% > {max_load:.0f}% - ramie trzyma pozycje")
        return self.joints()

    def _need_gripper_cal(self, value, which):
        if value is None:
            raise SO101Error(f"chwytak nieskalibrowany - ustaw go recznie i wpisz 'cal {which}'")
        return value

    def grip(self, angle=None, settle=0.6):
        angle = angle if angle is not None else self._need_gripper_cal(self.gripper_closed, "closed")
        self.move({"grip": angle}, wait=False)
        time.sleep(settle)

    def release(self, angle=None, settle=0.6):
        angle = angle if angle is not None else self._need_gripper_cal(self.gripper_open, "open")
        self.move({"grip": angle}, wait=False)
        time.sleep(settle)

    def hold(self):
        self.move(self.joints(), wait=False)

    def torque(self, on):
        for sid in self.ids.values():
            if on:
                # Cel = aktualna pozycja, inaczej serwo szarpnie do starego celu.
                self._write_goal(sid, max(0, self._read_pos(sid)), int(20 * TICKS_PER_DEG))
            self.ph.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, 1 if on else 0)
        self._torque_on = on


class MockSO101(SO101):
    """Udaje SO-101 bez sprzetu."""

    def __init__(self, speed=30, acc=50):
        self.ids = dict(SO101_IDS)
        self.speed = speed
        self.acc = acc
        self.gripper_open = None
        self.gripper_closed = None
        self._pose = {name: 0.0 for name in self.ids}

    def close(self):
        pass

    def joints(self):
        return dict(self._pose)

    def loads(self):
        return {name: 0.0 for name in self.ids}

    def move(self, pose, speed=None, wait=True, tol=3.0, timeout=15.0):
        for k, v in pose.items():
            if k in self._pose:
                self._pose[k] = float(v)
        print(f"  [mock] -> {pose} spd={speed or self.speed}")
        return dict(self._pose)

    def wait_reached(self, target, tol=2.0, timeout=15.0, settle_tol=10.0):
        return True

    def torque(self, on):
        print(f"  [mock] torque {'on' if on else 'off'}")

    def scan(self, max_id=20):
        return list(self.ids.values())


# =============================================================================
#  TWOJ PROGRAM - edytuj tylko to ponizej
# =============================================================================
#
#  Uruchom:  python so101.py      (albo przycisk Run w VS Code)
#  Program dziala CALY CZAS i czeka na sygnal do skanu:
#    Enter w terminalu                              -> skan (test bez RoArma)
#    HTTP:  http://<IP-tego-laptopa>:8765/skanuj    -> skan, odpowiedz JSON z kodem
#    HTTP:  http://<IP-tego-laptopa>:8765/status    -> co ramie teraz robi
#    k + Enter  -> dopisz ostatni kod do listy kaucji (kaucja.json)
#    q + Enter  -> koniec
#
#  Pozy uczysz w teach.py:  torque off -> ustaw ramie reka -> s skan1 -> ... -> torque on
#
#  Sciaga do wlasnych ruchow:
#    arm.move_slow(delta={"lift": 20})        o 20 st. od obecnej pozycji, powoli
#    arm.move_slow(target={"elbow": -45})     do konkretnego kata
#    arm.move(pozy["skan1"], speed=20)        do zapisanej pozy
#    arm.joints()                             gdzie jest teraz
#  Kierunki: elbow MINUS = rozprostowanie w gore.  Awaryjnie: Ctrl+C albo wyjmij zasilacz.

import json
import os
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------------- USTAWIENIA ----------------
PORT_HTTP = 8765
KAMERA = 1                    # 0 = kamera laptopa, 1 = zwykle kamera USB na nadgarstku
PLIK_POZ = "poses_so101.json"
POZA_CZEKANIA = "spoczynek"   # tu ramie czeka na sygnal i tu wraca po skanie
POZY_SKANU = ["skan1", "skan2", "skan3", "skan4"]   # objazd dookola butelki
PREDKOSC = 20                 # st./s przy objezdzie
PLIK_KAUCJI = "kaucja.json"   # lista kodow EAN objetych kaucja
FOLDER_ZDJEC = "skany"        # zdjecie z kazdej pozy bez kodu (do sprawdzania ostrosci); "" = nie zapisuj
# --------------------------------------------

_katalog = os.path.dirname(os.path.abspath(__file__))
_blokada = threading.Lock()
stan = {"stan": "start", "ostatni_kod": None}


def _sciezka(nazwa):
    return os.path.join(_katalog, nazwa)


def wczytaj_json(nazwa, domyslnie):
    try:
        with open(_sciezka(nazwa), encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return domyslnie


def bez_chwytaka(poza):
    return {j: v for j, v in poza.items() if j != "grip"}


def otworz_kamere():
    try:
        import cv2
    except ImportError:
        print("UWAGA: brak OpenCV - objazd bedzie dzialal, ale bez czytania kodow")
        return None
    kam = cv2.VideoCapture(KAMERA, cv2.CAP_DSHOW)
    if not kam.isOpened() or not kam.read()[0]:
        print(f"UWAGA: kamera {KAMERA} nie dziala - objazd bez czytania kodow (zmien KAMERA?)")
        return None
    print(f"Kamera {KAMERA} OK")
    return kam


def czytaj_kod(kamera, nazwa_pozy, klatki=8):
    """Kilka klatek z kamery -> pierwszy znaleziony kod kreskowy albo None."""
    if kamera is None:
        return None
    import cv2

    detektor = cv2.barcode.BarcodeDetector()
    klatka = None
    for _ in range(klatki):
        ok, klatka = kamera.read()
        if not ok:
            continue
        znalezione, kody, _typy, _pkt = detektor.detectAndDecodeWithType(klatka)
        kody = [k for k in (kody or []) if k]
        if znalezione and kody:
            return kody[0]
    if FOLDER_ZDJEC and klatka is not None:
        os.makedirs(_sciezka(FOLDER_ZDJEC), exist_ok=True)
        cv2.imwrite(_sciezka(os.path.join(FOLDER_ZDJEC, f"{nazwa_pozy}.jpg")), klatka)
    return None


def skanuj_butelke(arm, kamera):
    """Objazd butelki z kamera. Zwraca {"kod": ..., "kaucja": ..., "czas": ...}."""
    t0 = time.time()
    pozy = wczytaj_json(PLIK_POZ, {})
    brak = [p for p in POZY_SKANU + [POZA_CZEKANIA] if p not in pozy]
    if brak:
        raise SO101Error(f"brak poz {brak} - naucz je w teach.py (np. s skan1)")

    kod = None
    for nazwa in POZY_SKANU:
        stan["stan"] = f"skanuje: {nazwa}"
        print(f"  -> {nazwa}")
        arm.move(bez_chwytaka(pozy[nazwa]), speed=PREDKOSC)
        time.sleep(0.3)  # obraz sie uspokaja po ruchu
        kod = czytaj_kod(kamera, nazwa)
        if kod:
            print(f"  KOD: {kod}")
            break

    stan["stan"] = "wracam"
    arm.move(bez_chwytaka(pozy[POZA_CZEKANIA]), speed=PREDKOSC)

    kaucja = None
    if kod:
        kaucja = kod in wczytaj_json(PLIK_KAUCJI, [])
    stan["ostatni_kod"] = kod
    return {"kod": kod, "kaucja": kaucja, "czas": round(time.time() - t0, 1)}


def jeden_skan(arm, kamera):
    """Skan z blokada - tylko jeden naraz (Enter albo HTTP)."""
    if not _blokada.acquire(blocking=False):
        return {"blad": "zajety - skan juz trwa"}
    try:
        wynik = skanuj_butelke(arm, kamera)
    except SO101Error as e:
        wynik = {"blad": str(e)}
    finally:
        stan["stan"] = "czekam na sygnal"
        _blokada.release()
    print("WYNIK:", wynik)
    return wynik


def uruchom_serwer(arm, kamera):
    class Obsluga(BaseHTTPRequestHandler):
        def _odpowiedz(self, dane, kod=200):
            tresc = json.dumps(dane, ensure_ascii=False).encode()
            self.send_response(kod)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(tresc)

        def _obsluz(self):
            if self.path.startswith("/skanuj"):
                print("\nSYGNAL z sieci -> skanuje")
                wynik = jeden_skan(arm, kamera)
                self._odpowiedz(wynik, 409 if "zajety" in wynik.get("blad", "") else 200)
            elif self.path.startswith("/status"):
                self._odpowiedz(stan)
            else:
                self._odpowiedz({"blad": "uzyj /skanuj albo /status"}, 404)

        do_GET = _obsluz
        do_POST = _obsluz

        def log_message(self, *args):
            pass

    serwer = ThreadingHTTPServer(("0.0.0.0", PORT_HTTP), Obsluga)
    threading.Thread(target=serwer.serve_forever, daemon=True).start()
    return serwer


def moje_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def find_port():
    """Znajdz adapter serw SO-101 (chip CH343/CH340, VID 1A86)."""
    from serial.tools import list_ports

    ports = [p.device for p in list_ports.comports() if p.vid == 0x1A86]
    if not ports:
        raise SO101Error("nie widze adaptera serw (CH343) - podlaczony kabel USB?")
    return ports[0]


if __name__ == "__main__":
    mock = "--mock" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--mock"]
    arm = MockSO101() if mock else SO101(args[0] if args else find_port())
    kamera = otworz_kamere()
    serwer = uruchom_serwer(arm, kamera)

    pozy = wczytaj_json(PLIK_POZ, {})
    if POZA_CZEKANIA in pozy:
        print(f"Jade do pozy '{POZA_CZEKANIA}'...")
        arm.move_slow(target=bez_chwytaka(pozy[POZA_CZEKANIA]), verbose=False)
    stan["stan"] = "czekam na sygnal"
    print(f"\nGOTOWY. Sygnal od RoArma: http://{moje_ip()}:{PORT_HTTP}/skanuj")
    print("Enter = skan testowy,  k = dopisz ostatni kod do kaucji,  q = koniec\n")

    try:
        while True:
            cmd = input().strip().lower()
            if cmd == "q":
                break
            if cmd == "k":
                if stan["ostatni_kod"]:
                    lista = wczytaj_json(PLIK_KAUCJI, [])
                    if stan["ostatni_kod"] not in lista:
                        lista.append(stan["ostatni_kod"])
                        with open(_sciezka(PLIK_KAUCJI), "w", encoding="utf-8") as f:
                            json.dump(lista, f, indent=2)
                    print(f"dopisano {stan['ostatni_kod']} do {PLIK_KAUCJI}")
                else:
                    print("brak ostatniego kodu")
                continue
            print("Skan testowy...")
            jeden_skan(arm, kamera)
    except (KeyboardInterrupt, EOFError):
        print("\nPrzerwano.")
    finally:
        serwer.shutdown()
        try:
            arm.hold()
        except SO101Error:
            pass
        if kamera is not None:
            kamera.release()
        arm.close()
        print("Koniec - ramie trzyma pozycje.")


