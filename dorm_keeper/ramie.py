"""Ramie SO-101 - WSZYSTKO W JEDNYM PLIKU.

  python ramie.py                 (albo przycisk Run w VS Code) okno kamery + SLEDZENIE BUTELKI:
                             siec neuronowa wykrywa butelki/puszki, ramie trzyma najblizsza na srodku
                             obrazu, puszcza gdy sie oddali albo zgubi, przelacza na blizsza (T wl/wyl).
                             Kod kreskowy czytany przy okazji: zielona ramka = butelka kaucyjna.
  python ramie.py --bez-kamery    sterowanie z terminala + ogladanie puszki pod ENTER, bez kamery
  python ramie.py --mock          bez ramienia (sama kamera, ramie stoi)
  python ramie.py --web           bez okna: podglad, przyciski i suwaki w przegladarce http://<IP>:8765/
                                  (wlacza sie sam na Linuksie bez monitora, np. Raspberry przez SSH)
  (port adaptera znajduje sie sam; mozna podac np. COM5)

Klawisze (kliknij w okno kamery):
  W/S GORA/DOL     R/F DALEJ/BLIZEJ     A/D LEWO/PRAWO   (chwytak rusza sie jak hak dzwigu)
  J/L pochyl chwytak   U/O obroc chwytak   Z/X chwytak troche otworz/zamknij
  SPACJA chwyc/pusc (zamyka az poczuje opor)    1/2/3 predkosc    B stop
  ENTER  OGLADAJ PUSZKE: ramie objezdza puszke kamera, az zobaczy kod kreskowy
  P zapisz obecna poze jako skan1..skan4 (objazd)   C usun pozy skanu
  M zapisz poze domowa   H jedz do domu   Q/ESC koniec (ramie trzyma pozycje)
Pad Xbox: lewa galka lewo/prawo + gora/dol, prawa dalej/blizej + obrot, LB/RB pochylenie,
  A chwyc, B pusc, X stop, Y ogladaj puszke, BACK koniec.
Sygnal od RoArma: http://<IP-laptopa>:8765/skanuj  (odpowiedz JSON z kodem)
Inspekcja butelki (automat): SZUKAM -> CENTRUJE -> CZYTAM -> WYNIK. Wynik: http://<IP>:8765/wynik
  (JSON: wynik KAUCYJNA/BEZ_KAUCJI/BRAK_KODU, "obroc_butelke": true/false). Po obrocie: /obrocono
Awaryjnie: wyjmij wtyczke zasilacza serw.

Katy w stopniach od srodka zakresu enkodera: (ticks - 2048) * 360 / 4096.
"""

import ctypes
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

WINDOWS = sys.platform.startswith("win")
if not WINDOWS:
    # OpenCV z Qt na Linuksie (Wayland) potrzebuje X11, inaczej okno sie nie otworzy
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
# bez monitora (Raspberry przez SSH / jako usluga) okno OpenCV zabiloby program - wtedy panel w przegladarce
BEZ_OKNA = "--web" in sys.argv or (not WINDOWS and not os.environ.get("DISPLAY")
                                   and not os.environ.get("WAYLAND_DISPLAY"))

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
        if sid in {self.ids[n] for n in ZABLOKOWANE_STAWY if n in self.ids}:
            return  # staw zablokowany (np. kamera przykrecona na chwytaku) - nigdy go nie ruszamy
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
        for name, sid in self.ids.items():
            if name in ZABLOKOWANE_STAWY:  # bez momentu = serwo nie pcha, nie moze sie spalic
                self.ph.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, 0)
                continue
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


def _mock_write_goal(self, sid, ticks, speed_ticks):
    name = next(n for n, i in self.ids.items() if i == sid)
    if name in ZABLOKOWANE_STAWY:
        return
    self._pose[name] = _to_deg(ticks)


MockSO101._write_goal = _mock_write_goal


# =============================================================================
#  USTAWIENIA - to mozesz zmieniac
# =============================================================================
SPEEDS = [10.0, 25.0, 50.0]      # st./s dla predkosci 1 / 2 / 3 (obroty: podstawa, nadgarstek)
SPEEDS_MM = [20.0, 50.0, 100.0]  # mm/s dla predkosci 1 / 2 / 3 (gora/dol, dalej/blizej)
RAMIE_L1 = 116.0                 # mm: bark -> lokiec (SO-101)
RAMIE_L2 = 135.0                 # mm: lokiec -> nadgarstek (SO-101)
TRZYMAJ_KAT_CHWYTAKA = True      # przy gora/dol/dalej/blizej chwytak (kamera) nie zmienia pochylenia
ZNAK_KOMPENSACJI = 1             # odwroc na -1, jesli chwytak pochyla sie zamiast trzymac kat
KEY_STEP_TIME = 1 / 30           # trzymany klawisz powtarza sie ~30x/s
MAX_LEAD = 12.0                  # cel moze wyprzedzac prawdziwe ramie max o tyle st.
LIMIT_MARGIN = 3.0               # odstep od limitow katowych serw (st.)
CLAMP_SPEED = 30.0               # st./s przy zamykaniu na przedmiocie
CLAMP_LOAD = 20.0                # % obciazenia = "dotknal przedmiotu"
CLAMP_SQUEEZE = 4.0              # st. dodatkowego docisku po dotknieciu
RELEASE_OPEN = 35.0              # o ile st. otwiera sie chwytak przy puszczeniu
ZABLOKOWANE_STAWY = {"grip"}     # te stawy: moment wylaczony i zero ruchu (kamera przykrecona na chwytaku!)
BOOST_P = False                  # P=32 na lift/elbow do wylaczenia zasilania (z 16 opadaja)

KAMERA = None                    # None = sama wybierze (zewnetrzna, a jak nie ma - laptopa)
POZY_SKANU = ["skan1", "skan2", "skan3", "skan4"]
AUTO_RUCHY = [                   # rozgladanie wokol pozycji startowej, gdy brak poz skan1..4
    {"pan": -20}, {"pan": -10}, {"pan": 0}, {"pan": 10}, {"pan": 20},
    {"pan": 0, "wflex": -15}, {"pan": 0, "wflex": 15},
]
PAUZA = 0.8                      # ile s patrzy w kazdej pozycji
PREDKOSC_SKANU = 25              # st./s
# --- inspekcja butelki: SZUKAM -> CENTRUJE -> CZYTAM -> WYNIK ---
ROZGLADANIE = False              # False = kamera czeka w pozycji wyczekiwania (butelke przynosi drugie ramie)
POWROT_PO = 3.0                  # s bez butelki -> kamera wraca spokojnie do pozycji wyczekiwania
SZUKAJ_PO = 2.0                  # s bez butelki -> kamera zaczyna sie rozgladac (tylko gdy ROZGLADANIE = True)
SZUKAJ_ZAKRES_POZIOM = 30.0      # st.: rozgladanie w lewo/prawo od pozycji startowej
SZUKAJ_ZAKRES_PION = 15.0        # st.: rozgladanie w gore/dol od pozycji startowej
SZUKAJ_PREDKOSC = 12.0           # st./s: jak szybko sie rozglada (wolno = kamera widzi ostro)
WYCENTROWANY_CZAS = 0.3          # s spokojnie na srodku -> zaczyna czytac kod
KOD_CZAS = 4.0                   # s czytania kodu -> jak nic, to "BRAK KODU - obroc butelke"
DECYZJA_URL = ""                 # np. "http://192.168.1.30:8000/decyzja" - wynik wysylany tam automatycznie (POST JSON)
PORT_HTTP = 8765                 # sygnal od RoArma: http://<IP>:8765/skanuj
WEB_FPS = 20                     # najwyzej tyle klatek/s podgladu w przegladarce (reszte ogranicza WiFi)
WEB_SZER = 640                   # szerokosc klatek podgladu: mniejsze = wiecej klatek/s przez slabe WiFi
POZA_DOMOWA = "spoczynek"
PLIK_POZ = "poses_so101.json"
PLIK_KAUCJI = "kaucja.json"      # wlasna lista EAN z kaucja (oprocz api.kaucja.pl)

# --- sledzenie butelki kaucyjnej (tryb --kamera, klawisz T wlacza/wylacza) ---
SLEDZ = True                     # startuje wlaczone
# PLYNNE sledzenie: predkosc ramienia proporcjonalna do odleglosci celu od srodka obrazu.
SLEDZ_PLYNNIE = True             # False = stary tryb "popraw i poczekaj" (ruch - pauza - ruch)
SLEDZ_PETLA = 1.2                # szybkosc reakcji (1/s): wiecej = szybciej dogania, za duzo = przestrzeliwuje
                                 # (bylo 2.2 przy czulosci zawyzonej 3x = w praktyce 0.7; teraz czulosc prawdziwa)
SLEDZ_MAX_V = 30.0               # st./s: najszybszy ruch przy sledzeniu (szybciej = rozmazany obraz, kod nieczytelny)
SLEDZ_FILTR = 0.4                # wygladzanie pozycji celu (0..1, mniej = gladziej, ale wolniej reaguje)
CZULOSC_ZAKRES = (0.6, 2.0)      # nauka czulosci tylko w tym zakresie x wartosc startowa (zmierzona) - mniejsza =
                                 # za mocna reakcja = machanie, wieksza = ramie leniwie dogania butelke
SLEDZ_UCZ_CZULOSC = True         # mierz na biezaco, o ile przesuwa sie obraz na 1 st. ruchu (zalezy od odleglosci)
SLEDZ_START_PLYNNIE = 0.10       # stojace ramie rusza, gdy cel odjedzie dalej niz to (drgania wykrycia = bez ruchu)
SLEDZ_HAMOWANIE = 0.25           # s: jak szybko wyhamowuje po chwilowej utracie celu
SLEDZ_FF = 0.5                   # przewidywanie: jedz z predkoscia butelki (0 = wylaczone, 1 = pelne)
SLEDZ_OPOZNIENIE = 0.3           # s: opoznienie kamery (obraz pokazuje ramie sprzed tylu sekund) - gdy nie znamy wieku klatki
KAMERA_OPOZNIENIE = 0.17         # s: od polecenia dla serwa do klatki, ktora ten ruch pokazuje; zmierzone na SO-101 +
                                 # tej kamerze: serwo rusza po ~140 ms, obraz pokazuje ruch ~45 ms pozniej.
                                 # Program dolicza do tego wiek klatki (czas YOLO itd.)
SLEDZ_FF_PROG = 1.0              # st./s: wolniejszy ruch butelki = szum, ignoruj
SLEDZ_FF_GLADKOSC = 0.3          # 0..1: jak szybko przewidywanie reaguje na zmiane ruchu butelki
# Tryb "popraw i poczekaj" (SLEDZ_PLYNNIE = False): jeden ruch w strone celu, pauza, nowa klatka, kolejny ruch.
SLEDZ_KROK = 0.5                 # jaka czesc odleglosci do srodka pokonuje jednym ruchem (0.3 spokojnie, 0.8 szybko)
SLEDZ_CZULOSC_POZIOM = 0.020     # start: o ile przesuwa sie obraz (czesc polowy szerokosci) na 1 st. podstawy -
                                 # zmierzone: 3.2 px/st. przy 320 px (potem program sam sie douczy - zalezy od odleglosci)
SLEDZ_CZULOSC_PION = 0.030       # start: to samo dla nadgarstka (zmierzone: 2.7 px/st. przy 180 px wysokosci)
SLEDZ_CEL = 0.04                 # tak blisko srodka = wycentrowany, ramie staje
SLEDZ_START = 0.16               # wycentrowany rusza sie dopiero, gdy kod ucieknie dalej niz to (bez drgan)
SLEDZ_MIN_KROK = 0.5             # mniejszych ruchow nie robi (st.)
SLEDZ_MAX_KROK = 8.0             # najwiekszy pojedynczy ruch (st.)
SLEDZ_PAUZA = 0.7                # s po ruchu, zanim spojrzy znowu (opoznienie kamery + ramie przestaje sie bujac)
SLEDZ_PREDKOSC = 15              # st./s ruchow sledzenia - wolno = kamera sie nie buja
SLEDZ_ACC = 15                   # przyspieszenie ruchow sledzenia (mniej = lagodniejszy start i hamowanie)
SLEDZ_WYPRZEDZENIE = 0.3         # s: serwo dostaje cel tak daleko przed soba -> jedzie ciagle, nie staje co klatke
SLEDZ_CEL_W_KOD = False          # True = po odczycie celuj w sam kod (nizej; gora butelki moze wyjsc z kadru)
SLEDZ_CEL_WYSOKOSC = 0.45        # gdzie celowac w butelke: 0 = gora, 0.5 = srodek, 1 = dno (etykieta z kaucja)
SLEDZ_ASPEKT = {"butelka": 2.8, "puszka": 1.8}  # wysokosc/szerokosc - do szacowania ucietej butelki
UCIETA_MAX = 1.3                 # ucieta butelka: zgadywana wielkosc najwyzej tyle razy widoczna czesc
UCIETA_MAX_V = 12.0              # st./s: gdy butelka wystaje poza kadr, cel to tylko szacunek - jedz ostrozniej
BLISKA_OD = 0.72                 # butelka wyzsza niz 72% kadru = tuz przed kamera: w pionie nie celuj (etykieta widac)
SLEDZ_PRZYSP = 70.0              # st./s^2: predkosc sledzenia zmienia sie najwyzej tak szybko (plynny start i hamowanie)
SLEDZ_PROBKI = 3                 # z ilu klatek mediana pozycji kodu
# Wykrywanie butelek siecia neuronowa. YOLO11n (onnxruntime) widzi tez butelke czesciowo poza kadrem;
# SSD MobileNet v2 (samo OpenCV) - zapas, gdy nie ma onnxruntime.
DETEKTOR = "yolo"                # "yolo" albo "ssd"
YOLO_ROZMIARY = (320, 640)       # YOLO w dwoch skalach: 320 widzi bliskie butelki, 640 dalsze
YOLO_PRZEPLOT = True             # co klatke inna skala (wyniki drugiej z poprzedniej klatki) = 2x szybciej
YOLO_SLEDZENIE = 320             # sledzona butelka duza w kadrze: tylko ta skala, swieza w kazdej klatce (Pi: 32 ms)
YOLO_DUZA_OD = 0.22              # "duza" = wyzsza niz 22% obrazu (mniejsza: obie skale, bo w 320 by znikala)
YOLO_KLASY = {39: "butelka", 41: "puszka"}   # klasy COCO w YOLO: 39 butelka, 41 kubek (tak widzi puszki)
SLEDZ_KLASY = {44: "butelka", 47: "puszka"}  # to samo w numeracji SSD: 44 butelka, 47 kubek
BUTELKA_PROG = 0.25              # pewnosc (0..1), zeby ZLAPAC nowa butelke
PUSZKA_PROG = 0.50               # to samo dla "puszki" - wyzej, bo siec tak nazywa tez kubki
BUTELKA_PROG_TRZYMAJ = 0.15      # pewnosc, zeby TRZYMAC juz sledzona (chwilowy spadek pewnosci nie gubi celu)
MIN_BUTELKA = 0.12               # butelka nizsza niz 12% wysokosci obrazu = za daleko -> puszcza
SLEDZ_POTWIERDZ = 3              # w ilu kolejnych klatkach musi byc widac butelke, zeby ja zlapac
SLEDZ_PRZELACZ_KLATEK = 8        # ... a inna butelka musi byc wyraznie blizej przez tyle klatek (~0.4 s), zeby przelaczyc
SLEDZ_TYLKO_KAUCJA = False       # True = sledz tylko butelki z odczytanym kodem kaucyjnym
CZYTNIK_NA_SEKUNDE = 5           # ile razy na sekunde czytac kod (wiecej = szybciej, ale obciaza procesor)
KOD_POTWIERDZ = 2                # tyle zgodnych odczytow EAN, zeby go przyjac (1 bledny odczyt nie psuje wyniku)
KOD_BRAK_PO = 3.0                # s bez odczytu kodu sledzonej butelki -> "obroc butelke kodem do kamery"
SLEDZ_SZUKAJ_PO = 0.5            # s: szukaj dopiero po tylu sekundach bez kodu (pojedyncze zgubione klatki ignoruj)
SLEDZ_SZUKAJ_CZAS = 1.5          # s: po utracie kodu jedzie dalej w strone, w ktora kod uciekal
SLEDZ_SZUKAJ_MAX = 8.0          # st.: najdalej tyle "na slepo" po utracie kodu
SERWO_ACC = 20                   # przyspieszenie serw (mniej = lagodniej; bylo 50)
SERWO_MARTWA_STREFA = 1          # kroki enkodera (1 = fabrycznie); wiecej = wolne ruchy ida skokami
SLEDZ_LOG = "sledzenie.log"      # zapis przebiegu (do diagnozy); "" = bez zapisu
SLEDZ_LOG_CO = float(os.environ.get("DK_LOG_CO", "0.3"))  # s miedzy wpisami sterowania (DK_LOG_CO=0 = co klatke)
SLEDZ_ZNAK_POZIOM = -1           # odwroc na -1, jesli ramie UCIEKA od kodu w poziomie
SLEDZ_ZNAK_PION = -1             # odwroc na -1, jesli ucieka w pionie
SLEDZ_STAW_POZIOM = "pan"        # ktorym stawem celowac w poziomie
SLEDZ_STAW_PION = "wflex"        # ktorym stawem celowac w pionie
MIN_SZEROKOSC = 0.08             # kod wezszy niz 8% obrazu = za daleko -> puszcza cel
PRZELACZ_GDY = 1.3               # inna butelka kaucyjna 1.3x wiekszy kod (blizej) -> przelacz
ZGUBIONY_PO = 2.0                # s bez kodu -> cel zgubiony
SLEDZ_PROMIEN = 0.25             # nieodczytany (rozmazany) kod blizej niz 25% obrazu od ostatniej pozycji = ten sam
KAMERA_ROZDZIELCZOSC = (2560, 1440)  # ta kamera daje wtedy 1920x1080 przy 30 kl/s = ostry kod kreskowy
#                                      (zadanie wprost 1920x1080 = tylko 5 kl/s, 1280x720 = 10 kl/s)
KAMERA_EKSPOZYCJA = None         # None = auto (najjasniej). Stala: -4 jasno ale rozmywa, -5/-6 mniej rozmycia, trzeba lampki
PO_UTRACIE = "stoj"              # "stoj" albo "dom" (wroc do pozy domowej)

# Odwroc 1 / -1, jesli klawisz rusza w zla strone.
DIRECTION = {"pan": 1, "lift": -1, "elbow": -1, "wflex": 1, "wroll": 1}
# =============================================================================

KEYS = {  # klawisz -> (staw, znak) - obroty
    "a": ("pan", 1), "d": ("pan", -1),
    "j": ("wflex", 1), "l": ("wflex", -1),
    "u": ("wroll", 1), "o": ("wroll", -1),
}
KEYS_DZWIG = {  # klawisz -> (dalej/blizej, gora/dol) - chwytak porusza sie jak hak dzwigu
    "w": (0, 1), "s": (0, -1),
    "r": (1, 0), "f": (-1, 0),
}
MOVE_JOINTS = ["pan", "lift", "elbow", "wflex", "wroll"]

POMOC = (
    "W/S gora/dol | R/F dalej/blizej | A/D lewo/prawo | J/L pochyl | U/O obroc | SPACJA chwyc | 1 2 3 predkosc | B stop\n"
    "ENTER ogladaj puszke | T sledzenie | P zapisz poze objazdu | C usun pozy objazdu | M zapisz dom | H do domu | Q koniec"
)


def fk(lift, elbow):
    """Kinematyka ramienia w jego plaszczyznie: (dalej mm, wysokosc mm, kat przedramienia st.).
    Katy jak w programie: lift 0 = ramie pionowo (+ do tylu), elbow -90 = przedramie prosto w gore."""
    a = math.radians(-lift)
    b = a + math.radians(elbow + 90)
    return (RAMIE_L1 * math.sin(a) + RAMIE_L2 * math.sin(b),
            RAMIE_L1 * math.cos(a) + RAMIE_L2 * math.cos(b), math.degrees(b))


def ik(x, z):
    """Odwrotnie: (lift, elbow) dla nadgarstka w punkcie (x, z), lokiec zgiety do przodu. None = poza zasiegiem."""
    c = (x * x + z * z - RAMIE_L1 ** 2 - RAMIE_L2 ** 2) / (2 * RAMIE_L1 * RAMIE_L2)
    if not -1.0 <= c <= 0.999:  # za daleko albo ramie prawie proste (tam kinematyka wariuje)
        return None
    phi = math.acos(c)
    a = math.atan2(x, z) - math.atan2(RAMIE_L2 * math.sin(phi), RAMIE_L1 + RAMIE_L2 * math.cos(phi))
    a = (a + math.pi) % (2 * math.pi) - math.pi  # do zakresu -180..180 (inaczej 281 zamiast 79 st.)
    return -math.degrees(a), math.degrees(phi) - 90

_katalog = os.path.dirname(os.path.abspath(__file__))


def wczytaj(nazwa, domyslnie):
    try:
        with open(os.path.join(_katalog, nazwa), encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return domyslnie


def zapisz(nazwa, dane):
    with open(os.path.join(_katalog, nazwa), "w", encoding="utf-8") as f:
        json.dump(dane, f, indent=2, ensure_ascii=False)


def find_port():
    """Znajdz adapter serw SO-101 (chip CH343/CH340, VID 1A86)."""
    from serial.tools import list_ports

    ports = [p.device for p in list_ports.comports() if p.vid == 0x1A86]
    if not ports:
        raise SO101Error("nie widze adaptera serw (CH343) - podlaczony kabel USB?")
    return ports[0]


def polacz(port=None, mock=False):
    return MockSO101() if mock else SO101(port or find_port())


def moje_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


# ----------------------------------------------------------------------------- pad
class Gamepad:
    """Pad Xbox / XInput przez ctypes - bez dodatkowych bibliotek."""

    class _State(ctypes.Structure):
        _fields_ = [("packet", ctypes.c_uint32), ("buttons", ctypes.c_uint16),
                    ("lt", ctypes.c_uint8), ("rt", ctypes.c_uint8),
                    ("lx", ctypes.c_int16), ("ly", ctypes.c_int16),
                    ("rx", ctypes.c_int16), ("ry", ctypes.c_int16)]

    BTN = {"up": 0x0001, "down": 0x0002, "start": 0x0010, "back": 0x0020, "lb": 0x0100,
           "rb": 0x0200, "a": 0x1000, "b": 0x2000, "x": 0x4000, "y": 0x8000}

    def __init__(self):
        self.dll = None
        for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                self.dll = getattr(ctypes.windll, name)
                break
            except (OSError, AttributeError):
                continue
        self.prev = 0

    def read(self):
        if self.dll is None:
            return None
        st = self._State()
        if self.dll.XInputGetState(0, ctypes.byref(st)) != 0:
            return None

        def axis(v):
            v = max(-1.0, v / 32767)
            return 0.0 if abs(v) < 0.2 else (v - 0.2 * (1 if v > 0 else -1)) / 0.8

        held = {n for n, m in self.BTN.items() if st.buttons & m}
        pressed = {n for n, m in self.BTN.items() if st.buttons & m and not self.prev & m}
        self.prev = st.buttons
        return {"lx": axis(st.lx), "ly": axis(st.ly), "rx": axis(st.rx), "ry": axis(st.ry)}, pressed, held


# ----------------------------------------------------------------------------- sterowanie reczne
class Sterownik:
    """Reczne sterowanie (klawisze + pad) z bezpiecznikami. Wspolne dla obu trybow."""

    def __init__(self, arm):
        self.arm = arm
        self.mock = isinstance(arm, MockSO101)
        self.limits = self._limits()
        if BOOST_P and not self.mock:
            for j in ("lift", "elbow"):
                sid = arm.ids[j]
                if arm.ph.read1ByteTxRx(arm.port, sid, 55)[0] != 1:
                    arm.ph.write1ByteTxRx(arm.port, sid, 55, 1)  # EEPROM zablokowany -> do wyl. zasilania
                arm.ph.write1ByteTxRx(arm.port, sid, 21, 32)
        arm.acc = SERWO_ACC  # lagodne rozpedzanie i hamowanie = bez szarpniec
        if not self.mock:
            for j in arm.ARM_JOINTS:  # martwa strefa (do wylaczenia zasilania - EEPROM zablokowany)
                sid = arm.ids[j]
                if arm.ph.read1ByteTxRx(arm.port, sid, 55)[0] != 1:
                    arm.ph.write1ByteTxRx(arm.port, sid, 55, 1)
                arm.ph.write1ByteTxRx(arm.port, sid, 26, SERWO_MARTWA_STREFA)
                arm.ph.write1ByteTxRx(arm.port, sid, 27, SERWO_MARTWA_STREFA)
        self.wolne = set()  # stawy, ktore sledzenie chce ruszyc wolno i lagodnie
        self.w_sled = {}    # predkosc sledzenia (st./s, ze znakiem) - serwo jedzie ciagle zamiast skokami
        arm.torque(True)
        self.pad = Gamepad()
        self.pad_ok = self.pad.read() is not None
        self.level = 1
        self.clamp = "open"  # open | closing | holding
        self.komunikat = ""
        self.here = arm.joints()
        self.target = dict(self.here)
        self.sent = dict(self.here)
        self._t = time.time()

    def _limits(self):
        if self.mock:
            return {j: (-150.0, 150.0) for j in self.arm.JOINTS}
        lim = {}
        for j in self.arm.JOINTS:
            sid = self.arm.ids[j]
            lo = self.arm.ph.read2ByteTxRx(self.arm.port, sid, 9)[0]
            hi = self.arm.ph.read2ByteTxRx(self.arm.port, sid, 11)[0]
            lim[j] = (_to_deg(lo) + LIMIT_MARGIN, _to_deg(hi) - LIMIT_MARGIN)
        return lim

    def po_zadaniu(self):
        """Po automatycznym ruchu: przejmij pozycje stawow, chwytak trzyma dalej."""
        self.here = self.arm.joints()
        for j in MOVE_JOINTS:
            self.target[j] = self.sent[j] = self.here[j]
        self._t = time.time()

    def _grip(self, deg):
        lo, hi = self.limits["grip"]
        self.target["grip"] = min(max(deg, lo), hi)

    def przesun(self, dalej_mm, gora_mm):
        """Przesun chwytak w gore/dol i dalej/blizej (lift + elbow razem), trzymajac kat chwytaka."""
        x, z, beta = fk(self.target["lift"], self.target["elbow"])
        wynik = ik(x + dalej_mm, z + gora_mm)
        if wynik is None:
            if not self.komunikat:
                self.komunikat = "poza zasiegiem ramienia"
            return
        lift, elbow = wynik
        if not (self.limits["lift"][0] <= lift <= self.limits["lift"][1]
                and self.limits["elbow"][0] <= elbow <= self.limits["elbow"][1]):
            if not self.komunikat:
                self.komunikat = "koniec zakresu ramienia - rusz sie w inna strone"
            return
        # przy zlozonym ramieniu maly ruch chwytaka = ogromny ruch stawow -> blokuj zamiast szarpac
        skok = max(abs(lift - self.target["lift"]), abs(elbow - self.target["elbow"]))
        if skok > 3.0 * max(1.0, (abs(dalej_mm) + abs(gora_mm)) / 2.0):
            if not self.komunikat:
                self.komunikat = "ramie za bardzo zlozone - najpierw R (dalej), zeby je rozlozyc"
            return
        self.target["lift"], self.target["elbow"] = lift, elbow
        if TRZYMAJ_KAT_CHWYTAKA:
            self.target["wflex"] -= ZNAK_KOMPENSACJI * (fk(lift, elbow)[2] - beta)

    def klawisz(self, k):
        """Obsluga klawisza ruchu. Zwraca True, jesli klawisz byl ruchowy."""
        v = SPEEDS[self.level]
        if k in KEYS_DZWIG:
            dalej, gora = KEYS_DZWIG[k]
            krok = SPEEDS_MM[self.level] * KEY_STEP_TIME
            self.przesun(dalej * krok, gora * krok)
        elif k in KEYS:
            joint, sign = KEYS[k]
            self.target[joint] += sign * DIRECTION[joint] * v * KEY_STEP_TIME
        elif k in ("1", "2", "3"):
            self.level = int(k) - 1
        elif k in ("z", "x", " ") and "grip" in ZABLOKOWANE_STAWY:
            self.komunikat = "chwytak ZABLOKOWANY (kamera) - zmien ZABLOKOWANE_STAWY w ramie.py"
        elif k == "z":
            self.clamp = "open"
            self._grip(self.target["grip"] + 3)
        elif k == "x":
            self.clamp = "open"
            self._grip(self.target["grip"] - 3)
        elif k == " ":
            self.clamp = "closing" if self.clamp == "open" else "release"
        elif k == "b":
            self.target = dict(self.here)
            if self.clamp == "closing":
                self.clamp = "open"
        else:
            return False
        return True

    def krok(self):
        """Jeden obieg: pad, chwytak, bezpieczniki, wyslanie celow. Zwraca nowo wcisniete przyciski pada."""
        now_t = time.time()
        dt = min(now_t - self._t, 0.1)
        self._t = now_t
        v = SPEEDS[self.level]
        self.here = self.arm.joints()
        pressed = set()

        state = self.pad.read()
        if state:
            sticks, pressed, held = state
            self.target["pan"] += -sticks["lx"] * DIRECTION["pan"] * v * dt
            vmm = SPEEDS_MM[self.level] * dt
            if sticks["ly"] or sticks["ry"]:  # lewa galka gora/dol, prawa dalej/blizej
                self.przesun(sticks["ry"] * vmm, sticks["ly"] * vmm)
            self.target["wroll"] += sticks["rx"] * DIRECTION["wroll"] * v * dt
            if "lb" in held:
                self.target["wflex"] -= DIRECTION["wflex"] * v * dt
            if "rb" in held:
                self.target["wflex"] += DIRECTION["wflex"] * v * dt
            if "up" in pressed:
                self.level = min(self.level + 1, 2)
            if "down" in pressed:
                self.level = max(self.level - 1, 0)
            if "grip" not in ZABLOKOWANE_STAWY:
                if "a" in pressed and self.clamp == "open":
                    self.clamp = "closing"
                if "b" in pressed:
                    self.clamp = "release"
            if "x" in pressed:
                self.target = dict(self.here)

        if self.clamp == "closing":
            load = self.arm.loads()["grip"]
            if abs(load) > CLAMP_LOAD or self.here["grip"] <= self.limits["grip"][0] + 1:
                self._grip(self.here["grip"] - CLAMP_SQUEEZE)
                self.clamp = "holding"
                self.komunikat = f"CHWYCONE (opor {abs(load):.0f}%)"
                print(f"\n{self.komunikat}")
            else:
                self._grip(min(self.target["grip"], self.here["grip"] + 2) - CLAMP_SPEED * dt)
        elif self.clamp == "release":
            self._grip(self.here["grip"] + RELEASE_OPEN)
            self.clamp = "open"
            self.komunikat = "PUSZCZONE"
            print(f"\n{self.komunikat}")

        for j in MOVE_JOINTS:
            lo, hi = self.limits[j]
            self.target[j] = min(max(self.target[j], lo, self.here[j] - MAX_LEAD), hi, self.here[j] + MAX_LEAD)

        spd = int(max(v, CLAMP_SPEED) * 2 * TICKS_PER_DEG)
        for j in self.arm.JOINTS:
            if j in self.w_sled:
                # plynne sledzenie: cel wysuniety o SLEDZ_WYPRZEDZENIE sekund ruchu do przodu, predkosc = predkosc
                # sledzenia -> serwo jedzie ciagle i nie zdazy zahamowac przed kolejna klatka
                w = self.w_sled[j]
                lo, hi = self.limits[j]
                cel = min(max(self.target[j] + w * SLEDZ_WYPRZEDZENIE, lo), hi)
                self.arm.acc = SLEDZ_ACC
                self.arm._write_goal(self.arm.ids[j], _to_ticks(cel), max(1, int(max(abs(w), 3.0) * TICKS_PER_DEG)))
                self.arm.acc = SERWO_ACC
                self.sent[j] = self.target[j]
                continue
            prog = 0.05 if j in self.wolne else 0.2  # sledzenie krokowe: drobne kroki
            if abs(self.target[j] - self.sent.get(j, 1e9)) > prog:
                if j in self.wolne:  # ruch sledzenia krokowego: lagodnie
                    self.arm.acc = SLEDZ_ACC
                    self.arm._write_goal(self.arm.ids[j], _to_ticks(self.target[j]), max(1, int(SLEDZ_PREDKOSC * TICKS_PER_DEG)))
                    self.arm.acc = SERWO_ACC
                else:
                    self.arm._write_goal(self.arm.ids[j], _to_ticks(self.target[j]), spd)
                self.sent[j] = self.target[j]
        self.wolne.clear()
        self.w_sled.clear()
        return pressed

    def opis(self):
        pos = "  ".join(f"{j}={self.here[j]:6.1f}" for j in self.arm.JOINTS)
        return f"[predkosc {self.level + 1}] [{self.clamp:7s}] {pos}"


def do_domu(arm):
    pozy = wczytaj(PLIK_POZ, {})
    if POZA_DOMOWA not in pozy:
        raise SO101Error(f"brak pozy '{POZA_DOMOWA}' - ustaw ramie i nacisnij M")
    arm.move_slow(target={j: pozy[POZA_DOMOWA][j] for j in MOVE_JOINTS}, verbose=False)


def zapisz_poze(arm, nazwa):
    pozy = wczytaj(PLIK_POZ, {})
    pozy[nazwa] = arm.joints()
    zapisz(PLIK_POZ, pozy)
    return f"zapisano poze '{nazwa}'"


_terminal = {"stary": None}


def _linux_terminal_surowy():
    """Linux/macOS: terminal bez czekania na Enter (jak msvcrt na Windows). Przywracany przy wyjsciu."""
    import atexit
    import termios
    import tty

    if _terminal["stary"] is not None or not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    _terminal["stary"] = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    atexit.register(lambda: termios.tcsetattr(fd, termios.TCSADRAIN, _terminal["stary"]))


def read_keys():
    """Wcisniete klawisze bez czekania (Windows i Linux). Enter zawsze jako "\\r"."""
    keys = []
    if WINDOWS:
        import msvcrt

        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):  # strzalki/F-klawisze - pomijamy
                msvcrt.getwch()
                continue
            keys.append(ch.lower())
        return keys

    import select

    if not sys.stdin.isatty():
        return keys
    _linux_terminal_surowy()
    tekst = ""
    while select.select([sys.stdin], [], [], 0)[0]:
        dane = os.read(sys.stdin.fileno(), 64).decode(errors="ignore")
        if not dane:
            break
        tekst += dane
    i = 0
    while i < len(tekst):
        ch = tekst[i]
        if ch == "\x1b" and tekst[i + 1:i + 2] in ("[", "O"):  # strzalki: ESC [ A - pomijamy
            i += 3
            continue
        keys.append("\r" if ch == "\n" else ch.lower())
        i += 1
    return keys


def klawisz_poz(arm, c):
    """P/R/M zapis poz. Zwraca komunikat albo None, jesli to nie ten klawisz."""
    if c == "m":
        return zapisz_poze(arm, POZA_DOMOWA)
    if c == "p":
        pozy = wczytaj(PLIK_POZ, {})
        wolne = [p for p in POZY_SKANU if p not in pozy]
        return zapisz_poze(arm, wolne[0]) if wolne else "sa juz 4 pozy skanu - R usuwa"
    if c == "c":
        pozy = wczytaj(PLIK_POZ, {})
        for p in POZY_SKANU:
            pozy.pop(p, None)
        zapisz(PLIK_POZ, pozy)
        return "usunieto pozy skanu - ENTER bedzie sie rozgladal"
    return None


def sterowanie(extra_keys=None, on_start=None, port=None, mock=False, http=False):
    """Tryb terminalowy (bez kamery). Uzywa go tez pizza.py.

    extra_keys = {"p": funkcja(arm)} - wlasne klawisze (maja pierwszenstwo);
    on_start(arm) - akcja na starcie;  http=True - nasluch sygnalu od RoArma.
    """
    extra_keys = extra_keys or {}
    arm = polacz(port, mock)
    if on_start:
        on_start(arm)
    st = Sterownik(arm)
    srv = serwer_http(arm) if http else None
    print(f"Pad: {'PODLACZONY' if st.pad_ok else 'brak (tylko klawiatura)'}")
    print("Klawisze (terminal musi byc aktywny):\n" + POMOC)
    if extra_keys:
        print("Dodatkowe klawisze:", ", ".join(k.upper() for k in extra_keys))
    if srv:
        print(f"Sygnal od RoArma: http://{moje_ip()}:{PORT_HTTP}/skanuj")
    print()
    ostatni, byl_zajety = 0.0, False
    try:
        while True:
            for k in read_keys():
                if k in ("\x1b", "q"):
                    raise KeyboardInterrupt
                if k in extra_keys:
                    if _zajety.is_set():
                        continue
                    print()
                    try:
                        extra_keys[k](arm)
                    except SO101Error as e:
                        print("BLAD:", e)
                    st.po_zadaniu()
                elif k == "\r":
                    zadanie(arm, ogladaj_puszke, "skan") or print("\nramie zajete")
                elif _zajety.is_set():
                    continue
                elif k == "h":
                    zadanie(arm, do_domu, "dom")
                elif (msg := klawisz_poz(arm, k)) is not None:
                    print(f"\n{msg}")
                else:
                    st.klawisz(k)

            if _zajety.is_set():
                byl_zajety = True
            else:
                if byl_zajety:
                    st.po_zadaniu()
                    byl_zajety = False
                pressed = st.krok()
                if "y" in pressed or "start" in pressed:
                    zadanie(arm, ogladaj_puszke, "skan")
                if "back" in pressed:
                    raise KeyboardInterrupt
                if time.time() - ostatni > 0.25:
                    ostatni = time.time()
                    print(f"\r{st.opis()}   ", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nKoniec - ramie trzyma pozycje.")
    except SO101Error as e:
        print("\nBLAD:", e)
    finally:
        if srv:
            srv.shutdown()
        if _zajety.is_set():
            _skan_koniec.wait(timeout=30)
        try:
            arm.hold()
        except SO101Error:
            pass
        arm.close()


# ----------------------------------------------------------------------------- ogladanie puszki
stan = {"stan": "czekam", "kod": None, "wynik": None}
_zajety = threading.Event()      # trwa automatyczny ruch (skan / powrot do domu)
_skan_koniec = threading.Event()


def otworz_kamere():
    import cv2

    # KAMERA w ustawieniach albo zmienna srodowiskowa, np. na Linuksie:  KAMERA=4 python3 ramie.py
    wybrana = os.environ.get("KAMERA", KAMERA)
    if wybrana is not None:
        kolejnosc = [int(wybrana)]
    else:  # zewnetrzne najpierw, na koncu kamera laptopa (0)
        kolejnosc = [1, 2, 3, 0] if WINDOWS else [2, 4, 6, 1, 3, 5, 0]
    backend = cv2.CAP_DSHOW if WINDOWS else cv2.CAP_V4L2
    for i in kolejnosc:
        kam = cv2.VideoCapture(i, backend)
        # MJPG + 1280x720: ostrzejsze kody niz domyslne 640x480, a wciaz plynnie
        kam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        kam.set(cv2.CAP_PROP_FRAME_WIDTH, KAMERA_ROZDZIELCZOSC[0])
        kam.set(cv2.CAP_PROP_FRAME_HEIGHT, KAMERA_ROZDZIELCZOSC[1])
        # Ekspozycje ustawiamy zawsze jawnie (Windows ja zapamietuje w sterowniku).
        # Uwaga: AUTO_EXPOSURE ma rozne wartosci - Windows 1 = auto, Linux (V4L2) 3 = auto, 1 = reczna.
        if KAMERA_EKSPOZYCJA is None:
            kam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1 if WINDOWS else 3)
        else:
            if not WINDOWS:
                kam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            kam.set(cv2.CAP_PROP_EXPOSURE, KAMERA_EKSPOZYCJA)
        if kam.isOpened() and kam.read()[0]:
            print(f"Kamera {i}" + ((" (laptop)" if i == 0 else " (zewnetrzna)") if WINDOWS else f" (/dev/video{i})"))
            return kam
        kam.release()
    raise SO101Error("nie znalazlem zadnej kamery")


class KameraWatek:
    """Kamera czytana w osobnym watku: zawsze NAJNOWSZA klatka + czas, kiedy przyszla.

    Bez tego sterownik kamery trzyma kolejke starych klatek (petla wolniejsza niz 30 kl/s dostaje obraz sprzed
    100+ ms), a dekodowanie MJPEG 1080p (~25 ms) blokuje petle. Tu dekoduje sie rownolegle z YOLO.
    """

    def __init__(self, kam):
        self.kam = kam
        self._nowa = threading.Condition()
        self._klatka, self._t, self._nr, self._oddana = None, 0.0, 0, 0
        self._stop = False
        self._watek = threading.Thread(target=self._petla, daemon=True)
        self._watek.start()

    def _petla(self):
        while not self._stop:
            if not self.kam.grab():  # samo odebranie klatki (bez dekodowania) - tanie
                time.sleep(0.01)
                continue
            t = time.time()  # klatka wlasnie przyszla z kamery
            if self._nr != self._oddana and t - self._t < 0.04:
                continue  # poprzednia swieza klatka czeka na petle - nie dekoduj na zapas (procesor = cieplo)
            ok, klatka = self.kam.retrieve()
            if ok:
                with self._nowa:
                    self._klatka, self._t, self._nr = klatka, t, self._nr + 1
                    self._nowa.notify_all()

    def read(self, timeout=1.0):
        """(ok, klatka, czas_klatki) - czeka na klatke, ktorej jeszcze nie oddal."""
        with self._nowa:
            if not self._nowa.wait_for(lambda: self._nr != self._oddana, timeout=timeout):
                return False, None, 0.0
            self._oddana = self._nr
            return True, self._klatka, self._t

    def release(self):
        self._stop = True
        self._watek.join(timeout=1.0)
        self.kam.release()


def ogladaj_puszke(arm):
    """Objazd z kamera, konczy gdy kamera zobaczy kod. Zwraca wynik."""
    t0 = time.time()
    stan.update(stan="skanuje", kod=None, wynik=None)
    start = arm.joints()
    pozy = wczytaj(PLIK_POZ, {})
    if all(p in pozy for p in POZY_SKANU):
        ruchy = [(p, {j: pozy[p][j] for j in MOVE_JOINTS}) for p in POZY_SKANU]
        print("\nOGLADAM PUSZKE (nauczone pozy)")
    else:
        ruchy = [(f"auto{i}", {j: start[j] + d for j, d in r.items()}) for i, r in enumerate(AUTO_RUCHY, 1)]
        print("\nOGLADAM PUSZKE (rozgladanie - brak poz skan1..4)")
    kod = None
    for nazwa, poza in ruchy:
        stan["stan"] = f"skanuje: {nazwa}"
        arm.move(poza, speed=PREDKOSC_SKANU)
        koniec = time.time() + PAUZA
        while time.time() < koniec and not stan["kod"]:
            time.sleep(0.05)
        if stan["kod"]:
            kod = stan["kod"]
            break
    stan["stan"] = "wracam"
    arm.move({j: start[j] for j in MOVE_JOINTS}, speed=PREDKOSC_SKANU)
    kaucja = (kod in wczytaj(PLIK_KAUCJI, [])) if kod else None
    return {"kod": kod, "kaucja": kaucja, "czas": round(time.time() - t0, 1)}


def zadanie(arm, funkcja, nazwa):
    """Uruchom automatyczny ruch w tle (sterowanie reczne wstrzymane). False jesli cos juz trwa."""
    if _zajety.is_set():
        return False
    _zajety.set()
    _skan_koniec.clear()

    def run():
        try:
            wynik = funkcja(arm)
        except SO101Error as e:
            wynik = {"blad": str(e)}
        if nazwa == "skan":
            stan["wynik"] = wynik
            print("WYNIK:", wynik)
        stan["stan"] = "czekam"
        _zajety.clear()
        _skan_koniec.set()

    threading.Thread(target=run, daemon=True).start()
    return True


# ----------------------------------------------------------------------------- panel w przegladarce (bez okna)
# jpg = podglad z napisami (panel), czysty = same ramki butelek i celownik (widok /pokaz na prezentacje)
_web = {"jpg": None, "czysty": None, "nr_jpg": 0, "nr_czysty": 0, "widzowie": 0, "widzowie_czysty": 0, "sledz": False,
        "wykrycia": [],
        "yolo": "lokalnie"}
_web_nowa = threading.Condition()  # nowa klatka podgladu
_web_klawisze = queue.SimpleQueue()  # klawisze z przegladarki -> petla glowna

PANEL_HTML = """<!doctype html><html lang="pl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Ramie SO-101</title><style>
body{margin:0;background:#111;color:#eee;font:15px system-ui,sans-serif}
main{display:grid;grid-template-columns:minmax(0,3fr) minmax(260px,1fr);gap:12px;padding:12px}
@media(max-width:800px){main{grid-template-columns:1fr}}
img{width:100%;background:#000;border-radius:6px}
#wynik{font-size:22px;font-weight:700;padding:10px;border-radius:6px;background:#333;margin:0 0 10px}
.k{background:#1e6b1e}.b{background:#8a1c1c}.o{background:#a35c00}
.p{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:0 0 10px}
button{padding:10px 4px;font:inherit;border:0;border-radius:6px;background:#2d2d2d;color:#eee;touch-action:none}
button:active,button.on{background:#0a6ebd}#stop{background:#8a1c1c}
h3{margin:12px 0 6px;font-size:13px;color:#aaa;text-transform:uppercase}
label{display:grid;grid-template-columns:1fr 48px;font-size:13px;margin:2px 0}input{grid-column:1/3}
small{color:#888}</style></head><body><main><div><img src="/podglad" alt="podglad kamery">
<small>Klawiatura dziala jak w oknie: W/S R/F A/D J/L U/O, 1/2/3, B stop, T sledzenie, M, H, Y, Enter</small></div>
<div><div id="wynik">...</div><div class="p"><button id="obr">Obrocono</button><button data-k="t" id="sl">Sledzenie</button>
<button data-k="b" id="stop">STOP</button></div>
<h3>Ruch (przytrzymaj)</h3><div class="p">
<button data-h="j">J pochyl</button><button data-h="w">W gora</button><button data-h="l">L pochyl</button>
<button data-h="a">A lewo</button><button data-h="s">S dol</button><button data-h="d">D prawo</button>
<button data-h="r">R dalej</button><button data-h="f">F blizej</button><span></span>
<button data-h="u">U obroc</button><span></span><button data-h="o">O obroc</button></div>
<div class="p"><button data-k="1">wolno</button><button data-k="2">srednio</button><button data-k="3">szybko</button></div>
<h3>Pozy</h3><div class="p"><button data-k="m">M tu patrz (stol)</button><button data-k="h">H dom</button>
<button data-k="y">Y historia</button></div><h3>Ustawienia</h3><div id="suwaki"></div></div></main><script>
const k=c=>fetch('/klawisz?k='+encodeURIComponent(c));let trzymane={};
function wcisnij(c){if(trzymane[c])return;k(c);trzymane[c]=setInterval(()=>k(c),1000/30)}
function pusc(c){clearInterval(trzymane[c]);delete trzymane[c]}
function puscWszystko(){Object.keys(trzymane).forEach(pusc)}
document.querySelectorAll('[data-h]').forEach(b=>{const c=b.dataset.h;
 b.onpointerdown=e=>{b.setPointerCapture(e.pointerId);wcisnij(c)};b.onpointerup=b.onpointercancel=()=>pusc(c)});
document.querySelectorAll('[data-k]').forEach(b=>b.onclick=()=>k(b.dataset.k));
document.getElementById('obr').onclick=()=>fetch('/obrocono');
const RUCH='wsrfadjluo',AKCJE='123btmhycp';
onkeydown=e=>{if(e.target.tagName=='INPUT')return;const c=e.key.toLowerCase();
 if(RUCH.includes(c)&&c.length==1){wcisnij(c);e.preventDefault()}
 else if(AKCJE.includes(c)&&c.length==1&&!e.repeat)k(c);else if(e.key=='Enter'&&!e.repeat)k('\\r')};
onkeyup=e=>pusc(e.key.toLowerCase());onblur=puscWszystko;
fetch('/ustawienia').then(r=>r.json()).then(s=>{const d=document.getElementById('suwaki');s.forEach((u,i)=>{
 const l=document.createElement('label');l.innerHTML=`<span>${u.napis}</span><b>${u.v}</b>
 <input type=range min=${u.lo} max=${u.hi} value=${u.v}>`;const r=l.querySelector('input');
 r.oninput=()=>l.querySelector('b').textContent=r.value;r.onchange=()=>fetch(`/ustaw?i=${i}&v=${r.value}`);d.append(l)})});
async function odswiez(){try{const w=await(await fetch('/wynik')).json(),e=document.getElementById('wynik');
 const o=w.stan=='WYNIK'?w:(w.ostatni_wynik||{});const t={KAUCYJNA:'k',BEZ_KAUCJI:'b',BRAK_KODU:'o'};
 e.className=w.stan=='WYNIK'?(t[w.wynik]||''):'';
 e.textContent=w.stan=='WYNIK'?`${w.wynik}${w.kwota!=null?' '+w.kwota.toFixed(2)+' '+w.waluta:''} ${w.nazwa||w.kod||''}`
  :`${w.stan}${o.wynik?' | ostatni: '+o.wynik+' '+(o.kod||''):''}`;
 document.getElementById('sl').classList.toggle('on',!!w.sledzenie)}catch(e){}setTimeout(odswiez,700)}odswiez();
</script></body></html>"""


def _ustaw_suwak(i, v):
    """Suwak nr i (z okna albo z przegladarki) -> ustawienie w programie."""
    napis, zmienna, lo, hi, mn = SUWAKI[i]
    v = min(max(int(v), lo), hi)
    if zmienna is None:
        _web_klawisze.put("sledz1" if v else "sledz0")
    else:
        globals()[zmienna] = bool(v) if zmienna in ("SLEDZ_TYLKO_KAUCJA", "SLEDZ_PLYNNIE") else v * mn


def _stan_suwakow():
    out = []
    for napis, zmienna, lo, hi, mn in SUWAKI:
        v = int(_web["sledz"]) if zmienna is None else int(round(float(globals()[zmienna]) / mn))
        out.append({"napis": napis, "lo": lo, "hi": hi, "v": min(max(v, lo), hi)})
    return out


def web_klatka(ekran, czysty=False):
    """Podaj klatke do podgladu w przegladarce (tylko gdy ktos patrzy - JPEG kosztuje procesor)."""
    import cv2

    if ekran.shape[1] > WEB_SZER:  # mniej danych przez WiFi = plynniejszy podglad
        ekran = cv2.resize(ekran, (WEB_SZER, ekran.shape[0] * WEB_SZER // ekran.shape[1]), interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode(".jpg", ekran, [cv2.IMWRITE_JPEG_QUALITY, 65])
    if ok:
        klucz = "czysty" if czysty else "jpg"
        with _web_nowa:  # osobny licznik na strumien - inaczej kazdy wysylalby tez klatki drugiego (2x WiFi)
            _web[klucz], _web["nr_" + klucz] = jpg.tobytes(), _web.get("nr_" + klucz, 0) + 1
            _web_nowa.notify_all()


def _dane_butelek():
    """Dla RoArma (butelki.py): butelki w kadrze (piksele pelnej klatki) + czy kamera stoi w pozycji patrzenia.

    Pozycja patrzenia = poza zapisana klawiszem M (spoczynek). Tylko z niej obraz da sie przeliczyc na stol.
    """
    widok = dict(_web.get("widok") or {"butelki": [], "szer": 0, "wys": 0})
    stawy = _web.get("stawy") or {}
    poza = wczytaj(PLIK_POZ, {}).get(POZA_DOMOWA)
    widok["stawy"] = stawy
    widok["poza_zapisana"] = bool(poza)
    widok["w_pozie"] = bool(poza and stawy and all(abs(stawy[j] - poza[j]) < 4.0 for j in MOVE_JOINTS))
    widok["stoi"] = bool(stawy) and time.time() - _web.get("ruch_t", 0.0) > 0.5 and not _zajety.is_set()
    widok["sledzenie"] = _web["sledz"]
    # o ile px przesuwa sie obraz na 1 st. stawu - butelki.py poprawia tym drobne bledy powrotu do pozycji
    widok["px_na_st"] = {SLEDZ_STAW_POZIOM: SLEDZ_ZNAK_POZIOM * SLEDZ_CZULOSC_POZIOM * widok.get("szer", 0) / 2,
                         SLEDZ_STAW_PION: SLEDZ_ZNAK_PION * SLEDZ_CZULOSC_PION * widok.get("wys", 0) / 2}
    return widok


def _dane_pokazu():
    """Wszystko dla widoku /pokaz: etap, wynik, co widzi siec, historia i liczniki butelek."""
    wyniki = list(stan.get("wyniki", {}).values())  # ostatni wynik kazdej butelki
    kaucyjne = [w for w in wyniki if w["wynik"] == "KAUCYJNA"]
    return {
        "inspekcja": stan.get("inspekcja") or {"stan": "SZUKAM"},
        "ostatni_wynik": stan.get("ostatni_wynik"),
        "historia": stan.get("historia", [])[-12:][::-1],
        "licznik": {"butelek": len(wyniki), "kaucyjnych": len(kaucyjne),
                    "bez_kaucji": sum(w["wynik"] == "BEZ_KAUCJI" for w in wyniki),
                    "suma": round(sum(w["kwota"] or 0 for w in kaucyjne), 2)},
        "wykrycia": _web["wykrycia"], "sledzenie": _web["sledz"], "kl_s": stan.get("kl_s", 0),
        "yolo": _web.get("yolo", "lokalnie"), "yolo_kl_s": _zdalny["kl_s"], "laptop": yolo_laptop_aktywny(),
        "roarm": (_web.get("roarm") or {}).get("stan") if time.time() - (_web.get("roarm") or {}).get("t", 0) < 90
        else None,
    }


def serwer_http(arm):
    class Obsluga(BaseHTTPRequestHandler):
        # polaczenie zostaje otwarte miedzy zapytaniami (laptop z YOLO: bez nowego TCP na kazda klatke),
        # a male pakiety ida od razu (Nagle + opozniony ACK Windows = nawet 200 ms na odpowiedz)
        protocol_version = "HTTP/1.1"
        disable_nagle_algorithm = True

        def _json(self, dane, kod=200):
            tresc = json.dumps(dane, ensure_ascii=False).encode()
            self.send_response(kod)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(tresc)))
            self.end_headers()
            self.wfile.write(tresc)

        def _podglad(self, czysty=False):
            """MJPEG: przegladarka pokazuje to jak film (<img src="/podglad">)."""
            self.close_connection = True  # strumien bez konca - po nim polaczenie sie zamyka
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=klatka")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            licznik = "widzowie_czysty" if czysty else "widzowie"
            klucz = "czysty" if czysty else "jpg"
            _web[licznik] += 1
            nr = -1
            try:
                while True:
                    with _web_nowa:
                        _web_nowa.wait_for(lambda: _web.get("nr_" + klucz, 0) != nr, timeout=2)
                        jpg, nr = _web[klucz], _web.get("nr_" + klucz, 0)
                    if jpg:
                        self.wfile.write(b"--klatka\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                         % len(jpg) + jpg + b"\r\n")
            except OSError:  # przegladarka zamknieta
                pass
            finally:
                _web[licznik] -= 1

        def do_GET(self):
            adres = urlsplit(self.path)
            q = {k: v[0] for k, v in parse_qs(adres.query).items()}
            # tresc zawsze przeczytac - przy otwartym polaczeniu inaczej zostalaby jako "nastepne zapytanie"
            dlugosc = int(self.headers.get("Content-Length") or 0)
            cialo = self.rfile.read(dlugosc) if dlugosc > 0 else b""
            if adres.path == "/":
                tresc = PANEL_HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(tresc)))
                self.end_headers()
                self.wfile.write(tresc)
            elif adres.path == "/podglad":
                self._podglad(czysty="czysty" in q)
            elif adres.path == "/pokaz":  # widok na prezentacje (plik czytany za kazdym razem - mozna go edytowac)
                try:
                    with open(os.path.join(_katalog, "pokaz.html"), "rb") as f:
                        tresc = f.read()
                except FileNotFoundError:
                    return self._json({"blad": "brak pokaz.html obok ramie.py"}, 404)
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(tresc)))
                self.end_headers()
                self.wfile.write(tresc)
            elif adres.path == "/dane":
                self._json(_dane_pokazu())
            elif adres.path == "/butelki":  # RoArm: gdzie stoja butelki (piksele) + czy kamera w pozycji patrzenia
                self._json(_dane_butelek())
            elif adres.path == "/sledzenie" and q.get("wl") in ("0", "1"):  # RoArm wlacza/wylacza sledzenie
                _web_klawisze.put("sledz" + q["wl"])
                self._json({"ok": True})
            elif adres.path == "/patrz":  # SO-101 wraca do pozycji patrzenia na stol (poza z klawisza M)
                poza, stawy = wczytaj(PLIK_POZ, {}).get(POZA_DOMOWA), _web.get("stawy") or {}
                if not poza:
                    return self._json({"blad": "brak pozycji patrzenia - ustaw kamere na stol i nacisnij M"}, 409)
                # sledzenie rusza tylko podstawa i nadgarstkiem - duza roznica w lift/elbow = stara poza z innego
                # ustawienia ramienia; wielki ruch moglby uderzyc w RoArma
                if not stawy:
                    return self._json({"blad": "jeszcze nie wiem, gdzie jest ramie - sprobuj za chwile"}, 409)
                daleko = [j for j in ("lift", "elbow") if abs(stawy[j] - poza[j]) > 15]
                if daleko and q.get("wymus") != "1":
                    return self._json({"blad": f"pozycja patrzenia daleko od obecnej ({', '.join(daleko)}) - "
                                               "ustaw kamere na stol i nacisnij M w panelu"}, 409)
                _web_klawisze.put("h")
                self._json({"ok": True})
            elif adres.path == "/roarm":  # napis o RoArmie w widoku /pokaz
                _web["roarm"] = {"stan": q.get("stan", "")[:120], "t": time.time()}
                self._json({"ok": True})
            elif adres.path == "/yolo":  # laptop: wynik YOLO poprzedniej klatki -> w odpowiedzi nastepna klatka
                try:
                    wynik_nr = int(q.get("nr", 0))
                except ValueError:
                    wynik_nr = 0
                _yolo_od_laptopa(cialo, wynik_nr)
                jpg, nr = _yolo_klatka()
                if jpg is None:
                    self.send_response(204)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.send_header("X-Nr", str(nr))
                self.end_headers()
                self.wfile.write(jpg)
            elif adres.path == "/klawisz" and q.get("k"):
                _web_klawisze.put(q["k"][:1].lower())
                self._json({"ok": True})
            elif adres.path == "/ustawienia":
                self._json(_stan_suwakow())
            elif adres.path == "/ustaw" and "i" in q and "v" in q:
                try:
                    _ustaw_suwak(int(q["i"]), q["v"])
                except (ValueError, IndexError):
                    return self._json({"blad": "zly suwak"}, 400)
                self._json({"ok": True})
            elif self.path.startswith("/skanuj"):
                print("\nSYGNAL z sieci")
                if not zadanie(arm, ogladaj_puszke, "skan"):
                    return self._json({"blad": "ramie zajete"}, 409)
                _skan_koniec.wait(timeout=120)
                self._json(stan["wynik"] or {"blad": "timeout"})
            elif self.path.startswith("/status"):
                self._json(stan)
            elif self.path.startswith("/wynik"):  # wynik inspekcji butelki (kaucja / bez / obroc)
                self._json(dict(stan.get("inspekcja") or {"stan": "brak inspekcji"},
                                ostatni_wynik=stan.get("ostatni_wynik"), sledzenie=_web["sledz"]))
            elif self.path.startswith("/obrocono"):  # butelka obrocona - czytaj kod od nowa
                if _inspekcja:
                    _inspekcja.po_obrocie()
                self._json({"ok": True})
            else:
                self._json({"blad": "uzyj /skanuj, /status, /wynik albo /obrocono"}, 404)

        do_POST = do_GET

        def log_message(self, *a):
            pass

    class Serwer(ThreadingHTTPServer):
        daemon_threads = True

        def handle_error(self, request, client_address):
            if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
                return  # przegladarka albo laptop rozlaczyl sie w trakcie - normalne, bez smiecenia w logu
            super().handle_error(request, client_address)

    srv = Serwer(("0.0.0.0", PORT_HTTP), Obsluga)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def _ean_ok(ean):
    """Suma kontrolna EAN-8/EAN-13 - odrzuca bledne odczyty."""
    if not ean.isdigit() or len(ean) not in (8, 13):
        return False
    d = [int(c) for c in ean]
    wagi = [3, 1] * (len(d) // 2) if len(d) == 8 else [1, 3] * 6
    return (10 - sum(w * x for w, x in zip(wagi, d[:-1])) % 10) % 10 == d[-1]


def znajdz_kody(klatka):
    """Wszystkie kody EAN w klatce: [{"ean", "cx", "cy", "rozmiar", "rogi"}] (rozmiar = czesc szerokosci obrazu)."""
    import cv2
    import numpy as np

    szer = klatka.shape[1]
    gray = cv2.cvtColor(klatka, cv2.COLOR_BGR2GRAY)
    wyniki = []
    try:
        import zxingcpp

        surowe = zxingcpp.read_barcodes(gray)
        if not surowe:  # puszki z odblaskami: wieksz kontrast
            surowe = zxingcpp.read_barcodes(cv2.createCLAHE(2.0, (8, 8)).apply(gray))
        for r in surowe:
            p = r.position
            rogi = np.array([[p.top_left.x, p.top_left.y], [p.top_right.x, p.top_right.y],
                             [p.bottom_right.x, p.bottom_right.y], [p.bottom_left.x, p.bottom_left.y]])
            wyniki.append((r.text, rogi))
    except ImportError:  # zapas: czytnik wbudowany w OpenCV
        ok, teksty, _t, pkt = cv2.barcode.BarcodeDetector().detectAndDecodeWithType(klatka)
        if ok and pkt is not None:
            wyniki = [(t, r) for t, r in zip(teksty, pkt) if t]
    kody = []

    def dodaj(ean, rogi):
        xs, ys = rogi[:, 0], rogi[:, 1]
        kody.append({"ean": ean, "cx": float(xs.mean()), "cy": float(ys.mean()),
                     "rozmiar": float(max(xs.max() - xs.min(), ys.max() - ys.min())) / szer,
                     "rogi": rogi.astype(int).reshape(-1, 1, 2)})

    for ean, rogi in wyniki:
        if _ean_ok(ean):
            dodaj(ean, rogi)
    # Rozmazany kod: cyfr nie da sie odczytac, ale prostokat kodu widac -> ean=None (do sledzenia)
    global _detektor
    if _detektor is None:
        _detektor = cv2.barcode.BarcodeDetector()
    ok, pkt = _detektor.detect(klatka)
    if ok and pkt is not None:
        for rogi in pkt:
            cx, cy = rogi[:, 0].mean(), rogi[:, 1].mean()
            if all(abs(cx - k["cx"]) + abs(cy - k["cy"]) > 0.05 * szer for k in kody):
                dodaj(None, rogi)
    return kody


_detektor = None


def ostrosc(klatka, calosc=False):
    """Ostrosc srodka obrazu (albo calego wycinka) - wariancja Laplasjanu. Kod czytelny zwykle od ~100."""
    import cv2

    h, w = klatka.shape[:2]
    kawalek = klatka if calosc else klatka[h // 4:3 * h // 4, w // 4:3 * w // 4]
    srodek = cv2.cvtColor(kawalek, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(srodek, cv2.CV_64F).var()


def czytaj_przekrzywiony(gray, rogi):
    """Kod widoczny, ale nieodczytany: wyprostuj go (kreski pionowo), powieksz, wyostrz, sprobuj kilku progow."""
    import cv2
    import numpy as np

    try:
        import zxingcpp
    except ImportError:
        return None
    (cx, cy), (rw, rh), kat = cv2.minAreaRect(rogi.astype(np.float32))
    M = cv2.getRotationMatrix2D((cx, cy), kat, 1.0)
    obr = cv2.warpAffine(gray, M, (gray.shape[1], gray.shape[0]), flags=cv2.INTER_CUBIC)
    w2, h2 = rw * 1.3, rh * 1.3
    wyc = obr[max(0, int(cy - h2 / 2)):int(cy + h2 / 2), max(0, int(cx - w2 / 2)):int(cx + w2 / 2)]
    if wyc.size == 0:
        return None
    formaty = [zxingcpp.BarcodeFormat.EAN13, zxingcpp.BarcodeFormat.EAN8]
    for skala in (2, 3):
        duzy = cv2.resize(wyc, None, fx=skala, fy=skala, interpolation=cv2.INTER_CUBIC)
        for obraz in (duzy, cv2.addWeighted(duzy, 1.8, cv2.GaussianBlur(duzy, (0, 0), 2), -0.8, 0)):
            for prog in (zxingcpp.Binarizer.LocalAverage, zxingcpp.Binarizer.GlobalHistogram):
                for w in zxingcpp.read_barcodes(obraz, formats=formaty, try_rotate=True, binarizer=prog):
                    if _ean_ok(w.text):
                        return w.text
    return None


class CzytnikKodow:
    """Czyta kody kreskowe w osobnym watku - sledzenie nie czeka na wolne czytanie (stalo rowny rytm).

    Kazda klatka: caly obraz + powiekszony wycinek sledzonej butelki (maly kod latwiej odczytac).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._klatka = None
        self._cel = None
        self.kody = []         # ostatnie kody z calego obrazu
        self.kod_celu = {}     # id sledzonej butelki -> odczytany EAN
        self._glosy = {}       # id butelki -> {EAN: ile razy odczytany}
        self.pewne = set()     # EAN-y potwierdzone (KOD_POTWIERDZ zgodnych odczytow) - te wystarczy przeczytac raz
        self._stop = False
        self.pauza = False     # True = nic nie czytaj (wynik juz rozstrzygniety)
        self._ost = 0.0
        threading.Thread(target=self._petla, daemon=True).start()

    def podaj(self, klatka, cel_id=None, box=None):
        with self._lock:
            self._klatka = klatka
            self._cel = (cel_id, box) if cel_id and box else None

    def potrzebna_klatka(self):
        """Czy czytnik czeka na nowa klatke (kopiujemy ja tylko wtedy - oszczednosc czasu petli)."""
        return self._klatka is None and not self.pauza and time.time() - self._ost >= 1.0 / CZYTNIK_NA_SEKUNDE

    def zatrzymaj(self):
        self._stop = True

    def _petla(self):
        import cv2

        while not self._stop:
            with self._lock:
                klatka, cel = self._klatka, self._cel
                self._klatka = None
            if klatka is None:
                time.sleep(0.01)
                continue
            self._ost = time.time()
            try:
                if not cel:  # bez celu: caly obraz (np. objazd puszki pod ENTER)
                    self.kody = znajdz_kody(klatka)
                    continue
                self.kody = []
                cid, (x0, y0, x1, y1) = cel
                ean = None
                if ean is None:  # sam wycinek butelki (powiekszony, gdy maly) - duzo szybciej niz caly obraz 1080p
                    h, w = klatka.shape[:2]
                    mx, my = (x1 - x0) * 0.15, (y1 - y0) * 0.15
                    wyc = klatka[max(0, int(y0 - my)):min(h, int(y1 + my)), max(0, int(x0 - mx)):min(w, int(x1 + mx))]
                    if wyc.size:
                        if max(wyc.shape[:2]) < 700:
                            wyc = cv2.resize(wyc, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
                        kody_wyc = znajdz_kody(wyc)
                        ean = next((k["ean"] for k in kody_wyc if k["ean"]), None)
                        if ean is None:  # przekrzywiony kod w wycinku: wyprostuj i czytaj mocniej
                            gray = cv2.cvtColor(wyc, cv2.COLOR_BGR2GRAY)
                            for k in kody_wyc:
                                ean = czytaj_przekrzywiony(gray, k["rogi"].reshape(-1, 2))
                                if ean:
                                    break
                if ean:
                    # pojedynczy odczyt bywa bledny (pomylone cyfry, a suma kontrolna przypadkiem sie zgadza -
                    # tak wyszlo "0415191856075" zamiast 8445291856035): przyjmij dopiero zgodne odczyty
                    if len(self._glosy) > 200:
                        self._glosy.clear()
                    glosy = self._glosy.setdefault(cid, {})
                    glosy[ean] = glosy.get(ean, 0) + 1
                    if glosy[ean] >= KOD_POTWIERDZ or ean in self.pewne:
                        self.pewne.add(ean)
                        self.kod_celu[cid] = ean
            except Exception as e:  # czytnik nie moze wywalic calego programu
                print("\nczytnik kodow:", e)


class Kaucje:
    """Czy EAN jest kaucyjny: kaucja.json albo api.kaucja.pl (w tle, z pamiecia)."""

    def __init__(self):
        self.status = {}  # ean -> True / False / None (sprawdzam)
        self.nazwy = {}
        self.kwoty = {}   # ean -> (kwota, waluta)

    def sprawdz(self, ean):
        if ean in self.status:
            return self.status[ean]
        if ean in wczytaj(PLIK_KAUCJI, []):
            self.status[ean] = True
            return True
        self.status[ean] = None
        threading.Thread(target=self._api, args=(ean,), daemon=True).start()
        return None

    def _api(self, ean):
        try:
            import requests

            r = requests.get(f"https://api.kaucja.pl/buf/pos/product/{ean}", timeout=5)
            dane = r.json() if r.status_code == 200 else {}
            self.status[ean] = dane.get("deposit") is not None
            self.nazwy[ean] = dane.get("publishedName") or dane.get("name") or ""
            kaucja = dane.get("deposit") or {}
            if kaucja.get("amount") is not None:
                self.kwoty[ean] = (kaucja.get("amount"), kaucja.get("currency") or "PLN")
        except Exception:
            self.status[ean] = False  # brak sieci / blad -> traktuj jak nie-kaucyjny
        print(f"\n{ean}: {'KAUCJA' if self.status[ean] else 'bez kaucji'} {self.nazwy.get(ean, '')}")


_kolejka_logu = None


def _pisz_log(kolejka):
    with open(os.path.join(_katalog, SLEDZ_LOG), "a", encoding="utf-8") as f:
        while True:
            f.write(kolejka.get())
            if kolejka.empty():
                f.flush()


def _log(tekst):
    """Zapis do logu w tle (otwieranie pliku przy kazdym wpisie trwalo ~200 ms i zacinalo program)."""
    global _kolejka_logu
    if not SLEDZ_LOG:
        return
    if _kolejka_logu is None:
        import queue

        _kolejka_logu = queue.Queue()
        threading.Thread(target=_pisz_log, args=(_kolejka_logu,), daemon=True).start()
    teraz = time.time()
    _kolejka_logu.put(f"{time.strftime('%H:%M:%S', time.localtime(teraz))}.{int(teraz * 1000) % 1000:03d} {tekst}\n")


class Sledzenie:
    """Trzyma kod kaucyjny na srodku obrazu. Puszcza, gdy za daleko albo zgubiony;
    przelacza sie na inna butelke kaucyjna, jesli jest wyraznie blizej."""

    def __init__(self):
        self.cel = None          # ean sledzonej butelki
        self.rozmiar = 0.0
        self.widziany = 0.0
        self.xy = None           # ostatnia pozycja kodu (px)
        self._reset()

    def krok(self, kody, kaucje, szer, wys, st, dt):
        """Aktualizuje cel i przesuwa st.target. Zwraca komunikat albo None."""
        teraz = time.time()
        kaucyjne = [k for k in kody if k["ean"] and kaucje.sprawdz(k["ean"]) and k["rozmiar"] >= MIN_SZEROKOSC]
        najblizszy = max(kaucyjne, key=lambda k: k["rozmiar"], default=None)
        msg = None
        biezacy = next((k for k in kody if self.cel and k["ean"] == self.cel), None)
        if biezacy is None and self.cel and self.xy:
            # cyfr nie widac (rozmycie) - bierz najblizszy nieodczytany prostokat kodu
            bliskie = [k for k in kody if k["ean"] is None and
                       abs(k["cx"] - self.xy[0]) + abs(k["cy"] - self.xy[1]) < SLEDZ_PROMIEN * szer]
            biezacy = min(bliskie, key=lambda k: abs(k["cx"] - self.xy[0]) + abs(k["cy"] - self.xy[1]),
                          default=None)

        if self.cel is None:
            if najblizszy:
                self.cel = najblizszy["ean"]
                biezacy = najblizszy
                msg = f"SLEDZE {self.cel}"
        elif biezacy is not None:
            if biezacy["rozmiar"] < MIN_SZEROKOSC:
                msg, self.cel, biezacy = f"{self.cel} ODDALILA SIE - puszczam", None, None
            elif najblizszy and najblizszy["ean"] != self.cel and \
                    najblizszy["rozmiar"] > biezacy["rozmiar"] * PRZELACZ_GDY:
                msg = f"INNA BLIZEJ: {self.cel} -> {najblizszy['ean']}"
                self.cel, biezacy = najblizszy["ean"], najblizszy
        elif najblizszy:  # cel zniknal, ale jest inna butelka kaucyjna
            msg = f"INNA BUTELKA: {self.cel} -> {najblizszy['ean']}"
            self.cel, biezacy = najblizszy["ean"], najblizszy
        elif teraz - self.widziany > ZGUBIONY_PO:
            msg, self.cel = f"ZGUBIONA {self.cel} - puszczam", None

        if biezacy is None:
            if self.cel and not msg:
                self._szukaj(st, teraz)
            return msg
        self.widziany, self.rozmiar = teraz, biezacy["rozmiar"]
        self.zrodlo = "kod" if biezacy["ean"] else ("butelka" if biezacy.get("butelka") else "prostokat")
        self.xy = (biezacy["cx"], biezacy["cy"])
        self.blad = ((biezacy["cx"] - szer / 2) / (szer / 2), (biezacy["cy"] - wys / 2) / (wys / 2),
                     biezacy["ean"] is not None)
        if msg:  # nowy cel - zacznij od nowa
            self._reset()
        # historia pozycji kodu (ostatnie 0.6 s) - do przewidywania, gdzie ucieka
        self._hist = [h for h in self._hist if teraz - h[0] < 0.6] + [(teraz, self.blad[0], self.blad[1])]
        self._szukano = 0.0
        return self._steruj(st, msg, teraz)

    def _szukaj(self, st, teraz):
        """Kod chwilowo niewidoczny: jedz dalej tam, dokad uciekal (ograniczony czas i kat)."""
        if SLEDZ_PLYNNIE:
            if hasattr(self, "g"):
                self._szukaj_plynnie(st, teraz)
            return
        if not self._hist or teraz < self._nastepny or not hasattr(self, "g"):
            return
        od_utraty = teraz - self.widziany
        if od_utraty < SLEDZ_SZUKAJ_PO:  # chwilowy brak odczytu - stoj, nie machaj
            return
        if od_utraty > SLEDZ_SZUKAJ_CZAS or self._szukano >= SLEDZ_SZUKAJ_MAX:
            return
        (t0, x0, y0), (t1, x1, y1) = self._hist[0], self._hist[-1]
        # predkosc tylko z wiarygodnej historii - inaczej szum wykrycia udaje ruch butelki
        if len(self._hist) >= 3 and t1 - t0 >= 0.3:
            vx, vy = (x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0)
        else:
            vx = vy = 0.0
        horyzont = min(od_utraty + SLEDZ_PAUZA, 1.0)
        przew = {"x": max(-1.5, min(1.5, x1 + vx * horyzont)), "y": max(-1.5, min(1.5, y1 + vy * horyzont))}
        if max(abs(przew["x"]), abs(przew["y"])) < SLEDZ_START:
            return  # zniknal blisko srodka = problem z odczytem, a nie ucieczka - stoj
        stawy = {"x": SLEDZ_STAW_POZIOM, "y": SLEDZ_STAW_PION}
        zostalo = SLEDZ_SZUKAJ_MAX - self._szukano
        ruch = {}
        for o, staw in stawy.items():
            if abs(przew[o]) > SLEDZ_CEL:
                d = -SLEDZ_KROK * przew[o] / self.g[o]
                d = max(-min(SLEDZ_MAX_KROK, zostalo), min(min(SLEDZ_MAX_KROK, zostalo), d))
                if abs(d) >= SLEDZ_MIN_KROK:
                    st.target[staw] += d
                    st.wolne.add(staw)
                    ruch[staw] = round(d, 1)
        if ruch:
            self._szukano += max(abs(d) for d in ruch.values())
            self._nastepny = teraz + SLEDZ_PAUZA
            self._w_celu = False
            self._ostatni_ruch = None  # nie ucz czulosci z ruchow na slepo
            self._probki = []
            _log(f"SZUKAM cel={self.cel} przewidziany x={przew['x']:+.2f} y={przew['y']:+.2f} ruch={ruch} "
                 f"lacznie {self._szukano:.0f} st.")

    def _reset(self):
        self._probki = []          # pomiary z nieruchomego obrazu po ostatnim ruchu
        self._ostatni_ruch = None  # (blad_x, blad_y, ruch_x, ruch_y) - do nauki czulosci
        self._w_celu = False       # wycentrowany -> stoi, dopoki kod wyraznie nie ucieknie
        self._nastepny = 0.0
        self._poprz = None
        self._hist = []            # (czas, blad_x, blad_y) - do przewidywania ruchu kodu
        self._filtr = None         # wygladzony blad (x, y) - tryb plynny
        self.kod_wzgl = None       # gdzie na butelce jest kod (wzgledem ramki) - tam celujemy
        self._jedzie = {"x": False, "y": False}  # histereza trybu plynnego
        self._uciete = {"gora": False, "dol": False, "lewa": False, "prawa": False}
        self._w = {"x": 0.0, "y": 0.0}  # biezaca predkosc stawow (st./s) - tryb plynny
        self._t_ost = None
        self._szukano = 0.0        # ile st. przejechal "na slepo" od utraty kodu

    def _dt(self, teraz):
        dt = 0.05 if self._t_ost is None else min(max(teraz - self._t_ost, 0.0), 0.2)
        self._t_ost = teraz
        return dt

    def _opoznienie(self):
        """Sprzed ilu sekund jest ruch ramienia, ktory widac na obrazie (znany wiek klatki -> dokladnie)."""
        return getattr(self, "_opozn", None) or SLEDZ_OPOZNIENIE

    def zmierz_opoznienie(self, t_klatki, teraz):
        """Wiek biezacej klatki + stale opoznienie serwa i kamery (wygladzone - bez skokow)."""
        o = KAMERA_OPOZNIENIE + max(0.0, teraz - t_klatki)
        stare = getattr(self, "_opozn", None)
        self._opozn = o if stare is None else stare + 0.2 * (o - stare)

    def _jedz(self, st, dt):
        """Przesun cele stawow o predkosc * dt; serwo dostaje predkosc dopasowana do ruchu."""
        stawy = {"x": SLEDZ_STAW_POZIOM, "y": SLEDZ_STAW_PION}
        ruch = 0.0
        for o, staw in stawy.items():
            w = self._w[o]
            if abs(w) < 0.05:
                if abs(getattr(self, "_w_poprz", {}).get(o, 0.0)) >= 0.05:
                    st.wolne.add(staw)
                    st.w_sled[staw] = 0.0  # wlasnie stanal: wyslij cel bez wyprzedzenia
                continue
            st.target[staw] += w * dt
            st.wolne.add(staw)
            st.w_sled[staw] = w
            ruch = max(ruch, abs(w * dt))
        self._w_poprz = dict(self._w)
        return ruch

    def _steruj_plynnie(self, st, msg, teraz):
        """Plynne sledzenie: predkosc proporcjonalna do (wygladzonej) odleglosci celu od srodka."""
        dt = self._dt(teraz)
        ex, ey = self.blad[:2]
        if msg or self._filtr is None:
            self._filtr = (ex, ey)
            self._ff = {"x": 0.0, "y": 0.0}
            self._hist_w = []
            self._poprz_f = None
        a = SLEDZ_FILTR
        self._filtr = (self._filtr[0] + a * (ex - self._filtr[0]), self._filtr[1] + a * (ey - self._filtr[1]))
        # predkosc samej butelki w obrazie = zmiana obrazu minus to, co zrobil nasz wlasny ruch
        # (kamera pokazuje nasz ruch z opoznieniem SLEDZ_OPOZNIENIE)
        if self._poprz_f is not None and dt > 0:
            w_wtedy = next((w for t, w in reversed(self._hist_w) if t <= teraz - self._opoznienie()),
                           {"x": 0.0, "y": 0.0})
            for i, o in enumerate(("x", "y")):
                v_butelki = (self._filtr[i] - self._poprz_f[i]) / dt - self.g[o] * w_wtedy[o]
                self._ff[o] += SLEDZ_FF_GLADKOSC * (-v_butelki / self.g[o] - self._ff[o])
        self._poprz_f = self._filtr
        for i, o in enumerate(("x", "y")):
            e = self._filtr[i]
            rusza_sie = abs(self._ff[o]) > SLEDZ_FF_PROG  # butelka naprawde jedzie
            # histereza: stojace ramie rusza dopiero przy wyraznym odjechaniu, jadace dojezdza do samego srodka
            prog = SLEDZ_CEL if (self._jedzie[o] or rusza_sie) else SLEDZ_START_PLYNNIE
            nadmiar = max(0.0, abs(e) - SLEDZ_CEL) / (1.0 - SLEDZ_CEL) if abs(e) > prog else 0.0
            w = -math.copysign(nadmiar, e) * SLEDZ_PETLA / self.g[o]
            if rusza_sie:  # butelka sie rusza -> jedz razem z nia
                w += SLEDZ_FF * self._ff[o]
            self._jedzie[o] = abs(w) > 0.05
            uc = getattr(self, "_uciete", {})
            ucieta = (uc.get("gora") or uc.get("dol")) if o == "y" else (uc.get("lewa") or uc.get("prawa"))
            v_max = UCIETA_MAX_V if ucieta else SLEDZ_MAX_V  # cel to tylko szacunek -> ostrozniej
            w = max(-v_max, min(v_max, w))
            dw = SLEDZ_PRZYSP * dt  # predkosc zmienia sie stopniowo: plynny start i hamowanie, bez szarpniec
            self._w[o] += max(-dw, min(dw, w - self._w[o]))
        self._hist_w = [h for h in self._hist_w if teraz - h[0] < 1.0] + [(teraz, dict(self._w))]
        if SLEDZ_UCZ_CZULOSC:
            self._ucz_czulosc(st, teraz)
        self._jedz(st, dt)
        if teraz - getattr(self, "_log_t", 0.0) > SLEDZ_LOG_CO:
            self._log_t = teraz
            _log(f"cel={self.cel} x={self._filtr[0]:+.2f} y={self._filtr[1]:+.2f} {getattr(self, 'zrodlo', '?')} "
                 f"pan={st.here['pan']:.1f} wflex={st.here['wflex']:.1f} "
                 f"predkosc pan={self._w['x']:+.1f} wflex={self._w['y']:+.1f} st/s "
                 f"czulosc x={self.g['x']:+.3f} y={self.g['y']:+.3f} opozn={self._opoznienie():.2f}")
        return msg

    def _ucz_czulosc(self, st, teraz):
        """Czulosc = zmiana polozenia celu w obrazie / ruch stawu, ktory ja spowodowal (kamera widzi go z opoznieniem).

        Zalezy od odleglosci butelki (blisko = maly ruch, duze przesuniecie obrazu), wiec uczymy sie jej na biezaco.
        Pomiary, ktore nie pasuja (np. butelka sama sie ruszyla), odrzucamy.
        """
        stawy = {"x": SLEDZ_STAW_POZIOM, "y": SLEDZ_STAW_PION}
        h = getattr(self, "_hist_g", [])
        uc = getattr(self, "_uciete", {})
        ucieta = {"x": bool(uc.get("lewa") or uc.get("prawa")), "y": bool(uc.get("gora") or uc.get("dol"))}
        h = [x for x in h if teraz - x[0] < 2.0] + [(teraz, self._filtr, {o: st.here[j] for o, j in stawy.items()},
                                                     ucieta)]
        self._hist_g = h
        if teraz - getattr(self, "_g_t", 0.0) < 0.2:
            return
        self._g_t = teraz

        def wpis(t):
            return next((x for x in reversed(h) if x[0] <= t), None)

        okno, opozn = 0.4, self._opoznienie()
        teraz_e, stary_e = h[-1], wpis(teraz - okno)
        teraz_k, stary_k = wpis(teraz - opozn), wpis(teraz - okno - opozn)
        if not (stary_e and teraz_k and stary_k):
            return
        for i, o in enumerate(("x", "y")):
            if any(x[3][o] for x in h if teraz - x[0] <= okno + opozn):
                continue  # w oknie pomiaru butelka byla ucieta - nie ucz sie z tego
            dth = teraz_k[2][o] - stary_k[2][o]
            if abs(dth) < 0.8:
                continue
            g_obs = (teraz_e[1][i] - stary_e[1][i]) / dth
            if g_obs * self.g[o] > 0 and 0.25 <= g_obs / self.g[o] <= 4.0:
                g = self.g[o] + 0.3 * (g_obs - self.g[o])
                start = SLEDZ_CZULOSC_POZIOM if o == "x" else SLEDZ_CZULOSC_PION
                self.g[o] = math.copysign(min(max(abs(g), CZULOSC_ZAKRES[0] * start), CZULOSC_ZAKRES[1] * start), g)

    def _szukaj_plynnie(self, st, teraz):
        """Cel chwilowo niewidoczny: jesli uciekal - jedz dalej za nim, inaczej plynnie wyhamuj."""
        dt = self._dt(teraz)
        od_utraty = teraz - self.widziany
        uciekal = self._filtr is not None and max(abs(self._filtr[0]), abs(self._filtr[1])) > SLEDZ_START
        if not (uciekal and od_utraty < SLEDZ_SZUKAJ_CZAS and self._szukano < SLEDZ_SZUKAJ_MAX):
            zanik = math.exp(-dt / SLEDZ_HAMOWANIE)
            self._w = {o: w * zanik for o, w in self._w.items()}
        self._szukano += self._jedz(st, dt)

    def _steruj(self, st, msg, teraz):
        """Popraw i poczekaj: mediana kilku klatek z nieruchomego obrazu -> jeden ruch -> pauza."""
        if not hasattr(self, "_probki"):
            self._reset()
        if not hasattr(self, "g"):  # czulosc: o ile przesuwa sie kod (czesc polowy obrazu) na 1 st. stawu
            self.g = {"x": SLEDZ_ZNAK_POZIOM * SLEDZ_CZULOSC_POZIOM, "y": SLEDZ_ZNAK_PION * SLEDZ_CZULOSC_PION}
        if SLEDZ_PLYNNIE:
            return self._steruj_plynnie(st, msg, teraz)
        stawy = {"x": SLEDZ_STAW_POZIOM, "y": SLEDZ_STAW_PION}
        teraz_poz = {o: st.here[j] for o, j in stawy.items()}
        rusza_sie = self._poprz and any(abs(teraz_poz[o] - self._poprz[o]) > 0.3 for o in stawy)
        self._poprz = teraz_poz
        # czekaj, az minie pauza i serwa stana - dopiero wtedy obraz jest aktualny i ostry
        if teraz < self._nastepny or rusza_sie:
            self._probki = []
            return msg
        self._probki.append(self.blad[:2])
        if len(self._probki) < SLEDZ_PROBKI:
            return msg
        bx = sorted(p[0] for p in self._probki)[len(self._probki) // 2]
        by = sorted(p[1] for p in self._probki)[len(self._probki) // 2]
        self._probki = []
        blad = {"x": bx, "y": by}

        # nauka czulosci z poprzedniego ruchu (odrzuca pomiary, gdy butelka sama sie ruszyla)
        if self._ostatni_ruch:
            e0, d0 = self._ostatni_ruch
            for o in stawy:
                if abs(d0[o]) >= 1.0:
                    g_obs = (blad[o] - e0[o]) / d0[o]
                    if g_obs * self.g[o] > 0 and 0.3 <= g_obs / self.g[o] <= 3.0:
                        self.g[o] += 0.4 * (g_obs - self.g[o])
            self._ostatni_ruch = None

        ruch = {}
        prog = SLEDZ_START if self._w_celu else SLEDZ_CEL
        for o, staw in stawy.items():
            if abs(blad[o]) > prog:
                d = -SLEDZ_KROK * blad[o] / self.g[o]
                d = max(-SLEDZ_MAX_KROK, min(SLEDZ_MAX_KROK, d))
                if abs(d) >= SLEDZ_MIN_KROK:
                    st.target[staw] += d
                    st.wolne.add(staw)
                    ruch[o] = d
        self._w_celu = not ruch
        if ruch:
            self._ostatni_ruch = (blad, {o: ruch.get(o, 0.0) for o in stawy})
            self._nastepny = teraz + SLEDZ_PAUZA
        _log(f"cel={self.cel} x={bx:+.2f} y={by:+.2f} {getattr(self, 'zrodlo', '?')} "
             f"pan={st.here['pan']:.1f} wflex={st.here['wflex']:.1f} "
             f"ruch={ {stawy[o]: round(d, 1) for o, d in ruch.items()} } "
             f"czulosc x={self.g['x']:+.3f} y={self.g['y']:+.3f}{' W CELU' if self._w_celu else ''}")
        return msg


# ---- historia sledzenia (autor: Kakukoro) - pozycje butelek w czasie, zapis do tracking_history.json
class TrackedItem:
    """Represents a single tracked can/bottle with its history."""
    
    def __init__(self, ean, first_seen=None, name=""):
        self.ean = ean
        self.name = name
        self.first_seen = first_seen or time.time()
        self.last_seen = time.time()
        self.positions = []
        self.status = "active"
        self.total_tracking_time = 0.0
        self.pickup_count = 0
        self.last_pickup_time = None
        self.average_position = None
    
    def add_position(self, timestamp, cx, cy, rozmiar, pan, wflex):
        """Add a new position observation."""
        self.positions.append((timestamp, cx, cy, rozmiar, pan, wflex))
        self.last_seen = timestamp
        cutoff = timestamp - 300
        self.positions = [p for p in self.positions if p[0] >= cutoff]
    
    def get_velocity(self):
        """Calculate current velocity based on recent positions."""
        if len(self.positions) < 2:
            return (0.0, 0.0)
        recent = self.positions[-5:]
        if len(recent) < 2:
            return (0.0, 0.0)
        t0, x0, y0, _, _, _ = recent[0]
        t1, x1, y1, _, _, _ = recent[-1]
        dt = t1 - t0
        if dt < 0.1:
            return (0.0, 0.0)
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        return (vx, vy)
    
    def predict_position(self, future_time):
        """Predict position at a future time based on velocity."""
        if not self.positions:
            return None
        last_time, last_x, last_y, _, _, _ = self.positions[-1]
        dt = future_time - last_time
        vx, vy = self.get_velocity()
        pred_x = last_x + vx * dt
        pred_y = last_y + vy * dt
        return (pred_x, pred_y)
    
    def to_dict(self):
        """Convert to dictionary for serialization."""
        return {
            "ean": self.ean,
            "name": self.name,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
            "status": self.status,
            "total_tracking_time": self.total_tracking_time,
            "pickup_count": self.pickup_count,
            "last_pickup_time": self.last_pickup_time,
            "positions": self.positions,
            "average_position": self.average_position
        }
    
    @classmethod
    def from_dict(cls, data):
        """Create from dictionary."""
        item = cls(data["ean"], data.get("first_seen"), data.get("name", ""))
        item.last_seen = data.get("last_seen", time.time())
        item.status = data.get("status", "active")
        item.total_tracking_time = data.get("total_tracking_time", 0.0)
        item.pickup_count = data.get("pickup_count", 0)
        item.last_pickup_time = data.get("last_pickup_time")
        item.positions = data.get("positions", [])
        item.average_position = data.get("average_position")
        return item


class TrackingHistory:
    """Manages tracking history for all detected cans/bottles."""
    
    def __init__(self, max_age=3600):
        self.items = {}
        self.max_age = max_age
        self.last_cleanup = time.time()
    
    def update(self, ean, name, timestamp, cx, cy, rozmiar, pan, wflex):
        """Update tracking data for a specific EAN."""
        if ean not in self.items:
            self.items[ean] = TrackedItem(ean, timestamp, name)
        else:
            if name and not self.items[ean].name:
                self.items[ean].name = name
        self.items[ean].add_position(timestamp, cx, cy, rozmiar, pan, wflex)
        return self.items[ean]
    
    def mark_pickup(self, ean):
        """Mark an item as picked up."""
        if ean in self.items:
            self.items[ean].pickup_count += 1
            self.items[ean].last_pickup_time = time.time()
            self.items[ean].status = "picked_up"
    
    def mark_lost(self, ean):
        """Mark an item as lost."""
        if ean in self.items:
            self.items[ean].status = "lost"
    
    def cleanup(self, current_time=None):
        """Remove old items that haven't been seen in a while."""
        if current_time is None:
            current_time = time.time()
        if current_time - self.last_cleanup < 60:
            return
        self.last_cleanup = current_time
        to_remove = [
            ean for ean, item in self.items.items()
            if current_time - item.last_seen > self.max_age and item.status != "picked_up"
        ]
        for ean in to_remove:
            del self.items[ean]
    
    def get_active_items(self):
        """Get all currently active (recently seen) items."""
        self.cleanup()
        return {ean: item for ean, item in self.items.items()
                if item.status == "active"}
    
    def get_item(self, ean):
        """Get a specific tracked item."""
        return self.items.get(ean)
    
    def predict_position(self, ean, future_time):
        """Predict where an item will be at a future time."""
        item = self.items.get(ean)
        if item:
            return item.predict_position(future_time)
        return None
    
    def save(self, filepath=None):
        """Save tracking history to a file."""
        filepath = filepath or os.path.join(_katalog, "tracking_history.json")
        data = {
            "items": {ean: item.to_dict() for ean, item in self.items.items()},
            "last_cleanup": self.last_cleanup,
            "saved_at": time.time()
        }
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"Failed to save tracking history: {e}")
            return False
    
    def load(self, filepath=None):
        """Load tracking history from a file."""
        filepath = filepath or os.path.join(_katalog, "tracking_history.json")
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.items = {}
            for ean, item_data in data.get("items", {}).items():
                self.items[ean] = TrackedItem.from_dict(item_data)
            self.last_cleanup = data.get("last_cleanup", time.time())
            return True
        except FileNotFoundError:
            return False
        except Exception as e:
            print(f"Failed to load tracking history: {e}")
            return False


MODEL_URL = "http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_coco_2018_03_29.tar.gz"
MODEL_PBTXT_URL = ("https://raw.githubusercontent.com/opencv/opencv_extra/4.x/testdata/dnn/"
                   "ssd_mobilenet_v2_coco_2018_03_29.pbtxt")


def pobierz_model():
    """Sciagnij model wykrywania butelek do dorm_keeper/modele (jednorazowo, ok. 190 MB archiwum)."""
    import tarfile
    import urllib.request

    folder = os.path.join(_katalog, "modele")
    pb = os.path.join(folder, "ssd_mobilenet_v2_coco.pb")
    pbtxt = os.path.join(folder, "ssd_mobilenet_v2_coco.pbtxt")
    os.makedirs(folder, exist_ok=True)
    if not os.path.exists(pb):
        print("Pobieram model wykrywania butelek (jednorazowo, kilka minut)...")
        archiwum = os.path.join(folder, "ssd.tar.gz")
        urllib.request.urlretrieve(MODEL_URL, archiwum)
        with tarfile.open(archiwum) as tar:
            czlon = next(m for m in tar.getmembers() if m.name.endswith("frozen_inference_graph.pb"))
            with tar.extractfile(czlon) as src, open(pb, "wb") as dst:
                dst.write(src.read())
        os.remove(archiwum)
    if not os.path.exists(pbtxt):
        urllib.request.urlretrieve(MODEL_PBTXT_URL, pbtxt)
    return pb, pbtxt


YOLO_URL = "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"


def pobierz_yolo():
    import urllib.request

    sciezka = os.path.join(_katalog, "modele", "yolo11n.onnx")
    if not os.path.exists(sciezka):
        os.makedirs(os.path.dirname(sciezka), exist_ok=True)
        print("Pobieram model YOLO11n (jednorazowo, 11 MB)...")
        urllib.request.urlretrieve(YOLO_URL, sciezka)
    return sciezka


# ----------------------------------------------------------------------------- YOLO liczone na laptopie
# Na Raspberry YOLO daje ~10 kl/s. Laptop (yolo_laptop.py) sam laczy sie tutaj: POST /yolo z wynikiem poprzedniej
# klatki, w odpowiedzi dostaje nastepna (pomniejszona). Laptop nic nie nasluchuje = zapora Windows nie przeszkadza.
# Laptop sie nie odzywa dluzej niz ZDALNY_YOLO_CISZA -> YOLO znowu liczone tutaj.
ZDALNY_YOLO_ROZMIAR = 640        # dluzszy bok klatki dla laptopa (= najwieksza skala YOLO, wiec wynik ten sam)
ZDALNY_YOLO_CISZA = 0.5          # s
ZDALNY_YOLO_CZEKAJ = 0.15        # s: petla czeka tyle na swiezy wynik (1 obieg = 1 nowe wykrycie, jak lokalnie)
# nr = ostatnia wystawiona klatka, wydany = ostatnia zabrana przez laptopa (dwa watki laptopa = rozne klatki),
# wynik_nr = klatka, z ktorej jest wynik, oddany_nr = wynik juz podany petli
_zdalny = {"jpg": None, "nr": 0, "wydany": 0, "skala": 1.0, "wynik": [], "wynik_nr": 0, "oddany_nr": 0,
           "pyta": 0.0, "t": 0.0, "kl_s": 0.0}
_zdalny_nowa = threading.Condition()


def _yolo_dla_laptopa(klatka):
    """Wystaw klatke dla laptopa i zwroc najswiezsze wykrycia od niego (z klatki sprzed 1-2 obiegow)."""
    import cv2

    h, w = klatka.shape[:2]
    s = min(1.0, ZDALNY_YOLO_ROZMIAR / max(h, w))
    maly = cv2.resize(klatka, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA) if s < 1 else klatka
    ok, jpg = cv2.imencode(".jpg", maly, [cv2.IMWRITE_JPEG_QUALITY, 75])
    with _zdalny_nowa:
        if ok:
            _zdalny.update(jpg=jpg.tobytes(), nr=_zdalny["nr"] + 1, skala=s)
            _zdalny_nowa.notify_all()
        # sledzenie liczy predkosc z kolejnych wykryc - to samo wykrycie kilka razy pod rzad by je oszukalo
        _zdalny_nowa.wait_for(lambda: _zdalny["wynik_nr"] > _zdalny["oddany_nr"], timeout=ZDALNY_YOLO_CZEKAJ)
        _zdalny["oddany_nr"] = _zdalny["wynik_nr"]
        if time.time() - _zdalny["t"] > 0.5:
            return []  # wynik sprzed przerwy (np. sprzed sledzenia) - butelki moga byc juz gdzie indziej
        return [dict(b) for b in _zdalny["wynik"]]  # kopie: petla dopisuje do nich "ean"


def _yolo_od_laptopa(tresc, wynik_nr):
    """Wykrycia od laptopa (we wspolrzednych pomniejszonej klatki) -> piksele pelnej klatki."""
    _zdalny["pyta"] = time.time()
    if not tresc or wynik_nr <= _zdalny["wynik_nr"]:  # pusty albo spozniony (drugi watek byl szybszy)
        return
    try:
        butelki = json.loads(tresc)
        k = 1.0 / _zdalny["skala"]
        wynik = [{"cx": float(b["cx"]) * k, "cy": float(b["cy"]) * k, "w": float(b["w"]) * k, "h": float(b["h"]) * k,
                  "pewnosc": float(b["pewnosc"]), "klasa": b["klasa"], "ean": None,
                  "box": tuple(int(v * k) for v in b["box"])}
                 for b in butelki if b.get("klasa") in ("butelka", "puszka")]
    except (ValueError, KeyError, TypeError):
        return
    teraz = time.time()
    with _zdalny_nowa:
        if wynik_nr <= _zdalny["wynik_nr"]:
            return
        if _zdalny["t"]:
            _zdalny["kl_s"] = round(0.8 * _zdalny["kl_s"] + 0.2 / max(teraz - _zdalny["t"], 1e-3), 1)
        _zdalny.update(wynik=wynik, wynik_nr=wynik_nr, t=teraz)
        _zdalny_nowa.notify_all()


def _yolo_klatka():
    """Czekaj (max 1 s) na klatke, ktorej zaden watek laptopa jeszcze nie wzial. (jpg, nr) albo (None, 0)."""
    with _zdalny_nowa:
        for _ in range(5):  # czekajacy laptop tez jest "obecny" (przy sledzeniu Pi nie wystawia klatek)
            _zdalny["pyta"] = time.time()
            if _zdalny_nowa.wait_for(lambda: _zdalny["nr"] > _zdalny["wydany"] and _zdalny["jpg"], timeout=0.2):
                _zdalny["wydany"] = _zdalny["nr"]
                return _zdalny["jpg"], _zdalny["nr"]
        return None, 0


def yolo_laptop_aktywny():
    return time.time() - _zdalny["pyta"] < ZDALNY_YOLO_CISZA


class DetektorButelek:
    """Butelki (i puszki) w klatce: [{"cx","cy","w","h","pewnosc","klasa","box"}] w pikselach.

    Domyslnie YOLO11n przez onnxruntime; gdy go brak albo sie nie uda - SSD MobileNet przez OpenCV.
    """

    def __init__(self):
        self.yolo = None
        if DETEKTOR == "yolo":
            try:
                import onnxruntime as ort

                opcje = ort.SessionOptions()
                opcje.intra_op_num_threads = max(1, min(4, (os.cpu_count() or 4) // 2))
                # watki onnxruntime domyslnie kreca sie w kolko miedzy klatkami (100% rdzenia za nic) -
                # na Raspberry bez wentylatora to przegrzanie i zbicie zegara z 2.4 do 1.5 GHz
                opcje.add_session_config_entry("session.intra_op.allow_spinning", "0")
                opcje.add_session_config_entry("session.inter_op.allow_spinning", "0")
                self.yolo = ort.InferenceSession(pobierz_yolo(), opcje, providers=["CPUExecutionProvider"])
                self.wejscie = self.yolo.get_inputs()[0].name
                print("Wykrywanie butelek: YOLO11n")
            except Exception as e:
                print(f"YOLO niedostepne ({e}) - uzywam SSD. Doinstaluj: pip install onnxruntime")
        if self.yolo is None:
            import cv2

            try:
                pb, pbtxt = pobierz_model()
            except Exception as e:
                raise SO101Error(f"nie moge pobrac modelu butelek ({e}) - sprawdz internet") from e
            self.net = cv2.dnn.readNetFromTensorflow(pb, pbtxt)
            print("Wykrywanie butelek: SSD MobileNet")

    @staticmethod
    def _wpis(x0, y0, x1, y1, pewnosc, klasa, w, h):
        x0, y0, x1, y1 = max(0.0, x0), max(0.0, y0), min(float(w), x1), min(float(h), y1)
        if x1 - x0 < 4 or y1 - y0 < 4:
            return None
        return {"cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2, "w": x1 - x0, "h": y1 - y0,
                "pewnosc": float(pewnosc), "klasa": klasa, "box": (int(x0), int(y0), int(x1), int(y1)), "ean": None}

    def _wykryj_yolo(self, klatka, rozmiary=None):
        """YOLO w kilku skalach (bliska butelka lepiej wychodzi w malej, daleka w duzej) + wspolne NMS.

        rozmiary = tylko te skale, wszystkie swieze w tej klatce (sledzenie duzej butelki - bez wynikow z pamieci).
        """
        import cv2
        import numpy as np

        h, w = klatka.shape[:2]
        prog = min(BUTELKA_PROG, BUTELKA_PROG_TRZYMAJ)
        ramki, oceny, klasy = [], [], []
        if rozmiary:
            skale = list(rozmiary)
            self._pamiec = {}  # po powrocie do przeplotu nie mieszaj ze starymi ramkami
        elif YOLO_PRZEPLOT and len(YOLO_ROZMIARY) > 1:
            # w tej klatce jedna skala, pozostale z pamieci (z poprzednich klatek - przesuniecie znikome)
            self._nr = (getattr(self, "_nr", -1) + 1) % len(YOLO_ROZMIARY)
            skale = [YOLO_ROZMIARY[self._nr]]
            if not hasattr(self, "_pamiec"):
                self._pamiec = {}
            for rozm, (r_, o_, k_) in self._pamiec.items():
                if rozm != skale[0]:
                    ramki += r_
                    oceny += o_
                    klasy += k_
        else:
            skale = YOLO_ROZMIARY
        for rozm in skale:
            nowe_r, nowe_o, nowe_k = [], [], []
            s = rozm / max(h, w)
            img = cv2.resize(klatka, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
            ph, pw = (img.shape[0] + 31) // 32 * 32, (img.shape[1] + 31) // 32 * 32  # wielokrotnosci 32
            pad = np.full((ph, pw, 3), 114, np.uint8)
            pad[:img.shape[0], :img.shape[1]] = img
            x = cv2.cvtColor(pad, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None].astype(np.float32) / 255.0
            out = self.yolo.run(None, {self.wejscie: x})[0][0].T  # N x (4 + 80 klas)
            for nr, nazwa in YOLO_KLASY.items():
                sc = out[:, 4 + nr]
                for i in np.where(sc >= prog)[0]:
                    cx, cy, bw, bh = (float(v) / s for v in out[i, :4])
                    nowe_r.append([int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh)])
                    nowe_o.append(float(sc[i]))
                    nowe_k.append(nazwa)
            ramki += nowe_r
            oceny += nowe_o
            klasy += nowe_k
            if YOLO_PRZEPLOT:
                self._pamiec[rozm] = (nowe_r, nowe_o, nowe_k)
        wyniki = []
        if ramki:
            for i in np.array(cv2.dnn.NMSBoxes(ramki, oceny, prog, 0.5)).flatten():
                bx, by, bw, bh = ramki[i]
                wpis = self._wpis(bx, by, bx + bw, by + bh, oceny[i], klasy[i], w, h)
                if wpis:
                    wyniki.append(wpis)
        return wyniki

    def wykryj(self, klatka, sledzi=False, duza=False):
        """sledzi = ramie trzyma butelke (liczy sie kazda klatka: tylko tutaj, bez WiFi);
        duza = sledzona butelka duza w kadrze -> sama mala skala, swieza w kazdej klatce (szybko i bez skokow)."""
        import cv2

        # laptop tylko do wypatrywania butelek: przy sledzeniu opoznienie WiFi (60-400 ms, zmienne) rozbujaloby ramie
        zdalnie = yolo_laptop_aktywny() and not sledzi
        _web["yolo"] = "laptop" if zdalnie else "lokalnie"  # napis na obrazie i w /pokaz
        if zdalnie != getattr(self, "_zdalnie", False):
            self._zdalnie = zdalnie
            if yolo_laptop_aktywny():
                print(f"\nYOLO: {'laptop (szukam butelki)' if zdalnie else 'tutaj (sledze butelke - bez opoznien WiFi)'}")
        if zdalnie:
            return _yolo_dla_laptopa(klatka)
        if self.yolo is not None:
            return self._wykryj_yolo(klatka, (YOLO_SLEDZENIE,) if sledzi and duza else None)
        h, w = klatka.shape[:2]
        self.net.setInput(cv2.dnn.blobFromImage(klatka, size=(300, 300), swapRB=True))
        wyniki = []
        for d in self.net.forward()[0, 0]:
            klasa, pewnosc = int(d[1]), float(d[2])
            if klasa not in SLEDZ_KLASY or pewnosc < min(BUTELKA_PROG, BUTELKA_PROG_TRZYMAJ):
                continue
            # float(): zwykle liczby zamiast numpy.float32 (inaczej zapis historii do JSON sie wywala)
            wpis = self._wpis(float(d[3]) * w, float(d[4]) * h, float(d[5]) * w, float(d[6]) * h, pewnosc,
                              SLEDZ_KLASY[klasa], w, h)
            if wpis:
                wyniki.append(wpis)
        return wyniki


def pelna_ramka(bt, uciete):
    """Szacowana cala butelka, gdy czesc wystaje poza kadr (uciete = {"gora","dol","lewa","prawa"}: bool).

    Bez tego srodek ucietej ramki "ucieka" w strone krawedzi i ramie zjezdza na dno albo szyjke butelki.
    """
    x0, y0, x1, y1 = bt["box"]
    w, h = max(x1 - x0, 1), max(y1 - y0, 1)
    aspekt = SLEDZ_ASPEKT.get(bt["klasa"], 2.5)
    if w > h * 1.2:  # butelka lezy - dluzszy bok w poziomie
        dl = min(max(w, h * aspekt), w * UCIETA_MAX)
        if uciete["lewa"] and not uciete["prawa"]:
            x0 = x1 - dl
        elif uciete["prawa"] and not uciete["lewa"]:
            x1 = x0 + dl
        return x0, y0, x1, y1
    wys_pr = min(max(h, w * aspekt), h * UCIETA_MAX)
    if uciete["gora"] and not uciete["dol"]:
        y0 = y1 - wys_pr
    elif uciete["dol"] and not uciete["gora"]:
        y1 = y0 + wys_pr
    szer_pr = min(max(w, wys_pr / aspekt), w * UCIETA_MAX)
    if uciete["lewa"] and not uciete["prawa"]:
        x0 = x1 - szer_pr
    elif uciete["prawa"] and not uciete["lewa"]:
        x1 = x0 + szer_pr
    return x0, y0, x1, y1


def punkt_celowania(bt, szer, wys, uciete=None, wzgl=None):
    """Punkt na butelce, w ktory celuje ramie: zapamietane miejsce kodu (wzgl) albo srodek etykiety."""
    if uciete is None:
        m = 3
        x0, y0, x1, y1 = bt["box"]
        uciete = {"gora": y0 <= m, "dol": y1 >= wys - m, "lewa": x0 <= m, "prawa": x1 >= szer - m}
    x0, y0, x1, y1 = pelna_ramka(bt, uciete)
    rx, ry = wzgl or (0.5, SLEDZ_CEL_WYSOKOSC)
    px, py = x0 + rx * (x1 - x0), y0 + ry * (y1 - y0)
    if uciete["gora"] and uciete["dol"]:
        py = wys / 2  # butelka wyzsza niz kadr - w pionie stoj
    if uciete["lewa"] and uciete["prawa"]:
        px = szer / 2
    return px, py


class Inspekcja:
    """Automat inspekcji: SZUKAM -> CENTRUJE -> CZYTAM -> WYNIK (KAUCYJNA / BEZ_KAUCJI / BRAK_KODU).

    Wynik: ekran + log + http://<IP>:8765/wynik (+ opcjonalnie POST na DECYZJA_URL).
    BRAK_KODU -> "obroc_butelke": true. Po obrocie: /obrocono -> czyta kod od nowa.
    """

    # punkty rozgladania wzgledem pozycji startowej (x = lewo/prawo, y = gora/dol, w zakresach z ustawien)
    PUNKTY = [(0, 0), (-1, 0), (1, 0), (1, 1), (-1, 1), (-1, -1), (1, -1), (0, 0)]

    def __init__(self, st):
        teraz = time.time()
        self.srodek = {SLEDZ_STAW_POZIOM: st.here[SLEDZ_STAW_POZIOM], SLEDZ_STAW_PION: st.here[SLEDZ_STAW_PION]}
        zapisana = wczytaj(PLIK_POZ, {}).get(POZA_DOMOWA)
        if zapisana and all(abs(zapisana[j] - st.here[j]) < 10 for j in ("lift", "elbow")):
            # po restarcie czekaj tam, gdzie ustawiono M (ramie w tej samej postawie), a nie gdzie akurat stalo
            self.srodek = {j: zapisana[j] for j in (SLEDZ_STAW_POZIOM, SLEDZ_STAW_PION)}
        self.stan, self.od, self.bez_celu_od = "SZUKAM", teraz, teraz
        self.cel, self.wynik, self.wycentrowany_od = None, None, None
        self.punkt, self._t = 0, None
        self.reczny = 0.0  # kiedy ostatnio sterowano recznie (wtedy nie rozgladamy sie)

    def po_obrocie(self):
        """Butelka zostala obrocona - czytaj kod od nowa."""
        if self.cel:
            self.stan, self.od, self.wynik = "CZYTAM", time.time(), None
            stan["inspekcja"] = {"stan": "CZYTAM", "butelka": self.cel}

    def _wynik(self, wynik, kaucja, sled, kaucje):
        kw = kaucje.kwoty.get(sled.ean) if sled.ean else None
        self.wynik = {
            "wynik": wynik, "kaucja": kaucja, "obroc_butelke": wynik == "BRAK_KODU",
            "butelka": self.cel, "kod": sled.ean, "nazwa": kaucje.nazwy.get(sled.ean or "", ""),
            "kwota": float(kw[0]) if kw else None, "waluta": kw[1] if kw else None,
            "czas": time.strftime("%H:%M:%S"),
        }
        self.stan = "WYNIK"
        stan["inspekcja"] = dict(self.wynik, stan="WYNIK")
        stan["ostatni_wynik"] = dict(self.wynik)  # zostaje, nawet gdy butelka zniknie z kadru
        # widok /pokaz: historia zdarzen + ostatni wynik kazdej butelki (po obrocie BRAK_KODU -> KAUCYJNA = 1 butelka)
        stan["historia"] = (stan.get("historia", []) + [dict(self.wynik)])[-50:]
        wyniki = stan.setdefault("wyniki", {})
        wyniki.pop(self.cel, None)
        wyniki[self.cel] = dict(self.wynik)
        while len(wyniki) > 200:
            wyniki.pop(next(iter(wyniki)))
        print(f"\n=== WYNIK: {wynik} {self.wynik['kod'] or ''} {self.wynik['nazwa']} ===")
        _log(f"WYNIK {self.wynik}")
        if DECYZJA_URL:  # w tle - nie blokuje sledzenia
            def wyslij(dane=dict(self.wynik)):
                try:
                    import requests

                    requests.post(DECYZJA_URL, json=dane, timeout=3)
                except Exception as e:
                    print("\nnie wyslalem wyniku na", DECYZJA_URL, e)

            threading.Thread(target=wyslij, daemon=True).start()
        return f"WYNIK: {wynik}"

    def ustaw_czekanie(self, st):
        """Obecna pozycja kamery = pozycja wyczekiwania (tu drugie ramie przynosi butelke)."""
        self.srodek = {SLEDZ_STAW_POZIOM: st.here[SLEDZ_STAW_POZIOM], SLEDZ_STAW_PION: st.here[SLEDZ_STAW_PION]}

    def _rozgladaj(self, st, teraz, tylko_powrot=False):
        """Powolne przeszukiwanie okolicy kamera albo powrot do pozycji wyczekiwania (plynnie)."""
        dt = 0.05 if self._t is None else min(max(teraz - self._t, 0.0), 0.2)
        self._t = teraz
        px, py = self.PUNKTY[self.punkt]
        cel = {SLEDZ_STAW_POZIOM: self.srodek[SLEDZ_STAW_POZIOM] + px * SZUKAJ_ZAKRES_POZIOM,
               SLEDZ_STAW_PION: self.srodek[SLEDZ_STAW_PION] + py * SZUKAJ_ZAKRES_PION}
        dojechal = True
        for j, c in cel.items():
            lo, hi = st.limits[j]
            c = min(max(c, lo), hi)
            d = c - st.target[j]
            if abs(d) > 0.5:
                dojechal = False
                st.target[j] += math.copysign(min(abs(d), SZUKAJ_PREDKOSC * dt), d)
                st.wolne.add(j)
                st.w_sled[j] = math.copysign(SZUKAJ_PREDKOSC, d)
        if dojechal and not tylko_powrot:
            self.punkt = (self.punkt + 1) % len(self.PUNKTY)

    def krok(self, sled, kaucje, st, teraz, aktywna):
        """Jeden krok automatu. Zwraca komunikat albo None."""
        msg = None
        if sled.cel != self.cel:  # nowa butelka albo zgubiona
            self.cel, self.wynik, self.wycentrowany_od = sled.cel, None, None
            self.stan, self.od = ("CENTRUJE" if sled.cel else "SZUKAM"), teraz
            if not sled.cel:
                self.bez_celu_od = teraz
            self._t = None
            stan["inspekcja"] = {"stan": self.stan, "butelka": self.cel}
        if not aktywna:
            return None

        if self.stan == "SZUKAM":
            if teraz - self.reczny < 3.0:
                pass  # sterujesz recznie - nie przeszkadzaj
            elif ROZGLADANIE and teraz - self.bez_celu_od > SZUKAJ_PO:
                self._rozgladaj(st, teraz)
            elif not ROZGLADANIE and teraz - self.bez_celu_od > POWROT_PO:
                self.punkt = 0  # punkt (0, 0) = pozycja wyczekiwania
                self._rozgladaj(st, teraz, tylko_powrot=True)
        elif self.stan == "CENTRUJE":
            f = sled._filtr or sled.blad[:2] if hasattr(sled, "blad") else None
            spokojnie = f is not None and max(abs(f[0]), abs(f[1])) < 0.15 and \
                max(abs(w) for w in sled._w.values()) < 5.0
            if sled.ean and kaucje.sprawdz(sled.ean) is not None:  # kod odczytany juz w trakcie centrowania
                msg = self._wynik("KAUCYJNA" if kaucje.sprawdz(sled.ean) else "BEZ_KAUCJI",
                                  kaucje.sprawdz(sled.ean), sled, kaucje)
            elif spokojnie:
                self.wycentrowany_od = self.wycentrowany_od or teraz
                if teraz - self.wycentrowany_od > WYCENTROWANY_CZAS:
                    self.stan, self.od = "CZYTAM", teraz
                    stan["inspekcja"] = {"stan": "CZYTAM", "butelka": self.cel}
                    msg = "wycentrowana - czytam kod"
            else:
                self.wycentrowany_od = None
        elif self.stan == "CZYTAM":
            if sled.ean:
                kaucja = kaucje.sprawdz(sled.ean)
                if kaucja is not None:
                    msg = self._wynik("KAUCYJNA" if kaucja else "BEZ_KAUCJI", kaucja, sled, kaucje)
            elif teraz - self.od > KOD_CZAS:
                msg = self._wynik("BRAK_KODU", None, sled, kaucje)
        elif self.stan == "WYNIK" and self.wynik and self.wynik["wynik"] == "BRAK_KODU" and sled.ean:
            self.stan, self.od = "CZYTAM", teraz  # kod pojawil sie pozniej (np. po obrocie) - rozstrzygnij
        return msg

    def napis(self, teraz):
        """(tekst, kolor) do duzego napisu na ekranie."""
        if self.stan == "SZUKAM":
            if ROZGLADANIE and teraz - self.bez_celu_od > SZUKAJ_PO:
                return "SZUKAM BUTELKI - rozgladam sie...", (200, 200, 200)
            return "CZEKAM NA BUTELKE...", (200, 200, 200)
        if self.stan == "CENTRUJE":
            return "CENTRUJE BUTELKE...", (0, 220, 255)
        if self.stan == "CZYTAM":
            return f"CZYTAM KOD... {max(0.0, KOD_CZAS - (teraz - self.od)):.1f} s", (0, 220, 255)
        w = self.wynik or {}
        if w.get("wynik") == "KAUCYJNA":
            kw = f" {w['kwota']:.2f} {w['waluta']}" if w.get("kwota") is not None else ""
            return f"KAUCYJNA{kw}  {w.get('nazwa', '')}", (0, 200, 0)
        if w.get("wynik") == "BEZ_KAUCJI":
            return f"BEZ KAUCJI ({w.get('kod')})", (0, 0, 230)
        return "BRAK KODU - OBROC BUTELKE", (0, 140, 255)


_inspekcja = None


class SledzenieButelek(Sledzenie):
    """Trzyma najblizsza (najwieksza) butelke na srodku obrazu.

    Lapie ja dopiero po SLEDZ_POTWIERDZ klatkach, puszcza gdy za daleko albo zgubiona,
    przelacza na inna tylko gdy ta jest wyraznie blizej przez kilka klatek.
    Sam ruch ramienia (popraw-i-poczekaj, nauka czulosci, szukanie) dziedziczy z Sledzenie.
    """

    def __init__(self):
        super().__init__()
        self.box = None
        self.ean = None
        self._nr = 0
        self._kand, self._kand_n = None, 0
        self._inny_n = 0

    @staticmethod
    def _pole(b):
        # butelka ma pierwszenstwo przed "puszka" (siec tak nazywa tez kubki); potem wieksza = blizej
        return (b["klasa"] == "butelka", b["w"] * b["h"])

    def _blisko(self, b, cx, cy, szer):
        return abs(b["cx"] - cx) + abs(b["cy"] - cy) < SLEDZ_PROMIEN * szer

    @staticmethod
    def _nakladaja(a, b):
        """Dwie ramki jednej butelki (np. cala + sam kawalek): srodek jednej w drugiej albo wiekszosc wspolna."""
        ax0, ay0, ax1, ay1 = a["box"]
        bx0, by0, bx1, by1 = b["box"]
        if (ax0 <= b["cx"] <= ax1 and ay0 <= b["cy"] <= ay1) or (bx0 <= a["cx"] <= bx1 and by0 <= a["cy"] <= by1):
            return True
        wspolne = max(0, min(ax1, bx1) - max(ax0, bx0)) * max(0, min(ay1, by1) - max(ay0, by0))
        return wspolne > 0.5 * min((ax1 - ax0) * (ay1 - ay0), (bx1 - bx0) * (by1 - by0))

    def krok(self, butelki, kaucje, szer, wys, st, t_klatki=None):
        """t_klatki = kiedy kamera zrobila te klatke (dokladne opoznienie do przewidywania ruchu)."""
        teraz = time.time()
        if t_klatki:
            self.zmierz_opoznienie(t_klatki, teraz)
        msg = None

        def pasuje(b):
            prog = BUTELKA_PROG if b["klasa"] == "butelka" else PUSZKA_PROG
            if b["pewnosc"] < prog or b["h"] < MIN_BUTELKA * wys:
                return False
            return not SLEDZ_TYLKO_KAUCJA or (b["ean"] and kaucje.sprawdz(b["ean"]))

        dobre = [b for b in butelki if pasuje(b)]
        najwieksza = max(dobre, key=self._pole, default=None)
        biezacy = None
        if self.cel is not None and self.xy:
            bliskie = [b for b in butelki if self._blisko(b, self.xy[0], self.xy[1], szer)]
            biezacy = min(bliskie, key=lambda b: abs(b["cx"] - self.xy[0]) + abs(b["cy"] - self.xy[1]),
                          default=None)
            if biezacy is not None:  # ta sama butelka wykryta 2 razy (cala + kawalek) -> zawsze cala, bez skakania
                biezacy = max((b for b in butelki if self._nakladaja(b, biezacy)), key=lambda b: b["w"] * b["h"])

        if self.cel is None:
            if najwieksza:  # potwierdz w kilku klatkach - jedno falszywe wykrycie nie rusza ramieniem
                if self._kand and self._blisko(najwieksza, self._kand[0], self._kand[1], szer):
                    self._kand_n += 1
                else:
                    self._kand_n = 1
                self._kand = (najwieksza["cx"], najwieksza["cy"])
                if self._kand_n >= SLEDZ_POTWIERDZ:
                    self._nr += 1
                    self.cel, biezacy, self.ean = f"butelka{self._nr}", najwieksza, None
                    self._inny_n = 0
                    msg = f"SLEDZE {self.cel}"
            else:
                self._kand, self._kand_n = None, 0
        elif biezacy is not None:
            if biezacy["h"] < MIN_BUTELKA * wys:
                msg, self.cel, biezacy = f"{self.cel} ODDALILA SIE - puszczam", None, None
            elif najwieksza is not None and najwieksza is not biezacy and not self._nakladaja(najwieksza, biezacy) and (
                    (najwieksza["klasa"] == "butelka") > (biezacy["klasa"] == "butelka")
                    or (najwieksza["klasa"] == biezacy["klasa"]
                        and self._pole(najwieksza)[1] > self._pole(biezacy)[1] * PRZELACZ_GDY)):
                self._inny_n += 1
                if self._inny_n >= SLEDZ_PRZELACZ_KLATEK:
                    self._nr += 1
                    stary = self.cel
                    self.cel, biezacy, self.ean = f"butelka{self._nr}", najwieksza, None
                    self._inny_n = 0  # bez tego przelaczal sie znowu juz w nastepnej klatce (serie INNA BLIZEJ)
                    msg = f"INNA BLIZEJ: {stary} -> {self.cel}"
            else:
                self._inny_n = 0
        elif teraz - self.widziany > ZGUBIONY_PO:
            msg, self.cel = f"ZGUBIONA {self.cel} - puszczam", None

        if self.cel is None:
            self.box = None
            return msg
        if biezacy is None:
            if not msg:
                self._szukaj(st, teraz)
            return msg

        self.widziany, self.rozmiar = teraz, biezacy["h"] / wys
        self.xy = (biezacy["cx"], biezacy["cy"])
        self.box = biezacy["box"]
        if biezacy["ean"]:
            self.ean = biezacy["ean"]
        self.zrodlo = biezacy["klasa"]
        # krawedzie kadru z histereza (wlacza sie przy 3 px, gasnie od 25 px) - bez migotania
        x0, y0, x1, y1 = biezacy["box"]
        odl = {"gora": y0, "dol": wys - y1, "lewa": x0, "prawa": szer - x1}
        for kr, d in odl.items():
            self._uciete[kr] = d <= 3 if not self._uciete[kr] else d < 25
        # gdzie na butelce jest kod: zapamietaj (wzgledem pelnej ramki) i celuj tam stale - bez przeskakiwania
        # miedzy "kod odczytany" a "srodek etykiety"
        if biezacy.get("kod_xy"):
            f0, g0, f1, g1 = pelna_ramka(biezacy, self._uciete)
            rel = ((biezacy["kod_xy"][0] - f0) / max(f1 - f0, 1), (biezacy["kod_xy"][1] - g0) / max(g1 - g0, 1))
            if 0.0 <= rel[0] <= 1.0 and 0.0 <= rel[1] <= 1.0:
                k = self.kod_wzgl
                self.kod_wzgl = rel if k is None else (k[0] + 0.3 * (rel[0] - k[0]), k[1] + 0.3 * (rel[1] - k[1]))
        px, py = punkt_celowania(biezacy, szer, wys, self._uciete, self.kod_wzgl if SLEDZ_CEL_W_KOD else None)
        # butelka tuz przed kamera (wiekszosc wysokosci kadru): etykieta i tak jest w kadrze, a zgadywany srodek
        # ucietej butelki skacze (bylo: nadgarstek -12 -> +12 st/s) - w pionie stoj, centruj tylko w poziomie
        h_rel = biezacy["h"] / wys
        self._bliska = h_rel >= (BLISKA_OD - 0.12 if getattr(self, "_bliska", False) else BLISKA_OD)
        if self._bliska:
            py = wys / 2
        self.punkt = (px, py)
        self.blad = ((px - szer / 2) / (szer / 2), (py - wys / 2) / (wys / 2), True)
        if msg:
            self._reset()
        self._hist = [h for h in self._hist if teraz - h[0] < 0.6] + [(teraz, self.blad[0], self.blad[1])]
        self._szukano = 0.0
        return self._steruj(st, msg, teraz)


PANEL_KLAWISZY = [
    ("RUCH", None),
    ("W / S", "GORA / DOL"), ("R / F", "DALEJ / BLIZEJ"), ("A / D", "LEWO / PRAWO"),
    ("J / L", "pochyl chwytak"), ("U / O", "obroc chwytak"), ("1 2 3", "predkosc"), ("B", "STOP"),
    ("CHWYTAK", None),
    ("SPACJA", "chwyc / pusc"), ("Z / X", "otworz / zamknij troche"),
    ("KAMERA", None),
    ("T", "sledzenie wl / wyl"), ("ENTER", "ogladaj puszke (objazd)"),
    ("POZY", None),
    ("P", "zapisz poze objazdu"), ("C", "usun pozy objazdu"), ("M", "tu czekaj na butelke"), ("H", "jedz do domu"),
    ("", None),
    ("Y", "zapisz historie butelek"), ("TAB", "schowaj / pokaz pomoc"), ("Q / ESC", "koniec"),
]

# suwaki w oknie "Ustawienia": (napis, zmienna, min, max, mnoznik)
SUWAKI = [
    ("sledzenie 0/1", None, 0, 1, 1),
    ("plynnie 0/1", "SLEDZ_PLYNNIE", 0, 1, 1),
    ("reakcja x10", "SLEDZ_PETLA", 3, 40, 0.1),
    ("max predkosc", "SLEDZ_MAX_V", 3, 60, 1),
    ("przewidywanie %", "SLEDZ_FF", 0, 100, 0.01),
    ("krok % (krokowy)", "SLEDZ_KROK", 10, 100, 0.01),
    ("pauza ms", "SLEDZ_PAUZA", 200, 2000, 0.001),
    ("predkosc st/s", "SLEDZ_PREDKOSC", 5, 60, 1),
    ("strefa celu %", "SLEDZ_CEL", 3, 30, 0.01),
    ("start ruchu %", "SLEDZ_START", 5, 40, 0.01),
    ("min butelka %", "MIN_BUTELKA", 3, 60, 0.01),
    ("pewnosc %", "BUTELKA_PROG", 20, 90, 0.01),
    ("szukanie max st", "SLEDZ_SZUKAJ_MAX", 0, 40, 1),
    ("tylko kaucja 0/1", "SLEDZ_TYLKO_KAUCJA", 0, 1, 1),
]
OKNO_USTAWIEN = "Ustawienia (suwaki dzialaja na zywo)"


def rysuj_panel(klatka):
    """Polprzezroczysta tabliczka z klawiszami po prawej stronie obrazu."""
    import cv2

    h, w = klatka.shape[:2]
    x0 = w - 400
    kawalek = klatka[36:h - 50, x0:w]
    cv2.addWeighted(kawalek, 0.35, kawalek * 0, 0.65, 0, dst=kawalek)
    y = 64
    for klawisz, opis in PANEL_KLAWISZY:
        if opis is None:
            if klawisz:
                cv2.putText(klatka, klawisz, (x0 + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
            y += 26 if klawisz else 10
            continue
        cv2.putText(klatka, klawisz, (x0 + 18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        cv2.putText(klatka, opis, (x0 + 125, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (210, 210, 210), 1)
        y += 25


def utworz_suwaki(sledz):
    import cv2

    cv2.namedWindow(OKNO_USTAWIEN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(OKNO_USTAWIEN, 460, 420)
    for napis, zmienna, lo, hi, mn in SUWAKI:
        wartosc = int(sledz) if zmienna is None else int(round(float(globals()[zmienna]) / mn))
        cv2.createTrackbar(napis, OKNO_USTAWIEN, min(max(wartosc, lo), hi), hi, lambda _v: None)
        cv2.setTrackbarMin(napis, OKNO_USTAWIEN, lo)


def czytaj_suwaki(sledz):
    """Przepisz suwaki do ustawien. Zwraca stan wlacznika sledzenia (albo sledz, gdy okno zamkniete)."""
    import cv2

    try:
        for napis, zmienna, lo, hi, mn in SUWAKI:
            v = cv2.getTrackbarPos(napis, OKNO_USTAWIEN)
            if v < 0:
                return sledz
            if zmienna is None:
                sledz = bool(v)
            else:
                globals()[zmienna] = bool(v) if zmienna in ("SLEDZ_TYLKO_KAUCJA", "SLEDZ_PLYNNIE") else v * mn
    except cv2.error:  # okno ustawien zamkniete - zostaja ostatnie wartosci
        pass
    return sledz


def tryb_kamera(port=None, mock=False):
    """Glowny program: okno kamery + sterowanie reczne + ogladanie puszki na sygnal."""
    import cv2

    arm = polacz(port, mock)
    st = Sterownik(arm)
    kam = KameraWatek(otworz_kamere())
    kaucje, sled = Kaucje(), SledzenieButelek()
    historia = TrackingHistory()
    historia.load()
    inspekcja = Inspekcja(st)
    globals()["_inspekcja"] = inspekcja
    detektor = DetektorButelek()
    czytnik = CzytnikKodow()
    kody, cel_od, poprz_cel = [], 0.0, None
    sledz = SLEDZ and "--bez-sledzenia" not in sys.argv  # --bez-sledzenia: ramie rusza sie tylko od klawiszy
    t_klatki = time.time()
    srv = serwer_http(arm)
    print(f"Pad: {'PODLACZONY' if st.pad_ok else 'brak (tylko klawiatura)'}")
    print(f"GOTOWY. Sygnal od RoArma: http://{moje_ip()}:{PORT_HTTP}/skanuj")
    tytul = "Ramie SO-101"
    panel = f"http://{moje_ip()}:{PORT_HTTP}/"
    print(f"Widok na prezentacje: {panel}pokaz")
    if BEZ_OKNA:
        print(f"Bez okna - panel w przegladarce: {panel}")
        pomoc = False  # klawisze sa na stronie
    else:
        print("Klikni w okno kamery i steruj:\n" + POMOC)
        cv2.namedWindow(tytul, cv2.WINDOW_NORMAL)  # okno mozna zmniejszac/powiekszac
        cv2.resizeWindow(tytul, 960, 540)
        pomoc = True
        utworz_suwaki(sledz)
    komunikat, komunikat_do = "", 0.0
    byl_zajety = False
    t_web = t_czysty = 0.0
    nr_klatki, ostr = 0, 0.0
    szer_ekranu = 960 if BEZ_OKNA else 1280  # bez okna obraz idzie tylko do przegladarki (i tak 960 px)

    def pokaz(tekst):
        nonlocal komunikat, komunikat_do
        komunikat, komunikat_do = tekst, time.time() + 3
        print(f"\n{tekst}")

    try:
        while True:
            ok, klatka, t_zdjecia = kam.read()  # najnowsza klatka + kiedy przyszla z kamery
            if not ok:
                continue
            teraz = time.time()
            if teraz > t_klatki:  # kl/s petli (wygladzone) - na obrazie i w /status
                stan["kl_s"] = round(0.9 * stan.get("kl_s", 0.0) + 0.1 / (teraz - t_klatki), 1)
            dt, t_klatki = min(teraz - t_klatki, 0.2), teraz
            wys, szer = klatka.shape[:2]
            nr_klatki += 1
            if nr_klatki % 5 == 1:  # ostrosc tylko do napisu na ekranie - nie trzeba co klatke
                ostr = ostrosc(klatka)
            butelki = detektor.wykryj(klatka, sledzi=bool(sled.cel), duza=sled.rozmiar >= YOLO_DUZA_OD)
            # dla RoArma (butelki.py): gdzie w kadrze stoja butelki - /butelki
            _web["widok"] = {"szer": szer, "wys": wys, "t": t_zdjecia, "butelki": [
                {"klasa": b["klasa"], "pewnosc": round(b["pewnosc"], 3), "cx": round(b["cx"], 1),
                 "cy": round(b["cy"], 1), "box": [int(v) for v in b["box"]]}
                for b in butelki if b["pewnosc"] >= (BUTELKA_PROG if b["klasa"] == "butelka" else PUSZKA_PROG)]}
            czytnik.pauza = inspekcja.stan == "WYNIK" and bool(inspekcja.wynik) and inspekcja.wynik["kod"] is not None
            if czytnik.potrzebna_klatka():
                czytnik.podaj(klatka.copy(), sled.cel, sled.box)  # kody czytane w tle, na kopii klatki
            kody = czytnik.kody
            if sled.cel and sled.cel in czytnik.kod_celu:
                sled.ean = czytnik.kod_celu[sled.cel]
            # przetwarzanie w pelnej rozdzielczosci, a wyswietlanie na mniejszym obrazie
            sk = szer_ekranu / szer
            ekran = cv2.resize(klatka, (szer_ekranu, int(round(wys * sk))), interpolation=cv2.INTER_AREA) \
                if sk < 1 else klatka.copy()
            sk = min(sk, 1.0)
            ew, eh = ekran.shape[1], ekran.shape[0]
            # kod w srodku butelki = to jej kod (kaucja)
            for bt in butelki:  # (tylko potwierdzone kody - pojedynczy odczyt z calego obrazu bywa bledny)
                x0, y0, x1, y1 = bt["box"]
                kod = next((k for k in kody if k["ean"] in czytnik.pewne and x0 <= k["cx"] <= x1 and y0 <= k["cy"] <= y1),
                           None)
                bt["ean"] = kod["ean"] if kod else None
                bt["kod_xy"] = (kod["cx"], kod["cy"]) if kod else None
            for k in kody:
                if not k["ean"]:
                    continue
                kaucja = kaucje.sprawdz(k["ean"])
                kolor = (0, 200, 0) if kaucja else ((0, 0, 230) if kaucja is False else (0, 220, 255))
                rogi = (k["rogi"] * sk).astype(int)
                cv2.polylines(ekran, [rogi], True, kolor, 2)
                cv2.putText(ekran, k["ean"], tuple(rogi[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, kolor, 1)
                if _zajety.is_set() and not stan["kod"]:
                    stan["kod"] = k["ean"]
            for bt in butelki:
                x0, y0, x1, y1 = (int(v * sk) for v in bt["box"])
                cel = bool(sled.cel and sled.xy and abs(bt["cx"] - sled.xy[0]) + abs(bt["cy"] - sled.xy[1]) < 0.05 * szer)
                if bt["pewnosc"] < (BUTELKA_PROG if bt["klasa"] == "butelka" else PUSZKA_PROG) and not cel:
                    continue  # slabe wykrycie, ktore nie jest celem - nie zasmiecaj obrazu
                ean = bt["ean"] or (sled.ean if cel else None)
                kaucja = kaucje.sprawdz(ean) if ean else None
                # zielona = kaucja, czerwona = bez kaucji, zolta = nie wiadomo (kod nieodczytany)
                kolor = (0, 200, 0) if kaucja else ((0, 0, 230) if kaucja is False else (0, 220, 255))
                cv2.rectangle(ekran, (x0, y0), (x1, y1), kolor, 6 if cel else 2)
                if cel and getattr(sled, "punkt", None):  # tu celuje ramie
                    cv2.drawMarker(ekran, (int(sled.punkt[0] * sk), int(sled.punkt[1] * sk)), kolor,
                                   cv2.MARKER_TILTED_CROSS, 30, 3)
                opis = f"{bt['klasa']} {bt['pewnosc']:.0%}" + (" KAUCJA" if kaucja else "") + ("  <- CEL" if cel else "")
                cv2.putText(ekran, opis, (x0 + 4, max(y0 + 22, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, kolor, 2)
            # widok /pokaz: obraz z samymi ramkami (bez paskow z liczbami) + lista wykryc
            _web["wykrycia"] = [{"klasa": b["klasa"], "pewnosc": round(b["pewnosc"], 2)} for b in butelki
                                if b["pewnosc"] >= (BUTELKA_PROG if b["klasa"] == "butelka" else PUSZKA_PROG)]
            if _web["widzowie_czysty"] > 0 and teraz - t_czysty >= 1 / WEB_FPS:
                t_czysty = teraz
                web_klatka(ekran, czysty=True)

            # duzy napis: etap inspekcji i wynik (kaucja / bez / obroc butelke)
            if sledz:
                napis, kolor = inspekcja.napis(teraz)
                (tw, th), _ = cv2.getTextSize(napis, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                cv2.rectangle(ekran, (0, 32), (tw + 20, 32 + th + 20), (0, 0, 0), -1)
                cv2.putText(ekran, napis, (10, 32 + th + 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, kolor, 2)

            # sterowanie reczne tylko, gdy nie trwa automatyczny ruch
            if _zajety.is_set():
                byl_zajety = True
            else:
                if byl_zajety:
                    st.po_zadaniu()
                    byl_zajety = False
                if sledz:
                    mial_cel = sled.cel is not None
                    klucz_przed = sled.ean or sled.cel
                    msg = sled.krok(butelki, kaucje, szer, wys, st, t_klatki=t_zdjecia)
                    # historia: swiezy pomiar sledzonej butelki (klucz = EAN, a bez kodu - numer butelki)
                    if sled.cel and sled.xy and teraz - sled.widziany < 0.05:
                        historia.update(sled.ean or sled.cel, kaucje.nazwy.get(sled.ean or "", ""), teraz,
                                        sled.xy[0], sled.xy[1], sled.rozmiar, st.here["pan"], st.here["wflex"])
                    if mial_cel and sled.cel is None and klucz_przed:
                        historia.mark_lost(klucz_przed)
                    historia.cleanup(teraz)
                    if msg:
                        pokaz(msg)
                        _log(msg)
                    if sled.cel and teraz - getattr(sled, "_log", 0) > 0.3 and hasattr(sled, "blad"):
                        sled._log = teraz
                        ex, ey, odczyt = sled.blad
                        print(f"  {sled.cel} x={ex:+.2f} y={ey:+.2f}"
                              f" | pan={st.here['pan']:6.1f} wflex={st.here['wflex']:6.1f}"
                              f" | cel pan={st.target['pan']:6.1f} wflex={st.target['wflex']:6.1f}")
                    if mial_cel and sled.cel is None and PO_UTRACIE == "dom":
                        zadanie(arm, do_domu, "dom")
                msg_i = inspekcja.krok(sled, kaucje, st, teraz, sledz and not _zajety.is_set())
                if msg_i:
                    pokaz(msg_i)
                pressed = st.krok()
                # dla RoArma: czy kamera stoi (obraz = mapa stolu tylko w nieruchomej pozycji patrzenia)
                poprz = _web.get("stawy")
                _web["stawy"] = dict(st.here)
                _web["ruch_t"] = teraz if not poprz or any(abs(st.here[j] - poprz[j]) > 0.4 for j in MOVE_JOINTS) \
                    else _web.get("ruch_t", 0.0)
                if "y" in pressed or "start" in pressed:
                    zadanie(arm, ogladaj_puszke, "skan")
                if "back" in pressed:
                    break

            wejscie = []
            nowy = sledz
            if not BEZ_OKNA:
                k = cv2.waitKey(1) & 0xFF
                nowy = czytaj_suwaki(sledz)
                if k != 255:
                    wejscie.append("\r" if k in (10, 13) else ("\x1b" if k == 27 else chr(k).lower()))
            while not _web_klawisze.empty():  # klawisze i wlacznik sledzenia z panelu w przegladarce
                c = _web_klawisze.get()
                if c.startswith("sledz"):
                    nowy = c == "sledz1"
                elif c not in ("q", "\x1b"):  # z przegladarki nie wylaczamy programu
                    wejscie.append(c)
            if nowy != sledz:
                sledz, sled.cel = nowy, None
                if not BEZ_OKNA:  # wlacznik z przegladarki -> suwak w oknie (inaczej suwak przelaczy z powrotem)
                    try:
                        cv2.setTrackbarPos("sledzenie 0/1", OKNO_USTAWIEN, int(sledz))
                    except cv2.error:
                        pass
                pokaz(f"sledzenie {'WLACZONE' if sledz else 'WYLACZONE'}")
            wejscie += [c for c in read_keys() if c == "\r"]  # Enter tez z terminala
            for c in wejscie:
                if c in ("q", "\x1b"):
                    raise KeyboardInterrupt
                if c == "\r":
                    zadanie(arm, ogladaj_puszke, "skan") or pokaz("ramie zajete")
                elif _zajety.is_set():
                    continue
                elif c == "h":
                    zadanie(arm, do_domu, "dom") or pokaz("ramie zajete")
                elif c == "t":
                    sledz, sled.cel = not sledz, None
                    if not BEZ_OKNA:
                        try:
                            cv2.setTrackbarPos("sledzenie 0/1", OKNO_USTAWIEN, int(sledz))
                        except cv2.error:
                            pass
                    pokaz(f"sledzenie {'WLACZONE' if sledz else 'WYLACZONE'}")
                elif c == "\t":
                    pomoc = not pomoc
                elif c == "m":
                    inspekcja.ustaw_czekanie(st)
                    pokaz(zapisz_poze(arm, POZA_DOMOWA) + " + tu kamera czeka na butelke")
                elif c == "p":
                    pozy = wczytaj(PLIK_POZ, {})
                    wolne = [p for p in POZY_SKANU if p not in pozy]
                    pokaz(zapisz_poze(arm, wolne[0]) if wolne else "sa juz 4 pozy skanu - R usuwa")
                elif c == "c":
                    pozy = wczytaj(PLIK_POZ, {})
                    for p in POZY_SKANU:
                        pozy.pop(p, None)
                    zapisz(PLIK_POZ, pozy)
                    pokaz("usunieto pozy skanu - bedzie rozgladanie")
                elif c == "y":
                    pokaz(f"zapisano historie ({len(historia.items)} butelek)" if historia.save()
                          else "blad zapisu historii")
                elif st.klawisz(c):
                    inspekcja.reczny = time.time()  # sterujesz recznie - nie rozgladaj sie przez 3 s

            # napisy na obrazie
            if st.komunikat:
                pokaz(st.komunikat)
                st.komunikat = ""
            gora = stan["stan"] + (f" | SLEDZE {sled.cel}" if sled.cel else (" | sledzenie wl." if sledz else ""))
            gora += f" | {stan.get('kl_s', 0):.0f} kl/s"
            gora += f" | YOLO laptop {_zdalny['kl_s']:.0f}/s" if _web.get("yolo") == "laptop" else ""
            gora += f" | ostrosc {ostr:.0f}" + (" (NIEOSTRO - krec obiektywem)" if ostr < 60 else "")
            if stan["wynik"] and not _zajety.is_set():
                w = stan["wynik"]
                gora += f" | ostatni: {w.get('kod') or w.get('blad') or 'brak kodu'}"
                gora += " (KAUCJA)" if w.get("kaucja") else ""
            h, w_ = eh, ew
            cv2.drawMarker(ekran, (w_ // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
            cv2.rectangle(ekran, (0, 0), (w_, 28), (0, 0, 0), -1)
            cv2.putText(ekran, gora, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.rectangle(ekran, (0, h - 46), (w_, h), (0, 0, 0), -1)
            cv2.putText(ekran, st.opis()[:90], (6, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 255, 200), 1)
            stopka = (f"panel: {panel}" if BEZ_OKNA
                      else "TAB = pomoc z klawiszami | suwaki w oknie 'Ustawienia' | Q = koniec")
            cv2.putText(ekran, stopka, (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
            if pomoc:
                rysuj_panel(ekran)
            if komunikat and time.time() < komunikat_do:
                # pod duzym napisem o kaucji (ten zajmuje gore obrazu), zeby sie nie nakladaly
                cv2.putText(ekran, komunikat, (10, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            _web["sledz"] = sledz
            if not BEZ_OKNA:
                cv2.imshow(tytul, ekran)
            if _web["widzowie"] > 0 and teraz - t_web >= 1 / WEB_FPS:  # panel w przegladarce (tez obok okna)
                t_web = teraz
                web_klatka(ekran)
    except KeyboardInterrupt:
        pass
    except SO101Error as e:
        print("\nBLAD:", e)
    finally:
        czytnik.zatrzymaj()
        historia.save()
        print("\nKoniec - ramie trzyma pozycje.")
        srv.shutdown()
        kam.release()
        if not BEZ_OKNA:
            cv2.destroyAllWindows()
        if _zajety.is_set():
            _skan_koniec.wait(timeout=30)
        try:
            arm.hold()
        except SO101Error:
            pass
        arm.close()


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else None
    mock = "--mock" in sys.argv
    try:
        if "--bez-kamery" in sys.argv:
            sterowanie(port=port, mock=mock, http=True)
        else:
            tryb_kamera(port=port, mock=mock)
    except SO101Error as e:
        print("BLAD:", e)
