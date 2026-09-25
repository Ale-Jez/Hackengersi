"""Ramie SO-101 - WSZYSTKO W JEDNYM PLIKU.

  python ramie.py                 (albo przycisk Run w VS Code) okno kamery + SLEDZENIE butelki kaucyjnej:
                             ramie trzyma jej kod na srodku obrazu, puszcza gdy sie oddali
                             albo zgubi, przelacza na inna kaucyjna, jesli jest blizej (T wl/wyl)
  python ramie.py --bez-kamery    sterowanie z terminala + ogladanie puszki pod ENTER, bez kamery
  python ramie.py --mock          bez ramienia (sama kamera, ramie stoi)
  (port adaptera znajduje sie sam; mozna podac np. COM5)

Klawisze (kliknij w terminal; z --kamera w okno kamery):
  W/S ramie przod/tyl    I/K lokiec gora/dol    A/D obrot podstawy
  J/L zgiecie nadgarstka U/O obrot nadgarstka   Z/X chwytak troche otworz/zamknij
  SPACJA chwyc/pusc (zamyka az poczuje opor)    1/2/3 predkosc    F stop
  ENTER  OGLADAJ PUSZKE: ramie objezdza puszke kamera, az zobaczy kod kreskowy
  P zapisz obecna poze jako skan1..skan4 (objazd)   R usun pozy skanu
  M zapisz poze domowa   H jedz do domu   Q/ESC koniec (ramie trzyma pozycje)
Pad Xbox: lewa galka podstawa+ramie, prawa lokiec+obrot, LB/RB nadgarstek,
  A chwyc, B pusc, X stop, Y ogladaj puszke, BACK koniec.
Sygnal od RoArma: http://<IP-laptopa>:8765/skanuj  (odpowiedz JSON z kodem)
Awaryjnie: wyjmij wtyczke zasilacza serw.

Katy w stopniach od srodka zakresu enkodera: (ticks - 2048) * 360 / 4096.
"""

import ctypes
import json
import math
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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


def _mock_write_goal(self, sid, ticks, speed_ticks):
    name = next(n for n, i in self.ids.items() if i == sid)
    self._pose[name] = _to_deg(ticks)


MockSO101._write_goal = _mock_write_goal


# =============================================================================
#  USTAWIENIA - to mozesz zmieniac
# =============================================================================
SPEEDS = [10.0, 25.0, 50.0]      # st./s dla predkosci 1 / 2 / 3
KEY_STEP_TIME = 1 / 30           # trzymany klawisz powtarza sie ~30x/s
MAX_LEAD = 12.0                  # cel moze wyprzedzac prawdziwe ramie max o tyle st.
LIMIT_MARGIN = 3.0               # odstep od limitow katowych serw (st.)
CLAMP_SPEED = 30.0               # st./s przy zamykaniu na przedmiocie
CLAMP_LOAD = 20.0                # % obciazenia = "dotknal przedmiotu"
CLAMP_SQUEEZE = 4.0              # st. dodatkowego docisku po dotknieciu
RELEASE_OPEN = 35.0              # o ile st. otwiera sie chwytak przy puszczeniu
BOOST_P = False                  # P=32 na lift/elbow do wylaczenia zasilania (z 16 opadaja)

KAMERA = None                    # None = sama wybierze (zewnetrzna, a jak nie ma - laptopa)
POZY_SKANU = ["skan1", "skan2", "skan3", "skan4"]
AUTO_RUCHY = [                   # rozgladanie wokol pozycji startowej, gdy brak poz skan1..4
    {"pan": -20}, {"pan": -10}, {"pan": 0}, {"pan": 10}, {"pan": 20},
    {"pan": 0, "wflex": -15}, {"pan": 0, "wflex": 15},
]
PAUZA = 0.8                      # ile s patrzy w kazdej pozycji
PREDKOSC_SKANU = 25              # st./s
PORT_HTTP = 8765                 # sygnal od RoArma: http://<IP>:8765/skanuj
POZA_DOMOWA = "spoczynek"
PLIK_POZ = "poses_so101.json"
PLIK_KAUCJI = "kaucja.json"      # wlasna lista EAN z kaucja (oprocz api.kaucja.pl)

# --- sledzenie butelki kaucyjnej (tryb --kamera, klawisz T wlacza/wylacza) ---
SLEDZ = True                     # startuje wlaczone
# Sledzenie "popraw i poczekaj": jeden spokojny ruch w strone kodu, pauza, nowa klatka, kolejny ruch.
SLEDZ_KROK = 0.5                 # jaka czesc odleglosci do srodka pokonuje jednym ruchem (0.3 spokojnie, 0.8 szybko)
SLEDZ_CZULOSC_POZIOM = 0.12      # start: o ile przesuwa sie kod na 1 st. podstawy (potem program sam sie uczy)
SLEDZ_CZULOSC_PION = 0.10        # start: to samo dla nadgarstka
SLEDZ_CEL = 0.07                 # tak blisko srodka = wycentrowany, ramie staje
SLEDZ_START = 0.16               # wycentrowany rusza sie dopiero, gdy kod ucieknie dalej niz to (bez drgan)
SLEDZ_MIN_KROK = 0.5             # mniejszych ruchow nie robi (st.)
SLEDZ_MAX_KROK = 8.0             # najwiekszy pojedynczy ruch (st.)
SLEDZ_PAUZA = 0.5                # s po ruchu, zanim spojrzy znowu (kamera ma opoznienie)
SLEDZ_PROBKI = 3                 # z ilu klatek mediana pozycji kodu
SLEDZ_SZUKAJ_CZAS = 1.5          # s: po utracie kodu jedzie dalej w strone, w ktora kod uciekal
SLEDZ_SZUKAJ_MAX = 15.0          # st.: najdalej tyle "na slepo" po utracie kodu
SERWO_ACC = 20                   # przyspieszenie serw (mniej = lagodniej; bylo 50)
SLEDZ_LOG = "sledzenie.log"      # zapis przebiegu (do diagnozy); "" = bez zapisu
SLEDZ_ZNAK_POZIOM = -1           # odwroc na -1, jesli ramie UCIEKA od kodu w poziomie
SLEDZ_ZNAK_PION = -1             # odwroc na -1, jesli ucieka w pionie
SLEDZ_STAW_POZIOM = "pan"        # ktorym stawem celowac w poziomie
SLEDZ_STAW_PION = "wflex"        # ktorym stawem celowac w pionie
MIN_SZEROKOSC = 0.08             # kod wezszy niz 8% obrazu = za daleko -> puszcza cel
PRZELACZ_GDY = 1.3               # inna butelka kaucyjna 1.3x wiekszy kod (blizej) -> przelacz
ZGUBIONY_PO = 2.0                # s bez kodu -> cel zgubiony
SLEDZ_PROMIEN = 0.25             # nieodczytany (rozmazany) kod blizej niz 25% obrazu od ostatniej pozycji = ten sam
KAMERA_EKSPOZYCJA = None         # None = auto (najjasniej). Stala: -4 jasno ale rozmywa, -5/-6 mniej rozmycia, trzeba lampki
PO_UTRACIE = "stoj"              # "stoj" albo "dom" (wroc do pozy domowej)

# Odwroc 1 / -1, jesli klawisz rusza w zla strone.
DIRECTION = {"pan": 1, "lift": -1, "elbow": -1, "wflex": 1, "wroll": 1}
# =============================================================================

KEYS = {  # klawisz -> (staw, znak)
    "a": ("pan", 1), "d": ("pan", -1),
    "w": ("lift", 1), "s": ("lift", -1),
    "i": ("elbow", 1), "k": ("elbow", -1),
    "j": ("wflex", 1), "l": ("wflex", -1),
    "u": ("wroll", 1), "o": ("wroll", -1),
}
MOVE_JOINTS = ["pan", "lift", "elbow", "wflex", "wroll"]

POMOC = (
    "WASD/IJKL/UO ruch | SPACJA chwyc/pusc | Z/X chwytak | 1 2 3 predkosc | F stop\n"
    "ENTER ogladaj puszke | T sledzenie (kamera) | P zapisz poze skanu | R usun pozy skanu | M zapisz dom | H do domu | Q koniec"
)

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

    def klawisz(self, k):
        """Obsluga klawisza ruchu. Zwraca True, jesli klawisz byl ruchowy."""
        v = SPEEDS[self.level]
        if k in KEYS:
            joint, sign = KEYS[k]
            self.target[joint] += sign * DIRECTION[joint] * v * KEY_STEP_TIME
        elif k in ("1", "2", "3"):
            self.level = int(k) - 1
        elif k == "z":
            self.clamp = "open"
            self._grip(self.target["grip"] + 3)
        elif k == "x":
            self.clamp = "open"
            self._grip(self.target["grip"] - 3)
        elif k == " ":
            self.clamp = "closing" if self.clamp == "open" else "release"
        elif k == "f":
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
            self.target["lift"] += sticks["ly"] * DIRECTION["lift"] * v * dt
            self.target["elbow"] += sticks["ry"] * DIRECTION["elbow"] * v * dt
            self.target["wroll"] += sticks["rx"] * DIRECTION["wroll"] * v * dt
            if "lb" in held:
                self.target["wflex"] -= DIRECTION["wflex"] * v * dt
            if "rb" in held:
                self.target["wflex"] += DIRECTION["wflex"] * v * dt
            if "up" in pressed:
                self.level = min(self.level + 1, 2)
            if "down" in pressed:
                self.level = max(self.level - 1, 0)
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
            if abs(self.target[j] - self.sent.get(j, 1e9)) > 0.2:  # mikropoprawki = drzenie
                self.arm._write_goal(self.arm.ids[j], _to_ticks(self.target[j]), spd)
                self.sent[j] = self.target[j]
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


def read_keys():
    import msvcrt

    keys = []
    while msvcrt.kbhit():
        ch = msvcrt.getwch()
        if ch in ("\x00", "\xe0"):  # strzalki/F-klawisze - pomijamy
            msvcrt.getwch()
            continue
        keys.append(ch.lower())
    return keys


def klawisz_poz(arm, c):
    """P/R/M zapis poz. Zwraca komunikat albo None, jesli to nie ten klawisz."""
    if c == "m":
        return zapisz_poze(arm, POZA_DOMOWA)
    if c == "p":
        pozy = wczytaj(PLIK_POZ, {})
        wolne = [p for p in POZY_SKANU if p not in pozy]
        return zapisz_poze(arm, wolne[0]) if wolne else "sa juz 4 pozy skanu - R usuwa"
    if c == "r":
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

    for i in ([KAMERA] if KAMERA is not None else [1, 2, 3, 0]):
        kam = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        # MJPG + 1280x720: ostrzejsze kody niz domyslne 640x480, a wciaz plynnie
        kam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        kam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        kam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # Windows zapamietuje ekspozycje w sterowniku, wiec ustawiamy ja zawsze jawnie
        if KAMERA_EKSPOZYCJA is None:
            kam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        else:
            kam.set(cv2.CAP_PROP_EXPOSURE, KAMERA_EKSPOZYCJA)
        if kam.isOpened() and kam.read()[0]:
            print(f"Kamera {i} ({'laptop' if i == 0 else 'zewnetrzna'})")
            return kam
        kam.release()
    raise SO101Error("nie znalazlem zadnej kamery")


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


def serwer_http(arm):
    class Obsluga(BaseHTTPRequestHandler):
        def _json(self, dane, kod=200):
            self.send_response(kod)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(json.dumps(dane, ensure_ascii=False).encode())

        def do_GET(self):
            if self.path.startswith("/skanuj"):
                print("\nSYGNAL z sieci")
                if not zadanie(arm, ogladaj_puszke, "skan"):
                    return self._json({"blad": "ramie zajete"}, 409)
                _skan_koniec.wait(timeout=120)
                self._json(stan["wynik"] or {"blad": "timeout"})
            elif self.path.startswith("/status"):
                self._json(stan)
            else:
                self._json({"blad": "uzyj /skanuj albo /status"}, 404)

        do_POST = do_GET

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(("0.0.0.0", PORT_HTTP), Obsluga)
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


class Kaucje:
    """Czy EAN jest kaucyjny: kaucja.json albo api.kaucja.pl (w tle, z pamiecia)."""

    def __init__(self):
        self.status = {}  # ean -> True / False / None (sprawdzam)
        self.nazwy = {}

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
        except Exception:
            self.status[ean] = False  # brak sieci / blad -> traktuj jak nie-kaucyjny
        print(f"\n{ean}: {'KAUCJA' if self.status[ean] else 'bez kaucji'} {self.nazwy.get(ean, '')}")


def _log(tekst):
    if SLEDZ_LOG:
        with open(os.path.join(_katalog, SLEDZ_LOG), "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%H:%M:%S')} {tekst}\n")


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
        if not self._hist or teraz < self._nastepny or not hasattr(self, "g"):
            return
        od_utraty = teraz - self.widziany
        if od_utraty > SLEDZ_SZUKAJ_CZAS or self._szukano >= SLEDZ_SZUKAJ_MAX:
            return
        (t0, x0, y0), (t1, x1, y1) = self._hist[0], self._hist[-1]
        vx, vy = ((x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0)) if t1 - t0 > 0.05 else (0.0, 0.0)
        horyzont = min(od_utraty + SLEDZ_PAUZA, 1.0)
        przew = {"x": max(-1.5, min(1.5, x1 + vx * horyzont)), "y": max(-1.5, min(1.5, y1 + vy * horyzont))}
        stawy = {"x": SLEDZ_STAW_POZIOM, "y": SLEDZ_STAW_PION}
        zostalo = SLEDZ_SZUKAJ_MAX - self._szukano
        ruch = {}
        for o, staw in stawy.items():
            if abs(przew[o]) > SLEDZ_CEL:
                d = -SLEDZ_KROK * przew[o] / self.g[o]
                d = max(-min(SLEDZ_MAX_KROK, zostalo), min(min(SLEDZ_MAX_KROK, zostalo), d))
                if abs(d) >= SLEDZ_MIN_KROK:
                    st.target[staw] += d
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
        self._szukano = 0.0        # ile st. przejechal "na slepo" od utraty kodu

    def _steruj(self, st, msg, teraz):
        """Popraw i poczekaj: mediana kilku klatek z nieruchomego obrazu -> jeden ruch -> pauza."""
        if not hasattr(self, "_probki"):
            self._reset()
        if not hasattr(self, "g"):  # czulosc: o ile przesuwa sie kod (czesc polowy obrazu) na 1 st. stawu
            self.g = {"x": SLEDZ_ZNAK_POZIOM * SLEDZ_CZULOSC_POZIOM, "y": SLEDZ_ZNAK_PION * SLEDZ_CZULOSC_PION}
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
                    ruch[o] = d
        self._w_celu = not ruch
        if ruch:
            self._ostatni_ruch = (blad, {o: ruch.get(o, 0.0) for o in stawy})
            self._nastepny = teraz + SLEDZ_PAUZA
        _log(f"cel={self.cel} x={bx:+.2f} y={by:+.2f} {'odczyt' if self.blad[2] else 'prostokat'} "
             f"pan={st.here['pan']:.1f} wflex={st.here['wflex']:.1f} "
             f"ruch={ {stawy[o]: round(d, 1) for o, d in ruch.items()} } "
             f"czulosc x={self.g['x']:+.3f} y={self.g['y']:+.3f}{' W CELU' if self._w_celu else ''}")
        return msg


def tryb_kamera(port=None, mock=False):
    """Glowny program: okno kamery + sterowanie reczne + ogladanie puszki na sygnal."""
    import cv2
    import msvcrt

    arm = polacz(port, mock)
    st = Sterownik(arm)
    kam = otworz_kamere()
    kaucje, sled = Kaucje(), Sledzenie()
    sledz = SLEDZ
    t_klatki = time.time()
    srv = serwer_http(arm)
    print(f"Pad: {'PODLACZONY' if st.pad_ok else 'brak (tylko klawiatura)'}")
    print(f"GOTOWY. Sygnal od RoArma: http://{moje_ip()}:{PORT_HTTP}/skanuj")
    print("Klikni w okno kamery i steruj:\n" + POMOC)
    tytul = "Ramie SO-101"
    cv2.namedWindow(tytul, cv2.WINDOW_NORMAL)  # okno mozna zmniejszac/powiekszac
    cv2.resizeWindow(tytul, 960, 540)
    komunikat, komunikat_do = "", 0.0
    byl_zajety = False

    def pokaz(tekst):
        nonlocal komunikat, komunikat_do
        komunikat, komunikat_do = tekst, time.time() + 3
        print(f"\n{tekst}")

    try:
        while True:
            ok, klatka = kam.read()
            if not ok:
                continue
            teraz = time.time()
            dt, t_klatki = min(teraz - t_klatki, 0.2), teraz
            wys, szer = klatka.shape[:2]
            ostr = ostrosc(klatka)
            kody = znajdz_kody(klatka)
            for k in kody:
                if k["ean"] is None:  # prostokat kodu bez odczytu (rozmazany)
                    blisko = sled.xy and abs(k["cx"] - sled.xy[0]) + abs(k["cy"] - sled.xy[1]) < 5
                    cv2.polylines(klatka, [k["rogi"]], True, (160, 160, 160), 6 if blisko else 2)
                    (x0, y0), (x1, y1) = k["rogi"].min(axis=0)[0], k["rogi"].max(axis=0)[0]
                    roi = klatka[max(y0, 0):y1, max(x0, 0):x1]
                    if roi.size:  # ostrosc samego kodu - krec obiektywem, az wzrosnie i ramka zzielenieje
                        cv2.putText(klatka, f"NIEOSTRY KOD {ostrosc(roi, calosc=True):.0f}",
                                    (int(x0), max(int(y0) - 8, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    continue
                kaucja = kaucje.sprawdz(k["ean"])
                # zielony = kaucja, czerwony = bez kaucji, zolty = sprawdzam
                kolor = (0, 200, 0) if kaucja else ((0, 0, 230) if kaucja is False else (0, 220, 255))
                gruby = 6 if k["ean"] == sled.cel else 2
                cv2.polylines(klatka, [k["rogi"]], True, kolor, gruby)
                cv2.putText(klatka, f"{k['ean']} {k['rozmiar'] * 100:.0f}%", tuple(k["rogi"][0][0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, kolor, 2)
                if _zajety.is_set() and not stan["kod"]:
                    stan["kod"] = k["ean"]

            # sterowanie reczne tylko, gdy nie trwa automatyczny ruch
            if _zajety.is_set():
                byl_zajety = True
            else:
                if byl_zajety:
                    st.po_zadaniu()
                    byl_zajety = False
                if sledz:
                    mial_cel = sled.cel is not None
                    msg = sled.krok(kody, kaucje, szer, wys, st, dt)
                    if msg:
                        pokaz(msg)
                        _log(msg)
                    if sled.cel and teraz - getattr(sled, "_log", 0) > 0.3 and hasattr(sled, "blad"):
                        sled._log = teraz
                        ex, ey, odczyt = sled.blad
                        print(f"  kod x={ex:+.2f} y={ey:+.2f} {'odczyt' if odczyt else 'prostokat'}"
                              f" | pan={st.here['pan']:6.1f} wflex={st.here['wflex']:6.1f}"
                              f" | cel pan={st.target['pan']:6.1f} wflex={st.target['wflex']:6.1f}")
                    if mial_cel and sled.cel is None and PO_UTRACIE == "dom":
                        zadanie(arm, do_domu, "dom")
                pressed = st.krok()
                if "y" in pressed or "start" in pressed:
                    zadanie(arm, ogladaj_puszke, "skan")
                if "back" in pressed:
                    break

            k = cv2.waitKey(1) & 0xFF
            wejscie = []
            if k != 255:
                wejscie.append("\r" if k in (10, 13) else ("\x1b" if k == 27 else chr(k).lower()))
            while msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch == "\r":
                    wejscie.append("\r")
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
                    pokaz(f"sledzenie {'WLACZONE' if sledz else 'WYLACZONE'}")
                elif c == "m":
                    pokaz(zapisz_poze(arm, POZA_DOMOWA))
                elif c == "p":
                    pozy = wczytaj(PLIK_POZ, {})
                    wolne = [p for p in POZY_SKANU if p not in pozy]
                    pokaz(zapisz_poze(arm, wolne[0]) if wolne else "sa juz 4 pozy skanu - R usuwa")
                elif c == "r":
                    pozy = wczytaj(PLIK_POZ, {})
                    for p in POZY_SKANU:
                        pozy.pop(p, None)
                    zapisz(PLIK_POZ, pozy)
                    pokaz("usunieto pozy skanu - bedzie rozgladanie")
                else:
                    st.klawisz(c)

            # napisy na obrazie
            if st.komunikat:
                pokaz(st.komunikat)
                st.komunikat = ""
            gora = stan["stan"] + (f" | SLEDZE {sled.cel}" if sled.cel else (" | sledzenie wl." if sledz else ""))
            gora += f" | ostrosc {ostr:.0f}" + (" (NIEOSTRO - krec obiektywem)" if ostr < 60 else "")
            if stan["wynik"] and not _zajety.is_set():
                w = stan["wynik"]
                gora += f" | ostatni: {w.get('kod') or w.get('blad') or 'brak kodu'}"
                gora += " (KAUCJA)" if w.get("kaucja") else ""
            h, w_ = wys, szer
            cv2.drawMarker(klatka, (szer // 2, wys // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
            cv2.rectangle(klatka, (0, 0), (w_, 28), (0, 0, 0), -1)
            cv2.putText(klatka, gora, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.rectangle(klatka, (0, h - 46), (w_, h), (0, 0, 0), -1)
            cv2.putText(klatka, st.opis()[:90], (6, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 255, 200), 1)
            cv2.putText(klatka, "T sledzenie | ENTER ogladaj | WASD IJKL UO ruch | SPACJA chwyt | P/M/H | Q",
                        (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
            if komunikat and time.time() < komunikat_do:
                cv2.putText(klatka, komunikat, (6, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow(tytul, klatka)
    except KeyboardInterrupt:
        pass
    except SO101Error as e:
        print("\nBLAD:", e)
    finally:
        print("\nKoniec - ramie trzyma pozycje.")
        srv.shutdown()
        kam.release()
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
