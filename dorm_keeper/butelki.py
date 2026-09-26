"""RoArm sam zbiera butelki, ktore widzi kamera SO-101, i segreguje je: z kaucja / bez kaucji.

Cykl (python butelki.py):
  1. SO-101 patrzy na stol (pozycja z klawisza M w panelu) i nie sledzi
  2. butelka stoi spokojnie na stole -> jej podstawa w obrazie przeliczona na x, y RoArma (samokalibracja)
  3. RoArm chwyta ja z gory; kamera sprawdza, czy butelka nie zostala na stole (wtedy druga proba nizej/wyzej)
  4. RoArm podnosi ja przed kamere, SO-101 sledzi i czyta kod; BRAK_KODU -> RoArm obraca butelke (max 3 razy)
  5. KAUCYJNA -> pojemnik "kaucja", reszta -> pojemnik "inne"; RoArm odjezdza i od nowa

Przygotowanie (raz, ok. 2 minuty):
  - panel SO-101 (http://<pi>:8765/): kamera ma widziec stol z butelkami -> przycisk M (pozycja patrzenia)
  - python butelki.py kalibruj: postaw butelke, naprowadz na nia chwytak klawiszami (raz), reszte robi RoArm:
    sam przestawia butelke w kilka miejsc i patrzy kamera, gdzie ja widac -> przeliczenie obraz -> stol
  - albo python butelki.py kalibruj tag: AprilTag 0 (AprilTags/) przyklejony pionowo na szczece chwytaka, przodem
    do kamery SO-101; zmierz kal_tag_mm i kal_tag_nad_czubkiem_mm (roarm_kaucja.json). Dalej jak "kalibruj brev",
    tylko czubek chwytaka znajduje tag zamiast VLM (bez Brev, bez sieci, powtarzalnie)
  - albo python butelki.py kalibruj brev: bez butelki i bez czlowieka. RoArm dotyka stolu (wysokosc stolu z obciazenia
    serw), potem stawia czubek chwytaka w kilku miejscach tuz nad stolem, a VLM na Brev (Qwen2.5-VL) mowi, gdzie ten
    czubek widac w obrazie SO-101 -> to samo przeliczenie obraz -> stol
  - pojemniki: domyslnie po bokach RoArma, 28 cm od podstawy (lewo = kaucja, prawo = inne). Inne miejsca:
    ustaw RoArma nad pojemnikiem i python butelki.py zapisz kaucja|inne  (tak samo: kamera, czekaj)

Sprawdzanie:
    python butelki.py gdzie        gdzie SO-101 widzi butelke i gdzie RoArm by chwytal (RoArm stoi)
    python butelki.py celuj        RoArm staje nad butelka (bez chwytania)
    python butelki.py chwyc        RoArm chwyta butelke i podnosi ja (bez kamery/pojemnikow)
    python butelki.py punkty       punkty i kalibracja
    python butelki.py raz          jedna butelka;  python butelki.py  - bez konca (Ctrl+C konczy)
    python butelki.py --test       logika bez sprzetu (RoArm i SO-101 udawane)

Na laptopie, gdy ramie.py dziala na Pi: SO101_URL=http://192.168.32.114:8765 (w VS Code ustawione).
"""
import json
import math
import os
import re
import statistics
import sys
import time

import requests

_katalog = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(_katalog, "..", "Raspberry"))  # na laptopie: roarm_wifi.py z folderu kolegi
from roarm_wifi import Overheat, RoArm  # noqa: E402

PLIK = os.path.join(_katalog, "roarm_kaucja.json")
KONFIG_KOLEGI = [os.path.expanduser("~/Hackengersi/Raspberry/config.json"),  # repo (git pull = aktualny adres)
                 os.path.join(_katalog, "..", "Raspberry", "config.json")]
DOMYSLNE = {
    "roarm_ip": None,             # None = z Raspberry/config.json kolegi
    "so101_url": "http://localhost:8765",
    "spd": 0.2,                   # predkosc RoArma - plan: max 0.25 (bark sie grzeje)
    "podejscie_mm": 80,           # nad punktem chwytania/odlozenia najpierw tyle wyzej
    "chwyt_otwarty": 1.57,
    "chwyt_zamkniety": 3.14,
    "zasieg_mm": [120, 380],      # RoArm siega tak daleko od podstawy (poza tym - nie probuj)
    "obroty": [1.57, -1.57, 3.14],  # rad wzgledem pozy "kamera": +90, -90, 180 st. = 4 strony butelki
    "pauza_obrotu": 1.5,          # s na obrot nadgarstka (goto czeka tylko na x y z)
    "czas_oceny": 15.0,           # s czekania na wynik z kamery przy kazdym ustawieniu butelki
    "butelka_stoi_s": 1.0,        # tyle s butelka musi stac w miejscu, zanim RoArm po nia siegnie
    "poprawki_z": [0, -15, 15],   # mm: kolejne proby chwytu (butelka zostala na stole -> nizej, potem wyzej)
    "kalibracja_krok_mm": 70,     # samokalibracja: tak daleko RoArm przestawia butelke od punktu startu
    # punkty liczone same (mozna nadpisac: python butelki.py zapisz <nazwa>); z wzgledem wysokosci chwytu
    "kamera_nad_mm": 150,         # "kamera": srodek obszaru kalibracji, butelka tyle wyzej niz przy chwytaniu
    "pojemniki": {"kaucja": [0, 280, 150], "inne": [0, -280, 150]},  # x, y (od podstawy RoArma), z nad chwytem
    "czekaj": [100, 0, 250],      # x, y, z nad chwytem: RoArm wysoko przy podstawie (poza kadrem kamery)
    "punkty": {},
    "kalibracja": None,           # obraz SO-101 (pozycja patrzenia) -> x, y RoArma: python butelki.py kalibruj
    # --- kalibruj brev (bez butelki): wszystko do dostrojenia na prawdziwym stole ---
    "brev_url": None, "brev_model": None,  # None = z Raspberry/config.json kolegi (klucz: $BREV_KEY albo ~/.bashrc)
    "kal_brev_start": [250, 0],   # x, y (mm): srodek siatki - w zasiegu RoArma i w kadrze kamery SO-101
    "kal_brev_nad_stolem_mm": 5,
    "kal_g": 3.0,                 # chwytak przy kalibracji: prawie zamkniety - 3.14 sciska szczeki i grzeje serwo (57 C)
    "kal_t": 1.57, "kal_r": 0.0,  # nachylenie/obrot chwytaka przy kalibracji (1.57 = w dol); "kalibruj tag" bierze
                                  # je, jak i srodek siatki, z obecnej pozycji RoArma (ustawionej tak, by tag byl widac)  # czubek chwytaka tyle nad stolem, gdy kamera go oglada (plaszczyzna = podstawy butelek)
    "kal_brev_zgodnosc_px": 30,   # px pelnej klatki: dwie odpowiedzi VLM (dwie klatki) musza sie zgadzac, inaczej pomijam
    "kal_brev_ransac_mm": 20,     # punkt dalej niz tyle od dopasowanej homografii = zla odpowiedz VLM, odrzucony
    # --- kalibruj tag: AprilTag 36h11 przyklejony PIONOWO na szczece chwytaka, przodem do kamery SO-101 ---
    "kal_tag_id": 0,              # AprilTags/tag36h11_00.png
    "kal_tag_mm": 30,             # bok czarnego kwadratu tagu (zmierz linijka po wydruku)
    "kal_tag_nad_czubkiem_mm": 25,  # srodek tagu tyle nad czubkiem szczek (zmierz) - program przelicza na czubek
    # tag POZIOMO, twarza w gore, na "flagze" z kartonu przy czubku (chwytak w dol): srodek tagu przesuniety od czubka
    # o [od podstawy, w lewo] mm - para kalibracji to wtedy srodek tagu. Wtedy kal_tag_nad_czubkiem_mm = 0.
    "kal_tag_przesuniecie_mm": [0, 0],
    "szyjka_nad_stolem_mm": 180,  # wysokosc chwytu nad stolem: szyjka butelki 0,5 l (~20 cm wysokosci, pod nakretka)
    "stol_start_z": 0,            # mm: stad RoArm zaczyna schodzic do stolu (musi byc nad stolem!)
    "stol_dno_z": -200,           # mm: nizej nie schodzi (twardy limit) - brak stolu do tej wysokosci = blad
    "stol_krok_mm": 5,            # krok schodzenia
    "stol_spd": 0.1,              # predkosc przy stole
    "stol_pauza_s": 0.5,          # po kazdym kroku: serwa dojezdzaja, obciazenie sie ustala
    "stol_prog_obciazenia": 60,   # zmiana obciazenia barku/lokcia (jedn. firmware) wzgledem powietrza = dotkniecie
    "stol_prog_z_mm": 6,          # albo: ramie zostaje tyle nad celem (stol trzyma) wzgledem bledu w powietrzu
    "stol_max_obciazenie": 350,   # |obciazenie| barku/lokcia ponad to = STOP (jak OBCIAZENIE_STOP w roarm_panel.py)
}
NAZWY = ("kamera", "kaucja", "inne", "czekaj", "odbior")


def wczytaj():
    cfg = dict(DOMYSLNE)
    if os.path.exists(PLIK):
        with open(PLIK, encoding="utf-8") as f:
            cfg.update(json.load(f))
    if os.environ.get("SO101_URL"):  # butelki.py na laptopie, a ramie.py na Pi: SO101_URL=http://192.168.32.114:8765
        cfg["so101_url"] = os.environ["SO101_URL"]
    for p in KONFIG_KOLEGI:
        if os.path.exists(p):
            with open(p, encoding="utf-8") as f:
                kolega = json.load(f)
            for k in ("roarm_ip", "brev_url", "brev_model"):
                cfg[k] = cfg[k] or kolega.get(k)
            break
    return cfg


def zapisz(cfg):
    with open(PLIK, "w", encoding="utf-8") as f:
        json.dump({k: v for k, v in cfg.items() if k != "roarm_ip" or v}, f, indent=2, ensure_ascii=False)


def punkt(cfg, nazwa):
    """Punkt RoArma: nauczony (zapisz <nazwa>) albo policzony z kalibracji. None, gdy sie nie da."""
    if nazwa in cfg["punkty"]:
        return cfg["punkty"][nazwa]
    kal = cfg["kalibracja"]
    if not kal:
        return None
    ch = kal["chwyt"]
    if nazwa == "kamera":
        return dict(ch, x=kal["srodek"][0], y=kal["srodek"][1], z=ch["z"] + cfg["kamera_nad_mm"])
    xyz = cfg["pojemniki"].get(nazwa) or (cfg["czekaj"] if nazwa == "czekaj" else None)
    return dict(ch, x=xyz[0], y=xyz[1], z=ch["z"] + xyz[2]) if xyz else None


def jedz(arm, p, g=None, dz=0.0, droll=0.0, spd=0.2, tol=30.0, timeout=20.0):
    """Do punktu p (+dz mm w gore, +droll rad obrotu). Jak RoArm.goto, ale z obrotem nadgarstka (r).
    tol 30 mm: bark Feetech wisi do ~25 mm ponizej celu przy wyciagnietym ramieniu (zmierzone 2026-09-26)."""
    r = max(-3.14, min(3.14, p["r"] + droll))
    cel = {"x": p["x"], "y": p["y"], "z": p["z"] + dz}
    try:
        arm.send({"T": 104, **{k: round(v, 1) for k, v in cel.items()}, "t": round(p["t"], 3), "r": round(r, 3),
                  "g": round(DOMYSLNE["chwyt_otwarty"] if g is None else g, 3), "spd": spd})
    except RuntimeError as e:
        if "not answering" not in str(e):
            raise  # np. "Queue full"
        # RoArm nie odpowiada, dopoki jedzie (dluzszy ruch) - polecenie doszlo: patrz na pozycje ponizej
    if arm.mock:
        arm.fb.update(cel, r=r)
        return
    koniec = time.monotonic() + timeout
    while time.monotonic() < koniec:
        time.sleep(0.3)
        try:
            w = arm.where()
        except RuntimeError:
            continue  # zajety ruchem
        if max(abs(w[k] - cel[k]) for k in cel) < tol:
            return
    raise TimeoutError(f"RoArm nie dojechal do {cel} (jest {arm.where()})")


class SO101:
    """ramie.py (SO-101 z kamera) przez HTTP."""

    def __init__(self, url):
        self.url = url.rstrip("/")

    def _get(self, sciezka, **params):
        return requests.get(self.url + sciezka, params=params, timeout=3).json()

    def wynik(self):
        return self._get("/wynik")

    def obrocono(self):
        self._get("/obrocono")

    def butelki(self):
        return self._get("/butelki")

    def sledzenie(self, wl):
        self._get("/sledzenie", wl=int(bool(wl)))

    def patrz(self):
        odp = self._get("/patrz")
        if odp.get("blad"):
            raise RuntimeError(odp["blad"])

    def klatka(self):
        """Swieza klatka w pelnej rozdzielczosci z /klatka (1920x1080 - maly tag wychodzi ostrzej) -> (jpg, szer, wys).
        Stary ramie.py bez /klatka: podglad 640 px ponizej. Czekamy 0.3 s, zeby klatka byla sprzed ruchu, nie w trakcie."""
        import cv2
        import numpy as np

        time.sleep(0.3)
        r = requests.get(self.url + "/klatka", timeout=5)
        if r.status_code == 200:
            h, w = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_GRAYSCALE).shape
            return r.content, w, h
        return self._podglad_klatka()

    def _podglad_klatka(self):
        """Jedna swieza klatka z /podglad?czysty (JPEG 640 px szer., ramki YOLO bez napisow) -> (jpg, szer, wys).
        Pierwsza klatka strumienia bywa stara (sprzed ruchu) - bierzemy druga."""
        import cv2
        import numpy as np

        with requests.get(self.url + "/podglad", params={"czysty": 1}, stream=True, timeout=5) as r:
            buf, koniec = b"", time.monotonic() + 5
            for kawalek in r.iter_content(65536):
                buf += kawalek
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a)
                a2 = buf.find(b"\xff\xd8", b)
                b2 = buf.find(b"\xff\xd9", a2) if a2 >= 0 else -1
                if min(a, b, a2, b2) >= 0:
                    jpg = buf[a2:b2 + 2]
                    h, w = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE).shape
                    return jpg, w, h
                if time.monotonic() > koniec:
                    break
        raise RuntimeError("brak klatek z /podglad (ramie.py dziala? kamera?)")

    def stan(self, tekst):
        """Napis o RoArmie w widoku /pokaz (bledy sieci bez znaczenia)."""
        try:
            self._get("/roarm", stan=tekst)
        except requests.RequestException:
            pass


# ----------------------------------------------------------------------------- obraz SO-101 -> stol RoArma
def podstawa(b):
    """Punkt, w ktorym butelka stoi na stole: srodek dolnej krawedzi ramki (plaszczyzna stolu)."""
    return b["cx"], b["box"][3]


def stoi_pionowo(b):
    x0, y0, x1, y1 = b["box"]
    return (y1 - y0) > (x1 - x0)


def do_pozycji_kalibracji(u, v, widok, stawy_kal):
    """Kamera wraca do pozycji patrzenia z dokladnoscia do ~1-3 st. - przesun piksel tak, jakby stala dokladnie
    jak przy kalibracji (px_na_st: o ile px przesuwa sie obraz na 1 st. stawu)."""
    for staw, px in (widok.get("px_na_st") or {}).items():
        if staw in stawy_kal and staw in widok.get("stawy", {}):
            d = widok["stawy"][staw] - stawy_kal[staw]
            if staw == "pan":
                u -= px * d
            else:
                v -= px * d
    return u, v


def na_stol(kal, u, v):
    """Piksel (pozycja patrzenia) -> x, y RoArma w mm (homografia z kalibracji)."""
    H = kal["H"]
    w = H[2][0] * u + H[2][1] * v + H[2][2]
    return (H[0][0] * u + H[0][1] * v + H[0][2]) / w, (H[1][0] * u + H[1][1] * v + H[1][2]) / w


def czekaj_na_pozycje(so, timeout=40.0):
    """SO-101 w pozycji patrzenia i nieruchomo (tylko wtedy obraz = mapa stolu)."""
    koniec = time.monotonic() + timeout
    while time.monotonic() < koniec:
        w = so.butelki()
        if w.get("w_pozie") and w.get("stoi"):
            return w
        time.sleep(0.25)
    raise TimeoutError("SO-101 nie wrocil do pozycji patrzenia (ustawiona klawiszem M?)")


def stojaca_butelka(so, czas=1.0, timeout=None, tylko_jedna=False, pomin=()):
    """Czekaj, az butelka stoi spokojnie na stole (ta sama pozycja przez `czas` s). Zwraca (u, v, widok, b) -
    piksel podstawy - albo None po timeout. pomin = piksele, ktorych nie brac (np. butelka, ktorej nie da sie
    chwycic)."""
    koniec = None if timeout is None else time.monotonic() + timeout
    historia = []
    while koniec is None or time.monotonic() < koniec:
        w = so.butelki()
        dobre = [b for b in w.get("butelki", []) if b["klasa"] == "butelka" and b["box"][3] < w["wys"] - 4
                 and all(math.hypot(podstawa(b)[0] - pu, podstawa(b)[1] - pv) > 40 for pu, pv in pomin)]
        if not (w.get("w_pozie") and w.get("stoi")) or not dobre or (tylko_jedna and len(dobre) != 1):
            historia = []
        else:
            b = max(dobre, key=lambda b: (b["box"][2] - b["box"][0]) * (b["box"][3] - b["box"][1]))
            u, v = podstawa(b)
            teraz = time.monotonic()
            historia = [h for h in historia if math.hypot(h[1] - u, h[2] - v) < 15] + [(teraz, u, v)]
            if teraz - historia[0][0] >= czas and len(historia) >= 4:
                return statistics.median(h[1] for h in historia), statistics.median(h[2] for h in historia), w, b
        time.sleep(0.15)
    return None


def w_zasiegu(cfg, x, y):
    lo, hi = cfg["zasieg_mm"]
    return lo <= math.hypot(x, y) <= hi


def naprowadz_recznie(arm, cfg):
    """Raz na kalibracje: czlowiek ustawia chwytak na szyjce butelki (klawiatura, jak roarm_console)."""
    import roarm_console

    print("\nNaprowadz chwytak RoArma na szyjke butelki (otwarty chwytak dookola szyjki, jak do chwytania):\n"
          "  W/S przod/tyl  A/D lewo/prawo  R/F gora/dol  T/G pochylenie  Y/H obrot  [ ] krok  Q = gotowe\n"
          "  (albo ustaw go strona http://<roarm_ip>/ i nacisnij tu Q)")
    with roarm_console.raw_keys():
        roarm_console.main(arm, {"reach_mm": cfg["zasieg_mm"], "pick_t": 1.57, "grip_open": cfg["chwyt_otwarty"],
                                 "roarm_spd": cfg["spd"]})


def kalibruj(arm, so, cfg, naprowadz=naprowadz_recznie, log=print, naprowadzony=False):
    """Samokalibracja: RoArm przestawia butelke po stole, kamera SO-101 patrzy, gdzie ja widac.

    Pary: podstawa butelki w obrazie <-> miejsce, w ktorym RoArm ja postawil -> homografia obraz -> stol.
    Ta sama butelka i to samo YOLO co przy zbieraniu, wiec bledy sie znosza. Czlowiek tylko raz naprowadza chwytak
    (naprowadzony=True: chwytak juz jest na szyjce butelki - np. ustawiony strona RoArma).
    """
    import cv2
    import numpy as np

    otw, zam, spd, dz = cfg["chwyt_otwarty"], cfg["chwyt_zamkniety"], cfg["spd"], cfg["podejscie_mm"]
    log("SAMOKALIBRACJA. SO-101 patrzy na stol...")
    so.sledzenie(False)
    so.patrz()
    stawy = dict(czekaj_na_pozycje(so)["stawy"])
    if not naprowadzony:
        so.stan("kalibracja: czeka na butelkę")
        input("Postaw JEDNA butelke na stole (w kadrze kamery i w zasiegu RoArma) - ENTER: ")
        naprowadz(arm, cfg)
    r = arm.where()
    ch = {"z": r["z"], "t": r["tit"], "r": r.get("r", 0.0)}
    x0, y0 = r["x"], r["y"]
    tu = dict(ch, x=x0, y=y0)
    park = dict(ch, x=cfg["czekaj"][0], y=cfg["czekaj"][1], z=ch["z"] + cfg["czekaj"][2])
    log(f"Chwyt: x={x0:.0f} y={y0:.0f} z={ch['z']:.0f} mm. Dalej RoArm sam - nie ruszaj butelki.")
    so.stan("kalibracja: sam przestawia butelkę")
    # otwarty chwytak w gore i z kadru -> kamera widzi butelke, ktorej nikt nie ruszal -> pierwsza para
    arm.gripper(otw)
    jedz(arm, tu, otw, dz=dz, spd=spd)
    jedz(arm, park, otw, spd=spd)
    wynik = stojaca_butelka(so, czas=0.8, timeout=10)
    if wynik is None:
        raise RuntimeError("kamera nie widzi stojacej butelki (podstawa w kadrze?)")
    u, v, w, _ = wynik
    pary = [{"px": list(do_pozycji_kalibracji(u, v, w, stawy)), "x": x0, "y": y0}]
    log(f"  punkt 1: ({pary[0]['px'][0]:.0f}, {pary[0]['px'][1]:.0f}) px -> x={x0:.0f} y={y0:.0f} mm")
    jedz(arm, tu, otw, dz=dz, spd=spd)  # z powrotem po butelke
    jedz(arm, tu, otw, spd=spd)
    arm.gripper(zam)
    k = cfg["kalibracja_krok_mm"]
    for dx, dy in ((k, 0), (0, k), (-k, 0), (0, -k), (k, k), (-k, -k), (k, -k), (-k, k)):
        cel = dict(ch, x=x0 + dx, y=y0 + dy)
        if not w_zasiegu(cfg, cel["x"], cel["y"]):
            continue
        jedz(arm, tu, zam, dz=dz, spd=spd)            # butelka w gore
        jedz(arm, cel, zam, dz=dz, spd=spd)           # nad nowe miejsce
        jedz(arm, cel, zam, spd=spd)                  # postaw (ta sama wysokosc = stol plaski)
        arm.gripper(otw)
        jedz(arm, cel, otw, dz=dz, spd=spd)
        jedz(arm, park, otw, spd=spd)                 # z kadru
        wynik = stojaca_butelka(so, czas=0.8, timeout=6)
        if wynik and stoi_pionowo(wynik[3]):
            u, v, w, _ = wynik
            pary.append({"px": list(do_pozycji_kalibracji(u, v, w, stawy)), "x": cel["x"], "y": cel["y"]})
            log(f"  punkt {len(pary)}: ({pary[-1]['px'][0]:.0f}, {pary[-1]['px'][1]:.0f}) px -> "
                f"x={cel['x']:.0f} y={cel['y']:.0f} mm")
        elif wynik:
            raise RuntimeError("butelka sie przewrocila - postaw ja i zacznij od nowa (moze chwyt nizej?)")
        else:
            log(f"  x={cel['x']:.0f} y={cel['y']:.0f}: kamera tam butelki nie widzi (poza kadrem) - pomijam")
        jedz(arm, cel, otw, dz=dz, spd=spd)           # z powrotem po butelke
        jedz(arm, cel, otw, spd=spd)
        arm.gripper(zam)
        tu = cel
        if len(pary) >= 7:
            break
    jedz(arm, tu, zam, dz=dz, spd=spd)                # odstaw butelke tam, skad ja wziela
    start = dict(ch, x=x0, y=y0)
    jedz(arm, start, zam, dz=dz, spd=spd)
    jedz(arm, start, zam, spd=spd)
    arm.gripper(otw)
    jedz(arm, start, otw, dz=dz, spd=spd)
    jedz(arm, park, otw, spd=spd)
    if len(pary) < 4:
        raise RuntimeError(f"tylko {len(pary)} punktow w kadrze - postaw butelke blizej srodka kadru i powtorz")

    src = np.float32([p["px"] for p in pary])
    dst = np.float32([[p["x"], p["y"]] for p in pary])
    H, _ = cv2.findHomography(src, dst, 0)
    if H is None:
        raise RuntimeError("kalibracja nie wyszla - powtorz")
    kal = {"H": H.tolist(), "stawy": stawy, "szer": w["szer"], "wys": w["wys"], "pary": pary, "chwyt": ch,
           "srodek": [statistics.mean(p["x"] for p in pary), statistics.mean(p["y"] for p in pary)]}
    bledy = [math.hypot(*(a - b for a, b in zip(na_stol(kal, *p["px"]), (p["x"], p["y"])))) for p in pary]
    cfg["kalibracja"] = kal
    zapisz(cfg)
    so.stan("kalibracja gotowa")
    log(f"\nGotowe: {len(pary)} punktow, blad srednio {statistics.mean(bledy):.0f} mm, najwiekszy {max(bledy):.0f} mm"
        + (" - duzo: powtorz (butelka stala pewnie?)" if max(bledy) > 25 else " - OK"))
    return kal


# ----------------------------------------------------------------------------- kalibracja z Brev (bez butelki)
VLM_ROZMIAR = (896, 504)  # 16:9 jak kamera, wielokrotnosc 28 (siatka Qwen-VL) -> odpowiedz VLM w tych pikselach
PYTANIE_CZUBEK = """This {w}x{h} image is from a camera looking at a table. A black robot arm (RoArm) may reach into
view with its gripper pointing straight down at the table. Give the pixel point of the very tip of the gripper jaws:
the lowest point of the gripper, just above the table surface.
If no robot gripper is visible, answer {{"tip": null}}.
Answer with only JSON: {{"tip": [x, y]}}"""


def brev_klucz():
    """BREV_KEY z otoczenia, a na Pi (usluga nie czyta ~/.bashrc) z linii export BREV_KEY=... w ~/.bashrc."""
    if os.environ.get("BREV_KEY"):
        return os.environ["BREV_KEY"]
    try:
        with open(os.path.expanduser("~/.bashrc"), encoding="utf-8") as f:
            m = re.search(r"^\s*export\s+BREV_KEY=['\"]?([\w-]+)", f.read(), re.M)
        return m.group(1) if m else None
    except OSError:
        return None


def czubek_z_odpowiedzi(tekst, w, h):
    """Odpowiedz VLM -> (x, y) w obrazie w x h albo None (brak chwytaka, smieci, punkt poza obrazem)."""
    m = re.search(r"\{.*\}", tekst, re.S)  # modele lubia ```json i proze dookola
    try:
        tip = json.loads(m.group(0))["tip"] if m else None
        x, y = float(tip[0]), float(tip[1])
    except (ValueError, KeyError, TypeError, IndexError):
        return None
    return (x, y) if 0 <= x < w and 0 <= y < h else None


def zapytaj_brev(jpg, szer, wys, cfg):
    """Gdzie w klatce (szer x wys) jest czubek chwytaka RoArma - pyta Qwen2.5-VL na Brev. (u, v) albo None."""
    import base64

    import cv2
    import numpy as np

    w, h = VLM_ROZMIAR
    obraz = cv2.resize(cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR), VLM_ROZMIAR)
    maly = cv2.imencode(".jpg", obraz, [cv2.IMWRITE_JPEG_QUALITY, 90])[1]
    klucz = brev_klucz()
    r = requests.post(f"{cfg['brev_url'].rstrip('/')}/v1/chat/completions", timeout=60,
                      headers={"Authorization": f"Bearer {klucz}"} if klucz else {},
                      json={"model": cfg["brev_model"], "temperature": 0, "max_tokens": 60, "messages": [
                          {"role": "user", "content": [
                              {"type": "text", "text": PYTANIE_CZUBEK.format(w=w, h=h)},
                              {"type": "image_url", "image_url": {
                                  "url": "data:image/jpeg;base64," + base64.b64encode(maly).decode()}}]}]})
    r.raise_for_status()
    xy = czubek_z_odpowiedzi(r.json()["choices"][0]["message"]["content"], w, h)
    return None if xy is None else (xy[0] * szer / w, xy[1] * wys / h)


def znajdz_tag(jpg, szer, wys, cfg):
    """Czubek chwytaka z AprilTaga na szczece: srodek tagu przesuniety w dol o kal_tag_nad_czubkiem_mm.
    Skala z pionowej krawedzi samego tagu (tag stoi pionowo jak ta odleglosc, wiec skrot perspektywy sie zgadza).
    (u, v) w pikselach klatki albo None, gdy tagu nie widac."""
    import cv2
    import numpy as np

    obraz = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    rogi, ids, _ = det.detectMarkers(obraz)
    for r, i in zip(rogi, [] if ids is None else ids.ravel()):
        if i != cfg["kal_tag_id"]:
            continue
        r = r.reshape(4, 2)  # lewy-gorny, prawy-gorny, prawy-dolny, lewy-dolny
        pion = (np.linalg.norm(r[3] - r[0]) + np.linalg.norm(r[2] - r[1])) / 2  # px na kal_tag_mm w pionie
        u, v = r.mean(axis=0)
        v += cfg["kal_tag_nad_czubkiem_mm"] * pion / cfg["kal_tag_mm"]
        return (float(u), float(v)) if 0 <= v < wys else None
    return None


def tag_xy(cfg, x, y):
    """Srodek tagu na flagze: czubek (x, y) + przesuniecie [od podstawy, w lewo], obrocone z podstawa RoArma."""
    d_r, d_t = cfg.get("kal_tag_przesuniecie_mm") or (0, 0)
    b = math.atan2(y, x)
    return x + d_r * math.cos(b) - d_t * math.sin(b), y + d_r * math.sin(b) + d_t * math.cos(b)


def opis_temp(arm):
    return "temp " + ("/".join(f"{v:.0f}" for v in arm.temps.values()) + " C" if arm.temps else "?")


def pozycja(arm):
    """arm.where() z jedna powtorka (RoArm przez WiFi bywa zajety chwile po poleceniu)."""
    try:
        return arm.where()
    except RuntimeError:
        time.sleep(0.3)
        return arm.where()


def do_stolu(arm, cfg, x, y, log=print):
    """Czubek chwytaka (w dol, zamkniety) schodzi po kroku nad (x, y), az dotknie stolu -> z stolu (mm).

    Dotkniecie = obciazenie barku/lokcia odbiega od tego w powietrzu albo ramie zostaje nad celem (stol trzyma).
    Blad z w powietrzu odejmujemy: bark Feetech i tak wisi ~0,05 rad ponizej celu."""
    zam, spd, pauza = cfg["kal_g"], cfg["stol_spd"], cfg["stol_pauza_s"]
    p = {"x": x, "y": y, "z": cfg["stol_start_z"], "t": cfg["kal_t"], "r": cfg["kal_r"]}
    jedz(arm, p, zam, spd=spd)
    time.sleep(pauza)
    probki = [pozycja(arm) for _ in range(3)]
    baza = {k: statistics.median(w[k] for w in probki) for k in ("tS", "tE")}
    blad_z = statistics.median(w["z"] for w in probki) - p["z"]
    z = p["z"]
    while z - cfg["stol_krok_mm"] >= cfg["stol_dno_z"]:
        z -= cfg["stol_krok_mm"]
        arm.send({"T": 104, "x": round(x, 1), "y": round(y, 1), "z": round(z, 1), "t": p["t"], "r": p["r"], "g": zam,
                  "spd": spd})
        time.sleep(pauza)
        w = pozycja(arm)
        d_s, d_e, nad = w["tS"] - baza["tS"], w["tE"] - baza["tE"], w["z"] - z - blad_z
        log(f"  stol? z={w['z']:.0f} (cel {z:.0f}) bark {d_s:+.0f} lokiec {d_e:+.0f} nad celem {nad:+.0f} mm"
            f" | {opis_temp(arm)}")
        if max(abs(w["tS"]), abs(w["tE"])) > cfg["stol_max_obciazenie"]:
            arm.send({"T": 104, "x": round(w["x"], 1), "y": round(w["y"], 1), "z": round(w["z"], 1), "t": p["t"],
                      "r": p["r"], "g": zam, "spd": spd})  # STOP = zmierzona poza (nigdy T:0)
            raise RuntimeError(f"za duze obciazenie przy stole (bark {w['tS']}, lokiec {w['tE']}) - STOP")
        if max(abs(d_s), abs(d_e)) > cfg["stol_prog_obciazenia"] or nad > cfg["stol_prog_z_mm"]:
            jedz(arm, dict(p, z=w["z"] + 10), zam, spd=spd)  # odsun sie od stolu
            return w["z"]
    raise RuntimeError(f"nie ma stolu do z={cfg['stol_dno_z']} mm (stol nizej? stol_dno_z / stol_start_z)")


def kalibruj_brev(arm, so, cfg, pytaj=zapytaj_brev, log=print, zrodlo="brev"):
    """Samokalibracja bez butelki i bez czlowieka: RoArm stawia czubek chwytaka tuz nad stolem w kilku miejscach,
    VLM na Brev mowi, gdzie ten czubek widac w obrazie SO-101 -> homografia obraz -> stol (jak kalibruj())."""
    import cv2
    import numpy as np

    zam, spd = cfg["kal_g"], cfg["stol_spd"]
    log("SAMOKALIBRACJA (Brev). SO-101 patrzy na stol...")
    so.sledzenie(False)
    so.patrz()
    stawy = dict(czekaj_na_pozycje(so)["stawy"])
    x0, y0 = cfg["kal_brev_start"]
    if not w_zasiegu(cfg, x0, y0):
        raise RuntimeError(f"kal_brev_start {x0}, {y0} poza zasiegiem RoArma {cfg['zasieg_mm']}")
    so.stan("kalibracja: szuka stołu")
    z_stol = do_stolu(arm, cfg, x0, y0, log=log)
    log(f"Stol: z={z_stol:.0f} mm | {opis_temp(arm)}")
    so.stan("kalibracja: VLM szuka chwytaka")
    k = cfg["kalibracja_krok_mm"]
    pary = []
    for dx, dy in ((0, 0), (k, 0), (0, k), (-k, 0), (0, -k), (k, k), (-k, -k), (k, -k), (-k, k)):
        x, y = x0 + dx, y0 + dy
        if not w_zasiegu(cfg, x, y):
            continue
        gora = {"x": x, "y": y, "z": z_stol + cfg["podejscie_mm"], "t": cfg["kal_t"], "r": cfg["kal_r"]}
        jedz(arm, gora, zam, spd=spd)
        nisko = z_stol + cfg["kal_brev_nad_stolem_mm"]
        jedz(arm, dict(gora, z=nisko), zam, spd=spd)
        wisi = pozycja(arm)["z"] - nisko  # bark wisi -> czubek wyzej niz kazano: popraw raz o zmierzona roznice
        if wisi > 5:
            jedz(arm, dict(gora, z=nisko - wisi), zam, spd=spd)
        tu = pozycja(arm)  # para z ZMIERZONEJ pozycji (x, y), nie z polecenia
        widok = czekaj_na_pozycje(so)
        odp = []
        for _ in range(2):  # dwie klatki, dwa pytania: przypadkowa odpowiedz sie nie powtorzy
            jpg, w_obr, h_obr = so.klatka()
            uv = pytaj(jpg, w_obr, h_obr, cfg)
            odp.append(None if uv is None else (uv[0] * widok["szer"] / w_obr, uv[1] * widok["wys"] / h_obr))
        jedz(arm, gora, zam, spd=spd)  # w gore, zanim pojedzie dalej (nie szura po stole)
        if None in odp:
            log(f"  x={x:.0f} y={y:.0f}: VLM nie widzi chwytaka (poza kadrem?) - pomijam | {opis_temp(arm)}")
            continue
        rozrzut = math.hypot(odp[0][0] - odp[1][0], odp[0][1] - odp[1][1])
        if rozrzut > cfg["kal_brev_zgodnosc_px"]:
            log(f"  x={x:.0f} y={y:.0f}: odpowiedzi VLM rozne o {rozrzut:.0f} px - pomijam")
            continue
        u, v = (odp[0][0] + odp[1][0]) / 2, (odp[0][1] + odp[1][1]) / 2
        tx, ty = tag_xy(cfg, tu["x"], tu["y"])
        pary.append({"px": list(do_pozycji_kalibracji(u, v, widok, stawy)), "x": tx, "y": ty})
        log(f"  punkt {len(pary)}: ({pary[-1]['px'][0]:.0f}, {pary[-1]['px'][1]:.0f}) px -> x={tu['x']:.0f} "
            f"y={tu['y']:.0f} mm, czubek {tu['z'] - z_stol:+.0f} mm nad stolem"
            f" | {opis_temp(arm)}")
        if len(pary) >= 8:
            break
    if len(pary) < 4:
        raise RuntimeError(f"tylko {len(pary)} punktow z chwytakiem w kadrze - zmien kal_brev_start albo poze kamery")

    src = np.float32([p["px"] for p in pary])
    dst = np.float32([[p["x"], p["y"]] for p in pary])
    H, maska = cv2.findHomography(src, dst, cv2.RANSAC, cfg["kal_brev_ransac_mm"])
    if H is None:
        raise RuntimeError("kalibracja nie wyszla - powtorz")
    dobre = [p for p, m in zip(pary, maska.ravel()) if m]
    if len(dobre) < 4:
        raise RuntimeError(f"tylko {len(dobre)} zgodnych punktow (reszta: zle odpowiedzi VLM) - powtorz")
    # chwyt: butelka z gory za szyjke. Kamera widziala czubek na wysokosci stolu, a zbieranie liczy podstawe butelki
    # w obrazie - obie na plaszczyznie stolu, wiec homografia sie zgadza; wysokosc chwytu = stol + szyjka (knob).
    ch = {"z": z_stol + cfg["szyjka_nad_stolem_mm"], "t": 1.57, "r": 0.0}
    kal = {"H": H.tolist(), "stawy": stawy, "szer": widok["szer"], "wys": widok["wys"], "pary": dobre, "chwyt": ch,
           "srodek": [statistics.mean(p["x"] for p in dobre), statistics.mean(p["y"] for p in dobre)],
           "stol_z": z_stol, "zrodlo": zrodlo}
    bledy = [math.hypot(*(a - b for a, b in zip(na_stol(kal, *p["px"]), (p["x"], p["y"])))) for p in dobre]
    cfg["kalibracja"] = kal
    zapisz(cfg)
    so.stan("kalibracja gotowa")
    log(f"\nGotowe: {len(dobre)}/{len(pary)} punktow, blad srednio {statistics.mean(bledy):.0f} mm, najwiekszy "
        f"{max(bledy):.0f} mm" + (" - duzo: powtorz" if max(bledy) > 25 else " - OK") + f" | {opis_temp(arm)}")
    return kal


def cel_na_stole(so, cfg, timeout=None, pomin=()):
    """Butelka stojaca na stole -> ((x, y) RoArma, (u, v) piksel); None po timeout."""
    kal = cfg["kalibracja"]
    wynik = stojaca_butelka(so, czas=cfg["butelka_stoi_s"], timeout=timeout, pomin=pomin)
    if wynik is None:
        return None
    u, v, w, _ = wynik
    if w["szer"] != kal["szer"]:  # inna rozdzielczosc kamery niz przy kalibracji
        u, v = u * kal["szer"] / w["szer"], v * kal["wys"] / w["wys"]
    u, v = do_pozycji_kalibracji(u, v, w, kal["stawy"])
    return na_stol(kal, u, v), (wynik[0], wynik[1])


def zostala_na_stole(so, u, v, czas=1.2):
    """Po chwycie: czy butelka dalej stoi tam, gdzie stala (chwyt nie wyszedl)?"""
    koniec, trafienia, proby = time.monotonic() + czas, 0, 0
    while time.monotonic() < koniec:
        w = so.butelki()
        proby += 1
        if any(math.hypot(podstawa(b)[0] - u, podstawa(b)[1] - v) < 30 for b in w.get("butelki", [])):
            trafienia += 1
        time.sleep(0.2)
    return proby > 0 and trafienia >= max(2, proby // 2)


# ----------------------------------------------------------------------------- cykl jednej butelki
def czekaj_na_wynik(so, poprzedni, czas):
    """Nowy wynik inspekcji (inny niz poprzedni) albo None po czasie."""
    koniec = time.monotonic() + czas
    while time.monotonic() < koniec:
        try:
            w = so.wynik()
        except requests.RequestException:
            w = {}
        if w.get("stan") == "WYNIK" and (w.get("czas"), w.get("kod"), w.get("wynik")) != poprzedni:
            return w
        time.sleep(0.3)
    return None


def klucz(w):
    return (w.get("czas"), w.get("kod"), w.get("wynik")) if w else None


def chwyc(arm, so, cfg, odbior, px=None, log=print):
    """Chwyc butelke w punkcie odbior. px = gdzie ja widac - wtedy sprawdz kamera i ewentualnie sprobuj
    nizej/wyzej. True = trzyma (albo nie da sie sprawdzic)."""
    otw, zam, spd, dz = cfg["chwyt_otwarty"], cfg["chwyt_zamkniety"], cfg["spd"], cfg["podejscie_mm"]
    for i, poprawka in enumerate(cfg["poprawki_z"] if px else [0]):
        p = dict(odbior, z=odbior["z"] + poprawka)
        if i:
            log(f"   butelka zostala na stole - proba {i + 1} ({poprawka:+d} mm)")
            so.stan(f"chwyta jeszcze raz ({i + 1})")
        jedz(arm, p, otw, dz=dz, spd=spd)
        jedz(arm, p, otw, spd=spd)
        arm.gripper(zam)
        jedz(arm, p, zam, dz=dz, spd=spd)
        if not px or not zostala_na_stole(so, *px):
            return True
        arm.gripper(otw)
    return False


def jedna_butelka(arm, so, cfg, log=print, xy=None, px=None):
    """xy = butelka na stole (z kamery SO-101); None = ze stalego punktu "odbior"."""
    otw, zam, spd, dz = cfg["chwyt_otwarty"], cfg["chwyt_zamkniety"], cfg["spd"], cfg["podejscie_mm"]
    potrzebne = ["kamera", "kaucja", "inne"] + ([] if xy else ["odbior"])
    brak = [n for n in potrzebne if punkt(cfg, n) is None]
    if brak:
        raise SystemExit(f"brak punktow {brak}: python butelki.py kalibruj (albo zapisz <nazwa>)")
    odbior = dict(cfg["kalibracja"]["chwyt"], x=xy[0], y=xy[1]) if xy else punkt(cfg, "odbior")

    log(f"1. biore butelke (x={odbior['x']:.0f} y={odbior['y']:.0f} mm)")
    so.stan("chwyta butelkę")
    if not chwyc(arm, so, cfg, odbior, px, log):
        log("   nie udalo sie chwycic - pomijam te butelke")
        so.stan("nie udało się chwycić")
        jedz(arm, punkt(cfg, "czekaj") or odbior, otw, dz=0 if punkt(cfg, "czekaj") else dz, spd=spd)
        return {"wynik": "NIE_CHWYCONA", "pojemnik": None, "ocena": None}

    try:
        poprzedni = klucz(so.wynik())
    except requests.RequestException:
        poprzedni = None
    log("2. pokazuje kamerze")
    so.stan("pokazuje butelkę kamerze")
    kamera = punkt(cfg, "kamera")
    jedz(arm, kamera, zam, spd=spd)
    so.sledzenie(True)  # SO-101 lapie butelke w chwytaku, centruje i czyta kod
    wynik = czekaj_na_wynik(so, poprzedni, cfg["czas_oceny"])
    for i, obrot in enumerate(cfg["obroty"], 1):
        if wynik and wynik["wynik"] != "BRAK_KODU":
            break
        log(f"   {'brak kodu' if wynik else 'brak wyniku'} -> obracam butelke ({i}/{len(cfg['obroty'])})")
        so.stan(f"obraca butelkę ({i}/{len(cfg['obroty'])})")
        poprzedni = klucz(wynik) or poprzedni
        jedz(arm, kamera, zam, droll=obrot, spd=spd)
        time.sleep(cfg["pauza_obrotu"])
        try:
            so.obrocono()
        except requests.RequestException:
            pass
        wynik = czekaj_na_wynik(so, poprzedni, cfg["czas_oceny"]) or wynik

    rodzaj = (wynik or {}).get("wynik", "BRAK_WYNIKU")
    cel = "kaucja" if rodzaj == "KAUCYJNA" else "inne"
    opis = f"{rodzaj} {(wynik or {}).get('kod') or ''} {(wynik or {}).get('nazwa') or ''}".strip()
    log(f"3. {opis} -> pojemnik '{cel}'")
    so.stan("odkłada do pojemnika " + ("Z KAUCJĄ" if cel == "kaucja" else "bez kaucji"))
    if xy:
        so.sledzenie(False)  # kamera nie goni butelki do pojemnika - wraca patrzec na stol
    pojemnik = punkt(cfg, cel)
    jedz(arm, kamera, zam, dz=dz, spd=spd)  # najpierw wyzej, potem w bok - bez zahaczania o stol
    jedz(arm, pojemnik, zam, dz=dz, spd=spd)
    jedz(arm, pojemnik, zam, spd=spd)
    arm.gripper(otw)
    jedz(arm, pojemnik, otw, dz=dz, spd=spd)
    if punkt(cfg, "czekaj"):
        jedz(arm, punkt(cfg, "czekaj"), otw, spd=spd)
    return {"wynik": rodzaj, "pojemnik": cel, "ocena": wynik}


def zbieraj(arm, so, cfg, raz=False, log=print):
    """Bez konca: czekaj na butelke na stole -> chwyc -> kaucja -> pojemnik."""
    nieudane = []  # piksele butelek, ktorych nie dalo sie chwycic (nie probuj w kolko tej samej)
    while True:
        so.sledzenie(False)
        so.patrz()
        jedz(arm, punkt(cfg, "czekaj"), cfg["chwyt_otwarty"], spd=cfg["spd"])  # RoArm poza kadrem
        czekaj_na_pozycje(so)
        so.stan("czeka na butelkę na stole")
        log("\nCzekam na butelke na stole...")
        (x, y), px = cel_na_stole(so, cfg, pomin=nieudane[-5:])
        if not w_zasiegu(cfg, x, y):
            log(f"butelka poza zasiegiem RoArma (x={x:.0f} y={y:.0f} mm, {math.hypot(x, y):.0f} mm od podstawy)")
            so.stan("butelka poza zasięgiem")
            nieudane.append(px)
            time.sleep(2)
            continue
        wynik = jedna_butelka(arm, so, cfg, log=log, xy=(x, y), px=px)
        if wynik["wynik"] == "NIE_CHWYCONA":
            nieudane.append(px)
        log(f"=> {wynik['wynik']} -> {wynik['pojemnik']}")
        if raz:
            so.sledzenie(False)
            so.patrz()
            return wynik


# ----------------------------------------------------------------------------- test bez sprzetu
def test():
    """Udawany RoArm + udawana kamera: stol widziany przez homografie px -> mm = (u/2, v/2 + 100)."""
    otw, zam = DOMYSLNE["chwyt_otwarty"], DOMYSLNE["chwyt_zamkniety"]
    swiat = {"butelka": (230.0, 240.0), "trzyma": False}  # gdzie stoi butelka (mm RoArma) / czy w chwytaku

    arm = RoArm("mock", mock=True)
    arm.fb.update(x=230.0, y=240.0, z=30.0, tit=1.57, r=0.0)
    wysylane = []
    arm.send = lambda c: wysylane.append(c)

    def chwytak(g):
        wysylane.append({"g": g})
        blisko = math.hypot(arm.fb["x"] - swiat["butelka"][0], arm.fb["y"] - swiat["butelka"][1]) < 5 \
            and abs(arm.fb["z"] - 30.0) < 20
        if g == zam and blisko and not swiat["trzyma"]:
            swiat["trzyma"] = True
        elif g == otw and swiat["trzyma"]:
            swiat["trzyma"], swiat["butelka"] = False, (arm.fb["x"], arm.fb["y"])
    arm.gripper = chwytak

    orig_jedz = globals()["jedz"]

    def jedz_z_butelka(a, p, *args, **kw):  # butelka jedzie razem z chwytakiem
        orig_jedz(a, p, *args, **kw)
        if swiat["trzyma"]:
            swiat["butelka"] = (a.fb["x"], a.fb["y"])
    globals()["jedz"] = jedz_z_butelka

    class FakeSO:
        odp = [{"stan": "brak inspekcji"}, {"stan": "WYNIK", "wynik": "BRAK_KODU", "czas": "1", "kod": None},
               {"stan": "WYNIK", "wynik": "KAUCYJNA", "czas": "2", "kod": "5900541012218", "nazwa": "Zywiec"}]

        def __init__(self):
            self.obrocen, self.sledzi, self.patrzy, self.napisy, self.pokazana = 0, None, 0, [], False

        def wynik(self):
            return self.odp[min(self.obrocen + 1, 2)] if self.pokazana else self.odp[0]

        def obrocono(self):
            self.obrocen += 1

        def sledzenie(self, wl):
            self.sledzi = wl
            self.pokazana = self.pokazana or bool(wl)

        def patrz(self):
            self.patrzy += 1

        def stan(self, t):
            self.napisy.append(t)

        def butelki(self):  # kamera 3 st. obok pozycji z kalibracji (pan 13 zamiast 10) -> obraz przesuniety
            butelki = []
            if not swiat["trzyma"]:
                x, y = swiat["butelka"]
                u, v = 2 * x - 19.2 * 3, 2 * (y - 100)
                butelki = [{"klasa": "butelka", "pewnosc": 0.9, "cx": u, "cy": v - 100, "box": [u - 40, v - 200, u + 40, v]}]
            return {"szer": 1920, "wys": 1080, "w_pozie": True, "stoi": True, "stawy": {"pan": 13.0, "wflex": 0.0},
                    "px_na_st": {"pan": -19.2, "wflex": -16.2}, "butelki": butelki}

    import builtins

    orig_zapisz, orig_input = globals()["zapisz"], builtins.input
    globals()["zapisz"] = lambda c: None  # test nie nadpisuje prawdziwego pliku
    builtins.input = lambda *_: ""        # ENTER w kalibracji
    try:
        cfg = dict(DOMYSLNE, czas_oceny=0.4, pauza_obrotu=0, butelka_stoi_s=0.3, punkty={}, kalibracja=None)
        so = FakeSO()
        kal = kalibruj(arm, so, cfg, naprowadz=lambda a, c: None, log=lambda *_: None)
        assert len(kal["pary"]) >= 5, kal["pary"]
        x, y = na_stol(kal, *do_pozycji_kalibracji(2 * 250 - 19.2 * 3, 2 * (200 - 100), so.butelki(), kal["stawy"]))
        assert abs(x - 250) < 0.5 and abs(y - 200) < 0.5, (x, y)
        assert swiat["butelka"] == (230.0, 240.0) and not swiat["trzyma"], "butelka odstawiona na start"

        # zbieranie: butelka w nowym miejscu -> chwyt -> brak kodu -> obrot -> kaucja -> pojemnik "kaucja"
        swiat["butelka"] = (260.0, 180.0)
        wysylane.clear()
        so = FakeSO()
        w = zbieraj(arm, so, cfg, raz=True, log=lambda *_: None)
        assert w["pojemnik"] == "kaucja" and so.obrocen == 1, (w, so.obrocen)
        kauc = punkt(cfg, "kaucja")
        assert math.hypot(swiat["butelka"][0] - kauc["x"], swiat["butelka"][1] - kauc["y"]) < 1, swiat
        assert any(c.get("r") == 1.57 for c in wysylane) and all(c.get("T") != 0 for c in wysylane)
        assert so.sledzi is False and so.patrzy == 2 and "chwyta butelkę" in so.napisy

        # chwyt nie wychodzi (butelka zostaje na stole) -> 3 proby, potem pomija
        swiat["butelka"], swiat["trzyma"] = (240.0, 200.0), False
        arm.gripper = lambda g: wysylane.append({"g": g})
        w = jedna_butelka(arm, FakeSO(), cfg, log=lambda *_: None, xy=(240.0, 200.0), px=(2 * 240.0 - 57.6, 200.0))
        assert w["wynik"] == "NIE_CHWYCONA", w
        assert not w_zasiegu(cfg, 50, 0) and w_zasiegu(cfg, 200, 100)

        # kalibracja z Brev: bez butelki; stol na z=-100, udawany VLM widzi czubek chwytaka (klatka 640x360);
        # z 9 punktow siatki: 3 poza zasiegiem (>380 mm), 1 poza kadrem -> 5 par
        stol = -100.0
        arm2 = RoArm("mock", mock=True)
        arm2.fb.update(x=230.0, y=240.0, z=50.0, tit=1.57, r=0.0, tS=-150, tE=150)

        def send2(c):
            if c.get("T") == 104:  # stol zatrzymuje ramie i naciska na bark
                arm2.fb.update(x=c["x"], y=c["y"], z=max(c["z"], stol), tS=-150 + (90 if c["z"] < stol else 0))
        arm2.send = send2

        class FakeSO2(FakeSO):
            def klatka(self):
                return b"", 640, 360

        def fake_vlm(jpg, w, h, c):
            x, y = arm2.fb["x"], arm2.fb["y"]
            if (x, y) == (160.0, 170.0):
                return None  # tu chwytak poza kadrem
            return (2 * x - 19.2 * 3) * w / 1920, 2 * (y - 100) * h / 1080

        swiat["trzyma"] = True  # butelki nie ma na stole
        cfg2 = dict(cfg, kalibracja=None, kal_brev_start=[230, 240], stol_pauza_s=0)
        so2 = FakeSO2()
        kal = kalibruj_brev(arm2, so2, cfg2, pytaj=fake_vlm, log=lambda *_: None)
        assert abs(kal["stol_z"] - stol) < 1 and kal["chwyt"]["z"] == stol + cfg2["szyjka_nad_stolem_mm"], kal
        assert len(kal["pary"]) == 5 and all((p["x"], p["y"]) != (160.0, 170.0) for p in kal["pary"]), kal["pary"]
        x, y = na_stol(kal, *do_pozycji_kalibracji(2 * 260 - 19.2 * 3, 2 * (210 - 100), so2.butelki(), kal["stawy"]))
        assert abs(x - 260) < 2 and abs(y - 210) < 2, (x, y)
        assert czubek_z_odpowiedzi('```json\n{"tip": [100, 50]}\n```', 896, 504) == (100.0, 50.0)
        assert czubek_z_odpowiedzi('{"tip": null}', 896, 504) is None

        # AprilTag na chwytaku: tag 60x60 px na (300..360, 100..160), bok 30 mm, srodek 25 mm nad czubkiem -> czubek
        # 50 px pod srodkiem tagu; inny tag = None
        import cv2
        import numpy as np

        obraz = np.full((360, 640), 255, np.uint8)
        tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 60)
        obraz[100:160, 300:360] = tag
        jpg = cv2.imencode(".jpg", obraz)[1].tobytes()
        u, v = znajdz_tag(jpg, 640, 360, DOMYSLNE)
        assert abs(u - 329.5) < 2 and abs(v - (129.5 + 50)) < 3, (u, v)
        assert znajdz_tag(jpg, 640, 360, dict(DOMYSLNE, kal_tag_id=1)) is None
        # flaga 50 mm od podstawy i 20 mm w lewo; RoArm patrzy w +y (podstawa obrocona o 90 st.)
        tx, ty = tag_xy(dict(DOMYSLNE, kal_tag_przesuniecie_mm=[50, 20]), 0.0, 200.0)
        assert abs(tx + 20) < 1e-6 and abs(ty - 250) < 1e-6, (tx, ty)
        assert czubek_z_odpowiedzi('{"tip": [900, 50]}', 896, 504) is None and czubek_z_odpowiedzi("nie", 9, 9) is None
    finally:
        globals()["jedz"], globals()["zapisz"], builtins.input = orig_jedz, orig_zapisz, orig_input
    print("test ok")


def main():
    if "--test" in sys.argv:
        return test()
    cfg = wczytaj()
    cmd = sys.argv[1:2] or [""]
    arm = RoArm(cfg["roarm_ip"])
    so = SO101(cfg["so101_url"])
    if cmd == ["punkty"]:
        print(f"RoArm {cfg['roarm_ip']}, SO-101 {cfg['so101_url']}")
        for n in NAZWY:
            p = punkt(cfg, n)
            zrodlo = "nauczony" if n in cfg["punkty"] else "z kalibracji"
            print(f"  {n:8s}", f"x={p['x']:.0f} y={p['y']:.0f} z={p['z']:.0f} ({zrodlo})" if p else "- brak -")
        kal = cfg["kalibracja"]
        print("  kalibracja:", f"{len(kal['pary'])} pkt, chwyt z={kal['chwyt']['z']:.0f} mm" if kal else "- brak -")
    elif cmd == ["zapisz"] and sys.argv[2:3] and sys.argv[2] in NAZWY:
        w = arm.where()
        # bez chwytaka: jego odczyt bywa 0 (firmware), a otwarcie/zamkniecie i tak wynika z etapu
        cfg["punkty"][sys.argv[2]] = {"x": w["x"], "y": w["y"], "z": w["z"], "t": w["tit"], "r": w.get("r", 0.0)}
        zapisz(cfg)
        print("zapisano", sys.argv[2], cfg["punkty"][sys.argv[2]])
    elif cmd == ["idz"] and sys.argv[2:3] and punkt(cfg, sys.argv[2]):
        jedz(arm, punkt(cfg, sys.argv[2]), cfg["chwyt_otwarty"], spd=cfg["spd"])
    elif cmd == ["kalibruj"] and sys.argv[2:3] == ["tag"]:  # jak brev, ale czubek z AprilTaga na chwytaku
        w = arm.where()  # RoArm ustawiony tak, ze kamera widzi tag: tu srodek siatki, to nachylenie i obrot
        if not any(w.get(k) for k in ("tB", "tS", "tE", "tT", "tR")):  # silniki wyl. (przestawiony recznie):
            # cel serw = zmierzone katy, potem moment - ramie zostaje, gdzie jest (jak w panelu RoArma)
            arm.send({"T": 102, "base": w["b"], "shoulder": w["s"], "elbow": w["e"], "wrist": w["t"],
                      "roll": w["r"], "hand": w["g"], "spd": 50, "acc": 10})
            arm.send({"T": 210, "cmd": 1})
            time.sleep(1.0)
            w = arm.where()
        cfg.update(kal_brev_start=[w["x"], w["y"]], kal_t=w["tit"], kal_r=w.get("r", 0.0), stol_start_z=w["z"])
        print(f"srodek x={w['x']:.0f} y={w['y']:.0f} mm, nachylenie {w['tit']:.2f} rad")
        kalibruj_brev(arm, so, cfg, pytaj=znajdz_tag, zrodlo="tag")
    elif cmd == ["kalibruj"] and sys.argv[2:3] == ["brev"]:  # bez butelki i bez czlowieka: VLM na Brev
        kalibruj_brev(arm, so, cfg)
    elif cmd == ["kalibruj"]:  # "kalibruj gotowe" = chwytak juz stoi na szyjce butelki (bez klawiatury)
        kalibruj(arm, so, cfg, naprowadzony=sys.argv[2:3] == ["gotowe"])
    elif cmd in (["gdzie"], ["celuj"], ["chwyc"]):
        if not cfg["kalibracja"]:
            raise SystemExit("brak kalibracji: python butelki.py kalibruj")
        so.sledzenie(False)
        so.patrz()
        czekaj_na_pozycje(so)
        cel = cel_na_stole(so, cfg, timeout=15)
        if cel is None:
            raise SystemExit("nie widze stojacej butelki na stole")
        (x, y), (u, v) = cel
        print(f"butelka: ({u:.0f}, {v:.0f}) px -> RoArm x={x:.0f} y={y:.0f} mm, {math.hypot(x, y):.0f} mm od podstawy"
              + ("" if w_zasiegu(cfg, x, y) else "  POZA ZASIEGIEM"))
        if cmd == ["chwyc"] and w_zasiegu(cfg, x, y):
            odbior = dict(cfg["kalibracja"]["chwyt"], x=x, y=y)
            ok = chwyc(arm, so, cfg, odbior, px=(u, v))
            print("TRZYMA - RoArm czeka nad stolem (python butelki.py idz czekaj = odjazd z butelka)" if ok
                  else "nie chwycil (3 proby) - zmien szyjka_nad_stolem_mm albo powtorz kalibracje")
        elif cmd == ["celuj"] and w_zasiegu(cfg, x, y):
            jedz(arm, dict(cfg["kalibracja"]["chwyt"], x=x, y=y), cfg["chwyt_otwarty"],
                 dz=cfg["podejscie_mm"], spd=cfg["spd"])
            print(f"RoArm stoi {cfg['podejscie_mm']} mm nad miejscem chwytania - chwytak powinien byc nad szyjka")
    elif cmd in (["raz"], [""]):
        so.wynik()  # od razu blad, jesli ramie.py nie dziala
        if cfg["kalibracja"]:
            zbieraj(arm, so, cfg, raz=cmd == ["raz"])
        else:
            print("(brak kalibracji - butelka ze stalego punktu 'odbior'; zrob: python butelki.py kalibruj)")
            while True:
                print(jedna_butelka(arm, so, cfg)["wynik"])
                if cmd == ["raz"] or input("\nENTER = nastepna butelka, Q = koniec: ").strip().lower() == "q":
                    break
    else:
        print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKoniec.")
    except (RuntimeError, TimeoutError, Overheat, requests.RequestException) as e:
        sys.exit(f"BLAD: {e}")
