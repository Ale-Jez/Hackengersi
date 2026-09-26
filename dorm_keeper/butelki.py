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
  - pojemniki: domyslnie po bokach RoArma, 28 cm od podstawy (lewo = kaucja, prawo = inne). Inne miejsca:
    ustaw RoArma nad pojemnikiem i python butelki.py zapisz kaucja|inne  (tak samo: kamera, czekaj)

Sprawdzanie:
    python butelki.py gdzie        gdzie SO-101 widzi butelke i gdzie RoArm by chwytal (RoArm stoi)
    python butelki.py celuj        RoArm staje nad butelka (bez chwytania)
    python butelki.py punkty       punkty i kalibracja
    python butelki.py raz          jedna butelka;  python butelki.py  - bez konca (Ctrl+C konczy)
    python butelki.py --test       logika bez sprzetu (RoArm i SO-101 udawane)

Na laptopie, gdy ramie.py dziala na Pi: SO101_URL=http://192.168.32.114:8765 (w VS Code ustawione).
"""
import json
import math
import os
import statistics
import sys
import time

import requests

_katalog = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(_katalog, "..", "Raspberry"))  # na laptopie: roarm_wifi.py z folderu kolegi
from roarm_wifi import RoArm  # noqa: E402

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
}
NAZWY = ("kamera", "kaucja", "inne", "czekaj", "odbior")


def wczytaj():
    cfg = dict(DOMYSLNE)
    if os.path.exists(PLIK):
        with open(PLIK, encoding="utf-8") as f:
            cfg.update(json.load(f))
    if os.environ.get("SO101_URL"):  # butelki.py na laptopie, a ramie.py na Pi: SO101_URL=http://192.168.32.114:8765
        cfg["so101_url"] = os.environ["SO101_URL"]
    if not cfg["roarm_ip"]:
        for p in KONFIG_KOLEGI:
            if os.path.exists(p):
                with open(p, encoding="utf-8") as f:
                    cfg["roarm_ip"] = json.load(f).get("roarm_ip")
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


def jedz(arm, p, g=None, dz=0.0, droll=0.0, spd=0.2, tol=15.0, timeout=20.0):
    """Do punktu p (+dz mm w gore, +droll rad obrotu). Jak RoArm.goto, ale z obrotem nadgarstka (r)."""
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
    elif cmd == ["kalibruj"]:  # "kalibruj gotowe" = chwytak juz stoi na szyjce butelki (bez klawiatury)
        kalibruj(arm, so, cfg, naprowadzony=sys.argv[2:3] == ["gotowe"])
    elif cmd in (["gdzie"], ["celuj"]):
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
        if cmd == ["celuj"] and w_zasiegu(cfg, x, y):
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
    except (RuntimeError, TimeoutError, requests.RequestException) as e:
        sys.exit(f"BLAD: {e}")
