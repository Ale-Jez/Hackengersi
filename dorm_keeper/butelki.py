"""RoArm + SO-101: butelka pod kamere -> ocena kaucji -> odpowiedni pojemnik.

RoArm bierze butelke z miejsca "odbior", pokazuje ja kamerze SO-101 w punkcie "kamera" i czeka na wynik
z ramie.py (http://localhost:8765/wynik). BRAK_KODU -> obraca butelke nadgarstkiem (roll) i wysyla /obrocono,
az kod sie znajdzie (max 3 obroty). Potem: KAUCYJNA -> punkt "kaucja", reszta -> punkt "inne".

Butelke chwytac za szyjke, chwytakiem w dol: wtedy obrot nadgarstka kreci butelka wokol jej osi.

    python butelki.py zapisz odbior|kamera|kaucja|inne|czekaj   obecna poza RoArma -> punkt
                        (ramie ustaw strona http://<roarm_ip>/ albo: python ../Raspberry/roarm_console.py)
    python butelki.py idz kamera       jedz do punktu (sprawdzenie)
    python butelki.py punkty           pokaz zapisane punkty
    python butelki.py raz              jedna butelka
    python butelki.py                  butelka za butelka: ENTER = nastepna, Q = koniec
    python butelki.py --test           logika bez sprzetu (RoArm i SO-101 udawane)

Wymaga: dzialajacego ramie.py (na Pi usluga dorm-keeper) i RoArma w tej samej sieci.
"""
import json
import os
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
    "podejscie_mm": 80,           # nad punktem odbioru/odlozenia najpierw tyle wyzej
    "chwyt_otwarty": 1.57,
    "chwyt_zamkniety": 3.14,
    "obroty": [1.57, -1.57, 3.14],  # rad wzgledem pozy "kamera": +90, -90, 180 st. = 4 strony butelki
    "pauza_obrotu": 1.5,          # s na obrot nadgarstka (goto czeka tylko na x y z)
    "czas_oceny": 15.0,           # s czekania na wynik z kamery przy kazdym ustawieniu butelki
    "punkty": {},
}
NAZWY = ("odbior", "kamera", "kaucja", "inne", "czekaj")


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


def jedz(arm, p, g=None, dz=0.0, droll=0.0, spd=0.2, tol=15.0, timeout=12.0):
    """Do punktu p (+dz mm w gore, +droll rad obrotu). Jak RoArm.goto, ale z obrotem nadgarstka (r)."""
    r = max(-3.14, min(3.14, p["r"] + droll))
    cel = {"x": p["x"], "y": p["y"], "z": p["z"] + dz}
    arm.send({"T": 104, **{k: round(v, 1) for k, v in cel.items()}, "t": round(p["t"], 3), "r": round(r, 3),
              "g": round(DOMYSLNE["chwyt_otwarty"] if g is None else g, 3), "spd": spd})
    if arm.mock:
        arm.fb.update(cel, r=r)
        return
    koniec = time.monotonic() + timeout
    while time.monotonic() < koniec:
        time.sleep(0.3)
        w = arm.where()
        if max(abs(w[k] - cel[k]) for k in cel) < tol:
            return
    raise TimeoutError(f"RoArm nie dojechal do {cel} (jest {arm.where()})")


class SO101:
    """Wynik z ramie.py przez HTTP."""

    def __init__(self, url):
        self.url = url.rstrip("/")

    def wynik(self):
        return requests.get(self.url + "/wynik", timeout=3).json()

    def obrocono(self):
        requests.get(self.url + "/obrocono", timeout=3)


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


def jedna_butelka(arm, so, cfg, log=print):
    pk, spd, dz = cfg["punkty"], cfg["spd"], cfg["podejscie_mm"]
    brak = [n for n in ("odbior", "kamera", "kaucja", "inne") if n not in pk]
    if brak:
        raise SystemExit(f"brak punktow {brak}: python butelki.py zapisz <nazwa>")
    otw, zam = cfg["chwyt_otwarty"], cfg["chwyt_zamkniety"]

    log("1. biore butelke")
    jedz(arm, pk["odbior"], otw, dz=dz, spd=spd)
    jedz(arm, pk["odbior"], otw, spd=spd)
    arm.gripper(zam)
    jedz(arm, pk["odbior"], zam, dz=dz, spd=spd)

    try:
        poprzedni = klucz(so.wynik())
    except requests.RequestException:
        poprzedni = None
    log("2. pokazuje kamerze")
    jedz(arm, pk["kamera"], zam, spd=spd)
    wynik = czekaj_na_wynik(so, poprzedni, cfg["czas_oceny"])
    for i, obrot in enumerate(cfg["obroty"], 1):
        if wynik and wynik["wynik"] != "BRAK_KODU":
            break
        log(f"   {'brak kodu' if wynik else 'brak wyniku'} -> obracam butelke ({i}/{len(cfg['obroty'])})")
        poprzedni = klucz(wynik) or poprzedni
        jedz(arm, pk["kamera"], zam, droll=obrot, spd=spd)
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
    jedz(arm, pk[cel], zam, dz=dz, spd=spd)
    jedz(arm, pk[cel], zam, spd=spd)
    arm.gripper(otw)
    jedz(arm, pk[cel], otw, dz=dz, spd=spd)
    if "czekaj" in pk:
        jedz(arm, pk["czekaj"], otw, spd=spd)
    return {"wynik": rodzaj, "pojemnik": cel, "ocena": wynik}


def test():
    """Bez sprzetu: pierwsze ustawienie bez kodu, po obrocie kod kaucyjny -> pojemnik 'kaucja'."""
    arm = RoArm("mock", mock=True)
    wysylane = []
    arm.send = lambda c: wysylane.append(c)
    arm.gripper = lambda g: wysylane.append({"g": g})
    p = {"x": 200, "y": 0, "z": 100, "t": 1.57, "r": 0.0}
    cfg = dict(DOMYSLNE, czas_oceny=0.5, pauza_obrotu=0, punkty={n: dict(p, y=i * 50) for i, n in enumerate(NAZWY)})

    class FakeSO:
        odp = [{"stan": "brak inspekcji"}, {"stan": "WYNIK", "wynik": "BRAK_KODU", "czas": "1", "kod": None},
               {"stan": "WYNIK", "wynik": "KAUCYJNA", "czas": "2", "kod": "5900541012218", "nazwa": "Zywiec"}]
        obrocen = 0

        def wynik(self):
            return self.odp[min(self.obrocen + 1, 2)] if len(wysylane) > 4 else self.odp[0]

        def obrocono(self):
            self.obrocen += 1

    so = FakeSO()
    w = jedna_butelka(arm, so, cfg, log=lambda *_: None)
    assert w["pojemnik"] == "kaucja" and so.obrocen == 1, w
    assert any(c.get("r") == 1.57 for c in wysylane), "obrot nadgarstka nie wyslany"
    assert all(c.get("T") != 0 for c in wysylane)
    assert arm.fb["y"] == 200, "na koniec punkt 'czekaj'"
    # bez wyniku wcale -> 3 obroty i pojemnik "inne"
    so2 = FakeSO()
    so2.wynik = lambda: {"stan": "SZUKAM"}
    assert jedna_butelka(arm, so2, cfg, log=lambda *_: None)["pojemnik"] == "inne"
    print("test ok")


def main():
    if "--test" in sys.argv:
        return test()
    cfg = wczytaj()
    cmd = sys.argv[1:2] or [""]
    arm = RoArm(cfg["roarm_ip"])
    if cmd == ["punkty"]:
        print(f"RoArm {cfg['roarm_ip']}")
        for n in NAZWY:
            print(f"  {n:8s}", cfg["punkty"].get(n, "- brak -"))
    elif cmd == ["zapisz"] and sys.argv[2:3] and sys.argv[2] in NAZWY:
        w = arm.where()
        # bez chwytaka: jego odczyt bywa 0 (firmware), a otwarcie/zamkniecie i tak wynika z etapu
        cfg["punkty"][sys.argv[2]] = {"x": w["x"], "y": w["y"], "z": w["z"], "t": w["tit"], "r": w.get("r", 0.0)}
        zapisz(cfg)
        print("zapisano", sys.argv[2], cfg["punkty"][sys.argv[2]])
    elif cmd == ["idz"] and sys.argv[2:3] and sys.argv[2] in cfg["punkty"]:
        jedz(arm, cfg["punkty"][sys.argv[2]], cfg["chwyt_otwarty"], spd=cfg["spd"])
    elif cmd in (["raz"], [""]):
        so = SO101(cfg["so101_url"])
        so.wynik()  # od razu blad, jesli ramie.py nie dziala
        while True:
            print(jedna_butelka(arm, so, cfg)["wynik"])
            if cmd == ["raz"] or input("\nENTER = nastepna butelka, Q = koniec: ").strip().lower() == "q":
                break
    else:
        print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, TimeoutError, requests.RequestException) as e:
        sys.exit(f"BLAD: {e}")
