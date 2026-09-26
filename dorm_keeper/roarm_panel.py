"""Panel RoArma w przegladarce: http://<pi>:8765/roarm_panel (serwuje ramie.py).

Sterowanie przytrzymaniem (puszczasz = stoi), obraz z kamery SO-101 obok, ochrona serw:
wolne male kroki, limity zasiegu i wysokosci, podglad obciazen, STOP przy przeciazeniu.
Stad tez: nauka chwytu + samokalibracja (butelki.py kalibruj gotowe) i start/stop zbierania butelek.
"""
import json
import os
import subprocess
import sys
import threading
import time

import requests

_katalog = os.path.dirname(os.path.abspath(__file__))

PREDKOSCI = {"wolno": (20.0, 0.12), "normalnie": (40.0, 0.2), "szybko": (60.0, 0.25)}  # (mm/s, spd firmware <=0.25)
WYPRZEDZENIE = 0.6               # s: cel jest tyle ruchu przed ramieniem -> jedzie rowno, zamiast krok-stop-krok
PODNIES_MM = 100.0               # P: chwyc i podnies o tyle
SERWO_GORACE = (60, 65)          # st. C: od 65 jazda zablokowana (serwo odpoczywa), od 60 w dol znowu wolno
SERWA_ROARM = ["podstawa", "bark 1", "bark 2", "lokiec", "nadgarstek", "obrot", "chwytak"]  # kolejnosc "temp" z /ws
Z_ZAKRES = (-150.0, 350.0)       # mm
OBCIAZENIE_STOP = 350            # |obciazenie| barku/lokcia (jednostki firmware): ponad = STOP i trzymaj
PODTRZYMANIE = 0.35              # s: bez sygnalu z przegladarki ramie staje (puszczony przycisk, zerwane WiFi)
# ruch po ludzku (wzgledem podstawy RoArma): obrot calej podstawy, wysuniecie od podstawy, wysokosc, chwytak
KIERUNKI = {"obrot+": ("obrot", 1), "obrot-": ("obrot", -1), "wysun": ("zasieg", 1), "cofnij": ("zasieg", -1),
            "z+": ("z", 1), "z-": ("z", -1), "t+": ("t", 1), "t-": ("t", -1), "r+": ("r", 1), "r-": ("r", -1)}


def _konfig():
    sys.path.append(_katalog)
    import butelki

    return butelki.wczytaj()


class PanelRoArma:
    def __init__(self):
        cfg = _konfig()
        self.ip, self.zasieg = cfg["roarm_ip"], cfg["zasieg_mm"]
        self.otw, self.zam = cfg["chwyt_otwarty"], cfg["chwyt_zamkniety"]
        self.cel = None           # cel sterowania x y z t r (od pierwszego odczytu)
        self.t_cmd = self.r_cmd = 0.0  # zapamietane pochylenie i obrot chwytaka (zmieniaja je tylko J/L i U/O)
        self.t_ruch = 0.0
        self.bez_momentu = True   # dopoki nie wiadomo - bez jazdy
        self.g = self.otw
        self.kier, self.kier_t = None, 0.0
        self.predkosc = "normalnie"
        self.stan = {"polaczony": False, "komunikat": "lacze sie z RoArmem..."}
        self.polecenia = []       # jednorazowe: chwytak, stop
        self.proces, self.proces_nazwa, self.log = None, "", []
        self._lock = threading.Lock()
        self.goracy = False
        threading.Thread(target=self._petla, daemon=True).start()
        threading.Thread(target=self._websocket, daemon=True).start()

    def _websocket(self):
        """Temperatury 7 serw (T:1051 "temp", ~1.5/s) i alarmy (T:-15: przeciazenie, przegrzanie, napiecie)
        z WebSocketu RoArma - te same dane, z ktorych ostrzega jego wlasna strona."""
        try:
            import websocket
        except ImportError:
            self.stan["serwa"] = "brak biblioteki websocket-client (pip install websocket-client)"
            return
        while True:
            try:
                ws = websocket.create_connection(f"ws://{self.ip}/ws", timeout=5)
                while True:
                    d = json.loads(ws.recv())
                    if d.get("T") == 1051:
                        self._temperatury(d)
                    elif d.get("T") == -15:
                        self.stan["alarmy"] = {"przeciazenie": bool(d.get("Stalltor")),
                                               "przegrzanie": bool(d.get("Stalltep")),
                                               "napiecie": {1: "za wysokie", 2: "za niskie"}.get(d.get("Stallvol"))}
            except Exception:  # RoArm wylaczony / restart - probuj dalej
                self.stan.pop("temp_serw", None)
                time.sleep(2)

    def _temperatury(self, d):
        """T:1051 "temp" -> stan + blokada przegrzania. Z /ws (WiFi) albo z /js (USB przez roarm_usb.py - bez /ws)."""
        temp = d.get("temp")
        if isinstance(temp, list) and temp:
            self.stan["temp_serw"] = dict(zip(SERWA_ROARM, temp))
            self.stan["temp_t"] = time.time()
            najw = max(temp)
            self.goracy = najw >= SERWO_GORACE[1] or (self.goracy and najw > SERWO_GORACE[0])

    # ------------------------------------------------------------------ RoArm
    def _js(self, cmd, timeout=1.5):
        r = requests.get(f"http://{self.ip}/js", params={"json": json.dumps(cmd, separators=(",", ":"))},
                         timeout=timeout)
        return r.text

    def _gdzie(self):
        d = json.loads(self._js({"T": 105}))
        if d.get("T") != 1051 or d.get("x") is None:
            raise RuntimeError("RoArm nie podaje pozycji (zasilanie serw?)")
        self._temperatury(d)
        return d

    def _jedz(self, cel, spd=0.2):
        try:  # RoArm nie odpowiada, dopoki jedzie - polecenie i tak doszlo
            self._js({"T": 104, "x": round(cel["x"], 1), "y": round(cel["y"], 1), "z": round(cel["z"], 1),
                      "t": round(cel["t"], 3), "r": round(cel["r"], 3), "g": round(self.g, 3), "spd": spd})
        except requests.Timeout:
            pass

    def _przed_ramieniem(self, d, dt):
        """Nowy cel. Polozenie: WYPRZEDZENIE s ruchu przed obecna pozycja (jedzie rowno). Kat i obrot chwytaka:
        zapamietane (t_cmd, r_cmd) - jak hak dzwigu w SO-101: gora/dol/dalej/blizej nie zmieniaja pochylenia."""
        import math

        os_, znak = KIERUNKI[self.kier]
        v = PREDKOSCI[self.predkosc][0]
        if os_ in ("t", "r"):  # pochyl / obroc chwytak w miejscu: polozenie = ostatni cel
            katowa = v * 0.012  # rad/s: ~0.5 rad/s przy "normalnie"
            if os_ == "t":
                self.t_cmd = min(max(self.t_cmd + znak * katowa * dt, -3.14), 3.14)
            else:
                self.r_cmd = min(max(self.r_cmd + znak * katowa * dt, -3.14), 3.14)
            return dict(self.cel, t=self.t_cmd, r=self.r_cmd)
        v *= WYPRZEDZENIE  # mm przed ramieniem
        rho, kat, z = math.hypot(d["x"], d["y"]), math.atan2(d["y"], d["x"]), d["z"]
        if os_ == "zasieg":
            rho += znak * v
        elif os_ == "obrot":
            kat += znak * v / max(rho, 100.0)
        else:
            z += znak * v
        return self._ogranicz({"x": rho * math.cos(kat), "y": rho * math.sin(kat), "z": z,
                               "t": self.t_cmd, "r": self.r_cmd})

    def _trzymaj(self, d, komunikat):
        self.cel = {"x": d["x"], "y": d["y"], "z": d["z"], "t": self.t_cmd, "r": self.r_cmd}
        self._jedz(self.cel)
        self.kier = None
        self.stan["komunikat"] = komunikat

    def _moment_tutaj(self, d):
        """Serwa bez momentu (ramie przestawione recznie): cel kazdego serwa = jego zmierzony kat, potem moment wl.
        Ramie zostaje, gdzie jest - zadnej stalej pozy (tam moze juz cos stac, np. pojazd)."""
        self._js({"T": 102, "base": d["b"], "shoulder": d["s"], "elbow": d["e"], "wrist": d["t"], "roll": d["r"],
                  "hand": d["g"], "spd": 50, "acc": 10})
        self._js({"T": 210, "cmd": 1})  # tylko EnableTorque (cmd 0 najpierw jedzie do stalej pozy!)
        self.cel = {"x": d["x"], "y": d["y"], "z": d["z"], "t": d["tit"], "r": d.get("r", 0.0)}
        self.t_cmd, self.r_cmd, self.g = d["tit"], d.get("r", 0.0), d["g"]
        self.bez_momentu = False
        self.stan["komunikat"] = "silniki wlaczone tutaj - przytrzymaj przycisk, zeby jechac"

    def _chwytak(self, zamknij):
        self.g = self.zam if zamknij else self.otw
        try:
            self._js({"T": 106, "cmd": self.g, "spd": 0, "acc": 0})
        except requests.Timeout:
            pass
        self.stan["komunikat"] = "chwytak zamkniety (chwyta)" if zamknij else "chwytak otwarty (puscil)"

    def _ogranicz(self, c):
        import math

        lo, hi = self.zasieg
        r = math.hypot(c["x"], c["y"])
        if r > hi or r < lo:
            k = (hi if r > hi else lo) / max(r, 1e-6)
            c["x"], c["y"] = c["x"] * k, c["y"] * k
        c["z"] = min(max(c["z"], Z_ZAKRES[0]), Z_ZAKRES[1])
        c["t"] = min(max(c["t"], -3.14), 3.14)
        c["r"] = min(max(c["r"], -3.14), 3.14)
        return c

    def _petla(self):
        t_odczyt = 0.0
        while True:
            time.sleep(0.05)
            teraz = time.time()
            if self.proces and self.proces.poll() is None:
                continue  # kalibracja / zbieranie steruje RoArmem - panel nie przeszkadza
            try:
                jedzie = bool(self.kier) and teraz - self.kier_t < PODTRZYMANIE and not self.bez_momentu \
                    and not self.goracy
                if self.cel is None or teraz - t_odczyt > (0.2 if jedzie else 0.6):
                    d = self._gdzie()
                    t_odczyt = time.time()
                    self.stan.update(polaczony=True, x=d["x"], y=d["y"], z=d["z"], t=d["tit"], r=d.get("r", 0.0),
                                     g=d.get("g"), bark=d.get("tS", 0), lokiec=d.get("tE", 0), podstawa=d.get("tB", 0))
                    # wszystkie obciazenia 0 = serwa bez momentu: katy z odczytu bywaja wtedy falszywe (+-pi),
                    # a ruch "wzgledem nich" potrafi obrocic cale ramie - dlatego najpierw _moment_tutaj
                    self.bez_momentu = not any(d.get(k) for k in ("tB", "tS", "tE", "tT", "tR"))
                    if self.cel is None:  # tylko zapamietaj - zadnego ruchu przy polaczeniu
                        self.cel = {"x": d["x"], "y": d["y"], "z": d["z"], "t": d["tit"], "r": d.get("r", 0.0)}
                        self.t_cmd, self.r_cmd = d["tit"], d.get("r", 0.0)
                        if 1.0 <= (d.get("g") or 0) <= 3.5:
                            self.g = d["g"]  # chwytak zostaje, jak jest (inaczej pierwszy ruch by go otworzyl)
                        self.stan["komunikat"] = ("serwa bez momentu - pierwszy przycisk ruchu wlaczy je w miejscu" if self.bez_momentu
                                                  else "gotowy - przytrzymaj przycisk, zeby jechac")
                    elif max(abs(d.get("tS", 0)), abs(d.get("tE", 0))) > OBCIAZENIE_STOP:
                        self._trzymaj(d, "PRZECIAZENIE - ramie trzyma pozycje. Podnies je wyzej / blizej podstawy")
                        jedzie = False
                    if jedzie:  # cel stale ~0.6 s przed ramieniem: jedzie rowno; puszczony przycisk = staje 1-3 cm dalej
                        dt, self.t_ruch = min(max(t_odczyt - self.t_ruch, 0.05), 0.4), t_odczyt
                        self.cel = self._przed_ramieniem(d, dt)
                        self._jedz(self.cel, spd=PREDKOSCI[self.predkosc][1])
                        self.stan["komunikat"] = "jade"
                with self._lock:
                    polecenia, self.polecenia = self.polecenia, []
                for p in polecenia:
                    if p == "stop":
                        if self.bez_momentu:  # falszywe katy - nie wysylaj "trzymaj tutaj"
                            self.kier, self.stan["komunikat"] = None, "STOP"
                        else:
                            self._trzymaj(self._gdzie(), "STOP - trzyma pozycje")
                    elif p in ("otworz", "zamknij", "przelacz"):  # przelacz = SPACJA: chwyc / pusc (jak w SO-101)
                        zamknij = p == "zamknij" or (p == "przelacz" and self.g < (self.otw + self.zam) / 2)
                        self._chwytak(zamknij)
                    elif p == "podnies" and not self.bez_momentu:  # P: chwyc i podnies o 10 cm
                        self._chwytak(True)
                        time.sleep(0.8)  # palce sie zamykaja (brak czujnika na chwytaku)
                        d = self._gdzie()
                        self.cel = self._ogranicz({"x": d["x"], "y": d["y"], "z": d["z"] + PODNIES_MM,
                                                   "t": self.t_cmd, "r": self.r_cmd})
                        self._jedz(self.cel, spd=PREDKOSCI["wolno"][1])
                        self.stan["komunikat"] = f"chwycil i podnosi o {PODNIES_MM:.0f} mm"
                if self.kier and self.goracy:
                    self.kier = None
                    najw = max((self.stan.get("temp_serw") or {"?": 0}).items(), key=lambda kv: kv[1])
                    self.stan["komunikat"] = (f"SERWO GORACE ({najw[0]} {najw[1]} C) - jazda zablokowana do "
                                              f"{SERWO_GORACE[0]} C. Nizej / blizej podstawy odciaza bark")
                elif self.kier and self.bez_momentu:
                    self._moment_tutaj(self._gdzie())  # nastepny obieg juz jedzie
                elif self.kier and teraz - self.kier_t >= PODTRZYMANIE:
                    self.kier = None  # puszczony przycisk: bez nowych celow ramie dojezdza do ostatniego i stoi
                    self.stan["komunikat"] = "stoi"
            except (requests.RequestException, RuntimeError, ValueError) as e:
                self.stan.update(polaczony=False, komunikat=f"RoArm nie odpowiada ({type(e).__name__}) - "
                                                             "zasilanie 7.4-8.4 V? WiFi?")
                self.cel = None
                time.sleep(1.0)

    # ------------------------------------------------------------------ butelki.py (kalibracja / zbieranie)
    def uruchom(self, nazwa, argumenty):
        if self.proces and self.proces.poll() is None:
            return False
        self.kier = None
        env = dict(os.environ, SO101_URL="http://localhost:8765", PYTHONUNBUFFERED="1")
        self.proces = subprocess.Popen([sys.executable, "-u", "butelki.py", *argumenty], cwd=_katalog, env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                       text=True, errors="replace")
        self.proces_nazwa, self.log = nazwa, [f"--- {nazwa} ---"]

        def czytaj(p=self.proces):
            for linia in p.stdout:
                self.log = (self.log + [linia.rstrip()])[-200:]
            self.log.append(f"--- koniec ({nazwa}, kod {p.wait()}) ---")
            self.cel = None  # po procesie odczytaj pozycje od nowa

        threading.Thread(target=czytaj, daemon=True).start()
        return True

    def zatrzymaj_proces(self):
        if self.proces and self.proces.poll() is None:
            import signal

            self.proces.send_signal(signal.SIGINT)  # butelki.py konczy sie grzecznie ("Koniec.")
            try:
                self.proces.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proces.kill()
        with self._lock:
            self.polecenia.append("stop")

    def dane(self):
        dziala = bool(self.proces and self.proces.poll() is None)
        return dict(self.stan, predkosc=self.predkosc, proces=self.proces_nazwa if dziala else "", log=self.log[-40:],
                    kalibracja=bool(_konfig().get("kalibracja")))


_panel = None


def obsluz(h, sciezka, q):
    """Obsluga /roarm_panel i /roarm/... (h = handler z ramie.py: ma _json)."""
    global _panel
    if sciezka == "/roarm_panel":
        tresc = HTML.encode()
        h.send_response(200)
        h.send_header("Content-Type", "text/html; charset=utf-8")
        h.send_header("Content-Length", str(len(tresc)))
        h.end_headers()
        h.wfile.write(tresc)
        return
    if _panel is None:
        _panel = PanelRoArma()
    p = _panel
    if sciezka == "/roarm/stan":
        return h._json(p.dane())
    if sciezka == "/roarm/ruch" and q.get("k") in KIERUNKI:
        p.kier, p.kier_t = q["k"], time.time()
        return h._json({"ok": True})
    if sciezka == "/roarm/predkosc" and q.get("v") in PREDKOSCI:
        p.predkosc = q["v"]
        return h._json({"ok": True})
    if sciezka == "/roarm/polecenie" and q.get("p") in ("stop", "otworz", "zamknij", "przelacz", "podnies"):
        with p._lock:
            p.polecenia.append(q["p"])
        return h._json({"ok": True})
    if sciezka == "/roarm/uruchom" and q.get("co") in ("kalibracja", "zbieranie"):
        arg = ["kalibruj", "gotowe"] if q["co"] == "kalibracja" else []
        return h._json({"ok": p.uruchom(q["co"], arg)})
    if sciezka == "/roarm/zatrzymaj":
        p.zatrzymaj_proces()
        return h._json({"ok": True})
    h._json({"blad": "nieznane polecenie panelu RoArma"}, 404)


HTML = """<!doctype html><html lang="pl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>RoArm - panel</title><style>
:root{--tlo:#0b0f14;--panel:#121922;--linia:#243040;--tekst:#e8eef5;--przyg:#8a9aac;--akcent:#4cc2ff;--ok:#2ecc71;--zle:#ff5c5c;--uwaga:#ffb020}
*{box-sizing:border-box}body{margin:0;background:var(--tlo);color:var(--tekst);font:15px system-ui,"Segoe UI",sans-serif}
main{display:grid;grid-template-columns:minmax(0,1.4fr) minmax(320px,1fr);gap:14px;padding:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}
h1{font-size:20px;margin:0 0 4px}h2{font-size:13px;text-transform:uppercase;letter-spacing:1px;color:var(--przyg);margin:14px 0 8px}
.karta{background:var(--panel);border:1px solid var(--linia);border-radius:12px;padding:14px}
img{width:100%;border-radius:10px;background:#000;display:block}
#kom{font-weight:700;padding:10px 12px;border-radius:8px;background:#1a2330;margin:8px 0}
#kom.zle{background:#4d1f22;color:#ffb3b3}
.poz{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;font-variant-numeric:tabular-nums}
.poz div{background:#1a2330;border-radius:8px;padding:6px 8px}.poz b{display:block;font-size:18px}.poz span{font-size:12px;color:var(--przyg)}
.pasek{height:8px;background:#1a2330;border-radius:4px;overflow:hidden;margin:3px 0 8px}.pasek i{display:block;height:100%;background:var(--ok);width:0}
.siatka{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
button{font:inherit;padding:12px 6px;border:0;border-radius:8px;background:#243040;color:var(--tekst);cursor:pointer;touch-action:none;user-select:none}
button:active,button.on{background:var(--akcent);color:#06121e}button small{display:block;font-size:11px;opacity:.7}
#stop{background:var(--zle);font-weight:800;font-size:18px}.duzy{width:100%;margin-top:6px}
.zielony{background:#1b5e3a}.czerwony{background:#6b2124}
ol{margin:6px 0 0;padding-left:20px;color:var(--przyg);line-height:1.5}ol b{color:var(--tekst)}
pre{background:#05080b;border-radius:8px;padding:8px;height:170px;overflow:auto;font-size:12px;margin:8px 0 0;white-space:pre-wrap}
.stan .wiersz{display:grid;grid-template-columns:150px 1fr 74px;gap:8px;align-items:center;font-size:13px}
.stan .wiersz .pasek{margin:6px 0}.stan b{text-align:right;font-variant-numeric:tabular-nums}
#uwagi div{margin-top:6px;padding:7px 10px;border-radius:8px;background:#4d1f22;color:#ffb3b3;font-weight:700;font-size:13px}
#uwagi div.zolte{background:#4d3a12;color:#ffd98a}
details{margin-top:8px;font-size:13px;color:var(--przyg)}#serwa{display:grid;grid-template-columns:1fr 1fr;gap:2px 12px;margin-top:6px;font-variant-numeric:tabular-nums}
</style></head><body><main>
<div><div class="karta"><h1>RoArm - sterowanie</h1><span style="color:var(--przyg)">obraz z kamery SO-101 (to, co widzi program)</span>
<img src="/podglad?czysty=1" alt="kamera SO-101"></div>
<div class="karta" style="margin-top:14px"><h2>Nauka chwytu i kalibracja</h2><ol>
<li>Postaw <b>jedna butelke</b> na stole: widac jej dol na obrazie, 12-38 cm od podstawy RoArma.</li>
<li>Otworz chwytak i najedz nim <b>na szyjke butelki</b> - dokladnie tak, jak ma ja chwytac.</li>
<li>Kliknij <b>Gotowe</b> - RoArm sam przestawi butelke kilka razy i nauczy sie, gdzie co jest (~1.5 min).</li></ol>
<button class="duzy zielony" id="kal">Gotowe - ucz chwytu i kalibruj</button>
<h2>Zbieranie butelek</h2><div class="siatka" style="grid-template-columns:1fr 1fr">
<button class="zielony" id="zb">Start zbierania</button><button class="czerwony" id="zat">Zatrzymaj</button></div>
<pre id="log">(tu pojawi sie przebieg kalibracji / zbierania)</pre></div></div>
<div class="karta"><div id="kom">lacze sie...</div>
<div class="poz"><div><span>wysuniecie</span><b id="px">-</b></div><div><span>obrot podstawy</span><b id="py">-</b></div><div><span>wysokosc</span><b id="pz">-</b></div></div>
<h2>Stan systemu</h2><div class="stan">
<div class="wiersz"><span>Raspberry CPU</span><div class="pasek"><i id="s-cpu"></i></div><b id="t-cpu">-</b></div>
<div class="wiersz"><span>Raspberry RAM</span><div class="pasek"><i id="s-ram"></i></div><b id="t-ram">-</b></div>
<div class="wiersz"><span>Raspberry temp.</span><div class="pasek"><i id="s-temp"></i></div><b id="t-temp">-</b></div>
<div class="wiersz"><span>SO-101 najcieplejsze serwo</span><div class="pasek"><i id="s-so"></i></div><b id="t-so">-</b></div>
<div class="wiersz"><span>RoArm najcieplejsze serwo</span><div class="pasek"><i id="s-ra"></i></div><b id="t-ra">-</b></div>
<div class="wiersz"><span>RoArm obciazenie barku</span><div class="pasek"><i id="obark"></i></div><b id="t-bark">-</b></div>
<div class="wiersz"><span>RoArm obciazenie lokcia</span><div class="pasek"><i id="olok"></i></div><b id="t-lok">-</b></div>
<div id="uwagi"></div><details><summary>wszystkie serwa</summary><div id="serwa"></div></details></div>
<h2>Skad patrzysz na RoArma?</h2><div class="siatka" style="grid-template-columns:1fr 1fr">
<button data-widok="przod">Stoje PRZED nim<small>twarza do robota</small></button><button data-widok="tyl">Stoje ZA nim<small>patrze tam, gdzie on</small></button></div>
<h2>Ruch - jak SO-101 (przytrzymaj, puszczasz = stoi; chwytak trzyma swoj kat jak hak dzwigu)</h2><div class="siatka">
<button data-k="z+">GORA<small>W</small></button><button data-k="wysun">DALEJ<small>R - od podstawy</small></button><button data-l="1">LEWO<small>A</small></button>
<button data-k="z-">DOL<small>S</small></button><button data-k="cofnij">BLIZEJ<small>F - do podstawy</small></button><button data-l="-1">PRAWO<small>D</small></button>
<button data-k="t+">pochyl chwytak<small>J</small></button><button data-k="t-">pochyl chwytak<small>L</small></button><button id="stop">STOP<small>B</small></button>
<button data-k="r+">obroc chwytak<small>U</small></button><button data-k="r-">obroc chwytak<small>O</small></button><span></span></div>
<h2>Chwytak</h2><div class="siatka">
<button data-p="przelacz">CHWYC / PUSC<small>SPACJA</small></button><button data-p="podnies" class="zielony">CHWYC I PODNIES<small>P - 10 cm w gore</small></button><button data-p="otworz">otworz<small>Z</small></button></div>
<h2>Predkosc</h2><div class="siatka"><button data-v="wolno">1 wolno<small>celowanie</small></button><button data-v="normalnie" class="on">2 normalnie</button><button data-v="szybko">3 szybko</button></div>
</div></main><script>
const $=id=>document.getElementById(id),get=u=>fetch(u).then(r=>r.json()).catch(()=>({}));
let trzymany=null,petla=null;
function jedz(k){if(trzymany===k)return;stoj();trzymany=k;get('/roarm/ruch?k='+encodeURIComponent(k));petla=setInterval(()=>get('/roarm/ruch?k='+encodeURIComponent(k)),120)}
function stoj(){clearInterval(petla);petla=null;trzymany=null}
// "w lewo" = Twoje lewo: stojac przed robotem jego lewo to Twoje prawo (obrot podstawy + = lewo robota)
let widok='przod';try{widok=localStorage.getItem('roarm_widok')||'przod'}catch(e){}
const lewo=znak=>(widok==='tyl'?1:-1)*znak>0?'obrot+':'obrot-';
function ustawWidok(w){widok=w;try{localStorage.setItem('roarm_widok',w)}catch(e){}
 document.querySelectorAll('[data-widok]').forEach(b=>b.classList.toggle('on',b.dataset.widok===w))}ustawWidok(widok);
document.querySelectorAll('[data-widok]').forEach(b=>b.onclick=()=>ustawWidok(b.dataset.widok));
document.querySelectorAll('[data-k],[data-l]').forEach(b=>{const k=()=>b.dataset.k||lewo(+b.dataset.l);
 b.onpointerdown=e=>{b.setPointerCapture(e.pointerId);jedz(k())};b.onpointerup=b.onpointercancel=stoj});
document.querySelectorAll('[data-p]').forEach(b=>b.onclick=()=>get('/roarm/polecenie?p='+b.dataset.p));
document.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>predkosc(b.dataset.v));
$('stop').onclick=()=>{stoj();get('/roarm/polecenie?p=stop')};
$('kal').onclick=()=>{if(confirm('Chwytak jest otwarty wokol szyjki butelki? RoArm zacznie sam przestawiac butelke.'))get('/roarm/uruchom?co=kalibracja')};
$('zb').onclick=()=>get('/roarm/uruchom?co=zbieranie');$('zat').onclick=()=>get('/roarm/zatrzymaj');
// klawisze jak w SO-101 (ramie.py): W/S gora/dol, R/F dalej/blizej, A/D lewo/prawo, J/L pochyl, U/O obroc
const KL={w:'z+',s:'z-',r:'wysun',f:'cofnij',j:'t+',l:'t-',u:'r+',o:'r-'};
const PRED={'1':'wolno','2':'normalnie','3':'szybko'};
function predkosc(v){get('/roarm/predkosc?v='+v);document.querySelectorAll('[data-v]').forEach(x=>x.classList.toggle('on',x.dataset.v===v))}
const klawisz=k=>k==='a'?lewo(1):k==='d'?lewo(-1):KL[k];
onkeydown=e=>{const key=e.key.toLowerCase(),k=klawisz(key);
 if(k){jedz(k);e.preventDefault()}
 else if(key==='b'){stoj();get('/roarm/polecenie?p=stop')}
 else if(e.repeat){}
 else if(key===' '){get('/roarm/polecenie?p=przelacz');e.preventDefault()}
 else if(key==='p')get('/roarm/polecenie?p=podnies');
 else if(key==='z')get('/roarm/polecenie?p=otworz');
 else if(key==='x')get('/roarm/polecenie?p=zamknij');
 else if(PRED[key])predkosc(PRED[key])};
onkeyup=e=>{if(klawisz(e.key.toLowerCase())===trzymany)stoj()};onblur=stoj;
function pasek(el,v){const p=Math.min(100,Math.abs(v||0)/350*100);el.style.width=p+'%';el.style.background=p>80?'var(--zle)':p>55?'var(--uwaga)':'var(--ok)'}
// miernik: pasek do "max", kolor od progow zolty/czerwony, tekst obok
function miernik(id,v,max,zolty,czerwony,tekst){const el=$('s-'+id),t=$('t-'+id);
 if(v==null||isNaN(v)){el.style.width='0';t.textContent='-';return}
 el.style.width=Math.min(100,Math.max(0,v)/max*100)+'%';el.style.background=v>=czerwony?'var(--zle)':v>=zolty?'var(--uwaga)':'var(--ok)';t.textContent=tekst}
const najcieplejsze=o=>o?Object.entries(o).reduce((a,b)=>b[1]>a[1]?b:a,['',-1]):null;
let roarm={};
async function odswiezSystem(){const s=await get('/system'),uw=[];
 miernik('cpu',s.cpu,100,70,90,s.cpu!=null?Math.round(s.cpu)+' %':'-');
 miernik('ram',s.ram,100,75,90,s.ram!=null?Math.round(s.ram)+' %':'-');
 miernik('temp',s.temp,90,75,82,s.temp!=null?s.temp.toFixed(0)+' C'+(s.zegar_mhz?' '+(s.zegar_mhz/1000).toFixed(1)+'GHz':''):'-');
 const so=najcieplejsze(s.so101&&s.so101.temp);miernik('so',so&&so[1],75,55,65,so&&so[1]>=0?so[1]+' C':'-');
 const ra=najcieplejsze(roarm.temp_serw);miernik('ra',ra&&ra[1],75,55,65,ra&&ra[1]>=0?ra[1]+' C':'-');
 if(s.zbija_zegar)uw.push(['Raspberry sie przegrzewa - zegar zbity (wentylator!)','']);
 if(s.niskie_napiecie)uw.push(['Raspberry: za niskie napiecie zasilacza','']);
 if(so&&so[1]>=55)uw.push(['SO-101: '+so[0]+' '+so[1]+' C',so[1]>=65?'':'zolte']);
 if(ra&&ra[1]>=55)uw.push(['RoArm: '+ra[0]+' '+ra[1]+' C'+(ra[1]>=65?' - jazda zablokowana, niech odpocznie':' - odciaz (nizej / blizej podstawy)'),ra[1]>=65?'':'zolte']);
 const al=roarm.alarmy||{};if(al.przeciazenie)uw.push(['RoArm: PRZECIAZENIE serwa (zablokowany ruch?)','']);
 if(al.przegrzanie)uw.push(['RoArm: PRZEGRZANIE serwa - wylacz i odczekaj','']);if(al.napiecie)uw.push(['RoArm: napiecie '+al.napiecie,'']);
 $('uwagi').innerHTML=uw.map(u=>`<div class="${u[1]}">${u[0]}</div>`).join('');
 const wsz=[];if(s.so101&&s.so101.temp)for(const[k,v]of Object.entries(s.so101.temp))wsz.push(`<span>SO-101 ${k}: ${v} C${s.so101.napiecie&&s.so101.napiecie[k]?' / '+s.so101.napiecie[k].toFixed(1)+' V':''}</span>`);
 if(roarm.temp_serw)for(const[k,v]of Object.entries(roarm.temp_serw))wsz.push(`<span>RoArm ${k}: ${v} C</span>`);
 $('serwa').innerHTML=wsz.join('')||'brak danych';setTimeout(odswiezSystem,1000)}odswiezSystem();
async function odswiez(){const d=await get('/roarm/stan');if(d.x!==undefined){$('px').textContent=Math.round(Math.hypot(d.x,d.y))+' mm';$('py').textContent=Math.round(Math.atan2(d.y,d.x)*180/Math.PI)+' st.';$('pz').textContent=Math.round(d.z)+' mm'}
 roarm=d;pasek($('obark'),d.bark);pasek($('olok'),d.lokiec);
 $('t-bark').textContent=d.bark!=null?Math.abs(d.bark):'-';$('t-lok').textContent=d.lokiec!=null?Math.abs(d.lokiec):'-';
 $('kom').textContent=d.proces?('trwa: '+d.proces+' (panel wstrzymany)'):(d.komunikat||'');$('kom').className=(!d.polaczony||/PRZECIAZ/.test(d.komunikat||''))?'zle':'';
 if(d.log&&d.log.length){const l=$('log');l.textContent=d.log.join('\\n');l.scrollTop=l.scrollHeight}
 $('zb').disabled=!d.kalibracja;$('zb').title=d.kalibracja?'':'najpierw kalibracja';setTimeout(odswiez,500)}odswiez();
</script></body></html>"""
