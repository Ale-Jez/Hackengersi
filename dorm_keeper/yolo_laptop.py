"""YOLO dla Raspberry: laptop albo Brev (GPU, bash Brev/setup.sh startuje go sam) wypatruje butelek za Pi (obie skale YOLO w ~25 ms), Pi oszczedza procesor.

Kamera i SO-101 zostaja na Pi (ramie.py dziala tam jako usluga). Ten program pobiera z Pi pomniejszone klatki,
liczy YOLO11n i odsyla ramki butelek. To laptop laczy sie z Pi, wiec zapora Windows nie przeszkadza.
Gdy ramie SLEDZI butelke, Pi liczy YOLO samo (szybka skala 320, ~18 kl/s): opoznienie WiFi (60-400 ms, zmienne)
rozbujaloby ramie. Laptop pomaga wiec przy szukaniu (dalekie butelki, mniej ciepla na Pi bez wentylatora).
Zamkniesz go (Ctrl+C) - Pi po pol sekundy liczy wszystko samo, nic sie nie psuje.

    python yolo_laptop.py                        Pi pod http://malina:8765 (Tailscale)
    python yolo_laptop.py http://malina.local:8765
"""
import os
import sys
import threading
import time

import cv2
import numpy as np
import requests

import ramie

PI = (sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PI_URL", "http://malina:8765")).rstrip("/")
WATKI = 2  # tyle klatek naraz w drodze: gdy jedna jedzie przez WiFi, druga sie liczy (WiFi to ~60 ms na klatke)
POLA = ("cx", "cy", "w", "h", "pewnosc", "klasa", "box")

_stat = {"n": 0, "yolo": 0.0, "widze": [], "polaczony": None}
_lock = threading.Lock()


def watek(detektor):
    sesja = requests.Session()
    nr, wynik = 0, None
    while True:
        try:
            # wynik poprzedniej klatki -> w odpowiedzi nastepna klatka (jedno zapytanie na klatke)
            r = sesja.post(f"{PI}/yolo", params={"nr": nr}, json=wynik, timeout=3)
        except requests.RequestException as e:
            with _lock:
                if _stat["polaczony"] is not False:
                    print(f"\nPi nie odpowiada ({type(e).__name__}) - dziala ramie.py na Pi? Probuje dalej...")
                _stat["polaczony"] = False
            nr, wynik = 0, None
            time.sleep(1)
            continue
        if r.status_code == 404:
            print("\nPi odpowiada, ale ma stary ramie.py bez /yolo - wgraj: python wgraj_na_pi.py")
            os._exit(1)
        with _lock:
            if not _stat["polaczony"]:
                print("Polaczono - laptop szuka butelek za Pi (gdy ramie sledzi, Pi liczy samo). "
                      "Ctrl+C konczy.")
                _stat["polaczony"] = True
        nr, wynik = 0, None
        if r.status_code != 200:  # 204: Pi nie mialo nowej klatki przez 1 s
            continue
        klatka = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
        if klatka is None:
            continue
        t = time.perf_counter()
        wynik = [{k: b[k] for k in POLA} for b in detektor.wykryj(klatka)]
        nr = int(r.headers.get("X-Nr", 0))
        with _lock:
            _stat["n"] += 1
            _stat["yolo"] += time.perf_counter() - t
            _stat["widze"] = wynik


def main():
    ramie.YOLO_PRZEPLOT = False  # laptop zdazy obie skale (320 + 640) w kazdej klatce - pewniejsze wykrycie
    detektor = ramie.DetektorButelek()  # jedna siec dla obu watkow (onnxruntime na to pozwala)
    print(f"Lacze sie z {PI} ...")
    for _ in range(WATKI):
        threading.Thread(target=watek, args=(detektor,), daemon=True).start()
    t_stat = time.time()
    while True:
        time.sleep(2.0)
        with _lock:
            n, czas, widze = _stat["n"], _stat["yolo"], _stat["widze"]
            _stat["n"], _stat["yolo"] = 0, 0.0
        if n:
            opis = ", ".join("%s %.0f%%" % (b["klasa"], b["pewnosc"] * 100) for b in widze) or "-"
            print(f"\r{n / (time.time() - t_stat):5.1f} kl/s | YOLO {czas / n * 1000:4.0f} ms | widze: {opis:40s}",
                  end="", flush=True)
        t_stat = time.time()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKoniec - Pi liczy YOLO samo.")
