"""Podnoszenie pudelka po pizzy - SO-101.

Uruchom:  python pizza.py

1. Na starcie chwytak sie OTWIERA.
2. Sterujesz recznie i podjezdzasz do krawedzi pudelka:
     W/S ramie przod/tyl   I/K lokiec gora/dol   A/D obrot podstawy
     J/L zgiecie nadgarstka   U/O obrot nadgarstka   1/2/3 predkosc
3. SPACJA = zacisnij na pudelku (zamyka az poczuje opor, potem trzyma).
4. P = PODNIES pudelko powoli.   N = OPUSC z powrotem.
5. SPACJA jeszcze raz = pusc.   F = stop.   ESC = koniec.
"""

import time

import ramie

# ---------------- USTAWIENIA ----------------
OTWARCIE = 70.0        # kat otwartego chwytaka (wiecej = szerzej, max ~120)
PODNIES_LIFT = 8.0     # ile st. ramie (lift) przy podnoszeniu (+ = do tylu/gory)
PODNIES_ELBOW = -12.0  # ile st. lokiec przy podnoszeniu (- = do gory)
MAX_OBC = 80.0         # stop przy takim obciazeniu serwa (%)
# --------------------------------------------


def otworz(arm):
    print(f"Otwieram chwytak na {OTWARCIE} st....")
    arm.move({"grip": OTWARCIE}, speed=60, wait=False)
    time.sleep(1.5)


def podnies(arm):
    print("PODNOSZE pudelko...")
    arm.move_slow(delta={"lift": PODNIES_LIFT, "elbow": PODNIES_ELBOW}, max_load=MAX_OBC)
    print("Podniesione. N = opusc, SPACJA = pusc")


def opusc(arm):
    print("OPUSZCZAM pudelko...")
    arm.move_slow(delta={"lift": -PODNIES_LIFT, "elbow": -PODNIES_ELBOW}, max_load=MAX_OBC)
    print("Opuszczone. SPACJA = pusc")


if __name__ == "__main__":
    print(__doc__)
    ramie.sterowanie(extra_keys={"p": podnies, "n": opusc}, on_start=otworz)
