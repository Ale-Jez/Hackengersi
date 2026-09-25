"""Uczenie i odtwarzanie poz: RoArm-M3 albo SO-Arm-101.

Uzycie:
  py teach.py COM5                    RoArm (USB do FOLLOWERA) -> poses.json
  py teach.py --arm so101 COM6        SO-101 (adapter magistrali serw) -> poses_so101.json
  py teach.py --arm so101 COM6 --scan pokaz ID serw na magistrali
  py teach.py [--arm so101] --mock    bez sprzetu

RoArm: nagrywasz ruszajac leaderem; do odtwarzania ODLACZ 12 V od leadera.
SO-101: "torque off", ustawiasz ramie reka (trzymaj je!), "s nazwa", "torque on".
"""

import argparse
import json
import os
import sys

from roarm import MockRoArm, RoArm, RoArmError
from so101 import MockSO101, SO101, SO101Error

ARM_ERRORS = (RoArmError, SO101Error)
GRIPPER_KEY = "_gripper"

HELP = """
Komendy:
  f                 pokaz aktualne katy
  s NAZWA           zapisz aktualna poze (np. s pick)
  g NAZWA           jedz do pozy (chwytak zostaje jak jest)
  gg NAZWA          jedz do pozy razem z zapisanym chwytakiem
  p KROK KROK ...   sekwencja: nazwy poz oraz open / close
                    np.  p stow pick close scan trash open stow
  j STAW KAT        rusz jednym stawem o KAT stopni, np.  j wroll 30  /  j grip -10
                    (stawy: pan lift elbow wflex wroll grip  |  RoArm: b s e t r h)
  o / c             chwytak otworz / zamknij
  cal open|closed   zapamietaj aktualny chwytak jako otwarty / zamkniety
  v LICZBA          predkosc w st./s (teraz: {speed})
  l                 lista poz
  del NAZWA         usun poze
  stop              zatrzymaj ramie w miejscu
  torque on|off     moment serw (off = ramie OPADA, trzymaj je!)
  q                 wyjscie
"""


def load(path):
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    return {}


def save(path, poses):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(poses, f, indent=2, ensure_ascii=False)


def pose_names(poses):
    return [n for n in poses if not n.startswith("_")]


def fmt(arm, pose):
    return "  ".join(f"{j}={pose[j]:7.1f}" for j in arm.JOINTS if j in pose)


def go(arm, poses, name, with_gripper=False):
    if name not in pose_names(poses):
        print(f"brak pozy '{name}'. Znane: {', '.join(pose_names(poses)) or '(zadnych)'}")
        return False
    gripper = arm.JOINTS[-1]
    target = {j: v for j, v in poses[name].items() if with_gripper or j != gripper}
    print(f"-> {name}")
    arm.move(target)
    return True


def play(arm, poses, steps):
    for step in steps:
        if step == "open":
            print("-> open")
            arm.release()
        elif step == "close":
            print("-> close")
            arm.grip()
        elif not go(arm, poses, step):
            print("sekwencja przerwana")
            return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="np. COM5 albo /dev/ttyUSB0")
    ap.add_argument("--arm", choices=["roarm", "so101"], default="roarm")
    ap.add_argument("--mock", action="store_true", help="bez sprzetu")
    ap.add_argument("--poses", help="plik z pozami (domyslnie poses.json / poses_so101.json)")
    ap.add_argument("--speed", type=int, default=30)
    ap.add_argument("--scan", action="store_true", help="SO-101: pokaz ID serw i wyjdz")
    args = ap.parse_args()
    path = args.poses or ("poses_so101.json" if args.arm == "so101" else "poses.json")

    if args.scan:
        if args.arm != "so101" or not args.port:
            ap.error("--scan dziala z --arm so101 i portem")
        from scservo_sdk import PacketHandler, PortHandler

        port = PortHandler(args.port)
        port.openPort()
        port.setBaudRate(1_000_000)
        ph = PacketHandler(0)
        found = [sid for sid in range(1, 21) if ph.ping(port, sid)[1] == 0]
        print(f"serwa na magistrali: {found or 'ZADNE - sprawdz zasilanie i kabel'}")
        print("SO-101 oczekuje ID 1-6 (pan, lift, elbow, wflex, wroll, grip)")
        port.closePort()
        return

    if args.mock:
        arm = MockSO101(speed=args.speed) if args.arm == "so101" else MockRoArm(speed=args.speed)
    elif args.port:
        cls = SO101 if args.arm == "so101" else RoArm
        arm = cls(args.port, speed=args.speed)
    else:
        ap.error("podaj port (np. COM5) albo --mock")

    poses = load(path)
    cal = poses.get(GRIPPER_KEY, {})
    if "open" in cal:
        arm.gripper_open = cal["open"]
    if "closed" in cal:
        arm.gripper_closed = cal["closed"]

    print(f"Ramie: {args.arm}   pozy z {path}: {', '.join(pose_names(poses)) or '(brak)'}")
    print(f"Chwytak: otwarty={arm.gripper_open}  zamkniety={arm.gripper_closed}")
    try:
        print("Aktualnie:", fmt(arm, arm.joints()))
    except ARM_ERRORS as e:
        print("UWAGA:", e)
    print(HELP.format(speed=arm.speed))

    while True:
        try:
            line = input(f"{args.arm}> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, *rest = line.split()
        try:
            if cmd == "q":
                break
            elif cmd == "f":
                print(fmt(arm, arm.joints()))
            elif cmd == "s" and rest:
                if rest[0].startswith("_"):
                    print("nazwy zaczynajace sie od _ sa zarezerwowane")
                    continue
                poses[rest[0]] = arm.joints()
                save(path, poses)
                print(f"zapisano '{rest[0]}': {fmt(arm, poses[rest[0]])}")
            elif cmd == "g" and rest:
                go(arm, poses, rest[0])
            elif cmd == "gg" and rest:
                go(arm, poses, rest[0], with_gripper=True)
            elif cmd == "p" and rest:
                play(arm, poses, rest)
            elif cmd == "j" and len(rest) == 2:
                if rest[0] not in arm.JOINTS:
                    print(f"nieznany staw '{rest[0]}'. Stawy: {' '.join(arm.JOINTS)}")
                    continue
                delta = float(rest[1])
                if abs(delta) > 45:
                    print("max 45 st. na raz")
                    continue
                target = arm.joints()[rest[0]] + delta
                arm.move({rest[0]: target}, wait=rest[0] in arm.ARM_JOINTS)
                print(f"{rest[0]} -> {target:.1f}")
            elif cmd == "o":
                arm.release()
            elif cmd == "c":
                arm.grip()
            elif cmd == "cal" and rest in (["open"], ["closed"]):
                value = arm.joints()[arm.JOINTS[-1]]
                setattr(arm, f"gripper_{rest[0]}", value)
                poses.setdefault(GRIPPER_KEY, {})[rest[0]] = value
                save(path, poses)
                print(f"chwytak {rest[0]} = {value}")
            elif cmd == "v" and rest:
                arm.speed = max(1, int(rest[0]))
                print(f"predkosc = {arm.speed}")
            elif cmd == "l":
                for name in pose_names(poses):
                    print(f"  {name:12s} {fmt(arm, poses[name])}")
            elif cmd == "del" and rest:
                if rest[0] in pose_names(poses):
                    del poses[rest[0]]
                    save(path, poses)
                    print(f"usunieto '{rest[0]}'")
            elif cmd == "stop":
                arm.hold()
            elif cmd == "torque" and rest in (["on"], ["off"]):
                arm.torque(rest[0] == "on")
            else:
                print(HELP.format(speed=arm.speed))
        except ARM_ERRORS as e:
            print("BLAD:", e)
        except KeyboardInterrupt:
            print("\nprzerwano - zatrzymuje ramie")
            try:
                arm.hold()
            except ARM_ERRORS as e:
                print("BLAD:", e)

    arm.close()


if __name__ == "__main__":
    sys.exit(main())
