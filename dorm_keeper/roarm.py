"""Sterownik RoArm-M3 Pro (firmware 0.84-s1) przez port szeregowy (USB, 115200).

Komendy JSON z RoArm-M3_safe/json_cmd.h:
  {"T":105}  -> odpowiedz {"T":1051, "b","s","e","t","r","g" w radianach, "tB".."tG" obciazenie}
  {"T":122,"b","s","e","t","r","h" w stopniach,"spd","acc"}  -> ruch wszystkich stawow
  {"T":210,"cmd":0|1}  -> moment serw wyl./wl.
"""

import json
import math
import time

JOINTS = ["b", "s", "e", "t", "r", "h"]  # podstawa, ramie, lokiec, nadgarstek, obrot, chwytak
ARM_JOINTS = JOINTS[:5]
GRIPPER_OPEN = 90.0     # = 1.57 rad, "release" z json_cmd.h
GRIPPER_CLOSED = 180.0  # = 3.14 rad, "grab" z json_cmd.h

# Feedback uzywa "g" dla chwytaka, komenda T:122 uzywa "h".
_FEEDBACK_KEYS = {"b": "b", "s": "s", "e": "e", "t": "t", "r": "r", "h": "g"}


class RoArmError(Exception):
    pass


class RoArm:
    JOINTS = JOINTS
    ARM_JOINTS = ARM_JOINTS
    gripper_open = GRIPPER_OPEN
    gripper_closed = GRIPPER_CLOSED

    def __init__(self, port, baud=115200, speed=30, acc=10):
        import serial

        self.speed = speed
        self.acc = acc
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.1
        # Nie resetuj ESP32 przy otwieraniu portu (jak w serial_simple_ctrl.py).
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def send(self, cmd):
        self.ser.write((json.dumps(cmd) + "\n").encode())

    def _read_json(self, want_t, timeout=1.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line.startswith("{"):
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            if msg.get("T") == want_t:
                return msg
        raise RoArmError(f"brak odpowiedzi T:{want_t} w {timeout}s")

    def feedback(self):
        """Surowa odpowiedz T:1051."""
        self.ser.reset_input_buffer()
        self.send({"T": 105})
        return self._read_json(1051)

    def joints(self):
        """Aktualne katy stawow w stopniach: {"b":..,"s":..,"e":..,"t":..,"r":..,"h":..}."""
        fb = self.feedback()
        if fb.get("b") is None:
            raise RoArmError("feedback_valid=false: serwa bez zasilania 12 V?")
        return {j: round(math.degrees(fb[k]), 1) for j, k in _FEEDBACK_KEYS.items()}

    def move(self, pose, speed=None, wait=True, tol=3.0, timeout=15.0):
        """Ruch do pozy w stopniach. Brakujace stawy zostaja tam, gdzie sa."""
        target = dict(pose)
        if any(j not in target for j in JOINTS):
            current = self.joints()
            for j in JOINTS:
                target.setdefault(j, current[j])
        # spd=0 to dla serw Feetech "maksymalna predkosc" - nigdy tego nie wysylamy.
        spd = max(1, int(speed or self.speed))
        self.send({"T": 122, **{j: target[j] for j in JOINTS}, "spd": spd, "acc": self.acc})
        if wait:
            self.wait_reached(target, tol, timeout)
        return target

    def wait_reached(self, target, tol=3.0, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            now = self.joints()
            if all(abs(now[j] - target[j]) <= tol for j in ARM_JOINTS if j in target):
                return True
            time.sleep(0.1)
        raise RoArmError(f"nie dojechal do pozy w {timeout}s (przeszkoda albo poza zasiegiem?)")

    def grip(self, angle=None, settle=0.6):
        self.move({"h": self.gripper_closed if angle is None else angle}, wait=False)
        time.sleep(settle)

    def release(self, angle=None, settle=0.6):
        self.move({"h": self.gripper_open if angle is None else angle}, wait=False)
        time.sleep(settle)

    def hold(self):
        """Zatrzymaj ramie w miejscu (wyslij aktualna pozycje jako cel)."""
        self.move(self.joints(), wait=False)

    def torque(self, on):
        self.send({"T": 210, "cmd": 1 if on else 0})

    def home(self):
        self.send({"T": 100})


class MockRoArm(RoArm):
    """Udaje ramie bez sprzetu - do testow na laptopie."""

    def __init__(self, speed=30, acc=10):
        self.speed = speed
        self.acc = acc
        self._pose = {"b": 0.0, "s": 0.0, "e": 90.0, "t": 0.0, "r": 0.0, "h": 180.0}

    def close(self):
        pass

    def send(self, cmd):
        print(f"  [mock] -> {json.dumps(cmd)}")
        if cmd.get("T") == 122:
            self._pose = {j: float(cmd[j]) for j in JOINTS}

    def joints(self):
        return dict(self._pose)
