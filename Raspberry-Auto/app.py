"""Station panel: one click sends the car to a station, manual driving, taught routes, a live camera view,
STOP, battery.

    python app.py            then open http://<pi ip>:8000 on a phone or laptop in the same network
    MOCK=1 python app.py     no motors, no camera (grey frames): the panel and the state machine only
    NO_MOTORS=1 python app.py   real camera and driving logic, pretend wheels: move a tag by hand to try a trip

Stations (config.json "stations", one panel button each, in that order): "dok" = AprilTag 0, the docking
station where the car waits; "biurko" = AprilTag 1 by the desk, where people drop their rubbish;
"sortownia" = AprilTags 2, 3, 4, where the arms (SO-101 camera, RoArm-M3 gripper) sort it. A station with
several tags is reached at whichever of them the car sees first. Each stops stop_cm in front of its tag.
"home" is where the car is assumed to stand on the very first start.
"on_arrive": a URL called (GET) after docking, e.g. the arm Pi's "http://<ip>:8765/roarm/uruchom?co=zbieranie".

Leaving a docked station for another one:
  - a taught route (routes.json) exists for that pair: replay it, move by move, by wheel encoder rotation,
    then approach the tag (searching toward the route's last turn if it is not in view);
  - otherwise: back off (leave_back), then turn until the next tag is in view.
Teaching: dock at the start station, "Ucz trasy" to the target, drive with the arrows until the target tag
is in view, "Zapisz trasę". Every press of an arrow is one move; the rotation of both wheels from that press
to the next one (including the coast after letting go) is what gets replayed.

Manual driving: the page sends the held arrow every 100 ms; the car stops 0.5 s after the last one (a
dropped WiFi, a closed tab). The drives are disabled while parked and enabled only to move. A trip stops
on STOP, on an obstacle that stays for obstacle_timeout, when the tag is not found within search_timeout,
and when the battery drops below battery_stop_v. This program owns the camera: stop stream.py first.
"""
import json
import math
import os
import threading
import time
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np

import vision
from drive import Aborted, Guard, approach, timed
from motors import DT, MOCK, Wheels, load_config
from stream import annotate

HERE = os.path.dirname(os.path.abspath(__file__))
WHERE = os.path.join(HERE, "where.json")
ROUTES = os.path.join(HERE, "routes.json")
CALIB = os.path.join(HERE, "calib.json")  # {"turn90_rad": wheel rotation for a 90° turn in place, "saved": ...}
TINY_RAD = 0.05  # a move that turned the wheels less than this is a tap that did nothing: not taught


def merge_moves(moves):
    """Drop do-nothing taps, join neighbours with the same command (press, let go, press again)."""
    out = []
    for m in moves:
        if max(abs(x) for x in m["d"]) < TINY_RAD:
            continue
        if out and out[-1]["cmd"] == m["cmd"]:
            out[-1]["d"] = [round(a + b, 3) for a, b in zip(out[-1]["d"], m["d"])]
        else:
            out.append({"cmd": list(m["cmd"]), "d": list(m["d"])})
    return out


class Car:
    """The state machine behind the buttons. One trip at a time, in its own thread."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.cam = vision.Camera(None if MOCK else cfg["camera"], tuple(cfg["camera_size"]), cfg.get("camera_flip", False))
        self.wheels = Wheels(cfg, mock=MOCK or os.environ.get("NO_MOTORS") == "1", keep_trying=True)
        self.wheels.power(False)  # parked
        self.abort, self.lock, self.trip = threading.Event(), threading.Lock(), None
        self.guard = None
        self.view_obs = vision.Obstacles(cfg)  # corridor drawn in the live view while parked
        self.routes = self._load_json(ROUTES, {})
        self.calib_saved = self._load_json(CALIB, {})
        self.teach = None  # {"from", "to", "moves": [{"cmd", "d"}], "cmd": held command, "start": wheel pos at its press}
        # tail: moves made after a route was saved (e.g. turning round to park), kept or dropped by the user before
        # the pose is used again: {"key": "from>to", "moves", "cmd", "start"}. dock_then_tail: the route whose
        # save started a docking trip; its tail begins when that trip arrives.
        self.tail, self.dock_then_tail = None, None
        self.calib = None  # {"start": wheel pos} while the 90° calibration runs
        self.mlock = threading.Lock()  # manual driving + teaching
        self.manual_on = self.manual_moving = False
        self.manual_at = 0.0
        self._seen = (0.0, [])
        # where: station key the car is docked at (None = somewhere in between). Kept in where.json across
        # restarts; the very first start assumes "home".
        where = self._load_json(WHERE, cfg.get("home"))
        where = where if where in cfg["stations"] else None  # a renamed station: somewhere in between
        msg = f"stoi: {cfg['stations'][where]['name']}" if where else "stoi między stacjami"
        self.state = {"where": where, "phase": "idle", "target": None, "message": msg, "since": time.time()}
        threading.Thread(target=self._battery_watch, daemon=True).start()
        threading.Thread(target=self._manual_watch, daemon=True).start()

    # ---------- small helpers ----------
    @staticmethod
    def _load_json(path, default):
        try:
            with open(path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return default

    def _name(self, key):
        return self.cfg["stations"][key]["name"] if key in self.cfg["stations"] else str(key)

    def _tags(self, key):
        return "/".join(map(str, self.cfg["stations"][key]["tags"]))

    def _busy(self):
        return self.trip is not None and self.trip.is_alive()

    def _set(self, **kw):
        self.state.update(kw, since=time.time())
        if "where" in kw:
            try:
                with open(WHERE, "w") as f:
                    json.dump(kw["where"], f)
            except OSError:
                pass
        print(f"[{time.strftime('%H:%M:%S')}] {self.state['phase']}: {self.state['message']}")

    def seen(self, fresh=False):
        """Tags in view now: [{id, px, cm}], cached for 0.3 s (several phones poll /status)."""
        t, tags = self._seen
        if fresh or time.monotonic() - t > 0.3:
            img = self.cam.peek()
            f, size = self.cfg.get("tag_focal_px"), self.cfg.get("tag_size_cm", 6.0)
            found = vision.find_tags(img) if img is not None else {}
            tags = [{"id": i, "px": round(v[2]), "cm": round(f * size / v[2]) if f else None} for i, v in sorted(found.items())]
            self._seen = (time.monotonic(), tags)
        return tags

    def status(self):
        s = dict(self.state)
        v = self.wheels.volts
        s["volts"] = None if v is None else round(v, 1)
        s["battery"] = "?" if v is None else "low" if v < self.cfg["battery_warn_v"] else "ok"
        s["obstacle"] = bool(self.guard and self.guard.stopped_at is not None and self._busy())
        s["stations"] = [{"key": k, "name": st["name"], "button": st.get("button", st["name"]), "tags": st["tags"]}
                         for k, st in self.cfg["stations"].items()]
        t, tl = self.teach, self.tail
        s["teach"] = None if t is None else {"from": t["from"], "to": t["to"], "moves": len(self._rec_moves(t))}
        n = len(self._rec_moves(tl)) if tl else 0
        s["tail"] = {"from": tl["key"].split(">")[0], "to": tl["key"].split(">")[1], "moves": n} if n else None
        s["routes"] = [{"key": k, "from": k.split(">")[0], "to": k.split(">")[1], "moves": len(r["moves"]),
                        "dock": r.get("dock", True), "after": len(r.get("after", [])), "saved": r.get("saved")}
                       for k, r in self.routes.items()]
        s["seen"] = self.seen()
        rad, source = self.turn90()
        s["turn90"] = {"rad": round(rad, 2), "source": source, "saved": self.calib_saved.get("saved"),
                       "track_m": self.cfg.get("track_m"), "wheel_radius_m": self.cfg.get("wheel_radius_m")}
        s["calib"] = self.calib is not None
        s["use_tags"] = self.cfg.get("use_tags", True)
        s["motors"] = self.wheels.link_ok
        s["legs"] = self.cfg.get("demo_legs", [["dok", "biurko"], ["biurko", "sortownia"], ["sortownia", "dok"]])
        return s

    def turn90(self):
        """Wheel rotation (rad, each wheel, opposite ways) that turns the car 90° in place, and where it came
        from: "calib" (measured, includes the floor's slip), "config" (track_m / wheel_radius_m, measured with
        a ruler: each wheel rolls 90° of a circle of radius track/2), or "guess"."""
        if self.calib_saved.get("turn90_rad"):
            return self.calib_saved["turn90_rad"], "calib"
        track, radius = self.cfg.get("track_m"), self.cfg.get("wheel_radius_m")
        if track and radius:
            return math.pi / 2 * (track / 2) / radius, "config"
        return 3.5, "guess"

    def _reload(self):
        try:
            self.cfg = load_config()  # settings edited in config.json apply to the next move, no restart
        except (OSError, ValueError) as e:
            print(f"config.json not reloaded ({e}), keeping the old one")

    # ---------- trips ----------
    def go(self, key):
        """Start a trip; returns an error text or None."""
        if key not in self.cfg["stations"]:
            return "nieznana stacja"
        if self.teach is not None:
            return "trwa nauka trasy: najpierw Zapisz albo Anuluj"
        if self.calib is not None:
            return "trwa kalibracja obrotu: najpierw Zapisz albo Anuluj"
        err = self._tail_pending() or self._link_err()
        if err:
            return err
        with self.lock:
            if self._busy():
                return "robot już jedzie"
            v = self.wheels.volts
            if v is not None and v < self.cfg["battery_stop_v"]:
                return f"akumulator za słaby ({v:.1f} V): naładuj"
            if self.state["where"] == key:
                return f"robot już stoi: {self._name(key)}"
            self.abort.clear()
            self.trip = threading.Thread(target=self._trip, args=(key,), daemon=True)
            self.trip.start()
        return None

    def stop(self):
        self.abort.set()
        with self.mlock:
            self.wheels.set(0, 0)
            self.manual_moving = False

    def stop_home(self):
        """The presentation STOP: stop, then start over from the first station ("home"), where people carry the car back."""
        self.stop()
        if self.trip is not None:
            self.trip.join(timeout=5)
        if self.teach is not None or self.calib is not None:
            return None  # teaching or calibrating in the full panel: just stop
        home = self.cfg.get("home")
        self.set_where(home if home in self.cfg["stations"] else "")
        self._set(phase="idle", message=f"zatrzymany. Postaw łazik na starcie: {self._name(home)}")
        return None

    def _trip(self, key):
        self._reload()
        st, cfg = self.cfg["stations"][key], self.cfg
        stop_px = cfg["tag_focal_px"] * cfg["tag_size_cm"] / st["stop_cm"]
        frm = self.state["where"]
        route = self.routes.get(f"{frm}>{key}") if frm else None
        with self.mlock:
            self.manual_on = self.manual_moving = False  # the trip owns the wheels now
        self._set(phase="driving", target=key, where=None, message=f"jedzie: {st['name']} (tag {self._tags(key)})")
        use_tags = cfg.get("use_tags", True)
        if not route and not use_tags:
            self._set(phase="error", target=None, where=frm,
                      message=f"brak nauczonej trasy {self._name(frm) if frm else '(łazik nie stoi przy stacji)'} → "
                              f"{st['name']}, a AprilTagi są wyłączone (use_tags): naucz tę trasę")
            return
        try:
            self.wheels.power(True)
            self.guard = Guard(cfg, self.wheels)
            side, dock = -1, use_tags
            if route:
                self._set(message=f"jedzie: {st['name']}: nauczona trasa ({len(route['moves'])} ruchów)")
                side = self._replay(route["moves"])
                dock = use_tags and route.get("dock", True)  # False: the tag was not in view where the route was taught to end
                self._set(message=f"jedzie: {st['name']} (tag {self._tags(key)})")
            elif frm is not None:  # nose at the old tag: back off, then approach() turns until it sees the new one
                timed(self.cam, self.wheels, self.guard, *cfg["leave_back"], abort=self.abort)
            if dock:
                approach(cfg, self.cam, self.wheels, self.guard, st["tags"], stop_px, abort=self.abort, search_side=side)
            if route and route.get("after"):  # moves taught after arriving (turning round to park, ...)
                self._set(message=f"{st['name']}: ruchy na koniec trasy ({len(route['after'])})")
                self._replay(route["after"])
            self._set(phase="idle", where=key, target=None, message=f"na miejscu: {st['name']}")
            if self.dock_then_tail and self.dock_then_tail.endswith(f">{key}"):
                self.tail = {"key": self.dock_then_tail, "moves": [], "cmd": None, "start": None}
            if st.get("on_arrive"):
                threading.Thread(target=self._call, args=(st["on_arrive"],), daemon=True).start()
        except Aborted:
            self._set(phase="stopped", target=None, message=self.state.get("abort_reason") or "zatrzymany przyciskiem STOP")
        except TimeoutError as e:
            self._set(phase="error", target=None, message=f"nie dojechał: {e}")
        except Exception as e:  # CAN, camera: stop and show it instead of dying silently
            self._set(phase="error", target=None, message=f"błąd: {e!r}")
        finally:
            self.state.pop("abort_reason", None)
            self.dock_then_tail = None
            self.wheels.power(False)

    def _move(self, cmd, d):
        """One move: drive cmd until the wheel that has to turn most has turned |d| of it, minus what the
        speed ramp still adds after letting go (v^2 / 2a), then stop and let it settle. STOP (abort) halts it.
        Returns (wheel rotation actually done [left, right] rad, aborted)."""
        w, (l, r) = self.wheels, cmd
        k = 0 if abs(d[0]) >= abs(d[1]) else 1
        goal, sgn = abs(d[k]), (1 if d[k] > 0 else -1)
        start = list(w.pos)
        if goal < TINY_RAD:
            return [0.0, 0.0], False
        limit = time.monotonic() + 3 + 4 * goal / max(0.5, abs((l, r)[k]) * w.max)
        w.set(l, r)
        while not self.abort.is_set():
            done = (w.pos[k] - start[k]) * sgn
            v = w.speed()[k]
            if goal - done <= v * v / (2 * w.acc) or time.monotonic() > limit:
                break
            time.sleep(DT)
        aborted = self.abort.is_set()
        w.halt() if aborted else w.set(0, 0)
        while not w.still() and not self.abort.is_set():
            time.sleep(DT)
        time.sleep(0.15)  # the drives settle a little after the ramp reached zero
        return [round(p - s, 3) for p, s in zip(w.pos, start)], aborted

    def _replay(self, moves):
        """Drive the taught moves, each as far (wheel encoder rotation) as when taught.
        Returns the side of the last turn (+1 right, -1 left), where to look for the tag if it is not in view."""
        side = -1
        for n, mv in enumerate(moves, 1):
            (l, r), d = mv["cmd"], mv["d"]
            if l != r:
                side = 1 if l > r else -1
            got, aborted = self._move((l, r), d)
            print(f"  replay {n}/{len(moves)}: cmd L={l:+.2f} R={r:+.2f}  taught {d[0]:+.2f} {d[1]:+.2f} rad"
                  f"  done {got[0]:+.2f} {got[1]:+.2f} rad")
            if aborted:
                raise Aborted()
        return side

    def _call(self, url):
        try:
            with urllib.request.urlopen(url, timeout=5) as r:
                print(f"on_arrive {url} -> {r.status}")
        except Exception as e:
            print(f"on_arrive {url} failed: {e}")

    def _battery_watch(self):
        """Stops a trip on a flat battery or a lost link to the drives (Wheels reconnects by itself)."""
        while True:
            v = self.wheels.volts
            if self._busy() and not self.abort.is_set():
                if not self.wheels.link_ok:
                    self.state["abort_reason"] = "utracono połączenie z silnikami (CANdle): zatrzymany, łączę ponownie"
                    self.abort.set()
                elif v is not None and v < self.cfg["battery_stop_v"]:
                    self.state["abort_reason"] = f"akumulator {v:.1f} V: zatrzymany, naładuj"
                    self.abort.set()
            time.sleep(0.2)

    def _link_err(self):
        return None if self.wheels.link_ok else \
            "brak połączenia z silnikami (CANdle): sprawdź kabel USB i zasilanie silników, łączę ponownie co sekundę"

    # ---------- manual driving and teaching ----------
    def _dirs(self):
        f, t = self.cfg.get("manual_speed", 0.25), self.cfg.get("manual_turn", 0.18)
        return {"up": (f, f), "down": (-f, -f), "left": (-t, t), "right": (t, -t), "stop": (0.0, 0.0)}

    def manual(self, d):
        cmd = self._dirs().get(d)
        if cmd is None:
            return "nieznany kierunek"
        if self._busy():
            return "łazik jedzie sam: najpierw STOP"
        if cmd != (0.0, 0.0) and self._link_err():
            return self._link_err()
        with self.mlock:
            if cmd != (0.0, 0.0):
                if not self.manual_on:
                    self.wheels.power(True)
                    self.manual_on = True
                rec = self._rec()
                if rec is None and self.state["where"] is not None:
                    self._set(where=None, message="sterowanie ręczne")
                if rec is not None and cmd != rec["cmd"]:
                    self._close_move()
                    rec["cmd"], rec["start"] = cmd, list(self.wheels.pos)
            self.manual_moving = cmd != (0.0, 0.0)
            self.manual_at = time.monotonic()
            self.wheels.set(*cmd)
        return None

    def _rec(self):
        """Where manual moves are recorded: the route being taught, else the moves after one was saved."""
        return self.teach if self.teach is not None else self.tail

    def _close_move(self):
        rec = self._rec()
        if rec is not None and rec["cmd"] is not None:
            rec["moves"].append({"cmd": list(rec["cmd"]), "d": [round(e - s, 3) for e, s in zip(self.wheels.pos, rec["start"])]})
            rec["cmd"] = None

    def _rec_moves(self, rec):
        """A recorder's moves so far, merged, the one still being driven included (measured up to now)."""
        moves = list(rec["moves"])
        if rec["cmd"] is not None:
            moves.append({"cmd": list(rec["cmd"]), "d": [round(e - s, 3) for e, s in zip(self.wheels.pos, rec["start"])]})
        return merge_moves(moves)

    def _tail_pending(self):
        """Moves made after saving a route have to be kept or dropped before anything relies on the pose."""
        tl = self.tail
        if tl is None:
            return None
        n = len(self._rec_moves(tl))
        if not n:
            self.tail = None
            return None
        a, b = tl["key"].split(">")
        return (f"po zapisaniu trasy {self._name(a)} → {self._name(b)} łazik zrobił jeszcze {n} ruchów: "
                f"kliknij „Dopisz do trasy” albo „Nie dopisuj”")

    def _settle(self):
        with self.mlock:
            self.wheels.set(0, 0)
            self.manual_moving = False
        end = time.monotonic() + 3
        while not self.wheels.still() and time.monotonic() < end:
            time.sleep(DT)
        time.sleep(0.2)  # let the wheels settle, so the last move gets its whole coast

    def tail_keep(self):
        """Append the moves made after saving to that route: they run after it arrives (and docks)."""
        tl = self.tail
        if tl is None or tl["key"] not in self.routes:
            self.tail = None
            return "nie ma ruchów do dopisania"
        self._settle()
        with self.mlock:
            self._close_move()
            self.tail = None
        route, b = self.routes[tl["key"]], tl["key"].split(">")[1]
        route["after"] = merge_moves(route.get("after", []) + tl["moves"])
        route["saved"] = time.strftime("%Y-%m-%d %H:%M")
        self._save_routes()
        print(f"route {tl['key']} after: {route['after']}")
        self._set(phase="idle", where=b, message=f"dopisane: trasa kończy się teraz tutaj. Łazik stoi: {self._name(b)}")
        return None

    def tail_drop(self):
        tl = self.tail
        self.tail = None
        if tl is not None and self._rec_moves(tl):
            self.stop()
            self._set(phase="idle", where=None, message="ruchy po zapisaniu pominięte: łazik nie stoi tak, jak kończy się "
                                                        "trasa. Ustaw niżej, gdzie stoi, albo dojedź do stacji.")
        return None

    def _manual_watch(self):
        while True:
            time.sleep(0.05)
            with self.mlock:
                if not self.manual_on or self._busy():
                    continue
                idle = time.monotonic() - self.manual_at
                if self.manual_moving and idle > 0.5:  # no command from the page: the button is not held any more
                    self.wheels.set(0, 0)
                    self.manual_moving = False
                if idle > 5 and self.wheels.still():
                    self.wheels.power(False)
                    self.manual_on = False

    def turn(self, direction):
        """Turn 90° in place (left / right) by wheel encoder rotation. While teaching it is one move of the route."""
        if direction not in ("left", "right"):
            return "nieznany kierunek"
        if self._busy():
            return "łazik jedzie sam: najpierw STOP"
        if self.calib is not None:
            return "trwa kalibracja: obracaj strzałkami"
        if self._link_err():
            return self._link_err()
        self._reload()
        cmd, g = self._dirs()[direction], self.turn90()[0]
        self.abort.clear()
        with self.mlock:  # STOP still works: it sets abort first, which ends the move
            if not self.manual_on:
                self.wheels.power(True)
                self.manual_on = True
            rec = self._rec()
            if rec is None and self.state["where"] is not None:
                self._set(where=None, message="sterowanie ręczne")
            self._close_move()
            self.manual_moving = False
            got, aborted = self._move(cmd, [-g, g] if direction == "left" else [g, -g])
            if rec is not None:
                rec["moves"].append({"cmd": list(cmd), "d": got})
            self.manual_at = time.monotonic()
        print(f"  turn 90° {direction}: wheels {got[0]:+.2f} {got[1]:+.2f} rad (goal {g:.2f}){' STOPPED' if aborted else ''}")
        return "przerwane przyciskiem STOP" if aborted else None

    def calib_start(self):
        if self._busy() or self.teach is not None:
            return "najpierw zatrzymaj łazik / zakończ naukę trasy"
        err = self._tail_pending()
        if err:
            return err
        self.calib = {"start": list(self.wheels.pos)}
        self._set(phase="idle", message="kalibracja obrotu: obróć łazik strzałkami ⟲ ⟳ o dokładnie 90°, 180° albo 360°, "
                                        "wybierz kąt i kliknij Zapisz kalibrację")
        return None

    def calib_save(self, deg):
        if self.calib is None:
            return "kalibracja nie trwa"
        try:
            deg = float(deg)
        except ValueError:
            return "zły kąt"
        with self.mlock:
            self.wheels.set(0, 0)
            self.manual_moving = False
        end = time.monotonic() + 3
        while not self.wheels.still() and time.monotonic() < end:
            time.sleep(DT)
        time.sleep(0.2)
        d = [p - s for p, s in zip(self.wheels.pos, self.calib["start"])]
        turned = abs(d[1] - d[0]) / 2  # in place: the wheels turn opposite ways, the car turns by their difference
        if turned < 0.5:
            return f"za mały obrót kół ({turned:.2f} rad): obróć łazik strzałkami ⟲ lub ⟳"
        self.calib_saved = {"turn90_rad": round(turned * 90 / deg, 3), "saved": time.strftime("%Y-%m-%d %H:%M"),
                            "measured": {"deg": deg, "wheels_rad": [round(x, 3) for x in d]}}
        with open(CALIB, "w") as f:
            json.dump(self.calib_saved, f, indent=1)
        self.calib = None
        self._set(phase="idle", message=f"skalibrowane: 90° = {self.calib_saved['turn90_rad']:.2f} rad obrotu koła")
        return None

    def calib_cancel(self):
        self.calib = None
        self._set(phase="idle", message="kalibracja anulowana")
        return None

    def teach_start(self, to):
        if self._busy():
            return "łazik jedzie: najpierw STOP"
        if self.teach is not None:
            return "nauka już trwa"
        if self.calib is not None:
            return "najpierw skończ kalibrację obrotu"
        err = self._tail_pending()
        if err:
            return err
        frm = self.state["where"]
        if frm is None:
            return "trasa zaczyna się przy stacji: najpierw dojedź do stacji startowej (albo ustaw niżej, gdzie stoi łazik)"
        if to not in self.cfg["stations"] or to == frm:
            return "wybierz inną stację docelową"
        self.teach = {"from": frm, "to": to, "moves": [], "cmd": None, "start": None}
        self._set(phase="teach", where=None, target=to,
                  message=f"nauka trasy: {self._name(frm)} → {self._name(to)}. Prowadź strzałkami do stacji docelowej")
        return None

    def teach_save(self):
        """Save the route. If a tag of the target is in view, the car then docks at it (and so will every replay);
        if not (e.g. parked with the tag behind), it stays where it is, and that is the station's pose."""
        t = self.teach
        if t is None:
            return "nauka nie trwa"
        self._settle()
        with self.mlock:
            self._close_move()
            moves = merge_moves(t["moves"])
        if not moves:
            return "nic nie nagrano: przejedź łazikiem strzałkami"
        frm, to = t["from"], t["to"]
        key = f"{frm}>{to}"
        self._reload()
        visible = [x for x in self.seen(fresh=True) if x["id"] in self.cfg["stations"][to]["tags"]] if self.cfg.get("use_tags", True) else []
        self.routes[key] = {"moves": moves, "dock": bool(visible), "saved": time.strftime("%Y-%m-%d %H:%M")}
        self._save_routes()
        self.teach = None
        print(f"route {key}: dock={bool(visible)} {moves}")
        done = f"trasa zapisana: {self._name(frm)} → {self._name(to)} ({len(moves)} ruchów)"
        if visible:
            self._set(phase="idle", target=None, where=None, message=f"{done}. Widzę tag {visible[0]['id']}: dojeżdżam do niego")
            self.dock_then_tail = key
            err = self.go(to)
            if err:
                self.dock_then_tail = None
                self._set(message=f"{done}, ale nie dojadę do tagu: {err}")
        else:
            self.tail = {"key": key, "moves": [], "cmd": None, "start": None}
            self._set(phase="idle", target=None, where=to,
                      message=f"{done}. Łazik zostaje tutaj: to jest stacja „{self._name(to)}”"
                              + ("" if self.cfg.get("use_tags", True) else " (AprilTagi wyłączone)"))
        return None

    def teach_cancel(self):
        if self.teach is None:
            return "nauka nie trwa"
        self.stop()
        self.teach = None
        self._set(phase="idle", target=None, message="nauka anulowana")
        return None

    def route_delete(self, key):
        if self.routes.pop(key, None) is None:
            return "nie ma takiej trasy"
        self._save_routes()
        return None

    def _save_routes(self):
        with open(ROUTES, "w") as f:
            json.dump(self.routes, f, indent=1, ensure_ascii=False)

    def set_where(self, key):
        """Someone carried the car and says where it is now ("" = in between)."""
        if self._busy() or self.teach is not None:
            return "najpierw zatrzymaj łazik / zakończ naukę"
        if key and key not in self.cfg["stations"]:
            return "nieznana stacja"
        key, self.tail = key or None, None  # placed by hand: moves after the last save no longer matter
        self._set(phase="idle", where=key, message=f"stoi: {self._name(key)} (ustawione ręcznie)" if key
                  else "stoi między stacjami (ustawione ręcznie)")
        return None

    def learn_floor(self):
        if self._busy():
            return "najpierw zatrzymaj robota"
        img = self.cam.peek()
        if img is None:
            return "brak obrazu"
        obs = vision.Obstacles(self.cfg)
        obs.learn(img)
        np.save(vision.FLOOR, obs.hist)
        self.view_obs = obs
        return None


PAGE = """<!doctype html><html lang=pl><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Robot na śmieci</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--ink:#1b1d21;--mute:#6b7280;--go:#1f7a4d;--pad:#2f5d8a;--stop:#c62828;--warn:#b26a00;--line:#dde1e6}
@media (prefers-color-scheme:dark){:root{--bg:#121417;--card:#1c1f24;--ink:#eceff3;--mute:#9aa3ad;--line:#2c3138}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.4 system-ui,sans-serif}
main{max-width:720px;margin:auto;padding:16px;display:grid;gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
h1{font-size:20px;margin:0}h2{font-size:17px;margin:0 0 10px}#msg{font-size:18px;font-weight:600}.mute{color:var(--mute);font-size:14px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
button{flex:1 1 200px;font:600 18px system-ui;padding:18px;border-radius:10px;border:0;color:#fff;background:var(--go);cursor:pointer}
button:disabled{opacity:.4;cursor:default}#stop{background:var(--stop);font-size:22px}
.small{flex:0 0 auto;font-size:14px;padding:10px 14px;background:var(--mute)}
select{font:16px system-ui;padding:9px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink)}
.pad{display:grid;grid-template-columns:repeat(3,84px);grid-template-rows:repeat(3,64px);gap:8px;justify-content:center;margin:4px 0 10px}
.pad button{flex:none;padding:0;font-size:26px;background:var(--pad);touch-action:none;user-select:none;-webkit-user-select:none}
.pad button.on{outline:3px solid var(--ink)}
.turn{background:var(--pad);font-size:17px;padding:14px}
#bat.low{color:var(--stop);font-weight:700}#obs{color:var(--warn);font-weight:700}
img{width:100%;border-radius:8px;display:block;background:#000}#err{color:var(--stop);min-height:1.4em}
ul{margin:6px 0 0;padding-left:18px}li{margin:4px 0}
</style>
<main>
<div class=card><h1>Robot na śmieci <a href="/demo" style="font-size:14px;font-weight:400;color:var(--mute)">widok do prezentacji →</a></h1><div id=msg>…</div>
<div class=mute>Akumulator: <span id=bat>?</span> <span id=obs></span></div>
<div class=mute>Silniki: <span id=mot>?</span></div>
<div class=mute>Widzę: <span id=seen>—</span></div></div>
<div class="card row" id=buttons></div>
<button id=stop onclick="post('/stop')">STOP</button>
<div id=err></div>

<div class=card><h2>Sterowanie ręczne</h2>
<div class=pad>
 <span></span><button data-dir=up aria-label="do przodu">▲</button><span></span>
 <button data-dir=left aria-label="obrót w lewo">⟲</button><button data-dir=stop aria-label=stop style="background:var(--stop)">■</button><button data-dir=right aria-label="obrót w prawo">⟳</button>
 <span></span><button data-dir=down aria-label="do tyłu">▼</button><span></span>
</div>
<div class=mute>Przytrzymaj strzałkę; puszczenie = stop. Na komputerze: strzałki albo W A S D.</div>
<div class=row style="margin-top:12px">
 <button class=turn onclick="post('/turn?dir=left')">⟲ 90° w lewo</button>
 <button class=turn onclick="post('/turn?dir=right')">90° w prawo ⟳</button></div>
<div class=mute style="margin-top:6px">Jedno kliknięcie = obrót o 90° w miejscu. W nauce trasy zapisuje się jako ruch trasy.
 90° = <b id=t90>?</b> rad obrotu koła <span id=t90s></span></div>
<div id=c_idle class=row style="margin-top:8px"><span class=mute style="flex:1">Obrót o 90° nie wychodzi równo?</span>
 <button class=small onclick="post('/calib/start')">Kalibruj obrót</button></div>
<div id=c_on hidden style="margin-top:8px"><b>Kalibracja:</b> <span class=mute>obróć łazik strzałkami ⟲ ⟳ (nie przyciskami 90°) o dokładnie
 tyle stopni, ile wybierzesz. 360° jest najdokładniejsze: obróć, aż kamera znowu patrzy dokładnie tam, gdzie na starcie.</span>
 <div class=row style="margin-top:8px"><select id=c_deg><option value=90>90°</option><option value=180>180°</option><option value=360 selected>360°</option></select>
 <button class=small style="background:var(--go)" onclick="post('/calib/save?deg='+document.getElementById('c_deg').value)">Zapisz kalibrację</button>
 <button class=small onclick="post('/calib/cancel')">Anuluj</button></div></div></div>

<div class=card><h2>Nauka tras</h2>
<div id=t_idle>
 <div class=mute>Łazik stoi przy stacji startowej. Wybierz cel, prowadź go strzałkami do stacji docelowej i kliknij Zapisz.
 Jeśli tag celu jest wtedy w kadrze, łazik sam do niego dojedzie; jeśli nie (np. ma stać tyłem do tagu), zostanie tam, gdzie go zostawisz.</div>
 <div class=row style="margin-top:8px">Z: <b id=t_from>—</b> do: <select id=t_to></select>
 <button class=small id=t_go onclick="post('/teach/start?to='+document.getElementById('t_to').value)">Ucz trasy</button></div>
</div>
<div id=t_on hidden>
 <b id=t_what></b> <span class=mute id=t_moves></span>
 <div class=mute id=t_hint></div>
 <div class=row style="margin-top:8px"><button class=small style="background:var(--go)" onclick="post('/teach/save')">Zapisz trasę</button>
 <button class=small onclick="post('/teach/cancel')">Anuluj</button></div>
</div>
<div id=tail hidden style="margin-top:10px"><b id=tail_what></b>
 <div class=mute>Np. obrót na postój. Dopisać je, żeby łazik robił je sam po dojechaniu tą trasą?</div>
 <div class=row style="margin-top:8px"><button class=small style="background:var(--go)" onclick="post('/tail/keep')">Dopisz do trasy</button>
 <button class=small onclick="post('/tail/drop')">Nie dopisuj</button></div></div>
<div id=routes class=mute style="margin-top:10px"></div>
<div class=row style="margin-top:10px"><span class=mute>Przeniesiony ręką? Stoi teraz przy:</span><select id=w_at></select>
<button class=small onclick="post('/where?at='+document.getElementById('w_at').value)">Ustaw</button></div>
</div>

<div class=card><img src="/stream" alt="kamera robota">
<div class=row style="margin-top:10px"><span class=mute style="flex:1">Nowe miejsce albo inne światło? Ustaw robota przodem do pustej podłogi i:</span>
<button class=small onclick="post('/floor')">Naucz podłogę</button></div></div>
</main>
<script>
const $=id=>document.getElementById(id);
function err(t){$('err').textContent=t||''}
async function post(u){const r=await fetch(u,{method:'POST'});err(r.ok?'':await r.text());refresh()}
function go(k){post('/go?to='+k)}
// manual driving: the held direction goes out every 100 ms; the robot stops 0.5 s after the last one
let held=null,timer=null;
function send(){if(held)fetch('/drive?dir='+held,{method:'POST'}).then(async r=>{if(!r.ok)err(await r.text())})}
function hold(d){if(held===d)return;held=d;mark();send();clearInterval(timer);timer=setInterval(send,100)}
function release(){if(!held)return;held=null;mark();clearInterval(timer);fetch('/drive?dir=stop',{method:'POST'})}
function mark(){document.querySelectorAll('[data-dir]').forEach(b=>b.classList.toggle('on',b.dataset.dir===held))}
document.querySelectorAll('[data-dir]').forEach(b=>{const d=b.dataset.dir;
 b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);d=='stop'?(release(),post('/stop')):hold(d)});
 ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,release));
 b.addEventListener('contextmenu',e=>e.preventDefault())});
const KEYS={ArrowUp:'up',w:'up',ArrowDown:'down',s:'down',ArrowLeft:'left',a:'left',ArrowRight:'right',d:'right'};
addEventListener('keydown',e=>{const d=KEYS[e.key];if(d&&e.target.tagName!='SELECT'){e.preventDefault();hold(d)}});
addEventListener('keyup',e=>{if(KEYS[e.key])release()});
addEventListener('blur',release);document.addEventListener('visibilitychange',()=>{if(document.hidden)release()});

let built=false;
async function refresh(){try{const s=await(await fetch('/status')).json();
 const nm=k=>{const x=s.stations.find(st=>st.key==k);return x?x.name:k};
 const tg=k=>{const x=s.stations.find(st=>st.key==k);return x?x.tags.join(', '):'?'};
 $('msg').textContent=s.message;
 const b=$('bat');b.textContent=s.volts==null?'?':s.volts+' V'+(s.battery=='low'?' (niski, naładuj)':'');b.className=s.battery;
 $('obs').textContent=s.obstacle?'Przeszkoda: czekam':'';
 const m=$('mot');m.textContent=s.motors?'połączone':'BRAK POŁĄCZENIA (CANdle): sprawdź kabel USB i zasilanie, łączę ponownie…';
 m.style.color=s.motors?'':'var(--stop)';m.style.fontWeight=s.motors?'':'700';
 $('seen').textContent=s.seen.length?s.seen.map(t=>'tag '+t.id+' ('+t.px+' px'+(t.cm?', ~'+t.cm+' cm':'')+')').join(', '):'żadnego tagu';
 if(!built){built=true;const box=$('buttons');
  s.stations.forEach(st=>{const x=document.createElement('button');x.id='b_'+st.key;
   x.innerHTML=st.button+'<br><small style="font-weight:400;opacity:.8">tag '+st.tags.join(', ')+'</small>';x.onclick=()=>go(st.key);box.append(x);
   $('t_to').append(new Option(st.name+' (tag '+st.tags.join(', ')+')',st.key))});
  $('w_at').append(new Option('między stacjami',''));s.stations.forEach(st=>$('w_at').append(new Option(st.name,st.key)));}
 const busy=s.phase=='driving'||s.phase=='teach'||s.calib;
 s.stations.forEach(st=>{$('b_'+st.key).disabled=busy||s.where==st.key});
 const T=s.turn90;$('t90').textContent=T.rad;
 $('t90s').textContent=T.source=='calib'?'(skalibrowane '+T.saved+')':
  T.source=='config'?'(z configu: rozstaw '+(T.track_m*100).toFixed(1)+' cm, promień koła '+(T.wheel_radius_m*100).toFixed(2)+' cm)':
  '(szacunek: wpisz track_m i wheel_radius_m w config.json albo skalibruj)';
 document.querySelectorAll('.turn').forEach(x=>x.disabled=s.phase=='driving'||s.calib);
 $('c_idle').hidden=s.calib;$('c_on').hidden=!s.calib;
 $('t_idle').hidden=!!s.teach;$('t_on').hidden=!s.teach;
 $('t_from').textContent=s.where?nm(s.where):'— (łazik nie stoi przy stacji)';$('t_go').disabled=busy||!s.where;
 if(s.teach){$('t_what').textContent='Nagrywam: '+nm(s.teach.from)+' → '+nm(s.teach.to);$('t_moves').textContent='(ruchów: '+s.teach.moves+')';
  const ok=s.seen.some(t=>s.stations.find(st=>st.key==s.teach.to).tags.includes(t.id));
  $('t_hint').textContent=!s.use_tags?'AprilTagi wyłączone: doprowadź łazik dokładnie tam i tak, jak ma stać na stacji, potem Zapisz.':ok?'Widzę tag celu: po Zapisz łazik sam do niego dojedzie i zadokuje.':
   'Tag celu ('+tg(s.teach.to)+') nie jest w kadrze: po Zapisz łazik zostanie dokładnie tutaj, tak ma stać na stacji.';}
 $('tail').hidden=!s.tail;
 if(s.tail)$('tail_what').textContent='Po zapisaniu trasy '+nm(s.tail.from)+' → '+nm(s.tail.to)+' łazik zrobił jeszcze '+s.tail.moves+' ruch(ów).';
 $('routes').innerHTML=s.routes.length?'Nauczone trasy:<ul>'+s.routes.map(r=>'<li>'+nm(r.from)+' → '+nm(r.to)+': '+r.moves+' ruchów'+
  (r.after?', +'+r.after+' na koniec':'')+(r.dock?', dokuje do tagu ':', bez dokowania ')+
  '<button class=small style="padding:4px 10px" onclick="if(confirm(\\'Usunąć trasę?\\'))post(\\'/route/delete?key='+encodeURIComponent(r.key)+'\\')">usuń</button></li>').join('')+'</ul>':'Brak nauczonych tras.';
}catch(e){$('msg').textContent='brak połączenia z robotem'}}
refresh();setInterval(refresh,700);
</script>"""


DEMO = """<!doctype html><html lang=pl><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Sloppy</title>
<style>
:root{--bg:#0f1115;--card:#1a1d23;--ink:#f2f4f7;--mute:#98a1ad;--go:#1f7a4d;--here:#2f5d8a;--stop:#c62828;--line:#2a2f37}
@media (prefers-color-scheme:light){:root{--bg:#f4f5f7;--card:#fff;--ink:#16181c;--mute:#5f6773;--line:#dde1e6}}
*{box-sizing:border-box}html,body{height:100%}
body{margin:0;background:var(--bg);color:var(--ink);font:18px/1.35 system-ui,sans-serif;display:flex}
main{margin:auto;width:100%;max-width:980px;padding:16px;display:grid;gap:16px}
#msg{font-size:clamp(22px,4vw,34px);font-weight:700;text-align:center;min-height:1.4em}
#sub{color:var(--mute);text-align:center;font-size:16px;min-height:1.3em}
#err{color:var(--stop);text-align:center;font-weight:600;min-height:1.3em}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px}
button:not(.here):not(.going):disabled{background:#3a3f47}
button{font:700 clamp(20px,3vw,26px) system-ui;color:#fff;border:0;border-radius:18px;cursor:pointer;
 padding:clamp(22px,5vw,40px) 14px;background:var(--go);box-shadow:0 2px 0 rgba(0,0,0,.25)}
button small{display:block;font-weight:500;font-size:15px;opacity:.85;margin-top:6px}
button:disabled{opacity:.35;cursor:default}
button.here{background:var(--go);outline:4px solid var(--ink);opacity:1}
button.going{outline:4px solid var(--ink);animation:pulse 1.2s ease-in-out infinite}
@keyframes pulse{50%{filter:brightness(1.25)}}
#stop{background:var(--stop);font-size:clamp(26px,4vw,34px);padding:clamp(20px,4vw,30px)}
footer{display:flex;justify-content:space-between;color:var(--mute);font-size:14px}
footer a{color:var(--mute)}
</style>
<main>
<div id=msg>…</div><div id=sub></div>
<div class=grid id=buttons></div>
<button id=stop onclick="post('/stop?home=1')">STOP<small>i od początku</small></button>
<div id=err></div>
<footer><span id=bat></span><a href="/">pełny panel</a></footer>
</main>
<script>
const $=id=>document.getElementById(id);
async function post(u){const r=await fetch(u,{method:'POST'});$('err').textContent=r.ok?'':await r.text();refresh()}
// one button per leg, in order; only the leg that starts where the car stands (and has a taught route) is live
let built=false,legFrom={};
async function refresh(){try{const s=await(await fetch('/status')).json();
 const nm=k=>{const x=s.stations.find(st=>st.key==k);return x?x.name:k};
 if(!built){built=true;s.legs.forEach(([a,b],i)=>{const x=document.createElement('button');x.id='leg'+i;legFrom[i]=a;
  x.onclick=()=>post('/go?to='+b);$('buttons').append(x)})}
 const driving=s.phase=='driving',taught=new Set(s.routes.map(r=>r.key));
 let next=-1;
 s.legs.forEach(([a,b],i)=>{const x=$('leg'+i),going=driving&&s.target==b&&s.message.includes(nm(b)),
  mine=s.where==a,ok=mine&&(taught.has(a+'>'+b)||s.use_tags);
  if(ok&&!driving)next=i;
  x.innerHTML=(i+1)+'. '+nm(a)+' → '+nm(b)+'<small>'+(going?'jadę…':ok?'kliknij, żeby pojechać':
   mine?'brak nauczonej trasy':!taught.has(a+'>'+b)&&!s.use_tags?'brak nauczonej trasy':'łazik musi stać: '+nm(a))+'</small>';
  x.disabled=driving||!ok||s.phase=='teach'||s.calib||!s.motors;x.className=going?'going':ok&&!driving?'here':''});
 $('msg').textContent=driving?s.message:s.where?'Łazik stoi: '+nm(s.where):s.message;
 $('sub').textContent=!s.motors?'Silniki: brak połączenia (CANdle), łączę ponownie…':s.phase=='teach'?'Trwa nauka trasy w pełnym panelu':
  driving?'':s.phase=='error'||s.phase=='stopped'?s.message:next>=0?'Następny etap: '+(next+1):
  'Łazik nie stoi na początku żadnego etapu: ustaw w pełnym panelu, gdzie stoi';
 $('bat').textContent='Akumulator: '+(s.volts==null?'?':s.volts+' V'+(s.battery=='low'?' (niski!)':''));
}catch(e){$('msg').textContent='Brak połączenia z łazikiem'}}
refresh();setInterval(refresh,600);
</script>"""


def serve(car, port):
    seen = deque(maxlen=30)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="text/plain; charset=utf-8"):
            body = body if isinstance(body, bytes) else body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/status":
                return self._send(200, json.dumps(car.status()), "application/json")
            if path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        img = car.cam.peek()
                        if img is not None:
                            obs = car.guard.obs if car.guard is not None and car._busy() else car.view_obs
                            ok, buf = cv2.imencode(".jpg", annotate(img, car.cfg, obs, seen), [cv2.IMWRITE_JPEG_QUALITY, 70])
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
                        time.sleep(0.1)  # ~10 fps is plenty for a view, and leaves the CPU to the driving loop
                except (BrokenPipeError, ConnectionResetError):
                    return
            return self._send(200, DEMO if path.rstrip("/") == "/demo" else PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            u = urlsplit(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query, keep_blank_values=True).items()}
            actions = {
                "/go": lambda: car.go(q.get("to", "")),
                "/stop": lambda: car.stop_home() if q.get("home") == "1" else car.stop(),
                "/drive": lambda: car.manual(q.get("dir", "")),
                "/turn": lambda: car.turn(q.get("dir", "")),
                "/calib/start": car.calib_start,
                "/calib/save": lambda: car.calib_save(q.get("deg", "90")),
                "/calib/cancel": car.calib_cancel,
                "/teach/start": lambda: car.teach_start(q.get("to", "")),
                "/teach/save": car.teach_save,
                "/teach/cancel": car.teach_cancel,
                "/tail/keep": car.tail_keep,
                "/tail/drop": car.tail_drop,
                "/route/delete": lambda: car.route_delete(q.get("key", "")),
                "/where": lambda: car.set_where(q.get("at", "")),
                "/floor": car.learn_floor,
            }
            if u.path not in actions:
                return self._send(404, "?")
            err = actions[u.path]()
            self._send(409 if err else 200, err or "ok")

    print(f"panel: http://<pi ip>:{port}  (Ctrl-C to quit)")
    ThreadingHTTPServer(("", port), Handler).serve_forever()


if __name__ == "__main__":
    cfg = load_config()
    car = Car(cfg)
    try:
        serve(car, cfg["app_port"])
    finally:
        car.stop()
        car.wheels.close()
