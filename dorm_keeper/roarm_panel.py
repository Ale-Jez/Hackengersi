"""RoArm panel in the browser: http://<pi>:8765/roarm_panel (served by so101_station.py).

Hold-to-move control (release = stop), the SO-101 camera image next to it, servo protection:
slow small steps, reach and height limits, load display, STOP on overload. It never drives to a fixed pose.
"""
import json
import os
import re
import sys
import threading
import time

import requests

_dir = os.path.dirname(os.path.abspath(__file__))

SPEEDS = {"slow": (20.0, 0.12), "normal": (40.0, 0.2), "fast": (60.0, 0.25)}  # (mm/s, firmware spd <=0.25)
LEAD = 0.6                       # s: the target is this much motion ahead of the arm -> smooth, not step-stop-step
LIFT_MM = 100.0                  # P: grab and lift by this much
SERVO_HOT = (60, 65)             # deg C: from 65 motion is blocked (servo rests), from 60 down it may move again
ROARM_SERVOS = ["base", "shoulder 1", "shoulder 2", "elbow", "wrist", "roll", "gripper"]  # order of "temp" in /ws
Z_RANGE = (-150.0, 350.0)        # mm (RoArm upside down: upside_down, see _config)
LOAD_STOP = 350                  # |shoulder/elbow load| (firmware units): above = STOP and hold
KEEPALIVE = 0.6                  # s: no signal from the browser -> the arm stops (button released, WiFi dropped);
                                 # 0.35 stopped the arm on ordinary WiFi hiccups (PC -> Pi ping up to 156 ms)
# human moves (relative to the RoArm base): turn the whole base, reach out from the base, height, gripper
DIRECTIONS = {"turn+": ("turn", 1), "turn-": ("turn", -1), "reach+": ("reach", 1), "reach-": ("reach", -1),
              "z+": ("z", 1), "z-": ("z", -1), "t+": ("t", 1), "t-": ("t", -1), "r+": ("r", 1), "r-": ("r", -1)}
# single joints (T:101): one button = one joint, no other joint moves
JOINTS = {1: ("b", "base"), 2: ("s", "shoulder"), 3: ("e", "elbow"), 4: ("t", "wrist"), 5: ("r", "roll"),
          6: ("g", "gripper")}
DIRECTIONS.update({f"j{n}{z}": ("joint", n if z == "+" else -n) for n in JOINTS for z in "+-"})
JOINT_RAD_S = {"slow": 0.15, "normal": 0.3, "fast": 0.5}  # rad/s
COMMANDS = ("stop", "open", "close", "toggle", "lift")
POSE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,32}$")  # saved pose names: letters, digits, _ and -


def _config():
    sys.path.append(_dir)
    import roarm_pick

    return roarm_pick.load_config()


class RoArmPanel:
    def __init__(self):
        cfg = _config()
        self.ip, self.reach = cfg["roarm_ip"], cfg["reach_mm"]
        self.opened, self.closed = cfg["grip_open"], cfg["grip_closed"]
        self.up = -1 if cfg.get("upside_down") else 1  # RoArm upside down: world "up" = -z of the RoArm
        self.target = None        # control target x y z t r (from the first reading)
        self.t_cmd = self.r_cmd = 0.0  # remembered gripper tilt and roll (only J/L and U/O change them)
        self.t_move = 0.0
        self.no_torque = True     # until we know - no motion
        self.g = self.opened
        self.dir, self.dir_t = None, 0.0
        self.speed = "normal"
        self.state = {"connected": False, "message": "connecting to the RoArm..."}
        self.commands = []        # one-shot: gripper, stop
        self._lock = threading.Lock()
        self.hot = False
        self.playing = {"running": False, "log": []}  # the taught pick sequence (roarm_pick.play) in a thread
        self._stop = threading.Event()
        self.stopped = None       # "auto" / "pick": the last run ended in a Stop or failure -> Continue / Abort shown
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._websocket, daemon=True).start()

    def _websocket(self):
        """Temperatures of the 7 servos (T:1051 "temp", ~1.5/s) and alarms (T:-15: overload, overheat, voltage)
        from the RoArm WebSocket - the same data its own web page warns from."""
        try:
            import websocket
        except ImportError:
            self.state["servos"] = "websocket-client missing (pip install websocket-client)"
            return
        while True:
            try:
                ws = websocket.create_connection(f"ws://{self.ip}/ws", timeout=5)
                while True:
                    d = json.loads(ws.recv())
                    if d.get("T") == 1051:
                        self._temperatures(d)
                    elif d.get("T") == -15:
                        self.state["alarms"] = {"overload": bool(d.get("Stalltor")),
                                                "overheat": bool(d.get("Stalltep")),
                                                "voltage": {1: "too high", 2: "too low"}.get(d.get("Stallvol"))}
            except Exception:  # RoArm off / rebooting - keep trying
                self.state.pop("servo_temps", None)
                time.sleep(2)

    def _temperatures(self, d):
        """T:1051 "temp" -> state + overheat lock. From /ws (WiFi) or from /js (USB via roarm_usb.py - no /ws)."""
        temp = d.get("temp")
        if isinstance(temp, list) and temp:
            self.state["servo_temps"] = dict(zip(ROARM_SERVOS, temp))
            self.state["temp_t"] = time.time()
            hottest = max(temp)
            self.hot = hottest >= SERVO_HOT[1] or (self.hot and hottest > SERVO_HOT[0])

    # ------------------------------------------------------------------ RoArm
    def _js(self, cmd, timeout=1.5):
        r = requests.get(f"http://{self.ip}/js", params={"json": json.dumps(cmd, separators=(",", ":"))},
                         timeout=timeout)
        return r.text

    def _where(self):
        d = json.loads(self._js({"T": 105}))
        if d.get("T") != 1051 or d.get("x") is None:
            raise RuntimeError("the RoArm reports no position (servo power?)")
        self._temperatures(d)
        return d

    def _go(self, target, spd=0.2):
        try:  # the RoArm does not answer while it moves - the command arrived anyway
            self._js({"T": 104, "x": round(target["x"], 1), "y": round(target["y"], 1), "z": round(target["z"], 1),
                      "t": round(target["t"], 3), "r": round(target["r"], 3), "g": round(self.g, 3), "spd": spd})
        except requests.Timeout:
            pass

    def _ahead_of_arm(self, d, dt):
        """New target. Position: LEAD s of motion ahead of the current position (smooth). Gripper tilt and roll:
        remembered (t_cmd, r_cmd) - like the crane hook on the SO-101: up/down/out/in do not change the tilt."""
        import math

        axis, sign = DIRECTIONS[self.dir]
        v = SPEEDS[self.speed][0]
        if axis in ("t", "r"):  # tilt / roll the gripper in place: position = last target
            angular = v * 0.012  # rad/s: ~0.5 rad/s at "normal"
            if axis == "t":
                self.t_cmd = min(max(self.t_cmd + sign * angular * dt, -3.14), 3.14)
            else:
                self.r_cmd = min(max(self.r_cmd + sign * angular * dt, -3.14), 3.14)
            return dict(self.target, t=self.t_cmd, r=self.r_cmd)
        v *= LEAD  # mm ahead of the arm
        rho, angle, z = math.hypot(d["x"], d["y"]), math.atan2(d["y"], d["x"]), d["z"]
        if axis == "reach":
            rho += sign * v
        elif axis == "turn":
            angle += sign * v / max(rho, 100.0)
        else:
            z += sign * v * self.up  # z+ = up in the world (upside down: -z of the RoArm)
        return self._clamp({"x": rho * math.cos(angle), "y": rho * math.sin(angle), "z": z,
                            "t": self.t_cmd, "r": self.r_cmd})

    def _hold(self, d, message):
        self.target = {"x": d["x"], "y": d["y"], "z": d["z"], "t": self.t_cmd, "r": self.r_cmd}
        self._go(self.target)
        self.dir = None
        self.state["message"] = message

    def _torque_here(self, d):
        """Servos without torque (arm moved by hand): each servo's goal = its measured angle, then torque on.
        The arm stays where it is - no fixed pose (something may stand there, e.g. the vehicle)."""
        self._js({"T": 102, "base": d["b"], "shoulder": d["s"], "elbow": d["e"], "wrist": d["t"], "roll": d["r"],
                  "hand": d["g"], "spd": 50, "acc": 10})
        self._js({"T": 210, "cmd": 1})  # EnableTorque only (cmd 0 first drives to a fixed pose!)
        self.target = {"x": d["x"], "y": d["y"], "z": d["z"], "t": d["tit"], "r": d.get("r", 0.0)}
        self.t_cmd, self.r_cmd, self.g = d["tit"], d.get("r", 0.0), d["g"]
        self.no_torque = False
        self.state["message"] = "motors on here - hold a button to move"

    def _joint(self, d):
        """One joint, speed * LEAD ahead of its measured angle (T:101 - only this joint)."""
        n = abs(DIRECTIONS[self.dir][1])
        sign = 1 if DIRECTIONS[self.dir][1] > 0 else -1
        w = JOINT_RAD_S[self.speed]
        rad = d[JOINTS[n][0]] + sign * w * LEAD
        try:
            self._js({"T": 101, "joint": n, "rad": round(rad, 3), "spd": int(w * 4096 / 6.283), "acc": 10})
        except requests.Timeout:
            pass
        # the "like the SO-101" mode then starts from where the arm really is
        self.target = {"x": d["x"], "y": d["y"], "z": d["z"], "t": d["tit"], "r": d.get("r", 0.0)}
        self.t_cmd, self.r_cmd = d["tit"], d.get("r", 0.0)
        if n == 6:
            self.g = rad
        self.state["message"] = f"{JOINTS[n][1]} {'+' if sign > 0 else '-'}"

    def _gripper(self, close):
        self.g = self.closed if close else self.opened
        try:
            self._js({"T": 106, "cmd": self.g, "spd": 0, "acc": 0})
        except requests.Timeout:
            pass
        self.state["message"] = "gripper closed (grabbing)" if close else "gripper open (released)"

    def _clamp(self, c):
        import math

        lo, hi = self.reach
        r = math.hypot(c["x"], c["y"])
        if r > hi or r < lo:
            k = (hi if r > hi else lo) / max(r, 1e-6)
            c["x"], c["y"] = c["x"] * k, c["y"] * k
        lo, hi = Z_RANGE if self.up > 0 else (-Z_RANGE[1], 550.0)  # hanging: down = +z, further than 350
        c["z"] = min(max(c["z"], lo), hi)
        c["t"] = min(max(c["t"], -3.14), 3.14)
        c["r"] = min(max(c["r"], -3.14), 3.14)
        return c

    def _loop(self):
        t_read = 0.0
        while True:
            time.sleep(0.05)
            now = time.time()
            if self.playing["running"]:
                continue  # the pick sequence drives the RoArm - the panel does not send anything meanwhile
            try:
                moving = bool(self.dir) and now - self.dir_t < KEEPALIVE and not self.no_torque and not self.hot
                if self.target is None or now - t_read > (0.2 if moving else 0.6):
                    d = self._where()
                    t_read = time.time()
                    self.state.update(connected=True, x=d["x"], y=d["y"], z=d["z"], t=d["tit"], r=d.get("r", 0.0),
                                      g=d.get("g"), shoulder_load=d.get("tS", 0), elbow_load=d.get("tE", 0),
                                      base_load=d.get("tB", 0))
                    # all loads 0 = servos without torque: the angles read then can be false (+-pi), and a move
                    # "relative to them" can swing the whole arm - hence _torque_here first.
                    # torswitch* = torque on (firmware); loads fail when the arm hangs (gravity along the links)
                    if "torswitchS" in d:
                        self.no_torque = not any(d.get(k) for k in ("torswitchB", "torswitchS", "torswitchE"))
                    else:
                        self.no_torque = not any(d.get(k) for k in ("tB", "tS", "tE", "tT", "tR"))
                    if self.target is None:  # just remember - no motion on connect
                        self.target = {"x": d["x"], "y": d["y"], "z": d["z"], "t": d["tit"], "r": d.get("r", 0.0)}
                        self.t_cmd, self.r_cmd = d["tit"], d.get("r", 0.0)
                        if 1.0 <= (d.get("g") or 0) <= 3.5:
                            self.g = d["g"]  # the gripper stays as it is (else the first move would open it)
                        self.state["message"] = ("servos without torque - the first move button enables them in place"
                                                 if self.no_torque else "ready - hold a button to move")
                    elif max(abs(d.get("tS", 0)), abs(d.get("tE", 0))) > LOAD_STOP:
                        self._hold(d, "OVERLOAD - the arm holds its position. Move it higher / closer to the base")
                        moving = False
                    if moving:  # target always ~0.6 s ahead: smooth motion; released button = stops 1-3 cm later
                        dt, self.t_move = min(max(t_read - self.t_move, 0.05), 0.4), t_read
                        if self.dir.startswith("j"):
                            self._joint(d)
                        else:
                            self.target = self._ahead_of_arm(d, dt)
                            self._go(self.target, spd=SPEEDS[self.speed][1])
                            self.state["message"] = "moving"
                with self._lock:
                    commands, self.commands = self.commands, []
                for c in commands:
                    if c == "stop":
                        if self.no_torque:  # false angles - do not send "hold here"
                            self.dir, self.state["message"] = None, "STOP"
                        else:
                            self._hold(self._where(), "STOP - holding position")
                    elif c in ("open", "close", "toggle"):  # toggle = SPACE: grab / release (like the SO-101)
                        close = c == "close" or (c == "toggle" and self.g < (self.opened + self.closed) / 2)
                        self._gripper(close)
                    elif isinstance(c, tuple) and c[0] == "save_pose":
                        self._save_pose(c[1], c[2] if len(c) > 2 else None)
                    elif isinstance(c, tuple) and c[0] == "goto_pose":
                        self._goto_pose(c[1])
                    elif c == "lift" and not self.no_torque:  # P: grab and lift by 10 cm
                        self._gripper(True)
                        time.sleep(0.8)  # the fingers close (no sensor on the gripper)
                        d = self._where()
                        self.target = self._clamp({"x": d["x"], "y": d["y"], "z": d["z"] + self.up * LIFT_MM,
                                                   "t": self.t_cmd, "r": self.r_cmd})
                        self._go(self.target, spd=SPEEDS["slow"][1])
                        self.state["message"] = f"grabbed, lifting by {LIFT_MM:.0f} mm"
                if self.dir and self.hot:
                    self.dir = None
                    hottest = max((self.state.get("servo_temps") or {"?": 0}).items(), key=lambda kv: kv[1])
                    self.state["message"] = (f"SERVO HOT ({hottest[0]} {hottest[1]} C) - motion blocked until "
                                             f"{SERVO_HOT[0]} C. Lower / closer to the base relieves the shoulder")
                elif self.dir and self.no_torque:
                    self._torque_here(self._where())  # the next cycle already moves
                elif self.dir and now - self.dir_t >= KEEPALIVE:
                    self.dir = None  # button released: no new targets, the arm reaches the last one and stops
                    self.state["message"] = "stopped"
            except (requests.RequestException, RuntimeError, ValueError) as e:
                self.state.update(connected=False, message=f"RoArm not answering ({type(e).__name__}) - "
                                                           "servo power? USB/WiFi?")
                self.target = None
                time.sleep(1.0)

    # ------------------------------------------------------------------ saved poses (joint angles)
    def _save_pose(self, name, route=None):
        """Current joint angles (+ x y z for reference) -> roarm_calibration.json "poses"[name]. route: a NEW pose also
        joins that route, just before its closing rest (overwriting an existing pose never changes a route)."""
        import roarm_pick

        d = self._where()
        cfg = roarm_pick.load_config()
        new = name not in (cfg.get("poses") or {})
        cfg.setdefault("poses", {})[name] = {k: round(d[k], 4) for k in ("b", "s", "e", "t", "r", "g", "x", "y", "z", "tit")}
        if new and route in roarm_pick.route_names(cfg):
            roarm_pick.add_to_route(cfg, route, name)
        roarm_pick.save_config(cfg)
        self.state["message"] = f"pose '{name}' saved" + (f", added to the {route} route" if new and route else "")

    def _goto_pose(self, name):
        """To a saved pose by joint angles (T:102) - every joint straight to its angle, slowly; no x y z maths."""
        pose = (_config().get("poses") or {}).get(name)
        if not pose:
            self.state["message"] = f"no pose '{name}'"
            return
        if self.no_torque:
            self._torque_here(self._where())  # motors on where the arm is first, then move
        w = JOINT_RAD_S[self.speed]
        self._js({"T": 102, "base": pose["b"], "shoulder": pose["s"], "elbow": pose["e"], "wrist": pose["t"],
                  "roll": pose["r"], "hand": pose["g"], "spd": int(w * 4096 / 6.283), "acc": 10})
        self.target, self.g = None, pose["g"]  # re-read the position after the move (the panel starts from there)
        self.dir = None
        self.state["message"] = f"going to pose '{name}'"

    def delete_pose(self, name):
        import roarm_pick

        cfg = roarm_pick.load_config()
        (cfg.get("poses") or {}).pop(name, None)
        roarm_pick.save_config(cfg)

    # ------------------------------------------------------------------ taught pick sequence
    def start_play(self, restart, auto=False, route="bottle", resume=False):
        """roarm_pick.play of `route` (auto: the demo loop) in a thread; its log goes to the page. False when already
        running."""
        if self.playing["running"]:
            return False
        import roarm_pick

        self.dir = None
        self._stop.clear()
        self.stopped = None
        self.playing = {"running": True, "log": ["--- " + ("AUTO: waiting for a bottle" if auto else f"{route} route " + (
            "from the start" if restart else "(continue)")) + " ---"]}
        log = self._log

        def run():
            try:
                cfg = roarm_pick.load_config()
                arm = roarm_pick.RoArm(cfg["roarm_ip"])
                if auto:
                    roarm_pick.auto(arm, roarm_pick.SO101(cfg["so101_url"]), cfg, log=log, stop=self._stop,
                                    resume=resume)  # Continue: on at the stopped route step, no new detection
                else:
                    roarm_pick.play(arm, cfg, restart=restart, log=log, stop=self._stop, route=route)
            except roarm_pick.Stopped as e:
                self.stopped = "auto" if auto else route
                log(f"STOPPED: {e} - Continue resumes, Abort mission resets both arms")
            except Exception as e:  # any failure: shown on the page, progress kept for Continue
                self.stopped = "auto" if auto else route
                log(f"ERROR: {e} - Continue resumes at the failed step, Abort mission resets both arms")
            finally:
                self.playing["running"] = False
                self.target = None  # the panel re-reads the position before its next move

        threading.Thread(target=run, daemon=True).start()
        return True

    def _log(self, text):
        self.playing["log"] = (self.playing["log"] + [text])[-60:]
        self.state["message"] = text.strip()

    def start_abort(self):
        """After a Stop: abort the mission in total (roarm_pick.abort - progress cleared, RoArm to rest, SO-101 to M)
        in a thread, logged like a pick. False while something runs (Stop it first)."""
        if self.playing["running"]:
            return False
        import roarm_pick

        self.dir = None
        self.stopped = None
        self.playing = {"running": True, "log": self.playing["log"] + ["--- ABORT: RoArm to rest, SO-101 to M ---"]}

        def run():
            try:
                cfg = roarm_pick.load_config()
                roarm_pick.abort(roarm_pick.RoArm(cfg["roarm_ip"]), roarm_pick.SO101(cfg["so101_url"]), cfg,
                                 log=self._log)
            except Exception as e:  # RoArm not at rest -> the SO-101 was not moved either
                self._log(f"ABORT FAILED: {e}")
            finally:
                self.playing["running"] = False
                self.target = None

        threading.Thread(target=run, daemon=True).start()
        return True

    def data(self):
        import roarm_pick

        cfg = _config()
        routes = {r: roarm_pick.sequence_of(cfg, r) for r in roarm_pick.route_names(cfg)}
        stopped = self.stopped or roarm_pick.paused_route(cfg)  # "auto" or the stopped route (also after a restart)
        return dict(self.state, speed=self.speed, poses=cfg.get("poses") or {}, playing=self.playing,
                    sequence=routes["bottle"], routes=routes, paused=bool(stopped), stopped=stopped)


_panel = None


# Pages of the station's web site (so101_station.py serves them all): path, menu label, tab title, tab icon (emoji)
PAGES = [("/roarm_panel", "RoArm panel", "RoArm · Admin panel", "\U0001F9BE"),
         ("/so101", "SO-101 camera", "SO-101 · Camera arm", "\U0001F4F7"),
         ("/demo", "Live demo", "Dorm-Keeper · Live demo", "\U0001F3AC")]
MENU_CSS = """<style>#menu{position:sticky;top:0;z-index:50;display:flex;gap:4px;align-items:center;flex-wrap:wrap;
padding:6px 12px;background:#070a0e;border-bottom:1px solid #243040;font:600 14px system-ui,"Segoe UI",sans-serif}
#menu a{color:#8a9aac;text-decoration:none;padding:7px 12px;border-radius:8px}#menu a:hover{background:#18212c;color:#e8eef5}
#menu a.on{background:#18212c;color:#4cc2ff}#menu b{color:#e8eef5;margin-right:10px}:fullscreen #menu{display:none}</style>"""


def with_menu(page, path):
    """Any station page (bytes) + the menu strip on top, its tab title and emoji tab icon. The demo page is a
    full-screen grid (rows: header, main) - it gets one more row for the menu; full screen hides the menu."""
    _, _, title, icon = next(p for p in PAGES if p[0] == path)
    favicon = ('<link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 '
               f'viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>{icon}</text></svg>">')
    extra = "<style>body{grid-template-rows:auto auto 1fr}#menu{margin:-18px -22px 0}</style>" if path == "/demo" else ""
    menu = '<nav id="menu"><b>Hackengersi</b>' + "".join(
        f'<a href="{p}"{" class=on" if p == path else ""}>{i} {label}</a>' for p, label, _, i in PAGES) + "</nav>"
    html = page.decode("utf-8")
    html = re.sub(r"<title>.*?</title>", f"<title>{title}</title>", html, count=1, flags=re.S)
    html = html.replace("</head>", favicon + MENU_CSS + extra + "</head>", 1)
    html = re.sub(r"(<body[^>]*>)", lambda m: m.group(1) + menu, html, count=1)
    return html.encode("utf-8")


def handle(h, path, q):
    """Serve /roarm_panel and /roarm/... (h = the so101_station.py request handler: has _json)."""
    global _panel
    if path == "/roarm_panel":
        body = with_menu(HTML.encode(), path)
        h.send_response(200)
        h.send_header("Content-Type", "text/html; charset=utf-8")
        h.send_header("Content-Length", str(len(body)))
        h.end_headers()
        h.wfile.write(body)
        return
    if _panel is None:
        _panel = RoArmPanel()
    p = _panel
    if path == "/roarm/state":
        return h._json(p.data())
    if path == "/roarm/move" and q.get("k") in DIRECTIONS:
        p.dir, p.dir_t = q["k"], time.time()
        return h._json({"ok": True})
    if path == "/roarm/speed" and q.get("v") in SPEEDS:
        p.speed = q["v"]
        return h._json({"ok": True})
    if path in ("/roarm/save_pose", "/roarm/goto_pose") and POSE_NAME.match(q.get("name", "")):
        with p._lock:
            p.commands.append((path[7:], q["name"], q.get("route") or None))
        return h._json({"ok": True})
    if path == "/roarm/delete_pose" and POSE_NAME.match(q.get("name", "")):
        p.delete_pose(q["name"])
        return h._json({"ok": True})
    if path == "/roarm/play" and q.get("from") in ("continue", "start", "auto"):
        import roarm_pick

        route = q.get("route", "bottle")
        if route not in roarm_pick.route_names(_config()):
            return h._json({"error": f"no route '{route}'"}, 404)
        return h._json({"ok": p.start_play(q["from"] == "start", auto=q["from"] == "auto", route=route,
                                           resume=q.get("resume") == "1")})
    if path == "/roarm/command" and q.get("c") == "stop" and p.playing["running"]:
        p._stop.set()  # the pick sequence holds the arm where it is and stops
        return h._json({"ok": True})
    if path == "/roarm/abort":  # after Stop, confirmed on the page: the whole mission is dropped
        return h._json({"ok": p.start_abort()})
    if path == "/roarm/command" and q.get("c") in COMMANDS:
        with p._lock:
            p.commands.append(q["c"])
        return h._json({"ok": True})
    h._json({"error": "unknown RoArm panel command"}, 404)


HTML = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>RoArm panel</title><style>
:root{--bg:#0b0f14;--card:#121922;--line:#243040;--text:#e8eef5;--dim:#8a9aac;--accent:#4cc2ff;--ok:#2ecc71;--bad:#ff5c5c;--warn:#ffb020}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,"Segoe UI",sans-serif}
main{display:grid;grid-template-columns:minmax(0,1.4fr) minmax(320px,1fr);gap:14px;padding:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}
h1{font-size:20px;margin:0 0 4px}h2{font-size:13px;text-transform:uppercase;letter-spacing:1px;color:var(--dim);margin:14px 0 8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
img{width:100%;border-radius:10px;background:#000;display:block}
#msg{font-weight:700;padding:10px 12px;border-radius:8px;background:#1a2330;margin:10px 0 0;font-size:17px}
#msg.bad{background:#4d1f22;color:#ffb3b3}
.pos{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;font-variant-numeric:tabular-nums}
.pos div{background:#1a2330;border-radius:8px;padding:6px 8px}.pos b{display:block;font-size:18px}.pos span{font-size:12px;color:var(--dim)}
.bar{height:8px;background:#1a2330;border-radius:4px;overflow:hidden;margin:3px 0 8px}.bar i{display:block;height:100%;background:var(--ok);width:0}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
button{font:inherit;padding:12px 6px;border:0;border-radius:8px;background:#243040;color:var(--text);cursor:pointer;touch-action:none;user-select:none}
button:active,button.on{background:var(--accent);color:#06121e}button small{display:block;font-size:11px;opacity:.7}
#stop{background:var(--bad);font-weight:800;font-size:18px}.green{background:#1b5e3a}
.sys .row{display:grid;grid-template-columns:150px 1fr 74px;gap:8px;align-items:center;font-size:13px}
.sys .row .bar{margin:6px 0}.sys b{text-align:right;font-variant-numeric:tabular-nums}
#warnings div{margin-top:6px;padding:7px 10px;border-radius:8px;background:#4d1f22;color:#ffb3b3;font-weight:700;font-size:13px}
#warnings div.yellow{background:#4d3a12;color:#ffd98a}
details{margin-top:8px;font-size:13px;color:var(--dim)}#servos{display:grid;grid-template-columns:1fr 1fr;gap:2px 12px;margin-top:6px;font-variant-numeric:tabular-nums}
</style></head><body><main>
<div><div class="card"><h1>RoArm control</h1><span style="color:var(--dim)">SO-101 camera image (what the program sees)</span>
<img src="/preview?clean=1&fps=8" alt="SO-101 camera"><div id="msg">connecting...</div></div>
<div class="card" style="margin-top:14px">
<h2>Pick sequence</h2><div class="grid">
<button id="chicken" class="green">&#128020; Pick the chicken<small>chicken route from the start</small></button>
<button id="playstart">Play from start<small>bottle route</small></button><button id="playstop" style="background:var(--bad)">Stop</button>
<div id="stopped" style="grid-column:1/4;display:none;grid-template-columns:1fr 1fr;gap:6px">
<button id="continue" class="green">Continue<small>from the stopped step</small></button>
<button id="abort" style="background:var(--warn);color:#000">Abort mission<small>RoArm to rest, SO-101 to M</small></button></div>
<button id="auto" class="green" style="grid-column:1/4">AUTO: SO-101 show (2 s) -> SO-101 moves away -> RoArm picks the bottle, then the &#128020; chicken -> rest</button></div>
<div id="seq" style="margin-top:6px;font-size:12px;color:var(--dim);white-space:pre-line"></div>
<pre id="playlog" style="background:#05080b;border-radius:8px;padding:8px;height:150px;overflow:auto;font-size:12px;margin:8px 0 0;white-space:pre-wrap">(pick log)</pre>
<h2>Saved poses (joint angles)</h2><div class="grid" style="grid-template-columns:1.6fr 1.3fr 1fr">
<input id="posename" placeholder="name, e.g. above_neck" style="font:inherit;padding:10px;border-radius:8px;border:1px solid var(--line);background:#0b0f14;color:var(--text)">
<select id="saveroute" title="a new pose joins this route, just before its closing rest" style="font:inherit;padding:10px;border-radius:8px;border:1px solid var(--line);background:#0b0f14;color:var(--text)"><option value="">add to: no route</option></select>
<button id="savepose" class="green">Save pose</button></div>
<div id="poses" style="margin-top:6px;font-size:13px;font-variant-numeric:tabular-nums"></div></div></div>
<div class="card">
<div class="pos"><div><span>reach</span><b id="px">-</b></div><div><span>base angle</span><b id="py">-</b></div><div><span>height</span><b id="pz">-</b></div></div>
<h2>System</h2><div class="sys">
<div class="row"><span>Raspberry CPU</span><div class="bar"><i id="s-cpu"></i></div><b id="t-cpu">-</b></div>
<div class="row"><span>Raspberry RAM</span><div class="bar"><i id="s-ram"></i></div><b id="t-ram">-</b></div>
<div class="row"><span>Raspberry temp.</span><div class="bar"><i id="s-temp"></i></div><b id="t-temp">-</b></div>
<div class="row"><span>SO-101 hottest servo</span><div class="bar"><i id="s-so"></i></div><b id="t-so">-</b></div>
<div class="row"><span>RoArm hottest servo</span><div class="bar"><i id="s-ra"></i></div><b id="t-ra">-</b></div>
<div class="row"><span>RoArm shoulder load</span><div class="bar"><i id="lshoulder"></i></div><b id="t-shoulder">-</b></div>
<div class="row"><span>RoArm elbow load</span><div class="bar"><i id="lelbow"></i></div><b id="t-elbow">-</b></div>
<div id="warnings"></div><details><summary>all servos</summary><div id="servos"></div></details></div>
<h2>Where are you looking from?</h2><div class="grid" style="grid-template-columns:1fr 1fr">
<button data-view="front">In FRONT of it<small>facing the robot</small></button><button data-view="back">BEHIND it<small>looking where it looks</small></button></div>
<h2>Joints - each button moves ONE joint (hold)</h2><div class="grid" style="grid-template-columns:1.3fr 1fr 1fr">
<span>base</span><button data-k="j1-">-</button><button data-k="j1+">+</button>
<span>shoulder</span><button data-k="j2-">-</button><button data-k="j2+">+</button>
<span>elbow</span><button data-k="j3-">-</button><button data-k="j3+">+</button>
<span>wrist</span><button data-k="j4-">-</button><button data-k="j4+">+</button>
<span>gripper roll</span><button data-k="j5-">-</button><button data-k="j5+">+</button>
<span>gripper</span><button data-k="j6-">-</button><button data-k="j6+">+</button></div>
<h2>Move - like the SO-101 (hold, release = stop; the gripper keeps its angle like a crane hook)</h2><div class="grid">
<button data-k="z+">UP</button><button data-k="reach+">OUT<small>away from base</small></button><button data-l="1">LEFT</button>
<button data-k="z-">DOWN</button><button data-k="reach-">IN<small>toward base</small></button><button data-l="-1">RIGHT</button>
<button data-k="t+">tilt gripper</button><button data-k="t-">tilt gripper</button><button id="stop">STOP</button>
<button data-k="r+">roll gripper</button><button data-k="r-">roll gripper</button><span></span></div>
<h2>Gripper</h2><div class="grid">
<button data-c="toggle">GRAB / RELEASE</button><button data-c="lift" class="green">GRAB AND LIFT<small>10 cm up</small></button><button data-c="open">open</button></div>
<h2>Speed</h2><div class="grid"><button data-v="slow">slow<small>aiming</small></button><button data-v="normal" class="on">normal</button><button data-v="fast">fast</button></div>
</div></main><script>
const $=id=>document.getElementById(id),get=u=>fetch(u).then(r=>r.json()).catch(()=>({}));
let held=null,timer=null;
function move(k){if(held===k)return;stop();held=k;get('/roarm/move?k='+encodeURIComponent(k));timer=setInterval(()=>get('/roarm/move?k='+encodeURIComponent(k)),100)}
function stop(){clearInterval(timer);timer=null;held=null}
// "left" = YOUR left: standing in front of the robot its left is your right (base turn + = robot's left)
let view='front';try{view=localStorage.getItem('roarm_view')||'front'}catch(e){}
const left=sign=>(view==='back'?1:-1)*sign>0?'turn+':'turn-';
function setView(v){view=v;try{localStorage.setItem('roarm_view',v)}catch(e){}
 document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('on',b.dataset.view===v))}setView(view);
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setView(b.dataset.view));
document.querySelectorAll('[data-k],[data-l]').forEach(b=>{const k=()=>b.dataset.k||left(+b.dataset.l);
 b.onpointerdown=e=>{b.setPointerCapture(e.pointerId);move(k())};b.onpointerup=b.onpointercancel=stop});
document.querySelectorAll('[data-c]').forEach(b=>b.onclick=()=>get('/roarm/command?c='+b.dataset.c));
document.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>speed(b.dataset.v));
$('stop').onclick=()=>{stop();get('/roarm/command?c=stop')};
// no keyboard shortcuts: typing a pose name must not move the arm - buttons only
function speed(v){get('/roarm/speed?v='+v);document.querySelectorAll('[data-v]').forEach(x=>x.classList.toggle('on',x.dataset.v===v))}
onblur=stop;
function bar(el,v){const p=Math.min(100,Math.abs(v||0)/350*100);el.style.width=p+'%';el.style.background=p>80?'var(--bad)':p>55?'var(--warn)':'var(--ok)'}
// gauge: bar up to "max", colour from the yellow/red thresholds, text next to it
function gauge(id,v,max,yellow,red,text){const el=$('s-'+id),t=$('t-'+id);
 if(v==null||isNaN(v)){el.style.width='0';t.textContent='-';return}
 el.style.width=Math.min(100,Math.max(0,v)/max*100)+'%';el.style.background=v>=red?'var(--bad)':v>=yellow?'var(--warn)':'var(--ok)';t.textContent=text}
const hottest=o=>o?Object.entries(o).reduce((a,b)=>b[1]>a[1]?b:a,['',-1]):null;
let roarm={};
async function refreshSystem(){const s=await get('/system'),w=[];
 gauge('cpu',s.cpu,100,70,90,s.cpu!=null?Math.round(s.cpu)+' %':'-');
 gauge('ram',s.ram,100,75,90,s.ram!=null?Math.round(s.ram)+' %':'-');
 gauge('temp',s.temp,90,75,82,s.temp!=null?s.temp.toFixed(0)+' C'+(s.clock_mhz?' '+(s.clock_mhz/1000).toFixed(1)+'GHz':''):'-');
 const so=hottest(s.so101&&s.so101.temp);gauge('so',so&&so[1],75,55,65,so&&so[1]>=0?so[1]+' C':'-');
 const ra=hottest(roarm.servo_temps);gauge('ra',ra&&ra[1],75,55,65,ra&&ra[1]>=0?ra[1]+' C':'-');
 if(s.throttled)w.push(['Raspberry overheating - clock throttled (fan!)','']);
 if(s.undervoltage)w.push(['Raspberry: power supply voltage too low','']);
 if(so&&so[1]>=55)w.push(['SO-101: '+so[0]+' '+so[1]+' C',so[1]>=65?'':'yellow']);
 if(ra&&ra[1]>=55)w.push(['RoArm: '+ra[0]+' '+ra[1]+' C'+(ra[1]>=65?' - motion blocked, let it rest':' - relieve it (lower / closer to the base)'),ra[1]>=65?'':'yellow']);
 const al=roarm.alarms||{};if(al.overload)w.push(['RoArm: servo OVERLOAD (motion blocked?)','']);
 if(al.overheat)w.push(['RoArm: servo OVERHEAT - switch off and wait','']);if(al.voltage)w.push(['RoArm: voltage '+al.voltage,'']);
 $('warnings').innerHTML=w.map(u=>`<div class="${u[1]}">${u[0]}</div>`).join('');
 const all=[];if(s.so101&&s.so101.temp)for(const[k,v]of Object.entries(s.so101.temp))all.push(`<span>SO-101 ${k}: ${v} C${s.so101.voltage&&s.so101.voltage[k]?' / '+s.so101.voltage[k].toFixed(1)+' V':''}</span>`);
 if(roarm.servo_temps)for(const[k,v]of Object.entries(roarm.servo_temps))all.push(`<span>RoArm ${k}: ${v} C</span>`);
 $('servos').innerHTML=all.join('')||'no data';setTimeout(refreshSystem,1000)}refreshSystem();
// editing a pose: "edit" picks it from any folder (the RoArm goes there), adjust it, Save pose overwrites it
let editing=null;
function markEditing(){const n=$('posename').value.trim(),known=!!(lastState.poses||{})[n];editing=known?n:null;
 $('savepose').innerHTML=known?'Save changes<small>overwrite '+n+'</small>':'Save pose';
 document.querySelectorAll('#poses [data-row]').forEach(r=>r.style.outline=r.dataset.row===editing?'2px solid var(--accent)':'')}
$('posename').oninput=markEditing;
$('savepose').onclick=()=>{const n=$('posename').value.trim();if(!/^[A-Za-z0-9_-]{1,32}$/.test(n)){alert('Name: letters, digits, _ or - (max 32)');return}
 if((lastState.poses||{})[n]&&!confirm('Overwrite pose '+n+' with where the RoArm is now?'))return;
 get('/roarm/save_pose?name='+n+'&route='+encodeURIComponent($('saveroute').value))};
// route picker next to Save pose: a NEW pose goes into that route just before its closing rest; the choice is remembered
let routeChoice='';try{routeChoice=localStorage.getItem('roarm_saveroute')||''}catch(e){}
$('saveroute').onchange=()=>{routeChoice=$('saveroute').value;try{localStorage.setItem('roarm_saveroute',routeChoice)}catch(e){}};
function showRoutePicker(routes){const names=Object.keys(routes||{}),key=names.join();if($('saveroute').dataset.k===key)return;
 $('saveroute').dataset.k=key;$('saveroute').innerHTML='<option value="">add to: no route</option>'+names.map(r=>`<option value="${r}">add to: ${ROUTE_ICON[r]||''} ${r} route</option>`).join('');
 $('saveroute').value=names.includes(routeChoice)?routeChoice:''}
let posesShown='';const closed=new Set();  // folders the user folded stay folded when the list is redrawn
function poseRow(n,v,step){return `<div data-row="${n}" style="display:grid;grid-template-columns:1fr auto auto auto;gap:6px;align-items:center;margin:4px 0;border-radius:8px;padding:2px">
 <span>${step?`<small style="color:var(--dim)">${step}.</small> `:''}<b>${n}</b><br>${v?`<small style="color:var(--dim)">b ${v.b.toFixed(2)} s ${v.s.toFixed(2)} e ${v.e.toFixed(2)} t ${v.t.toFixed(2)} r ${v.r.toFixed(2)} g ${v.g.toFixed(2)}</small>`:'<small style="color:var(--bad)">not saved</small>'}</span>
 <button data-go="${n}">go</button><button data-edit="${n}">edit</button><button data-del="${n}">del</button></div>`}
function folder(id,title,rows,icon){return `<details data-f="${id}"${closed.has(id)?'':' open'} style="border:1px solid var(--line);border-radius:8px;padding:6px 10px;margin:6px 0">
 <summary style="cursor:pointer;font-weight:700">${icon||'&#128193;'} ${title}</summary>${rows}</details>`}
const ROUTE_ICON={bottle:'&#127870;',chicken:'&#128020;'};  // one folder per route: the bottle, the chicken, ...
function showPoses(p,routes){p=p||{};routes=routes||{};const key=JSON.stringify([p,routes]);if(key===posesShown)return;posesShown=key;
 const used=new Set(Object.values(routes).flat()),other=Object.keys(p).filter(n=>!used.has(n));
 $('poses').innerHTML=Object.entries(routes).filter(([r,seq])=>seq.length).map(([r,seq])=>folder('route-'+r,
   `${r} route <small style="color:var(--dim)">${seq.length} steps</small>`,seq.map((n,i)=>poseRow(n,p[n],i+1)).join(''),ROUTE_ICON[r])).join('')
  +(other.length?folder('other',`other poses <small style="color:var(--dim)">${other.length}, not in a route</small>`,other.map(n=>poseRow(n,p[n])).join('')):'')
  ||'<small style="color:var(--dim)">none yet</small>';
 document.querySelectorAll('#poses details').forEach(d=>d.ontoggle=()=>d.open?closed.delete(d.dataset.f):closed.add(d.dataset.f));
 document.querySelectorAll('[data-go]').forEach(b=>b.onclick=()=>{if(confirm('Move the RoArm to pose '+b.dataset.go+'?'))get('/roarm/goto_pose?name='+b.dataset.go)});
 document.querySelectorAll('[data-edit]').forEach(b=>b.onclick=()=>{const n=b.dataset.edit;
  if(!confirm('Edit pose '+n+'?\\n\\nThe RoArm moves to it. Adjust it with the controls, then press Save changes to overwrite '+n+'.'))return;
  $('posename').value=n;get('/roarm/goto_pose?name='+n);markEditing();$('posename').scrollIntoView({block:'center'})});
 document.querySelectorAll('[data-del]').forEach(b=>b.onclick=()=>{if(confirm('Delete pose '+b.dataset.del+'?'))get('/roarm/delete_pose?name='+b.dataset.del)});
 markEditing()}
$('chicken').onclick=()=>{if(confirm('Pick the chicken? The RoArm plays the chicken route from its first pose.'))get('/roarm/play?from=start&route=chicken')};
$('playstart').onclick=()=>{if(confirm('Run the whole pick sequence from the first pose?'))get('/roarm/play?from=start')};
$('playstop').onclick=()=>get('/roarm/command?c=stop');
let actedAt=0;const hideStopped=()=>{actedAt=Date.now();$('stopped').style.display='none'};
let lastState={};  // Continue resumes what was stopped: AUTO again (its route resumes at the stopped step) or the pick
$('continue').onclick=()=>{hideStopped();get('/roarm/play?from='+(lastState.stopped==='auto'?'auto&resume=1':'continue&route='+(lastState.stopped||'bottle')))};
$('abort').onclick=()=>{if(confirm('Abort the whole mission?\\n\\nThe RoArm goes back to its rest pose, the SO-101 to its M pose, and the stopped progress is discarded (the next Play pick starts from the first pose).')){hideStopped();get('/roarm/abort')}};
$('auto').onclick=()=>{if(confirm('Start AUTO? The SO-101 does its show for 2 s and moves away, then the RoArm picks the bottle and right away the chicken, then goes back to rest.'))get('/roarm/play?from=auto')};
function showPlay(d){lastState=d;const p=d.playing||{};$('chicken').disabled=$('playstart').disabled=$('auto').disabled=!!p.running;
 // Continue / Abort only after a Stop (or failure): gone while anything runs and right after one is clicked
 $('stopped').style.display=!p.running&&d.paused&&Date.now()-actedAt>2000?'grid':'none';
 $('seq').textContent=Object.entries(d.routes||{}).map(([r,seq])=>r+' route: '+seq.join(' > ')).join('\\n');
 if(p.log&&p.log.length){const l=$('playlog'),t=p.log.join('\\n');if(l.textContent!==t){l.textContent=t;l.scrollTop=l.scrollHeight}}}
async function refresh(){const d=await get('/roarm/state');showPlay(d);showPoses(d.poses,d.routes);showRoutePicker(d.routes);if(d.x!==undefined){$('px').textContent=Math.round(Math.hypot(d.x,d.y))+' mm';$('py').textContent=Math.round(Math.atan2(d.y,d.x)*180/Math.PI)+' deg';$('pz').textContent=Math.round(d.z)+' mm'}
 roarm=d;bar($('lshoulder'),d.shoulder_load);bar($('lelbow'),d.elbow_load);
 $('t-shoulder').textContent=d.shoulder_load!=null?Math.abs(d.shoulder_load):'-';$('t-elbow').textContent=d.elbow_load!=null?Math.abs(d.elbow_load):'-';
 $('msg').textContent=d.message||'';$('msg').className=(!d.connected||/OVERLOAD/.test(d.message||''))?'bad':'';
 setTimeout(refresh,500)}refresh();
</script></body></html>"""
