"""RoArm over USB, served on localhost with the same HTTP API as the arm's WiFi: roarm_ip = "localhost:8766".

The arm's port labelled USB (CP2102) goes into the Pi. Firmware 0.84-temp prints its feedback line
(T:1051: pose, loads, "temp", "v") on serial at 10 Hz and takes the same JSON commands as over WiFi, so:
GET /js?json=<cmd>  ->  cmd goes down the cable, the reply is the latest feedback line (like the WiFi firmware).
T:105 is not forwarded: the 10 Hz stream already holds its answer. No /ws (roarm_panel reads temps from /js).

    python roarm_usb.py            serve on 127.0.0.1:8766 (Pi: systemd user service roarm-usb)
    python roarm_usb.py --test     self-check without hardware
"""
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PORT = 8766
STALE_S = 1.0  # no feedback line for this long = arm off or rebooting -> 503 (callers: "not answering")
MAX_CMD = 255  # firmware line buffer

_latest = {"line": None, "t": 0.0}
_write_lock = threading.Lock()


def find_port():
    from serial.tools import list_ports

    ports = [p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x10C4, 0xEA60)]  # CP2102 (SO-101: 1A86)
    if not ports:
        raise SystemExit("RoArm USB (CP2102) not found: is the arm's port labelled USB plugged into the Pi?")
    return ports[0]


def open_serial(dev):
    import serial

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = dev, 115200, 1
    s.dtr = s.rts = False  # before open(): pyserial's default toggles them and resets the ESP32 (arm runs its boot pose)
    s.open()
    return s


def take(line):
    """One serial line; keeps the newest feedback line."""
    line = line.strip()
    if line.startswith(b'{"T":1051'):
        _latest.update(line=line, t=time.monotonic())


def reader(ser):
    while True:
        take(ser.readline())  # SerialException (cable out) kills the thread -> main exits -> systemd restarts


class Handler(BaseHTTPRequestHandler):
    def _reply(self, code, body):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path != "/js":
            return self._reply(404, b'{"error":"only /js"}')
        if time.monotonic() - _latest["t"] > STALE_S:  # checked first: never send motion to an arm we can't see
            return self._reply(503, b'{"error":"no feedback from the arm over USB (power off? rebooting?)"}')
        cmd = parse_qs(u.query).get("json", [""])[0].strip()
        if cmd and '"T":105' not in cmd.replace(" ", ""):
            if len(cmd) >= MAX_CMD:
                return self._reply(400, b'{"error":"command longer than the firmware accepts (255)"}')
            with _write_lock:
                self.server.ser.write(cmd.encode() + b"\n")
        self._reply(200, _latest["line"])

    def log_message(self, *a):
        pass


def serve(ser, port=PORT):
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)  # localhost only: nothing else on the network drives the arm
    srv.ser = ser
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def test():
    import json
    import urllib.error
    import urllib.request

    class FakeSerial:
        written = []

        def write(self, b):
            self.written.append(b)

    ser = FakeSerial()
    srv = serve(ser, port=0)
    url = f"http://127.0.0.1:{srv.server_address[1]}/js?json="
    try:
        urllib.request.urlopen(url + '{"T":104}')
        raise SystemExit("stale feedback must refuse commands")
    except urllib.error.HTTPError as e:
        assert e.code == 503
    assert ser.written == []
    take(b"boot noise\r\n")
    take(b'{"T":1051,"x":1,"temp":[30,40,30,30,30,30,30],"v":1204}\r\n')
    d = json.load(urllib.request.urlopen(url + '{"T":105}'))
    assert d["temp"][1] == 40 and ser.written == [], "T:105 is answered from the stream, not forwarded"
    json.load(urllib.request.urlopen(url + '{"T":104,"x":250}'))
    assert ser.written == [b'{"T":104,"x":250}\n']
    srv.shutdown()
    print("usb bridge ok")


if __name__ == "__main__":
    if "--test" in sys.argv:
        test()
        sys.exit()
    dev = find_port()
    ser = open_serial(dev)
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()
    serve(ser)
    print(f"RoArm on {dev} -> http://127.0.0.1:{PORT}/js", flush=True)
    t.join()
    sys.exit("serial reader stopped (USB unplugged?)")
