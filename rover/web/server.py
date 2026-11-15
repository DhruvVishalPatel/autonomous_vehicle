"""Dashboard web server (standard library only).
GET / page, /api/state, /api/map, /api/maps · POST /api/joy {x,y}, /api/estop {on},
/api/map/save {name}, /api/map/load {name}, /api/map/clear"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PAGE = (Path(__file__).parent / "index.html").read_bytes()


def serve(robot, port):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, json.dumps(robot.state()).encode(), "application/json")
            elif self.path == "/api/map":
                self._send(200, robot.map_bytes(), "application/octet-stream")
            elif self.path == "/api/maps":
                self._send(200, json.dumps(robot.list_maps()).encode(), "application/json")
            else:
                self._send(404, b"", "text/plain")

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n) or b"{}")
                if self.path == "/api/joy":
                    robot.joystick(d.get("x", 0), d.get("y", 0))
                elif self.path == "/api/estop":
                    robot.set_estop(d.get("on", True))
                elif self.path == "/api/map/save":
                    robot.save_map(d.get("name", "map"))
                elif self.path == "/api/map/load":
                    robot.load_map(d["name"])
                elif self.path == "/api/map/clear":
                    robot.clear_map()
                elif self.path == "/api/goto":
                    robot.go_to(float(d["x"]), float(d["y"]))
                elif self.path == "/api/cancel":
                    robot.nav.cancel("cancelled")
                else:
                    return self._send(404, b"", "text/plain")
                self._send(200, b'{"ok":true}', "application/json")
            except Exception as e:
                self._send(400, json.dumps({"error": str(e)}).encode(), "application/json")

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv
