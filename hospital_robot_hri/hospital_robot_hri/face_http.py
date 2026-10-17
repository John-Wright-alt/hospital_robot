"""
face_http.py  --  tiny web server for the face page. Python standard library
only (no FastAPI, no internet needed), and NO ROS code, so it can be tested on
its own.

    GET  /           the face page (web/index.html)
    GET  /state      the latest face state as JSON (handy with curl)
    GET  /events     Server-Sent Events: the browser receives every new state
    POST /touch      a tap on the screen, or a menu choice (Anna's wake-up flow):
                     {"kind": "tap"}  or  {"kind": "choice", "value": "pharmacy"}

The ONLY things a browser can send are a tap and a menu choice, and by default
only the robot's own screen (127.0.0.1) may send them. Other computers get
"403 Forbidden" unless allow_remote_touch is switched on for testing. A tap
can at most start the wake-up flow; it cannot arm the robot or drive it.
"""

import json
import re
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class FaceHttpServer:

    LOCAL = ('127.0.0.1', '::1', '::ffff:127.0.0.1')

    def __init__(self, web_dir: str, host: str = '0.0.0.0', port: int = 8080,
                 on_touch=None, allow_remote_touch: bool = False):
        index = os.path.join(web_dir, 'index.html')
        with open(index, 'rb') as f:              # fail early and clearly if missing
            self._index = f.read()
        self._web_dir = web_dir
        self._latest = json.dumps({'expression': 'sleeping', 'gaze_data': 'starting'})
        self._seq = 0
        self._cond = threading.Condition()
        self._stopping = False
        self.clients = 0
        self.on_touch = on_touch                 # callable(dict) or None (= POST disabled)
        self.allow_remote_touch = allow_remote_touch
        self.touches = 0
        self.rejected = 0

        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):          # keep the ROS log clean
                pass

            def _send(self, code, ctype, body: bytes):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if path in ('/', '/index.html'):
                    outer._reload_if_changed()
                    self._send(200, 'text/html; charset=utf-8', outer._index)
                elif path == '/state':
                    self._send(200, 'application/json', outer._latest.encode())
                elif path == '/events':
                    outer._stream(self)
                else:
                    self._send(404, 'text/plain', b'not found')

            def do_POST(self):
                path = self.path.split('?', 1)[0]
                if path != '/touch' or outer.on_touch is None:
                    self._send(404, 'text/plain', b'not found')
                    return
                if not outer.allow_remote_touch and self.client_address[0] not in outer.LOCAL:
                    outer.rejected += 1
                    self._send(403, 'text/plain', b'touch is only accepted from the robot screen')
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except ValueError:
                    length = -1
                if not 0 < length <= 512:
                    self._send(400, 'text/plain', b'bad length')
                    return
                touch = outer.parse_touch(self.rfile.read(length))
                if touch is None:
                    self._send(400, 'text/plain', b'bad touch message')
                    return
                outer.touches += 1
                try:
                    outer.on_touch(touch)
                finally:
                    self._send(204, 'text/plain', b'')

        self._httpd = ThreadingHTTPServer((host, port), Handler)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    # ------------------------------------------------------------------
    def _reload_if_changed(self):
        """With colcon --symlink-install you can edit index.html and just
        refresh the browser."""
        try:
            with open(os.path.join(self._web_dir, 'index.html'), 'rb') as f:
                self._index = f.read()
        except OSError:
            pass

    def _stream(self, handler):
        handler.send_response(200)
        handler.send_header('Content-Type', 'text/event-stream')
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('Connection', 'keep-alive')
        handler.end_headers()
        last = -1
        with self._cond:
            self.clients += 1
        try:
            while True:
                with self._cond:
                    self._cond.wait_for(lambda: self._seq != last or self._stopping,
                                        timeout=1.0)
                    if self._stopping:
                        break
                    changed = self._seq != last
                    last, data = self._seq, self._latest
                msg = f'data: {data}\n\n' if changed else ': ping\n\n'
                handler.wfile.write(msg.encode())
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                                   # the browser went away
        finally:
            with self._cond:
                self.clients -= 1

    @staticmethod
    def parse_touch(body: bytes):
        """Only two well-formed messages are accepted; anything else -> None."""
        try:
            d = json.loads(body.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return None
        if not isinstance(d, dict):
            return None
        if d.get('kind') == 'tap':
            return {'kind': 'tap'}
        v = d.get('value')
        if d.get('kind') == 'choice' and isinstance(v, str) and re.fullmatch(r'[a-z_]{1,24}', v):
            return {'kind': 'choice', 'value': v}
        return None

    # ------------------------------------------------------------------
    @property
    def port(self):
        return self._httpd.server_address[1]

    def start(self):
        self._thread.start()

    def publish(self, state: dict):
        data = json.dumps(state, separators=(',', ':'))
        with self._cond:
            self._latest = data
            self._seq += 1
            self._cond.notify_all()

    def stop(self):
        with self._cond:
            self._stopping = True
            self._cond.notify_all()
        self._httpd.shutdown()
        self._httpd.server_close()
