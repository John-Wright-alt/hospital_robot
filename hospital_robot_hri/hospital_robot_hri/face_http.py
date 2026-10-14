"""
face_http.py  --  tiny web server for the face page. Python standard library
only (no FastAPI, no internet needed), and NO ROS code, so it can be tested on
its own.

    GET /            the face page (web/index.html)
    GET /state       the latest face state as JSON (handy with curl)
    GET /events      Server-Sent Events: the browser receives every new state

The browser only RECEIVES data. Nothing a browser does on this server can
command the robot.
"""

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class FaceHttpServer:

    def __init__(self, web_dir: str, host: str = '0.0.0.0', port: int = 8080):
        index = os.path.join(web_dir, 'index.html')
        with open(index, 'rb') as f:              # fail early and clearly if missing
            self._index = f.read()
        self._web_dir = web_dir
        self._latest = json.dumps({'expression': 'sleeping', 'gaze_data': 'starting'})
        self._seq = 0
        self._cond = threading.Condition()
        self._stopping = False
        self.clients = 0

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
