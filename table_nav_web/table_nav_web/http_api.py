"""
HTTP side of the table navigation web app.

Nothing in here imports ROS, so the web page and API can be tested on any PC
with a fake backend. The ROS node (web_server.py) is the real backend and must
provide:

    get_status() -> dict          robot pose, navigation state, Nav2 readiness
    get_pose()   -> dict | None   {'x', 'y', 'yaw'} in the map frame
    get_map()    -> dict | None   occupancy grid prepared for the browser
    go_to(name, x, y, yaw)        -> (ok, message)
    cancel()                      -> (ok, message)
    set_initial_pose(x, y, yaw)   -> (ok, message)
"""
import json
import math
import os
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9 _\-]{0,31}$')
MAX_BODY_BYTES = 64 * 1024


class LocationStore:
    """Named poses (x, y in metres, yaw in radians, map frame) kept in a YAML file."""

    def __init__(self, path, seed_path=None):
        self.path = os.path.expanduser(path)
        self._lock = threading.Lock()
        self._locations = {}
        if os.path.exists(self.path):
            self._locations = self._read(self.path)
        elif seed_path and os.path.exists(seed_path):
            self._locations = self._read(seed_path)
            self._write()

    @staticmethod
    def _read(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        locations = {}
        for name, pose in (data.get('locations') or {}).items():
            locations[str(name)] = {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'yaw': float(pose.get('yaw', 0.0)),
            }
        return locations

    def _write(self):
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump({'locations': self._locations}, f, sort_keys=False)
        os.replace(tmp, self.path)

    def all(self):
        with self._lock:
            return {name: dict(pose) for name, pose in self._locations.items()}

    def get(self, name):
        with self._lock:
            pose = self._locations.get(name)
            return dict(pose) if pose else None

    def put(self, name, x, y, yaw):
        with self._lock:
            self._locations[name] = {
                'x': round(x, 3), 'y': round(y, 3), 'yaw': round(_wrap_angle(yaw), 3)}
            self._write()

    def delete(self, name):
        with self._lock:
            if name not in self._locations:
                return False
            del self._locations[name]
            self._write()
            return True


def _wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def _number(body, key, default=None):
    value = body.get(key, default)
    if value is None:
        raise ValueError(f"'{key}' is required")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"'{key}' must be a finite number")
    return value


def make_server(backend, store, web_dir, host='0.0.0.0', port=8080):
    """Create (but do not start) the HTTP server. Call serve_forever() on it."""

    def resolve_target(body):
        """A request names a saved place ({'name'}) or a map point ({'x','y','yaw'})."""
        if body.get('name'):
            name = str(body['name'])
            pose = store.get(name)
            if pose is None:
                return None, 404, f"No saved place called '{name}'"
            return (name, pose['x'], pose['y'], pose['yaw']), 200, ''
        x, y = _number(body, 'x'), _number(body, 'y')
        yaw = _number(body, 'yaw', 0.0)
        return (str(body.get('label') or 'map point'), x, y, yaw), 200, ''

    def goto(body):
        target, code, message = resolve_target(body)
        if target is None:
            return code, {'ok': False, 'message': message}
        ok, message = backend.go_to(*target)
        return (200 if ok else 503), {'ok': ok, 'message': message}

    def cancel(_body):
        ok, message = backend.cancel()
        return (200 if ok else 503), {'ok': ok, 'message': message}

    def initial_pose(body):
        target, code, message = resolve_target(body)
        if target is None:
            return code, {'ok': False, 'message': message}
        ok, message = backend.set_initial_pose(*target[1:])
        return (200 if ok else 503), {'ok': ok, 'message': message}

    def save_location(body):
        name = str(body.get('name', '')).strip()
        if not NAME_RE.match(name):
            return 400, {'ok': False, 'message':
                         'Name must be 1-32 letters, digits, spaces, - or _ (e.g. "table1")'}
        if 'x' in body and 'y' in body:
            x, y, yaw = _number(body, 'x'), _number(body, 'y'), _number(body, 'yaw', 0.0)
        else:
            pose = backend.get_pose()
            if pose is None:
                return 409, {'ok': False, 'message':
                             'Robot position is unknown. Start Nav2 and set the initial pose first.'}
            x, y, yaw = pose['x'], pose['y'], pose['yaw']
        store.put(name, x, y, yaw)
        return 200, {'ok': True, 'message': f"Saved '{name}'", 'locations': store.all()}

    def delete_location(body):
        name = str(body.get('name', ''))
        if not store.delete(name):
            return 404, {'ok': False, 'message': f"No saved place called '{name}'"}
        return 200, {'ok': True, 'message': f"Deleted '{name}'", 'locations': store.all()}

    post_routes = {
        '/api/goto': goto,
        '/api/cancel': cancel,
        '/api/initial_pose': initial_pose,
        '/api/locations': save_location,
        '/api/locations/delete': delete_location,
    }

    class Handler(BaseHTTPRequestHandler):
        server_version = 'TableNav/1.0'

        def log_message(self, fmt, *args):
            pass  # status polling would flood the ROS console

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path in ('/', '/index.html'):
                self._send_file(os.path.join(web_dir, 'index.html'), 'text/html; charset=utf-8')
            elif path == '/api/status':
                self._send_json(200, backend.get_status())
            elif path == '/api/locations':
                self._send_json(200, {'locations': store.all()})
            elif path == '/api/map':
                grid = backend.get_map()
                if grid is None:
                    self._send_json(404, {'ok': False, 'message': 'No map received on /map yet'})
                else:
                    self._send_json(200, grid)
            else:
                self._send_json(404, {'ok': False, 'message': 'Not found'})

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            route = post_routes.get(path)
            if route is None:
                self._send_json(404, {'ok': False, 'message': 'Not found'})
                return
            # Requiring a JSON content type makes browsers send a CORS preflight for
            # cross-site requests, which this server never approves. That stops other
            # web pages open on the same Wi-Fi from silently driving the robot.
            if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
                self._send_json(415, {'ok': False, 'message': 'Content-Type must be application/json'})
                return
            try:
                length = int(self.headers.get('Content-Length') or 0)
                if length > MAX_BODY_BYTES:
                    raise ValueError('Request body too large')
                body = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(body, dict):
                    raise ValueError('Request body must be a JSON object')
                code, payload = route(body)
            except (ValueError, TypeError) as e:
                code, payload = 400, {'ok': False, 'message': str(e)}
            self._send_json(code, payload)

        def _send_json(self, code, payload):
            data = json.dumps(payload).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _send_file(self, path, content_type):
            try:
                with open(path, 'rb') as f:
                    data = f.read()
            except OSError:
                self._send_json(500, {'ok': False, 'message': f'Missing web page: {path}'})
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
