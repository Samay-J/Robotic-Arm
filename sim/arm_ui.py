"""Teaching UI for the arm: joint sliders, FK readout + drawing, IK form, and a box where students write short Python.

  python sim\\arm_ui.py                  starts the page at http://127.0.0.1:8000 (connect to the board inside the page)
  python sim\\arm_ui.py --port COM8      connects to the board first
  python sim\\arm_ui.py --http 8080      other web port

Needs numpy and pyserial. Works without a board too: FK and IK still compute, moves just say "not connected".
The page is served to this computer only (127.0.0.1). Student code runs in this process with a short list of
allowed functions; that stops accidents (no imports, no files), it is not a security sandbox.
"""
import argparse, builtins, json, math, os, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kinematics as K
import arm_control as ac

LOCK = threading.RLock()        # one hardware action at a time
STOP = threading.Event()        # set by the Stop button; checked by moves, waits and student code lines


class ArmError(Exception):
    pass


# ---------------------------------------------------------------- hardware
def current_pose():
    return ac.POSE.copy() if ac.POSE is not None else K.HOME.copy()


def connect(port):
    if ac.LINK is not None:
        ac.LINK.close(); ac.LINK = None; ac.POSE = None
    try:
        link = ac.ArmLink(port)
    except Exception as e:
        raise ArmError(f'could not open {port}: {e}')
    pose = link.connect()
    if pose is None or len(pose) != 6:
        link.close(); raise ArmError('connected, but the Arduino did not report its pose (is ArmHome.ino running at 115200 baud?)')
    ac.LINK, ac.POSE = link, pose


def disconnect():
    if ac.LINK is not None:
        ac.LINK.close()
    ac.LINK = None; ac.POSE = None


def do_move(goal):
    if ac.LINK is None or ac.POSE is None:
        raise ArmError('not connected')
    goal = np.array(goal, float)
    bad = K.violations(goal)
    if bad:
        raise ArmError('outside the joint limits: ' + '; '.join(f'{n} {v:g} (allowed {lo:.0f} to {hi:.0f})' for n, v, lo, hi in bad))
    if STOP.is_set():
        raise ArmError('stopped')
    T = ac.duration(ac.POSE, goal)
    ac.LINK.send_move(goal)
    got = ac.LINK.finish_move(T)
    if got is None:
        raise ArmError('no reply from the Arduino, pose not updated (wait until the arm stops, then press Read pose)')
    ac.POSE = got
    return got


def merged(vals):
    """First len(vals) joints from vals, the rest from the current pose."""
    g = current_pose(); g[:len(vals)] = vals
    return g


def solve_ik(x, y, z, phi):
    cur = current_pose()
    q = K.ik(x, y, z, phi, cur[4], cur[5])
    if q is None:
        return None, ac.why_not(x, y, z, phi, cur[4], cur[5])
    return q, ''


def fk_info(s):
    s = np.array(s, float)
    f = K.fk(s); a1, a2, a3, yaw = K.abs_angles(s)
    c, sn = math.cos(yaw), math.sin(yaw)
    def rz(p): return [round(float(p[0] * c + p[1] * sn), 2), round(float(p[2]), 2)]   # (distance out along the arm's heading, height)
    return dict(
        tip=[round(float(v), 1) for v in f['tip']], phi=round(f['phi_deg'], 1),
        yaw=round(math.degrees(yaw), 1), a1=round(math.degrees(a1), 1), a2=round(math.degrees(a2), 1), a3=round(math.degrees(a3), 1),
        r=rz(f['tip'])[0], z=round(float(f['tip'][2]), 1),
        joints={k: [round(float(v), 1) for v in f[k]] for k in ('shoulder', 'elbow', 'wrist', 'tip')},
        profile=[[0, 0], [0, K.H], rz(f['elbow']), rz(f['wrist']), rz(f['tip'])],
        top=[round(float(f['tip'][0]), 1), round(float(f['tip'][1]), 1)])


# ---------------------------------------------------------------- student code
SAFE_BUILTINS = {n: getattr(builtins, n) for n in (
    'range', 'len', 'abs', 'round', 'min', 'max', 'sum', 'int', 'float', 'str', 'bool', 'list', 'tuple', 'dict',
    'enumerate', 'zip', 'sorted', 'reversed', 'any', 'all', 'isinstance')}


def run_student(code):
    out = []
    steps = [0]

    def _print(*a, sep=' ', end='\n'):
        out.append(sep.join(str(x) for x in a) + end)

    def flat(a):
        if len(a) == 1 and isinstance(a[0], (list, tuple, np.ndarray)):
            a = a[0]
        return [float(v) for v in a]

    def move(*a):
        v = flat(a)
        if not 4 <= len(v) <= 6:
            raise ArmError('move() needs 4 to 6 angles: move(waist, shoulder, elbow, wrist [, roll, grip])')
        return [round(float(x), 1) for x in do_move(merged(v))]

    def move_xyz(x, y, z, phi=None):
        phi = K.fk(current_pose())['phi_deg'] if phi is None else phi
        q, why = solve_ik(x, y, z, phi)
        if q is None:
            raise ArmError('IK failed: ' + why)
        return move(q)

    def fk(*a):
        v = flat(a)
        if len(v) < 4:
            raise ArmError('fk() needs 4 angles: fk(waist, shoulder, elbow, wrist)')
        f = K.fk(merged(v))
        return (round(float(f['tip'][0]), 3), round(float(f['tip'][1]), 3), round(float(f['tip'][2]), 3), round(f['phi_deg'], 3))

    def ik(x, y, z, phi=0):
        q, _ = solve_ik(x, y, z, phi)
        return None if q is None else [round(float(v), 1) for v in q]

    def home():
        return move(K.HOME)

    def pose():
        return [round(float(v), 1) for v in current_pose()]

    def wait(sec):
        end = time.time() + min(float(sec), 30)
        while time.time() < end:
            if STOP.is_set(): raise ArmError('stopped')
            time.sleep(0.05)

    def wrap_deg(a):
        return (a + 180.0) % 360.0 - 180.0

    def to_link_angles(*a):
        v = flat(a)
        if len(v) < 4:
            raise ArmError('to_link_angles() needs 4 servo angles')
        g, s = K.GAIN, K.SIGN
        return (round(float(s[0] * g[0] * (v[0] - K.WAIST_ZERO)), 3), round(float(90 + s[1] * g[1] * (v[1] - K.SHOULDER_ZERO)), 3),
                round(float(s[2] * g[2] * (v[2] - K.ELBOW_ZERO)), 3), round(float(s[3] * g[3] * (v[3] - K.WRIST_ZERO)), 3))

    def to_servo(yaw, a1, bend2, bend3):
        g, s = K.GAIN, K.SIGN
        return [round(float(K.WAIST_ZERO + yaw * s[0] / g[0]), 2), round(float(K.SHOULDER_ZERO + (wrap_deg(a1) - 90) * s[1] / g[1]), 2),
                round(float(K.ELBOW_ZERO + wrap_deg(bend2) * s[2] / g[2]), 2), round(float(K.WRIST_ZERO + wrap_deg(bend3) * s[3] / g[3]), 2)]

    env = dict(__builtins__=dict(SAFE_BUILTINS, print=_print), math=math, pi=math.pi, sin=math.sin, cos=math.cos, sqrt=math.sqrt,
               atan2=math.atan2, acos=math.acos, to_link_angles=to_link_angles, to_servo=to_servo, radians=math.radians, degrees=math.degrees,
               move=move, move_xyz=move_xyz, fk=fk, ik=ik, home=home, pose=pose, wait=wait)

    def tracer(frame, event, arg):
        if frame.f_code.co_filename != '<student>':
            return None
        def local(frame, event, arg):
            if event == 'line':
                steps[0] += 1
                if STOP.is_set(): raise ArmError('stopped')
                if steps[0] > 300000: raise ArmError('too many steps (an endless loop?)')
            return local
        return local

    STOP.clear()
    try:
        co = compile(code, '<student>', 'exec')
        sys.settrace(tracer)
        try:
            exec(co, env)
        finally:
            sys.settrace(None)
    except SyntaxError as e:
        out.append(f'Syntax error on line {e.lineno}: {e.msg}\n')
    except Exception as e:
        tb, ln = e.__traceback__, None
        while tb:
            if tb.tb_frame.f_code.co_filename == '<student>': ln = tb.tb_lineno
            tb = tb.tb_next
        out.append(f'Error{f" on line {ln}" if ln else ""}: {e}\n' if isinstance(e, ArmError)
                   else f'Error{f" on line {ln}" if ln else ""}: {type(e).__name__}: {e}\n')
    return ''.join(out)


# ---------------------------------------------------------------- web
def state():
    ports = []
    try:
        import serial.tools.list_ports as lp
        ports = [f'{p.device}  {p.description}' for p in lp.comports()]
    except Exception:
        pass
    pose = ac.POSE
    return dict(connected=ac.LINK is not None, pose=None if pose is None else [round(float(v), 1) for v in pose],
                home=K.HOME.tolist(), lo=K.LO.tolist(), hi=K.HI.tolist(), ports=ports,
                names=ac.NAMES, signs_verified=bool(K.SIGNS_VERIFIED))


def api(path, b):
    if path == '/api/state':
        return state()
    if path == '/api/stop':
        STOP.set(); return dict(ok=True)
    with LOCK:
        if path == '/api/connect':
            connect(b['port']); return dict(ok=True, **state())
        if path == '/api/disconnect':
            disconnect(); return dict(ok=True, **state())
        if path == '/api/fk':
            return dict(ok=True, **fk_info(b['angles']))
        if path == '/api/move':
            STOP.clear()
            got = do_move(merged([float(v) for v in b['angles']]))
            return dict(ok=True, pose=[round(float(v), 1) for v in got], **fk_info(got))
        if path == '/api/home':
            STOP.clear()
            got = do_move(K.HOME)
            return dict(ok=True, pose=[round(float(v), 1) for v in got], **fk_info(got))
        if path == '/api/read':
            if ac.LINK is None: raise ArmError('not connected')
            p = ac.LINK.query_pose()
            if p is None: raise ArmError('no reply from the Arduino')
            ac.POSE = p
            return dict(ok=True, pose=[round(float(v), 1) for v in p], **fk_info(p))
        if path == '/api/ik':
            q, why = solve_ik(float(b['x']), float(b['y']), float(b['z']), float(b['phi']))
            if q is None:
                return dict(ok=False, error='IK failed: ' + why)
            res = dict(ok=True, angles=[round(float(v), 1) for v in q], **fk_info(q))
            if b.get('move'):
                STOP.clear()
                res['pose'] = [round(float(v), 1) for v in do_move(q)]
            return res
        if path == '/api/run':
            return dict(ok=True, output=run_student(b.get('code', '')))
    raise ArmError('unknown request')


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _send(self, code, body, ctype='application/json'):
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store'); self.end_headers(); self.wfile.write(data)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            with open(os.path.join(HERE, 'ui', 'index.html'), 'rb') as f:
                self._send(200, f.read(), 'text/html; charset=utf-8')
        elif self.path == '/api/state':
            self._send(200, json.dumps(api('/api/state', {})))
        else:
            self._send(404, '{}')

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length') or 0)
            b = json.loads(self.rfile.read(n) or b'{}')
            self._send(200, json.dumps(api(self.path, b)))
        except ArmError as e:
            self._send(200, json.dumps(dict(ok=False, error=str(e))))
        except Exception as e:
            self._send(200, json.dumps(dict(ok=False, error=f'{type(e).__name__}: {e}')))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', help='serial port of the Arduino, e.g. COM8')
    ap.add_argument('--http', type=int, default=8000)
    ap.add_argument('--no-browser', action='store_true')
    a = ap.parse_args()
    if a.port:
        try: connect(a.port); print(f'connected on {a.port}')
        except ArmError as e: print(e)
    srv = ThreadingHTTPServer(('127.0.0.1', a.http), Handler)
    url = f'http://127.0.0.1:{a.http}'
    print(f'Arm UI at {url}   (Ctrl+C to stop)')
    if not a.no_browser: webbrowser.open(url)
    try: srv.serve_forever()
    except KeyboardInterrupt: pass
    disconnect()


if __name__ == '__main__':
    main()
