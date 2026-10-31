"""Drive the real arm over serial (Arduino running ArmHome.ino), by joint angles or by tip position.

  python sim\\arm_control.py --port COM8                       interactive prompt
  python sim\\arm_control.py --port COM8 angles 90 60 120 45   one move by servo angles (FK reports the tip), then exit
  python sim\\arm_control.py --port COM8 xyz 260 80 250 0      one move by tip position + tool pitch (IK), then exit

Opening the port resets most Arduinos, so the arm first runs its power-up home move; the script waits for it and
then reads the arm's own lastSent[] back. Every move is sent as one line, "m a0 a1 a2 a3 a4 a5", and the firmware
runs the synchronised smooth motion. After the move the pose is read back from the Arduino.
Angles and IK targets are checked against the joint limits here first, so nothing out of range is sent.
"""
import os, re, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K
from kinematics import fk, ik, ik_branches, violations, LO, HI, HOME, MAX_DPS, H, L1, L2, L3
NAMES = ['waist', 'shoulder', 'elbow', 'wrist', 'roll', 'grip']


class ArmLink:
    """Serial link to the Arduino running ArmHome.ino (115200 baud, newline-terminated)."""

    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.serial_for_url(port, baudrate=baud, timeout=0.2)

    def _lines(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            raw = self.ser.readline()
            if raw:
                yield raw.decode(errors='replace').strip()

    def wait_for(self, prefix, timeout, echo=True):
        for line in self._lines(timeout):
            if echo and line:
                print('  [arduino]', line)
            if line.startswith(prefix):
                return line
        return None

    @staticmethod
    def parse_pose(line):
        return np.array([float(v) for v in re.findall(r'-?\d+\.?\d*', line.split(':', 1)[1])])

    def connect(self):
        """Opening the port usually resets the board; setup() then runs the whole home move."""
        print('  waiting for the Arduino to boot and reach home (up to 25 s)...')
        self.wait_for('Home reached', 25)
        return self.query_pose()

    def query_pose(self):
        self.ser.reset_input_buffer()
        self.ser.write(b'p\n')
        line = self.wait_for('lastSent:', 5, echo=False)
        return self.parse_pose(line) if line else None

    def send_move(self, goal):
        self.ser.reset_input_buffer()
        self.ser.write(('m ' + ' '.join(f'{g:.2f}' for g in goal) + '\n').encode())

    def finish_move(self, duration):
        """The firmware answers with lastSent: once the (blocking) move has finished."""
        line = self.wait_for('lastSent:', duration + 8, echo=False)
        return self.parse_pose(line) if line else None

    def close(self):
        self.ser.close()


LINK = None
POSE = None          # the arm's pose as last reported by the Arduino


def fmt(v): return '[' + ', '.join(f'{x:.1f}' for x in v) + ']'


def duration(start, goal):
    """Seconds the firmware's moveTo() will take (slowest joint at its slew limit, min-jerk peak factor 1.875)."""
    return float(np.max(np.abs(goal - start) * 1.875 / MAX_DPS))


def need_link():
    if LINK is None or POSE is None:
        print('  not connected. Use --port COMx, or type: connect COMx')
        return False
    return True


def move_to(goal):
    global POSE
    T = duration(POSE, goal)
    print(f'  from {fmt(POSE)}\n  to   {fmt(goal)}\n  takes about {T:.1f} s')
    LINK.send_move(goal)
    got = LINK.finish_move(T)
    if got is None:
        print('  WARNING: no reply from the Arduino. Pose not updated; type "sync" once the arm is idle.')
        return
    off = np.max(np.abs(got - goal))
    print(f'  arduino reports {fmt(got)}' + ('' if off < 0.2 else f'  (differs from the goal by {off:.1f} deg)'))
    POSE = got


def do_angles(vals):
    """Servo angles in, FK reports the tip, then the arm moves."""
    if not need_link(): return
    g = POSE.copy(); g[:len(vals)] = vals                       # missing values keep their current angle
    bad = violations(g)
    if bad:
        print('  refused, outside the joint limits: ' +
              '; '.join(f'{n} {v:g} (allowed {lo:.0f} to {hi:.0f})' for n, v, lo, hi in bad)); return
    f = fk(g)
    print(f'  FK: tip = {fmt(f["tip"])} mm, tool pitch = {f["phi_deg"]:.1f} deg')
    move_to(g)


def why_not(x, y, z, phi, roll, grip):
    """Explain an IK failure in plain words."""
    sols = ik_branches(x, y, z, phi, roll, grip)
    if not sols:
        r = np.hypot(x, y); rw = r - L3 * np.cos(np.radians(phi)) if x >= 0 else -r - L3 * np.cos(np.radians(phi))
        zw = z - L3 * np.sin(np.radians(phi)) - H
        return (f'out of reach: the wrist point would be {np.hypot(rw, zw):.0f} mm from the shoulder, '
                f'the arm spans {abs(L1 - L2):.0f} to {L1 + L2:.0f} mm')
    out = []
    for name, s in zip(('elbow bent forward', 'elbow bent back'), sols):
        out.append(name + ': ' + ', '.join(f'{n} would need {v:.0f} (allowed {lo:.0f} to {hi:.0f})'
                                           for n, v, lo, hi in violations(s)))
    return 'joint limit. ' + ' | '.join(out)


def do_xyz(vals):
    """Tip position (+ tool pitch) in, IK solves, then the arm moves."""
    if not need_link(): return
    if not K.SIGNS_VERIFIED:
        print('  note: joint directions (SIGN in kinematics.py) are not checked on this arm; if a joint moves the',
              'wrong way, flip its SIGN. Type "signs" to compare.')
    x, y, z = vals[:3]
    phi = vals[3] if len(vals) > 3 else fk(POSE)['phi_deg']     # no pitch given: keep the current one
    q = ik(x, y, z, phi, POSE[4], POSE[5])
    if q is None:
        print('  IK failed,', why_not(x, y, z, phi, POSE[4], POSE[5])); return
    print(f'  IK: servo angles = {fmt(q)}  (tool pitch {phi:.1f} deg)')
    print(f'  check: FK of that = {fmt(fk(q)["tip"])} mm')
    move_to(q)


def predicted_motion():
    """What a +10 deg command on each of s0..s3 should do to the real arm at HOME, from the current SIGN."""
    base = K.fk(HOME)
    rows = []
    for i, name in enumerate(['waist', 'shoulder', 'elbow', 'wrist']):
        s = HOME.copy(); s[i] += 10
        n = K.fk(s)
        if i == 0: txt = ('the whole arm turns to the LEFT (counter-clockwise seen from above)' if n['tip'][1] > 0
                          else 'the whole arm turns to the RIGHT (clockwise seen from above)')
        elif i == 1: txt = 'the upper arm leans ' + ('FORWARD, toward the side the gripper points' if n['elbow'][0] > 0 else 'BACKWARD, away from the side the gripper points')
        elif i == 2: txt = 'the forearm leans ' + ('FORWARD, toward the side the gripper points' if n['wrist'][0] > base['wrist'][0] else 'BACKWARD, away from the side the gripper points')
        else: txt = 'the gripper tilts ' + ('UP' if n['tip'][2] > base['tip'][2] else 'DOWN')
        rows.append(f'  channel {i} ({name:8s}) +10 deg: {txt}')
    return rows


def connect(port):
    """Open the board and read back the pose it reports."""
    global LINK, POSE
    try:
        LINK = ArmLink(port)
    except Exception as e:
        LINK = None
        print(f'  could not open {port}: {e}'); return
    pose = LINK.connect()
    if pose is None or len(pose) != 6:
        print('  connected, but the Arduino did not report its pose. Is ArmHome.ino running at 115200 baud?')
        LINK.close(); LINK = None; return
    POSE = pose
    print(f'  connected on {port}. Arm at {fmt(pose)}.')


HELP = """commands:
  angles s0 s1 s2 s3 [s4 s5]   move to servo angles (degrees, 0-180); FK reports the tip
  xyz x y z [phi]              move the tip to (x, y, z) mm; phi = tool pitch in deg, negative = down
  home                         go to HOME
  show                         read the pose from the Arduino and print it with the tip position
  sync                         re-read the pose from the Arduino (after a missing reply)
  signs                        what +10 deg on each joint should do on the real arm (direction check)
  connect COMx, disconnect, ports
  help, quit
x is forward, y is left, z is up, from the ground at the waist axis."""


def main():
    global LINK, POSE
    args = sys.argv[1:]
    port = None
    if '--trust-signs' in args:
        args.remove('--trust-signs'); K.SIGNS_VERIFIED = True
        print('  WARNING: --trust-signs, xyz moves are on although SIGN has not been checked. Keep a hand on the supply.')
    if '--port' in args:
        i = args.index('--port'); port = args[i + 1]; del args[i:i + 2]
    if port:
        connect(port)
        if LINK is None: return
    if args:
        if not port:
            print('  one-shot moves need --port COMx'); return
        mode, vals = args[0], [float(a) for a in args[1:]]
        {'angles': do_angles, 'xyz': do_xyz}.get(mode, lambda *_: print(__doc__))(vals)
        LINK.close()
        return
    print(HELP)
    if not LINK: print('  (not connected: type "connect COMx")')
    while True:
        try:
            line = input('arm> ').split()
        except (EOFError, KeyboardInterrupt):
            break
        if not line: continue
        cmd, rest = line[0].lower(), line[1:]
        try:
            if cmd in ('quit', 'q', 'exit'): break
            elif cmd == 'help': print(HELP)
            elif cmd == 'angles': do_angles([float(v) for v in rest])
            elif cmd == 'xyz': do_xyz([float(v) for v in rest])
            elif cmd == 'home':
                if need_link(): move_to(HOME)
            elif cmd == 'signs':
                print('  Model prediction for +10 deg on each joint, starting from HOME.\n'
                      '  Send each on the real arm (home, then e.g. "angles 90 90 85 45 165 90") and compare:')
                print('\n'.join(predicted_motion()))
                print('  A joint that moves the other way: flip its entry in SIGN in kinematics.py (+1 <-> -1).\n'
                      '  When all four match, set SIGNS_VERIFIED = True there.')
            elif cmd == 'trust':
                K.SIGNS_VERIFIED = True
                print('  xyz moves enabled for this session. If a joint moves the wrong way, flip its SIGN in kinematics.py.')
            elif cmd == 'ports':
                import serial.tools.list_ports as lp
                for p in lp.comports(): print(f'  {p.device}  {p.description}')
            elif cmd == 'connect' and rest: connect(rest[0])
            elif cmd == 'disconnect':
                if LINK: LINK.close(); LINK = None; POSE = None
                print('  disconnected')
            elif cmd in ('sync', 'show'):
                pose = LINK.query_pose() if LINK else None
                if pose is None: print('  not connected, or no reply')
                else:
                    POSE = pose; f = fk(pose)
                    print(f'  pose = {fmt(pose)}\n  tip  = {fmt(f["tip"])} mm, tool pitch {f["phi_deg"]:.1f} deg')
            else: print('  unknown command, type help')
        except ValueError:
            print('  numbers only, e.g.  xyz 260 80 250 0')
    if LINK: LINK.close()


if __name__ == '__main__':
    main()
