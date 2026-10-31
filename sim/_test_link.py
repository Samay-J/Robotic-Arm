"""Protocol test for arm_control.py with a fake Arduino (no hardware). Run from the project root:
    python sim\\_test_link.py
"""
import sys, collections, time
sys.path.insert(0, 'sim')
import numpy as np
import serial
import arm_control as ac

LO = ac.LO; HI = ac.HI


class FakeArduino:
    """Mimics ArmHome.ino: boots, goes home, answers p and m (blocking move, then lastSent:)."""
    silent = False

    def __init__(self, *a, **k):
        self.pose = np.array([30, 20, 100, 90, 20, 100.])    # what EEPROM held
        self.rx = collections.deque()
        self.log = []
        self._line('lastSent: ' + ', '.join(f'{v:.1f}' for v in self.pose))
        self.pose = ac.HOME.copy()
        self._line('Home reached. Commands: h | p | <ch> <deg> | set <ch> <deg>')

    def _line(self, s): self.rx.append((s + '\n').encode())
    def _state(self): self._line('lastSent: ' + ', '.join(f'{v:.1f}' for v in self.pose))
    def reset_input_buffer(self): self.rx.clear()
    def close(self): pass

    def write(self, data):
        cmd = data.decode().strip(); self.log.append(cmd)
        if self.silent: return
        if cmd == 'p': self._state()
        elif cmd.startswith('m '):
            v = [float(x) for x in cmd[2:].split()]
            if len(v) != 6: self._line('err: m needs 6 angles'); return
            self.pose = np.clip(v, LO, HI); self._state()

    def readline(self):
        if self.rx: return self.rx.popleft()
        time.sleep(0.01); return b''


made = []
serial.serial_for_url = lambda *a, **k: made.append(FakeArduino()) or made[-1]
ok = True
def check(name, cond):
    global ok; ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name)

n = 0
ac.do_angles([63, 63, 49, 20]); check('commands refused when not connected', True)   # prints, must not crash

ac.connect('FAKE')
fake = made[0]
check('connect reads the arm pose', np.allclose(ac.POSE, ac.HOME))

ac.do_angles([100, 50, 100, 70])
check('angles -> one "m" with all six joints', fake.log[-1] == 'm 100.00 50.00 100.00 70.00 90.00 90.00')
check('pose taken from the hardware reply', np.allclose(ac.POSE, [100, 50, 100, 70, 90, 90], atol=0.1))

ac.do_xyz([260, 80, 250, 0])
sent = np.array([float(x) for x in fake.log[-1].split()[1:]])
check('xyz -> IK angles sent', np.allclose(ac.fk(sent)['tip'], [260, 80, 250], atol=0.05))

n = len(fake.log); ac.do_xyz([600, 0, 100, 0])
check('unreachable target sends nothing', len(fake.log) == n)
ac.do_angles([63, 190, 49, 20])
check('out-of-limit angle sends nothing', len(fake.log) == n)

before = ac.POSE.copy(); fake.silent = True
orig = ac.ArmLink.finish_move
ac.ArmLink.finish_move = lambda self, d: self.wait_for('lastSent:', 0.3, echo=False) and None
ac.do_angles([80, 60, 90, 50])
check('no reply -> pose not updated', np.allclose(ac.POSE, before))
ac.ArmLink.finish_move = orig

check('module has no simulation/plotting', 'matplotlib' not in sys.modules)
ac.LINK.close()
print('ALL PASS' if ok else 'FAILURES'); sys.exit(0 if ok else 1)
