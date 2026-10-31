# Robotics Arm: every command, what it does, with examples

Joints (PCA9685 channel = joint number): **0 waist, 1 shoulder, 2 elbow, 3 wrist pitch, 4 wrist roll, 5 gripper.**
Angles are servo degrees. Limits: about 0 to 145 (shoulder to 131, gripper 7 to 110.7).
HOME = `63.0, 63.0, 49.3, 19.9, 123.7, 58.9` (upper arm and forearm vertical, gripper level, pointing forward).
Positions are in mm: **x forward, y left, z up**, from the ground at the waist axis. At HOME the tip is at about (84, 0, 380).

There are three places you type commands:

1. [Starting the PC program](#1-starting-the-pc-program) (PowerShell)
2. [Commands inside the `arm>` prompt](#2-commands-inside-the-arm-prompt) (PC, python)
3. [Arduino Serial Monitor commands](#3-arduino-serial-monitor-commands) (115200 baud, line ending Newline)

Only one program can use the COM port at a time: close the Serial Monitor before running the PC program, and the other way round.

---

## 1. Starting the PC program

Run from the project root. Replace `COM8` with your port.

| Command | What it does |
|---|---|
| `python sim\arm_control.py --port COM8` | Connects, waits for the arm to finish its power-up home move, then opens the `arm>` prompt |
| `python sim\arm_control.py --port COM8 angles 90 60 120 45` | One move by servo angles, then exits |
| `python sim\arm_control.py --port COM8 xyz 260 80 250 0` | One move by tip position, then exits (needs `SIGNS_VERIFIED = True`) |
| `python sim\arm_control.py` | Opens the prompt without a connection; use `ports` and `connect COMx` inside |
| `python sim\kinematics.py` | Self-test of FK and IK. No hardware needed |
| `python sim\_test_link.py` | Tests the serial protocol against a fake Arduino. No hardware needed |

Examples:

```powershell
cd "E:\Claude\Robotics Arm"
python sim\arm_control.py --port COM8
python sim\arm_control.py --port COM8 angles 63 63 49.3 19.9      # same as HOME
python sim\arm_control.py --port COM8 xyz 260 80 250 0
```

If `python` is not the 3.10 install:

```powershell
& "C:\Users\Samay Jain\AppData\Local\Programs\Python\Python310\python.exe" sim\arm_control.py --port COM8
```

---

## 2. Commands inside the `arm>` prompt

### `ports`
Lists the serial ports the PC can see. Use it to find the Arduino.
```
arm> ports
  COM8  Arduino Uno (COM8)
```

### `connect COMx`
Opens the port, waits for the arm to reach home (up to 25 s), then reads the arm's pose.
```
arm> connect COM8
  waiting for the Arduino to boot and reach home (up to 25 s)...
  connected on COM8. Arm at [63.0, 63.0, 49.3, 19.9, 123.7, 58.9].
```

### `disconnect`
Closes the port. Use it before opening the Arduino Serial Monitor.
```
arm> disconnect
  disconnected
```

### `show`
Reads the pose from the Arduino and prints it with the tip position (FK). No motion.
```
arm> show
  pose = [63.0, 63.0, 49.3, 19.9, 123.7, 58.9]
  tip  = [83.8, 0.1, 380.0] mm, tool pitch 0.0 deg
```

### `sync`
Same read as `show`. Use it after `WARNING: no reply from the Arduino`, once the arm has stopped moving, so the PC's pose matches the arm again.

### `home`
Moves to HOME.
```
arm> home
  from [90.0, 60.0, 120.0, 45.0, 123.7, 58.9]
  to   [63.0, 63.0, 49.3, 19.9, 123.7, 58.9]
  takes about 4.9 s
  arduino reports [63.0, 63.0, 49.3, 19.9, 123.7, 58.9]
```

### `angles s0 s1 s2 s3 [s4 s5]`
Moves the joints to the servo angles you give. FK first prints where the tip will be. Joints you leave out keep their current angle. All joints move together and arrive at the same time.
```
arm> angles 90 60 120 45
  FK: tip = [169.4, 96.6, 198.4] mm, tool pitch = -51.9 deg
  from [63.0, 63.0, 49.3, 19.9, 123.7, 58.9]
  to   [90.0, 60.0, 120.0, 45.0, 123.7, 58.9]
  takes about 4.9 s
  arduino reports [90.0, 60.0, 120.0, 45.0, 123.7, 58.9]
```
More examples:
```
arm> angles 100                      turn only the waist to 100
arm> angles 63 63 49.3 19.9 90 110   keep the arm at HOME, roll to 90, open the gripper to 110
arm> angles 63 140 49 20             refused: shoulder 140 is outside 0 to 131 (nothing is sent)
```

### `xyz x y z [phi]`
Moves the gripper tip to (x, y, z) mm. `phi` is the tool pitch in degrees: 0 = pointing forward, negative = pointing down. If you leave `phi` out, the current pitch is kept. IK picks the servo angles, FK double-checks them, then the arm moves. Wrist roll and gripper keep their current values.

**Refused until you have checked the joint directions** (see `signs`, and set `SIGNS_VERIFIED = True` in `sim\kinematics.py`).
```
arm> xyz 260 80 250 0
  IK: servo angles = [78.5, 43.1, 104.8, 91.6, 123.7, 58.9]  (tool pitch 0.0 deg)
  check: FK of that = [260.0, 80.0, 250.0] mm
  from [63.0, 63.0, 49.3, 19.9, 123.7, 58.9]
  to   [78.5, 43.1, 104.8, 91.6, 123.7, 58.9]
  takes about 3.8 s
  arduino reports [78.5, 43.1, 104.8, 91.6, 123.7, 58.9]
```
More examples:
```
arm> xyz 250 0 130 -60        reach forward and low with the tool pointing 60 degrees down
arm> xyz 200 100 200          keep the current tool pitch
arm> xyz 600 0 100 0          IK failed, out of reach (arm spans 3 to 277 mm from the shoulder); nothing is sent
```

### `signs`
Prints what +10 degrees on each joint should do to the real arm, according to the current `SIGN` setting. You compare it with what the arm really does.
```
arm> signs
  channel 0 (waist   ) +10 deg: the whole arm turns to the LEFT (counter-clockwise seen from above)
  channel 1 (shoulder) +10 deg: the upper arm leans FORWARD, ...
  ...
```
Check procedure:
1. `home`
2. `angles 73 63 49.3 19.9` (waist +10), look at the arm, then `home`
3. Repeat for the shoulder (`angles 63 73 ...`), elbow (`angles 63 63 59.3 ...`) and wrist (`angles 63 63 49.3 29.9`)
4. For a joint that moves the other way, flip its entry in `SIGN` in `sim\kinematics.py` (`[1, 1, -1, 1]`, +1 <-> -1) and in `arduino\ArmHome\kinematics.h`
5. When all four match, set `SIGNS_VERIFIED = True`

### `help`, `quit`
`help` reprints the command list. `quit` (also `q`, `exit`, or Ctrl+C) closes the port and leaves.

### Messages you may see
| Message | Meaning |
|---|---|
| `not connected. Use --port COMx, or type: connect COMx` | Connect first |
| `refused, outside the joint limits: ...` | An angle is beyond the allowed range; nothing was sent |
| `IK failed, out of reach ...` | The target is farther than the arm can stretch |
| `IK failed, joint limit. ...` | Reachable, but a joint would need an angle it cannot do. Try a different `phi` |
| `refused: the joint directions ... have not been checked` | Do the `signs` check, then set `SIGNS_VERIFIED = True` |
| `WARNING: no reply from the Arduino` | Wait until the arm stops, then `sync` |
| `numbers only, e.g. xyz 260 80 250 0` | A value was not a number |

---

## 3. Arduino Serial Monitor commands

Arduino IDE, Tools > Serial Monitor, **115200 baud**, line ending **Newline**. These are the same lines the PC program sends.

| Type | What it does | Example | Reply |
|---|---|---|---|
| `h` | Glides to HOME (never faster than 1.5 s) | `h` | `lastSent: 63.0, 63.0, 49.3, 19.9, 123.7, 58.9` |
| `p` | Prints the angles the Arduino believes each joint is at. No motion | `p` | `lastSent: ...` |
| `<ch> <deg>` | Moves one joint smoothly, others stay | `2 100` (elbow to 100) | `lastSent: ...` |
| `set <ch> <deg>` | **No motion.** Tells the Arduino "this joint is physically at `<deg>`" and saves it to EEPROM | `set 2 100` | `lastSent: ...` |
| `m a0 a1 a2 a3 a4 a5` | Moves all six joints together, arriving at the same time | `m 63 50 70 20 123.7 58.9` | `lastSent: ...` |

Notes:
- Every target is clamped to the limits `ANG_MIN[]` / `ANG_MAX[]` at the top of the sketch.
- The Arduino has no encoders, so "position" means the last value it sent (`lastSent`). It is saved to EEPROM, so after a power cycle it starts from the last pose, then glides home.
- `set` is for when you moved the arm by hand or the believed pose is wrong: park the arm at a known pose, then `set` each joint.
- `m` needs exactly six numbers, otherwise: `err: m needs 6 angles`.

Examples:
```
h                        go home
p                        where does the Arduino think the arm is?
0 90                     turn the waist to 90
5 100                    open the gripper to 100
m 80 60 90 40 123.7 58.9 move everything at once
set 1 63                 I put the shoulder at 63 by hand, update the record
```

---

## 4. FK and IK in your own Python code

```python
import sys; sys.path.insert(0, "sim")
from kinematics import fk, ik, HOME

f = fk(HOME)                 # dict: tip, wrist, elbow, shoulder, phi_deg
print(f["tip"], f["phi_deg"])    # [83.8 0.1 380.0], 0.0

q = ik(260, 80, 250, 0)      # x, y, z (mm), tool pitch (deg) -> [s0..s5], or None
print(q)                     # [78.5 43.1 104.8 91.6 90. 90.]
print(fk(q)["tip"])          # [260. 80. 250.]  (round trip)

ik(600, 0, 100, 0)           # None: out of reach
```

| Function | Meaning |
|---|---|
| `fk(s)` | Servo angles (at least the first 4) to tip, wrist, elbow, shoulder positions and tool pitch |
| `ik(x, y, z, phi_deg, roll=90, grip=90)` | Tip position and pitch to six servo angles; `None` if unreachable or outside joint limits |
| `ik_branches(...)` | Both elbow solutions without limit checks (for diagnosing a failure) |
| `violations(s)` | List of joints outside their limits |
| `reach_ok(x, y, z, phi)` | True if `ik` finds a solution |

---

## 5. Arduino IK (optional, in the sketch)

Put `kinematics.h` next to `ArmHome.ino`:

```cpp
#include "kinematics.h"

float q[6] = { 0, 0, 0, 0, lastSent[4], lastSent[5] };   // keep roll and gripper
if (kin::ik(200, 80, 60, -60, q)) moveTo(q);               // false = unreachable, arm stays
kin::Pose p = kin::fk(lastSent);                           // p.x, p.y, p.z, p.phi
```

---

## 6. Typical session

```
1. Close the Serial Monitor.
2. python sim\arm_control.py --port COM8     (wait for "connected")
3. show                                       check the pose
4. signs ... compare ...                      once, to check joint directions
5. angles 90 60 100 40                        first moves by angle
6. xyz 250 0 130 -60                          then by position (after SIGNS_VERIFIED = True)
7. home
8. quit
```

Safety on first power-up: horns off or the arm held clear, servos on their own 5 to 6 V supply with grounds shared, and a hand near the supply switch.

More detail (wiring, settings, troubleshooting): `RUNBOOK.md`.
