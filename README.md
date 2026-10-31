# Robotics Arm

6-DOF arm. 3 x MG995 (waist, shoulder, elbow) + 3 x SG90 (wrist roll, wrist pitch, gripper) on a PCA9685, channels 0 to 5, no encoders. Controlled from the PC (Python) over serial through an Arduino-IDE sketch.

Joints (PCA9685 channel = joint number): **0 waist, 1 shoulder, 2 elbow, 3 wrist roll, 4 wrist pitch, 5 gripper.**
Angles are servo degrees (0 to 180 = PCA ticks 150 to 600). Positions are in mm: **x forward, y left, z up**, from the ground at the waist axis.

## Contents
1. [Project files](#1-project-files)
2. [Hardware and firmware setup](#2-hardware-and-firmware-setup)
3. [Run it from the PC](#3-run-it-from-the-pc)
4. [Every command, with examples](#4-every-command-with-examples)
5. [Arduino Serial Monitor commands](#5-arduino-serial-monitor-commands)
6. [FK and IK](#6-fk-and-ik)
7. [Calibration and joint directions](#7-calibration-and-joint-directions)
8. [First power-up checklist](#8-first-power-up-checklist)
9. [Troubleshooting](#9-troubleshooting)
10. [Known limits](#10-known-limits)

---

## 1. Project files

```
Robotics Arm\
  Arm_Bot_STLs\Arm_Bot_STLs\*.STL   the 11 CAD parts
  arduino\ArmHome\ArmHome.ino       firmware: home, smooth moves, EEPROM state, serial commands
  arduino\ArmHome\kinematics.h      FK / IK in C++ (not called by the sketch yet)
  sim\arm_control.py                PC controller: angles or xyz in, drives the real arm over serial (hardware only)
  sim\arm_ui.py, sim\ui\index.html  teaching UI in the browser: sliders, FK drawing, IK form, code box
  sim\kinematics.py                 FK / IK + calibration constants + 20,000-pose self-test
  sim\_test_link.py                 tests the serial protocol against a fake board
  sim\_views.py, sim\_ortho.py      CAD inspection helpers (STL renders), not used by arm_control.py
  sim\build_cad_model.py            reads the SolidWorks assembly dump and writes sim\ui\cad_model.json (lengths, joint axes, part placement)
  sim\ui\arm3d.html                 3D page built from the assembly (served at /3d)
  photo\home_real.png               photo of the real arm
  README.md                         this file
```

(The folder is still called `sim` for historical reasons; there is no simulation left in it.)

**Home pose** is `HOME[]` in `ArmHome.ino`, currently `{90, 90, 90, 60, 90, 90}`. Keep `HOME`, `LO`, `HI` and `MAX_DPS` in `sim\kinematics.py` and `kinematics.h` equal to the sketch. Current limits in the sketch: 0 to 180 on every joint.

---

## 2. Hardware and firmware setup

### One-time setup
1. Install the Arduino IDE.
2. Sketch > Include Library > Manage Libraries > install **Adafruit PWM Servo Driver Library**.
3. Wiring (Uno / Nano pins; on other boards use their SDA / SCL):

| PCA9685 | Connect to |
|---|---|
| VCC | Arduino 5 V (logic) |
| GND | Arduino GND **and** the servo supply GND |
| SDA | A4 |
| SCL | A5 |
| V+ (terminal block) | separate 5 to 6 V supply for the servos, sized for the MG995 stall current |
| Channel 0 to 5 | waist, shoulder, elbow, wrist roll, wrist pitch, gripper |

Do not power the servos from the Arduino 5 V pin.

### Upload
1. Open `arduino\ArmHome\ArmHome.ino`. The folder name must stay `ArmHome`.
2. Tools > Board and Tools > Port.
3. Upload.
4. Tools > Serial Monitor, **115200 baud**, line ending **Newline**.

### What happens on power-up
1. Starts I2C and the PCA9685 at 50 Hz.
2. Loads the last saved angles from EEPROM, or `PARK[]` if EEPROM is blank.
3. Sends that believed pose, waits 500 ms.
4. Glides to `HOME[]` (never faster than 1.5 s).
5. Waits for serial commands.

### Settings at the top of `ArmHome.ino`

| Constant | Meaning |
|---|---|
| `HOME[]` | Home pose, servo degrees |
| `PARK[]` | Pose assumed on the very first boot (blank EEPROM) |
| `ANG_MIN[]`, `ANG_MAX[]` | Soft limits per joint |
| `TRIM[]` | Per-servo offset in degrees for horn error |
| `TICK_MIN[]`, `TICK_MAX[]` | PCA9685 ticks at 0 and 180 degrees. 150 and 600 = `map(angle, 0, 180, 150, 600)`. Narrow `TICK_MAX` if a servo buzzes at its end |
| `MAX_DPS[]` | Speed limit per joint, degrees per second |
| `HOME_MIN_MS` | Minimum time for the move home |
| (no `setOscillatorFrequency`) | The library default 25 MHz is used on purpose |

---

## 3. Run it from the PC

Python 3.10 with numpy and pyserial. Run from the project root. Replace `COM8` with your port.

1. Upload the current `ArmHome.ino` (it has the `m` command), then **close the Arduino Serial Monitor**. Only one program can hold the port.
2. Find the port: Arduino IDE Tools > Port, or `python sim\arm_control.py` then type `ports`.
3. Start:

| Command | What it does |
|---|---|
| `python sim\arm_control.py --port COM8` | Connects, waits for the arm to finish its power-up home move, opens the `arm>` prompt |
| `python sim\arm_control.py --port COM8 angles 90 60 120 45` | One move by servo angles, then exits |
| `python sim\arm_control.py --port COM8 xyz 260 80 250 0` | One move by tip position, then exits |
| `python sim\arm_control.py` | Opens the prompt without a connection; use `ports` and `connect COMx` inside |
| `python sim\kinematics.py` | Self-test of FK and IK. No hardware needed |
| `python sim\_test_link.py` | Protocol test against a fake Arduino. No hardware needed |

```powershell
cd path\to\Robotics-Arm
pip install numpy pyserial
python sim\arm_control.py --port COM8
```

If `python` is not on your PATH, call the interpreter by its full path (or use `py -3.10`).

### What it does
- Opening the port usually resets the Arduino, so the arm first runs its power-up home move (up to about 12 s). The script waits, then reads the arm's own `lastSent[]` back.
- Each move is one line, `m a0 a1 a2 a3 a4 a5`. The firmware runs the synchronised smooth motion and answers `lastSent:` when finished; that reply is the new pose.
- If the Arduino does not answer, the pose is not updated; once the arm is idle type `sync`.
- Joint-limit violations, unreachable targets and failed IK are caught on the PC, so nothing is sent for them.

### Teaching UI (browser)

```powershell
python sim\arm_ui.py                  # opens http://127.0.0.1:8000, connect to the board inside the page
python sim\arm_ui.py --port COM8      # connect first
```

The **3D view** (`http://127.0.0.1:8000/3d`, linked from the top bar) shows the arm built from the CAD assembly in black and white. Each joint has a slider, a Spin button and a ring showing the axis it turns about, and the tool-tip position is read from the model. The serial port and Connect controls are hidden in the lab page; connect with `python sim\arm_ui.py --port COM8`.

- **Sliders** for the six joints, with the forward-kinematics numbers (tip position, tool pitch, waist angle, link angles) updating as you drag. Move arm sends the pose; "move the arm when I let go of a slider" does it automatically.
- **Inverse kinematics** tab: type x, y, z and tool pitch. Solve (just shows the angles, the sliders jump to them) or Solve and move arm.
- **Write code** tab: short Python (10 to 20 lines) with the FK and IK equations written out and the numbers to edit at the top (link lengths, servo angles, target x, y, z, pitch). Run with the button or Ctrl+Enter. Examples: *Forward kinematics (equations)*, *Inverse kinematics (equations)*, *FK then IK (round trip)*, *Move once*, *Sweep the waist*, *Square in the air (IK)*.
  - Maths: `sin cos atan2 acos sqrt radians degrees pi`, plus `for`, `if`, `range`, `round`, `print`. No `import`, no file access.
  - Helpers: `to_link_angles(s0, s1, s2, s3)` (servo degrees to link angles), `to_servo(yaw, a1, bend2, bend3)` (link angles back to servo degrees).
  - Arm: `move(a0, a1, a2, a3)`, `move_xyz(x, y, z, pitch)`, `home()`, `pose()`, `wait(s)`.
  - Built-in answers to compare with: `fk(a0, a1, a2, a3)` returns `(x, y, z, pitch)`; `ik(x, y, z, pitch)` returns the angles or `None`.
  - Stop ends a running script. It is a guard against accidents, not a security sandbox, so use it on a trusted machine.
- Without a board, FK and IK still work; moves answer "not connected".
- The page is served to this computer only.

---

## 4. Every command, with examples

### `ports`
Lists the serial ports the PC can see.
```
arm> ports
  COM8  Arduino Uno (COM8)
```

### `connect COMx`
Opens the port, waits for the arm to reach home (up to 25 s), reads the pose.
```
arm> connect COM8
  waiting for the Arduino to boot and reach home (up to 25 s)...
  connected on COM8. Arm at [90.0, 90.0, 90.0, 60.0, 90.0, 90.0].
```

### `disconnect`
Closes the port. Use it before opening the Arduino Serial Monitor.

### `show`
Reads the pose from the Arduino and prints it with the tip position (FK). No motion.
```
arm> show
  pose = [90.0, 90.0, 90.0, 60.0, 90.0, 90.0]
  tip  = [32.9, 18.8, 400.6] mm, tool pitch 31.3 deg
```

### `sync`
Same read as `show`. Use it after `WARNING: no reply from the Arduino`, once the arm has stopped.

### `home`
Moves to HOME.
```
arm> home
  from [100.0, 70.0, 130.0, 40.0, 90.0, 90.0]
  to   [90.0, 90.0, 90.0, 60.0, 90.0, 90.0]
  takes about 2.7 s
  arduino reports [90.0, 90.0, 90.0, 60.0, 90.0, 90.0]
```

### `angles s0 s1 s2 s3 [s4 s5]`
Moves to the servo angles you give. FK prints where the tip will be. Angles you leave out keep their current value. All joints move together and arrive at the same time.
```
arm> angles 100 70 130 40
  FK: tip = [122.3, 105.2, 192.5] mm, tool pitch = -57.7 deg
  from [90.0, 90.0, 90.0, 60.0, 90.0, 90.0]
  to   [100.0, 70.0, 130.0, 40.0, 90.0, 90.0]
  takes about 2.7 s
  arduino reports [100.0, 70.0, 130.0, 40.0, 90.0, 90.0]
```
```
arm> angles 100                  turn only the waist to 100
arm> angles 90 90 90 60 90 120   keep the arm at HOME, open the gripper to 120
arm> angles 90 190 90 60         refused: shoulder 190 is outside 0 to 180; nothing is sent
```

### `xyz x y z [phi]`
Moves the gripper tip to (x, y, z) mm. `phi` is the tool pitch in degrees: 0 = pointing forward, negative = pointing down; left out, the current pitch is kept. IK picks the servo angles, FK double-checks them, then the arm moves. Wrist roll and gripper keep their current values.

Until the joint directions are checked (section 7) it prints a one-line note first; it does not refuse.
```
arm> xyz 260 80 250 0
  IK: servo angles = [78.5, 43.1, 104.8, 91.6, 90.0, 90.0]  (tool pitch 0.0 deg)
  check: FK of that = [260.0, 80.0, 250.0] mm
  from [100.0, 70.0, 130.0, 40.0, 90.0, 90.0]
  to   [78.5, 43.1, 104.8, 91.6, 90.0, 90.0]
  takes about 1.9 s
  arduino reports [78.5, 43.1, 104.8, 91.6, 90.0, 90.0]
```
```
arm> xyz 250 0 130 -60        reach forward and low with the tool 60 degrees down
arm> xyz 200 100 200          keep the current tool pitch
arm> xyz 600 0 100 0          IK failed, out of reach (arm spans 3 to 277 mm from the shoulder); nothing is sent
```

### `signs`
Prints what +10 degrees on each joint should do according to the current `SIGN`, to compare with the real arm (procedure in section 7).
```
arm> signs
  channel 0 (waist   ) +10 deg: the whole arm turns to the LEFT (counter-clockwise seen from above)
  channel 1 (shoulder) +10 deg: ...
```

### `trust`
Marks the joint directions as checked for this session (sets `SIGNS_VERIFIED`) and silences the note on `xyz`. `--trust-signs` on the command line does the same.

### `help`, `quit`
`help` reprints the list. `quit` (also `q`, `exit`, Ctrl+C) closes the port and leaves.

### Messages you may see

| Message | Meaning |
|---|---|
| `not connected. Use --port COMx, or type: connect COMx` | Connect first |
| `refused, outside the joint limits: ...` | An angle is beyond the allowed range; nothing sent |
| `IK failed, out of reach ...` | The target is farther than the arm can stretch |
| `IK failed, joint limit. ...` | Reachable, but a joint would need an angle it cannot do; try a different `phi` |
| `WARNING: no reply from the Arduino` | Wait until the arm stops, then `sync` |
| `numbers only, e.g. xyz 260 80 250 0` | A value was not a number |

Example numbers above come from `kinematics.py` with HOME `{90, 90, 90, 60, 90, 90}`. Move times are estimates from the speed limits.

### Typical session
```
1. Close the Serial Monitor.
2. python sim\arm_control.py --port COM8     (wait for "connected")
3. show                                       check the pose
4. signs ... compare ...                      once, to check joint directions
5. angles 100 70 130 40                       first moves by angle
6. xyz 250 0 130 -60                          then by position
7. home
8. quit
```

---

## 5. Arduino Serial Monitor commands

Arduino IDE, Tools > Serial Monitor, **115200 baud**, line ending **Newline**. These are the same lines the PC program sends.

| Type | What it does | Example | Reply |
|---|---|---|---|
| `h` | Glides to HOME (never faster than 1.5 s) | `h` | `lastSent: 90.0, 90.0, 90.0, 60.0, 90.0, 90.0` |
| `p` | Prints the angles the Arduino believes each joint is at. No motion | `p` | `lastSent: ...` |
| `<ch> <deg>` | Moves one joint smoothly, others stay | `2 100` (elbow to 100) | `lastSent: ...` |
| `set <ch> <deg>` | **No motion.** Declares "this joint is physically at `<deg>`" and saves it to EEPROM | `set 2 100` | `lastSent: ...` |
| `m a0 a1 a2 a3 a4 a5` | Moves all six joints together, arriving at the same time | `m 80 60 90 40 90 90` | `lastSent: ...` |

- Every target is clamped to `ANG_MIN[]` / `ANG_MAX[]` in the sketch.
- With no encoders, "position" means the last value sent (`lastSent`). It is saved to EEPROM, so after a power cycle the Arduino starts from the last pose, then glides home.
- Use `set` when you moved the arm by hand or the believed pose is wrong: park the arm at a known pose, then `set` each joint.
- `m` needs exactly six numbers, otherwise `err: m needs 6 angles`.

```
h                         go home
p                         where does the Arduino think the arm is?
0 90                      turn the waist to 90
5 100                     open the gripper to 100
m 80 60 90 40 90 90       move everything at once
set 1 90                  I put the shoulder at 90 by hand, update the record
```

---

## 6. FK and IK

### From your own Python
```python
import sys; sys.path.insert(0, "sim")
from kinematics import fk, ik, HOME

f = fk(HOME)                    # dict: tip, wrist, elbow, shoulder, phi_deg
print(f["tip"], f["phi_deg"])   # tip in mm, tool pitch in deg

q = ik(260, 80, 250, 0)         # x, y, z in mm; tool pitch in deg (negative = down)
print(q)                        # [s0..s5] servo degrees, or None if unreachable / outside limits
print(fk(q)["tip"])             # [260. 80. 250.]  round trip
```

| Function | Meaning |
|---|---|
| `fk(s)` | Servo angles (at least the first 4) to tip, wrist, elbow, shoulder positions and tool pitch |
| `ik(x, y, z, phi_deg, roll=90, grip=90)` | Tip position and pitch to six servo angles; `None` if unreachable or outside joint limits |
| `ik_branches(...)` | Both elbow solutions without limit checks (for diagnosing a failure) |
| `violations(s)` | Joints outside their limits |
| `reach_ok(x, y, z, phi)` | True if `ik` finds a solution |

Both elbow bends are tried (forward first); the first one inside the limits is returned. `python sim\kinematics.py` runs the self-test (FK of HOME, IK back, 20,000-pose round trip, sample targets).

### From the sketch (optional)
Put `kinematics.h` next to `ArmHome.ino`:

```cpp
#include "kinematics.h"

float q[6] = { 0, 0, 0, lastSent[3], 0, lastSent[5] };   // keep roll (ch 3) and gripper
if (kin::ik(200, 80, 60, -60, q)) moveTo(q);               // false = unreachable, arm stays
kin::Pose p = kin::fk(lastSent);                           // p.x, p.y, p.z, p.phi
```

Keep the limits and link lengths in `kinematics.h` equal to the sketch and to `sim\kinematics.py`.

---

## 7. Calibration and joint directions

Planar 3-link arm with a waist yaw. Constants at the top of `sim\kinematics.py` (copy any change into `kinematics.h`):

| Constant | Value | Where it came from |
|---|---|---|
| `H` | 93.8 mm | CAD assembly, underside of the feet to the shoulder axis |
| `L1` | 140.0 mm | CAD assembly, shoulder axis to elbow axis |
| `L2` | 131.2 mm | CAD assembly, elbow axis to the wrist-pitch axis |
| `L3` | 100.2 mm | CAD assembly, wrist-pitch axis to the middle of the jaw tips |
| `TOOL_OFF` | 4.0 deg | The pivot-to-tip line is 4 degrees off the gripper body axis |
| `LAT` | 4.4 mm | The tip really sits this far left of the arm plane (roll at `ROLL_ZERO`). A constant: FK and IK ignore it |
| `ROLL_ZERO` | 60 | Roll angle where the pitch hinge is parallel to the elbow hinge (confirmed: the jaws open sideways at HOME roll 60) |
| `WAIST_ZERO` | 90 | Waist straight ahead |
| `SHOULDER_ZERO` | 90 | Upper arm vertical |
| `ELBOW_ZERO` | 90 | Forearm in line with the upper arm |
| `WRIST_ZERO` | 90 | Gripper in line with the forearm (channel 4, the wrist pitch) |
| `GAIN` | 1.098, 1.098, 1.098, 1.1558 | Physical degrees per servo degree |

The lengths were measured on `Arm_Bot_STLs\CADs\5_DOF_ARM_Assembly.SLDASM` (joint axes from its mates). Rebuild them with `python sim\build_cad_model.py` if the CAD changes; it needs `sim\ui\assembly.json`, `part_boxes.json` (dumped once through SolidWorks). The servo bodies are suppressed in that assembly, so the 3D page draws plain boxes for them.

**Joint order in the real arm:** waist, shoulder, elbow, then **roll about the forearm axis (ch 3)**, then **pitch (ch 4)** on a hinge that the roll carries, then the jaws. With roll at `ROLL_ZERO` the whole arm is planar and the FK / IK below are exact (the 3D page agrees with them to 0.1 mm). Rolling away from that swings the gripper out of the arm plane; only the 3D page models that.

**The zeros match the sketch's HOME `{90, 90, 90, 60, 90, 90}`**, where the real arm stands straight up with the gripper pointing up (confirmed). The tip is then at about (-7, 4.4, 465) mm. Wrist roll is channel 3 and wrist pitch is channel 4; in the Python code box the student order stays waist, shoulder, elbow, wrist, roll, grip.

### How FK and IK work (equations)
Both are plain trigonometry, in `sim\kinematics.py` and in the Write code box (`to_link_angles`, `to_servo`).

* **FK:** servo degrees become link angles first (`to_link_angles`). Each angle is added onto the one before it: `a2 = a1 + bend2`, `a3 = a2 + bend3` (= tool pitch). Then `r = L1 cos a1 + L2 cos a2 + L3 cos a3` and `z = H + L1 sin a1 + L2 sin a2 + L3 sin a3`. The waist yaw spins that reach into `x = r cos(yaw)` and `y = r sin(yaw)`.
* **IK:** (1) the waist: `yaw = atan2(y, x)` and `r = sqrt(x^2 + y^2)`. (2) With the chosen tool pitch, step back along the gripper to the wrist point `(rw, zw) = (r - L3 cos pitch, z - H - L3 sin pitch)`. (3) Two links remain, so the law of cosines gives `cos d = (rw^2 + zw^2 - L1^2 - L2^2) / (2 L1 L2)` for the elbow bend `d`, and the shoulder is `a1 = atan2(zw, rw) - atan2(L2 sin d, L1 + L2 cos d)`. The two signs of `d` are the two elbow solutions; limits then pick one, and `|cos d| > 1` means out of reach.

### Joint directions (`SIGN`)
A photo cannot show which way a servo turns, so `SIGN` is a guess (`[1, 1, -1, 1]`). Check once:

1. Connect and send `home`.
2. Type `signs` to see what +10 degrees on each joint should do.
3. On the real arm send each, for example `angles 100 90 90 60 90 90` (waist), `angles 90 100 90 60 90 90` (shoulder), `angles 90 90 100 60 90 90` (elbow), `angles 90 90 90 60 100 90` (wrist pitch, channel 4), then `home` again.
4. Where the real arm moves the other way, flip that joint's entry in `SIGN` (+1 / -1) in `sim\kinematics.py` and `kinematics.h`.
5. When all four match, set `SIGNS_VERIFIED = True` (or just use `trust`).

---

## 8. First power-up checklist
1. Horns off, or the arm held clear of everything.
2. Servos on their own supply, grounds shared.
3. Upload, open the Serial Monitor, watch the first `lastSent:` line.
4. Send `h` and confirm each joint moves the expected way; flip a horn by 180 degrees if one is reversed.
5. Compare the arm at HOME with the pose the model assumes (section 7). If it is off by about 10 degrees, the PCA9685 clock may be 25 MHz instead of 27 MHz: fix with `TRIM[]` and the zeros in `kinematics.py`.
6. Trim small offsets with `TRIM[]`; narrow `TICK_MIN[]` / `TICK_MAX[]` if a servo buzzes.
7. Check the joint directions (`signs`) and measure the real arm: edit `H`, `L1`, `L2`, `L3` in `sim\kinematics.py` and `kinematics.h`.

---

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| `could not open COMx` | Close the Arduino Serial Monitor (one program per port) and check the port with `ports` |
| `connected, but the Arduino did not report its pose` | `ArmHome.ino` is not running, or the baud rate is not 115200 |
| `WARNING: no reply from the Arduino` | Wait until the arm stops moving, then `sync` |
| Arm snaps on the first boot | The believed pose did not match the real pose. Park the arm at `PARK[]` before the first upload, or use `set` after powering |
| Believed angles are wrong after a power cut or a push by hand | Park the arm at a known pose, then `set <ch> <deg>` for each joint, or send `h` |
| Wants a blank start | Change `MAGIC` in the sketch. The old EEPROM is then ignored and `PARK[]` is used |
| Servo buzzes at an end | The 150 to 600 tick range reaches about 732 to 2928 us, past most servos' end stops. Narrow `TICK_MIN` / `TICK_MAX` or the limits |
| `ik()` returns `None` | Out of reach, or a joint would exceed its limit. Steep straight-down tool angles need a wrist below 0 degrees, so use a shallower pitch |
| `xyz` lands in the wrong place | Check the zeros (section 7), the joint directions (`signs`) and the link lengths |

## 10. Known limits
- Joint directions (`SIGN`) and the servo gains are not verified on the real arm. Lengths come from the CAD assembly, not from the built arm.
- IK assumes the roll is at `ROLL_ZERO`.
- The sketch has not been compiled in this project; expect to fix small things on the first compile.
- No collision or torque model.
- Without encoders, the believed position is only as good as the servo's ability to follow the commands.
