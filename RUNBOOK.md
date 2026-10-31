# Robotics Arm: run book

6-DOF arm. 3 x MG995 (waist, shoulder, elbow) + 3 x SG90 (wrist pitch, wrist roll, gripper) on a PCA9685, channels 0 to 5, no encoders.

```
Robotics Arm\
  Arm_Bot_STLs\Arm_Bot_STLs\*.STL   the 11 CAD parts
  arduino\ArmHome\ArmHome.ino       firmware: home, smooth moves, EEPROM state, serial commands
  arduino\ArmHome\kinematics.h      FK / IK in C++ (not called by the sketch yet)
  sim\arm_control.py                PC controller: angles or xyz in, drives the real arm over serial (hardware only)
  sim\kinematics.py                 FK / IK + calibration constants + 20,000-pose self-test
  sim\_test_link.py                 tests the serial protocol against a fake board
  sim\_views.py, sim\_ortho.py      CAD inspection helpers (STL renders), not used by arm_control.py
  photo\home_real.png               photo of the real arm at HOME, used to calibrate kinematics.py
  RUNBOOK.md                        this file
```

Current home pose, taken from `HOME[]` in `ArmHome.ino`: `{63.0, 63.0, 49.3, 19.9, 123.7, 58.9}`. This is the photo pose (earlier written `{90, 90, 75, 45, 165, 90}`) converted to the 150 to 600 tick mapping by matching pulse width. If you change `HOME[]` or the limits in the sketch, change `HOME`, `LO`, `HI` and `MAX_DPS` in `sim\kinematics.py` and `kinematics.h` too.

---

## 1. Control the real arm from the PC

Python 3.10 with numpy and pyserial. Run from the project root.

### Steps
1. Upload the current `ArmHome.ino` (it has the `m` command), then **close the Arduino Serial Monitor**. Only one program can hold the port.
2. Find the port: Arduino IDE Tools > Port, or `python sim\arm_control.py` then type `ports`.
3. Run:

```powershell
cd "E:\Claude\Robotics Arm"
python sim\arm_control.py --port COM8                         # interactive prompt
python sim\arm_control.py --port COM8 angles 90 60 120 45     # one move by servo angles, then exit
python sim\arm_control.py --port COM8 xyz 260 80 250 0        # one move by tip position + tool pitch, then exit
```

If `python` is not the 3.10 install, use the full path:

```powershell
& "C:\Users\Samay Jain\AppData\Local\Programs\Python\Python310\python.exe" sim\arm_control.py --port COM8
```

### What it does
- Opening the port usually resets the Arduino, so the arm first runs its power-up home move (up to about 12 s). The script waits for it, then reads the arm's own `lastSent[]` back.
- Each move is one line, `m a0 a1 a2 a3 a4 a5`. The firmware runs the synchronised smooth motion and answers with `lastSent:` when it has finished; that reply is the new pose.
- If the Arduino does not answer, the pose is not updated; once the arm is idle type `sync`.
- Joint-limit violations, unreachable targets and failed IK are caught on the PC, so nothing is sent for them.
- **First time:** horns off or the arm clear, servos on their own supply.

### Commands in the prompt

| Command | Effect |
|---|---|
| `angles s0 s1 s2 s3 [s4 s5]` | Move to servo angles (within the limits, about 0 to 145; shoulder to 131). FK prints the tip first. Angles you leave out keep their current value |
| `xyz x y z [phi]` | Move the tip to (x, y, z) mm. `phi` is the tool pitch in degrees, negative = down; left out, it keeps the current pitch |
| `home` | Go to HOME |
| `show` | Read the pose from the Arduino and print it with the tip position |
| `sync` | Re-read the pose from the Arduino |
| `signs` | What +10 degrees on each joint should do on the real arm (direction check, below) |
| `connect COMx`, `disconnect`, `ports` | Manage the serial connection |
| `help`, `quit` | |

x is forward, y is left, z is up, measured from the ground at the waist axis. Without a connection every move command just says "not connected".

Test without hardware: `python sim\_test_link.py` runs the whole path against a fake board.

### The model behind angles and xyz (`sim\kinematics.py`)

Calibrated against the photo of the real arm at HOME `[63, 63, 49.3, 19.9, 123.7, 58.9]`: upper arm and forearm vertical, gripper level pointing forward. Constants at the top of `sim\kinematics.py` (copy any change into `arduino\ArmHome\kinematics.h`):

| Constant | Value | Where it came from |
|---|---|---|
| `L1` | 140 mm | CAD, Arm_1 hole to hole (the photo agrees) |
| `H` | 103 mm | Photo; the CAD gives 98 plus the thickness of the star feet |
| `L2` | 137 mm | Photo, elbow axis to wrist axis |
| `L3` | 84 mm | Photo, wrist axis to gripper tips |
| `WAIST_ZERO` | 62.95 | Waist straight ahead |
| `SHOULDER_ZERO` | 62.95 | Upper arm vertical |
| `ELBOW_ZERO` | 49.29 | Forearm is in line with the upper arm |
| `WRIST_ZERO` | 97.79 | Gripper is in line with the forearm, so HOME's 19.9 holds it level |
| `GAIN` | 1.098, 1.098, 1.098, 1.1558 | Physical degrees per servo degree (the tick mapping spans about 2196 us per 180 deg against 2000 / 1900 us for the nominal servo) |

The photo scale (about 1.75 px/mm) comes from the MG995 servo size, so `H`, `L2`, `L3` are good to about 10 %. Measure with a ruler and edit them.

**Joint directions are not known.** A photo cannot show which way a servo turns, so `SIGN` in `kinematics.py` is a guess (`[1, 1, -1, 1]`) and `SIGNS_VERIFIED` is `False`. While it is `False`, `xyz` moves are refused (`angles` moves still work). Check once:

1. Connect and send `home`.
2. Type `signs` to see what +10 degrees on each joint should do.
3. On the real arm send each, for example `angles 63 63 59.3 19.9 123.7 58.9` for the elbow, then `home` again.
4. Where the real arm moves the other way, flip that joint's entry in `SIGN` (+1 / -1).
5. When all four match, set `SIGNS_VERIFIED = True`.

The elbow has about 95 servo degrees of travel one way and 49 the other (servo range 0 to 144.9, straight at 49.3), so targets close to the base can need more fold than the servo allows and are refused with the reason.

### Use FK / IK from your own Python

```python
import sys; sys.path.insert(0, "sim")
from kinematics import fk, ik, HOME

f = fk(HOME)                      # dict: tip, wrist, elbow, shoulder, phi_deg
print(f["tip"], f["phi_deg"])     # tip in mm (x forward, y left, z up), tool pitch in deg

q = ik(260, 80, 250, 0)           # x, y, z in mm; tool pitch in deg (negative = down)
print(q)                          # [s0..s5] servo degrees, or None if unreachable / outside limits
```

`python sim\kinematics.py` runs the self-test (FK of HOME, IK back, a 20,000-pose round trip, sample targets). `ik(x, y, z, phi_deg, roll=90, grip=90)` returns `None` when the target is out of reach or any joint would leave its limit. Both elbow bends are tried (forward first) and the first one inside the limits is returned.

---

## 2. Arduino firmware

### One-time setup
1. Install the Arduino IDE.
2. Sketch > Include Library > Manage Libraries > install **Adafruit PWM Servo Driver Library**.
3. Wiring (Uno / Nano pins; on other boards use their SDA / SCL):

| PCA9685 | Connect to |
|---|---|
| VCC | Arduino 5 V |
| GND | Arduino GND **and** the servo supply GND |
| SDA | A4 |
| SCL | A5 |
| V+ (terminal block) | separate 5 to 6 V supply for the servos, sized for the MG995 stall current |
| Channel 0 to 5 | waist, shoulder, elbow, wrist pitch, wrist roll, gripper |

Do not power the servos from the Arduino 5 V pin.

### Upload
1. Open `arduino\ArmHome\ArmHome.ino`. The folder name must stay `ArmHome`.
2. Tools > Board and Tools > Port.
3. Upload.
4. Tools > Serial Monitor, **115200 baud**, line ending **Newline**.

The sketch was written but not compiled or run on hardware here. Expect to fix small things on the first compile.

### What happens on power-up
1. Starts I2C and the PCA9685 at 50 Hz.
2. Loads the last saved angles from EEPROM, or `PARK[]` if EEPROM is blank.
3. Sends that believed pose, waits 500 ms.
4. Glides to `HOME[]` (never faster than 1.5 s).
5. Waits for serial commands.

### Serial commands

| Type | Effect | Example |
|---|---|---|
| `h` | Glide to HOME | `h` |
| `p` | Print the believed angles of all six joints | `p` |
| `<ch> <deg>` | Move one joint, others stay | `2 120` |
| `set <ch> <deg>` | Declare the joint is physically at `<deg>`. No motion; saves to EEPROM | `set 2 120` |
| `m a0 a1 a2 a3 a4 a5` | Move all six joints together (what `arm_control.py` sends) | `m 63 50 70 20 123.7 58.9` |

Channel numbers: 0 waist, 1 shoulder, 2 elbow, 3 wrist pitch, 4 wrist roll, 5 gripper. Every target is clamped to `ANG_MIN[]` / `ANG_MAX[]`. Reply format: `lastSent: 63.0, 63.0, 49.3, 19.9, 123.7, 58.9`.

### Settings at the top of `ArmHome.ino`

| Constant | Meaning |
|---|---|
| `HOME[]` | Home pose, servo degrees |
| `PARK[]` | Pose assumed on the very first boot (blank EEPROM) |
| `ANG_MIN[]`, `ANG_MAX[]` | Soft limits per joint. Set the gripper range first |
| `TRIM[]` | Per-servo offset in degrees for horn error |
| `TICK_MIN[]`, `TICK_MAX[]` | PCA9685 ticks at 0 and 180 degrees. 150 and 600 = the reference sketch's `map(angle, 0, 180, 150, 600)`, so an angle sends the same pulse as that sketch. Narrow `TICK_MAX` if a servo buzzes at its end |
| `MAX_DPS[]` | Speed limit per joint, degrees per second |
| `HOME_MIN_MS` | Minimum time for the move home |
| (no `setOscillatorFrequency`) | The library default 25 MHz is used on purpose, as in the reference sketch |

### Use IK from the sketch (optional)
Put `kinematics.h` next to `ArmHome.ino`, then:

```cpp
#include "kinematics.h"

float q[6] = { 0, 0, 0, 0, lastSent[4], lastSent[5] };   // keep roll and gripper
if (kin::ik(200, 80, 60, -60, q)) moveTo(q);               // false = unreachable, arm stays
kin::Pose p = kin::fk(lastSent);                           // where the tip is now
```

Keep the limits and link lengths in `kinematics.h` equal to the sketch and to `sim\kinematics.py`.

---

## 3. First power-up checklist
1. Horns off, or the arm held clear of everything.
2. Servos on their own supply, grounds shared.
3. Upload, open the Serial Monitor, watch the first `lastSent:` line.
4. Send `h` and confirm each joint moves the expected way. Flip a horn by 180 degrees or adjust the convention if one is reversed.
5. Horn positions are already right if the arm looks like the photo at HOME: upper arm and forearm vertical, gripper level pointing forward (shoulder 63 = vertical, elbow 49.3 = straight, wrist 97.8 = in line with the forearm). If it is off by about 10 degrees, the PCA9685 clock is probably 25 MHz rather than the 27 MHz the old numbers assumed: fix it with `TRIM[]` and the zeros in `kinematics.py`.
6. Trim small offsets with `TRIM[]`, narrow `TICK_MIN[]` / `TICK_MAX[]` if a servo buzzes.
7. Check the joint directions (`signs`, see section 1) and measure the real arm: edit `H`, `L1`, `L2`, `L3` (103, 140, 137, 84 mm) in `sim\kinematics.py` and `kinematics.h`.

## 4. Troubleshooting

| Symptom | Fix |
|---|---|
| `could not open COMx` | Close the Arduino Serial Monitor (one program per port) and check the port with `ports` |
| `connected, but the Arduino did not report its pose` | `ArmHome.ino` is not running, or the baud rate is not 115200 |
| `WARNING: no reply from the Arduino` | Wait until the arm stops moving, then type `sync` |
| Arm snaps on the first boot | The believed pose did not match the real pose. Park the arm at `PARK[]` before the first upload, or use `set` after powering |
| Believed angles are wrong after a power cut or a push by hand | Park the arm by hand at a known pose, then `set <ch> <deg>` for each joint, or send `h` |
| Wants a blank start | Change `MAGIC` in the sketch. The old EEPROM is then ignored and `PARK[]` is used |
| Servo buzzes at an end | The 150 to 600 tick range reaches about 732 to 2928 us, past most servos' end stops. The soft limits (0 to about 145) already stay under 2500 us; narrow `TICK_MIN` / `TICK_MAX` or the limits further if it still buzzes |
| `ik()` returns `None` | Out of reach, or a joint would exceed its limit. Steep straight-down tool angles need a wrist below 0 degrees, so use a shallower pitch |

## 5. Known limits
- The channel to joint order and joint directions are assumptions; the link lengths come from the CAD and a photo (about 10 % accurate). There is no assembly in the folder.
- No collision or torque model.
- Without encoders, the believed position is only as good as the servo's ability to follow the commands.
