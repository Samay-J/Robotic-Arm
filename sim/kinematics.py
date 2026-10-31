"""FK / IK for the Robotics Arm, calibrated against the photo of the real arm at HOME.

Frame: origin on the ground at the waist axis, x forward, y left, z up (mm).
All angles a1..a3 are absolute, in the vertical arm plane, measured from +x (forward) counter-clockwise.

  waist    s0 : yaw = SIGN[0]*GAIN[0] * (s0 - WAIST_ZERO)             WAIST_ZERO = straight ahead
  shoulder s1 : a1  = 90 + SIGN[1]*GAIN[1] * (s1 - SHOULDER_ZERO)     SHOULDER_ZERO = upper arm vertical
  elbow    s2 : a2  = a1 + SIGN[2]*GAIN[2] * (s2 - ELBOW_ZERO)        ELBOW_ZERO = forearm in line with upper arm
  wrist    s3 : a3  = a2 + SIGN[3]*GAIN[3] * (s3 - WRIST_ZERO)        WRIST_ZERO = gripper in line with forearm
  roll s4, grip s5 do not move the tool tip.
phi = a3 is the tool pitch (0 = pointing forward, -90 = pointing down).

Servo angles use the sketch's mapping: 0..180 = 150..600 PCA9685 ticks (about 732..2928 us). GAIN is the physical
degrees a joint turns per servo degree (a servo's real travel is about 2196 us / 180 deg of nominal, against
the 2000 us (MG995) or 1900 us (SG90) it was rated for), so GAIN is about 1.1.

Calibration source: the photo of the arm at HOME shows the upper arm and forearm both vertical and the gripper
horizontal pointing forward. The photo angles 90, 90, 75, 45, 165, 90 were taken in the old 500..2500 us mapping;
the zeros below are those same poses converted to the tick mapping by matching pulse width.
If the real arm does not look like the photo at HOME, the clock was probably 25 MHz when the photo was taken
(about 8 % longer pulses): add the difference with TRIM[] in ArmHome.ino and move the zeros here.
The photo cannot show which way each servo turns: SIGN is a guess. Check it with the jog test in
RUNBOOK.md and flip an entry to -1 if a joint moves the wrong way on the real arm.
"""
import numpy as np

# Lengths in mm. L1 is the Arm_1 hole-to-hole distance (CAD). H, L2, L3 are measured from the photo
# (scale from the MG995 servo size), so each is good to roughly +-10 %. Measure and edit.
H = 103.0       # ground to shoulder axis
L1 = 140.0      # shoulder axis to elbow axis
L2 = 137.0      # elbow axis to wrist axis
L3 = 84.0       # wrist axis to gripper tips

WAIST_ZERO = 62.95       # servo angle at which each joint is in its reference position (see top)
SHOULDER_ZERO = 62.95
ELBOW_ZERO = 49.29
WRIST_ZERO = 97.79
GAIN = np.array([1.098, 1.098, 1.098, 1.1558])   # physical degrees per servo degree for s0..s3
# Direction of s0..s3: +1 = increasing the angle turns the joint counter-clockwise (x forward, z up, seen from
# the arm's left side; waist: counter-clockwise from above). NOT visible in the photo, so these are guesses:
# waist, shoulder, wrist +1, and elbow -1 (with the elbow zero at 75 the servo has 105 deg of travel on one side
# and only 65 on the other; -1 puts the big range on the forward fold, which is the one reaching needs).
SIGN = np.array([1, 1, -1, 1])
# Flip to True only after the jog test (`signs` in arm_control.py) matched the real arm. Until then
# arm_control.py will not send xyz (IK) moves to the hardware.
SIGNS_VERIFIED = False

LO = np.array([0, 0, 0.0, 0, 0, 0.0])               # joint limits, same as ANG_MIN / ANG_MAX in the sketch
HI = np.array([180, 180.0, 180.0, 180.0, 180.0, 180.0])
HOME = np.array([90.0, 90.0, 90.0, 60.0, 90.0, 90.0])   # same as HOME[] in the sketch
MAX_DPS = np.array([36.4, 27.3, 27.3, 51.9, 51.9, 51.9])  # same as MAX_DPS[] in the sketch


def abs_angles(s):
    """servo angles -> absolute link angles a1, a2, a3 (radians) and yaw (radians)."""
    a1 = np.radians(90 + SIGN[1] * GAIN[1] * (s[1] - SHOULDER_ZERO))
    a2 = a1 + np.radians(SIGN[2] * GAIN[2] * (s[2] - ELBOW_ZERO))
    a3 = a2 + np.radians(SIGN[3] * GAIN[3] * (s[3] - WRIST_ZERO))
    return a1, a2, a3, np.radians(SIGN[0] * GAIN[0] * (s[0] - WAIST_ZERO))


def fk(s):
    """servo angles (>= 4 values, deg) -> dict(tip, wrist, elbow, shoulder, phi_deg)."""
    a1, a2, a3, yaw = abs_angles(s)
    def pt(r, z): return np.array([r * np.cos(yaw), r * np.sin(yaw), z])
    r1, z1 = L1 * np.cos(a1), H + L1 * np.sin(a1)
    r2, z2 = r1 + L2 * np.cos(a2), z1 + L2 * np.sin(a2)
    r3, z3 = r2 + L3 * np.cos(a3), z2 + L3 * np.sin(a3)
    return dict(shoulder=pt(0, H), elbow=pt(r1, z1), wrist=pt(r2, z2), tip=pt(r3, z3),
                phi_deg=float(np.degrees(a3)))


def wrap(a):
    """radians -> (-pi, pi]"""
    return np.pi - (np.pi - a) % (2 * np.pi)


def _solve(r, yaw, z, phi, roll, grip, side):
    """One IK branch. side = -1: forearm bends forward of the upper arm line; +1: backward."""
    rw, zw = r - L3 * np.cos(phi), z - L3 * np.sin(phi) - H      # wrist centre, shoulder-relative
    c = (rw**2 + zw**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c) > 1 + 1e-9:
        return None                                              # out of reach
    d = side * np.arccos(np.clip(c, -1, 1))                      # a2 - a1
    a1 = wrap(np.arctan2(zw, rw) - np.arctan2(L2 * np.sin(d), L1 + L2 * np.cos(d)))
    a2 = a1 + d
    g = SIGN / GAIN                                              # +-1 for the sign, 1/GAIN for servo degrees
    return np.array([WAIST_ZERO + np.degrees(yaw) * g[0],
                     SHOULDER_ZERO + (np.degrees(a1) - 90) * g[1],
                     ELBOW_ZERO + np.degrees(d) * g[2],
                     WRIST_ZERO + np.degrees(wrap(phi - a2)) * g[3],
                     roll, grip])


def ik_branches(x, y, z, phi_deg, roll=90.0, grip=90.0):
    """Both elbow solutions, limits NOT checked: [forward-bent, backward-bent], or [] if out of reach."""
    # waist only spans +-90 deg about +x, so a target with x < 0 is reached leaning BACK
    # (signed radius r < 0, waist turned toward the opposite side).
    if x >= 0:
        yaw, r = np.arctan2(y, x), np.hypot(x, y)
    else:
        yaw, r = np.arctan2(-y, -x), -np.hypot(x, y)
    sols = [_solve(r, yaw, z, np.radians(phi_deg), roll, grip, side) for side in (-1, 1)]
    return [] if sols[0] is None else sols


def violations(s):
    """joints (names) that s puts outside their limits."""
    names = ['waist', 'shoulder', 'elbow', 'wrist', 'roll', 'grip']
    return [(names[i], s[i], LO[i], HI[i]) for i in range(6) if s[i] < LO[i] - 1e-9 or s[i] > HI[i] + 1e-9]


def ik(x, y, z, phi_deg, roll=90.0, grip=90.0, check_limits=True):
    """tip position (mm) + tool pitch (deg, absolute) -> servo angles [s0..s5], or None.
    Tries the forward-bent elbow first, then the backward-bent one, and returns the first within the limits."""
    for s in ik_branches(x, y, z, phi_deg, roll, grip):
        if not check_limits or not violations(s):
            return s
    return None


def reach_ok(x, y, z, phi_deg): return ik(x, y, z, phi_deg) is not None


if __name__ == '__main__':
    np.set_printoptions(precision=2, suppress=True)
    f = fk(HOME); print('HOME tip', f['tip'], 'phi', f['phi_deg'])
    print('IK(home tip) ->', ik(*f['tip'], f['phi_deg']))

    # round-trip FK -> IK -> FK over random poses inside the joint limits
    rng = np.random.default_rng(0); n = bad = 0; worst = 0
    for _ in range(20000):
        s = LO + rng.random(6) * (HI - LO)
        t = fk(s); q = ik(*t['tip'], t['phi_deg'], s[4], s[5])
        n += 1
        if q is None: bad += 1; continue
        e = np.linalg.norm(fk(q)['tip'] - t['tip']); worst = max(worst, e)
        assert abs(np.degrees(wrap(np.radians(fk(q)['phi_deg'] - t['phi_deg'])))) < 1e-6   # pitch is an angle: compare mod 360
    print(f'round trip: {n} poses, IK returned a pose for {n-bad}, worst tip error {worst:.2e} mm')
    print('unreachable (600,0,100):', ik(600, 0, 100, 0), '| behind base (-150,0,150):', ik(-150, 0, 150, 0))
    print('sample: tip (260, 80, 250) tool level ->', ik(260, 80, 250, 0))
    print('sample: tip (250, 0, 130) tool 60 deg down ->', ik(250, 0, 130, -60))
    print('sample: tip (200, 80, 150) tool level ->', ik(200, 80, 150, 0), '(needs more elbow fold than the servo allows)')
