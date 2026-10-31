// FK / IK for the arm. Port of sim/kinematics.py (verified there, not compiled here).
// Frame: origin on the ground at the waist axis, x forward, y left, z up (mm).
// Calibrated against the photo of the real arm at HOME = {63, 63, 49.3, 19.9, 123.7, 58.9} (150..600 tick mapping):
// upper arm and forearm vertical, gripper horizontal pointing forward.
//   yaw = SIGN[0]*GAIN[0]*(s0-WAIST_ZERO) | a1 = 90 + SIGN[1]*GAIN[1]*(s1-SHOULDER_ZERO)
//   a2 = a1 + SIGN[2]*GAIN[2]*(s2-ELBOW_ZERO) | a3 = a2 + SIGN[3]*GAIN[3]*(s3-WRIST_ZERO)
//   (a3 = tool pitch phi, 0 = forward, negative = down)
// Keep every constant equal to sim/kinematics.py.
#pragma once
#include <math.h>
#include <stdint.h>

namespace kin {
const float H = 103.0f, L1 = 140.0f, L2 = 137.0f, L3 = 84.0f;   // ground->shoulder, upper arm, forearm, wrist->tips (mm)
const float WAIST_ZERO = 62.95f, SHOULDER_ZERO = 62.95f, ELBOW_ZERO = 49.29f, WRIST_ZERO = 97.79f;
const float GAIN[4] = { 1.098f, 1.098f, 1.098f, 1.1558f };      // physical degrees per servo degree
const int8_t SIGN[4] = { 1, 1, -1, 1 };                         // joint directions: UNVERIFIED guesses, check on the arm
const float LO[6] = {   0,   0,   0,   0,   0,   0 };           // keep equal to ANG_MIN / ANG_MAX
const float HI[6] = { 180, 180, 180, 180, 180, 180 };

struct Pose { float x, y, z, phi; };                              // phi = tool pitch, deg

inline float rad(float d) { return d * (float)M_PI / 180.0f; }
inline float deg(float r) { return r * 180.0f / (float)M_PI; }
inline float wrap(float a) { return a - 2.0f * (float)M_PI * floorf((a + (float)M_PI) / (2.0f * (float)M_PI)); }

// s[0..3] servo degrees -> tip position and tool pitch
inline Pose fk(const float s[]) {
  float a1 = rad(90.0f + SIGN[1] * GAIN[1] * (s[1] - SHOULDER_ZERO));
  float a2 = a1 + rad(SIGN[2] * GAIN[2] * (s[2] - ELBOW_ZERO));
  float a3 = a2 + rad(SIGN[3] * GAIN[3] * (s[3] - WRIST_ZERO));
  float r = L1 * cosf(a1) + L2 * cosf(a2) + L3 * cosf(a3);
  float z = H + L1 * sinf(a1) + L2 * sinf(a2) + L3 * sinf(a3);
  float yaw = rad(SIGN[0] * GAIN[0] * (s[0] - WAIST_ZERO));
  return { r * cosf(yaw), r * sinf(yaw), z, deg(a3) };
}

// target tip (mm) + tool pitch (deg) -> out[0..3] servo degrees. false = unreachable / outside limits.
// Tries the forward-bent elbow first, then the backward-bent one. x < 0 is reached leaning back.
inline bool ik(float x, float y, float z, float phiDeg, float out[]) {
  float yaw, r;
  if (x >= 0) { yaw = atan2f(y, x);   r =  sqrtf(x * x + y * y); }
  else        { yaw = atan2f(-y, -x); r = -sqrtf(x * x + y * y); }
  float phi = rad(phiDeg);
  float rw = r - L3 * cosf(phi), zw = z - L3 * sinf(phi) - H;    // wrist centre rel. to shoulder
  float c = (rw * rw + zw * zw - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (fabsf(c) > 1.000001f) return false;
  c = constrain(c, -1.0f, 1.0f);
  for (int side = -1; side <= 1; side += 2) {
    float d  = side * acosf(c);                                  // a2 - a1
    float a1 = wrap(atan2f(zw, rw) - atan2f(L2 * sinf(d), L1 + L2 * cosf(d)));
    float a2 = a1 + d;
    out[0] = WAIST_ZERO + deg(yaw) * SIGN[0] / GAIN[0];
    out[1] = SHOULDER_ZERO + (deg(a1) - 90.0f) * SIGN[1] / GAIN[1];
    out[2] = ELBOW_ZERO + deg(d) * SIGN[2] / GAIN[2];
    out[3] = WRIST_ZERO + deg(wrap(phi - a2)) * SIGN[3] / GAIN[3];
    bool ok = true;
    for (uint8_t i = 0; i < 4; i++) if (out[i] < LO[i] - 1e-3f || out[i] > HI[i] + 1e-3f) ok = false;
    if (ok) return true;
  }
  return false;
}
}  // namespace kin

/* Use from ArmHome.ino:
     #include "kinematics.h"
     float q[6] = { 0, 0, 0, 0, lastSent[4], lastSent[5] };       // keep roll / gripper
     if (kin::ik(260, 80, 250, 0, q)) moveTo(q);                  // else: unreachable, arm stays put
     kin::Pose p = kin::fk(lastSent);                              // where is the tip right now?
*/
