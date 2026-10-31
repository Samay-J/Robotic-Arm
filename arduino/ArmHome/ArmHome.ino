/*
  ArmHome - 6-DOF arm (3x MG995 + 3x SG90) on a PCA9685, no encoders.

  PCA9685 channel map
    0  MG995  waist     (Rotating_Waste yaw, vertical axis)
    1  MG995  shoulder  (Arm_1 pitch)
    2  MG995  elbow     (Arm_2 pitch)
    3  SG90   wrist roll
    4  SG90   wrist pitch
    5  SG90   gripper

  No feedback exists, so "where the arm is" == "the last angle we sent".
  lastSent[] is that belief. It is mirrored to EEPROM after every move so a
  reset/power cycle starts from the real pose instead of snapping to a guess.

  Angle meaning at HOME = {90, 90, 90, 60, 90, 90} (confirmed on the real arm: it stands straight up):
    waist    90    = straight ahead
    shoulder 90    = upper arm vertical
    elbow    90    = forearm in line with the upper arm (straight)
    wrist    90    = gripper in line with the forearm, pointing straight up (channel 4)
    roll     60    = channel 3, does not move the tip
  One servo degree now turns a joint about 1.1 physical degrees (see GAIN in sim/kinematics.py).
  Which way each servo turns is not confirmed; see RUNBOOK.md and sim/kinematics.py (SIGN).

  Serial (115200, newline):
    h              go home
    p              print lastSent[]
    <ch> <deg>     move one joint, e.g. "2 120"
    m a0 a1 a2 a3 a4 a5   move all six together (used by sim/arm_control.py), e.g. "m 90 60 120 45 90 90"
    set <ch> <deg> declare the arm is physically AT <deg> (re-sync belief, no motion)
*/
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

const uint8_t N = 6;
Adafruit_PWMServoDriver pwm(0x40);

// ---------- tune these ----------
// Angles are in the 150..600-tick units below. They were converted from the earlier 500..2500 us numbers by
// matching pulse width, so the arm still sits in the photo pose (upper arm and forearm vertical, gripper level).
const float HOME[N]    = {  90.0,  90.0,  90.0,  60.0, 90.0,  90.0 };
// Pose assumed on the very first boot (empty EEPROM). Set to how you hand-park the arm.
const float PARK[N]    = {  63.0,  63.0,  63.0,  19.9,  58.9,  58.9 };
// Soft limits keep every pulse inside the old range (<= 2500 us) so no servo is driven past its end stop.
const float ANG_MIN[N] = {   0,   0,   0,   0,   0,   0 };
const float ANG_MAX[N] = { 180.0, 180.0, 180.0, 180.0, 180, 180.0 };
const float TRIM[N]    = {   0,   0,   0,   0,   0,   0 };   // per-servo offset, deg
// Angle -> PCA9685 ticks, identical to the reference sketch: map(angle, 0, 180, 150, 600) at 50 Hz, 25 MHz clock
// (1 tick = 4.88 us, so 0 deg = ~732 us, 90 deg = 375 ticks = ~1830 us, 180 deg = ~2928 us).
const int   TICK_MIN[N] = { 150, 150, 150, 150, 150, 150 };  // ticks at 0 deg
const int   TICK_MAX[N] = { 600, 600, 600, 600, 600, 600 };  // ticks at 180 deg
const float MAX_DPS[N] = {  36.4, 27.3, 27.3, 51.9, 51.9, 51.9 };   // slew limit, deg/s
const uint16_t HOME_MIN_MS = 1500;                            // never faster than this
// ---------------------------------

const uint16_t MAGIC = 0xA5C3;          // EEPROM: [magic][N * int16 angle*10]
float lastSent[N];                      // the arm's believed position

float clampAng(uint8_t i, float a) { return constrain(a, ANG_MIN[i], ANG_MAX[i]); }

void writeServo(uint8_t i, float a) {
  a = clampAng(i, a);
  lastSent[i] = a;                                         // << the "encoder"
  float t = TICK_MIN[i] + (TICK_MAX[i] - TICK_MIN[i]) * (a + TRIM[i]) / 180.0f;
  pwm.setPWM(i, 0, (uint16_t)(t + 0.5f));
}

void saveState() {
  int addr = 0;
  EEPROM.put(addr, MAGIC); addr += sizeof(MAGIC);
  for (uint8_t i = 0; i < N; i++) {
    int16_t v = (int16_t)(lastSent[i] * 10.0f + 0.5f);
    int16_t old; EEPROM.get(addr, old);
    if (old != v) EEPROM.put(addr, v);                     // only write on change (wear)
    addr += sizeof(v);
  }
}

bool loadState() {
  uint16_t m; int addr = 0;
  EEPROM.get(addr, m); addr += sizeof(m);
  if (m != MAGIC) return false;
  for (uint8_t i = 0; i < N; i++) {
    int16_t v; EEPROM.get(addr, v); addr += sizeof(v);
    lastSent[i] = clampAng(i, v / 10.0f);
  }
  return true;
}

// Move every joint together; all finish at the same time (min-jerk easing),
// duration set by the slowest joint. Blocking.
void moveTo(const float tgt[N], uint16_t minMs = 0) {
  float from[N], dur = minMs / 1000.0f;
  for (uint8_t i = 0; i < N; i++) {
    from[i] = lastSent[i];
    float t = clampAng(i, tgt[i]);
    // peak speed of min-jerk is 1.875x the mean, so scale by that
    dur = max(dur, fabs(t - from[i]) * 1.875f / MAX_DPS[i]);
  }
  uint32_t t0 = millis(), T = (uint32_t)(dur * 1000.0f);
  while (true) {
    uint32_t e = millis() - t0;
    float s = (e >= T) ? 1.0f : (float)e / T;
    float k = s * s * s * (10 + s * (-15 + 6 * s));        // smootherstep
    for (uint8_t i = 0; i < N; i++)
      writeServo(i, from[i] + (clampAng(i, tgt[i]) - from[i]) * k);
    if (s >= 1.0f) break;
    delay(20);                                             // 50 Hz servo frame
  }
  saveState();
}

void goHome() { moveTo(HOME, HOME_MIN_MS); }

void printState() {
  Serial.print(F("lastSent: "));
  for (uint8_t i = 0; i < N; i++) { Serial.print(lastSent[i], 1); Serial.print(i < N - 1 ? ", " : "\n"); }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  // No setOscillatorFrequency(): default 25 MHz, same as the reference sketch, so a given tick count means the same pulse.
  pwm.setPWMFreq(50);
  delay(10);

  if (!loadState()) for (uint8_t i = 0; i < N; i++) lastSent[i] = PARK[i];

  // Re-assert the believed pose first so servos hold where they physically are
  // (the PCA9685 powers up with no pulses), then glide home.
  for (uint8_t i = 0; i < N; i++) writeServo(i, lastSent[i]);
  delay(500);
  printState();
  goHome();
  Serial.println(F("Home reached. Commands: h | p | <ch> <deg> | set <ch> <deg>"));
}

void loop() {
  if (!Serial.available()) return;
  String l = Serial.readStringUntil('\n'); l.trim();
  int ch, a;
  if (l == "h") { goHome(); printState(); }
  else if (l == "p") printState();
  else if (l.startsWith("set ")) {
    if (sscanf(l.c_str() + 4, "%d %d", &ch, &a) == 2 && ch >= 0 && ch < N) {
      lastSent[ch] = clampAng(ch, a); saveState(); printState();  // belief only, no pulse change
    }
  } else if (l.startsWith("m ")) {                            // m s0 s1 s2 s3 s4 s5 : all joints, together
    float t[N]; char buf[80]; uint8_t n = 0;
    l.substring(2).toCharArray(buf, sizeof(buf));
    for (char *tok = strtok(buf, " "); tok && n < N; tok = strtok(NULL, " ")) t[n++] = atof(tok);  // sscanf has no %f on AVR
    if (n == N) { moveTo(t); printState(); }
    else Serial.println(F("err: m needs 6 angles"));
  } else if (sscanf(l.c_str(), "%d %d", &ch, &a) == 2 && ch >= 0 && ch < N) {
    float t[N]; for (uint8_t i = 0; i < N; i++) t[i] = lastSent[i];
    t[ch] = a; moveTo(t); printState();
  }
}
