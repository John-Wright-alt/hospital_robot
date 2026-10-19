#pragma once
// =============================================================================
//  lift.h  -  head-column lift for pico_motor_firmware  (Phase 8, firmware 1.1.0)
//
//  Hardware = wiring diagram rev 2, page 3:
//     DRV8825 (1/8 step hard-wired)  +  NEMA 17 42BYGHW208  +  KW11-2 home switch
//     GP10 -> STEP    GP11 -> DIR    GP12 -> EN (active LOW, R16 10k pull-up)
//     GP13 <- home switch: C -> GND, NC -> GP13, internal pull-up
//     VMOT comes from the switched motor bus, so the lift ONLY has power while
//     the firmware is ARMED (E-stop and relay protect the lift too).
//
//  New serial commands (next to your existing v / a / c / i / k / h):
//     l h              home: find the bottom switch, that becomes 0 mm
//     l p              park: fast down to LIFT_PARK_APPROACH_MM, creep onto the
//                      switch, re-zero (corrects any lost steps)
//     l g <mm> [v]     go to <mm> above the switch at v mm/s (default 15)
//     l s              stop immediately
//     l ?              print lift settings and the switch reading
//
//  New status line, Pico -> Pi, every 100 ms and whenever something changes:
//     l <state> <homed> <pos_mm> <target_mm> <switch> <fault>
//        state : 0 IDLE  1 MOVING  2 HOMING  3 PARKING  4 FAULT
//        homed : 1 = position is known (0 after power-up or motor power loss)
//        switch: 1 = home switch pressed (or its wire is broken - fail-safe)
//        fault : 0 none  1 switch never found  2 switch stuck pressed
//                3 switch not found on the slow approach
//
//  Dual-core safe: the 20 kHz step interrupt runs on the core that called
//  liftBegin() (core 0), while liftUpdate()/liftCommand() are called from core 1.
//  All shared values are protected by an RP2040 critical section (hardware
//  spinlock), which works across both cores.
// =============================================================================
#include <Arduino.h>
#include <stdarg.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "pico/critical_section.h"

// ------------------------------- SETTINGS -----------------------------------
// Each can be overridden by #defining it (e.g. in config.h) BEFORE #include "lift.h".
// Pins - wiring rev 2, page 3 and the master pin map on page 7
#ifndef PIN_LIFT_STEP
#define PIN_LIFT_STEP   10
#endif
#ifndef PIN_LIFT_DIR
#define PIN_LIFT_DIR    11
#endif
#ifndef PIN_LIFT_EN
#define PIN_LIFT_EN     12
#endif
#ifndef PIN_LIFT_HOME
#define PIN_LIFT_HOME   13
#endif

#ifndef LIFT_DIR_HIGH_MEANS_UP
#define LIFT_DIR_HIGH_MEANS_UP   1     // lift goes the wrong way? change to 0
#endif
#ifndef LIFT_SWITCH_NC
#define LIFT_SWITCH_NC           1     // 1 = C->GND, NC->GP13 (your wiring)
#endif

// Mechanics
#ifndef LIFT_STEPS_PER_REV
#define LIFT_STEPS_PER_REV     200     // 42BYGHW208: 1.8 deg/step
#endif
#ifndef LIFT_MICROSTEPS
#define LIFT_MICROSTEPS        8       // M0 + M1 to 3V3, M2 to GND (page 3)
#endif
#ifndef LIFT_MM_PER_REV
#define LIFT_MM_PER_REV        8.0f    // lead screw lead (T8x8 = 8, T8x2 = 2). MEASURE
#endif
#ifndef LIFT_TRAVEL_MM
#define LIFT_TRAVEL_MM         400.0f  // switch -> mechanical top. MEASURE
#endif
#ifndef LIFT_SOFT_MIN_MM
#define LIFT_SOFT_MIN_MM       5.0f    // 'l g' never goes below this (keeps off the switch)
#endif
#ifndef LIFT_SOFT_MAX_MM
#define LIFT_SOFT_MAX_MM       300.0f  // 'l g' never goes above this
#endif

// Speeds (mm/s) - slow, this moves at a patient's face height
#ifndef LIFT_MAX_SPEED_MM_S
#define LIFT_MAX_SPEED_MM_S    20.0f
#endif
#ifndef LIFT_DEFAULT_SPEED_MM_S
#define LIFT_DEFAULT_SPEED_MM_S 15.0f
#endif
#ifndef LIFT_HOME_FAST_MM_S
#define LIFT_HOME_FAST_MM_S    10.0f
#endif
#ifndef LIFT_HOME_SLOW_MM_S
#define LIFT_HOME_SLOW_MM_S     3.0f
#endif
#ifndef LIFT_ACCEL_MM_S2
#define LIFT_ACCEL_MM_S2       40.0f
#endif
#ifndef LIFT_BACKOFF_MM
#define LIFT_BACKOFF_MM         5.0f
#endif
#ifndef LIFT_PARK_APPROACH_MM
#define LIFT_PARK_APPROACH_MM  10.0f
#endif
#ifndef LIFT_DISABLE_WHEN_PARKED
#define LIFT_DISABLE_WHEN_PARKED 1     // driver off while resting on the switch
#endif
#ifndef LIFT_STATUS_MS
#define LIFT_STATUS_MS         100     // 'l' status line period
#endif

// How lift messages are printed. If your firmware's liftSay() uses a lock, add
//   #define LIFT_PRINTLN(text) liftSay("%s", text)
// ABOVE the #include "lift.h" line.
#ifndef LIFT_PRINTLN
#define LIFT_PRINTLN(text) Serial.println(text)
#endif
// ----------------------------------------------------------------------------

namespace lift {

constexpr int32_t  TICK_HZ        = 20000;   // step interrupt: every 50 us
constexpr int32_t  MAX_STEP_RATE  = 8000;
constexpr int32_t  DEBOUNCE_TICKS = 40;      // switch must be stable for 2 ms
constexpr float    STEPS_PER_MM   = (float)LIFT_STEPS_PER_REV * LIFT_MICROSTEPS / LIFT_MM_PER_REV;
constexpr uint32_t PHASE_FULL     = (uint32_t)TICK_HZ * 256u;

static_assert(LIFT_MAX_SPEED_MM_S * (LIFT_STEPS_PER_REV * LIFT_MICROSTEPS / LIFT_MM_PER_REV) <= 8000.0f,
              "LIFT_MAX_SPEED_MM_S needs more than 8000 steps/s - lower it or the microstepping");
static_assert(LIFT_SOFT_MAX_MM <= LIFT_TRAVEL_MM, "LIFT_SOFT_MAX_MM must be <= LIFT_TRAVEL_MM");
static_assert(LIFT_SOFT_MIN_MM > 0.0f, "LIFT_SOFT_MIN_MM must be above 0");

enum Mode  : uint8_t { MODE_IDLE = 0, MODE_POSITION, MODE_SEEK_DOWN, MODE_DECEL };
enum State : uint8_t { ST_IDLE = 0, ST_MOVING = 1, ST_HOMING = 2, ST_PARKING = 3, ST_FAULT = 4 };
enum Phase : uint8_t { PH_BACKOFF_FIRST, PH_FAST_DOWN, PH_BACKOFF, PH_APPROACH, PH_SLOW_DOWN };

// ---- shared between the interrupt and loop() -------------------------------
static volatile int32_t  s_pos = 0, s_target = 0;
static volatile int32_t  s_v = 0, s_vmax = 0;         // steps/s * 65536
static volatile int32_t  s_accelTick = 1, s_accel = 1, s_vsnap = 100;
static volatile uint8_t  s_mode = MODE_IDLE;
static volatile int8_t   s_dir = -1;
static volatile bool     s_stepHigh = false;
static volatile uint32_t s_phase = 0;
static volatile bool     s_bottom = false;
static volatile uint16_t s_bottomCnt = 0;
static volatile bool     s_bottomHit = false, s_topHit = false;
static volatile bool     s_softMaxOn = false;
static volatile int32_t  s_softMax = INT32_MAX;

// ---- loop()-only state ------------------------------------------------------
static repeating_timer_t s_timer;
static critical_section_t s_cs;            // works across both cores
static bool     s_started = false;
static bool     s_enabled = false;
static State    s_state = ST_IDLE;
static Phase    s_phaseHome = PH_FAST_DOWN;
static bool     s_homed = false;
static uint8_t  s_fault = 0;
static int32_t  s_seekStart = 0, s_seekLimit = 0;
static bool     s_wasArmed = false;
static bool     s_dirty = true;
static uint32_t s_lastStatusMs = 0;
static float    s_lastCorrectionMm = 0.0f;

// ---- helpers ------------------------------------------------------------------
inline int32_t mmToSteps(float mm)  { return (int32_t)lroundf(mm * STEPS_PER_MM); }
inline float   stepsToMm(int32_t s) { return (float)s / STEPS_PER_MM; }

static void liftSay(const char* fmt, ...) {
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "# lift: ");
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf + n, sizeof(buf) - n, fmt, ap);
  va_end(ap);
  LIFT_PRINTLN(buf);
}

static inline bool switchRaw() {
  bool high = gpio_get(PIN_LIFT_HOME);
  return LIFT_SWITCH_NC ? high : !high;   // NC opens when pressed -> pull-up -> HIGH
}

static inline void setDirPin(int8_t dir) {
  bool high = (dir > 0) ? (LIFT_DIR_HIGH_MEANS_UP != 0) : (LIFT_DIR_HIGH_MEANS_UP == 0);
  gpio_put(PIN_LIFT_DIR, high);
}

// ---- the 20 kHz interrupt -----------------------------------------------------
static inline void tickBody() {
  bool justLowered = false;
  if (s_stepHigh) { gpio_put(PIN_LIFT_STEP, 0); s_stepHigh = false; justLowered = true; }

  // de-bounce the home switch
  bool raw = switchRaw();
  if (raw != s_bottom) { if (++s_bottomCnt >= DEBOUNCE_TICKS) { s_bottom = raw; s_bottomCnt = 0; } }
  else s_bottomCnt = 0;

  const uint8_t mode = s_mode;
  if (mode == MODE_IDLE) { s_v = 0; return; }

  int32_t v = s_v;
  int32_t vInt = v / 65536;
  int32_t desired = 0;

  if (mode == MODE_POSITION) {
    const int32_t d = s_target - s_pos;
    if (d == 0) {
      int32_t av = vInt < 0 ? -vInt : vInt;
      if (av <= s_vsnap) { s_v = 0; s_phase = 0; s_mode = MODE_IDLE; return; }
      desired = 0;
    } else {
      const int8_t toward = d > 0 ? 1 : -1;
      const int32_t ad = d > 0 ? d : -d;
      const int32_t stopDist = (vInt * vInt) / (2 * s_accel);
      const bool sameWay = (vInt == 0) || ((vInt > 0) == (toward > 0));
      desired = (sameWay && ad > stopDist) ? toward * s_vmax : 0;
    }
  } else if (mode == MODE_SEEK_DOWN) {
    desired = -s_vmax;
  } else {  // MODE_DECEL
    if (vInt == 0) { s_v = 0; s_phase = 0; s_mode = MODE_IDLE; return; }
    desired = 0;
  }

  const int32_t a = s_accelTick;
  if (v < desired)      { v += a; if (v > desired) v = desired; }
  else if (v > desired) { v -= a; if (v < desired) v = desired; }
  s_v = v;

  const int8_t want = v > 0 ? 1 : (v < 0 ? -1 : 0);
  if (want == 0) return;
  if (want != s_dir) { setDirPin(want); s_dir = want; s_phase = 0; return; }

  // hard safety limits - checked before EVERY step
  if (want < 0 && s_bottom) { s_v = 0; s_phase = 0; s_mode = MODE_IDLE; s_bottomHit = true; return; }
  if (want > 0 && s_softMaxOn && s_pos >= s_softMax) {
    s_v = 0; s_phase = 0; s_mode = MODE_IDLE; s_topHit = true; return;
  }

  const uint32_t speed256 = (uint32_t)((v < 0 ? -v : v) >> 8);
  uint32_t ph = s_phase + speed256;
  if (ph >= PHASE_FULL && !justLowered) {
    ph -= PHASE_FULL;
    gpio_put(PIN_LIFT_STEP, 1);
    s_stepHigh = true;
    s_pos = s_pos + want;
  }
  if (ph > 2 * PHASE_FULL) ph = PHASE_FULL;
  s_phase = ph;
}

static bool onTick(repeating_timer_t*) {
  critical_section_enter_blocking(&s_cs);   // core 1 may be changing the target
  tickBody();
  critical_section_exit(&s_cs);
  return true;
}

// ---- motion primitives (called from loop) -------------------------------------
static void enableDriver(bool on) {
  s_enabled = on;
  gpio_put(PIN_LIFT_EN, on ? 0 : 1);            // active LOW
}

static void setProfile(float speedMmS) {
  if (speedMmS > LIFT_MAX_SPEED_MM_S) speedMmS = LIFT_MAX_SPEED_MM_S;
  if (speedMmS < 0.5f) speedMmS = 0.5f;
  float sps = speedMmS * STEPS_PER_MM;
  if (sps > MAX_STEP_RATE) sps = MAX_STEP_RATE;
  float acc = LIFT_ACCEL_MM_S2 * STEPS_PER_MM;
  if (acc < 50.0f) acc = 50.0f;
  int32_t accelTick = (int32_t)(acc * 65536.0f / TICK_HZ);
  if (accelTick < 1) accelTick = 1;
  int32_t vsnap = (int32_t)sqrtf(4.0f * acc);
  if (vsnap < 20) vsnap = 20;
  critical_section_enter_blocking(&s_cs);
  s_vmax = (int32_t)(sps * 65536.0f);
  s_accel = (int32_t)acc;
  s_accelTick = accelTick;
  s_vsnap = vsnap;
  critical_section_exit(&s_cs);
}

static void moveTo(int32_t target, float speedMmS) {
  setProfile(speedMmS);
  if (!s_enabled) { enableDriver(true); delay(2); }
  critical_section_enter_blocking(&s_cs);
  s_target = target;
  s_mode = MODE_POSITION;
  critical_section_exit(&s_cs);
}

static void seekDown(float speedMmS) {
  setProfile(speedMmS);
  if (!s_enabled) { enableDriver(true); delay(2); }
  critical_section_enter_blocking(&s_cs);
  s_bottomHit = false;
  s_mode = MODE_SEEK_DOWN;
  critical_section_exit(&s_cs);
}

static void stopNow() {
  critical_section_enter_blocking(&s_cs);
  s_mode = MODE_IDLE; s_v = 0; s_phase = 0;
  critical_section_exit(&s_cs);
}

static void setPosition(int32_t p) {
  critical_section_enter_blocking(&s_cs);
  s_pos = p; s_target = p;
  critical_section_exit(&s_cs);
}

static void setSoftMax(bool on, int32_t maxSteps) {
  critical_section_enter_blocking(&s_cs);
  s_softMaxOn = on; s_softMax = maxSteps;
  critical_section_exit(&s_cs);
}

static bool takeFlag(volatile bool& f) {
  critical_section_enter_blocking(&s_cs);
  bool r = f; f = false;
  critical_section_exit(&s_cs);
  return r;
}

inline bool moving() { return s_mode != MODE_IDLE; }

static void setState(State s) { if (s != s_state) { s_state = s; s_dirty = true; } }

static void fault(uint8_t code, const char* why) {
  stopNow();
  s_fault = code;
  setState(ST_FAULT);
  liftSay("FAULT %u: %s. Fix it, then send 'l h'.", code, why);
}

static void beginSeek(float speed, float maxMm) {
  s_seekStart = s_pos;
  s_seekLimit = mmToSteps(maxMm);
  seekDown(speed);
}

static void startHoming() {
  stopNow();
  s_homed = false;
  s_fault = 0;
  setSoftMax(false, 0);
  takeFlag(s_bottomHit); takeFlag(s_topHit);
  setState(ST_HOMING);
  if (s_bottom) {                          // already on the switch: lift off first
    s_phaseHome = PH_BACKOFF_FIRST;
    moveTo(s_pos + mmToSteps(LIFT_BACKOFF_MM), LIFT_HOME_FAST_MM_S);
  } else {
    s_phaseHome = PH_FAST_DOWN;
    beginSeek(LIFT_HOME_FAST_MM_S, LIFT_TRAVEL_MM + 50.0f);
  }
  liftSay("homing - moving DOWN to the switch");
}

static void startParking() {
  if (!s_homed || s_state == ST_FAULT) { startHoming(); return; }
  if (s_state == ST_IDLE && s_bottom && s_pos == 0) { liftSay("already parked"); s_dirty = true; return; }
  takeFlag(s_bottomHit);
  s_fault = 0;
  setState(ST_PARKING);
  s_phaseHome = PH_APPROACH;
  int32_t approach = mmToSteps(LIFT_PARK_APPROACH_MM);
  if (s_pos > approach) moveTo(approach, LIFT_MAX_SPEED_MM_S);
  else stopNow();
  liftSay("parking - lowering onto the switch");
}

static void finishAtBottom() {
  int32_t posAtHit = s_pos;
  bool wasHomed = s_homed;
  setPosition(0);
  setSoftMax(true, mmToSteps(LIFT_TRAVEL_MM));
  s_homed = true;
  s_fault = 0;
  if (wasHomed) {
    s_lastCorrectionMm = stepsToMm(posAtHit);
    liftSay("parked. position error corrected: %.1f mm", s_lastCorrectionMm);
  } else {
    liftSay("homed. switch = 0 mm");
  }
  if (LIFT_DISABLE_WHEN_PARKED) enableDriver(false);
  setState(ST_IDLE);
}

static void updateHomingParking() {
  const bool hit = takeFlag(s_bottomHit);
  const int32_t pos = s_pos;
  switch (s_phaseHome) {
    case PH_BACKOFF_FIRST:
      if (!moving()) {
        if (s_bottom) { fault(2, "switch still PRESSED after moving up (check NC wiring / broken wire)"); return; }
        s_phaseHome = PH_FAST_DOWN;
        beginSeek(LIFT_HOME_FAST_MM_S, LIFT_TRAVEL_MM + 50.0f);
      }
      break;
    case PH_FAST_DOWN:
      if (hit) {
        s_phaseHome = PH_BACKOFF;
        moveTo(pos + mmToSteps(LIFT_BACKOFF_MM), LIFT_HOME_FAST_MM_S);
      } else if (s_seekStart - pos > s_seekLimit) {
        fault(1, "switch never triggered (wiring? or lift moving UP while homing -> flip LIFT_DIR_HIGH_MEANS_UP)");
      }
      break;
    case PH_BACKOFF:
      if (!moving()) {
        if (s_bottom) { fault(2, "switch still PRESSED after moving up (check NC wiring / broken wire)"); return; }
        s_phaseHome = PH_SLOW_DOWN;
        beginSeek(LIFT_HOME_SLOW_MM_S, LIFT_BACKOFF_MM + 10.0f);
      }
      break;
    case PH_APPROACH:
      if (hit) { finishAtBottom(); return; }   // switch came early: steps were lost lifting
      if (!moving()) {
        s_phaseHome = PH_SLOW_DOWN;
        beginSeek(LIFT_HOME_SLOW_MM_S, LIFT_PARK_APPROACH_MM + 15.0f);
      }
      break;
    case PH_SLOW_DOWN:
      if (hit) { finishAtBottom(); return; }
      if (s_seekStart - pos > s_seekLimit) {
        if (s_state == ST_PARKING) { liftSay("switch not where expected - full homing run"); startHoming(); }
        else fault(3, "switch not found on the slow approach");
      }
      break;
  }
}

static void printStatus() {
  char buf[72];
  snprintf(buf, sizeof(buf), "l %u %u %.1f %.1f %u %u",
           (unsigned)s_state, s_homed ? 1u : 0u, stepsToMm(s_pos), stepsToMm(s_target),
           s_bottom ? 1u : 0u, (unsigned)s_fault);
  LIFT_PRINTLN(buf);
}

}  // namespace lift

// =============================================================================
//  The five functions your firmware calls
// =============================================================================

// In setup(), after Serial.begin():
inline void liftBegin() {
  using namespace lift;
  critical_section_init(&s_cs);
  gpio_init(PIN_LIFT_EN);   gpio_put(PIN_LIFT_EN, 1);   gpio_set_dir(PIN_LIFT_EN, GPIO_OUT);   // disabled
  gpio_init(PIN_LIFT_STEP); gpio_put(PIN_LIFT_STEP, 0); gpio_set_dir(PIN_LIFT_STEP, GPIO_OUT);
  gpio_init(PIN_LIFT_DIR);  gpio_set_dir(PIN_LIFT_DIR, GPIO_OUT);
  setDirPin(-1);
  s_dir = -1;
  pinMode(PIN_LIFT_HOME, INPUT_PULLUP);
  delay(5);
  s_bottom = switchRaw();
  s_enabled = false;
  add_repeating_timer_us(-(1000000 / TICK_HZ), onTick, nullptr, &s_timer);
  s_started = true;
}

// E-stop: call this from driversOffNow() so the lift stops in the same instant.
inline void liftEmergencyStop() {
  if (!lift::s_started) return;
  lift::stopNow();
  lift::enableDriver(false);
}

// Every pass of loop(). 'armed' = (armState == ARMED).
inline void liftUpdate(bool armed) {
  using namespace lift;
  if (!s_started) return;
  const uint32_t now = millis();

  if (!armed) {
    if (s_wasArmed || moving()) {
      stopNow();
      enableDriver(false);
      if (s_homed || s_state == ST_HOMING || s_state == ST_PARKING || s_state == ST_MOVING)
        liftSay("motor power off - lift position forgotten, send 'l h' after arming");
      s_homed = false;
      setSoftMax(false, 0);
      if (s_state != ST_FAULT) setState(ST_IDLE);
      s_dirty = true;
    }
    s_wasArmed = false;
  } else {
    s_wasArmed = true;
  }

  switch (s_state) {
    case ST_HOMING:
    case ST_PARKING:
      updateHomingParking();
      break;
    case ST_MOVING:
      if (takeFlag(s_bottomHit)) {
        setPosition(0);
        liftSay("WARNING: switch hit unexpectedly - position re-zeroed");
        setState(ST_IDLE);
      } else if (takeFlag(s_topHit)) {
        liftSay("WARNING: top limit (LIFT_TRAVEL_MM) reached - stopped");
        setState(ST_IDLE);
      } else if (!moving()) {
        setState(ST_IDLE);
      }
      break;
    default:
      takeFlag(s_bottomHit);
      takeFlag(s_topHit);
      break;
  }

  if (s_dirty || now - s_lastStatusMs >= LIFT_STATUS_MS) {
    s_dirty = false;
    s_lastStatusMs = now;
    printStatus();
  }
}

// From your serial parser when a line starts with 'l'. Pass the text AFTER the 'l'.
inline void liftPrintInfo();
inline void liftCommand(const char* args, bool armed) {
  using namespace lift;
  if (!s_started) return;
  while (*args == ' ') args++;
  const char c = *args ? *args++ : '?';

  if (c == '?') { liftPrintInfo(); return; }
  if (c == 's') {
    stopNow();
    if (s_state != ST_FAULT) setState(ST_IDLE);
    s_dirty = true;
    liftSay("stopped");
    return;
  }
  if (!armed) { liftSay("not armed - send 'a 1' first (the lift is powered through the relay)"); return; }

  if (c == 'h') { startHoming(); return; }
  if (c == 'p') { startParking(); return; }
  if (c == 'g') {
    char* end = nullptr;
    float mm = strtof(args, &end);
    if (end == args) { liftSay("usage: l g <mm> [speed_mm_s]"); return; }
    float speed = strtof(end, &end);
    if (speed <= 0.0f) speed = LIFT_DEFAULT_SPEED_MM_S;
    if (s_state == ST_FAULT)  { liftSay("in FAULT - send 'l h' first"); return; }
    if (!s_homed)             { liftSay("not homed - send 'l h' first"); return; }
    if (s_state == ST_HOMING || s_state == ST_PARKING) { liftSay("busy homing/parking - send 'l s' first"); return; }
    if (mm < LIFT_SOFT_MIN_MM) mm = LIFT_SOFT_MIN_MM;
    if (mm > LIFT_SOFT_MAX_MM) mm = LIFT_SOFT_MAX_MM;
    takeFlag(s_bottomHit); takeFlag(s_topHit);
    moveTo(mmToSteps(mm), speed);
    setState(ST_MOVING);
    s_dirty = true;
    return;
  }
  liftSay("unknown. Use: l h | l p | l g <mm> [speed] | l s | l ?");
}

// Also call from your 'i' command if you like.
inline void liftPrintInfo() {
  using namespace lift;
  liftSay("%.1f steps/mm, travel %.0f mm, soft limits %.0f-%.0f mm, max %.0f mm/s",
      STEPS_PER_MM, LIFT_TRAVEL_MM, LIFT_SOFT_MIN_MM, LIFT_SOFT_MAX_MM, LIFT_MAX_SPEED_MM_S);
  bool level = gpio_get(PIN_LIFT_HOME);
  liftSay("switch GP%d reads %s -> %s (%s wiring)", PIN_LIFT_HOME, level ? "HIGH" : "LOW",
      switchRaw() ? "PRESSED" : "released", LIFT_SWITCH_NC ? "NC" : "NO");
  liftSay("homed=%d pos=%.1f mm driver=%s last park correction=%.1f mm",
      s_homed ? 1 : 0, stepsToMm(s_pos), s_enabled ? "ON" : "off", s_lastCorrectionMm);
}
