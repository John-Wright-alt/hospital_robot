#pragma once
// =====================================================================
//  motion.h  -  moves the stepper motor.
//
//  A hardware timer interrupts the processor 20 000 times per second.
//  Every interrupt it:
//    1. reads (and de-bounces) the limit switch(es)
//    2. speeds the motor up / slows it down smoothly (trapezoid profile)
//    3. decides whether it is time to send the next STEP pulse
//  Because this runs on a timer, the motor moves smoothly even while the
//  face detection is busy doing heavy maths on the other processor core.
//
//  SAFETY: any movement DOWN is stopped instantly when the bottom switch
//  is pressed, and any movement UP is stopped at the top switch (if
//  fitted) or at COLUMN_TRAVEL_MM once the column has been homed.
//
//  Units: position is counted in motor steps. + is UP, 0 is the switch.
// =====================================================================
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "config.h"

namespace motion {

constexpr int32_t TICK_HZ        = 20000;   // interrupt rate (every 50 us)
constexpr int32_t MAX_STEP_RATE  = 8000;    // never ask for more steps/second than this
constexpr int32_t DEBOUNCE_TICKS = 40;      // switch must be stable for 2 ms
constexpr float   STEPS_PER_MM   = (float)MOTOR_STEPS_PER_REV * MICROSTEPS / MM_PER_REV;
constexpr uint32_t PHASE_FULL    = (uint32_t)TICK_HZ * 256u;

enum Mode : uint8_t { MODE_IDLE = 0, MODE_POSITION, MODE_SEEK_DOWN, MODE_DECEL };

// ---- values shared between the interrupt and the main program ----
static volatile int32_t  s_pos        = 0;          // current position (steps)
static volatile int32_t  s_target     = 0;          // MODE_POSITION goal (steps)
static volatile int32_t  s_v          = 0;          // velocity: steps/s * 65536, + = up
static volatile int32_t  s_vmax       = 0;          // max speed: steps/s * 65536
static volatile int32_t  s_accelTick  = 1;          // velocity change per tick
static volatile int32_t  s_accel      = 1;          // steps/s^2
static volatile int32_t  s_vsnap      = 100;        // "slow enough to just stop" (steps/s)
static volatile uint8_t  s_mode       = MODE_IDLE;
static volatile int8_t   s_dir        = -1;         // direction the DIR pin is set to
static volatile bool     s_stepHigh   = false;
static volatile uint32_t s_phase      = 0;
static volatile bool     s_bottom     = false;      // de-bounced switch states
static volatile bool     s_top        = false;
static volatile uint16_t s_bottomCnt  = 0;
static volatile uint16_t s_topCnt     = 0;
static volatile bool     s_bottomHit  = false;      // latched: stopped by bottom switch
static volatile bool     s_topHit     = false;      // latched: stopped by top switch/limit
static volatile bool     s_softMaxOn  = false;
static volatile int32_t  s_softMax    = INT32_MAX;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static hw_timer_t*  s_timer   = nullptr;
static bool         s_started = false;
static bool         s_enabled = false;

// ---- very fast pin access (safe to use inside the interrupt) ----
static inline void IRAM_ATTR pinHigh(int pin) {
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << pin);
  else          REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (pin - 32));
}
static inline void IRAM_ATTR pinLow(int pin) {
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << pin);
  else          REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
}
static inline bool IRAM_ATTR pinRead(int pin) {
  if (pin < 32) return (REG_READ(GPIO_IN_REG) >> pin) & 1UL;
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1UL;
}

// true when the switch is PRESSED (before de-bouncing)
static inline bool IRAM_ATTR switchRaw(int pin) {
#if LIMIT_SWITCH_NC
  return pinRead(pin);     // NC contact opens when pressed -> pull-up reads HIGH
#else
  return !pinRead(pin);    // NO contact closes to GND when pressed -> reads LOW
#endif
}

static inline void IRAM_ATTR debounce(int pin, volatile bool& state, volatile uint16_t& cnt) {
  bool raw = switchRaw(pin);
  if (raw != state) {
    if (++cnt >= DEBOUNCE_TICKS) { state = raw; cnt = 0; }
  } else {
    cnt = 0;
  }
}

static inline void IRAM_ATTR setDirPin(int8_t dir) {
  bool high = (dir > 0) ? (DIR_HIGH_MEANS_UP != 0) : (DIR_HIGH_MEANS_UP == 0);
  if (high) pinHigh(PIN_DIR); else pinLow(PIN_DIR);
}

// ---- the interrupt itself ----
static inline void IRAM_ATTR tickBody() {
  bool justLowered = false;
  if (s_stepHigh) { pinLow(PIN_STEP); s_stepHigh = false; justLowered = true; }

  debounce(PIN_LIMIT_BOTTOM, s_bottom, s_bottomCnt);
  if (PIN_LIMIT_TOP >= 0) debounce(PIN_LIMIT_TOP, s_top, s_topCnt);

  const uint8_t mode = s_mode;
  if (mode == MODE_IDLE) { s_v = 0; return; }

  int32_t v    = s_v;
  int32_t vInt = v / 65536;              // current speed in steps/s (signed)
  int32_t desired = 0;

  if (mode == MODE_POSITION) {
    const int32_t d = s_target - s_pos;
    if (d == 0) {
      int32_t av = vInt < 0 ? -vInt : vInt;
      if (av <= s_vsnap) { s_v = 0; s_phase = 0; s_mode = MODE_IDLE; return; }  // arrived
      desired = 0;
    } else {
      const int8_t toward = d > 0 ? 1 : -1;
      const int32_t ad = d > 0 ? d : -d;
      const int32_t stopDist = (vInt * vInt) / (2 * s_accel);
      const bool sameWay = (vInt == 0) || ((vInt > 0) == (toward > 0));
      if (sameWay && ad > stopDist) desired = toward * s_vmax;
      else                          desired = 0;           // brake (and reverse later)
    }
  } else if (mode == MODE_SEEK_DOWN) {
    desired = -s_vmax;
  } else {  // MODE_DECEL
    desired = 0;
    if (vInt == 0) { s_v = 0; s_phase = 0; s_mode = MODE_IDLE; return; }
  }

  // accelerate / decelerate toward the desired velocity
  const int32_t a = s_accelTick;
  if (v < desired)      { v += a; if (v > desired) v = desired; }
  else if (v > desired) { v -= a; if (v < desired) v = desired; }
  s_v = v;

  const int8_t want = v > 0 ? 1 : (v < 0 ? -1 : 0);
  if (want == 0) return;

  if (want != s_dir) {            // change direction, step on a later tick
    setDirPin(want);
    s_dir = want;
    s_phase = 0;
    return;
  }

  // ---- hard safety limits ----
  if (want < 0 && s_bottom) {
    s_v = 0; s_phase = 0; s_mode = MODE_IDLE; s_bottomHit = true; return;
  }
  if (want > 0 && (s_top || (s_softMaxOn && s_pos >= s_softMax))) {
    s_v = 0; s_phase = 0; s_mode = MODE_IDLE; s_topHit = true; return;
  }

  // ---- time for a step? ----
  const uint32_t speed256 = (uint32_t)((v < 0 ? -v : v) >> 8);   // steps/s * 256
  uint32_t ph = s_phase + speed256;
  if (ph >= PHASE_FULL && !justLowered) {
    ph -= PHASE_FULL;
    pinHigh(PIN_STEP);
    s_stepHigh = true;
    s_pos = s_pos + want;
  }
  if (ph > 2 * PHASE_FULL) ph = PHASE_FULL;
  s_phase = ph;
}

static void IRAM_ATTR onTick() {
  portENTER_CRITICAL_ISR(&s_mux);
  tickBody();
  portEXIT_CRITICAL_ISR(&s_mux);
}

// =====================================================================
//  Functions used by the rest of the program
// =====================================================================
inline int32_t mmToSteps(float mm)  { return (int32_t)lroundf(mm * STEPS_PER_MM); }
inline float   stepsToMm(int32_t s) { return (float)s / STEPS_PER_MM; }

inline void enableDriver(bool on) {
  s_enabled = on;
  if (PIN_ENABLE < 0) return;
  bool levelHigh = ENABLE_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_ENABLE, levelHigh ? HIGH : LOW);
}
inline bool driverEnabled() { return s_enabled; }

inline void begin() {
  pinMode(PIN_STEP, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR, OUTPUT);
  setDirPin(-1);
  s_dir = -1;
  if (PIN_ENABLE >= 0) pinMode(PIN_ENABLE, OUTPUT);
  enableDriver(false);
  pinMode(PIN_LIMIT_BOTTOM, INPUT_PULLUP);
  if (PIN_LIMIT_TOP >= 0) pinMode(PIN_LIMIT_TOP, INPUT_PULLUP);
  delay(5);
  s_bottom = switchRaw(PIN_LIMIT_BOTTOM);
  if (PIN_LIMIT_TOP >= 0) s_top = switchRaw(PIN_LIMIT_TOP);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  s_timer = timerBegin(1000000);                       // 1 MHz timer clock
  timerAttachInterrupt(s_timer, &onTick);
  timerAlarm(s_timer, 1000000 / TICK_HZ, true, 0);
#else
  s_timer = timerBegin(0, 80, true);                   // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(s_timer, &onTick, false);
  timerAlarmWrite(s_timer, 1000000 / TICK_HZ, true);
  timerAlarmEnable(s_timer);
#endif
  s_started = true;
}

inline bool started() { return s_started; }

// Set speed (mm/s) and acceleration used by the next move
inline void setProfile(float speedMmS) {
  if (speedMmS > MAX_SPEED_MM_S) speedMmS = MAX_SPEED_MM_S;
  if (speedMmS < 0.5f) speedMmS = 0.5f;
  float sps = speedMmS * STEPS_PER_MM;
  if (sps > MAX_STEP_RATE) sps = MAX_STEP_RATE;
  float acc = ACCEL_MM_S2 * STEPS_PER_MM;               // steps/s^2
  if (acc < 50.0f) acc = 50.0f;
  int32_t accelTick = (int32_t)(acc * 65536.0f / TICK_HZ);
  if (accelTick < 1) accelTick = 1;
  int32_t vsnap = (int32_t)sqrtf(4.0f * acc);
  if (vsnap < 20) vsnap = 20;
  portENTER_CRITICAL(&s_mux);
  s_vmax = (int32_t)(sps * 65536.0f);
  s_accel = (int32_t)acc;
  s_accelTick = accelTick;
  s_vsnap = vsnap;
  portEXIT_CRITICAL(&s_mux);
}

inline void ensureEnabled() {
  if (!s_enabled) { enableDriver(true); delay(3); }
}

// Go to an absolute position (steps). Can be called again while moving:
// the motor smoothly changes to the new target.
inline void moveTo(int32_t targetSteps, float speedMmS) {
  if (!s_started) return;
  setProfile(speedMmS);
  ensureEnabled();
  portENTER_CRITICAL(&s_mux);
  s_target = targetSteps;
  s_mode = MODE_POSITION;
  portEXIT_CRITICAL(&s_mux);
}

// Move down at a constant speed until the bottom switch is pressed.
inline void seekDown(float speedMmS) {
  if (!s_started) return;
  setProfile(speedMmS);
  ensureEnabled();
  portENTER_CRITICAL(&s_mux);
  s_bottomHit = false;
  s_mode = MODE_SEEK_DOWN;
  portEXIT_CRITICAL(&s_mux);
}

// Slow down smoothly and stop
inline void stop() {
  portENTER_CRITICAL(&s_mux);
  if (s_mode != MODE_IDLE) s_mode = MODE_DECEL;
  portEXIT_CRITICAL(&s_mux);
}

// Stop immediately (used for STOP command and faults)
inline void emergencyStop() {
  portENTER_CRITICAL(&s_mux);
  s_mode = MODE_IDLE;
  s_v = 0;
  s_phase = 0;
  portEXIT_CRITICAL(&s_mux);
}

inline bool isMoving() { return s_mode != MODE_IDLE; }
inline int32_t position() { return s_pos; }
inline int32_t target() { return s_target; }
inline float velocityMmS() { return stepsToMm(s_v / 65536); }

inline void setPosition(int32_t p) {
  portENTER_CRITICAL(&s_mux);
  s_pos = p;
  s_target = p;
  portEXIT_CRITICAL(&s_mux);
}

inline void setSoftMax(bool on, int32_t maxSteps) {
  portENTER_CRITICAL(&s_mux);
  s_softMaxOn = on;
  s_softMax = maxSteps;
  portEXIT_CRITICAL(&s_mux);
}

inline bool bottomPressed() { return s_bottom; }
inline bool topPressed() { return s_top; }
inline bool bottomRawPressed() { return switchRaw(PIN_LIMIT_BOTTOM); }
inline int  bottomPinLevel() { return digitalRead(PIN_LIMIT_BOTTOM); }

inline bool takeBottomHit() {
  portENTER_CRITICAL(&s_mux);
  bool r = s_bottomHit; s_bottomHit = false;
  portEXIT_CRITICAL(&s_mux);
  return r;
}
inline bool takeTopHit() {
  portENTER_CRITICAL(&s_mux);
  bool r = s_topHit; s_topHit = false;
  portEXIT_CRITICAL(&s_mux);
  return r;
}

}  // namespace motion
