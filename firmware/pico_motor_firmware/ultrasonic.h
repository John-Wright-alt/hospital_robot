#pragma once
// =============================================================================
//  ultrasonic.h  -  3x HC-SR04 low-obstacle sensors for pico_motor_firmware
//                   (Phase 7, firmware 1.2.0)
//
//  Wiring = Robot_Wiring.kicad_sch rev 2 (28/09/2026), "ULTRASONIC SENSORS
//  (5 V, Rail B)" block. Read from the schematic, NOT yet checked on the robot:
//
//     sensor            schematic ref   TRIG (Pico)   ECHO (Pico, via 1k/2k divider)
//     US1 "left"        US1             GP16          GP17
//     US2 "front-centre" US2            GP20          GP21
//     US3 "right"       US3             GP22          GP26
//
//  TRIG is driven straight from the Pico at 3.3 V (the HC-SR04 accepts it).
//  ECHO is a 5 V signal; the schematic shows a 1 k / 2 k divider on each one,
//  so the Pico pin sees ~3.33 V. The Pico pins are NOT 5 V tolerant: the
//  Phase 7 guide checks those dividers with a meter before this code runs.
//
//  How it works (nothing here can move the robot):
//    * One sensor at a time, round-robin, a new ping every US_PERIOD_MS
//      (60 ms - the schematic note "fire round-robin >= 60 ms apart"), so the
//      sensors cannot hear each other's echoes. Each sensor: ~5.5 readings/s.
//    * The echo pulse is timed by a GPIO interrupt on CORE 1 (this file is
//      started from setup1()). Core 0's FOC loops and the lift's 20 kHz step
//      interrupt are not touched. No pulseIn(), nothing blocks for more
//      than the 10 us trigger pulse.
//    * Every finished ping prints one line, Pico -> host:
//         u <n> <mm>      n = 1 left, 2 front, 3 right
//                         mm = distance in millimetres
//                         mm = -1  no echo within US_TIMEOUT_US (nothing in range,
//                                  or the sensor is unplugged)
//                         mm = -2  ECHO was still HIGH when it was time to ping
//                                  (sensor stuck / wiring fault)
//    * Host -> Pico:   u 1  start    u 0  stop    u ?  info
//
//  Older pico_bridge versions ignore 'u' lines (parse_line returns None), so
//  this firmware is safe to run with the old bridge too.
// =============================================================================
#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

// ------------------------------- SETTINGS -----------------------------------
// TRIG pins come from config.h (PIN_US1_TRIG / PIN_US2_TRIG / PIN_US3_TRIG).
#ifndef PIN_US1_ECHO
#define PIN_US1_ECHO   17     // schematic net US1_ECHO -> Pico pin 22 (GP17)
#endif
#ifndef PIN_US2_ECHO
#define PIN_US2_ECHO   21     // schematic net US2_ECHO -> Pico pin 27 (GP21)
#endif
#ifndef PIN_US3_ECHO
#define PIN_US3_ECHO   26     // schematic net US3_ECHO -> Pico pin 31 (GP26)
#endif

#ifndef US_ENABLED_AT_BOOT
#define US_ENABLED_AT_BOOT  1     // 0 = stay quiet until the host sends 'u 1'
#endif
#ifndef US_PERIOD_MS
#define US_PERIOD_MS        60    // one ping every 60 ms, sensors in turn
#endif
#ifndef US_TIMEOUT_US
#define US_TIMEOUT_US       30000 // ~5 m of sound travel; HC-SR04 is rated to 4 m
#endif
#ifndef US_MAX_MM
#define US_MAX_MM           4000  // anything longer is reported as -1 (no echo)
#endif
#ifndef US_MIN_MM
#define US_MIN_MM           20    // HC-SR04 blind zone; shorter -> reported as US_MIN_MM
#endif
// speed of sound 343 m/s at 20 C -> 0.1715 mm per microsecond of echo (there and back)
#ifndef US_MM_PER_US
#define US_MM_PER_US        0.1715f
#endif
// ----------------------------------------------------------------------------

static void say(const char* fmt, ...);    // defined in pico_motor_firmware.ino

namespace us {

struct Channel {
  uint8_t trig, echo;
  volatile uint32_t rise, fall;
  volatile bool gotRise, gotFall;
};

static Channel ch[3] = {
  {PIN_US1_TRIG, PIN_US1_ECHO, 0, 0, false, false},
  {PIN_US2_TRIG, PIN_US2_ECHO, 0, 0, false, false},
  {PIN_US3_TRIG, PIN_US3_ECHO, 0, 0, false, false},
};
static const char* NAMES[3] = {"left", "front", "right"};

static volatile int8_t s_active = -1;     // which sensor the interrupt listens to
static int8_t   s_cur = -1;               // ping in progress (-1 = none)
static uint8_t  s_next = 0;
static uint32_t s_trigUs = 0, s_lastTrigMs = 0;
static bool     s_started = false;
static bool     s_enabled = US_ENABLED_AT_BOOT;
static uint32_t s_pings[3] = {0, 0, 0}, s_noEcho[3] = {0, 0, 0}, s_stuck[3] = {0, 0, 0};
static int32_t  s_last[3] = {-1, -1, -1};

template <int N>
static void echoIsr() {
  const uint32_t t = time_us_32();
  if (s_active != N) return;
  Channel& c = ch[N];
  if (gpio_get(c.echo)) {                 // rising edge: echo pulse starts
    c.rise = t;
    c.gotRise = true;
  } else if (c.gotRise && !c.gotFall) {   // falling edge: echo pulse ends
    c.fall = t;
    c.gotFall = true;
  }
}

static void finish(uint32_t nowUs) {
  Channel& c = ch[s_cur];
  int32_t mm = -1;
  if (c.gotFall) {
    const uint32_t width = c.fall - c.rise;
    mm = (int32_t)(width * US_MM_PER_US + 0.5f);
    if (mm > US_MAX_MM) mm = -1;
    else if (mm < US_MIN_MM) mm = US_MIN_MM;
  }
  (void)nowUs;
  s_active = -1;
  s_pings[s_cur]++;
  if (mm < 0) s_noEcho[s_cur]++;
  s_last[s_cur] = mm;
  say("u %d %ld", s_cur + 1, (long)mm);
  s_cur = -1;
}

}  // namespace us

// =============================================================================
//  The four functions the firmware calls (all from CORE 1)
// =============================================================================

// In setup1(): attaches the echo interrupts to core 1.
inline void usBegin() {
  using namespace us;
  for (int i = 0; i < 3; i++) {
    pinMode(ch[i].trig, OUTPUT);
    digitalWrite(ch[i].trig, LOW);
    pinMode(ch[i].echo, INPUT);           // the 2k of the divider is the pull-down
  }
  attachInterrupt(digitalPinToInterrupt(PIN_US1_ECHO), echoIsr<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_US2_ECHO), echoIsr<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_US3_ECHO), echoIsr<2>, CHANGE);
  s_started = true;
}

// Every pass of loop1(). Never blocks for more than the 10 us trigger pulse.
inline void usUpdate(uint32_t nowMs) {
  using namespace us;
  if (!s_started) return;

  if (s_cur >= 0) {
    const uint32_t nowUs = time_us_32();
    if (ch[s_cur].gotFall || nowUs - s_trigUs > US_TIMEOUT_US) finish(nowUs);
  }
  if (!s_enabled || s_cur >= 0 || nowMs - s_lastTrigMs < US_PERIOD_MS) return;

  const int n = s_next;
  s_next = (s_next + 1) % 3;
  s_lastTrigMs = nowMs;
  Channel& c = ch[n];
  if (gpio_get(c.echo)) {                 // still high from an earlier ping: stuck
    s_stuck[n]++;
    say("u %d -2", n + 1);
    return;
  }
  c.gotRise = false;
  c.gotFall = false;
  s_active = n;
  digitalWrite(c.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(c.trig, LOW);
  s_trigUs = time_us_32();
  s_cur = n;
}

inline void usPrintInfo() {
  using namespace us;
  say("# ultrasonic: %s, one ping every %d ms, timeout %d us",
      s_enabled ? "ON" : "off", US_PERIOD_MS, US_TIMEOUT_US);
  for (int i = 0; i < 3; i++) {
    say("#   US%d %-5s TRIG GP%d ECHO GP%d (now %s)  last=%ld mm  pings=%lu no_echo=%lu stuck=%lu",
        i + 1, NAMES[i], ch[i].trig, ch[i].echo, gpio_get(ch[i].echo) ? "HIGH" : "low",
        (long)s_last[i], (unsigned long)s_pings[i], (unsigned long)s_noEcho[i],
        (unsigned long)s_stuck[i]);
  }
}

// From the serial parser when a line starts with 'u'. Pass the text AFTER the 'u'.
inline void usCommand(const char* args) {
  using namespace us;
  while (*args == ' ') args++;
  if (*args == '1')      { s_enabled = true;  say("# ultrasonic ON"); }
  else if (*args == '0') { s_enabled = false; say("# ultrasonic off"); }
  else                   usPrintInfo();
}
