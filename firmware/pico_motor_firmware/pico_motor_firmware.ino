// =============================================================================
//  pico_motor_firmware.ino
//  Hospital Robot — Raspberry Pi Pico W drive-base firmware (Phase 4)
//  1.0.1 (Phase 6): E-stop glitch filter + VMOT-lost debounce. Search "1.0.1".
//  1.1.0 (Phase 8): head-column lift (lift.h) on GP10-GP13. Search "1.1.0".
//
//  Arduino IDE settings:  Board = "Raspberry Pi Pico W"  (arduino-pico core)
//  Library:               Simple FOC 2.4.x
//  Edit config.h (the other tab) — you should not need to change this file.
//
//  WHAT RUNS WHERE
//    Core 0 : both motor FOC loops, as fast as possible. Nothing else.
//    Core 1 : USB serial protocol, ramping, command watchdog, E-stop, relay,
//             battery monitoring, stall/encoder fault detection.
//
//  SERIAL PROTOCOL — plain text, one message per line ('\n').
//  Any baud rate works EXCEPT 1200 (1200 = "reboot into upload mode").
//
//   Host -> Pico
//     v <left> <right>   wheel speeds, rad/s, + = forward. Resend at 20–50 Hz.
//                        If none arrive for CMD_TIMEOUT_MS the robot ramps to 0.
//     a 1  |  a 0        arm (motor power ON)  |  disarm (motor power OFF)
//     c                  clear latched faults (only if the cause has gone away)
//     k <P> <I> <D>      set velocity PID gains on both motors (not saved)
//     i                  print settings, loop rate and faults
//     h                  help
//     l h | l p          LIFT: home | park (down onto the switch, re-zero)     1.1.0
//     l g <mm> [speed]   LIFT: go to <mm> above the switch (speed in mm/s)
//     l s | l ?          LIFT: stop | print lift settings and switch reading
//                        The lift only moves while ARMED (its power comes
//                        through the E-stop and relay, like the wheels).
//
//   Pico -> Host
//     e <ticksL> <ticksR> <velL> <velR> <micros>    every 20 ms (50 Hz)
//          ticks = motor encoder counts, + = forward (GEAR_RATIO x 4096 per wheel turn)
//          vel   = wheel speed in rad/s, + = forward
//     s <flags_hex> <arm_state> <vbat_V> <vmot_V>   on change + every 500 ms
//          arm_state: 0 DISARMED, 1 ARMING, 2 ARMED, 3 DISARMING
//     l <state> <homed> <pos_mm> <target_mm> <switch> <fault>   10 Hz    1.1.0
//          state: 0 IDLE, 1 MOVING, 2 HOMING, 3 PARKING, 4 FAULT
//     # <text>                                      human-readable message
//
//  SAFETY LAYERS (defence in depth — none depends on the one above it)
//    1. Hardware E-stop cuts motor power (wiring, not software).
//    2. This firmware: speed clamp, accel ramp, 500 ms command watchdog,
//       stall detection, encoder check, low-battery cutoff, relay control.
//    3. RP2040 hardware watchdog: if either core hangs, the chip reboots and
//       the 10k pull-downs switch both motor drivers off.
//    For the lift (1.1.0): its driver is disabled by a 10k pull-up on EN, its
//    home switch is checked inside the step interrupt every 50 us, and losing
//    motor power makes it forget its position so it must home again.
// =============================================================================

#include <SimpleFOC.h>
#include <hardware/watchdog.h>
#include <stdarg.h>
#include "config.h"

// ---- 1.1.0: head-column lift ------------------------------------------------
// lift.h prints through this firmware's queued say(), so lift lines can never
// be cut into the middle of an 'e' or 's' line. say() is defined further down,
// so it is declared here first.
static void say(const char* fmt, ...);
#define LIFT_PRINTLN(text) say("%s", text)
#include "lift.h"                   // adds liftBegin/liftUpdate/liftCommand/...
#include "ultrasonic.h"             // 1.2.0: 3x HC-SR04 low-obstacle sensors

bool core1_separate_stack = true;   // give core 1 its own 8 KB stack

// ----------------------------------------------------------------------------
// Status flags. "Live" flags clear themselves when the cause goes away.
// "Latched" faults (upper byte) stay set until you send 'c'.
// ----------------------------------------------------------------------------
enum : uint16_t {
  F_ESTOP       = 0x0001,  // live:    E-stop button is pressed
  F_NO_VMOT     = 0x0002,  // live:    no motor power (relay open / blue switch / E-stop)
  F_CMD_TIMEOUT = 0x0004,  // live:    no 'v' command recently -> wheels held at 0
  F_BATT_WARN   = 0x0008,  // live:    battery below VBAT_WARN_V
  F_NO_BATT     = 0x0010,  // live:    no battery voltage at all (USB-only bench work)
  F_LOOP_SLOW   = 0x0020,  // live:    FOC loop slower than 1 kHz
  F_BATT_CUTOFF = 0x0100,  // latched: battery below VBAT_CUTOFF_V
  F_STALL_L     = 0x0200,  // latched: left wheel not following its command
  F_STALL_R     = 0x0400,  // latched: right wheel not following its command
  F_ENCODER_L   = 0x0800,  // latched: left encoder not answering on I2C
  F_ENCODER_R   = 0x1000,  // latched: right encoder not answering on I2C
  F_RELAY       = 0x2000,  // latched: relay did not open when told to
};
static const uint16_t LATCHED_MASK = 0xFF00;

enum ArmState : uint8_t { DISARMED = 0, ARMING = 1, ARMED = 2, DISARMING = 3 };
static const char* ARM_NAMES[] = {"DISARMED", "ARMING", "ARMED", "DISARMING"};

// ----------------------------------------------------------------------------
// Hardware objects (used by core 0 only, except where noted)
// ----------------------------------------------------------------------------
BLDCMotor         motorL(POLE_PAIRS);
BLDCMotor         motorR(POLE_PAIRS);
BLDCDriver3PWM    driverL(PIN_L_IN1, PIN_L_IN2, PIN_L_IN3, PIN_L_EN);
BLDCDriver3PWM    driverR(PIN_R_IN1, PIN_R_IN2, PIN_R_IN3, PIN_R_EN);
MagneticSensorI2C encL(AS5600_I2C);
MagneticSensorI2C encR(AS5600_I2C);

// ----------------------------------------------------------------------------
// Variables shared between the two cores. Each is 32 bits or smaller, and the
// RP2040 reads/writes those in one instruction, so no locks are needed.
// ----------------------------------------------------------------------------
volatile float    g_targetL = 0.0f, g_targetR = 0.0f;  // motor rad/s     core1 -> core0
volatile bool     g_enableReq = false;                  // drivers on?     core1 -> core0
volatile bool     g_enabled   = false;                  // actual state    core0 -> core1
volatile int32_t  g_ticksL = 0, g_ticksR = 0;           // forward ticks   core0 -> core1
volatile float    g_velL = 0.0f, g_velR = 0.0f;         // motor rad/s fwd core0 -> core1
volatile uint32_t g_encErrL = 0, g_encErrR = 0;         // consecutive I2C errors
volatile uint32_t g_loops = 0;                          // core 0 loop counter (heartbeat)
volatile bool     g_core0Ready = false;
volatile bool     g_encFoundL = false, g_encFoundR = false;

// =============================================================================
//                                   CORE 0
// =============================================================================

// Encoder position as a whole number of ticks. Uses SimpleFOC's integer
// full-rotation counter so it never loses precision, however far the robot drives.
static int32_t forwardTicks(MagneticSensorI2C& enc, BLDCMotor& m, int sign) {
  int32_t raw = enc.getFullRotations() * (int32_t)ENCODER_CPR
              + (int32_t)(enc.getMechanicalAngle() * (ENCODER_CPR / _2PI) + 0.5f);
  if (m.sensor_direction == Direction::CCW) raw = -raw;  // match the motor's + direction
  return sign * raw;                                     // then make + = robot forward
}

static bool encoderAnswers(TwoWire& bus) {
  bus.beginTransmission(0x36);          // AS5600 fixed address
  return bus.endTransmission() == 0;
}

static void setupMotor(BLDCMotor& m, BLDCDriver3PWM& d, MagneticSensorI2C& enc,
                       float zeroAngle, Direction dir) {
  d.voltage_power_supply = SUPPLY_VOLTAGE;
  d.voltage_limit        = SUPPLY_VOLTAGE;
  d.pwm_frequency        = PWM_FREQUENCY;
  d.init();
  m.linkDriver(&d);
  m.linkSensor(&enc);

  // SimpleFOC 2.4: phase_resistance/KV are only used in estimated_current mode.
  m.phase_resistance  = PHASE_RESISTANCE;
  m.KV_rating         = MOTOR_KV;
  m.torque_controller = TorqueControlType::estimated_current;
  m.controller        = MotionControlType::velocity;
  m.foc_modulation    = FOCModulationType::SpaceVectorPWM;
  m.current_limit     = CURRENT_LIMIT;
  m.voltage_limit     = VOLTAGE_LIMIT;
  m.velocity_limit    = MAX_WHEEL_SPEED * GEAR_RATIO * 1.2f;
  m.PID_velocity.P    = VEL_P;
  m.PID_velocity.I    = VEL_I;
  m.PID_velocity.D    = VEL_D;
  m.LPF_velocity.Tf   = VEL_LPF_TF;

  // Calibration from test06. Because both are set, initFOC() skips the
  // start-up "wiggle" — the wheels do not twitch at power-on.
  m.zero_electric_angle = zeroAngle;
  m.sensor_direction    = dir;

  m.init();      // NOTE: SimpleFOC switches the driver ON at the end of init()...
  m.initFOC();
  m.disable();   // ...so switch it straight back OFF. Nothing moves until armed.
}

void setup() {
  // 1) Put every output into its SAFE state before anything else happens.
  pinMode(PIN_L_EN, OUTPUT);        digitalWrite(PIN_L_EN, LOW);        // driver off
  pinMode(PIN_R_EN, OUTPUT);        digitalWrite(PIN_R_EN, LOW);        // driver off
  pinMode(PIN_RELAY_TRIG, OUTPUT);  digitalWrite(PIN_RELAY_TRIG, LOW);  // no pulse
  pinMode(PIN_LIFT_EN, OUTPUT);     digitalWrite(PIN_LIFT_EN, HIGH);    // lift driver off
  pinMode(PIN_LIFT_STEP, OUTPUT);   digitalWrite(PIN_LIFT_STEP, LOW);
  pinMode(PIN_US1_TRIG, OUTPUT);    digitalWrite(PIN_US1_TRIG, LOW);
  pinMode(PIN_US2_TRIG, OUTPUT);    digitalWrite(PIN_US2_TRIG, LOW);
  pinMode(PIN_US3_TRIG, OUTPUT);    digitalWrite(PIN_US3_TRIG, LOW);
  pinMode(PIN_ESTOP_SENSE, INPUT);  // the external 10k to GND is the pull-down

  Serial.begin(115200);             // never wait for the host: the robot must boot alone

  // 1.1.0: start the lift. Called from CORE 0, so its 20 kHz step interrupt
  // runs on core 0 next to the FOC loops (about 5% of that core). liftUpdate()
  // and liftCommand() are called from core 1; lift.h guards everything they
  // share with a critical section, which is safe across both cores.
  liftBegin();

  // 2) Encoders: LEFT on I2C0 (Wire), RIGHT on I2C1 (Wire1).
  Wire.setSDA(PIN_ENC_L_SDA);   Wire.setSCL(PIN_ENC_L_SCL);
  Wire1.setSDA(PIN_ENC_R_SDA);  Wire1.setSCL(PIN_ENC_R_SCL);
  Wire.setClock(I2C_CLOCK_HZ);  Wire1.setClock(I2C_CLOCK_HZ);
  Wire.setTimeout(3, true);     Wire1.setTimeout(3, true);   // 3 ms, not the 1 s default
  encL.init(&Wire);
  encR.init(&Wire1);
  g_encFoundL = encoderAnswers(Wire);
  g_encFoundR = encoderAnswers(Wire1);

  // 3) Motors (~2 s — SimpleFOC waits for the drivers to settle).
  setupMotor(motorL, driverL, encL, LEFT_ZERO_ELEC_ANGLE,  LEFT_SENSOR_DIRECTION);
  setupMotor(motorR, driverR, encR, RIGHT_ZERO_ELEC_ANGLE, RIGHT_SENSOR_DIRECTION);

  g_core0Ready = true;              // core 1 starts its work once it sees this
}

void loop() {
  motorL.loopFOC();                 // reads the encoder, sets the 3 phase voltages
  motorR.loopFOC();

  // Follow core 1's enable request. enable() also resets the PID controllers.
  const bool want = g_enableReq;
  if (want != g_enabled) {
    if (want) { motorL.enable();  motorR.enable();  }
    else      { motorL.disable(); motorR.disable(); }
    g_enabled = want;
  }

  // Velocity loop. Called even when disabled so measured speed keeps updating.
  motorL.move(want ? g_targetL : 0.0f);
  motorR.move(want ? g_targetR : 0.0f);

  // Publish measurements for core 1.
  g_ticksL  = forwardTicks(encL, motorL, LEFT_FORWARD_SIGN);
  g_ticksR  = forwardTicks(encR, motorR, RIGHT_FORWARD_SIGN);
  g_velL    = LEFT_FORWARD_SIGN  * motorL.shaft_velocity;
  g_velR    = RIGHT_FORWARD_SIGN * motorR.shaft_velocity;
  g_encErrL = encL.currWireError ? g_encErrL + 1 : 0;
  g_encErrR = encR.currWireError ? g_encErrR + 1 : 0;
  g_loops++;
}

// =============================================================================
//                                   CORE 1
// =============================================================================

// ---- Output queue: core 1 never blocks on USB. If the host stops reading,
//      lines are dropped instead of freezing the safety code. -----------------
static const uint32_t OUT_SIZE = 2048;          // must be a power of two
static char     outBuf[OUT_SIZE];
static uint32_t outHead = 0, outTail = 0;       // free-running byte counters

static uint32_t outUsed() { return outHead - outTail; }

static void say(const char* fmt, ...) {
  char line[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(line) - 2) n = sizeof(line) - 2;
  line[n++] = '\n';
  if ((uint32_t)n > OUT_SIZE - outUsed()) return;          // no room: drop the line
  for (int i = 0; i < n; i++) outBuf[(outHead + i) & (OUT_SIZE - 1)] = line[i];
  outHead += n;
}

static void pumpOutput() {
  static uint32_t notConnectedSinceMs = 0;
  if (outUsed() == 0) return;
  // "!Serial" is also briefly true while the USB stack is busy on the other
  // core, so only throw the queue away after 200 ms of genuinely no host.
  if (!Serial) {
    uint32_t now = millis();
    if (!notConnectedSinceMs) notConnectedSinceMs = now;
    else if (now - notConnectedSinceMs > 200) outTail = outHead;
    return;
  }
  notConnectedSinceMs = 0;
  while (outUsed() > 0) {
    int room = Serial.availableForWrite();
    if (room <= 0) return;
    uint32_t start = outTail & (OUT_SIZE - 1);
    uint32_t chunk = outUsed();
    if (chunk > OUT_SIZE - start) chunk = OUT_SIZE - start;
    if (chunk > (uint32_t)room)   chunk = room;
    size_t w = Serial.write((const uint8_t*)&outBuf[start], chunk);
    if (w == 0) return;
    outTail += w;
  }
}

// ---- Core 1 state --------------------------------------------------------------
static ArmState armState = DISARMED;
static uint16_t latched = 0;
static uint16_t flags = 0, lastFlags = 0xFFFF;
static float    cmdL = 0, cmdR = 0;          // last 'v' command, wheel rad/s (clamped)
static float    rampL = 0, rampR = 0;        // ramped wheel targets actually used
static uint32_t lastCmdMs = 0;
static bool     everCommanded = false;
static float    vbat = 0, vmot = 0;
static bool     vmotLive = false;
static bool     estop = false;
static uint32_t estopLowSinceMs = 0;
static float    stallTimeL = 0, stallTimeR = 0;
static bool     battLow = false;
static uint32_t battLowSinceMs = 0;
static uint32_t loopHz = 0;
static uint32_t lastNotArmedMsgMs = 0;
static bool     vmotLowSeen = false;         // 1.0.1: VMOT-lost debounce
static uint32_t vmotLowSinceMs = 0;

// Latching relay: one pulse TOGGLES it. We queue pulses and space them out.
static uint8_t  relayPending = 0;
static bool     relayPulsing = false;
static uint32_t relayPulseEndMs = 0, relayIdleSinceMs = 0, relayLastPulseMs = 0;

static float readVolts(int pin, float divider) {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return (sum / 8.0f) * (ADC_VREF / 4095.0f) * divider;
}

static void latchFault(uint16_t f, const char* why) {
  if (latched & f) return;
  latched |= f;
  say("# FAULT: %s", why);
}

// Immediate, from core 1: pull both EN pins low right now (does not wait for
// core 0), and tell core 0 to disable the drivers and zero the PWM as well.
static void driversOffNow() {
  liftEmergencyStop();              // 1.1.0: the lift stops in the same instant
  digitalWrite(PIN_L_EN, LOW);
  digitalWrite(PIN_R_EN, LOW);
  g_enableReq = false;
  g_targetL = 0.0f;  g_targetR = 0.0f;
  cmdL = cmdR = rampL = rampR = 0.0f;
  stallTimeL = stallTimeR = 0.0f;
}

static void relayService(uint32_t now) {
  if (relayPulsing) {
    if ((int32_t)(now - relayPulseEndMs) >= 0) {
      digitalWrite(PIN_RELAY_TRIG, LOW);
      relayPulsing = false;
      relayIdleSinceMs = now;
    }
  } else if (relayPending > 0 && now - relayIdleSinceMs >= 200) {
    digitalWrite(PIN_RELAY_TRIG, HIGH);
    relayPulsing = true;
    relayPulseEndMs = now + RELAY_PULSE_MS;
    relayLastPulseMs = now;
    relayPending--;
  }
}
static bool relayBusy() { return relayPulsing || relayPending > 0; }

static void disarm(const char* why) {
  driversOffNow();
  if (armState == ARMED) {
    relayPending++;                                    // relay is closed: open it
  } else if (armState == ARMING) {
    if (relayPending > 0) relayPending--;              // close-pulse not sent yet: cancel
    else relayPending++;                               // already sent: send another
  } else {
    return;                                            // already disarmed / disarming
  }
  armState = DISARMING;
  say("# DISARMED: %s", why);
}

static void requestArm() {
  if (armState == ARMED)    { say("# already armed"); return; }
  if (armState != DISARMED) { say("# relay busy, try again in a second"); return; }
  if (latched)              { say("# cannot arm: latched fault 0x%04X - fix it, then send 'c'", latched); return; }
  if (estop)                { say("# cannot arm: E-stop is pressed"); return; }
  if (flags & F_NO_BATT)    { say("# cannot arm: no battery voltage (green switch / battery?)"); return; }
  cmdL = cmdR = rampL = rampR = 0.0f;
  vmotLowSeen = false;                                 // 1.0.1: fresh debounce for this arm
  if (vmotLive) { armState = ARMED; say("# ARMED (motor power was already on)"); return; }
  relayPending++;                                      // close the relay
  armState = ARMING;
  say("# arming...");
}

static void clearFaults() {
  uint16_t keep = 0;
  if ((latched & F_BATT_CUTOFF) && vbat > VBAT_PRESENT_V && vbat < VBAT_CUTOFF_V + 0.3f) keep |= F_BATT_CUTOFF;
  if ((latched & F_ENCODER_L) && g_encErrL > 0) keep |= F_ENCODER_L;
  if ((latched & F_ENCODER_R) && g_encErrR > 0) keep |= F_ENCODER_R;
  latched = keep;
  battLow = false;
  if (keep) say("# some faults could not be cleared (cause still present): 0x%04X", keep);
  else      say("# faults cleared");
}

// Limit how fast the two wheel targets change. Both wheels are scaled by the
// same factor, so the robot keeps the same curve while it speeds up/slows down.
static float stepAllowed(float cur, float want, float dt) {
  bool speedingUp = fabsf(want) > fabsf(cur) && (want * cur >= 0.0f);
  return (speedingUp ? MAX_WHEEL_ACCEL : MAX_WHEEL_DECEL) * dt;
}
static void rampPair(float wantL, float wantR, float dt) {
  float dl = wantL - rampL, dr = wantR - rampR, scale = 1.0f;
  float ml = stepAllowed(rampL, wantL, dt), mr = stepAllowed(rampR, wantR, dt);
  if (fabsf(dl) > ml) scale = fminf(scale, ml / fabsf(dl));
  if (fabsf(dr) > mr) scale = fminf(scale, mr / fabsf(dr));
  rampL += dl * scale;
  rampR += dr * scale;
}

// "Stall" = commanded to move, but not moving at least 20% of that speed in the
// right direction. Also catches a wheel spinning the WRONG way (bad calibration).
static void stallCheck(float target, float measured, float& timer, float dt,
                       uint16_t flag, const char* msg) {
  bool following = measured * (target >= 0 ? 1.0f : -1.0f) >= STALL_SPEED_FRAC * fabsf(target);
  if (fabsf(target) > STALL_MIN_CMD && !following) timer += dt; else timer = 0.0f;
  if (timer * 1000.0f > STALL_TIME_MS) latchFault(flag, msg);
}

static void controlTick(uint32_t now, float dt) {
  // ---- Measure supply voltages (hysteresis so VMOT does not flicker) --------
  vbat = readVolts(PIN_VBAT_ADC, VBAT_DIVIDER);
  vmot = readVolts(PIN_VMOT_ADC, VMOT_DIVIDER);
  if (vmotLive && vmot < VMOT_OFF_V)      vmotLive = false;
  else if (!vmotLive && vmot > VMOT_ON_V) vmotLive = true;

  // ---- Battery: must stay below the cutoff for VBAT_CUTOFF_MS ----------------
  bool battPresent = vbat > VBAT_PRESENT_V;
  if (battPresent && vbat < VBAT_CUTOFF_V) {
    if (!battLow) { battLow = true; battLowSinceMs = now; }
    else if (now - battLowSinceMs > VBAT_CUTOFF_MS)
      latchFault(F_BATT_CUTOFF, "battery below cutoff - charge it");
  } else {
    battLow = false;
  }

  // ---- Encoders --------------------------------------------------------------
  if (g_encErrL > ENCODER_MAX_ERRORS) latchFault(F_ENCODER_L, "LEFT encoder not answering (I2C0: GP0/GP1)");
  if (g_encErrR > ENCODER_MAX_ERRORS) latchFault(F_ENCODER_R, "RIGHT encoder not answering (I2C1: GP18/GP19)");

  // ---- Arm state machine ------------------------------------------------------
  switch (armState) {
    case ARMING:
      if (!relayBusy() && now - relayLastPulseMs >= RELAY_SETTLE_MS) {
        if (vmotLive) { armState = ARMED; say("# ARMED - motor power ON"); }
        else {
          armState = DISARMED;
          say("# arm failed: motor bus did not come on. Is the blue switch on? Relay wired?");
        }
      }
      break;

    case DISARMING:
      // Opening the relay does not drop VMOT instantly: the 470 uF bus capacitors
      // take a moment to drain. So we allow RELAY_OPEN_TIMEOUT_MS.
      if (!relayBusy()) {
        if (!vmotLive) { armState = DISARMED; say("# motor power OFF"); }
        else if (now - relayLastPulseMs > RELAY_OPEN_TIMEOUT_MS) {
          armState = DISARMED;
          latchFault(F_RELAY, "relay did NOT open. Drivers are off; use the E-stop or blue switch");
        }
      }
      break;

    case ARMED:
      // 1.0.1: VMOT must stay low for VMOT_LOST_CONFIRM_MS (ignores a momentary sag)
      if (vmot < VMOT_ON_V) {
        if (!vmotLowSeen) { vmotLowSeen = true; vmotLowSinceMs = now; }
        else if (now - vmotLowSinceMs >= VMOT_LOST_CONFIRM_MS)
          disarm("motor power lost (E-stop, blue switch or flat battery)");
      } else {
        vmotLowSeen = false;
      }
      if (armState == ARMED && latched) disarm("latched fault");
      break;

    case DISARMED:
      // While disarmed, motor power should be OFF. If it is on (at boot, or the
      // blue switch was turned on with the relay still closed), open the relay.
      if (vmotLive && !estop && !(latched & F_RELAY) && !relayBusy()
          && now - relayLastPulseMs > 1000) {
        relayPending++;
        armState = DISARMING;
        say("# motor bus is live while disarmed - opening relay");
      }
      break;
  }

  // ---- Velocity targets -------------------------------------------------------
  const bool canDrive = (armState == ARMED) && !estop && !latched;
  const bool stale    = !everCommanded || (now - lastCmdMs > CMD_TIMEOUT_MS);

  if (canDrive) {
    rampPair(stale ? 0.0f : cmdL, stale ? 0.0f : cmdR, dt);
    if (g_enabled) {
      stallCheck(rampL, g_velL / GEAR_RATIO, stallTimeL, dt, F_STALL_L, "LEFT wheel not following command (stall, or wrong direction)");
      stallCheck(rampR, g_velR / GEAR_RATIO, stallTimeR, dt, F_STALL_R, "RIGHT wheel not following command (stall, or wrong direction)");
    }
    g_targetL   = rampL * GEAR_RATIO * LEFT_FORWARD_SIGN;    // wheel -> motor rad/s
    g_targetR   = rampR * GEAR_RATIO * RIGHT_FORWARD_SIGN;
    g_enableReq = true;
  } else {
    driversOffNow();
  }

  // ---- Flags --------------------------------------------------------------------
  uint16_t live = 0;
  if (estop)                       live |= F_ESTOP;
  if (!vmotLive)                   live |= F_NO_VMOT;
  if (stale)                       live |= F_CMD_TIMEOUT;
  if (!battPresent)                live |= F_NO_BATT;
  else if (vbat < VBAT_WARN_V)     live |= F_BATT_WARN;
  if (loopHz && loopHz < 1000)     live |= F_LOOP_SLOW;
  flags = live | latched;
}

static void printFlagNames(uint16_t f) {
  if (!f) { say("#   (no flags)"); return; }
  if (f & F_ESTOP)       say("#   ESTOP        E-stop pressed");
  if (f & F_NO_VMOT)     say("#   NO_VMOT      no motor power");
  if (f & F_CMD_TIMEOUT) say("#   CMD_TIMEOUT  no recent 'v' command (wheels held at 0)");
  if (f & F_BATT_WARN)   say("#   BATT_WARN    battery low");
  if (f & F_NO_BATT)     say("#   NO_BATT      no battery voltage");
  if (f & F_LOOP_SLOW)   say("#   LOOP_SLOW    FOC loop under 1 kHz");
  if (f & F_BATT_CUTOFF) say("#   BATT_CUTOFF  (latched) battery below cutoff");
  if (f & F_STALL_L)     say("#   STALL_L      (latched) left wheel not following");
  if (f & F_STALL_R)     say("#   STALL_R      (latched) right wheel not following");
  if (f & F_ENCODER_L)   say("#   ENCODER_L    (latched) left encoder fault");
  if (f & F_ENCODER_R)   say("#   ENCODER_R    (latched) right encoder fault");
  if (f & F_RELAY)       say("#   RELAY        (latched) relay did not open");
}

static void printInfo() {
  say("# pico_motor_firmware %s", FIRMWARE_VERSION);
  say("# state=%s flags=0x%04X  vbat=%.2f V  vmot=%.2f V  estop=%s",
      ARM_NAMES[armState], flags, vbat, vmot, estop ? "PRESSED" : "released");
  printFlagNames(flags);
  say("# FOC loop: %lu Hz (each motor)   encoder errors: L=%lu R=%lu",
      (unsigned long)loopHz, (unsigned long)g_encErrL, (unsigned long)g_encErrR);
  say("# gear=%.3f  ticks per wheel rev=%.0f", GEAR_RATIO, ENCODER_CPR * GEAR_RATIO);
  say("# limits: speed=%.2f rad/s accel=%.1f decel=%.1f rad/s^2 timeout=%d ms",
      MAX_WHEEL_SPEED, MAX_WHEEL_ACCEL, MAX_WHEEL_DECEL, CMD_TIMEOUT_MS);
  say("# motor: R=%.2f ohm KV=%.0f I_lim=%.2f A V_lim=%.2f V supply=%.2f V",
      PHASE_RESISTANCE, MOTOR_KV, CURRENT_LIMIT, VOLTAGE_LIMIT, SUPPLY_VOLTAGE);
  say("# PID: P=%.4f I=%.4f D=%.4f  Tf=%.3f",
      motorL.PID_velocity.P, motorL.PID_velocity.I, motorL.PID_velocity.D, VEL_LPF_TF);
  say("# forward signs: L=%d R=%d", LEFT_FORWARD_SIGN, RIGHT_FORWARD_SIGN);
  liftPrintInfo();                  // 1.1.0
  usPrintInfo();                    // 1.2.0
}

static void handleLine(char* line, uint32_t now) {
  while (*line == ' ') line++;
  if (*line == 0) return;
  char cmd = line[0];
  char* p = line + 1;
  char* end;

  switch (cmd) {
    case 'v': {
      float l = strtof(p, &end); if (end == p) { say("# usage: v <left> <right>"); return; } p = end;
      float r = strtof(p, &end); if (end == p) { say("# usage: v <left> <right>"); return; }
      if (!isfinite(l) || !isfinite(r)) { say("# rejected: not a number"); return; }
      float biggest = fmaxf(fabsf(l), fabsf(r));
      if (biggest > MAX_WHEEL_SPEED) {            // clamp both by the same factor
        float s = MAX_WHEEL_SPEED / biggest;      // so the robot's curve is kept
        l *= s; r *= s;
      }
      lastCmdMs = now;
      everCommanded = true;
      if (armState == ARMED) { cmdL = l; cmdR = r; }
      else if (now - lastNotArmedMsgMs > 2000) {
        lastNotArmedMsgMs = now;
        say("# ignoring 'v': not armed (send 'a 1')");
      }
      break;
    }
    case 'a': {
      long v = strtol(p, &end, 10);
      if (end == p) { say("# usage: a 1 (arm) | a 0 (disarm)"); return; }
      if (v) requestArm(); else disarm("requested by host");
      break;
    }
    case 'c': clearFaults(); break;
    case 'k': {
      float P = strtof(p, &end); if (end == p) { say("# usage: k <P> <I> <D>"); return; } p = end;
      float I = strtof(p, &end); if (end == p) { say("# usage: k <P> <I> <D>"); return; } p = end;
      float D = strtof(p, &end); if (end == p) D = 0.0f;
      if (!(P >= 0 && P <= 5 && I >= 0 && I <= 100 && D >= 0 && D <= 1)) {
        say("# rejected: need 0<=P<=5, 0<=I<=100, 0<=D<=1"); return;
      }
      motorL.PID_velocity.P = P; motorL.PID_velocity.I = I; motorL.PID_velocity.D = D;
      motorR.PID_velocity.P = P; motorR.PID_velocity.I = I; motorR.PID_velocity.D = D;
      say("# PID set: P=%.4f I=%.4f D=%.4f (copy into config.h when happy)", P, I, D);
      break;
    }
    case 'l':                                          // 1.1.0: lift commands
      liftCommand(p, armState == ARMED);
      break;
    case 'u': usCommand(p); break;                  // 1.2.0: u 1 | u 0 | u ?
    case 'i': printInfo(); break;
    case 'h': case '?':
      say("# commands: v <left> <right> | a 1 | a 0 | c | k <P> <I> <D> | i | h");
      say("# wheel speeds in rad/s, + = forward. Resend 'v' at least every %d ms.", CMD_TIMEOUT_MS);
      say("# lift: l h (home) | l p (park) | l g <mm> [speed] | l s (stop) | l ? (info)");
      say("# ultrasonic: u 1 (on) | u 0 (off) | u ? (info)");
      break;
    default:
      say("# unknown command '%c' (send h for help)", cmd);
  }
}

static char    inLine[96];
static uint8_t inLen = 0;
static bool    inOverflow = false;

static void pollSerial(uint32_t now) {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      if (!inOverflow) { inLine[inLen] = 0; handleLine(inLine, now); }
      inLen = 0; inOverflow = false;
    } else if (inLen < sizeof(inLine) - 1) {
      inLine[inLen++] = (char)c;
    } else {
      inOverflow = true;                     // too long: ignore the whole line
    }
  }
}

void setup1() {
  while (!g_core0Ready) delay(1);            // wait until the motors are set up
  analogReadResolution(12);
  usBegin();                         // 1.2.0: echo interrupts on core 1
  estopLowSinceMs = millis();

  bool wdReboot = watchdog_enable_caused_reboot();   // true only after a real timeout
  rp2040.wdt_begin(HW_WATCHDOG_MS);          // from here on, both cores must keep running

  say("# pico_motor_firmware %s ready%s. Send 'h' for help.", FIRMWARE_VERSION,
      wdReboot ? " (!! last reset was caused by the WATCHDOG)" : "");
  if (!g_encFoundL) latchFault(F_ENCODER_L, "LEFT encoder not found at boot (I2C0: GP0/GP1)");
  if (!g_encFoundR) latchFault(F_ENCODER_R, "RIGHT encoder not found at boot (I2C1: GP18/GP19)");
}

void loop1() {
  static uint32_t lastControlMs = 0, lastReportMs = 0, lastStatusMs = 0;
  static uint32_t lastWdMs = 0, lastSeenLoops = 0, lastRateMs = 0, lastRateLoops = 0;
  const uint32_t now = millis();

  // ---- E-stop (1.0.1): act once GP14 has been HIGH for ESTOP_CONFIRM_MS;
  //      accept release only after 50 ms low. Shorter HIGH blips are noise:
  //      they are counted and reported, but do not disarm.
  //      (1.0.0 acted on the FIRST high reading, so a noise spike when the relay
  //      closed could trigger a false E-stop -> a second relay pulse -> relay
  //      opened again. That was the "have to arm twice" bug.)
  static bool     estopHighSeen = false;
  static uint32_t estopHighSinceMs = 0;
  static uint32_t estopGlitches = 0, estopGlitchesReported = 0, lastGlitchMsgMs = 0;

  if (digitalRead(PIN_ESTOP_SENSE) == HIGH) {
    if (!estopHighSeen) { estopHighSeen = true; estopHighSinceMs = now; }
    if (!estop && now - estopHighSinceMs >= ESTOP_CONFIRM_MS) {
      estop = true;
      driversOffNow();
      disarm("E-STOP PRESSED");
      say("# E-STOP PRESSED");
    }
    estopLowSinceMs = now;
  } else {
    if (estopHighSeen && !estop) estopGlitches++;   // HIGH, but too short to be a press
    estopHighSeen = false;
    if (estop && now - estopLowSinceMs >= 50) {
      estop = false;
      say("# E-stop released. Motors stay OFF until you send 'a 1'.");
    }
  }
  if (estopGlitches != estopGlitchesReported && now - lastGlitchMsgMs >= 1000) {
    lastGlitchMsgMs = now;
    estopGlitchesReported = estopGlitches;
    say("# E-stop input: ignored %lu short glitch(es) so far (noise on GP14)",
        (unsigned long)estopGlitches);
  }

  pollSerial(now);
  liftUpdate(armState == ARMED);   // 1.1.0: lift state machine + 'l' status line
  usUpdate(now);                   // 1.2.0: ultrasonic round-robin, 'u' lines
  relayService(now);

  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlMs) / 1000.0f;
    if (dt > 0.05f) dt = 0.05f;
    lastControlMs = now;
    controlTick(now, dt);
  }

  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = now;
    if (outUsed() < 512)                       // skip if the host is falling behind
      say("e %ld %ld %.4f %.4f %lu", (long)g_ticksL, (long)g_ticksR,
          g_velL / GEAR_RATIO, g_velR / GEAR_RATIO, (unsigned long)micros());
  }

  if (flags != lastFlags || now - lastStatusMs >= STATUS_PERIOD_MS) {
    uint16_t changed = (flags ^ lastFlags) & ~F_CMD_TIMEOUT;
    if (lastFlags != 0xFFFF && changed) { say("# flags now 0x%04X:", flags); printFlagNames(flags); }
    lastFlags = flags;
    lastStatusMs = now;
    say("s %04X %u %.2f %.2f", flags, (unsigned)armState, vbat, vmot);
  }

  if (now - lastRateMs >= 1000) {              // measure the FOC loop rate
    uint32_t l = g_loops;
    loopHz = (l - lastRateLoops) * 1000UL / (now - lastRateMs);
    lastRateLoops = l;
    lastRateMs = now;
  }

  // Hardware watchdog: only feed it if core 0 is ALSO still running.
  if (now - lastWdMs >= 20) {
    lastWdMs = now;
    uint32_t l = g_loops;
    if (l != lastSeenLoops) { rp2040.wdt_reset(); lastSeenLoops = l; }
  }

  pumpOutput();
}
