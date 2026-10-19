// =============================================================================
//  config.h — EVERY number you may need to change lives in this file.
//
//  Hospital Robot — Pico W drive firmware (Phase 4)
//  Board:   Raspberry Pi Pico W   (Arduino IDE + "arduino-pico" core by Earle Philhower)
//  Library: Simple FOC 2.4.x
//
//  Lines marked  >>> MEASURE <<<  must be filled in from YOUR bench tests.
//  The Phase 4 guide tells you which test sketch gives you each number.
// =============================================================================
#pragma once

#define FIRMWARE_VERSION "1.1.0"   // 1.1.0: head-column lift (lift.h, Phase 8)

// -----------------------------------------------------------------------------
// 1. PIN MAP — copied from Robot_Wiring_Guide.md §9.1. Do NOT change these
//    unless you change the physical wiring.
// -----------------------------------------------------------------------------
// LEFT SimpleFOCmini
#define PIN_L_IN1   2     // GP2  (PWM slice 1A)
#define PIN_L_IN2   3     // GP3  (PWM slice 1B)
#define PIN_L_IN3   4     // GP4  (PWM slice 2A)
#define PIN_L_EN    5     // GP5  (10k pull-down on the board)
// RIGHT SimpleFOCmini
#define PIN_R_IN1   6     // GP6  (PWM slice 3A)
#define PIN_R_IN2   7     // GP7  (PWM slice 3B)
#define PIN_R_IN3   8     // GP8  (PWM slice 4A)
#define PIN_R_EN    9     // GP9  (10k pull-down on the board)
// AS5600 encoders — one per I2C bus because both are fixed at address 0x36
#define PIN_ENC_L_SDA  0  // GP0  I2C0 -> Wire
#define PIN_ENC_L_SCL  1  // GP1
#define PIN_ENC_R_SDA 18  // GP18 I2C1 -> Wire1
#define PIN_ENC_R_SCL 19  // GP19
// Safety chain
#define PIN_ESTOP_SENSE 14  // GP14 HIGH = E-stop pressed (via 33k/10k divider — see guide!)
#define PIN_RELAY_TRIG  15  // GP15 -> MOSFET -> latching relay (a pulse TOGGLES it)
#define PIN_VBAT_ADC    27  // GP27 / ADC1  battery voltage (100k/33k divider)
#define PIN_VMOT_ADC    28  // GP28 / ADC2  motor-bus voltage (100k/33k divider)
// Pins owned by later phases — this firmware only parks them in a safe state
#define PIN_LIFT_STEP   10  // GP10 DRV8825 STEP  (held LOW)
#define PIN_LIFT_EN     12  // GP12 DRV8825 EN, active LOW (held HIGH = disabled)
#define PIN_US1_TRIG    16  // HC-SR04 triggers (held LOW)
#define PIN_US2_TRIG    20
#define PIN_US3_TRIG    22

// -----------------------------------------------------------------------------
// 2. DRIVETRAIN
// -----------------------------------------------------------------------------
#define ENCODER_CPR  4096     // AS5600 = 12-bit = 4096 counts per MOTOR revolution
#define GEAR_RATIO   3.75f    // motor turns per wheel turn (60T wheel / 16T motor)
                              // >>> MEASURE <<< with test03_encoders (turn wheel 1 rev)
// Wheel radius is NOT needed here: this firmware works in wheel rad/s.
// (It is 0.072 m in your URDF; the Pi's bridge node will use that in Phase 6.)

// -----------------------------------------------------------------------------
// 3. MOTOR + DRIVER (DFRobot 4015 BLDC + SimpleFOCmini)
// -----------------------------------------------------------------------------
#define POLE_PAIRS        11      // from datasheet; confirmed by "PP check: OK!" in test06
#define PHASE_RESISTANCE  4.2f    // ohms, LINE-TO-LINE = multimeter between two motor wires
                                  // >>> MEASURE <<<  (your two documents disagree: 4.8 vs 7.2)
#define MOTOR_KV          61.0f   // rpm per volt, from datasheet
#define SUPPLY_VOLTAGE    12.6f   // Use the FULL-CHARGE pack voltage, not 11.1 V.
                                  // SimpleFOC converts volts -> PWM duty using this number.
                                  // If the real battery is higher than this, every voltage
                                  // (and current) is higher than you asked for. Using the
                                  // maximum means "real <= requested" — the safe direction.
#define PWM_FREQUENCY     20000   // Hz

// Current protection. There is NO current sensor, so these are ESTIMATES.
//   CURRENT_LIMIT caps the torque the velocity loop may ask for (normal limit).
//   VOLTAGE_LIMIT is the hard backstop. Worst-case (stalled rotor) peak current:
//        I_stall  ~=  2 x VOLTAGE_LIMIT / PHASE_RESISTANCE
//   e.g. 2 x 4.5 / 4.8 = 1.9 A.  Keep this <= 2.0 A (DRV8313 with no heatsink).
//   VOLTAGE_LIMIT also sets top speed: max motor rad/s ~= VOLTAGE_LIMIT x 11 (no load).
#define CURRENT_LIMIT     0.65f   // amps (estimated) — datasheet rated current @12 V
#define VOLTAGE_LIMIT     4.5f    // volts — see formula above

// -----------------------------------------------------------------------------
// 4. CALIBRATION — >>> MEASURE <<< with test06_closed_loop_single.
//    Run test06 once for LEFT and once for RIGHT, then paste the two lines it
//    prints for each motor here and delete the "#error" lines below.
//    Re-run test06 if you ever remove or re-glue an encoder magnet.
// -----------------------------------------------------------------------------
#define LEFT_ZERO_ELEC_ANGLE   5.5131f
#define LEFT_SENSOR_DIRECTION  Direction::CCW
#define RIGHT_ZERO_ELEC_ANGLE  5.7325f
#define RIGHT_SENSOR_DIRECTION Direction::CCW

#if !defined(LEFT_ZERO_ELEC_ANGLE) || !defined(LEFT_SENSOR_DIRECTION)
#error "LEFT motor not calibrated: run test06 with MOTOR_SIDE LEFT and paste the result into config.h section 4"
#endif
#if !defined(RIGHT_ZERO_ELEC_ANGLE) || !defined(RIGHT_SENSOR_DIRECTION)
#error "RIGHT motor not calibrated: run test06 with MOTOR_SIDE RIGHT and paste the result into config.h section 4"
#endif

// -----------------------------------------------------------------------------
// 5. WHICH WAY IS FORWARD?  >>> MEASURE <<< in the bench test (guide Part 8).
//    The motors are mirror images, so one of these is usually -1.
//    Send "v 1 1" with the robot on blocks: BOTH wheels must turn so the robot
//    would drive forward. If a wheel turns backwards, flip ITS sign.
// -----------------------------------------------------------------------------
#define LEFT_FORWARD_SIGN   (-1) //was (+1)
#define RIGHT_FORWARD_SIGN  (+1) //was (-1)

// -----------------------------------------------------------------------------
// 6. VELOCITY LOOP (PID) — starting values; tune with the 'k' command, then
//    copy your final numbers back here. Units: amps per (motor rad/s).
// -----------------------------------------------------------------------------
#define VEL_P        0.12f
#define VEL_I        0.9f
#define VEL_D        0.0f
#define VEL_LPF_TF   0.02f    // velocity low-pass filter time constant (s).
                              // AS5600 over I2C is noisy; 0.01–0.02 s is typical.

// -----------------------------------------------------------------------------
// 7. SAFETY LIMITS — these are enforced here no matter what the Pi sends.
//    Your URDF wheel radius is 0.072 m, so wheel rad/s x 0.072 = m/s at the rim.
// -----------------------------------------------------------------------------
#define MAX_WHEEL_SPEED   9.0f   // rad/s at the wheel  (= 0.65 m/s rim speed).
                                 // Must stay ABOVE what Nav2 can ask for:
                                 // (vx_max + wz_max x 0.13) / 0.072 = (0.5 + 1.9x0.13)/0.072 = 10.4
                                 // -> lower wz_max in Nav2 to ~1.0 in Phase 7, or combined
                                 //    fast turns get scaled down (safely, curvature kept).
#define MAX_WHEEL_ACCEL   7.0f   // rad/s^2 when speeding up  (~0.5 m/s^2)
#define MAX_WHEEL_DECEL  14.0f   // rad/s^2 when slowing down (~1.0 m/s^2)

#define CMD_TIMEOUT_MS    500    // no 'v' command for this long -> ramp to a stop
#define STALL_TIME_MS     2000   // commanded to move but not moving for this long -> fault
#define STALL_MIN_CMD     1.0f   // wheel rad/s; stall check only above this command
#define STALL_SPEED_FRAC  0.2f   // "not moving" = measured < 20% of commanded
#define ENCODER_MAX_ERRORS 50    // consecutive failed I2C reads -> encoder fault

// Battery (3S LiPo) — Robot_Wiring_Guide.md §8.6
#define VBAT_PRESENT_V    5.0f   // below this: no battery connected (USB-only bench work)
#define VBAT_WARN_V      11.1f   // warning flag
#define VBAT_CUTOFF_V    10.5f   // latched fault: refuse to drive
#define VBAT_CUTOFF_MS    2000   // must stay low this long (ignores brief sag under load)
// Motor bus present / absent thresholds (hysteresis stops flicker near the edge)
#define VMOT_ON_V         8.0f
#define VMOT_OFF_V        6.0f

// --- Added in 1.0.1 (Phase 6 guide, Part 0.2) -----------------------------------
// E-stop input glitch filter. GP14 must read HIGH continuously for this long
// before the firmware treats it as a real press. Shorter HIGH blips (electrical
// noise, e.g. from the inrush spike when the relay closes) are counted and
// reported with a '#' message instead. This does NOT weaken the E-stop: the
// button cuts motor power in hardware (Layer 1) instantly; this only affects how
// fast the firmware NOTICES, and 3 ms is nothing next to that.
#define ESTOP_CONFIRM_MS      3
// While ARMED, VMOT must stay below VMOT_ON_V this long before the firmware
// decides motor power was lost. Ignores a momentary sag; a real loss (blue
// switch, E-stop) lasts far longer.
#define VMOT_LOST_CONFIRM_MS  20

// Voltage dividers: 100k over 33k -> scale (100+33)/33 = 4.03.
// >>> MEASURE <<< with test04: compare with a multimeter and correct these.
#define VBAT_DIVIDER     4.2841f
#define VMOT_DIVIDER     4.2572f
#define ADC_VREF         3.3f

// Latching relay pulse (FC-73): one pulse toggles it
#define RELAY_PULSE_MS    100
#define RELAY_SETTLE_MS   250    // after CLOSING: VMOT must be up within this time
#define RELAY_OPEN_TIMEOUT_MS 3000 // after OPENING: VMOT must fall below VMOT_OFF_V within
                                 // this time. The 3 x 470 uF bus capacitors take a moment
                                 // to drain. If test04 shows > 3 s, add a 1 kohm 0.5 W
                                 // bleed resistor from VMOT to GND.

// Encoder I2C speed. 400 kHz is safe. The AS5600 supports 1 MHz, which speeds up
// the FOC loop — try 1000000 only if your encoder cables are short and twisted.
#define I2C_CLOCK_HZ      400000

// Hardware watchdog: reboots the Pico if either core hangs (drivers then
// switch off through their 10k pull-downs).
#define HW_WATCHDOG_MS    250

// Serial protocol timing
#define REPORT_PERIOD_MS  20     // 'e' lines at 50 Hz
#define STATUS_PERIOD_MS  500    // 's' line at least this often
#define CONTROL_PERIOD_MS 5      // ramp/safety tick (200 Hz)
