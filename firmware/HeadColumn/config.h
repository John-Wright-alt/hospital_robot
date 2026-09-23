#pragma once
// =====================================================================
//  config.h  -  EVERY setting you are likely to change lives in here.
//  You should not need to edit any other file.
//  After changing anything here, upload the sketch again.
// =====================================================================


// ---------------------------------------------------------------------
// 1. WiFi
// ---------------------------------------------------------------------
// Must be a 2.4 GHz network. The ESP32-S3 cannot see 5 GHz networks.
#define WIFI_SSID          "Vodacom-V6WF"
#define WIFI_PASSWORD      "}+=S^>(rm]JgqV?6"
#define DEVICE_HOSTNAME    "headcolumn"      // browser: http://headcolumn.local

// If the WiFi above can't be joined within 15 s, the board ALSO makes its
// own WiFi network so you can still reach it at http://192.168.4.1
// (it keeps trying to join your WiFi in the background).
#define FALLBACK_AP_ENABLED   1
#define FALLBACK_AP_SSID      "HeadColumn-Setup"
#define FALLBACK_AP_PASSWORD  "headcolumn123"  // at least 8 characters


// ---------------------------------------------------------------------
// 2. What is connected right now?
// ---------------------------------------------------------------------
#define ENABLE_CAMERA   1   // 1 = camera + face detection running
#define ENABLE_MOTOR    0   // WIRING REV 2: ALWAYS 0. The stepper, driver and home
                            // switch are wired to the Pico in the robot body, not
                            // to this board. This board only watches for faces and
                            // reports them to the Pi over its USB cable.
#define ENABLE_WIFI     1   // 1 = browser preview (http://headcolumn.local). The
                            // robot does NOT need WiFi - the Pi gets face data over
                            // USB. Set 0 on the finished robot (boots 15 s faster).

// What happens at power-up (only used when ENABLE_MOTOR is 1)
#define AUTO_HOME_ON_BOOT       1   // 1 = drive down to the micro switch first
#define AUTO_SEARCH_AFTER_HOME  1   // 1 = then lift and look for a face
                                    // 0 = stay parked until told "search"


// ---------------------------------------------------------------------
// 3. Pins - NOT USED with wiring rev 2 (the lift is on the Pico GP10-GP13).
//    Leave them as they are. (GPIO numbers = number BEFORE the slash on the board,
//    e.g. the pin printed "12/D12" is GPIO 12)
// ---------------------------------------------------------------------
#define PIN_STEP           12   // board label 12/D12 -> driver STEP
#define PIN_DIR            13   // board label 13/D11 -> driver DIR
#define PIN_ENABLE         14   // board label 14/D10 -> driver EN  (-1 if not wired)
#define PIN_LIMIT_BOTTOM    9   // board label 9/D7   -> bottom micro switch
#define PIN_LIMIT_TOP      -1   // -1 = no top switch. Board label 18/D6 if you add one.
#define PIN_STATUS_LED     21   // the small LED on the board (-1 to disable)

// Electrical polarity
#define ENABLE_ACTIVE_LOW   1   // A4988 / DRV8825 / TMC2209: EN LOW = motor powered
#define DIR_HIGH_MEANS_UP   1   // if the column moves the WRONG way, change to 0
#define LIMIT_SWITCH_NC     1   // 1 = switch wired COM->GND and NC->pin (recommended:
                                //     a broken wire reads as "pressed" = safe)
                                // 0 = switch wired COM->GND and NO->pin


// ---------------------------------------------------------------------
// 4. Mechanics of your column
// ---------------------------------------------------------------------
#define MOTOR_STEPS_PER_REV   200     // normal 1.8 degree stepper = 200
#define MICROSTEPS            8       // set by the MS pins on the driver board
#define MM_PER_REV            8.0f    // lead screw "lead" (T8x8 = 8.0, T8x2 = 2.0)
                                      // belt: pulley teeth x belt pitch (20T GT2 = 40.0)
#define COLUMN_TRAVEL_MM      400.0f  // total travel from the bottom switch to the top
#define SOFT_LIMIT_BOTTOM_MM  5.0f    // tracking never goes lower than this
#define SOFT_LIMIT_TOP_MM     390.0f  // tracking/searching never goes higher than this


// ---------------------------------------------------------------------
// 5. Speeds (mm per second) - keep them slow, this moves at face height
// ---------------------------------------------------------------------
#define MAX_SPEED_MM_S        20.0f
#define SEARCH_SPEED_MM_S     15.0f   // while looking for a face
#define TRACK_SPEED_MM_S      20.0f   // while following a face
#define PARK_SPEED_MM_S       20.0f   // going down before navigation
#define HOMING_FAST_MM_S      10.0f   // first approach to the switch
#define HOMING_SLOW_MM_S       3.0f   // final gentle approach to the switch
#define ACCEL_MM_S2           40.0f   // how quickly speed changes
#define HOMING_BACKOFF_MM      5.0f   // move up this far after first touching the switch
#define PARK_APPROACH_MM      10.0f   // park: fast down to here, then creep to the switch
#define DISABLE_MOTOR_WHEN_PARKED 1   // 1 saves power and heat. Set 0 if your column
                                      // can slide down by itself (e.g. a belt drive)


// ---------------------------------------------------------------------
// 6. Face tracking behaviour
// ---------------------------------------------------------------------
#define TARGET_Y_FRACTION      0.50f  // where the face centre should sit in the picture
                                      // (0.0 = top edge, 0.5 = middle, 1.0 = bottom)
#define AVERAGE_FACE_WIDTH_MM  150.0f // used to convert pixels into millimetres
#define TRACK_GAIN             0.8f   // 1.0 = full correction each frame, lower = calmer
#define TRACK_DEADBAND_MM      15.0f  // don't bother moving for errors smaller than this
#define MIN_FACE_WIDTH_PX      28     // ignore small faces (people far away)
#define FACE_CONFIRM_FRAMES    2      // frames in a row needed before a face is believed
#define FACE_LOST_TIMEOUT_MS   15000  // no face this long while tracking -> search again
#define SEARCH_TIMEOUT_MS      60000  // give up searching after this -> wait in place


// ---------------------------------------------------------------------
// 7. Camera
// ---------------------------------------------------------------------
#define CAMERA_VFLIP           0      // set 1 if the camera is mounted upside down
#define CAMERA_HMIRROR         0      // set 1 to mirror the picture left/right
#define FACE_DETECT_TWO_STAGE  1      // 1 = more accurate (slower), 0 = faster
#define STREAM_JPEG_QUALITY    70     // web preview quality, 10 (small) .. 95 (sharp)


// =====================================================================
//  Safety checks - these stop the upload with a clear message if a
//  setting is impossible. You don't need to change anything below.
// =====================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#error "Wrong ESP32 board package. Tools > Board > Boards Manager > 'esp32 by Espressif Systems' > choose version 2.0.17 > Install. (Version 3.x no longer contains the face detector.)"
#endif
#if ENABLE_CAMERA && !defined(BOARD_HAS_PSRAM)
#error "PSRAM is switched off. Tools > PSRAM > 'OPI PSRAM', then upload again. The camera needs it."
#endif

constexpr bool pinIsReserved(int p) {
  // camera, flash/PSRAM, USB, BOOT button and the onboard power button
  return p == 0 || p == 1 || p == 2 || p == 4 || p == 5 || p == 6 || p == 7 ||
         p == 8 || p == 19 || p == 20 || (p >= 26 && p <= 37) || p == 39 ||
         p == 40 || p == 41 || p == 42 || p == 45 || p == 46 || p == 47 || p == 48;
}
static_assert(!pinIsReserved(PIN_STEP), "PIN_STEP is a pin used by the camera/flash/USB. Pick another.");
static_assert(!pinIsReserved(PIN_DIR), "PIN_DIR is a pin used by the camera/flash/USB. Pick another.");
static_assert(!pinIsReserved(PIN_ENABLE), "PIN_ENABLE is a pin used by the camera/flash/USB. Pick another.");
static_assert(!pinIsReserved(PIN_LIMIT_BOTTOM), "PIN_LIMIT_BOTTOM is a pin used by the camera/flash/USB. Pick another.");
static_assert(!pinIsReserved(PIN_LIMIT_TOP), "PIN_LIMIT_TOP is a pin used by the camera/flash/USB. Pick another.");
static_assert(PIN_STEP >= 0 && PIN_DIR >= 0 && PIN_LIMIT_BOTTOM >= 0,
              "PIN_STEP, PIN_DIR and PIN_LIMIT_BOTTOM must be real pins.");
static_assert(PIN_STEP != PIN_DIR && PIN_STEP != PIN_LIMIT_BOTTOM && PIN_DIR != PIN_LIMIT_BOTTOM,
              "Two functions are set to the same pin.");
static_assert(SOFT_LIMIT_TOP_MM <= COLUMN_TRAVEL_MM, "SOFT_LIMIT_TOP_MM must be <= COLUMN_TRAVEL_MM");
static_assert(SOFT_LIMIT_BOTTOM_MM > 0.0f, "SOFT_LIMIT_BOTTOM_MM must be above 0 so tracking never presses the switch");
static_assert(MAX_SPEED_MM_S * (MOTOR_STEPS_PER_REV * MICROSTEPS / MM_PER_REV) <= 8000.0f,
              "MAX_SPEED_MM_S needs more than 8000 steps per second. Lower MICROSTEPS or MAX_SPEED_MM_S.");
