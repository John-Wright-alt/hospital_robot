/*
  =====================================================================
   HeadColumn.ino
   Face-tracking height column for a patient-guide robot
   Board: DFRobot FireBeetle 2 ESP32-S3 (with OV2640 camera)
  =====================================================================
   What it does
    1. Power-up: drives DOWN onto the bottom micro switch ("homing") so it
       knows where 0 mm is. Steppers can't tell where they are by themselves.
    2. SEARCHING: lifts slowly while the camera looks for a face.
    3. TRACKING:  keeps the face centred up/down in the picture.
    4. "park" command (from the touchscreen via ROS): drives down onto the
       micro switch -> PARKED. The status then says "parked": true and
       ROS can start navigating.
    5. "search" command: lifts and looks for a face again (e.g. on arrival).

   Talk to it:
    - Arduino Serial Monitor at 115200 baud, type  help
    - Browser: http://headcolumn.local  (or the IP shown in Serial Monitor)

   All settings are in config.h
  =====================================================================
*/
#include <WiFi.h>
#include <ESPmDNS.h>
#include "config.h"
#include "motion.h"
#include "vision.h"
#include "web.h"

// ---------------------------------------------------------------------
// States
// ---------------------------------------------------------------------
enum class State : uint8_t {
  STARTING, CAMERA_ONLY, HOMING, PARKING, PARKED,
  SEARCHING, TRACKING, WAITING, MANUAL, STOPPED, FAULT
};

enum class Phase : uint8_t { BACKOFF_FIRST, FAST_DOWN, BACKOFF, APPROACH, SLOW_DOWN };

static State    g_state        = State::STARTING;
static State    g_afterHome    = State::PARKED;
static Phase    g_phase        = Phase::FAST_DOWN;
static bool     g_homed        = false;
static uint32_t g_stateSince   = 0;
static int32_t  g_seekStart    = 0;
static int32_t  g_seekLimit    = 0;
static char     g_fault[128]   = "";
static float    g_lastDriftMm  = 0.0f;

static vision::Face g_face;
static uint32_t g_lastFrame    = 0;
static int      g_faceStreak   = 0;
static uint32_t g_lastFaceMs   = 0;
static float    g_errorMm      = 0.0f;   // + = face ABOVE the target line -> go UP
static bool     g_sweepUp      = true;

static bool     g_apStarted    = false;
static bool     g_wifiReported = false;
static uint32_t g_lastWifiTry  = 0;
static uint32_t g_lastStatusMs = 0;

static bool     g_faceLines    = false;   // "faces on": one 'f' line per camera frame for the Pi

static char     g_line[64];
static uint8_t  g_lineLen      = 0;

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------
static const char* stateName(State s) {
  switch (s) {
    case State::STARTING:    return "STARTING";
    case State::CAMERA_ONLY: return "CAMERA_ONLY";
    case State::HOMING:      return "HOMING";
    case State::PARKING:     return "PARKING";
    case State::PARKED:      return "PARKED";
    case State::SEARCHING:   return "SEARCHING";
    case State::TRACKING:    return "TRACKING";
    case State::WAITING:     return "WAITING";
    case State::MANUAL:      return "MANUAL";
    case State::STOPPED:     return "STOPPED";
    case State::FAULT:       return "FAULT";
  }
  return "?";
}

static int32_t softMinSteps() { return motion::mmToSteps(SOFT_LIMIT_BOTTOM_MM); }
static int32_t softMaxSteps() { return motion::mmToSteps(SOFT_LIMIT_TOP_MM); }
static float   heightMm()     { return motion::stepsToMm(motion::position()); }

static void setState(State s) {
  if (s != g_state) Serial.printf("[state] %s -> %s\n", stateName(g_state), stateName(s));
  g_state = s;
  g_stateSince = millis();
}

static void fault(const char* msg) {
  motion::emergencyStop();
  strlcpy(g_fault, msg, sizeof(g_fault));
  Serial.printf("\n!!! FAULT: %s\n!!! Fix the problem, then send 'home' to try again.\n\n", msg);
  setState(State::FAULT);
}

// ---------------------------------------------------------------------
// Face information from the vision task
// ---------------------------------------------------------------------
static bool pollFace() {                    // true when a NEW picture was processed
  if (!vision::cameraOk()) return false;
  vision::Face f = vision::latestFace();
  if (f.frame == g_lastFrame) return false;
  g_lastFrame = f.frame;
  g_face = f;
  if (f.found) {
    if (g_faceStreak < 1000) g_faceStreak++;
    g_lastFaceMs = millis();
    // pixels between face centre and target line, converted to millimetres
    // using the face width as a ruler (a face is roughly 150 mm wide)
    float errPx = TARGET_Y_FRACTION * vision::FRAME_H - f.cy;
    g_errorMm = errPx * (AVERAGE_FACE_WIDTH_MM / (float)f.w);
  } else {
    g_faceStreak = 0;
  }
  return true;
}

static bool faceConfirmed() { return g_faceStreak >= FACE_CONFIRM_FRAMES; }

// ---------------------------------------------------------------------
// Entering states
// ---------------------------------------------------------------------
static void enterParked() {
  setState(State::PARKED);
  if (DISABLE_MOTOR_WHEN_PARKED) motion::enableDriver(false);
  Serial.println(">>> PARKED on the bottom switch - safe for ROS to navigate.");
}

static void enterSearching() {
  motion::takeBottomHit();
  motion::takeTopHit();
  g_faceStreak = 0;
  g_sweepUp = true;
  setState(State::SEARCHING);
  motion::moveTo(softMaxSteps(), SEARCH_SPEED_MM_S);
  Serial.println("Searching for a face (lifting)...");
}

static void applyTracking() {
  if (!g_face.found) return;
  // Standing still and already close enough? Then don't move (no jitter).
  if (!motion::isMoving() && fabsf(g_errorMm) < TRACK_DEADBAND_MM) return;
  // Target = where the motor was when the picture was taken + correction.
  float targetMm = motion::stepsToMm(g_face.motorPosAtCapture) + TRACK_GAIN * g_errorMm;
  targetMm = constrain(targetMm, SOFT_LIMIT_BOTTOM_MM, SOFT_LIMIT_TOP_MM);
  motion::moveTo(motion::mmToSteps(targetMm), TRACK_SPEED_MM_S);
}

static void enterTracking() {
  setState(State::TRACKING);
  g_lastFaceMs = millis();
  Serial.println("Face found - tracking.");
  applyTracking();
}

static void enterWaiting() {
  motion::stop();
  g_faceStreak = 0;
  setState(State::WAITING);
}

// ---------------------------------------------------------------------
// Homing and parking (both end with the column resting on the switch)
// ---------------------------------------------------------------------
static void beginSeek(float speedMmS, float maxMm) {
  g_seekStart = motion::position();
  g_seekLimit = motion::mmToSteps(maxMm);
  motion::seekDown(speedMmS);
}

static void startHoming(State after) {
  g_afterHome = after;
  g_homed = false;
  g_fault[0] = 0;
  motion::emergencyStop();
  motion::setSoftMax(false, 0);
  motion::takeBottomHit();
  motion::takeTopHit();
  setState(State::HOMING);
  Serial.println("Homing: moving DOWN to find the bottom switch...");
  if (motion::bottomPressed()) {
    g_phase = Phase::BACKOFF_FIRST;           // already on the switch: lift off first
    motion::moveTo(motion::position() + motion::mmToSteps(HOMING_BACKOFF_MM), HOMING_FAST_MM_S);
  } else {
    g_phase = Phase::FAST_DOWN;
    beginSeek(HOMING_FAST_MM_S, COLUMN_TRAVEL_MM + 50.0f);
  }
}

static void startParking() {
  if (!g_homed) { startHoming(State::PARKED); return; }
  if (g_state == State::PARKED && motion::bottomPressed()) {
    Serial.println("Already parked.");
    return;
  }
  motion::takeBottomHit();
  g_fault[0] = 0;
  setState(State::PARKING);
  g_phase = Phase::APPROACH;
  int32_t approach = motion::mmToSteps(PARK_APPROACH_MM);
  if (motion::position() > approach) motion::moveTo(approach, PARK_SPEED_MM_S);
  else motion::stop();
  Serial.println("Parking: lowering to the bottom switch...");
}

static void finishAtBottom() {
  int32_t posAtHit = motion::position();
  bool wasHomed = g_homed;
  motion::setPosition(0);
  motion::setSoftMax(true, motion::mmToSteps(COLUMN_TRAVEL_MM));
  g_homed = true;
  if (wasHomed) {
    g_lastDriftMm = motion::stepsToMm(posAtHit);
    Serial.printf("Bottom switch reached. Position error corrected: %.1f mm\n", g_lastDriftMm);
  } else {
    Serial.println("Homing complete. Bottom switch = 0 mm.");
  }
  State next = (g_state == State::HOMING) ? g_afterHome : State::PARKED;
  if (next == State::SEARCHING) enterSearching();
  else enterParked();
}

static void updateHomingParking() {
  const bool hit = motion::takeBottomHit();
  const int32_t pos = motion::position();

  switch (g_phase) {
    case Phase::BACKOFF_FIRST:
      if (!motion::isMoving()) {
        if (motion::bottomPressed()) {
          fault("Bottom switch still reads PRESSED after moving up. Check the switch wiring and LIMIT_SWITCH_NC in config.h");
          return;
        }
        g_phase = Phase::FAST_DOWN;
        beginSeek(HOMING_FAST_MM_S, COLUMN_TRAVEL_MM + 50.0f);
      }
      break;

    case Phase::FAST_DOWN:
      if (hit) {
        g_phase = Phase::BACKOFF;             // touched it: lift off a little...
        motion::moveTo(pos + motion::mmToSteps(HOMING_BACKOFF_MM), HOMING_FAST_MM_S);
      } else if (g_seekStart - pos > g_seekLimit) {
        fault("Bottom switch never triggered. Check the switch wiring, and that the column moves DOWN while homing (else flip DIR_HIGH_MEANS_UP)");
      }
      break;

    case Phase::BACKOFF:
      if (!motion::isMoving()) {
        if (motion::bottomPressed()) {
          fault("Bottom switch still reads PRESSED after moving up. Check the switch wiring and LIMIT_SWITCH_NC in config.h");
          return;
        }
        g_phase = Phase::SLOW_DOWN;           // ...then come back slowly for accuracy
        beginSeek(HOMING_SLOW_MM_S, HOMING_BACKOFF_MM + 10.0f);
      }
      break;

    case Phase::APPROACH:
      if (hit) { finishAtBottom(); return; }  // switch came early (steps were lost lifting)
      if (!motion::isMoving()) {
        g_phase = Phase::SLOW_DOWN;
        beginSeek(HOMING_SLOW_MM_S, PARK_APPROACH_MM + 15.0f);
      }
      break;

    case Phase::SLOW_DOWN:
      if (hit) { finishAtBottom(); return; }
      if (g_seekStart - pos > g_seekLimit) {
        if (g_state == State::PARKING) {
          Serial.println("WARNING: switch not where expected - doing a full homing run.");
          startHoming(State::PARKED);
        } else {
          fault("Bottom switch did not trigger during the slow approach");
        }
      }
      break;
  }
}

// A limit was hit while tracking/searching/manual moving
static void handleUnexpectedLimits() {
  if (motion::takeBottomHit()) {
    motion::setPosition(0);                  // we are definitely at the bottom now
    Serial.println("WARNING: bottom switch hit unexpectedly - position reset to 0 mm.");
    if (g_state == State::SEARCHING) {
      g_sweepUp = true;
      motion::moveTo(softMaxSteps(), SEARCH_SPEED_MM_S);
    }
  }
  if (motion::takeTopHit()) {
    Serial.println("WARNING: top limit reached - movement stopped.");
    if (g_state == State::SEARCHING) {
      g_sweepUp = false;
      motion::moveTo(softMinSteps(), SEARCH_SPEED_MM_S);
    }
  }
}

// ---------------------------------------------------------------------
// State updates (called every loop)
// ---------------------------------------------------------------------
static void updateSearching() {
  if (faceConfirmed()) { enterTracking(); return; }
  if (millis() - g_stateSince > SEARCH_TIMEOUT_MS) {
    Serial.println("No face found - holding position (send 'search' to try again).");
    enterWaiting();
    return;
  }
  if (!motion::isMoving()) {                 // reached one end: sweep the other way
    g_sweepUp = !g_sweepUp;
    motion::moveTo(g_sweepUp ? softMaxSteps() : softMinSteps(), SEARCH_SPEED_MM_S);
  }
}

static void updateTracking(bool newFrame) {
  if (newFrame && g_face.found) applyTracking();
  if (millis() - g_lastFaceMs > FACE_LOST_TIMEOUT_MS) {
    Serial.println("Face lost - searching again.");
    enterSearching();
  }
}

// ---------------------------------------------------------------------
// Commands (from Serial Monitor or the web/ROS)
// ---------------------------------------------------------------------
static void printHelp() {
  Serial.println(F(
    "\nCommands (type and press Enter):\n"
    "  help            this list\n"
    "  status          show everything\n"
    "  switch          show the bottom switch reading (wiring test)\n"
    "  home            find the bottom switch (sets 0 mm)\n"
    "  search          lift and look for a face, then track it\n"
    "  park            lower onto the micro switch (before navigation)\n"
    "  wait            hold position; start tracking if a face appears\n"
    "  up 20           move up 20 mm      down 20   move down 20 mm\n"
    "  goto 150        move to 150 mm above the switch\n"
    "  motor off       switch the driver off (column may drop if belt driven!)\n"
    "  motor on        switch the driver on\n"
    "  stop            STOP immediately\n"
    "  faces on / off  send one 'f' line per camera frame (the Pi turns this on)\n"));
}

static void printSwitch() {
  int level = motion::bottomPinLevel();
  bool pressed = motion::bottomRawPressed();
  Serial.printf("Bottom switch  GPIO%d reads %s  ->  %s   (wiring mode: %s)\n",
                PIN_LIMIT_BOTTOM, level ? "HIGH" : "LOW", pressed ? "PRESSED" : "released",
                LIMIT_SWITCH_NC ? "NC (COM->GND, NC->pin)" : "NO (COM->GND, NO->pin)");
  if (!ENABLE_MOTOR) Serial.println("(ENABLE_MOTOR is 0, so the pin has no pull-up yet - reading may float)");
}

static void printStatus() {
  Serial.printf("\nState: %s | homed: %s | height: %.1f mm (target %.1f) | moving: %s\n",
                stateName(g_state), g_homed ? "yes" : "NO", heightMm(),
                motion::stepsToMm(motion::target()), motion::isMoving() ? "yes" : "no");
  if (ENABLE_MOTOR) {
    Serial.printf("Bottom switch: %s | Top switch: %s | Driver: %s | last park correction: %.1f mm\n",
                  motion::bottomPressed() ? "PRESSED" : "released",
                  PIN_LIMIT_TOP < 0 ? "not fitted" : (motion::topPressed() ? "PRESSED" : "released"),
                  motion::driverEnabled() ? "ON" : "off", g_lastDriftMm);
  }
  if (vision::cameraOk()) {
    if (g_face.found)
      Serial.printf("Camera: %.1f fps | Face: YES centre y=%d, width=%d px -> error %+.1f mm (%s)\n",
                    vision::fps(), g_face.cy, g_face.w, g_errorMm,
                    fabsf(g_errorMm) < TRACK_DEADBAND_MM ? "centred" : (g_errorMm > 0 ? "column should go UP" : "column should go DOWN"));
    else
      Serial.printf("Camera: %.1f fps | Face: none\n", vision::fps());
  } else {
    Serial.println("Camera: NOT running");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("WiFi: %s  (http://%s  or  http://%s.local)  signal %d dBm\n",
                  WIFI_SSID, WiFi.localIP().toString().c_str(), DEVICE_HOSTNAME, WiFi.RSSI());
  else
    Serial.println("WiFi: not connected");
  if (g_apStarted)
    Serial.printf("Own network: '%s' -> http://%s\n", FALLBACK_AP_SSID, WiFi.softAPIP().toString().c_str());
  if (g_fault[0]) Serial.printf("FAULT: %s\n", g_fault);
  Serial.println();
}

static void processCommand(const char* line) {
  char word[16] = {0};
  char arg[16] = {0};
  int i = 0;
  while (*line == ' ') line++;
  while (*line && *line != ' ' && i < 15) word[i++] = (char)tolower(*line++);
  while (*line == ' ') line++;
  i = 0;
  while (*line && *line != ' ' && i < 15) arg[i++] = (char)tolower(*line++);
  const bool hasArg = arg[0] != 0;
  const float value = atof(arg);

  if (!strcmp(word, "help") || !strcmp(word, "?")) { printHelp(); return; }
  if (!strcmp(word, "status")) { printStatus(); return; }
  if (!strcmp(word, "switch")) { printSwitch(); return; }
  if (!strcmp(word, "faces")) {             // wiring rev 2: the Pi reads face data over USB
    g_faceLines = strcmp(arg, "off") != 0;
    Serial.printf("# face lines %s\n", g_faceLines ? "ON" : "off");
    return;
  }
  if (!strcmp(word, "stop")) {
    motion::emergencyStop();
    if (ENABLE_MOTOR) setState(State::STOPPED);
    Serial.println("STOPPED. Send 'search', 'park' or 'home' to continue.");
    return;
  }
  if (!ENABLE_MOTOR) { Serial.println("Motor commands need ENABLE_MOTOR 1 in config.h."); return; }

  if (!strcmp(word, "home")) { startHoming(State::PARKED); return; }
  if (!strcmp(word, "park") || !strcmp(word, "lower") || !strcmp(word, "navigate")) { startParking(); return; }

  if (g_state == State::FAULT) {
    Serial.println("In FAULT - fix the problem, then send 'home' (or 'park').");
    return;
  }

  if (!strcmp(word, "search") || !strcmp(word, "start")) {
    if (g_state == State::HOMING) { g_afterHome = State::SEARCHING; Serial.println("Will search after homing."); }
    else if (!g_homed) startHoming(State::SEARCHING);
    else enterSearching();
    return;
  }
  if (!strcmp(word, "wait") || !strcmp(word, "hold")) {
    if (!g_homed) { Serial.println("Not homed yet - send 'home' first."); return; }
    enterWaiting();
    Serial.println("Holding position. Will track if a face appears.");
    return;
  }
  if (!strcmp(word, "up") || !strcmp(word, "down") || !strcmp(word, "goto")) {
    if (!g_homed) { Serial.println("Not homed yet - send 'home' first."); return; }
    if (!hasArg) { Serial.println("Give a distance in mm, e.g.  up 20"); return; }
    float t = heightMm();
    if (word[0] == 'u') t += value;
    else if (word[0] == 'd') t -= value;
    else t = value;
    t = constrain(t, SOFT_LIMIT_BOTTOM_MM, SOFT_LIMIT_TOP_MM);
    setState(State::MANUAL);
    motion::moveTo(motion::mmToSteps(t), TRACK_SPEED_MM_S);
    Serial.printf("Moving to %.1f mm\n", t);
    return;
  }
  if (!strcmp(word, "motor")) {
    if (motion::isMoving()) { Serial.println("Stop the motor first."); return; }
    bool on = !strcmp(arg, "on");
    motion::enableDriver(on);
    Serial.printf("Driver %s\n", on ? "ON" : "OFF");
    return;
  }
  Serial.printf("Unknown command '%s'. Type 'help'.\n", word);
}

static void handleSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (g_lineLen) {
        g_line[g_lineLen] = 0;
        Serial.printf("> %s\n", g_line);
        processCommand(g_line);
        g_lineLen = 0;
      }
    } else if (g_lineLen < sizeof(g_line) - 1) {
      g_line[g_lineLen++] = ch;
    }
  }
}

// ---------------------------------------------------------------------
// Status JSON (read by the web page and the ROS bridge)
// ---------------------------------------------------------------------
static void publishStatus() {
  if (millis() - g_lastStatusMs < 100) return;
  g_lastStatusMs = millis();
  const bool parked = (g_state == State::PARKED) && motion::bottomPressed();
  const bool faceNow = g_face.found && (millis() - g_lastFaceMs < 1000);
  char ip[20] = "";
  if (WiFi.status() == WL_CONNECTED) strlcpy(ip, WiFi.localIP().toString().c_str(), sizeof(ip));
  char buf[768];
  snprintf(buf, sizeof(buf),
    "{\"state\":\"%s\",\"homed\":%s,\"parked\":%s,\"height_mm\":%.1f,\"target_mm\":%.1f,"
    "\"velocity_mm_s\":%.1f,\"moving\":%s,\"bottom_switch\":%s,\"top_switch\":%s,"
    "\"motor_enabled\":%s,\"motor_configured\":%s,\"camera_ok\":%s,\"face\":%s,"
    "\"face_cx\":%d,\"face_cy\":%d,\"face_w\":%d,\"error_mm\":%.1f,\"fps\":%.1f,"
    "\"last_park_correction_mm\":%.1f,\"wifi_rssi\":%d,\"ip\":\"%s\",\"uptime_s\":%lu,\"fault\":\"%s\"}",
    stateName(g_state), g_homed ? "true" : "false", parked ? "true" : "false",
    heightMm(), motion::stepsToMm(motion::target()), motion::velocityMmS(),
    motion::isMoving() ? "true" : "false",
    motion::bottomPressed() ? "true" : "false", motion::topPressed() ? "true" : "false",
    motion::driverEnabled() ? "true" : "false", ENABLE_MOTOR ? "true" : "false",
    vision::cameraOk() ? "true" : "false", faceNow ? "true" : "false",
    g_face.cx, g_face.cy, g_face.w, faceNow ? g_errorMm : 0.0f, vision::fps(),
    g_lastDriftMm, (int)(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0), ip,
    (unsigned long)(millis() / 1000), g_fault);
  web::setStatus(buf);
}

// ---------------------------------------------------------------------
// Status LED:  solid = face visible | slow blink = homing/parking/searching
//              fast blink = FAULT   | off = idle
// ---------------------------------------------------------------------
static void updateLed() {
  if (PIN_STATUS_LED < 0) return;
  const uint32_t t = millis();
  const bool faceNow = g_face.found && (t - g_lastFaceMs < 500);
  bool on;
  if (g_state == State::FAULT) on = (t / 100) % 2;
  else if (faceNow) on = true;
  else if (g_state == State::HOMING || g_state == State::PARKING || g_state == State::SEARCHING) on = (t / 500) % 2;
  else on = false;
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------
static void printWifiInfo() {
  Serial.println("==========================================================");
  Serial.printf(" WiFi connected!  Open  http://%s   (or http://%s.local)\n",
                WiFi.localIP().toString().c_str(), DEVICE_HOSTNAME);
  Serial.println("==========================================================");
}

static void startWifi() {
  WiFi.persistent(false);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[wifi] connecting to '%s' ", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    printWifiInfo();
    g_wifiReported = true;
  } else {
    Serial.println("[wifi] could not connect - check name/password (network must be 2.4 GHz).");
    if (FALLBACK_AP_ENABLED) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASSWORD);
      g_apStarted = true;
      Serial.printf("[wifi] started own network '%s' (password '%s') -> http://%s\n",
                    FALLBACK_AP_SSID, FALLBACK_AP_PASSWORD, WiFi.softAPIP().toString().c_str());
      Serial.println("[wifi] still trying to join your WiFi in the background.");
    }
  }
  g_lastWifiTry = millis();
  if (MDNS.begin(DEVICE_HOSTNAME)) MDNS.addService("http", "tcp", 80);
}

static void updateWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!g_wifiReported) { printWifiInfo(); g_wifiReported = true; }
    return;
  }
  if (g_wifiReported) { Serial.println("[wifi] connection lost - retrying..."); g_wifiReported = false; }
  if (millis() - g_lastWifiTry > 15000) {
    g_lastWifiTry = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ---------------------------------------------------------------------
// setup() runs once at power-up, loop() runs forever
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(20);   // never let an unread USB port slow the control loop
#endif
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2500) delay(10);
  delay(200);
  Serial.println("\n\n=========== Head Column firmware ===========");
  Serial.printf("Camera: %s | Motor: %s | %.1f steps per mm\n",
                ENABLE_CAMERA ? "ON" : "off", ENABLE_MOTOR ? "ON" : "off", motion::STEPS_PER_MM);
  if (PIN_STATUS_LED >= 0) pinMode(PIN_STATUS_LED, OUTPUT);

  if (ENABLE_MOTOR) motion::begin();     // first, so the driver is OFF while booting
  if (ENABLE_CAMERA) vision::begin();
  if (ENABLE_WIFI) {
    startWifi();
    web::begin();
  } else {
    Serial.println("[wifi] off (ENABLE_WIFI 0) - face data goes to the Pi over USB.");
  }

  if (ENABLE_MOTOR) {
    if (AUTO_HOME_ON_BOOT) {
      startHoming(AUTO_SEARCH_AFTER_HOME ? State::SEARCHING : State::PARKED);
    } else {
      setState(State::STOPPED);
      Serial.println("Auto-homing is off. Type 'home' when ready.");
    }
  } else {
    setState(State::CAMERA_ONLY);
    Serial.println("Camera-only mode: the Pico drives the lift (wiring rev 2).");
    Serial.println("Type 'status' to see which way the column WOULD move, 'faces on' to see the Pi's data.");
  }
  Serial.println("Type 'help' for commands.");
}

void loop() {
  handleSerial();

  char cmd[40];
  while (web::popCommand(cmd, sizeof(cmd))) {
    Serial.printf("[web] %s\n", cmd);
    processCommand(cmd);
  }

  const bool newFrame = pollFace();

  // Wiring rev 2: face measurement for the Pi, one line per processed frame:
  //   f <frame> <found 0/1> <cx> <cy> <width_px> <error_mm (+ = go UP)> <fps>
  if (newFrame && g_faceLines) {
    Serial.printf("f %lu %d %d %d %d %.1f %.1f\n", (unsigned long)g_face.frame,
                  g_face.found ? 1 : 0, g_face.cx, g_face.cy, g_face.w,
                  g_face.found ? g_errorMm : 0.0f, vision::fps());
  }

  if (ENABLE_MOTOR) {
    switch (g_state) {
      case State::HOMING:
      case State::PARKING:   updateHomingParking(); break;
      case State::SEARCHING: handleUnexpectedLimits(); updateSearching(); break;
      case State::TRACKING:  handleUnexpectedLimits(); updateTracking(newFrame); break;
      case State::WAITING:   handleUnexpectedLimits(); if (faceConfirmed()) enterTracking(); break;
      case State::MANUAL:    handleUnexpectedLimits(); break;
      default: break;
    }
  }

  if (ENABLE_WIFI) updateWifi();
  updateLed();
  publishStatus();
  delay(5);
}
