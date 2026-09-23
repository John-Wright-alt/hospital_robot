#pragma once
// =====================================================================
//  vision.h  -  camera + on-board face detection.
//
//  Runs as its own task on processor core 0 so it never disturbs the
//  motor (core 1). For every camera frame it:
//    1. grabs a 320x240 picture
//    2. runs Espressif's face detector (esp-dl, built into the ESP32
//       Arduino core 2.0.x) directly on the chip - no PC, no cloud
//    3. keeps the biggest (= closest) face and remembers where the motor
//       was when the picture was taken
//    4. if someone is watching the web page, draws the box and makes a
//       JPEG for the live stream
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <list>
#include <vector>
#include "esp_camera.h"
#include "img_converters.h"
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "config.h"
#include "motion.h"

namespace vision {

// FireBeetle 2 ESP32-S3 camera connector (from DFRobot's pin table)
constexpr int CAM_PIN_PWDN  = -1;
constexpr int CAM_PIN_RESET = -1;
constexpr int CAM_PIN_XCLK  = 45;
constexpr int CAM_PIN_SIOD  = 1;
constexpr int CAM_PIN_SIOC  = 2;
constexpr int CAM_PIN_D7    = 48;   // "Y9"
constexpr int CAM_PIN_D6    = 46;   // "Y8"
constexpr int CAM_PIN_D5    = 8;    // "Y7"
constexpr int CAM_PIN_D4    = 7;    // "Y6"
constexpr int CAM_PIN_D3    = 4;    // "Y5"
constexpr int CAM_PIN_D2    = 41;   // "Y4"
constexpr int CAM_PIN_D1    = 40;   // "Y3"
constexpr int CAM_PIN_D0    = 39;   // "Y2"
constexpr int CAM_PIN_VSYNC = 6;
constexpr int CAM_PIN_HREF  = 42;
constexpr int CAM_PIN_PCLK  = 5;

constexpr int FRAME_W = 320;
constexpr int FRAME_H = 240;

// One detection result (a copy is handed to the main program)
struct Face {
  uint32_t frame = 0;              // goes up by one for every processed picture
  bool     found = false;
  int16_t  cx = 0, cy = 0;         // face centre (pixels)
  int16_t  w = 0, h = 0;           // face box size (pixels)
  float    score = 0.0f;
  int32_t  motorPosAtCapture = 0;  // motor position (steps) when picture was taken
  uint32_t millisAtCapture = 0;
};

static Face           s_face;
static portMUX_TYPE   s_faceMux = portMUX_INITIALIZER_UNLOCKED;
static volatile float s_fps = 0.0f;
static volatile bool  s_cameraOk = false;
static bool           s_axpFound = false;

// Latest JPEG for the web stream
volatile int              s_streamClients = 0;
static SemaphoreHandle_t  s_jpgMutex = nullptr;
static uint8_t*           s_jpg = nullptr;
static size_t             s_jpgLen = 0;
static volatile uint32_t  s_jpgSeq = 0;

static HumanFaceDetectMSR01* s_det1 = nullptr;
#if FACE_DETECT_TWO_STAGE
static HumanFaceDetectMNP01* s_det2 = nullptr;
#endif

// ---------------------------------------------------------------------
// Camera power.
// Board versions V1.0/V1.1 power the camera through an AXP313A power chip
// at I2C address 0x36 (SDA = GPIO1, SCL = GPIO2). These are exactly the
// register writes done by DFRobot's official DFRobot_AXP313A library
// (begin() + enableCameraPower(eOV2640)), so no extra library is needed.
// Board V1.2 and later have fixed regulators: the chip is absent and
// nothing needs to be done.
// ---------------------------------------------------------------------
static void axpWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x36);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool powerCamera() {
  bool found = false;
  Wire.begin((int)CAM_PIN_SIOD, (int)CAM_PIN_SIOC, (uint32_t)100000);
  for (int attempt = 0; attempt < 3 && !found; attempt++) {
    Wire.beginTransmission(0x36);
    Wire.write(0x00);
    found = (Wire.endTransmission() == 0);
    if (!found) delay(50);
  }
  if (found) {
    axpWrite(0x00, 0x04);   // begin()
    axpWrite(0x10, 0x19);   // enable camera supply rails
    delay(10);
    axpWrite(0x16, 7);      // ALDO: 1.2 V (camera core)
    delay(10);
    axpWrite(0x17, 22);     // DLDO: the value the official library sends for "2.8 V"
    delay(10);
  }
  Wire.end();               // hand GPIO1/2 over to the camera driver
  delay(found ? 150 : 10);  // let the supplies settle
  return found;
}

static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_D0;  c.pin_d1 = CAM_PIN_D1;  c.pin_d2 = CAM_PIN_D2;  c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4;  c.pin_d5 = CAM_PIN_D5;  c.pin_d6 = CAM_PIN_D6;  c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk     = CAM_PIN_XCLK;
  c.pin_pclk     = CAM_PIN_PCLK;
  c.pin_vsync    = CAM_PIN_VSYNC;
  c.pin_href     = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn     = CAM_PIN_PWDN;
  c.pin_reset    = CAM_PIN_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;     // raw pixels: what the face detector needs
  c.frame_size   = FRAMESIZE_QVGA;       // 320 x 240
  c.jpeg_quality = 12;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;   // always give us the newest picture

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[camera] init FAILED, error 0x%x (see Troubleshooting in the guide)\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, CAMERA_VFLIP);
    s->set_hmirror(s, CAMERA_HMIRROR);
  }
  return true;
}

// ---------------------------------------------------------------------
// Face detection
// ---------------------------------------------------------------------
static void detectFaces(camera_fb_t* fb, Face& out) {
  std::vector<int> shape = {(int)fb->height, (int)fb->width, 3};
#if FACE_DETECT_TWO_STAGE
  std::list<dl::detect::result_t>& candidates = s_det1->infer((uint16_t*)fb->buf, shape);
  std::list<dl::detect::result_t>& results = s_det2->infer((uint16_t*)fb->buf, shape, candidates);
#else
  std::list<dl::detect::result_t>& results = s_det1->infer((uint16_t*)fb->buf, shape);
#endif
  int bestW = 0;
  for (auto& r : results) {
    if (r.box.size() < 4) continue;
    int x1 = r.box[0], y1 = r.box[1], x2 = r.box[2], y2 = r.box[3];
    int w = x2 - x1, h = y2 - y1;
    if (w < MIN_FACE_WIDTH_PX || w <= bestW) continue;   // keep the biggest face
    bestW = w;
    out.found = true;
    out.cx = (x1 + x2) / 2;
    out.cy = (y1 + y2) / 2;
    out.w = w;
    out.h = h;
    out.score = r.score;
  }
}

// ---------------------------------------------------------------------
// Drawing on the RGB565 picture (for the web preview only)
// ---------------------------------------------------------------------
static inline void px(uint8_t* buf, int x, int y, uint16_t c) {
  if (x < 0 || y < 0 || x >= FRAME_W || y >= FRAME_H) return;
  uint8_t* p = buf + 2 * (y * FRAME_W + x);
  p[0] = c >> 8;
  p[1] = c & 0xFF;
}
static void hline(uint8_t* b, int x0, int x1, int y, uint16_t c, int dash = 0) {
  for (int x = x0; x <= x1; x++) if (!dash || ((x / dash) & 1) == 0) px(b, x, y, c);
}
static void vline(uint8_t* b, int x, int y0, int y1, uint16_t c) {
  for (int y = y0; y <= y1; y++) px(b, x, y, c);
}
static void box(uint8_t* b, int x0, int y0, int x1, int y1, uint16_t c) {
  for (int t = 0; t < 2; t++) {
    hline(b, x0 - t, x1 + t, y0 - t, c);
    hline(b, x0 - t, x1 + t, y1 + t, c);
    vline(b, x0 - t, y0 - t, y1 + t, c);
    vline(b, x1 + t, y0 - t, y1 + t, c);
  }
}

static void drawOverlay(camera_fb_t* fb, const Face& f) {
  if (fb->width != FRAME_W || fb->height != FRAME_H) return;
  uint8_t* b = fb->buf;
  const uint16_t CYAN = 0x07FF, GREEN = 0x07E0, YELLOW = 0xFFE0;
  int ty = (int)(TARGET_Y_FRACTION * FRAME_H);
  hline(b, 0, FRAME_W - 1, ty, CYAN, 6);                    // where the face should be
  if (f.found) {
    box(b, f.cx - f.w / 2, f.cy - f.h / 2, f.cx + f.w / 2, f.cy + f.h / 2, GREEN);
    for (int dy = -2; dy <= 2; dy++) hline(b, f.cx - 2, f.cx + 2, f.cy + dy, YELLOW);
    vline(b, f.cx, f.cy, ty, YELLOW);                       // error bar
  }
}

// ---------------------------------------------------------------------
// The vision task (runs forever on core 0)
// ---------------------------------------------------------------------
static void visionTask(void*) {
  uint32_t frames = 0;
  uint32_t t0 = millis();
  for (;;) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    Face f;
    f.motorPosAtCapture = motion::position();
    f.millisAtCapture = millis();
    if (fb->format == PIXFORMAT_RGB565) detectFaces(fb, f);

    portENTER_CRITICAL(&s_faceMux);
    f.frame = s_face.frame + 1;
    s_face = f;
    portEXIT_CRITICAL(&s_faceMux);

    if (s_streamClients > 0) {
      drawOverlay(fb, f);
      uint8_t* jpg = nullptr;
      size_t len = 0;
      if (frame2jpg(fb, STREAM_JPEG_QUALITY, &jpg, &len)) {
        xSemaphoreTake(s_jpgMutex, portMAX_DELAY);
        uint8_t* old = s_jpg;
        s_jpg = jpg;
        s_jpgLen = len;
        s_jpgSeq = s_jpgSeq + 1;
        xSemaphoreGive(s_jpgMutex);
        free(old);
      }
    }
    esp_camera_fb_return(fb);

    frames++;
    uint32_t now = millis();
    if (now - t0 >= 1000) {
      s_fps = frames * 1000.0f / (now - t0);
      frames = 0;
      t0 = now;
    }
    vTaskDelay(1);   // let other tasks (WiFi, watchdog) breathe
  }
}

// ---------------------------------------------------------------------
// Public functions
// ---------------------------------------------------------------------
inline bool begin() {
  s_jpgMutex = xSemaphoreCreateMutex();
  if (!psramFound()) {
    Serial.println("[camera] PSRAM NOT FOUND -> Tools > PSRAM must be 'OPI PSRAM'. Camera disabled.");
    return false;
  }
  s_axpFound = powerCamera();
  Serial.printf("[camera] power chip AXP313A %s\n",
                s_axpFound ? "found and switched on (board V1.0/V1.1)"
                           : "not present (board V1.2+ powers the camera itself)");
  if (!initCamera()) return false;

#if FACE_DETECT_TWO_STAGE
  s_det1 = new HumanFaceDetectMSR01(0.1F, 0.5F, 10, 0.2F);
  s_det2 = new HumanFaceDetectMNP01(0.5F, 0.3F, 5);
#else
  s_det1 = new HumanFaceDetectMSR01(0.3F, 0.5F, 10, 0.2F);
#endif

  s_cameraOk = true;
  xTaskCreatePinnedToCore(visionTask, "vision", 16384, nullptr, 2, nullptr, 0);
  Serial.println("[camera] running, face detection active");
  return true;
}

inline bool cameraOk() { return s_cameraOk; }
inline float fps() { return s_fps; }

inline Face latestFace() {
  Face f;
  portENTER_CRITICAL(&s_faceMux);
  f = s_face;
  portEXIT_CRITICAL(&s_faceMux);
  return f;
}

}  // namespace vision
