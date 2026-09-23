/*
  BoardCheck.ino  -  your FIRST upload.
  Proves that: uploading works, the Serial Monitor works, PSRAM is
  switched on (the camera needs it), and tells you your board version.
  The onboard LED blinks once per second.
*/
#include <Wire.h>

const int LED_PIN = 21;   // onboard LED on the FireBeetle 2 ESP32-S3
bool ledOn = false;
bool axpFound = false;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  pinMode(LED_PIN, OUTPUT);

  // Is the AXP313A camera power chip present? (I2C address 0x36 on GPIO1/GPIO2)
  Wire.begin(1, 2);
  Wire.beginTransmission(0x36);
  axpFound = (Wire.endTransmission() == 0);
  Wire.end();
}

void loop() {
  ledOn = !ledOn;
  digitalWrite(LED_PIN, ledOn ? HIGH : LOW);

  Serial.println("----------------------------------------------");
  Serial.printf("Chip: %s, %d cores, %d MHz\n", ESP.getChipModel(), ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("Flash: %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  if (psramFound()) {
    Serial.printf("PSRAM: %u MB   [OK]\n", ESP.getPsramSize() / (1024 * 1024));
  } else {
    Serial.println("PSRAM: NOT FOUND   [PROBLEM] -> Tools > PSRAM > 'OPI PSRAM', then upload again");
  }
  Serial.printf("Camera power chip AXP313A: %s\n",
                axpFound ? "found (board V1.0/V1.1) - the firmware switches it on for you"
                         : "not found (board V1.2 or newer - camera is powered automatically)");
  Serial.printf("Arduino ESP32 core version: %d.%d.%d  %s\n", ESP_ARDUINO_VERSION_MAJOR,
                ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
                ESP_ARDUINO_VERSION_MAJOR == 2 ? "[OK]" : "[PROBLEM] -> install version 2.0.17 (see guide)");
  delay(1000);
}
