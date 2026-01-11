#include "ProjectInfo.h"

// Check if the secrets.h file exists (only for testing here)
#if __has_include("secrets.h")
    #include "secrets.h"
#else
    // Fallback if the file is missing (e.g., on a fresh GitHub clone)
    #define WIFI_SSID "REPLACE_ME"
    #define WIFI_PASS "REPLACE_ME"
    #warning "secrets.h not found! Using dummy credentials."
#endif
// 1. Define the hardware model FIRST so camera_pins.h knows which pins to use
#define CAMERA_MODEL_XIAO_ESP32S3 

#include "esp_camera.h"
#include "camera_pins.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// --- Pin Definitions ---
#define PIR_SENSOR_PIN  1    // D0 on XIAO ESP32-S3
#define SD_CS_PIN       21   // Built-in SD CS for Sense board

int photoCount = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(PIR_SENSOR_PIN, INPUT);

  // Initialize SD Card
  if(!SD.begin(SD_CS_PIN)){
    Serial.println("❌ SD Mount Failed");
    return;
  }

  // Camera Configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  
  // These constants are now pulled directly from camera_pins.h
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  config.xclk_freq_hz = 20000000;
  
  // --- Bandwidth Optimization ---
  // Using VGA (640x480) for faster processing and lower latency
  config.frame_size = FRAMESIZE_VGA; 
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Initialize Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed 0x%x", err);
    return;
  }

  Serial.println("🛡️ Intrusion Monitor Active (VGA Mode)...");
}

void captureAndSave() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed");
    return;
  }

  String path = "/motion_" + String(photoCount++) + ".jpg";
  
  File file = SD.open(path.c_str(), FILE_WRITE);
  if (file) {
    file.write(fb->buf, fb->len);
    Serial.printf("📸 Motion Detected! Saved: %s (%u bytes)\n", path.c_str(), fb->len);
  } else {
    Serial.println("❌ Failed to open file for writing");
  }
  file.close();
  esp_camera_fb_return(fb);
}

void loop() {
  if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
    captureAndSave();
    delay(5000); // Cool-down period
    Serial.println("🛡️ Monitoring for movement...");
  }
  delay(100);
}
