/**
 * @file test_mic_alert.ino
 * @author Matha Goram
 * @brief Adaptive VAD with MQTT Burst, Light Sleep, Battery & RSSI Monitoring.
 * * --- WORKFLOW DESCRIPTION ---
 * 1. INITIALIZATION: Hardware setup and NTP time sync via a brief WiFi burst.
 * 2. POWER MANAGEMENT: Uses Automatic Light Sleep to conserve power between buffer reads.
 * 3. AUDIO ANALYSIS: 16kHz sampling with DC-offset removal and Frequency detection.
 * 4. MONITORING: Reads Battery Voltage (GPIO 1) and WiFi RSSI before MQTT transmission.
 * 5. LOCAL WARNING: LED blinks rapidly if battery < 3.4V, independent of WiFi status.
 * 6. BURST REPORTING: Wakes WiFi/MQTT to send JSON payload with audio, battery, and signal data.
 * * --- OPTIMIZATION PARAMETERS ---
 * - SIGNAL_BOOST: 12.0 (dB above floor)
 * - REQUIRED_DUR_MS: 150 (ms persistence)
 * - MIN_FREQ: 400.0 (Hz cutoff)
 * * --- LICENSE ---
 * MIT License
 * Copyright (c) 2026 ParkCircus Productions; All Rights Reserved
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include "driver/i2s_pdm.h"
#include "esp_pm.h"
#include <math.h>
#include "secrets.h"

// Audio and VAD Settings
#define SAMPLE_RATE     16000
#define BUFFER_SIZE     512    
#define OFFSET_ALPHA    0.999  
#define REFERENCE_16BIT 32768.0
#define SIGNAL_BOOST    12.0  
#define MIN_FREQ        400.0 
#define REQUIRED_DUR_MS 150   

// Battery and Signal Settings
#define BATTERY_PIN      1    
#define LOW_BATT_CUTOFF  3.40 
#define BLINK_INTERVAL   500  

// Hardware Pins
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_42
#define I2S_MIC_DATA_IN      GPIO_NUM_41
#define LED_PIN              LED_BUILTIN 

#define MQTT_CLIENT "XIAO1"
#define MQTT_TOPIC "ha/XIAO/audio"

// Globals
WiFiClient espClient;
PubSubClient client(espClient);
i2s_chan_handle_t rx_handle = NULL;

float dc_offset = 0;
float ambient_db = -60.0;
unsigned long soundStartTime = 0;
unsigned long lastBlinkTime = 0;
bool soundIsActive = false;
bool eventSent = false;
bool ledState = HIGH;

float readBattery() {
  uint32_t raw = analogRead(BATTERY_PIN);
  return (float)raw * 2.0 * 3.3 / 4095.0; 
}

String getWaqt() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "Time_Error";
  char timeString[25];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeString);
}

void setup_wifi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) delay(100);
  if(WiFi.status() == WL_CONNECTED) {
    configTzTime("CST6CDT,M3.2.0,M11.1.0", "utcnist2.colorado.edu", "0.us.pool.ntp.org");
  }
}

void stop_wifi() {
  client.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void reconnect_mqtt() {
  int attempts = 0;
  while (!client.connected() && attempts < 3) {
    if (client.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS)) break;
    attempts++;
    delay(500);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  analogReadResolution(12); 

  esp_pm_config_esp32s3_t pm_config = {
    .max_freq_mhz = 240, 
    .min_freq_mhz = 40,  
    .light_sleep_enable = true 
  };
  esp_pm_configure(&pm_config);

  setup_wifi();
  stop_wifi();

  client.setServer(MQTT_SERVER, 1883);

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {.clk = I2S_MIC_SERIAL_CLOCK, .din = I2S_MIC_DATA_IN, .invert_flags = {.clk_inv = false}},
  };
  i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg);
  i2s_channel_enable(rx_handle);
}

void loop() {
  int16_t samples[BUFFER_SIZE];
  size_t bytes_read = 0;

  if (i2s_channel_read(rx_handle, samples, sizeof(samples), &bytes_read, portMAX_DELAY) == ESP_OK) {
    int num_samples = bytes_read / sizeof(int16_t);
    if (num_samples > 0) {
      double sum_sq = 0;
      int zero_crossings = 0;
      float prev_centered = 0;

      for (int i = 0; i < num_samples; i++) {
        dc_offset = (dc_offset * OFFSET_ALPHA) + (samples[i] * (1.0 - OFFSET_ALPHA));
        float centered = (float)samples[i] - dc_offset;
        sum_sq += (centered * centered);
        if ((prev_centered > 0 && centered <= 0) || (prev_centered < 0 && centered >= 0)) zero_crossings++;
        prev_centered = centered;
      }

      float current_db = 20.0 * log10(sqrt(sum_sq / num_samples) / REFERENCE_16BIT);
      float frequency_hz = (zero_crossings / 2.0) / ((float)num_samples / SAMPLE_RATE);
      float currentBatt = readBattery();

      if (current_db < ambient_db + 5.0) ambient_db = (ambient_db * 0.99) + (current_db * 0.01);

      // Low Battery Local Blink
      if (currentBatt < LOW_BATT_CUTOFF) {
        if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
          lastBlinkTime = millis();
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState);
        }
      }

      // Voice Activity Detection Trigger
      if (current_db > (ambient_db + SIGNAL_BOOST) && frequency_hz > MIN_FREQ) {
        if (!soundIsActive) {
          soundStartTime = millis();
          soundIsActive = true;
          eventSent = false;
        }

        if (millis() - soundStartTime > REQUIRED_DUR_MS && !eventSent) {
          if (currentBatt >= LOW_BATT_CUTOFF) digitalWrite(LED_PIN, LOW);

          setup_wifi();
          if (WiFi.status() == WL_CONNECTED) {
            int32_t rssi = WiFi.RSSI();
            reconnect_mqtt();
            if (client.connected()) {
              JsonDocument doc;
              doc["DeviceID"] = MQTT_CLIENT;
              doc["Waqt"] = getWaqt();
              doc["Freq_Hz"] = round(frequency_hz);
              doc["Loudness_dB"] = round(current_db);
              doc["Batt_V"] = serialized(String(currentBatt, 2));
              doc["RSSI"] = rssi;

              char buffer[256];
              serializeJson(doc, buffer);
              client.publish(MQTT_TOPIC, buffer);
            }
          }
          stop_wifi();
          eventSent = true;
        }
      } else if (currentBatt >= LOW_BATT_CUTOFF) {
        digitalWrite(LED_PIN, HIGH);
        soundIsActive = false;
      }
    }
  }
}