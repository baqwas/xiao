/**
 * @file test_mic_alert.ino
 * @author Matha Goram
 * @brief Adaptive Voice Activity Detection (VAD) with MQTT "Burst" reporting for XIAO ESP32-S3 Sense.
 * * --- WORKFLOW DESCRIPTION ---
 * 1. INITIALIZATION: Sets up hardware and performs a brief WiFi connection to synchronize 
 * [cite_start]the system clock via NTP using a POSIX TZ string[cite: 5, 7, 13].
 * 2. POWER MANAGEMENT: Shuts down WiFi radio after time sync to minimize noise interference 
 * [cite_start]with the high-sensitivity PDM microphone[cite: 9, 14].
 * 3. SIGNAL PROCESSING: Samples audio at 16kHz via I2S, applies a DC-offset removal filter, 
 * [cite_start]and calculates RMS Loudness (dB) and Zero-Crossing Frequency (Hz)[cite: 15, 16, 21, 24].
 * 4. ADAPTIVE SENSING: Continuously tracks the ambient noise floor to maintain a floating 
 * [cite_start]trigger threshold, ensuring robustness in changing environments[cite: 25].
 * 5. VALIDATION: Filters out transient peaks (clicks/pops) by requiring the sound to 
 * [cite_start]persist for a specified duration before triggering[cite: 27].
 * 6. BURST REPORTING: Upon a valid event, the system wakes WiFi, connects to MQTT, 
 * [cite_start]publishes a JSON payload with a timestamp (Waqt), and powers down again[cite: 28, 29, 31, 32].
 * * --- OPTIMIZATION PARAMETERS ---
 * - [cite_start]SIGNAL_BOOST: dB delta above noise floor required to trigger (Default: 12.0)[cite: 26].
 * - [cite_start]REQUIRED_DUR_MS: Minimum duration in ms to qualify as a valid event (Default: 150)[cite: 27].
 * - [cite_start]MIN_FREQ: High-pass frequency filter in Hz (Default: 400.0)[cite: 26].
 * - [cite_start]OFFSET_ALPHA: Smoothing factor for DC-offset tracking (Default: 0.999)[cite: 20].
 * * --- HARDWARE REQUIREMENTS ---
 * - [cite_start]Board: Seeed Studio XIAO ESP32-S3 Sense[cite: 16].
 * - [cite_start]Pins: SCLK (GPIO 42), DIN (GPIO 41)[cite: 16].
 * * --- LICENSE ---
 * MIT License
 * * Copyright (c) 2026 ParkCircus Productions; All Rights Reserved
 * * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include "driver/i2s_pdm.h"
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

// Hardware Pins for XIAO ESP32-S3 Sense
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
bool soundIsActive = false;
bool eventSent = false;

// Function to get formatted timestamp
String getWaqt() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Time_Error";
  }
  char timeString[25];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeString);
}

void setup_wifi() {
  Serial.print("Waking WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.disconnect(true);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(MQTT_CLIENT); 
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(100);
    Serial.print(".");
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected.");
    configTzTime("CST6CDT,M3.2.0,M11.1.0", "utcnist2.colorado.edu", "0.us.pool.ntp.org");
  } else {
    Serial.println("\nWiFi Connect Failed.");
  }
}

void stop_wifi() {
  client.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi Powered Down.");
}

void reconnect_mqtt() {
  int attempts = 0;
  while (!client.connected() && attempts < 3) {
    Serial.print("Connecting MQTT...");
    if (client.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      attempts++;
      delay(500);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Off

  // Initial connection to sync time and verify network
  setup_wifi();
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("Initial Sync Complete.");
    delay(1000); // Allow time for NTP sync
  }
  stop_wifi();

  client.setServer(MQTT_SERVER, 1883);

  // Initialize I2S PDM
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);

  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .clk = I2S_MIC_SERIAL_CLOCK,
      .din = I2S_MIC_DATA_IN,
      .invert_flags = { .clk_inv = false }
    },
  };

  i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg);
  i2s_channel_enable(rx_handle);
}

void loop() {
  int16_t samples[BUFFER_SIZE];
  size_t bytes_read = 0;

  if (i2s_channel_read(rx_handle, samples, sizeof(samples), &bytes_read, 100) == ESP_OK) {
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

      if (current_db < ambient_db + 5.0) {
        ambient_db = (ambient_db * 0.99) + (current_db * 0.01);
      }

      if (current_db > (ambient_db + SIGNAL_BOOST) && frequency_hz > MIN_FREQ) {
        if (!soundIsActive) {
          soundStartTime = millis();
          soundIsActive = true;
          eventSent = false;
        }

        if (millis() - soundStartTime > REQUIRED_DUR_MS && !eventSent) {
          digitalWrite(LED_PIN, LOW); // Visual alert start

          // Burst WiFi connection
          setup_wifi();
          if (WiFi.status() == WL_CONNECTED) {
            reconnect_mqtt();
            if (client.connected()) {
              JsonDocument doc;
              doc["DeviceID"] = MQTT_CLIENT;
              doc["Waqt"] = getWaqt();
              doc["Freq_Hz"] = round(frequency_hz);
              doc["Loudness_dB"] = round(current_db);

              char buffer[256];
              serializeJson(doc, buffer);
              client.publish(MQTT_TOPIC, buffer);
              Serial.println("Event Published Successfully.");
            }
          }
          stop_wifi();
          
          eventSent = true; 
        }
      } else {
        digitalWrite(LED_PIN, HIGH);
        soundIsActive = false;
      }
    }
  }
}