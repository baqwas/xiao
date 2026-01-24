/**
 * @file test_mic_deep_sleep_life.ino
 * @author Gemini AI (Collaborative Development)
 * @brief Deep Sleep Pulse-VAD with Estimated Remaining Battery Life Reporting.
 * * --- WORKFLOW ---
 * 1. Wake from Deep Sleep.
 * 2. Pulse-listen for audio (500ms). Update RTC-persistent noise baseline.
 * 3. Calculate "Days of Life" based on duty cycle and current battery voltage.
 * 4. If sound detected: Burst WiFi/MQTT with JSON (Audio + Battery + Life Est).
 * 5. Hibernate via Deep Sleep.
 * * --- LICENSE ---
 * MIT License (c) 2024
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include "driver/i2s_pdm.h"
#include <math.h>
#include "secrets.h"

// --- ADJUSTABLE PARAMETERS ---
#define SLEEP_DURATION_SEC  30    // Increase for longer life (e.g., 60 or 300)
#define LISTEN_WINDOW_MS    500   // Time spent listening per wake
#define BATT_CAPACITY_MAH   350   // Your battery size in mAh
#define CURRENT_AWAKE_MA    30.0  // Avg mA while listening
#define CURRENT_SLEEP_MA    0.02  // Avg mA in Deep Sleep (20uA)

// Audio/VAD Settings
#define SAMPLE_RATE     16000
#define BUFFER_SIZE     512    
#define OFFSET_ALPHA    0.999  
#define REFERENCE_16BIT 32768.0
#define SIGNAL_BOOST    12.0  
#define MIN_FREQ        400.0 

// Hardware
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_42
#define I2S_MIC_DATA_IN      GPIO_NUM_41
#define LED_PIN              LED_BUILTIN 
#define BATTERY_PIN          1

// RTC Memory (Survives Sleep)
RTC_DATA_ATTR float ambient_db_history = -60.0;

#define MQTT_CLIENT "XIAO1"

i2s_chan_handle_t rx_handle = NULL;
WiFiClient espClient;
PubSubClient client(espClient);

float readBattery() {
  uint32_t raw = analogRead(BATTERY_PIN);
  return (float)raw * 2.0 * 3.3 / 4095.0; 
}

// Estimates days of life remaining based on the current duty cycle
float estimateDaysLeft(float currentVolts) {
  // Rough LiPo discharge: 4.2V (100%) to 3.4V (0% usable for this logic)
  float chargePercent = (currentVolts - 3.4) / (4.2 - 3.4);
  if (chargePercent > 1.0) chargePercent = 1.0;
  if (chargePercent < 0.0) chargePercent = 0.0;

  float remaining_mAh = BATT_CAPACITY_MAH * chargePercent;
  
  // Duty Cycle Math
  float t_awake = (float)LISTEN_WINDOW_MS / 1000.0;
  float t_sleep = (float)SLEEP_DURATION_SEC;
  float duty_cycle = t_awake / (t_awake + t_sleep);
  
  float avg_current_ma = (CURRENT_AWAKE_MA * duty_cycle) + (CURRENT_SLEEP_MA * (1.0 - duty_cycle));
  
  float hours_left = remaining_mAh / avg_current_ma;
  return hours_left / 24.0;
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) delay(100);
  if(WiFi.status() == WL_CONNECTED) {
    configTzTime("CST6CDT,M3.2.0,M11.1.0", "utcnist2.colorado.edu", "0.us.pool.ntp.org");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  analogReadResolution(12);

  // Initialize I2S
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {.clk = I2S_MIC_SERIAL_CLOCK, .din = I2S_MIC_DATA_IN, .invert_flags = {.clk_inv = false}},
  };
  i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg);
  i2s_channel_enable(rx_handle);

  bool soundDetected = false;
  float dc_offset = 0;
  unsigned long listenStart = millis();
  float final_db = 0, final_hz = 0;

  // Pulse Listening Window
  while (millis() - listenStart < LISTEN_WINDOW_MS) {
    int16_t samples[BUFFER_SIZE];
    size_t bytes_read = 0;
    if (i2s_channel_read(rx_handle, samples, sizeof(samples), &bytes_read, 100) == ESP_OK) {
      int num_samples = bytes_read / sizeof(int16_t);
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
      final_db = 20.0 * log10(sqrt(sum_sq / num_samples) / REFERENCE_16BIT);
      final_hz = (zero_crossings / 2.0) / ((float)num_samples / SAMPLE_RATE);

      if (final_db < ambient_db_history + 5.0) ambient_db_history = (ambient_db_history * 0.95) + (final_db * 0.05);
      if (final_db > (ambient_db_history + SIGNAL_BOOST) && final_hz > MIN_FREQ) {
        soundDetected = true;
        break; 
      }
    }
  }

  if (soundDetected) {
    digitalWrite(LED_PIN, LOW);
    setup_wifi();
    if (WiFi.status() == WL_CONNECTED) {
      client.setServer(MQTT_SERVER, 1883);
      if (client.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS)) {
        float vBatt = readBattery();
        JsonDocument doc;
        doc["DeviceID"] = MQTT_CLIENT;
        doc["Batt_V"] = serialized(String(vBatt, 2));
        doc["Days_Left"] = serialized(String(estimateDaysLeft(vBatt), 1));
        doc["Freq_Hz"] = round(final_hz);
        doc["RSSI"] = WiFi.RSSI();
        
        char buffer[256];
        serializeJson(doc, buffer);
        client.publish(MQTT_TOPIC, buffer);
        client.disconnect();
      }
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  i2s_channel_disable(rx_handle);
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() {}