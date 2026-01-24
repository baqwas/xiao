/**
 * @file test_mic_smooth_wave.ino
 * @author Gemini AI (Collaborative Development)
 * @brief DMA-driven PDM Audio Streamer for XIAO ESP32-S3 Sense.
 * * --- PROCESSING WORKFLOW ---
 * 1. INITIALIZATION: Configures the I2S PDM (Pulse Density Modulation) peripheral 
 * in Master-Receive mode
 * 2. DMA MANAGEMENT: Utilizes the Direct Memory Access (DMA) engine to fill 
 * internal ring buffers without CPU intervention
 * 3. BUFFER PROCESSING: Once the buffer is full, the CPU processes the entire block
 * This ensures a stable, continuous stream that accurately represents sound wave shapes
 * 4. SERIAL TELEMETRY: Batched audio samples are decimated (1:4 ratio) and streamed 
 * via UART at 115200 baud for real-time visualization in the Serial Plotter
 * * --- OPTIMIZATION & PARAMETERS ---
 * - SAMPLE_RATE: 16000 Hz (Standard for voice/low-frequency analysis)
 * - BUFFER_SIZE: 512 samples (Balances latency vs. processing stability)
 * - DECIMATION: % 4 (Keeps the Serial Plotter fluid and responsive)
 * * --- HARDWARE ASSUMPTIONS ---
 * - Board: XIAO ESP32-S3 Sense (integrated digital PDM microphone).
 * - Pins: Clock (GPIO 42), Data (GPIO 41)
 * - Driver: ESP-IDF I2S PDM v5.x (Required for ESP32 Arduino Core v3.0+).
 * * --- LICENSE ---
 * MIT License
 * Copyright (c) 2026 ParkCircus Productions; All Rights Reserved
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include "driver/i2s_pdm.h"

// Configuration Constants
#define SAMPLE_RATE     16000
#define BUFFER_SIZE     512   
#define I2S_MIC_SCLK    GPIO_NUM_42
#define I2S_MIC_DATA    GPIO_NUM_41

// Global I2S Handle
i2s_chan_handle_t rx_handle = NULL;

/**
 * @brief Validates hardware state and provides serial feedback.
 */
void check_err(esp_err_t result, const char* stage) {
  if (result != ESP_OK) {
    Serial.printf("❌ FAIL: %s | Error: %s\n", stage, esp_err_to_name(result));
    while (1) delay(1000); // Halt on critical failure
  } else {
    Serial.printf("✅ SUCCESS: %s\n", stage);
  }
}

void setup() {
  // Initialize Serial with a timeout to prevent hanging if not connected
  Serial.begin(115200);
  while (!Serial && millis() < 5000); 
  delay(1000);
  
  Serial.println("\n--- Initializing Robust Audio Ingestion ---");

  // 1. Channel Allocation
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  check_err(i2s_new_channel(&chan_cfg, NULL, &rx_handle), "I2S Channel Allocation");

  // 2. PDM RX Configuration
  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .clk = I2S_MIC_SCLK,
      .din = I2S_MIC_DATA,
      .invert_flags = { .clk_inv = false }
    },
  };

  // 3. Initialize & Enable Peripheral
  check_err(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg), "PDM Mode Init");
  check_err(i2s_channel_enable(rx_handle), "I2S Channel Enable");

  Serial.println("--- Stream Active: Ready for Serial Plotter ---\n");
  Serial.println("Audio_Signal"); 
}

void loop() {
  int16_t s_buffer[BUFFER_SIZE];
  size_t bytes_read = 0;

  // Attempt to read a block of data from the DMA engine
  esp_err_t result = i2s_channel_read(rx_handle, s_buffer, sizeof(s_buffer), &bytes_read, 100);

  if (result == ESP_OK && bytes_read > 0) {
    int samples_count = bytes_read / sizeof(int16_t);
    for (int i = 0; i < samples_count; i++) {
      // Sub-sample 1:4 for Serial Plotter performance
      if (i % 4 == 0) Serial.println(s_buffer[i]);
    }
  } 
  else if (result != ESP_OK) {
    // Report transient errors without halting the loop
    Serial.printf("⚠️ Read Error: %s\n", esp_err_to_name(result));
  }
}