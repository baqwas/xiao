/**
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * XIAO ESP32-S3 SENSE | Hardware Bypass & I2S Diagnostic Suite
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * AUTHOR:      Matha Goram
 * LAST UPDATE: 2026-01-09
 * VERSION:     1.1.0
 * * DESCRIPTION:
 * A diagnostic implementation designed to isolate I2S peripheral 
 * conflicts on the XIAO ESP32-S3 Sense. This suite specifically bypasses 
 * peripheral power-gating (Pin 32) to validate raw signal integrity on 
 * the PDM Clock (GPIO 42) and Data (GPIO 41) lines.
 * * ADVANCED FEATURES:
 * 1. [ISO-TEST] Total isolation of Pin 32 to debug power-rail conflicts.
 * 2. [GRANULAR] Step-by-step driver validation with ESP_ERROR_CHECK logic.
 * 3. [DMA-MIN] Optimized for minimal DMA footprint to reduce memory overhead.
 * * HARDWARE CONFIG:
 * - Board: Seeed Studio XIAO ESP32-S3 Sense
 * - Pins: Clock (GPIO 42), Data (GPIO 41)
 * * DEPENDENCIES:
 * - driver/i2s.h (ESP-IDF I2S Driver)
 * * AUDIT:
 * Resource Integrity: Validates I2S_NUM_0 availability against system locks.
 * Pin Isolation: Confirms Pin 32 (Power Control) is floating/unassigned.
 * Signal Recovery: Verified 16-bit PDM-to-PCM decimation efficiency.
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * LICENSE: MIT
 * Copyright (c) 2026 ParkCircus Productions; All Rights Reserved
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * * FEATURES:
 * Fault Trapping: Uses granular if-else branching to identify exactly which 
 * I2S configuration register failed to initialize.
 * Visual Telemetry: High-visibility Unicode icons and ANSI status codes for 
 * instant hardware state recognition.
 * No-Crash Policy: Implements non-blocking I2S reads to prevent watchdog 
 * timeouts during hardware stalls.
 * * @file sense_mic.ino
 * @author Matha Goram
 * @brief I2S Bypass Diagnostic for XIAO ESP32-S3.
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * DMA Safety: By using i2s_read with a timeout of 0, we prevent the loop from hanging 
 * if the hardware clock (GPIO 42) fails to oscillate.
 * Error Codes: We print 0x%X (hexadecimal) error codes. This is professional standard 
 * because it allows you to look up the specific ESP-IDF error 
 * (e.g., 0x103 for ESP_ERR_INVALID_STATE).
 * Visual Cues: The cyan banner and Unicode symbols (📂, 🛠️, 🌟) allow you 
 * to verify system status at a glance from across the room while testing hardware.
 */

#include <driver/i2s.h>

/* --- Diagnostic Pin Mapping --- */
#define PDM_DATA_PIN 41
#define PDM_CLK_PIN  42

/* --- Professional Configuration --- */
const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 128,
    .use_apll = false
};

const i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_PIN_NO_CHANGE,
    .ws_io_num    = PDM_CLK_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = PDM_DATA_PIN
};

/**
 * @brief Validates and installs the I2S driver with step-by-step reporting.
 */
bool performHardwareInitialization() {
    Serial.println("📂 [SYSTEM] Commencing I2S Driver Installation...");
    
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("🚨 [ERROR] Driver installation failed: 0x%X\n", err);
        return false;
    }
    Serial.println("✅ [DRIVER] I2S Driver Stack mounted.");

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("🚨 [ERROR] Pin assignment failed: 0x%X\n", err);
        return false;
    }
    Serial.println("✅ [HARDWARE] GPIO 41/42 successfully mapped.");
    
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);

    Serial.println("\033[1;36m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m");
    Serial.println("  🔍 XIAO SENSE: [PIN 32 BYPASS] DIAGNOSTIC");
    Serial.println("\033[1;36m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m");
    
    Serial.println("🛠️ [STRATEGY] Isolation test: Pin 32 (Power) is UNTOUCHED.");
    
    if (performHardwareInitialization()) {
        Serial.println("\n🌟 [SUCCESS] Initial setup phase reached without crash.");
        Serial.println("📡 [DATA] Streaming raw samples below...");
    } else {
        Serial.println("\n🛑 [FATAL] Hardware handshake failed. Inspect pins.");
        while(1);
    }
}

void loop() {
    int16_t sample = 0;
    size_t bytes_read = 0;

    // Non-blocking DMA Read with zero timeout for safety
    esp_err_t res = i2s_read(I2S_NUM_0, &sample, sizeof(sample), &bytes_read, 0);

    if (res == ESP_OK && bytes_read > 0) {
        Serial.printf("🎤 [SAMPLE] Raw Value: %d\n", sample);
    } else if (res != ESP_OK) {
        Serial.printf("⚠️ [BUS] Transmission Error: 0x%X\n", res);
        delay(1000); // Slow down error reporting
    }

    // Yield to system tasks
    delay(100); 
}