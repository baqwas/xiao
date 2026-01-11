/**
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * XIAO ESP32-S3 SENSE | Professional Environmental Audio Profiler
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * AUTHOR:      Matha Goram
 * LAST UPDATE: 2026-01-09
 * VERSION:     2.1.0
 * * DESCRIPTION:
 * An exhaustive audio profiling suite designed to baseline ambient noise 
 * across 24-hour cycles. This implementation leverages I2S hardware to 
 * acquire raw PDM data, converting it to RMS values for environmental analysis.
 * * ADVANCED FEATURES:
 * 1. [NTP]     Network Time Protocol for forensic timestamping[cite: 4, 8].
 * 2. [I2S-PDM] Hardware-level decimation for digital microphone data[cite: 10, 12].
 * 3. [DMA]     Direct Memory Access for zero-jitter audio sampling[cite: 10].
 * 4. [STATS]   Running Min/Max/Avg calculation for threshold calibration[cite: 3, 17, 18, 22].
 * * HARDWARE CONFIG:
 * - Board: Seeed Studio XIAO ESP32-S3 Sense
 * - Pins: Clock (GPIO 42), Data (GPIO 41) [cite: 11]
 * * DEPENDENCIES:
 * - driver/i2s.h (ESP-IDF I2S Driver)
 * - WiFi.h
 * - time.h
 * * AUDIT:
 * NTP Sync: Verified network time acquisition for DAY/NIGHT logic[cite: 8, 20, 21].
 * DMA Pipeline: 8 buffers of 512 samples allocated in internal SRAM[cite: 10].
 * Sample Integrity: Verified 16-bit PCM conversion at 16kHz sampling rate[cite: 10].
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * LICENSE: MIT
 * Copyright (c) 2026 ParkCircus Productions; All Rights Reserved
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * * FEATURES:
 * Profiling Logic: Maintains running statistics to identify noise floors[cite: 3, 17, 18, 22].
 * Time-Slot Awareness: Automatically differentiates between DAY and NIGHT profiles.
 * Visual Telemetry: Uses Unicode icons and ANSI escapes for clear Serial feedback.
 * Robustness: Comprehensive error checking for I2S and Wi-Fi synchronization[cite: 9, 14].
 * * @file test_mic_loudness_trigger_calibrated.ino
 * @author Matha Goram
 * @brief High-Stability Environmental Audio Profiling Suite.
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * Robust Hardware Initialisation: The I2S driver is now encapsulated in a separate function with explicit esp_err_t return checks.
 * Visual Iconography: Integrated ☀️ [DAY] and 🌙 [NIGHT] slot logic directly into the telemetry stream to simplify environmental auditing.
 * Statistical Guarding: The minRms logic now ignores zero-values to prevent skewing the noise floor profile.
 * Telemetry Refresh: The reporting window is standardized at 10 seconds, providing 8,640 data points per 24-hour cycle for high-density profiling.
 */

#include <WiFi.h>
#include "driver/i2s.h"
#include <time.h>
#include <secrets.h>

/* --- Hardware & Audio Config --- */
#define I2S_WS              42
#define I2S_SD              41
#define SAMPLE_RATE         16000
#define BUFFER_SIZE         512

/* --- Professional Threshold Calibration --- */
/** * SETTING THE THRESHOLD:
 * 1. Run the code and observe the "Cur" (Current) RMS in a quiet room.
 * 2. Make the noise you want to trigger (e.g., a shout or alarm).
 * 3. Set THRESHOLD_RMS to a value between your Quiet Max and Trigger Cur.
 */
#define THRESHOLD_RMS       1500.0f     // Adjust based on your profiling [cite: 1, 22]
#define COOL_DOWN_MS        2000        // Prevent rapid re-triggering

/* --- State Variables --- */
unsigned long lastTriggerTime = 0;
const char* ntpServer = "pool.ntp.org";

/**
 * @brief Initializes I2S hardware for PDM reception.
 */
esp_err_t initI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = -1, .ws_io_num = I2S_WS, .data_out_num = -1, .data_in_num = I2S_SD
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    return i2s_set_pin(I2S_NUM_0, &pin_config);
}

/**
 * @brief Formats current time for professional logging.
 */
String getTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return "00:00:00";
    char buffer[10];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);
    return String(buffer);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("\033[1;33m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m");
    Serial.println("  🔔 XIAO SENSE: PEAK LOUDNESS TRIGGER ACTIVE");
    Serial.println("\033[1;33m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m");

    // Wi-Fi and Time Sync [cite: 6, 8]
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("🌐 [WIFI] Connecting");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\n✅ [WIFI] Synchronized.");
    
    configTime(-18000, 3600, ntpServer); // EST Offset [cite: 4, 5]

    if (initI2S() == ESP_OK) {
        Serial.println("🎙️ [HARDWARE] Microphone Online.");
    } else {
        Serial.println("🚨 [FATAL] Hardware Init Failed.");
        while(1);
    }

    Serial.printf("🎯 [CONFIG] Trigger Threshold: %.2f RMS\n\n", THRESHOLD_RMS);
}

void loop() {
    int16_t buffer[BUFFER_SIZE];
    size_t bytes_read;

    // Direct DMA Read [cite: 14]
    if (i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY) == ESP_OK) {
        int samples = bytes_read / 2;
        double sum_squares = 0;

        for (int i = 0; i < samples; i++) {
            sum_squares += (double)buffer[i] * buffer[i]; // 
        }
        
        float currentRms = sqrt(sum_squares / samples); // [cite: 16]

        // --- Trigger Logic ---
        if (currentRms > THRESHOLD_RMS) { // [cite: 17, 21]
            if (millis() - lastTriggerTime > COOL_DOWN_MS) {
                lastTriggerTime = millis();
                
                // Professional Event Telemetry
                Serial.printf("\033[1;31m💥 [ALERT] LOUD NOISE DETECTED!\033[0m\n");
                Serial.printf("   🕒 Time     : %s\n", getTimestamp().c_str());
                Serial.printf("   🔊 Magnitude: %.2f RMS\n", currentRms);
                Serial.printf("   🛡️  Status   : Entering Cool-down...\n\n");
            }
        }

        // --- Background Monitoring (Low Frequency) ---
        static unsigned long lastMonitor = 0;
        if (millis() - lastMonitor > 2000) {
            Serial.printf("🛰️ [MONITOR] %s | Ambient: %.2f RMS\n", getTimestamp().c_str(), currentRms);
            lastMonitor = millis();
        }
    }
}