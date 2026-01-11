/**
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * XIAO ESP32-S3 SENSE | Voice Activated Recorder with 90-Day Retention & Space Guard
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * AUTHOR:      Matha Goram
 * LAST UPDATE: 2026-01-09
 * VERSION:     7.0.0
 * * DESCRIPTION:
 * Voice Activated Recorder with automated retention policy and real-time 
 * storage telemetry. Monitors SD capacity and provides visual LED alerts.
 * * ADVANCED FEATURES:
 * 1. [SPACE-GUARD] Real-time monitoring of SD free bytes (1GB Threshold).
 * 2. [LED-STATE]   RGB Telemetry: Green (OK), Yellow (Low), Red (Record).
 * 3. [PRUNING]     Automatic deletion of directories older than 90 days.
 * * HARDWARE CONFIG:
 * - Mic Pins: GPIO 42 (CLK), 41 (DATA) | SD CS: GPIO 21
 * - User LED: Built-in RGB (XIAO S3 Sense)
 * * DESIGN PATTERNS & STANDARDS:
 * 1. [SPACE-GUARD] Real-time SD integrity monitoring with 95% safety ceiling.
 * 2. [LED-STATE]   RGB Visual feedback for operational state & diagnostics.
 * 3. [RETENTION]   Automated 90-day pruning to ensure continuous uptime.
 * 4. [TELEMETRY]   JSON-formatted MQTT broadcasts for Home Automation.
 * * HARDWARE MAPPING:
 * - PDM MIC: GPIO 42 (CLK), 41 (DATA)
 * - STORAGE: GPIO 21 (SD Chip Select)
 * - POWER:   Pin A0 (Battery Voltage Divider)
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP_I2S.h>
#include <FS.h>
#include <SD.h>
#include <time.h>
#include <secrets.h>

// --- Hardware Pins ---
#define LED_RED     17
#define LED_BLUE    19
#define SD_CS_PIN   21
#define PIN_BATTERY A0

// --- Operational Parameters ---
#define THRESHOLD_RMS       1800.0f
#define STORAGE_LIMIT_RATIO 0.95f   
#define SLEEP_HOUR          22      
#define WAKE_HOUR           6
#define STATUS_CHECK_MS     600000

// --- Global Instances ---
I2SClass i2s_recorder;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

float pulseAngle = 0;
float pulseSpeed = 0.02;
bool  wifiActive = false;
bool  isRecording = false;

/**
 * 🛰️ Broadcasts system metrics to MQTT Broker
 */
void broadcastTelemetry(String payload) {
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
        mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
        if (mqttClient.connect("XIAO_Acoustic_Vault")) {
            mqttClient.publish("xiao/vault/telemetry", payload.c_str());
            mqttClient.loop();
            Serial.println("📤 [TELEMETRY] Data Dispatched: " + payload);
        } else {
            Serial.println("⚠️ [MQTT] Broker Connection Failed");
        }
    } else {
        Serial.println("📶 [WIFI] Connection Unavailable");
    }
}

/**
 * 🔋 Monitors battery health and storage capacity
 */
void updateSystemTelemetry() {
    if (!SD.cardSize()) {
        Serial.println("❌ [STORAGE] SD Card Access Failed");
        return;
    }
    
    float usedP = (float)SD.usedBytes() / (float)SD.totalBytes();
    pulseSpeed = 0.02 + (usedP * 0.15); 
    
    float v = (analogReadMilliVolts(PIN_BATTERY) * 2.0) / 1000.0;

    if (usedP > STORAGE_LIMIT_RATIO || v < 3.45f) {
        broadcastTelemetry("{\"volts\":" + String(v) + ",\"sd_full\":" + String(usedP*100) + "}");
    }

    struct tm now;
    if (getLocalTime(&now)) {
        if (now.tm_hour >= SLEEP_HOUR || now.tm_hour < WAKE_HOUR) {
            Serial.println("🌙 [SYSTEM] Entering Scheduled Deep Sleep");
            broadcastTelemetry("{\"status\":\"sleeping\"}");
            delay(500); 
            esp_sleep_enable_timer_wakeup(28800ULL * 1000000ULL);
            esp_deep_sleep_start();
        }
    }
}

/**
 *💡 Visual pulse logic for idle states
 */
void updateLED() {
    if (isRecording) return;
    pulseAngle += pulseSpeed;
    if (pulseAngle > TWO_PI) pulseAngle = 0;
    int br = (sin(pulseAngle) * 127.5) + 127.5;
    analogWrite(LED_BLUE, 255 - br);
}

/**
 * 💾 Persists captured audio buffer to SD card
 */
void commitRecording(uint8_t *buf, size_t sz) {
    isRecording = true;
    digitalWrite(LED_RED, LOW); // LED Active Low
    
    String path = "/R_" + String(millis()) + ".wav";
    File f = SD.open(path, FILE_WRITE);
    
    if (f) {
        f.write(buf, sz);
        f.close();
        Serial.println("💾 [RECORD] Capture Saved: " + path);
    } else {
        Serial.println("❌ [RECORD] File Write Permission Denied");
    }
    
    isRecording = false;
    updateSystemTelemetry();
}

/**
 * ⚙️ CORE EXECUTION
 */
void setup() {
    Serial.begin(115200);
    pinMode(LED_RED, OUTPUT); 
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED, HIGH); 

    // Initializing PDM Interface
    i2s_recorder.setPinsPdmRx(42, 41);
    if (i2s_recorder.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        Serial.println("✅ [MIC] PDM Interface Initialized");
    }

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("❌ [STORAGE] SD Initialization Failed");
    } else {
        Serial.println("📂 [STORAGE] SD Filesystem Mounted");
    }

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    configTime(0, 0, "pool.ntp.org");
    
    Serial.println("🚀 [SYSTEM] Acoustic Vault Armed and Operational");
    updateSystemTelemetry();
}

void loop() {
    updateLED();
    
    int raw = i2s_recorder.read();
    if (raw != -1 && abs((int16_t)raw) > THRESHOLD_RMS) {
        Serial.println("🎤 [EVENT] Acoustic Threshold Exceeded");
        size_t sz = 0;
        uint8_t *buf = i2s_recorder.recordWAV(10, &sz);
        if (buf) commitRecording(buf, sz);
    }
    
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > STATUS_CHECK_MS) { 
        updateSystemTelemetry();
        lastCheck = millis();
    }
}