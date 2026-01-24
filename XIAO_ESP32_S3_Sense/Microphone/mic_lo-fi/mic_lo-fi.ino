/**
 * @file   test_mic_lo-fi.ino
 * @author  Matha Goram
 * @version 1.1.0
 * @date    January 22, 2026
 * * @brief   High-sensitivity PDM Microphone Diagnostic and Visualizer for XIAO ESP32-S3 Sense.
 * This firmware utilizes the I2S peripheral to capture PDM (Pulse Density Modulation)
 * audio data, calculates peak-to-peak amplitude for real-time serial plotting, 
 * and implements an adaptive threshold for Voice Activity Detection (VAD) via the onboard LED.
 * * @hardware_target: Seeed Studio XIAO ESP32-S3 Sense (Expansion Board required for Mic)
 * @peripheral_usage: I2S0 (PDM Mode), GPIO 41 (Data), GPIO 42 (Clock), GPIO 21 (Built-in LED)
 * * @workflow_overview:
 * 1. Initialization: Resets the I2S driver to clear potential DMA hangs from warm boots.
 * 2. Acquisition: Captures 512-sample blocks of 16-bit PCM data converted from the PDM stream.
 * 3. Analysis: Identifies Min/Max peaks within the buffer to determine the signal's dynamic range.
 * 4. Scaling: Applies a bit-shift gain (VOL_BOOST) to map low-level PDM signals to a 4-digit range.
 * 5. Adaptive VAD: Implements a Low-Pass Filter (LPF) to track the ambient noise floor.
 * 6. Visual Feedback: Compares instantaneous volume against the adaptive baseline to trigger the LED.
 * * @tweaking_guide:
 * [DATA ACQUISITION]
 * - SAMPLE_RATE: Higher values (e.g., 32000) increase frequency resolution but consume more CPU.
 * - STACK_SIZE: Larger buffers (e.g., 1024) provide more stable peak detection but increase latency.
 * * [DATA ANALYSIS]
 * - VOL_BOOST: Adjust (5-8) to control the "height" of the spikes in the Serial Plotter.
 * - sensitivity: Adjust (1.5 - 4.0) to change how much louder than the room a sound must be
 * to trigger the LED. Lower is more sensitive; higher reduces false positives.
 * - LPF Factor (0.95): Lowering to 0.80 makes the LED adapt to new room noise faster but may
 * mask sustained speech.
 * @license MIT
 */

#include <driver/i2s_pdm.h>

// Pins for XIAO ESP32-S3 Sense
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_42
#define I2S_MIC_DATA_IN      GPIO_NUM_41

#define SAMPLE_RATE 20000 
#define STACK_SIZE  512   
#define VOL_BOOST   7     

// Global handle for the I2S RX channel
i2s_chan_handle_t rx_handle = NULL;

// Variables for the LED logic
float ambientNoiseLevel = 500.0; 
const float sensitivity = 2.5;

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for Serial on S3

  // LED_BUILTIN on XIAO ESP32-S3 is Active LOW
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  // --- 1. Create I2S Channel ---
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

  // --- 2. Configure PDM RX Mode ---
  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .clk = I2S_MIC_SERIAL_CLOCK,
      .din = I2S_MIC_DATA_IN,
      .invert_flags = { .clk_inv = false }
    },
  };

  // --- 3. Initialize and Start ---
  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

  Serial.println(">>> MODERN I2S DRIVER ACTIVE <<<");
}

void loop() {
  int16_t samples[STACK_SIZE];
  size_t bytesRead = 0;

  // Use the new channel-based read function
  if (i2s_channel_read(rx_handle, samples, sizeof(samples), &bytesRead, 100) == ESP_OK && bytesRead > 0) {
    int16_t minVal = 32767;
    int16_t maxVal = -32768;

    int numSamples = bytesRead / sizeof(int16_t);
    for (int i = 0; i < numSamples; i++) {
      if (samples[i] < minVal) minVal = samples[i];
      if (samples[i] > maxVal) maxVal = samples[i];
    }

    int32_t delta = (int32_t)maxVal - minVal;
    int32_t output = delta << VOL_BOOST;

    // --- Dynamic LED Feedback ---
    ambientNoiseLevel = (ambientNoiseLevel * 0.95) + (output * 0.05);

    if (output > (ambientNoiseLevel * sensitivity) && output > 2000) {
      digitalWrite(LED_BUILTIN, LOW);  // ON
    } else {
      digitalWrite(LED_BUILTIN, HIGH); // OFF
    }

    // Serial Plotter output
    if (output > 25000) output = 25000;
    Serial.println(output);

  } else {
    Serial.println(0); 
  }

  delay(25); 
}