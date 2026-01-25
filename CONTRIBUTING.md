# Contributing to Hardware Validation

Thank you for helping improve the reliability of the solutions presented here. To maintain the highest standards of hardware traceability, please follow these guidelines.

## 🛠️ Developmental Philosophy
We use a **Milestone-Based Frozen Baseline** approach. 
1. **Never** modify a core milestone file to add a new feature.
2. **Instead**, create a new milestone (e.g., `Milestone 6`) to maintain the diagnostic history.

## 📬 Pull Request Process
- Ensure the sketch includes the standard professional header and MIT license.
- Verify that the sketch compiles via the `XIAO S3 Build` GitHub Action.
- Document the specific hardware revision (e.g., v1.1) used for testing.

---

# 🔍 Troubleshooting Hardware Failures

If a milestone test fails, use the following matrix to identify the likely hardware failure point.

### 1. Camera Initialization Failures (`0x105`, `0x102`)
| Symptom | Potential Root Cause | Fix |
| :--- | :--- | :--- |
| `0x105` (No Probe) | Ribbon cable is skewed or not fully seated. | Re-seat the FPC connector; check for dust in the socket. |
| `0x102` (No Sensor) | I2C pull-up resistors failing or bus contention. | Ensure no other I2C devices are on Pins 39/40 during init. |



### 2. PSRAM & Memory Failures
| Symptom | Potential Root Cause | Fix |
| :--- | :--- | :--- |
| `NULL` Frame Buffer | OPI PSRAM chip is not receiving enough power. | Check USB-C cable quality; use a powered hub. |
| Image Corruption | Timing skew on the 80MHz OPI bus. | Lower `xclk_freq_hz` to 10-15MHz in the config. |



### 3. Concurrency & RF Failures (Milestone 3 & 4)
| Symptom | Potential Root Cause | Fix |
| :--- | :--- | :--- |
| Reboot during SD Write | Current spike causing a Voltage Sag. | Add a 10uF - 47uF capacitor across 3V3 and GND. |
| "Rolling Bars" on Image | RF Noise from Wi-Fi Antenna bleeding into DVP. | Re-route the antenna away from the camera ribbon cable. |
| SD Mount Fail | SPI Bus conflict with Camera XCLK. | Ensure SD is initialized in 1-bit mode for the Sense board. |



### 4. Deep Sleep & Stability (Milestone 5)
| Symptom | Potential Root Cause | Fix |
| :--- | :--- | :--- |
| Stuck after Wakeup | Camera reset logic failed to clear internal state. | Toggle the Camera Power pin (if available) before `esp_camera_init`. |
| Watchdog Trigger | Infinite loop in the TCP/IP stack or MQTT wait. | Ensure `client.loop()` is called frequently; check Wi-Fi signal. |

---

## 🏗️ Technical Standards
All contributions must adhere to:
- **Encoding:** UTF-8 (for Unicode icon support).
- **Versioning:** Semantic Versioning (SemVer) 2.0.0.
- **Documentation:** Doxygen-style headers for all functions.
