# XIAO ESP32-S3 Series: Modular Project Collection
### A Stepwise Approach to ESP32-S3 Development

Welcome to the central repository for projects targeting the **Seeed Studio XIAO ESP32-S3** ecosystem. This collection is designed with modularity in mind, separating core hardware capabilities from complex integrated solutions.

---

## 🚀 Hardware Ecosystem
This repository supports three specific variants of the XIAO S3 family:

| Board | Core Features | Primary Focus |
| :--- | :--- | :--- |
| **XIAO ESP32-S3** | Dual-core, Wi-Fi/BLE, Small Form Factor | IoT Nodes, TinyML |
| **XIAO ESP32-S3 Sense** | Camera (OV2640), Digital Mic, SD Slot | Vision, Voice, Logging |
| **XIAO ESP32-S3 Plus** | Expanded Pins, Battery Management | Handhelds, Complex I/O |

---

## 📁 Repository Structure
Following the principle of "Separation of Concerns," the code is organized by board type and then by functional category.

* **`XIAO_ESP32_S3/`**: Generic IoT and Wi-Fi sketches.
* **`XIAO_ESP32_S3_Sense/`**: Multimedia and storage-heavy projects.
    * `Camera/`: Snapshot and streaming examples.
    * `Microphone/`: PDM audio capture.
    * `SD_Card/`: File system operations and logging.
    * **`Solutions/`**: End-to-end applications (e.g., Doorbell, Intrusion Monitor).
* **`XIAO_ESP32_S3_Plus/`**: Advanced pinout and peripheral integration.

---
