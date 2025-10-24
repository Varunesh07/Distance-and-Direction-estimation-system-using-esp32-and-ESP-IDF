# ESP32 Wi-Fi & BLE Scanner with Web Dashboard

## Overview
This project enables the ESP32 to scan surrounding Wi-Fi and BLE devices, filter noisy RSSI signals using a Kalman filter, estimate distances, and serve a dynamic dashboard via a local webserver.

It’s ideal for indoor tracking, signal analysis, and distance monitoring applications.

---

## 📁 File Structure

```
project_root/
├── main.c
├── webserver.c
├── server.h
└── README.md
```

---

## ⚙️ Components Overview

### 1. `main.c`
- Initializes Wi-Fi and BLE scanning.
- Calls signal processing logic (Kalman filter, RSSI smoothing).
- Handles target tracking and updates global structures.
- Launches the web server task.

### 2. `webserver.c`
- Hosts a simple HTTP server (ESP-IDF `esp_http_server`).
- Provides endpoints:
  - `/` – main dashboard page.
  - `/data` – returns JSON of detected devices and RSSI.
  - `/target` – endpoint to select a specific target device.

### 3. `server.h`
- Contains shared structures:
  ```c
  typedef struct {
      char ssid[32];
      int rssi;
      float distance;
      bool isTarget;
  } DeviceData;
  ```
- Function prototypes for server initialization and data exchange.

---

## 🔍 How It Works

1. **Scanning:**
   ESP32 periodically scans nearby Wi-Fi and BLE signals.
2. **RSSI Filtering:**
   Raw signals are filtered using a Kalman or exponential smoothing filter.
3. **Distance Estimation:**
   RSSI → distance conversion via empirical formula.
4. **Data Sync:**
   The processed list is shared with the web server task.
5. **Web Dashboard:**
   Displays devices with live RSSI, distance, and selection option for tracking.

---

## 🧰 Build & Flash Instructions

> Ensure ESP-IDF is installed and configured.

### 1. Configure project:
```bash
idf.py menuconfig
```

### 2. Build firmware:
```bash
idf.py build
```

### 3. Flash to board:
```bash
idf.py -p COMx flash
```

### 4. Monitor output:
```bash
idf.py monitor
```

---

## 🌐 Web Interface

- Hosted locally at: `http://<ESP_IP_ADDRESS>/`
- Displays table of devices with:
  - Name / MAC
  - RSSI (strength bar)
  - Distance (calculated)
- User can mark a device as **Target**, and servo logic in `main.c` adjusts direction.

---

## 🧩 Data Flow

```
WiFi/BLE scan
   ↓
RSSI data
   ↓
Kalman filtering
   ↓
Distance calculation
   ↓
Global list (DeviceData[])
   ↓
Webserver (JSON → HTML Dashboard)
```

---

## ⚠️ Gotchas

- Ensure `CONFIG_ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD` is correctly set.
- Use smaller scan intervals to avoid watchdog resets.
- Flash size ≥ 4MB recommended for both Wi-Fi and BLE runtime.
- Network isolation can prevent web access — connect to same LAN.
- Change the wifi name and password in the main.c file
---

## 🚀 Future Improvements

- Add chart visualization (live RSSI vs time)
- Save data logs to SPIFFS or SD card
- Add OTA (Over-the-Air) firmware update
- Multi-client web support

---

## 🧠 Credits
Developed by **Nithiish** , **Varunesh** , **Harshith Shiva** 
Guided by real-world signal tracking applications and ESP-IDF best practices.
