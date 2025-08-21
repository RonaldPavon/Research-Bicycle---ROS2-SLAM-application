# 🚴‍♂️ Bobbi Research Bicycle

BobbiV2 Research Bicycle is an experimental research platform built on a Jetson ORIN AGX embedded system.  
It provides a modular data acquisition framework where different sensor configurations can be launched through a web-based HMI.  
All recorded data is stored in ROS2 `.bag` format and can be downloaded directly from the interface.

---

## 📦 System Components

- **Jetson Orin AGX** running two Docker containers:
  - **Web Application (HMI):** Provides a browser-based interface to select operation mode, start/stop recording, and download data.
  - **ROS2 Drivers Container:** Holds all ROS2 drivers and a launch manager to start the sensor stack based on the selected mode.

- **Sensor Hardware Stack**:
  - [Livox Horizon LiDAR](https://www.livoxtech.com/3296f540ecf5458a8829e01cf429798e/assets/horizon/Livox%20Horizon%20user%20manual%20v1.0.pdf) – High-resolution LiDAR for front/rear perception.
  - [ZED X Camera](https://www.stereolabs.com/zed-x/) – Wide FOV stereo camera for visual data.
  - [u-blox GNSS](https://www.u-blox.com/en/product/zed-f9p-module) – High-precision GPS module for localization.
---

## 🌐 Network Access

- **WiFi SSID:** `bobbi-desktop`  
- **Password:** _[to be shared separately]_  
- Once connected, the system is accessible at:  
  `http://10.42.0.1:5000`

---

## ⚙️ Operating Modes

The HMI supports multiple modes of operation:

- **Front Sensors** – Only front-mounted LiDARs and cameras.  
- **Rear Sensors** – Only rear-mounted LiDARs and cameras.  
- **All Cameras** – All camera feeds enabled.  
- **All LiDAR** – All LiDAR sensors enabled.  
- **Max. Perception** – All available sensors enabled simultaneously.  

---

## 🚀 How to Use

1. **Power On**  
   - Turn on the Jetson system.  
   - Enable the required sensors (front, rear, or both LiDARs) using the physical switches.  

2. **Connect to WiFi**  
   - Connect to the network `bobbi-desktop`.  
   - Use the provided password.  

3. **Access the Web Application**  
   - Open a browser and go to:  
     `http://10.42.0.1:5000`  

4. **Select Operation Mode**  
   - Choose the desired mode (e.g., Front Sensors, All LiDAR, etc.).  
   - Wait for confirmation that the system launched correctly.  

5. **Record Data**  
   - Once the system is running, the **Record** button becomes available.  
   - Click **Start Recording** to capture data.  
   - Click **Stop Recording** to finish.  

6. **Download Data**  
   - Access:  
     `http://10.42.0.1:5000/files`  
   - Download the generated `.bag` files.  

---

## 📊 System Diagram

```mermaid
flowchart TD
    A[Physical Sensors<br>(LiDARs, Cameras)] -->|Power Switches| B[Jetson System]
    B -->|Docker| C[ROS2 Drivers Container]
    B -->|Docker| D[Web Application HMI]
    D -->|Browser: http://10.42.0.1:5000| E[User Interface]
    C -->|ROS2 Data| D
    D -->|Recording| F[Bagfiles<br>/mnt/mcap]
    E -->|Download| F
