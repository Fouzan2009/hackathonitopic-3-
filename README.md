# Landslide Early Warning System
### ESP32-Based Multi-Node IoT Monitoring and 3D Terrain Intelligence

An IoT-based landslide monitoring and early-warning prototype designed around the challenges of mountainous regions, particularly the Himalayan region of **Nepal, India and China**.

The system combines multiple ESP32 sensor nodes, wireless communication, risk analysis, local alerts and a live 3D terrain dashboard to demonstrate how environmental monitoring can support disaster preparedness.

## 🌏 The Problem: Himalayan Landslide Risks

The Himalayan region, shared by Nepal, India and China, is vulnerable to natural hazards such as:

- Heavy rainfall and rainfall-induced landslides.
- Soil instability and slope failures.
- Earthquakes and earthquake-triggered slope instability.
- Glacier melting and glacial lake outburst flood (GLOF) risks.
- River blockages and sudden downstream flooding.

These hazards can threaten mountain communities, roads, infrastructure and river systems.

Continuous environmental monitoring and timely warnings can help improve disaster preparedness and support decision-making.

## 💡 Our Solution

The **Landslide Early Warning System** uses five ESP32-based units to monitor environmental conditions across different points of a demonstration terrain.

Sensor readings are transmitted wirelessly using ESP-NOW to a central ESP32-S3 hub. The hub analyses the incoming data, calculates a prototype risk score from 0–100%, activates local warning indicators and displays the information through a live 3D dashboard.

## ⚙️ System Architecture

### Sensor Network

The project uses five ESP32 units:

| Node | Hardware | Main Function |
|---|---|---|
| Node 1 | ESP32-S3 | Rainfall, soil moisture and temperature monitoring |
| Node 2 | ESP32-S3 | Vibration and soil pressure monitoring |
| Node 3 | ESP32-S3 | Water-level and soil-pressure monitoring |
| Node 4 | ESP32-S3-CAM / ESP32-WROOM | Checkpoint monitoring |
| Node 5 | ESP32-S3 Hub | Data processing, risk analysis, alarms and dashboard |

*Update individual node assignments if your final firmware uses a different sensor configuration.*

### Main Features

- **Multi-sensor monitoring:** Rainfall, soil moisture, soil pressure, vibration, water level and temperature.
- **Wireless communication:** ESP-NOW communication between sensor nodes and the central hub.
- **Risk analysis:** Prototype 0–100% risk scoring based on incoming sensor readings.
- **Local warning system:** LED and buzzer alerts.
- **Data logging:** SD card storage for monitoring records.
- **Live 3D dashboard:** Browser-based terrain visualization with sensor locations and monitoring information.
- **Node health monitoring:** Detects disconnected or silent sensor nodes.

## 🖥️ Live 3D Dashboard

Connect a phone or laptop to the hub's Wi-Fi network and open:

`http://192.168.4.1`

The dashboard provides:

- 3D terrain visualization.
- Sensor-node location markers.
- Live sensor readings.
- Risk-level indicators.
- Monitoring history and event timeline.
- Demonstration evacuation drill and settings.

## 🔬 Prototype Status and Limitations

This project is a working educational prototype, not a field-validated disaster prediction system.

- The terrain model is a demonstration environment.
- Sensor positions are illustrative placeholders.
- Risk scores are experimental and are not calibrated against real landslide datasets.
- ESP-NOW communication is intended for the prototype's local demonstration network.

It must not be treated as a replacement for professional monitoring systems or official disaster warnings.

## 🚀 Future Development

Planned improvements include:

1. **LoRa communication:** Extend communication range between remote sensor stations.
2. **GSM connectivity:** Support remote SMS warning notifications.
3. **GPS integration:** Provide more accurate sensor-node locations.
4. **Advanced prediction:** Explore rainfall trends, soil conditions and historical environmental data.
5. **Regional adaptation:** Explore applications for landslide-prone Himalayan areas and flood-prone communities.

## 🛠️ Technologies Used

- ESP32-S3 and ESP32 boards
- ESP-NOW wireless communication
- Arduino IDE and C++
- HTML, CSS and JavaScript
- Three.js 3D visualization
- SD card data logging
- Multi-sensor IoT monitoring

## 📁 Repository Structure

```text
README.md
.gitignore
docs/
  project-report.pdf
firmware/
  hub/
    hub.ino
    web_files.h
    data/
    tools/
  node1_soil_rain_temp/
  node2_vibration/
  node3_water_pressure/
  node4_checkpoint_a/
```

## ⚡ Getting Started

1. Install Arduino IDE and ESP32 board support.
2. Open the appropriate firmware sketch for each ESP32 unit.
3. Select the correct board and configure its upload settings.
4. Upload the hub firmware to the ESP32-S3.
5. Upload the corresponding sensor-node firmware to the other boards.
6. Power the sensor network and connect to the hub's Wi-Fi.
7. Open `http://192.168.4.1` to view the dashboard.

If dashboard files inside `firmware/hub/data/` are modified, regenerate the embedded web files using:

```bash
python firmware/hub/tools/embed_web.py
```

## ⚠️ Safety Notice

This system is intended for educational demonstrations, research and future pilot development. It is not certified for operational disaster warnings. Always follow official warnings and instructions from the relevant authorities, including India's IMD, CWC, NDMA and state disaster management authorities, Nepal's relevant disaster management authorities, and China's relevant emergency management authorities.

## 📄 Project Report

See [`docs/project-report.pdf`](docs/project-report.pdf) for the project report.

## License

MIT License
