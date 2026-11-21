# Landslide Early Warning System (ESP32 Prototype)

A rain-triggered landslide early-warning prototype built on ESP32 boards. Four sensor nodes watch a slope and send readings to a hub, which calculates a landslide risk level, sounds a local alarm and serves a live 3D dashboard.

> **Status:** working prototype. It has not been field-tested or calibrated against real landslide data. The terrain on the map is a demo hill and the node positions are placeholders. Use it for demonstrations, learning and pilot studies alongside official warnings (IMD, Kerala SDMA), never as the only safety measure.

## How it works

- **Sensor nodes** measure rain, soil moisture, soil pressure, vibration, water level and air temperature, and broadcast their readings over ESP-NOW.
- **The hub** (ESP32-S3) receives the readings, computes a risk score from 0 to 100%, drives a buzzer and LED, logs to an SD card and hosts a dashboard on its own Wi-Fi.
- **Safety logic:** a node that is silent for 5 seconds is shown as offline. If two or more nodes are offline or a key sensor is faulty, the level never shows "Normal".

## Nodes

| Node | Board | Measures |
|------|-------|----------|
| 1 | ESP32-S3 | Soil moisture, rain, temperature, air temperature and humidity |
| 2 | ESP32-C3 | Vibration, soil pressure |
| 3 | ESP32 | Main water level, soil pressure |
| 4 | ESP32 | Water checkpoint A |
| 5 (hub) | ESP32-S3 | Water checkpoint B, SD logging, dashboard, alarm |

## Dashboard

Join the hub's Wi-Fi network and open `http://192.168.4.1` on a phone. The dashboard shows a 3D terrain map with node pins, live gauges, trend history, an event timeline, an evacuation drill and a settings page.

## Getting started

1. Install the Arduino IDE with ESP32 board support.
2. Upload each sketch in `firmware/` to its board.
3. For the hub, choose board **ESP32S3 Dev Module** and partition scheme **Huge APP (3MB No OTA/1MB SPIFFS)**.
4. If you edit anything in `firmware/hub/data/`, run `python tools/embed_web.py` and upload again.
5. Power the boards, join the hub Wi-Fi and open the dashboard.

**Security note:** the code ships with a default Wi-Fi password and admin PIN for demo use. Change both before any real deployment.

## Planned upgrades

- LoRa for long-range node links
- GSM for SMS alerts
- GPS for node positioning
- Field calibration with real rainfall and soil data

## Repository layout

```
firmware/   sketches for the hub and four sensor nodes
docs/       project report
```

## License

MIT
