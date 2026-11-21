// Landslide early warning: Node 3 (classic ESP32, ESP32-WROOM DevKit)
// Reads the main water level (A) and soil pressure sensor P1, then broadcasts the raw readings
// to the hub (Node 5) over ESP-NOW once a second. Calibration lives on the hub.
// Pressure sensor P2 is on Node 2.
//
// Wiring
//   Main water level sensor AO -> GPIO 34
//   Pressure P1 (FSR type): sensor from 3.3V to GPIO 35, 10k resistor from GPIO 35 to GND
// Only ADC1 pins (GPIO 32-39) work on a classic ESP32 while the radio is on.
//
// Board: ESP32 Dev Module. ESPNOW_CHANNEL and the packet layout must match the hub.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int ESPNOW_CHANNEL = 1;
const uint8_t NODE_ID = 3;

const int WATER_PIN = 34;
const int PRESSURE_PIN = 35;
const int SAMPLE_COUNT = 8;
const unsigned long SEND_INTERVAL_MS = 1000;

const uint8_t PACKET_MAGIC = 0x4C;
const uint8_t PACKET_VERSION = 1;
const int PACKET_VALUES = 6;

struct __attribute__((packed)) NodePacket {
  uint8_t magic;
  uint8_t version;
  uint8_t node;
  uint8_t count;
  uint16_t adcMax;
  uint32_t seq;
  int16_t values[PACKET_VALUES];
};

uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t seq = 0;
unsigned long lastSendMs = 0;

int readSensor(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) sum += analogRead(pin);
  return sum / SAMPLE_COUNT;
}

void startEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW start failed, restarting");
    delay(1000);
    ESP.restart();
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void sendReadings() {
  NodePacket packet = {};
  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.node = NODE_ID;
  packet.count = 2;
  packet.adcMax = 4095;
  packet.seq = ++seq;
  packet.values[0] = readSensor(WATER_PIN);
  packet.values[1] = readSensor(PRESSURE_PIN);
  esp_err_t result = esp_now_send(BROADCAST, (uint8_t*)&packet, sizeof(packet));

  Serial.printf("#%lu water %d  P1 %d  %s\n", (unsigned long)packet.seq,
                packet.values[0], packet.values[1], result == ESP_OK ? "sent" : "send failed");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);
  pinMode(WATER_PIN, INPUT);
  analogSetPinAttenuation(WATER_PIN, ADC_11db);
  pinMode(PRESSURE_PIN, INPUT);
  analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);
  startEspNow();
  Serial.print("Node 3 (water A, pressure P1) MAC ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendReadings();
  }
}
