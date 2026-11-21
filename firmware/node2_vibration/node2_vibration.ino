//landslide warning: Node 2 (ESP32)
// Counts vibration pulses over the last 10 seconds and reads soil pressure sensor P2, then
// broadcasts both to the hub (Node 5) over ESP-NOW once a second, and right away when a new
// shake is counted. Pressure calibration lives on the hub.
//
// Wiring
//   Vibration board (SW-420 style) DO -> GPIO 4, VCC -> 3.3V, GND -> GND
//   Pressure P2 (FSR type): sensor from 3.3V to GPIO 34, 10k resistor from GPIO 34 to GND
// The vibration board goes HIGH when it shakes. Set VIBRATION_ACTIVE_HIGH to false if yours goes LOW.
// GPIO 34 is ADC1 (input only), which keeps working while the radio is on.
// Do not use ADC2 pins (GPIO 0, 2, 4, 12-15, 25-27) for analog reads: they fail when WiFi/ESP-NOW is running.
// GPIO 3 is the USB serial RX on the ESP32, so it is not used here.
//
// Board: ESP32 Dev Module. Serial goes over the board's USB-serial chip, no extra setting needed.
// ESPNOW_CHANNEL and the packet layout must match the hub.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int ESPNOW_CHANNEL = 1;
const uint8_t NODE_ID = 2;

const int VIBRATION_PIN = 4;
const bool VIBRATION_ACTIVE_HIGH = true;
const int PRESSURE_PIN = 34;
const int SAMPLE_COUNT = 8;
const unsigned long VIBRATION_WINDOW_MS = 10000;
const unsigned long VIBRATION_DEBOUNCE_MS = 40;
const unsigned long VIBRATION_HOLD_MS = 300;
const int VIBRATION_HISTORY = 16;
const unsigned long SEND_INTERVAL_MS = 1000;
const unsigned long SEND_MIN_GAP_MS = 200;

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

unsigned long vibrationTimes[VIBRATION_HISTORY];
int vibrationIndex = 0;
int vibrationCount = 0;
int sentCount = -1;
bool vibrationWasHigh = false;
bool vibrationActive = false;
unsigned long lastVibrationEdgeMs = 0;

int readSensor(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) sum += analogRead(pin);
  return sum / SAMPLE_COUNT;
}

int countRecentVibrations(unsigned long now) {
  int count = 0;
  for (int i = 0; i < VIBRATION_HISTORY; i++) {
    if (vibrationTimes[i] != 0 && now - vibrationTimes[i] <= VIBRATION_WINDOW_MS) count++;
  }
  return count;
}

void pollVibration() {
  bool active = digitalRead(VIBRATION_PIN) == HIGH;
  if (!VIBRATION_ACTIVE_HIGH) active = !active;
  unsigned long now = millis();
  bool rising = active && !vibrationWasHigh;
  bool held = active && vibrationWasHigh && now - lastVibrationEdgeMs >= VIBRATION_HOLD_MS;
  if ((rising || held) && now - lastVibrationEdgeMs >= VIBRATION_DEBOUNCE_MS) {
    vibrationTimes[vibrationIndex] = now == 0 ? 1 : now;
    vibrationIndex = (vibrationIndex + 1) % VIBRATION_HISTORY;
    lastVibrationEdgeMs = now;
  }
  vibrationWasHigh = active;
  vibrationActive = active;
  vibrationCount = countRecentVibrations(now);
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
  packet.count = 3;
  // adcMax applies to the pressure reading; the hub reads the vibration values as they are.
  packet.adcMax = 4095;
  packet.seq = ++seq;
  packet.values[0] = vibrationCount;
  packet.values[1] = vibrationActive ? 1 : 0;
  packet.values[2] = readSensor(PRESSURE_PIN);
  esp_err_t result = esp_now_send(BROADCAST, (uint8_t*)&packet, sizeof(packet));
  sentCount = vibrationCount;

  Serial.printf("#%lu shakes %d  %s  P2 %d  %s\n", (unsigned long)packet.seq, vibrationCount,
                vibrationActive ? "active" : "still", packet.values[2],
                result == ESP_OK ? "sent" : "send failed");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(VIBRATION_PIN, INPUT_PULLDOWN);
  analogReadResolution(12);
  pinMode(PRESSURE_PIN, INPUT);
  analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);
  startEspNow();
  Serial.print("Node 2 (vibration, pressure P2) MAC ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  pollVibration();
  unsigned long now = millis();
  bool due = now - lastSendMs >= SEND_INTERVAL_MS;
  bool newShake = vibrationCount > sentCount && now - lastSendMs >= SEND_MIN_GAP_MS;
  if (due || newShake) {
    lastSendMs = now;
    sendReadings();
  }
}
