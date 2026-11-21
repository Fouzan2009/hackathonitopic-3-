// Landslide early warning: Node 1 (ESP32-S3)
// Reads soil moisture, rain, the NTC temperature sensor, and a DHT11 (air temperature and
// humidity), then broadcasts the readings to the hub (Node 5) over ESP-NOW once a second.
// Calibration lives on the hub.
//
// Wiring
//   Soil moisture (capacitive) AO -> GPIO 7
//   Rain board AO                 -> GPIO 6
//   NTC: 10k resistor from 3.3V to GPIO 8, 10k thermistor from GPIO 8 to GND
//   DHT11 DATA -> GPIO 17, VCC -> 3.3V, GND -> GND
//     (a bare 4-pin DHT11 also needs a 10k resistor from DATA to 3.3V; 3-pin modules have it)
// The analog pins are ADC1, which keeps working while the radio is on.
//
// Library: "DHT sensor library" by Adafruit (plus "Adafruit Unified Sensor").
// Board: ESP32S3 Dev Module. ESPNOW_CHANNEL and the packet layout must match the hub.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <DHT.h>

const int ESPNOW_CHANNEL = 1;
const uint8_t NODE_ID = 1;

const int SOIL_PIN = 7;
const int RAIN_PIN = 6;
const int TEMP_PIN = 8;
const int DHT_PIN = 17;
const int SAMPLE_COUNT = 8;
const unsigned long SEND_INTERVAL_MS = 1000;
// The DHT11 cannot be read faster than once a second.
const unsigned long DHT_INTERVAL_MS = 2000;
const int DHT_MAX_FAILS = 3;
const int16_t NO_READING = -32768;

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
unsigned long lastDhtMs = 0;

DHT dht(DHT_PIN, DHT11);
int16_t airTenths = NO_READING;
int16_t humidityTenths = NO_READING;
int dhtFails = DHT_MAX_FAILS;

int readSensor(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) sum += analogRead(pin);
  return sum / SAMPLE_COUNT;
}

// Keeps the last good reading through a couple of failed reads, then reports no reading.
void readDht() {
  float humidity = dht.readHumidity();
  float celsius = dht.readTemperature();
  if (isnan(humidity) || isnan(celsius)) {
    if (dhtFails < DHT_MAX_FAILS) dhtFails++;
    if (dhtFails >= DHT_MAX_FAILS) {
      airTenths = NO_READING;
      humidityTenths = NO_READING;
    }
    return;
  }
  dhtFails = 0;
  airTenths = (int16_t)lroundf(celsius * 10.0f);
  humidityTenths = (int16_t)lroundf(humidity * 10.0f);
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
  packet.count = 5;
  packet.adcMax = 4095;
  packet.seq = ++seq;
  packet.values[0] = readSensor(SOIL_PIN);
  packet.values[1] = readSensor(RAIN_PIN);
  packet.values[2] = readSensor(TEMP_PIN);
  packet.values[3] = airTenths;
  packet.values[4] = humidityTenths;
  esp_err_t result = esp_now_send(BROADCAST, (uint8_t*)&packet, sizeof(packet));

  Serial.printf("#%lu soil %d  rain %d  temp %d  ", (unsigned long)packet.seq,
                packet.values[0], packet.values[1], packet.values[2]);
  if (airTenths == NO_READING) Serial.print("DHT11 no reading  ");
  else Serial.printf("air %.1f C  humidity %.1f%%  ", airTenths / 10.0f, humidityTenths / 10.0f);
  Serial.println(result == ESP_OK ? "sent" : "send failed");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);
  pinMode(SOIL_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  analogSetPinAttenuation(RAIN_PIN, ADC_11db);
  analogSetPinAttenuation(TEMP_PIN, ADC_11db);
  dht.begin();
  startEspNow();
  Serial.print("Node 1 (soil, rain, temperature, DHT11) MAC ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  unsigned long now = millis();

