// landslide warning: Node 5 hub (ESP32-S3)
// Receives readings from four sensor nodes over ESP-NOW, reads water checkpoint B itself,
// works out the landslide chance, serves the dashboard on Wi-Fi "land", logs to the SD card,
// and drives the alert LED, buzzer, and onboard RGB LED.
//
// Nodes (each has its own sketch in a subfolder of this project)
//   Node 1  ESP32-S3   soil moisture, rain, NTC temperature, DHT11 air temperature and humidity
//   Node 2  ESP32-C3   vibration, soil pressure P2
//   Node 3  ESP32      main water level (A), soil pressure P1
//   Node 4  ESP32      water checkpoint A
//   Node 5  this hub   water checkpoint B, SD card log
// Every board broadcasts on Wi-Fi channel ESPNOW_CHANNEL, so no MAC addresses need pairing.
// The hub's access point must stay on that channel, so it does not join a home network.
//
// Hub wiring (SD card on 38/39/40, as on ESP32-S3-CAM boards; no camera is used)
//   Water checkpoint B AO -> GPIO 1
//   Alert LED (with resistor) -> GPIO 14
//   Buzzer + -> GPIO 21, - -> GND (driven with a tone, so active and passive buzzers both sound)
//
// Arduino IDE: board "ESP32S3 Dev Module" (PSRAM not needed),
// Partition Scheme "Huge APP (3MB No OTA/1MB SPIFFS)".
//
// Join Wi-Fi "land" (password 12345678) and open http://192.168.4.1
//
// The dashboard (3D terrain map with the node pins, plus every gauge) lives in data/ and is compiled
// into the firmware as web_files.h. After editing anything in data/ (index.html, nodes.json for pin
// positions, terrain), run  python tools/embed_web.py  and upload again.
//
// Admin PIN (settings, calibration, drill, test mode): 1234 until you change it on the Settings page.
// Forgot it? Set FACTORY_RESET_PIN to true, upload once, then set it back to false and upload again.
// Test without node hardware: Settings > Test mode, or /api/fake?on=1&pin=YOURPIN; /api/fake?on=0 stops it.
//
// Features: risk levels with local alarms; phone alarm screen; silence buzzer with auto re-arm;
// water arrival estimate; evacuation drill; trend history; event timeline; today's summary;
// sensor fault detection; settings with demo / field timing; sensor calibration helper.

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <FS.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include <esp_random.h>
#include <math.h>
#include <stdarg.h>
#include <sys/time.h>
#include "esp32-hal-rgb-led.h"
#include "web_files.h"

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

const char* AP_SSID = "land";
const char* AP_PASSWORD = "12345678";
const int ESPNOW_CHANNEL = 1;

const int WATER_C_PIN = 1;
const int ALERT_LED_PIN = 14;
const int BUZZER_PIN = 21;
const unsigned int BUZZER_HZ = 2700;
const unsigned long BUZZER_TEST_MS = 1500;

const int SD_CLK_PIN = 39;
const int SD_CMD_PIN = 38;
const int SD_D0_PIN = 40;
const char* LOG_PATH = "/landslide_log.csv";
const char* LOG_OLD_PATH = "/landslide_log_old.csv";
const char* LOG_HEADER =
    "time,uptime_s,chance,level,water_a_pct,point_a_wet,point_b_wet,rise_pct_min,rise_s,"
    "rain_pct,rain_min,soil_pct,p1_pct,p2_pct,p1_rate,p2_rate,shakes,temp_c,air_c,humidity_pct,nodes_online,flags";
const unsigned long LOG_INTERVAL_MS = 10000;
const unsigned long SD_RETRY_MS = 30000;

// The values below are the defaults. Thresholds, calibration and the timing mode can be changed
// from the dashboard's Settings page; changes are saved on the hub and survive a restart.
const bool FACTORY_RESET_PIN = false;   // true for one upload clears saved settings and sets the PIN to 1234
const char* DEFAULT_PIN = "1234";

// 12-bit ADC scale (0..4095). Each packet carries its ADC range, so other resolutions are scaled to match.
int WET_THRESHOLD = 800;
// Capacitive soil sensor on Node 1. Higher reading means drier.
int SOIL_DRY = 3200;
int SOIL_WET = 1400;
int SOIL_MOIST = 40;
int SOIL_SATURATED = 70;
int SOIL_CRITICAL = 85;
// Rain board on Node 1. Higher reading means drier.
int RAIN_DRY = 3500;
int RAIN_WET = 1000;
int RAIN_LIGHT = 25;
int RAIN_STEADY = 55;
int RAIN_HEAVY = 80;
// Pressure P1 on Node 3, P2 on Node 2. PRESSURE_REST is the settled reading once buried.
int PRESSURE_REST[2] = {150, 150};
int PRESSURE_FULL[2] = {3800, 3800};
// true when a sensor reads high at rest and drops when pressed; its reading is flipped (4095 - raw).
const bool PRESSURE_INVERTED[2] = {true, true};
int PRESS_LOADED = 30;
int PRESS_HIGH = 60;
int PRESS_CRITICAL = 85;
int PRESS_RISE_STEADY = 2;
int PRESS_FAST = 8;
int PRESS_SURGE = 15;
int PRESS_DROP = 15;
const int PRESS_UNEVEN = 35;
const int PRESS_LIMIT_RAW = 4090;
// Water path for the arrival estimate: stream (Node 3) -> checkpoint A (Node 4) -> checkpoint B (hub).
int DIST_AB_M = 918;
int DIST_BC_M = 1440;
int WATER_SPEED_CMS = 100;   // assumed flow speed until one is measured, in cm/s
// Water sensor readings saved by the calibration helper (the wet threshold sits halfway between them).
int WATER_DRY_RAW = 0;
int WATER_WET_RAW = 0;
// NTC on Node 1: 10k to ground, 10k series resistor to 3.3V.
const float TEMP_SERIES_OHMS = 10000.0f;
const float TEMP_NOMINAL_OHMS = 10000.0f;
const float TEMP_NOMINAL_C = 25.0f;
const float TEMP_BETA = 3950.0f;
// DHT11 on Node 1 sends tenths of a degree and tenths of a percent, or DHT_NO_READING.
const int16_t DHT_NO_READING = -32768;
const int HUMID_HIGH = 90;
// Vibration counts arrive from Node 2 as shakes in the last 10 seconds.
int VIBRATION_LIGHT = 1;
int VIBRATION_SHAKING = 3;
int VIBRATION_STRONG = 7;

int CHANCE_ADVISORY = 20;
int CHANCE_WATCH = 45;
int CHANCE_WARNING = 70;

// Timing. "Demo" plays a storm out in minutes (testing, shows); "Field" uses hours, like a real monsoon.
// applyTimingMode() (in the Settings section) sets these from the chosen mode.
int timingMode = 0;   // 0 demo, 1 field
int RISE_STEADY = 2;
int RISE_FAST = 8;
int RISE_SURGE = 20;
int CHECK_FAST_S = 60;
int CHECK_MED_S = 180;
int CHECK_SLOW_S = 600;
unsigned long CHECK_RESET_MS = 30000;
unsigned long RAIN_FORGET_MS = 10UL * 60UL * 1000UL;
unsigned long RAIN_FULL_MS = 30UL * 60UL * 1000UL;
unsigned long RAIN_ACCUM_CAP_MS = 3UL * 60UL * 60UL * 1000UL;
unsigned long RATE_WINDOW_MS = 60000;
unsigned long RATE_SAMPLE_MS = 5000;
unsigned long RAIN_STOP_HOLD_MS = 60000;      // rain must stay below "light" this long to count as stopped
unsigned long STUCK_MS = 10UL * 60UL * 1000UL; // a sensor that never changes this long is flagged
unsigned long DOUBT_RAIN_MS = 2UL * 60UL * 1000UL;
const int RATE_HISTORY = 16;

const int SAMPLE_COUNT = 8;
const unsigned long READ_INTERVAL_MS = 500;
const unsigned long SERIAL_INTERVAL_MS = 2000;
const unsigned long NODE_TIMEOUT_MS = 5000;
const unsigned long BOOT_GRACE_MS = 15000;
// Test mode: simulated nodes step once a second; a real node takes back over for this long after each packet.
const unsigned long FAKE_STEP_MS = 1000;
const unsigned long FAKE_REAL_HOLD_MS = 10000;
const float FAKE_STORM_S = 180.0f;

// ESP-NOW packet, identical in every node sketch.
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

const int REMOTE_NODES = 4;

struct NodeState {
  NodePacket packet;
  unsigned long lastSeen;
  int rssi;
  uint32_t packets;
  bool online;
};

struct SensorReading {
  int raw;
  bool wet;
  bool online;
};

const int TRACK_WATER = 0;
const int TRACK_COUNT = 3;

struct RateTrack {
  int values[RATE_HISTORY];
  unsigned long times[RATE_HISTORY];
  int index;
  int count;
  unsigned long lastSample;
};

NodeState nodes[REMOTE_NODES + 1];
NodeState nodeView[REMOTE_NODES + 1];
portMUX_TYPE nodeMux = portMUX_INITIALIZER_UNLOCKED;

SensorReading readings[3];
RateTrack tracks[TRACK_COUNT];
int soilRaw = 0;
int soilPercent = 0;
int rainRaw = 0;
int rainAmount = 0;
int tempRaw = 0;
int tempTenths = 0;
int airTenths = 0;
int humidityTenths = 0;
bool airOnline = false;
bool dhtFault = false;
int pressureRaw[2];
int pressurePercent[2];
int pressureRate[2];
int vibrationCount = 0;
bool soilOnline = false;
bool vibrationOnline = false;
bool pressureOnline[2] = {false, false};
const int PRESSURE_NODE[2] = {3, 2};
const int PRESSURE_SLOT[2] = {1, 2};

int warningRank = 0;
int chance = 0;
int risePerMin = 0;
int riseSeconds = -1;
int waitSeconds = 0;
int rainMinutes = 0;
bool bReached = false;
bool cReached = false;
bool orderFault = false;
bool easing = false;
bool vibIgnored = false;
bool tempFault = false;
bool unevenLoad = false;
bool pressureRelease = false;
bool pressureFault = false;
bool degraded = false;
unsigned long bWetMs = 0;
unsigned long cWetMs = 0;
unsigned long bothDrySince = 0;
unsigned long rainAccumMs = 0;
unsigned long rainDryMs = 0;
unsigned long lastRiskMs = 0;

unsigned long lastReadMs = 0;
unsigned long lastSerialMs = 0;
unsigned long lastLogMs = 0;
unsigned long lastSdTryMs = 0;
int lastLoggedRank = -1;
uint32_t logRows = 0;
bool sdOk = false;
bool netOk = false;          // access point is up
bool espNowOk = false;       // ESP-NOW receiver is listening
unsigned long lastNetTryMs = 0;
const unsigned long NET_RETRY_MS = 15000;
bool clockSet = false;
long tzOffsetMin = 0;
int rgbRank = -1;
bool buzzerSounding = false;
unsigned long buzzerTestUntil = 0;
bool fakeNodes = false;
unsigned long realSeen[REMOTE_NODES + 1];
unsigned long lastFakeMs = 0;
uint32_t fakeSeq = 0;

// ---- Settings storage and admin PIN ----
Preferences prefs;
bool prefsOk = false;
char adminPin[9] = "1234";
int pinFails = 0;
unsigned long pinLockUntil = 0;
const int PIN_MAX_FAILS = 5;
const unsigned long PIN_LOCK_MS = 60000;
uint32_t bootId = 0;

// ---- Silence buzzer ----
const unsigned long SILENCE_WATCH_MAX_MS = 10UL * 60UL * 1000UL;
const unsigned long SILENCE_WARNING_MAX_MS = 2UL * 60UL * 1000UL;
bool silenced = false;
unsigned long silenceUntil = 0;
int silenceRank = 0;

// ---- Evacuation drill: 30 s advisory, 60 s watch, 90 s warning ----
const unsigned long DRILL_MS = 180000;
bool drillOn = false;
unsigned long drillStart = 0;

// ---- Water arrival estimate ----
bool aReached = false;
unsigned long aWetMs = 0;
int etaState = 0;     // 0 nothing, 1 water heading to checkpoint A, 2 heading to checkpoint B, 3 arrived
long etaSeconds = -1;
int etaBasis = 0;     // 0 none, 1 measured speed, 2 assumed speed
bool etaOverdue = false;

// ---- Trend history: one sample every 30 s for 6 hours ----
struct HistSample {
  uint32_t t;   // uptime seconds
  uint8_t chance, rank, rain, soil, water, p1, p2, shakes, flags;
};
const int HIST_LEN = 720;
const unsigned long HIST_MS = 30000;
const uint8_t NO_VALUE = 255;
HistSample history[HIST_LEN];
int histCount = 0;
int histNext = 0;
unsigned long lastHistMs = 0;
uint8_t histPeakChance = 0;
uint8_t histPeakRank = 0;
uint8_t histPeakShakes = 0;

// ---- Event timeline ----
enum EventType : uint8_t {
  EV_BOOT, EV_LEVEL, EV_NODE_OFF, EV_NODE_ON, EV_RAIN_START, EV_RAIN_STOP, EV_HEAVY_RAIN, EV_SOIL_SAT,
  EV_WATER_A, EV_WATER_B, EV_WATER_C, EV_WATER_CLEAR, EV_ORDER_FAULT, EV_P_RELEASE, EV_UNEVEN, EV_VIB_STRONG,
  EV_FAULT_ON, EV_FAULT_OFF, EV_SILENCE, EV_SILENCE_END, EV_DRILL_START, EV_DRILL_END, EV_TEST_ON, EV_TEST_OFF,
  EV_SETTINGS, EV_CALIBRATE, EV_SD_MISSING, EV_SD_OK, EV_CLOCK_SET, EV_DEGRADED_ON, EV_DEGRADED_OFF, EV_PIN_LOCK
};
const char* EVENT_NAMES[] = {
  "boot", "level", "node-off", "node-on", "rain-start", "rain-stop", "heavy-rain", "soil-saturated",
  "water-stream", "water-checkpoint-a", "water-checkpoint-b", "water-clear", "order-fault", "pressure-release",
  "uneven-load", "strong-vibration", "fault", "fault-clear", "silence", "silence-end", "drill-start", "drill-end",
  "test-on", "test-off", "settings", "calibrate", "sd-missing", "sd-ok", "clock-set", "degraded", "degraded-end", "pin-lock"
};
struct Event {
  uint32_t seq;
  uint32_t t;   // uptime seconds
  uint8_t type;
  int16_t a, b;
};
const int EVENT_LEN = 150;
Event events[EVENT_LEN];
int eventCount = 0;
int eventNext = 0;
uint32_t eventSeq = 0;
const char* EVENT_PATH = "/events.csv";
unsigned long eventWindowStart = 0;
unsigned long eventTypeMs[32];   // last time each sensor-driven event type was logged
int eventsInWindow = 0;
// edge detectors
int eventRank = -1;
unsigned long lowerRankSince = 0;
bool rainActive = false;
unsigned long rainBelowSince = 0;
bool heavyActive = false;
bool soilSatActive = false;
bool aWetPrev = false, bWetPrev = false, cWetPrev = false;
bool orderPrev = false, releasePrev = false, unevenPrev = false, strongPrev = false, degradedPrev = false;
bool nodePrev[REMOTE_NODES + 1];
unsigned long nodeEventMs[REMOTE_NODES + 1];
bool sdPrev = true;

// ---- Today's summary ----
struct DayStats {
  int32_t day;            // local days since 1970, -1 while the clock is not set
  uint32_t startEpoch;    // when counting began (0 if the clock was not set yet)
  uint32_t startUptime;
  uint32_t rainSec;
  uint32_t levelSec[4];
  uint8_t maxChance;
  uint32_t maxChanceUp;   // uptime of the peak
  uint8_t maxSoil, maxWater, maxP1, maxP2, maxShakes;
  int16_t minAir, maxAir; // tenths of a degree, 32767 / -32768 when no reading
  int16_t maxHumid;       // tenths of a percent, -1 when no reading
  uint16_t drops[REMOTE_NODES + 1];
  uint16_t silences;
  uint32_t testSec;
};
DayStats today, yesterday;
bool haveYesterday = false;
unsigned long lastStatsMs = 0;
unsigned long statsCarryMs = 0;
const char* DAILY_PATH = "/daily.csv";

// ---- Sensor faults ----
enum SensorChannel : uint8_t { CH_SOIL, CH_RAIN, CH_NTC, CH_P1, CH_P2, CH_WATER, CH_POINT_A, CH_POINT_B, CH_DHT, CH_COUNT };
const char* CHANNEL_NAMES[] = {"soil", "rain", "ntc", "p1", "p2", "water", "point-a", "point-b", "dht"};
const int CHANNEL_NODE[] = {1, 1, 1, 3, 2, 3, 4, 5, 1};
enum FaultKind : uint8_t { FK_NONE, FK_STUCK, FK_RANGE, FK_ERRATIC, FK_IMPLAUSIBLE, FK_NO_READING, FK_DOUBTFUL };
const char* FAULT_NAMES[] = {"none", "stuck", "range", "erratic", "implausible", "no-reading", "doubtful"};
struct ChannelHealth {
  int lastRaw;
  bool haveRaw;
  unsigned long lastChangeMs;
  unsigned long badSince;     // the current bad condition started (0 = fine)
  unsigned long goodSince;    // condition has been fine since (for clearing)
  uint8_t pending;            // kind seen now
  uint8_t active;             // kind reported
  unsigned long activeSince;
  unsigned long spikes[4];
  int spikeCount;
  // rolling average for calibration (last 10 reads, about 5 s)
  int recent[10];
  int recentCount;
  int recentNext;
};
ChannelHealth health[CH_COUNT];
const unsigned long FAULT_CONFIRM_MS = 30000;
const unsigned long FAULT_CLEAR_MS = 10000;
const int SPIKE_RAW = 1500;
const unsigned long SPIKE_WINDOW_MS = 60000;
bool sensorFault = false;       // any channel has an active fault
bool keySensorFault = false;    // soil, pressure or stream water: risk can't be trusted fully

WebServer server(80);
DNSServer dnsServer;

// ---------- ESP-NOW ----------

void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(NodePacket)) return;
  NodePacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != PACKET_MAGIC || packet.version != PACKET_VERSION) return;
  if (packet.node < 1 || packet.node > REMOTE_NODES || packet.adcMax == 0) return;
  int rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : 0;
  unsigned long now = millis();
  portENTER_CRITICAL(&nodeMux);
  nodes[packet.node].packet = packet;
  nodes[packet.node].lastSeen = now == 0 ? 1 : now;
  nodes[packet.node].rssi = rssi;
  nodes[packet.node].packets++;
  realSeen[packet.node] = nodes[packet.node].lastSeen;
  portEXIT_CRITICAL(&nodeMux);
}

void startEspNow() {
  if (!netOk || espNowOk) return;
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW start failed. Sensor nodes will show offline.");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  espNowOk = true;
  Serial.print("ESP-NOW listening on channel ");
  Serial.println(ESPNOW_CHANNEL);
}

void refreshNodeView(unsigned long now) {
  portENTER_CRITICAL(&nodeMux);
  memcpy(nodeView, nodes, sizeof(nodes));
  portEXIT_CRITICAL(&nodeMux);
  for (int id = 1; id <= REMOTE_NODES; id++) {
    bool wasOnline = nodes[id].online;
    bool online = nodeView[id].lastSeen != 0 && now - nodeView[id].lastSeen <= NODE_TIMEOUT_MS;
    nodeView[id].online = online;
    nodes[id].online = online;
    if (wasOnline != online) {
      Serial.printf("Node %d is %s\n", id, online ? "online" : "offline");
      if (!online && id == 3) {
        tracks[TRACK_WATER].count = 0;
        tracks[1].count = 0;
      }
      if (!online && id == 2) tracks[2].count = 0;
    }
  }
}

bool nodeOnline(int id) {
  if (id == 5) return true;
  return nodeView[id].online;
}

int nodeAge(int id, unsigned long now) {
  if (id == 5) return 0;
  if (nodeView[id].lastSeen == 0) return -1;
  return (int)((now - nodeView[id].lastSeen) / 1000UL);
}

// ---------- Test mode ----------
// A 3-minute storm for nodes with no hardware: rain and soil build, pressure and water follow,
// then checkpoint A floods. Values are written as raw packets, so the full risk logic runs on them.

float stormLevel(float seconds) {
  float phase = fmodf(seconds, FAKE_STORM_S);
  if (phase < 0) phase += FAKE_STORM_S;
  float s = sinf(PI * phase / FAKE_STORM_S);
  return s * s;
}

int16_t fakePressureRaw(int i, float load) {
  int raw = PRESSURE_REST[i] + (int)((PRESSURE_FULL[i] - PRESSURE_REST[i]) * load);
  return (int16_t)(PRESSURE_INVERTED[i] ? 4095 - raw : raw);
}

void setFakeNode(int id, unsigned long now, int count, const int16_t* values) {
  portENTER_CRITICAL(&nodeMux);
  bool realActive = realSeen[id] != 0 && now - realSeen[id] < FAKE_REAL_HOLD_MS;
  if (!realActive) {
    NodePacket& p = nodes[id].packet;
    p.magic = PACKET_MAGIC;
    p.version = PACKET_VERSION;
    p.node = id;
    p.count = count;
    p.adcMax = 4095;
    p.seq = fakeSeq;
    memset(p.values, 0, sizeof(p.values));
    memcpy(p.values, values, count * sizeof(int16_t));
    nodes[id].lastSeen = now == 0 ? 1 : now;
    nodes[id].rssi = -56 - id * 5;
    nodes[id].packets++;
  }
  portEXIT_CRITICAL(&nodeMux);
}

void updateFakeNodes(unsigned long now) {
  if (!fakeNodes || now - lastFakeMs < FAKE_STEP_MS) return;
  lastFakeMs = now;
  fakeSeq++;
  float t = now / 1000.0f;
  float rain = stormLevel(t);
  float soil = 0.3f + 0.65f * stormLevel(t - 10);
  float water = stormLevel(t - 15);
  int shakes = rain > 0.92f ? 8 : (rain > 0.8f ? 3 : 0);

  int16_t n1[5] = {(int16_t)(SOIL_DRY - (SOIL_DRY - SOIL_WET) * soil), (int16_t)(RAIN_DRY - (RAIN_DRY - RAIN_WET) * rain),
                   2048, (int16_t)(260 - 40 * rain), (int16_t)(600 + 380 * rain)};
  int16_t n2[3] = {(int16_t)shakes, (int16_t)(shakes > 0 ? 1 : 0), fakePressureRaw(1, 0.75f * stormLevel(t - 20))};
  int16_t n3[2] = {(int16_t)(4095 * (0.1f + 0.75f * water)), fakePressureRaw(0, 0.8f * stormLevel(t - 12))};
  int16_t n4[1] = {(int16_t)(stormLevel(t - 30) > 0.5f ? 2600 : 200)};
  setFakeNode(1, now, 5, n1);
  setFakeNode(2, now, 3, n2);
  setFakeNode(3, now, 2, n3);
  setFakeNode(4, now, 1, n4);
}

void stopFakeNodes() {
  portENTER_CRITICAL(&nodeMux);
  for (int id = 1; id <= REMOTE_NODES; id++) nodes[id].lastSeen = realSeen[id];
  portEXIT_CRITICAL(&nodeMux);
}

int scaledValue(int id, int index) {
  const NodePacket& p = nodeView[id].packet;
  long raw = p.values[index];
  if (raw < 0) raw = 0;
  if (raw > p.adcMax) raw = p.adcMax;
  return (int)(raw * 4095L / p.adcMax);
}

// ---------- Sensor maths ----------

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int signalPercent(int raw) {
  return clampInt((int)(raw * 100L / 4095), 0, 100);
}

int invertedPercent(int raw, int dryValue, int wetValue) {
  long span = (long)dryValue - wetValue;
  if (span == 0) return 0;
  return clampInt((int)(((long)dryValue - raw) * 100L / span), 0, 100);
}

int pressurePercentFromRaw(int i, int raw) {
  long span = (long)PRESSURE_FULL[i] - PRESSURE_REST[i];
  if (span <= 0) return 0;
  return clampInt((int)(((long)raw - PRESSURE_REST[i]) * 100L / span), 0, 100);
}

int readLocal(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) sum += analogRead(pin);
  return sum / SAMPLE_COUNT;
}

int ntcTenths(int raw) {
  if (raw < 1) raw = 1;
  if (raw > 4094) raw = 4094;
  float resistance = TEMP_SERIES_OHMS * ((float)raw / (4095.0f - (float)raw));
  float steinhart = log(resistance / TEMP_NOMINAL_OHMS);
  steinhart /= TEMP_BETA;
  steinhart += 1.0f / (TEMP_NOMINAL_C + 273.15f);
  steinhart = 1.0f / steinhart;
  float celsius = steinhart - 273.15f;
  if (celsius > 125.0f) celsius = 125.0f;
  if (celsius < -20.0f) celsius = -20.0f;
  return (int)(celsius * 10.0f + (celsius >= 0.0f ? 0.5f : -0.5f));
}

void formatTenths(char* out, size_t size, bool valid, int tenths) {
  if (!valid) {
    snprintf(out, size, "null");
    return;
  }
  int mag = tenths < 0 ? -tenths : tenths;
  snprintf(out, size, "%s%d.%d", tenths < 0 ? "-" : "", mag / 10, mag % 10);
}

void formatTemp(char* out, size_t size) {
  formatTenths(out, size, soilOnline, tempTenths);
}

const char* humidityBandName() {
  int percent = humidityTenths / 10;
  if (percent >= HUMID_HIGH) return "saturated";
  if (percent >= 70) return "humid";
  if (percent >= 40) return "normal";
  return "dry";
}

const char* soilBandName() {
  if (soilPercent >= SOIL_CRITICAL) return "critical";
  if (soilPercent >= SOIL_SATURATED) return "saturated";
  if (soilPercent >= SOIL_MOIST) return "moist";
  return "dry";
}

const char* rainBandName() {
  if (rainAmount >= RAIN_HEAVY) return "heavy";
  if (rainAmount >= RAIN_STEADY) return "steady";
  if (rainAmount >= RAIN_LIGHT) return "light";
  return "dry";
}

const char* pressureBandName(int i) {
  if (pressurePercent[i] >= PRESS_CRITICAL) return "critical";
  if (pressurePercent[i] >= PRESS_HIGH) return "high";
  if (pressurePercent[i] >= PRESS_LOADED) return "loaded";
  return "normal";
}

const char* vibrationBandName() {
  if (vibrationCount >= VIBRATION_STRONG) return "strong";
  if (vibrationCount >= VIBRATION_SHAKING) return "shaking";
  if (vibrationCount >= VIBRATION_LIGHT) return "light";
  return "quiet";
}

int vibrationPercent() {
  return clampInt(vibrationCount * 100 / VIBRATION_STRONG, 0, 100);
}

const char* levelName(int rank) {
  if (rank <= 0) return "normal";
  if (rank == 1) return "advisory";
  if (rank == 2) return "watch";
  return "warning";
}

int maxPressure() {
  return pressurePercent[0] > pressurePercent[1] ? pressurePercent[0] : pressurePercent[1];
}

int maxPressureRate() {
  return pressureRate[0] > pressureRate[1] ? pressureRate[0] : pressureRate[1];
}

// ---------- Rates ----------

void sampleTrack(int id, int percent, unsigned long now) {
  RateTrack* t = &tracks[id];
  if (t->count > 0 && now - t->lastSample < RATE_SAMPLE_MS) return;
  t->values[t->index] = percent;
  t->times[t->index] = now;
  t->index = (t->index + 1) % RATE_HISTORY;
  if (t->count < RATE_HISTORY) t->count++;
  t->lastSample = now;
}

int trackPerMin(int id, unsigned long now) {
  RateTrack* t = &tracks[id];
  if (t->count < 2) return 0;
  int newest = (t->index + RATE_HISTORY - 1) % RATE_HISTORY;
  int oldest = newest;
  for (int n = 1; n < t->count; n++) {
    int idx = (newest + RATE_HISTORY - n) % RATE_HISTORY;
    oldest = idx;
    if (now - t->times[idx] >= RATE_WINDOW_MS) break;
  }
  unsigned long dt = t->times[newest] - t->times[oldest];
  if (dt < 10000UL) return 0;
  long delta = (long)t->values[newest] - t->values[oldest];
  return (int)(delta * 60000L / (long)dt);
}

// ---------- Risk parts ----------

int soilFactor() {
  if (soilPercent <= 0) return 0;
  if (soilPercent >= SOIL_CRITICAL) return 100;
  if (soilPercent >= SOIL_SATURATED) {
    return 70 + (soilPercent - SOIL_SATURATED) * 30 / (SOIL_CRITICAL - SOIL_SATURATED);
  }
  if (soilPercent >= SOIL_MOIST) {
    return 25 + (soilPercent - SOIL_MOIST) * 45 / (SOIL_SATURATED - SOIL_MOIST);
  }
  return soilPercent * 25 / SOIL_MOIST;
}

int rainIntensityScore() {
  if (rainAmount <= RAIN_LIGHT) return 0;
  if (rainAmount >= RAIN_HEAVY) return 100;
  return (rainAmount - RAIN_LIGHT) * 100 / (RAIN_HEAVY - RAIN_LIGHT);
}

void updateRainAccum(unsigned long now) {
  unsigned long delta = 0;
  if (lastRiskMs != 0 && now >= lastRiskMs) delta = now - lastRiskMs;
  if (rainAmount >= RAIN_LIGHT) {
    rainAccumMs += delta;
    if (rainAccumMs > RAIN_ACCUM_CAP_MS) rainAccumMs = RAIN_ACCUM_CAP_MS;
    rainDryMs = 0;
  } else {
    rainDryMs += delta;
    if (rainDryMs >= RAIN_FORGET_MS) {
      rainAccumMs = 0;
      rainDryMs = RAIN_FORGET_MS;
    }
  }
  rainMinutes = (int)(rainAccumMs / 60000UL);
  lastRiskMs = now;
}

int rainScore() {
  int duration = rainAccumMs >= RAIN_FULL_MS ? 100 : (int)(rainAccumMs * 100UL / RAIN_FULL_MS);
  int mix = (rainIntensityScore() * 7 + duration * 3) / 10;
  int gate = 40 + soilFactor() * 60 / 100;
  if (rainAmount >= RAIN_HEAVY && gate < 75) gate = 75;
  return mix * gate / 100;
}

int pressureLoadScore(int percent) {
  if (percent <= PRESS_LOADED) return 0;
  if (percent >= PRESS_CRITICAL) return 100;
  return (percent - PRESS_LOADED) * 100 / (PRESS_CRITICAL - PRESS_LOADED);
}

int pressureRiseScore(int perMin) {
  if (perMin <= PRESS_RISE_STEADY) return 0;
  if (perMin >= PRESS_SURGE) return 100;
  return (perMin - PRESS_RISE_STEADY) * 100 / (PRESS_SURGE - PRESS_RISE_STEADY);
}

void updatePressure(unsigned long now) {
  pressureFault = false;
  pressureRelease = false;
  unevenLoad = false;
  for (int i = 0; i < 2; i++) {
    if (!pressureOnline[i]) {
      pressureRate[i] = 0;
      continue;
    }
    sampleTrack(1 + i, pressurePercent[i], now);
    pressureRate[i] = trackPerMin(1 + i, now);
    if (pressureRaw[i] >= PRESS_LIMIT_RAW) pressureFault = true;
    // A quick fall from a real load means the soil moved off the sensor.
    if (pressureRate[i] <= -PRESS_DROP && pressurePercent[i] - pressureRate[i] >= 50) pressureRelease = true;
  }
  if (!pressureOnline[0] || !pressureOnline[1]) return;
  int gap = pressurePercent[0] - pressurePercent[1];
  if (gap < 0) gap = -gap;
  unevenLoad = gap >= PRESS_UNEVEN && maxPressure() >= PRESS_LOADED;
}

int pressureScore() {
  int best = 0;
  for (int i = 0; i < 2; i++) {
    int load = pressureLoadScore(pressurePercent[i]);
    int rise = pressureRiseScore(pressureRate[i]);
    int hi = load > rise ? load : rise;
    int lo = load > rise ? rise : load;
    int mix = (hi * 3 + lo) / 4;
    if (mix > best) best = mix;
  }
  if (unevenLoad) best = clampInt(best + 15, 0, 100);
  if (pressureRelease) best = 100;
  int gate = 70 + soilFactor() * 30 / 100;
  return best * gate / 100;
}

int riseScore(int perMin) {
  if (perMin <= RISE_STEADY) return 0;
  if (perMin >= RISE_SURGE) return 100;
  return (perMin - RISE_STEADY) * 100 / (RISE_SURGE - RISE_STEADY);
}

int levelScore(int percent) {
  if (percent < 15) return 0;
  return clampInt((percent - 15) * 100 / 70, 0, 100);
}

void updateCheckpoints(unsigned long now) {
  bool bOnline = readings[1].online;
  bool bNow = bOnline && readings[1].wet;
  bool cNow = readings[2].wet;
  if (!bReached && !cReached && bNow && cNow) {
    bReached = true;
    cReached = true;
    bWetMs = now;
    cWetMs = now;
    orderFault = false;
    // Already wet at startup has no start time. Wet after a dry reading rose inside one sample.
    riseSeconds = bothDrySince != 0 ? 0 : -1;
  } else {
    if (!bReached && bNow) {
      bReached = true;
      bWetMs = now;
    }
    if (!cReached && cNow) {
      cReached = true;
      cWetMs = now;
      if (!bReached) {
        // Without Node 4 the order cannot be judged, so it is not a fault.
        orderFault = bOnline;
        riseSeconds = -1;
      } else {
        orderFault = false;
        riseSeconds = (int)((cWetMs - bWetMs) / 1000UL);
      }
    }
  }

  if (!bNow && !cNow) {
    if (bothDrySince == 0) bothDrySince = now;
    else if ((bReached || cReached) && now - bothDrySince >= CHECK_RESET_MS) {
      bReached = false;
      cReached = false;
      orderFault = false;
      riseSeconds = -1;
      bWetMs = 0;
      cWetMs = 0;
      bothDrySince = 0;
    }
  } else {
    bothDrySince = 0;
  }

  if (bReached && !cReached && bWetMs != 0) waitSeconds = (int)((now - bWetMs) / 1000UL);
  else waitSeconds = 0;
}

int checkpointScore() {
  if (orderFault) return 70;
  if (bReached && cReached) {
    if (riseSeconds < 0) return 40;
    if (riseSeconds <= CHECK_FAST_S) return 100;
    if (riseSeconds <= CHECK_MED_S) return 75;
    if (riseSeconds <= CHECK_SLOW_S) return 45;
    return 25;
  }
  if (bReached) return 30;
  return 0;
}

bool slopeQuiet(int levelPercent) {
  return rainAmount < RAIN_LIGHT && maxPressure() < PRESS_LOADED && maxPressureRate() < PRESS_RISE_STEADY &&
         !pressureRelease && soilPercent < SOIL_MOIST && levelPercent < 15 && !bReached && !cReached;
}

int vibrationScore(int levelPercent) {
  vibIgnored = false;
  if (vibrationCount >= VIBRATION_STRONG) return 100;
  if (vibrationCount >= VIBRATION_SHAKING) return 70;
  if (vibrationCount >= VIBRATION_LIGHT) {
    if (slopeQuiet(levelPercent)) {
      vibIgnored = true;
      return 0;
    }
    return 15;
  }
  return 0;
}

void evaluateRisk(unsigned long now) {
  int levelPercent = readings[0].online ? signalPercent(readings[0].raw) : 0;
  if (readings[0].online) {
    sampleTrack(TRACK_WATER, levelPercent, now);
    risePerMin = trackPerMin(TRACK_WATER, now);
  } else {
    risePerMin = 0;
  }
  updateCheckpoints(now);
  updateArrival(now);
  updateRainAccum(now);
  updatePressure(now);
  tempFault = soilOnline && (tempTenths <= -200 || tempTenths >= 1250);

  int offline = 0;
  for (int id = 1; id <= REMOTE_NODES; id++) {
    if (!nodeOnline(id)) offline++;
  }
  // Two nodes gone, or a key sensor (soil, pressure, stream water) misbehaving: "normal" can't be trusted.
  degraded = now > BOOT_GRACE_MS && (offline >= 2 || keySensorFault);

  int topLoad = maxPressure();
  int topRate = maxPressureRate();
  int rainPart = rainScore();
  int pressurePart = pressureScore();
  int soilPart = soilFactor();
  int waterPart = (levelScore(levelPercent) + riseScore(risePerMin)) / 2;
  int checkPart = checkpointScore();
  int vibPart = vibrationScore(levelPercent);
  int score = (rainPart * 25 + pressurePart * 25 + soilPart * 20 + waterPart * 15 + checkPart * 10 + vibPart * 5) / 100;

  easing = risePerMin < 0 && rainAmount < RAIN_LIGHT && topRate <= 0 && !pressureRelease && !cReached;
  if (easing) score = score * 7 / 10;
  if (orderFault && score < 55) score = 55;

  if (rainAmount >= RAIN_HEAVY && score < 20) score = 20;
  if (rainAmount >= RAIN_HEAVY && soilPercent >= SOIL_SATURATED && score < 45) score = 45;
  if (topLoad >= PRESS_HIGH && score < 25) score = 25;
  if ((topRate >= PRESS_FAST || unevenLoad) && score < 45) score = 45;
  if (topLoad >= PRESS_CRITICAL && score < 55) score = 55;
  if (pressureRelease && score < 60) score = 60;

  bool fastWater = risePerMin >= RISE_FAST ||
                   (bReached && cReached && riseSeconds >= 0 && riseSeconds <= CHECK_FAST_S);
  if (soilPercent >= SOIL_SATURATED &&
      ((rainAmount >= RAIN_HEAVY && fastWater) || topRate >= PRESS_FAST || topLoad >= PRESS_CRITICAL ||
       (fastWater && topLoad >= PRESS_LOADED))) {
    if (score < 85) score = 85;
  }

  if (vibrationCount >= VIBRATION_STRONG) {
    if (score < 92) score = 92;
  } else if (vibrationCount >= VIBRATION_SHAKING) {
    int floor = (soilPercent >= SOIL_MOIST || rainAmount >= RAIN_LIGHT || topLoad >= PRESS_LOADED) ? 60 : 30;
    if (pressureRelease) floor = 85;
    if (score < floor) score = floor;
  }

  if (degraded && score < CHANCE_ADVISORY) score = CHANCE_ADVISORY;

  chance = clampInt(score, 0, 100);
  if (chance < CHANCE_ADVISORY) warningRank = 0;
  else if (chance < CHANCE_WATCH) warningRank = 1;
  else if (chance < CHANCE_WARNING) warningRank = 2;
  else warningRank = 3;
}

void updateSensors() {
  unsigned long now = millis();
  refreshNodeView(now);

  readings[0].online = nodeOnline(3);
  readings[0].raw = readings[0].online ? scaledValue(3, 0) : 0;
  readings[1].online = nodeOnline(4);
  readings[1].raw = readings[1].online ? scaledValue(4, 0) : 0;
  readings[2].online = true;
  readings[2].raw = readLocal(WATER_C_PIN);
  for (int i = 0; i < 3; i++) readings[i].wet = readings[i].online && readings[i].raw >= WET_THRESHOLD;

  soilOnline = nodeOnline(1);
  if (soilOnline) {
    soilRaw = scaledValue(1, 0);
    rainRaw = scaledValue(1, 1);
    tempRaw = scaledValue(1, 2);
    soilPercent = invertedPercent(soilRaw, SOIL_DRY, SOIL_WET);
    rainAmount = invertedPercent(rainRaw, RAIN_DRY, RAIN_WET);
    tempTenths = ntcTenths(tempRaw);
  } else {
    soilPercent = 0;
    rainAmount = 0;
  }
  const NodePacket& n1 = nodeView[1].packet;
  airOnline = soilOnline && n1.count >= 5 && n1.values[3] != DHT_NO_READING && n1.values[4] != DHT_NO_READING;
  dhtFault = soilOnline && !airOnline;
  if (airOnline) {
    airTenths = n1.values[3];
    humidityTenths = clampInt(n1.values[4], 0, 1000);
  }

  for (int i = 0; i < 2; i++) {
    int id = PRESSURE_NODE[i];
    int slot = PRESSURE_SLOT[i];
    pressureOnline[i] = nodeOnline(id) && nodeView[id].packet.count > slot;
    pressureRaw[i] = 0;
    if (pressureOnline[i]) {
      pressureRaw[i] = scaledValue(id, slot);
      if (PRESSURE_INVERTED[i]) pressureRaw[i] = 4095 - pressureRaw[i];
    }
    pressurePercent[i] = pressureOnline[i] ? pressurePercentFromRaw(i, pressureRaw[i]) : 0;
  }

  vibrationOnline = nodeOnline(2);
  vibrationCount = vibrationOnline ? clampInt(nodeView[2].packet.values[0], 0, 99) : 0;

  updateHealth(now);
  evaluateRisk(now);
}

// ---------- Alerts ----------

void setBuzzer(bool on) {
  if (on == buzzerSounding) return;
  buzzerSounding = on;
  if (on) tone(BUZZER_PIN, BUZZER_HZ);
  else noTone(BUZZER_PIN);
}

void showRgb(int rank) {
  if (rank == rgbRank) return;
  rgbRank = rank;
  if (rank <= 0) neopixelWrite(RGB_BUILTIN, 0, 40, 8);
  else if (rank == 1) neopixelWrite(RGB_BUILTIN, 48, 36, 0);
  else if (rank == 2) neopixelWrite(RGB_BUILTIN, 64, 20, 0);
  else neopixelWrite(RGB_BUILTIN, 72, 0, 0);
}

void driveAlerts(unsigned long now) {
  serviceDrill(now);
  int rank = alarmRank(now);
  serviceSilence(now, rank);
  showRgb(rank);
  bool ledOn = false;
  bool buzzOn = false;
  if (rank == 1) {
    ledOn = true;
  } else if (rank == 2) {
    ledOn = (now / 500UL) % 2UL == 0;
    buzzOn = (now % 1400UL) < 160UL;
  } else if (rank >= 3) {
    ledOn = (now / 160UL) % 2UL == 0;
    buzzOn = (now % 320UL) < 160UL;
  }
  if (silenced) buzzOn = false;   // the light keeps flashing; only the sound is muted
  if (buzzerTestUntil != 0) {
    if ((long)(now - buzzerTestUntil) < 0) buzzOn = (now % 400UL) < 250UL;
    else buzzerTestUntil = 0;
  }
  digitalWrite(ALERT_LED_PIN, ledOn ? HIGH : LOW);
  setBuzzer(buzzOn);
}

// ---------- SD card log ----------

void formatClock(char* out, size_t size) {
  if (!clockSet) {
    snprintf(out, size, "unset");
    return;
  }
  time_t t = time(NULL) + tzOffsetMin * 60L;
  struct tm parts;
  gmtime_r(&t, &parts);
  strftime(out, size, "%Y-%m-%d %H:%M:%S", &parts);
}

void startSd() {
  lastSdTryMs = millis();
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true)) {
    sdOk = false;
    Serial.println("SD card not found. Logging is off until a card is inserted.");
    return;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    SD_MMC.end();
    sdOk = false;
    Serial.println("SD card not found. Logging is off until a card is inserted.");
    return;
  }
  sdOk = true;
  if (SD_MMC.exists(LOG_PATH)) {
    File old = SD_MMC.open(LOG_PATH, FILE_READ);
    String firstLine = old ? old.readStringUntil('\n') : String(LOG_HEADER);
    if (old) old.close();
    firstLine.trim();
    if (firstLine != LOG_HEADER) {
      // Columns changed since this file was started, so keep it aside and begin a new one.
      SD_MMC.remove(LOG_OLD_PATH);
      SD_MMC.rename(LOG_PATH, LOG_OLD_PATH);
      Serial.println("Old log moved to " + String(LOG_OLD_PATH));
    }
  }
  if (!SD_MMC.exists(LOG_PATH)) {
    File f = SD_MMC.open(LOG_PATH, FILE_WRITE);
    if (f) {
      f.println(LOG_HEADER);
      f.close();
    }
  }
  Serial.println("SD card ready, logging to " + String(LOG_PATH));
}

void logRow(unsigned long now) {
  if (!sdOk) return;
  File f = SD_MMC.open(LOG_PATH, FILE_APPEND);
  if (!f) {
    sdOk = false;
    SD_MMC.end();
    Serial.println("SD card write failed. Will retry.");
    return;
  }
  char clockText[24];
  formatClock(clockText, sizeof(clockText));
  char tempText[8];
  formatTemp(tempText, sizeof(tempText));
  char airText[8];
  formatTenths(airText, sizeof(airText), airOnline, airTenths);
  char humidText[8];
  formatTenths(humidText, sizeof(humidText), airOnline, humidityTenths);
  char online[8];
  int n = 0;
  for (int id = 1; id <= REMOTE_NODES; id++) {
    if (nodeOnline(id)) online[n++] = '0' + id;
  }
  online[n++] = '5';
  online[n] = '\0';
  char flags[160];
  snprintf(flags, sizeof(flags), "%s%s%s%s%s%s%s%s%s%s%s",
           fakeNodes ? "test-mode " : "", orderFault ? "order " : "", unevenLoad ? "uneven " : "",
           pressureRelease ? "release " : "", degraded ? "degraded " : "", dhtFault ? "dht-fault " : "",
           vibIgnored ? "shake-ignored " : "", drillOn ? "drill " : "", silenced ? "silenced " : "",
           sensorFault ? "sensor-fault " : "", timingMode == 1 ? "field" : "demo");
  f.printf("%s,%lu,%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%s,%s,%s\n",
           clockText, now / 1000UL, chance, levelName(warningRank),
           readings[0].online ? signalPercent(readings[0].raw) : -1,
           readings[1].wet ? 1 : 0, readings[2].wet ? 1 : 0, risePerMin, riseSeconds,
           soilOnline ? rainAmount : -1, rainMinutes, soilOnline ? soilPercent : -1,
           pressureOnline[0] ? pressurePercent[0] : -1, pressureOnline[1] ? pressurePercent[1] : -1,
           pressureRate[0], pressureRate[1], vibrationOnline ? vibrationCount : -1,
           strcmp(tempText, "null") == 0 ? "" : tempText, airOnline ? airText : "",
           airOnline ? humidText : "", online, flags);
  f.close();
  logRows++;
}

void serviceLog(unsigned long now) {
  if (!sdOk && now - lastSdTryMs >= SD_RETRY_MS) startSd();
  bool levelChanged = warningRank != lastLoggedRank;
  if (levelChanged || now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    lastLoggedRank = warningRank;
    logRow(now);
  }
}

void handleLog() {
  if (!sdOk) {
    server.send(503, "text/plain", "SD card not available");
    return;
  }
  File f = SD_MMC.open(LOG_PATH, FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "No log yet");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=landslide_log.csv");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleTime() {
  if (server.hasArg("t")) {
    long long t = atoll(server.arg("t").c_str());
    if (t > 1600000000LL) {
      struct timeval tv = {(time_t)t, 0};
      settimeofday(&tv, NULL);
      if (server.hasArg("tz")) tzOffsetMin = clampInt(server.arg("tz").toInt(), -720, 840);
      if (!clockSet) {
        clockSet = true;
        if (today.startEpoch == 0) today.startEpoch = bootEpoch() + today.startUptime;
        addEvent(EV_CLOCK_SET, (int)tzOffsetMin, 0);
      }
    }
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------- Web ----------

static char json[4096];
static size_t jsonUsed = 0;

void jsonAdd(const char* fmt, ...) {
  if (jsonUsed >= sizeof(json)) return;
  va_list args;
  va_start(args, fmt);
  int wrote = vsnprintf(json + jsonUsed, sizeof(json) - jsonUsed, fmt, args);
  va_end(args);
  if (wrote > 0) jsonUsed += wrote;
  if (jsonUsed >= sizeof(json)) jsonUsed = sizeof(json) - 1;
}

const char* tf(bool value) {
  return value ? "true" : "false";
}

void handleApi() {
  unsigned long now = millis();
  char tempText[8];
  formatTemp(tempText, sizeof(tempText));
  jsonUsed = 0;
  jsonAdd("{\"level\":\"%s\",\"chance\":%d,\"risePerMin\":%d,\"riseSec\":%d,\"waitSec\":%d,\"rainMin\":%d,",
          levelName(warningRank), chance, risePerMin, riseSeconds, waitSeconds, rainMinutes);
  jsonAdd("\"bReached\":%s,\"cReached\":%s,\"orderFault\":%s,\"easing\":%s,\"vibIgnored\":%s,\"tempFault\":%s,",
          tf(bReached), tf(cReached), tf(orderFault), tf(easing), tf(vibIgnored), tf(tempFault));
  jsonAdd("\"unevenLoad\":%s,\"pressureRelease\":%s,\"pressureFault\":%s,\"degraded\":%s,\"dhtFault\":%s,",
          tf(unevenLoad), tf(pressureRelease), tf(pressureFault), tf(degraded), tf(dhtFault));
  jsonAdd("\"camera\":false,\"sd\":%s,\"logRows\":%lu,\"clock\":%s,\"fake\":%s,\"sensors\":[",
          tf(sdOk), (unsigned long)logRows, tf(clockSet), tf(fakeNodes));
  const int waterNodes[3] = {3, 4, 5};
  for (int i = 0; i < 3; i++) {
    jsonAdd("%s{\"name\":\"%c\",\"node\":%d,\"online\":%s,\"raw\":%d,\"percent\":%d,\"wet\":%s}",
            i == 0 ? "" : ",", 'A' + i, waterNodes[i], tf(readings[i].online), readings[i].raw,
            signalPercent(readings[i].raw), tf(readings[i].wet));
  }
  jsonAdd("],\"pressure\":[");
  for (int i = 0; i < 2; i++) {
    jsonAdd("%s{\"name\":\"P%d\",\"node\":%d,\"online\":%s,\"raw\":%d,\"percent\":%d,\"rate\":%d,\"band\":\"%s\"}",
            i == 0 ? "" : ",", i + 1, PRESSURE_NODE[i], tf(pressureOnline[i]), pressureRaw[i], pressurePercent[i], pressureRate[i],
            pressureBandName(i));
  }
  jsonAdd("],\"rain\":{\"node\":1,\"online\":%s,\"raw\":%d,\"percent\":%d,\"band\":\"%s\"},",
          tf(soilOnline), rainRaw, rainAmount, rainBandName());
  jsonAdd("\"soil\":{\"node\":1,\"online\":%s,\"raw\":%d,\"percent\":%d,\"band\":\"%s\"},",
          tf(soilOnline), soilRaw, soilPercent, soilBandName());
  jsonAdd("\"vibration\":{\"node\":2,\"online\":%s,\"count\":%d,\"percent\":%d,\"band\":\"%s\"},",
          tf(vibrationOnline), vibrationCount, vibrationPercent(), vibrationBandName());
  jsonAdd("\"temperature\":{\"node\":1,\"online\":%s,\"raw\":%d,\"celsius\":%s},",
          tf(soilOnline), tempRaw, tempText);
  char airText[8];
  formatTenths(airText, sizeof(airText), airOnline, airTenths);
  char humidText[8];
  formatTenths(humidText, sizeof(humidText), airOnline, humidityTenths);
  jsonAdd("\"air\":{\"node\":1,\"online\":%s,\"reading\":%s,\"celsius\":%s,\"humidity\":%s,\"band\":\"%s\"},\"nodes\":[",
          tf(soilOnline), tf(airOnline), airText, humidText, humidityBandName());
  for (int id = 1; id <= 5; id++) {
    int rssi = id == 5 ? 0 : nodeView[id].rssi;
    jsonAdd("%s{\"id\":%d,\"online\":%s,\"age\":%d,\"rssi\":%d}", id == 1 ? "" : ",", id,
            tf(nodeOnline(id)), nodeAge(id, now), rssi);
  }
  int rank = alarmRank(now);
  jsonAdd("],\"boot\":%lu,\"up\":%lu,\"epoch\":%lu,\"evSeq\":%lu,\"mode\":\"%s\",\"alarm\":\"%s\",",
          (unsigned long)bootId, now / 1000UL, (unsigned long)bootEpoch(), (unsigned long)eventSeq,
          timingMode == 1 ? "field" : "demo", levelName(rank));
  jsonAdd("\"drill\":{\"on\":%s,\"rank\":%d,\"left\":%ld},", tf(drillOn), drillRankAt(now),
          drillOn ? (long)((DRILL_MS - (now - drillStart)) / 1000UL) : 0L);
  jsonAdd("\"silence\":{\"on\":%s,\"left\":%ld,\"can\":%s,\"maxMin\":%lu},", tf(silenced), silenceLeft(now),
          tf(rank >= 2), (rank >= 3 ? SILENCE_WARNING_MAX_MS : SILENCE_WATCH_MAX_MS) / 60000UL);
  jsonAdd("\"eta\":{\"state\":%d,\"sec\":%ld,\"basis\":%d,\"overdue\":%s},", etaState, etaSeconds, etaBasis,
          tf(etaOverdue));
  addFaultsJson(now);
  jsonAdd("}");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

const WebFile* findWebFile(const String& path) {
  for (size_t i = 0; i < WEB_FILE_COUNT; i++) {
    if (path == WEB_FILES[i].path) return &WEB_FILES[i];
  }
  return nullptr;
}

// Changes on every firmware build, so phones re-download the website only after a new upload.
String webEtag;

bool isAssetPath(const String& path) {
  return path.endsWith(".js") || path.endsWith(".json") || path.endsWith(".bin") || path.endsWith(".css") ||
         path.endsWith(".png") || path.endsWith(".svg") || path.endsWith(".map") || path.endsWith(".gz");
}

// Files are stored gzipped.
// - A missing script / data file gets a real 404. (Answering with the dashboard page made the 3D map
//   fail silently: the browser got HTML where it expected JavaScript or JSON.)
// - Any other unknown address, phone captive-portal checks included, is redirected to the dashboard
//   at "/", so the page always runs from the root and its files resolve.
void handleFile() {
  String path = server.uri();
  if (path.endsWith("/")) path += "index.html";
  const WebFile* file = findWebFile(path);
  if (!file) {
    if (path == "/index.html") {
      server.send(500, "text/plain", "Dashboard missing: run tools/embed_web.py and upload again");
    } else if (isAssetPath(path)) {
      server.sendHeader("Cache-Control", "no-store");
      server.send(404, "text/plain", "Not found");
    } else {
      server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
      server.sendHeader("Cache-Control", "no-store");
      server.send(302, "text/plain", "");
    }
    return;
  }
  if (webEtag.length() && server.header("If-None-Match") == webEtag) {
    server.send(304);
    return;
  }
  // three.js never changes, so phones keep it a week; everything else re-checks so new uploads show up.
  server.sendHeader("Cache-Control", String(file->path).endsWith(".js") ? "max-age=604800" : "no-cache");
  if (webEtag.length()) server.sendHeader("ETag", webEtag);
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, file->mime, (const char*)file->data, file->len);
}

// Test mode: switching it on needs the admin PIN (a fake storm must not be started by accident);
// anyone may switch it off.
void handleFake() {
  if (server.hasArg("on")) {
    bool on = server.arg("on") != "0";
    if (on && !fakeNodes && !requirePin()) return;
    if (fakeNodes && !on) stopFakeNodes();
    if (on != fakeNodes) addEvent(on ? EV_TEST_ON : EV_TEST_OFF, 0, 0);
    fakeNodes = on;
    Serial.printf("Test mode %s\n", fakeNodes ? "ON: simulated storm on nodes without hardware" : "off");
  }
  replyJson(200, fakeNodes ? "{\"fake\":true}" : "{\"fake\":false}");
}

void handleBeep() {
  buzzerTestUntil = millis() + BUZZER_TEST_MS;
  if (buzzerTestUntil == 0) buzzerTestUntil = 1;
  Serial.println("Buzzer test");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleFavicon() {
  server.send(204);
}

void printMemory(const char* when) {
  Serial.printf("[memory %s] internal free %u B (largest block %u B), PSRAM %s",
                when, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                psramFound() ? "found" : "not used");
  if (psramFound()) Serial.printf(", %u B free", (unsigned)ESP.getFreePsram());
  Serial.println();
}

// Starts the access point, retrying a few times. Wi-Fi's receive buffers need internal RAM, so it
// starts before anything else that allocates memory.
bool startNetwork() {
  lastNetTryMs = millis();
  if (strlen(AP_PASSWORD) > 0 && strlen(AP_PASSWORD) < 8) {
    Serial.println("AP_PASSWORD must be at least 8 characters (or empty for an open network).");
    return false;
  }
  if (ESPNOW_CHANNEL < 1 || ESPNOW_CHANNEL > 13) {
    Serial.println("ESPNOW_CHANNEL must be 1 to 13.");
    return false;
  }
  WiFi.persistent(false);   // no flash writes on every boot
  for (int attempt = 1; attempt <= 3 && !netOk; attempt++) {
    if (WiFi.mode(WIFI_AP) && WiFi.softAP(AP_SSID, AP_PASSWORD, ESPNOW_CHANNEL)) {
      netOk = true;
    } else {
      Serial.printf("Wi-Fi start failed (try %d of 3)\n", attempt);
      printMemory("after Wi-Fi failure");
      WiFi.mode(WIFI_OFF);
      delay(300 * attempt);
    }
  }
  if (!netOk) {
    Serial.println("Wi-Fi is off: alarms, LED and buzzer still work; retrying every 15 s.");
    return false;
  }
  WiFi.setSleep(false);
  dnsServer.start(53, "*", WiFi.softAPIP());
  Serial.print("Access point: ");
  Serial.print(AP_SSID);
  Serial.print("  password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Open http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("Hub MAC ");
  Serial.println(WiFi.softAPmacAddress());
  return true;
}

// The network stack only exists once Wi-Fi has started, so the web server waits for it.
bool webServerOn = false;
void startWebServer() {
  if (webServerOn || !netOk) return;
  server.begin();
  webServerOn = true;
}

// Wi-Fi is what the whole network depends on, so keep retrying it.
void serviceNetwork(unsigned long now) {
  if (netOk || now - lastNetTryMs < NET_RETRY_MS) return;
  if (startNetwork()) {
    startEspNow();
    startWebServer();
  }
}

// ---------- Small helpers ----------

uint32_t nowEpoch() {
  return clockSet ? (uint32_t)time(NULL) : 0;
}

// Wall-clock time of boot, so the dashboard can turn uptime seconds into clock times.
uint32_t bootEpoch() {
  return clockSet ? (uint32_t)(time(NULL) - (time_t)(millis() / 1000UL)) : 0;
}

// Local day number (days since 1970 in the phone's time zone), or -1 while the clock is unknown.
int32_t localDay() {
  if (!clockSet) return -1;
  long long t = (long long)time(NULL) + (long long)tzOffsetMin * 60LL;
  return (int32_t)(t / 86400LL);
}

// In test mode a node with no real hardware sends simulated readings.
bool nodeSimulated(int id) {
  if (!fakeNodes || id < 1 || id > REMOTE_NODES) return false;
  unsigned long seen = realSeen[id];
  return !(seen != 0 && millis() - seen < FAKE_REAL_HOLD_MS);
}

long elapsedSec(unsigned long now, unsigned long since) {
  return (long)((now - since) / 1000UL);
}

// ---------- Settings (saved on the hub) ----------

// Sets every timing value from the chosen mode. Demo plays a storm out in minutes; field uses hours.
void applyTimingMode() {
  if (timingMode == 1) {
    RISE_STEADY = 1; RISE_FAST = 4; RISE_SURGE = 10;
    PRESS_RISE_STEADY = 1; PRESS_FAST = 3; PRESS_SURGE = 6; PRESS_DROP = 10;
    CHECK_FAST_S = 600; CHECK_MED_S = 1800; CHECK_SLOW_S = 7200;
    CHECK_RESET_MS = 60UL * 60UL * 1000UL;
    RAIN_FORGET_MS = 12UL * 60UL * 60UL * 1000UL;
    RAIN_FULL_MS = 6UL * 60UL * 60UL * 1000UL;
    RAIN_ACCUM_CAP_MS = 72UL * 60UL * 60UL * 1000UL;
    RATE_WINDOW_MS = 15UL * 60UL * 1000UL;
    RATE_SAMPLE_MS = 60000;
    RAIN_STOP_HOLD_MS = 15UL * 60UL * 1000UL;
    STUCK_MS = 3UL * 60UL * 60UL * 1000UL;
    DOUBT_RAIN_MS = 15UL * 60UL * 1000UL;
  } else {
    RISE_STEADY = 2; RISE_FAST = 8; RISE_SURGE = 20;
    PRESS_RISE_STEADY = 2; PRESS_FAST = 8; PRESS_SURGE = 15; PRESS_DROP = 15;
    CHECK_FAST_S = 60; CHECK_MED_S = 180; CHECK_SLOW_S = 600;
    CHECK_RESET_MS = 30000;
    RAIN_FORGET_MS = 10UL * 60UL * 1000UL;
    RAIN_FULL_MS = 30UL * 60UL * 1000UL;
    RAIN_ACCUM_CAP_MS = 3UL * 60UL * 60UL * 1000UL;
    RATE_WINDOW_MS = 60000;
    RATE_SAMPLE_MS = 5000;
    RAIN_STOP_HOLD_MS = 60000;
    STUCK_MS = 10UL * 60UL * 1000UL;
    DOUBT_RAIN_MS = 2UL * 60UL * 1000UL;
  }
}


struct IntSetting {
  const char* key;   // NVS key and the name used by the dashboard (15 characters at most)
  int* var;
  int def;
  int lo;
  int hi;
  bool calibration;
};

IntSetting SETTINGS[] = {
  {"chAdv", &CHANCE_ADVISORY, 20, 5, 90, false},
  {"chWatch", &CHANCE_WATCH, 45, 10, 95, false},
  {"chWarn", &CHANCE_WARNING, 70, 15, 99, false},
  {"soilMoist", &SOIL_MOIST, 40, 5, 95, false},
  {"soilSat", &SOIL_SATURATED, 70, 10, 98, false},
  {"soilCrit", &SOIL_CRITICAL, 85, 15, 99, false},
  {"rainLight", &RAIN_LIGHT, 25, 5, 95, false},
  {"rainSteady", &RAIN_STEADY, 55, 10, 98, false},
  {"rainHeavy", &RAIN_HEAVY, 80, 15, 99, false},
  {"pLoaded", &PRESS_LOADED, 30, 5, 95, false},
  {"pHigh", &PRESS_HIGH, 60, 10, 98, false},
  {"pCrit", &PRESS_CRITICAL, 85, 15, 99, false},
  {"vibLight", &VIBRATION_LIGHT, 1, 1, 40, false},
  {"vibShake", &VIBRATION_SHAKING, 3, 2, 60, false},
  {"vibStrong", &VIBRATION_STRONG, 7, 3, 99, false},
  {"distAB", &DIST_AB_M, 918, 10, 20000, false},
  {"distBC", &DIST_BC_M, 1440, 10, 20000, false},
  {"speed", &WATER_SPEED_CMS, 100, 5, 1000, false},
  {"mode", &timingMode, 0, 0, 1, false},
  {"wetThr", &WET_THRESHOLD, 800, 50, 4000, true},
  {"soilDry", &SOIL_DRY, 3200, 0, 4095, true},
  {"soilWet", &SOIL_WET, 1400, 0, 4095, true},
  {"rainDry", &RAIN_DRY, 3500, 0, 4095, true},
  {"rainWet", &RAIN_WET, 1000, 0, 4095, true},
  {"p1Rest", &PRESSURE_REST[0], 150, 0, 4095, true},
  {"p1Full", &PRESSURE_FULL[0], 3800, 0, 4095, true},
  {"p2Rest", &PRESSURE_REST[1], 150, 0, 4095, true},
  {"p2Full", &PRESSURE_FULL[1], 3800, 0, 4095, true},
  {"wDry", &WATER_DRY_RAW, -1, -1, 4095, true},
  {"wWet", &WATER_WET_RAW, -1, -1, 4095, true},
};
const int SETTING_COUNT = sizeof(SETTINGS) / sizeof(SETTINGS[0]);
const int MIN_SPAN_RAW = 300;   // dry and wet readings closer than this can't give a useful percentage
const int MIN_GAP = 5;          // percentage lines (moist / saturated / critical ...) at least this far apart

int findSetting(const char* key) {
  for (int i = 0; i < SETTING_COUNT; i++) {
    if (strcmp(SETTINGS[i].key, key) == 0) return i;
  }
  return -1;
}

bool increasing3(int a, int b, int c) {
  return b - a >= MIN_GAP && c - b >= MIN_GAP;
}

// Checks the values now in the variables. On failure names the first setting at fault.
bool settingsValid(const char** badKey) {
  for (int i = 0; i < SETTING_COUNT; i++) {
    int v = *SETTINGS[i].var;
    if (v < SETTINGS[i].lo || v > SETTINGS[i].hi) { *badKey = SETTINGS[i].key; return false; }
  }
  if (!increasing3(CHANCE_ADVISORY, CHANCE_WATCH, CHANCE_WARNING)) { *badKey = "chWatch"; return false; }
  if (!increasing3(SOIL_MOIST, SOIL_SATURATED, SOIL_CRITICAL)) { *badKey = "soilSat"; return false; }
  if (!increasing3(RAIN_LIGHT, RAIN_STEADY, RAIN_HEAVY)) { *badKey = "rainSteady"; return false; }
  if (!increasing3(PRESS_LOADED, PRESS_HIGH, PRESS_CRITICAL)) { *badKey = "pHigh"; return false; }
  if (!(VIBRATION_LIGHT < VIBRATION_SHAKING && VIBRATION_SHAKING < VIBRATION_STRONG)) { *badKey = "vibShake"; return false; }
  if (abs(SOIL_DRY - SOIL_WET) < MIN_SPAN_RAW) { *badKey = "soilWet"; return false; }
  if (abs(RAIN_DRY - RAIN_WET) < MIN_SPAN_RAW) { *badKey = "rainWet"; return false; }
  for (int i = 0; i < 2; i++) {
    if (PRESSURE_FULL[i] - PRESSURE_REST[i] < MIN_SPAN_RAW) { *badKey = i == 0 ? "p1Full" : "p2Full"; return false; }
  }
  return true;
}

void snapshotSettings(int* out) {
  for (int i = 0; i < SETTING_COUNT; i++) out[i] = *SETTINGS[i].var;
}

void restoreSettings(const int* in) {
  for (int i = 0; i < SETTING_COUNT; i++) *SETTINGS[i].var = in[i];
}

void defaultSettings(bool includeCalibration) {
  for (int i = 0; i < SETTING_COUNT; i++) {
    if (includeCalibration || !SETTINGS[i].calibration) *SETTINGS[i].var = SETTINGS[i].def;
  }
}

bool pinFormatOk(const String& pin) {
  if (pin.length() < 4 || pin.length() > 8) return false;
  for (size_t i = 0; i < pin.length(); i++) {
    if (pin[i] < '0' || pin[i] > '9') return false;
  }
  return true;
}

// Timing changed: rates measured over the old window would be wrong, so start them again.
void timingChanged() {
  applyTimingMode();
  for (int i = 0; i < TRACK_COUNT; i++) tracks[i].count = 0;
  if (rainAccumMs > RAIN_ACCUM_CAP_MS) rainAccumMs = RAIN_ACCUM_CAP_MS;
  for (int c = 0; c < CH_COUNT; c++) health[c].lastChangeMs = millis();
}

void saveSettings() {
  if (!prefsOk) return;
  for (int i = 0; i < SETTING_COUNT; i++) {
    int v = *SETTINGS[i].var;
    if (prefs.getInt(SETTINGS[i].key, INT32_MIN) != v) prefs.putInt(SETTINGS[i].key, v);
  }
}

void loadSettings() {
  prefsOk = prefs.begin("fouzante", false);
  if (!prefsOk) {
    Serial.println("Settings storage unavailable: using built-in defaults (changes won't survive a restart).");
    return;
  }
  if (FACTORY_RESET_PIN) {
    prefs.clear();
    Serial.println("FACTORY_RESET_PIN is on: saved settings cleared, PIN is 1234. Set it back to false and upload again.");
  }
  for (int i = 0; i < SETTING_COUNT; i++) *SETTINGS[i].var = prefs.getInt(SETTINGS[i].key, SETTINGS[i].def);
  const char* bad = nullptr;
  if (!settingsValid(&bad)) {
    Serial.printf("Saved setting %s is not valid; using defaults.\n", bad ? bad : "?");
    defaultSettings(true);
  }
  String pin = prefs.getString("pin", DEFAULT_PIN);
  if (!pinFormatOk(pin)) pin = DEFAULT_PIN;
  strncpy(adminPin, pin.c_str(), sizeof(adminPin) - 1);
  adminPin[sizeof(adminPin) - 1] = '\0';
}

bool pinIsDefault() {
  return strcmp(adminPin, DEFAULT_PIN) == 0;
}

long pinLockLeft(unsigned long now) {
  if (pinLockUntil == 0) return 0;
  long left = (long)(pinLockUntil - now);
  if (left <= 0) {
    pinLockUntil = 0;
    return 0;
  }
  return (left + 999) / 1000;
}

// Every change needs the admin PIN. Five wrong tries lock it for a minute.
// Returns true when the request may go ahead; otherwise the reply has already been sent.
bool requirePin() {
  unsigned long now = millis();
  long locked = pinLockLeft(now);
  if (locked > 0) {
    char out[80];
    snprintf(out, sizeof(out), "{\"ok\":false,\"error\":\"locked\",\"retry\":%ld}", locked);
    replyJson(429, out);
    return false;
  }
  String given = server.arg("pin");
  bool match = given.length() == strlen(adminPin);
  uint8_t diff = 0;
  for (size_t i = 0; i < given.length() && i < strlen(adminPin); i++) diff |= (uint8_t)(given[i] ^ adminPin[i]);
  if (match && diff == 0) {
    pinFails = 0;
    return true;
  }
  pinFails++;
  if (pinFails >= PIN_MAX_FAILS) {
    pinFails = 0;
    pinLockUntil = now + PIN_LOCK_MS;
    if (pinLockUntil == 0) pinLockUntil = 1;
    addEvent(EV_PIN_LOCK, PIN_LOCK_MS / 1000, 0);
    replyJson(429, "{\"ok\":false,\"error\":\"locked\",\"retry\":60}");
    return false;
  }
  replyJson(403, given.length() ? "{\"ok\":false,\"error\":\"pin\"}" : "{\"ok\":false,\"error\":\"pin-missing\"}");
  return false;
}

void replyJson(int code, const char* body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

// ---------- Events ----------

void appendEventToSd(const Event& e) {
  if (!sdOk) return;
  bool fresh = !SD_MMC.exists(EVENT_PATH);
  File f = SD_MMC.open(EVENT_PATH, FILE_APPEND);
  if (!f) return;
  if (fresh) f.println("time,uptime_s,event,a,b");
  char clockText[24];
  formatClock(clockText, sizeof(clockText));
  f.printf("%s,%lu,%s,%d,%d\n", clockText, (unsigned long)e.t, EVENT_NAMES[e.type], e.a, e.b);
  f.close();
}

void addEvent(uint8_t type, int a, int b) {
  unsigned long now = millis();
  // Flood guard: a flapping node or wire can't push everything else out of the list.
  if (now - eventWindowStart >= 60000UL) {
    eventWindowStart = now;
    eventsInWindow = 0;
  }
  if (eventsInWindow >= 30 && (type == EV_NODE_OFF || type == EV_NODE_ON || type == EV_FAULT_ON || type == EV_FAULT_OFF)) return;
  if (eventsInWindow >= 60) return;
  // Sensor-driven news repeats while a value hovers at a line: once a minute (10 min in field timing) is enough.
  bool repeats = type == EV_RAIN_START || type == EV_RAIN_STOP || type == EV_HEAVY_RAIN || type == EV_SOIL_SAT ||
                 type == EV_UNEVEN || type == EV_P_RELEASE || type == EV_VIB_STRONG || type == EV_WATER_CLEAR;
  if (repeats && type < 32) {
    unsigned long hold = timingMode == 1 ? 600000UL : 60000UL;
    if (eventTypeMs[type] != 0 && now - eventTypeMs[type] < hold) return;
    eventTypeMs[type] = now == 0 ? 1 : now;
  }
  eventsInWindow++;
  Event& e = events[eventNext];
  e.seq = ++eventSeq;
  e.t = now / 1000UL;
  e.type = type;
  e.a = (int16_t)clampInt(a, -32768, 32767);
  e.b = (int16_t)clampInt(b, -32768, 32767);
  eventNext = (eventNext + 1) % EVENT_LEN;
  if (eventCount < EVENT_LEN) eventCount++;
  Serial.printf("Event: %s %d %d\n", EVENT_NAMES[type], e.a, e.b);
  appendEventToSd(e);
}

void detectEvents(unsigned long now) {
  // Level: a rise is logged at once; a fall only after it has held for 30 s.
  if (eventRank < 0) eventRank = warningRank;
  if (warningRank > eventRank) {
    eventRank = warningRank;
    lowerRankSince = 0;
    addEvent(EV_LEVEL, warningRank, chance);
  } else if (warningRank < eventRank) {
    if (lowerRankSince == 0) lowerRankSince = now;
    else if (now - lowerRankSince >= 30000UL) {
      eventRank = warningRank;
      lowerRankSince = 0;
      addEvent(EV_LEVEL, warningRank, chance);
    }
  } else {
    lowerRankSince = 0;
  }

  for (int id = 1; id <= REMOTE_NODES; id++) {
    bool online = nodeOnline(id);
    if (online == nodePrev[id]) continue;
    nodePrev[id] = online;
    if (nodeSimulated(id) || now < BOOT_GRACE_MS) continue;   // start-up and test mode are not news
    if (now - nodeEventMs[id] < 20000UL && nodeEventMs[id] != 0) continue;   // flapping: keep the list readable
    nodeEventMs[id] = now;
    addEvent(online ? EV_NODE_ON : EV_NODE_OFF, id, 0);
    if (!online && !fakeNodes) today.drops[id]++;
  }

  if (soilOnline) {
    if (rainAmount >= RAIN_LIGHT) {
      rainBelowSince = 0;
      if (!rainActive) {
        rainActive = true;
        addEvent(EV_RAIN_START, rainAmount, 0);
      }
    } else if (rainActive) {
      if (rainBelowSince == 0) rainBelowSince = now;
      else if (now - rainBelowSince >= RAIN_STOP_HOLD_MS) {
        rainActive = false;
        rainBelowSince = 0;
        addEvent(EV_RAIN_STOP, rainMinutes, 0);
      }
    }
    if (!heavyActive && rainAmount >= RAIN_HEAVY) {
      heavyActive = true;
      addEvent(EV_HEAVY_RAIN, rainAmount, 0);
    } else if (heavyActive && rainAmount < RAIN_STEADY) {
      heavyActive = false;
    }
    if (!soilSatActive && soilPercent >= SOIL_SATURATED) {
      soilSatActive = true;
      addEvent(EV_SOIL_SAT, soilPercent, 0);
    } else if (soilSatActive && soilPercent < SOIL_MOIST) {
      soilSatActive = false;
    }
  }

  if (aReached && !aWetPrev) addEvent(EV_WATER_A, readings[0].online ? signalPercent(readings[0].raw) : -1, 0);
  if (bReached && !bWetPrev) addEvent(EV_WATER_B, 0, 0);
  if (cReached && !cWetPrev) addEvent(EV_WATER_C, riseSeconds, 0);
  if ((aWetPrev || bWetPrev || cWetPrev) && !aReached && !bReached && !cReached) addEvent(EV_WATER_CLEAR, 0, 0);
  aWetPrev = aReached;
  bWetPrev = bReached;
  cWetPrev = cReached;

  if (orderFault && !orderPrev) addEvent(EV_ORDER_FAULT, 0, 0);
  orderPrev = orderFault;
  if (pressureRelease && !releasePrev) addEvent(EV_P_RELEASE, maxPressure(), 0);
  releasePrev = pressureRelease;
  if (unevenLoad && !unevenPrev) addEvent(EV_UNEVEN, pressurePercent[0], pressurePercent[1]);
  unevenPrev = unevenLoad;
  bool strong = vibrationOnline && vibrationCount >= VIBRATION_STRONG;
  if (strong && !strongPrev) addEvent(EV_VIB_STRONG, vibrationCount, 0);
  strongPrev = strong;
  if (degraded != degradedPrev) addEvent(degraded ? EV_DEGRADED_ON : EV_DEGRADED_OFF, keySensorFault ? 1 : 0, 0);
  degradedPrev = degraded;
  if (sdOk != sdPrev) {
    sdPrev = sdOk;
    addEvent(sdOk ? EV_SD_OK : EV_SD_MISSING, 0, 0);
  }
}

void handleEvents() {
  uint32_t since = server.hasArg("since") ? (uint32_t)strtoul(server.arg("since").c_str(), nullptr, 10) : 0;
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  char buf[160];
  snprintf(buf, sizeof(buf), "{\"boot\":%lu,\"up\":%lu,\"epoch\":%lu,\"seq\":%lu,\"events\":[",
           (unsigned long)bootId, millis() / 1000UL, (unsigned long)bootEpoch(), (unsigned long)eventSeq);
  server.sendContent(buf);
  bool first = true;
  int start = (eventNext - eventCount + EVENT_LEN) % EVENT_LEN;
  for (int n = 0; n < eventCount; n++) {
    const Event& e = events[(start + n) % EVENT_LEN];
    if (e.seq <= since) continue;
    snprintf(buf, sizeof(buf), "%s[%lu,%lu,\"%s\",%d,%d]", first ? "" : ",", (unsigned long)e.seq,
             (unsigned long)e.t, EVENT_NAMES[e.type], e.a, e.b);
    server.sendContent(buf);
    first = false;
  }
  server.sendContent("]}");
  server.sendContent("");
}

// ---------- Trend history ----------

uint8_t pctOrNone(bool online, int value) {
  return online ? (uint8_t)clampInt(value, 0, 100) : NO_VALUE;
}

void sampleHistory(unsigned long now) {
  if (chance > histPeakChance) histPeakChance = (uint8_t)chance;
  if (warningRank > histPeakRank) histPeakRank = (uint8_t)warningRank;
  if (vibrationOnline && vibrationCount > histPeakShakes) histPeakShakes = (uint8_t)clampInt(vibrationCount, 0, 99);
  if (now - lastHistMs < HIST_MS) return;
  lastHistMs = now;
  HistSample& s = history[histNext];
  s.t = now / 1000UL;
  s.chance = histPeakChance;   // the peak inside the 30 s, so a short spike still shows
  s.rank = histPeakRank;
  s.rain = pctOrNone(soilOnline, rainAmount);
  s.soil = pctOrNone(soilOnline, soilPercent);
  s.water = pctOrNone(readings[0].online, signalPercent(readings[0].raw));
  s.p1 = pctOrNone(pressureOnline[0], pressurePercent[0]);
  s.p2 = pctOrNone(pressureOnline[1], pressurePercent[1]);
  s.shakes = vibrationOnline ? histPeakShakes : NO_VALUE;
  s.flags = (fakeNodes ? 1 : 0) | (drillOn ? 2 : 0) | (degraded ? 4 : 0) | (silenced ? 8 : 0) | (sensorFault ? 16 : 0);
  histNext = (histNext + 1) % HIST_LEN;
  if (histCount < HIST_LEN) histCount++;
  histPeakChance = (uint8_t)chance;
  histPeakRank = (uint8_t)warningRank;
  histPeakShakes = 0;
}

void handleHistory() {
  uint32_t since = server.hasArg("since") ? (uint32_t)strtoul(server.arg("since").c_str(), nullptr, 10) : 0;
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  char buf[700];
  snprintf(buf, sizeof(buf), "{\"boot\":%lu,\"up\":%lu,\"epoch\":%lu,\"step\":%lu,\"rows\":[",
           (unsigned long)bootId, millis() / 1000UL, (unsigned long)bootEpoch(), HIST_MS / 1000UL);
  server.sendContent(buf);
  int start = (histNext - histCount + HIST_LEN) % HIST_LEN;
  bool first = true;
  size_t used = 0;
  buf[0] = '\0';
  for (int n = 0; n < histCount; n++) {
    const HistSample& s = history[(start + n) % HIST_LEN];
    if (s.t <= since) continue;
    used += snprintf(buf + used, sizeof(buf) - used, "%s[%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u]", first ? "" : ",",
                     (unsigned long)s.t, s.chance, s.rank, s.rain, s.soil, s.water, s.p1, s.p2, s.shakes, s.flags);
    first = false;
    if (used > sizeof(buf) - 80) {   // send in pieces; the whole history would not fit in RAM twice
      server.sendContent(buf);
      used = 0;
      buf[0] = '\0';
    }
  }
  if (used) server.sendContent(buf);
  server.sendContent("]}");
  server.sendContent("");
}

// ---------- Today's summary ----------

void resetStats(DayStats& d, int32_t day) {
  memset(&d, 0, sizeof(d));
  d.day = day;
  d.startEpoch = nowEpoch();
  d.startUptime = millis() / 1000UL;
  d.minAir = 32767;
  d.maxAir = -32768;
  d.maxHumid = -1;
}

void writeDailyRow(const DayStats& d) {
  if (!sdOk) return;
  bool fresh = !SD_MMC.exists(DAILY_PATH);
  File f = SD_MMC.open(DAILY_PATH, FILE_APPEND);
  if (!f) return;
  if (fresh) f.println("day,rain_min,max_chance,max_soil,max_water,max_p1,max_p2,max_shakes,normal_min,advisory_min,watch_min,warning_min,drops_n1,drops_n2,drops_n3,drops_n4,silences");
  time_t dayStart = (time_t)d.day * 86400;
  struct tm parts;
  gmtime_r(&dayStart, &parts);
  char date[12];
  strftime(date, sizeof(date), "%Y-%m-%d", &parts);
  f.printf("%s,%lu,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u\n", date, (unsigned long)(d.rainSec / 60),
           d.maxChance, d.maxSoil, d.maxWater, d.maxP1, d.maxP2, d.maxShakes,
           (unsigned long)(d.levelSec[0] / 60), (unsigned long)(d.levelSec[1] / 60), (unsigned long)(d.levelSec[2] / 60),
           (unsigned long)(d.levelSec[3] / 60), d.drops[1], d.drops[2], d.drops[3], d.drops[4], d.silences);
  f.close();
}

void updateStats(unsigned long now) {
  unsigned long dt = lastStatsMs ? now - lastStatsMs : 0;
  lastStatsMs = now;
  int32_t day = localDay();
  if (day >= 0) {
    if (today.day < 0) {
      today.day = day;   // the clock just arrived: what was counted so far belongs to today
    } else if (day != today.day) {
      if (day > today.day) writeDailyRow(today);
      yesterday = today;
      haveYesterday = day > yesterday.day;
      resetStats(today, day);
    }
  }
  statsCarryMs += dt;
  unsigned long secs = statsCarryMs / 1000UL;
  statsCarryMs %= 1000UL;
  if (fakeNodes) {   // simulated storms don't belong in the day's record
    today.testSec += secs;
    return;
  }
  if (secs) {
    if (soilOnline && rainAmount >= RAIN_LIGHT) today.rainSec += secs;
    today.levelSec[clampInt(warningRank, 0, 3)] += secs;
  }
  if (chance > today.maxChance) {
    today.maxChance = (uint8_t)chance;
    today.maxChanceUp = now / 1000UL;
  }
  if (soilOnline && soilPercent > today.maxSoil) today.maxSoil = (uint8_t)soilPercent;
  if (readings[0].online) {
    int w = signalPercent(readings[0].raw);
    if (w > today.maxWater) today.maxWater = (uint8_t)w;
  }
  if (pressureOnline[0] && pressurePercent[0] > today.maxP1) today.maxP1 = (uint8_t)pressurePercent[0];
  if (pressureOnline[1] && pressurePercent[1] > today.maxP2) today.maxP2 = (uint8_t)pressurePercent[1];
  if (vibrationOnline && vibrationCount > today.maxShakes) today.maxShakes = (uint8_t)clampInt(vibrationCount, 0, 99);
  if (airOnline && health[CH_DHT].active == FK_NONE) {
    if (airTenths < today.minAir) today.minAir = (int16_t)airTenths;
    if (airTenths > today.maxAir) today.maxAir = (int16_t)airTenths;
    if (humidityTenths > today.maxHumid) today.maxHumid = (int16_t)humidityTenths;
  }
}

void addStatsJson(const DayStats& d) {
  jsonAdd("{\"day\":%ld,\"startEpoch\":%lu,\"startUp\":%lu,\"rainSec\":%lu,\"levelSec\":[%lu,%lu,%lu,%lu],",
          (long)d.day, (unsigned long)d.startEpoch, (unsigned long)d.startUptime, (unsigned long)d.rainSec,
          (unsigned long)d.levelSec[0], (unsigned long)d.levelSec[1], (unsigned long)d.levelSec[2], (unsigned long)d.levelSec[3]);
  jsonAdd("\"maxChance\":%u,\"maxChanceUp\":%lu,\"maxSoil\":%u,\"maxWater\":%u,\"maxP1\":%u,\"maxP2\":%u,\"maxShakes\":%u,",
          d.maxChance, (unsigned long)d.maxChanceUp, d.maxSoil, d.maxWater, d.maxP1, d.maxP2, d.maxShakes);
  if (d.minAir <= d.maxAir) jsonAdd("\"minAir\":%d,\"maxAir\":%d,", d.minAir, d.maxAir);
  else jsonAdd("\"minAir\":null,\"maxAir\":null,");
  if (d.maxHumid >= 0) jsonAdd("\"maxHumid\":%d,", d.maxHumid);
  else jsonAdd("\"maxHumid\":null,");
  jsonAdd("\"drops\":[%u,%u,%u,%u],\"silences\":%u,\"testSec\":%lu}", d.drops[1], d.drops[2], d.drops[3], d.drops[4],
          d.silences, (unsigned long)d.testSec);
}

void handleSummary() {
  jsonUsed = 0;
  jsonAdd("{\"boot\":%lu,\"up\":%lu,\"epoch\":%lu,\"clock\":%s,\"today\":", (unsigned long)bootId, millis() / 1000UL,
          (unsigned long)bootEpoch(), tf(clockSet));
  addStatsJson(today);
  jsonAdd(",\"yesterday\":");
  if (haveYesterday) addStatsJson(yesterday);
  else jsonAdd("null");
  jsonAdd("}");
  replyJson(200, json);
}

// ---------- Sensor fault detection ----------

void recordRecent(ChannelHealth& h, bool valid, int raw) {
  if (!valid) {
    h.recentCount = 0;
    h.recentNext = 0;
    return;
  }
  h.recent[h.recentNext] = raw;
  h.recentNext = (h.recentNext + 1) % 10;
  if (h.recentCount < 10) h.recentCount++;
}

// Average of the last few seconds; spread is max - min (a large spread means the reading isn't steady).
int recentAverage(const ChannelHealth& h, int* spread) {
  if (h.recentCount == 0) {
    if (spread) *spread = 0;
    return -1;
  }
  long sum = 0;
  int lo = 4095, hi = 0;
  for (int i = 0; i < h.recentCount; i++) {
    sum += h.recent[i];
    if (h.recent[i] < lo) lo = h.recent[i];
    if (h.recent[i] > hi) hi = h.recent[i];
  }
  if (spread) *spread = hi - lo;
  return (int)(sum / h.recentCount);
}

// Tracks change and spikes; returns the kind of trouble seen right now (FK_NONE if fine).
uint8_t observe(int ch, int raw, unsigned long now, bool checkStuck, bool checkSpikes) {
  ChannelHealth& h = health[ch];
  uint8_t kind = FK_NONE;
  if (!h.haveRaw) {
    h.haveRaw = true;
    h.lastRaw = raw;
    h.lastChangeMs = now;
    return FK_NONE;
  }
  if (raw != h.lastRaw) h.lastChangeMs = now;
  if (checkSpikes && abs(raw - h.lastRaw) >= SPIKE_RAW) {
    for (int i = 3; i > 0; i--) h.spikes[i] = h.spikes[i - 1];
    h.spikes[0] = now;
    if (h.spikeCount < 4) h.spikeCount++;
  }
  h.lastRaw = raw;
  if (checkSpikes && h.spikeCount >= 4 && now - h.spikes[3] <= SPIKE_WINDOW_MS) kind = FK_ERRATIC;
  if (checkStuck && now - h.lastChangeMs >= STUCK_MS) kind = FK_STUCK;
  return kind;
}

void judge(int ch, uint8_t kind, unsigned long now, unsigned long confirmMs) {
  ChannelHealth& h = health[ch];
  if (kind != FK_NONE) {
    h.goodSince = 0;
    if (h.pending != kind) {
      h.pending = kind;
      h.badSince = now;
    }
    if (h.active != kind && now - h.badSince >= confirmMs) {
      h.active = kind;
      h.activeSince = now;
      addEvent(EV_FAULT_ON, ch, kind);
    }
  } else {
    h.pending = FK_NONE;
    h.badSince = 0;
    if (h.active != FK_NONE) {
      if (h.goodSince == 0) h.goodSince = now;
      else if (now - h.goodSince >= FAULT_CLEAR_MS) {
        addEvent(EV_FAULT_OFF, ch, h.active);
        h.active = FK_NONE;
        h.goodSince = 0;
      }
    }
  }
}

// A channel that can't be judged (node offline or simulated) keeps its last verdict and restarts its tracking.
void skipChannel(int ch) {
  health[ch].haveRaw = false;
  health[ch].pending = FK_NONE;
  health[ch].badSince = 0;
  health[ch].spikeCount = 0;
}

void updateHealth(unsigned long now) {
  recordRecent(health[CH_SOIL], soilOnline, soilRaw);
  recordRecent(health[CH_RAIN], soilOnline, rainRaw);
  recordRecent(health[CH_NTC], soilOnline, tempRaw);
  recordRecent(health[CH_P1], pressureOnline[0], pressureRaw[0]);
  recordRecent(health[CH_P2], pressureOnline[1], pressureRaw[1]);
  recordRecent(health[CH_WATER], readings[0].online, readings[0].raw);
  recordRecent(health[CH_POINT_A], readings[1].online, readings[1].raw);
  recordRecent(health[CH_POINT_B], readings[2].online, readings[2].raw);

  if (now < BOOT_GRACE_MS) return;
  bool n1 = soilOnline && !nodeSimulated(1);
  if (n1) {
    uint8_t k = observe(CH_SOIL, soilRaw, now, true, true);
    if (soilRaw < 100 || soilRaw > 4050) k = FK_RANGE;   // capacitive probe unplugged or shorted
    judge(CH_SOIL, k, now, k == FK_STUCK ? 0 : FAULT_CONFIRM_MS);

    uint8_t r = FK_NONE;
    observe(CH_RAIN, rainRaw, now, false, false);   // a dry rain board never changes, so no stuck check
    if (airOnline && rainAmount >= RAIN_HEAVY && humidityTenths < 500) r = FK_DOUBTFUL;   // "heavy rain" in dry air
    judge(CH_RAIN, r, now, DOUBT_RAIN_MS);

    uint8_t tk = observe(CH_NTC, tempRaw, now, true, true);
    if (tempRaw <= 5 || tempRaw >= 4090) tk = FK_RANGE;
    judge(CH_NTC, tk, now, tk == FK_STUCK ? 0 : FAULT_CONFIRM_MS);

    uint8_t dk = FK_NONE;
    if (dhtFault) dk = FK_NO_READING;
    else if (airOnline && (humidityTenths < 50 || humidityTenths > 1000 || airTenths < -100 || airTenths > 600)) dk = FK_IMPLAUSIBLE;
    judge(CH_DHT, dk, now, FAULT_CONFIRM_MS);
  } else {
    skipChannel(CH_SOIL);
    skipChannel(CH_RAIN);
    skipChannel(CH_NTC);
    skipChannel(CH_DHT);
  }
  for (int i = 0; i < 2; i++) {
    int ch = i == 0 ? CH_P1 : CH_P2;
    if (pressureOnline[i] && !nodeSimulated(PRESSURE_NODE[i])) {
      // An unpressed sensor sits at a fixed value at the end of its range, so "stuck" is only judged mid-range.
      bool midRange = pressureRaw[i] > 200 && pressureRaw[i] < 3900;
      uint8_t k = observe(ch, pressureRaw[i], now, midRange, true);
      if (pressureRaw[i] >= PRESS_LIMIT_RAW) k = FK_RANGE;
      judge(ch, k, now, k == FK_STUCK ? 0 : FAULT_CONFIRM_MS);
    } else {
      skipChannel(ch);
    }
  }
  // Water sensors sit at a steady value for days when dry, so only their range is judged by the order check.
  sensorFault = false;
  keySensorFault = false;
  for (int c = 0; c < CH_COUNT; c++) {
    if (health[c].active == FK_NONE) continue;
    sensorFault = true;
    if (c == CH_SOIL || c == CH_P1 || c == CH_P2 || c == CH_WATER) keySensorFault = true;
  }
}

// ---------- Water arrival estimate ----------

unsigned long allDrySince = 0;

void updateArrival(unsigned long now) {
  bool aNow = readings[0].online && readings[0].wet;
  if (aNow && !aReached) {
    aReached = true;
    aWetMs = now == 0 ? 1 : now;
  }
  bool anyWet = aNow || readings[1].wet || readings[2].wet;   // what the sensors say now, not the latched flags
  if (anyWet) {
    allDrySince = 0;
  } else if (aReached) {
    if (allDrySince == 0) allDrySince = now;
    else if (now - allDrySince >= CHECK_RESET_MS) {
      aReached = false;
      aWetMs = 0;
      allDrySince = 0;
    }
  }

  etaState = 0;
  etaSeconds = -1;
  etaBasis = 0;
  etaOverdue = false;
  if (orderFault) return;   // the water sensors disagree, so no estimate is honest
  long long speed = WATER_SPEED_CMS;
  bool measured = false;
  if (aReached && bReached && aWetMs != 0 && bWetMs != 0 && (long)(bWetMs - aWetMs) >= 5000L) {
    long long dtMs = (long long)(bWetMs - aWetMs);
    long long s = (long long)DIST_AB_M * 100000LL / dtMs;   // cm per second
    // In field timing a speed outside 5 cm/s .. 10 m/s means a sensor problem, so the assumed speed is used.
    // Demo timing squeezes hours into minutes, so any measured speed is accepted there.
    bool plausible = timingMode == 1 ? (s >= 5 && s <= 1000) : (s >= 1);
    if (plausible) {
      speed = s;
      measured = true;
    }
  }
  if (cReached) {
    etaState = 3;
    etaSeconds = 0;
    etaBasis = measured ? 1 : 2;
    return;
  }
  long long travel = 0;
  long elapsed = 0;
  if (bReached && bWetMs != 0) {
    etaState = 2;
    travel = (long long)DIST_BC_M * 100LL / speed;
    elapsed = elapsedSec(now, bWetMs);
    etaBasis = measured ? 1 : 2;
  } else if (aReached && aWetMs != 0) {
    elapsed = elapsedSec(now, aWetMs);
    if (readings[1].online) {
      etaState = 1;
      travel = (long long)DIST_AB_M * 100LL / speed;
    } else {   // checkpoint A is offline: estimate straight to the village checkpoint
      etaState = 2;
      travel = (long long)(DIST_AB_M + DIST_BC_M) * 100LL / speed;
    }
    etaBasis = 2;
  } else {
    return;
  }
  long left = (long)(travel - elapsed);
  etaSeconds = left > 0 ? left : 0;
  etaOverdue = elapsed > travel * 2 + 60;   // long past due: absorbed, diverted, or a sensor problem
}

// ---------- Silence and drill ----------

int drillRankAt(unsigned long now) {
  if (!drillOn) return 0;
  unsigned long e = now - drillStart;
  if (e < 30000UL) return 1;
  if (e < 90000UL) return 2;
  return 3;
}

// What the buzzer, LEDs and phones act on: the real level, or the drill's if a drill is running higher.
int alarmRank(unsigned long now) {
  int d = drillRankAt(now);
  return d > warningRank ? d : warningRank;
}

void endSilence(int reason, int rank) {
  silenced = false;
  silenceUntil = 0;
  addEvent(EV_SILENCE_END, reason, rank);
}

void serviceSilence(unsigned long now, int rank) {
  if (!silenced) return;
  if (rank > silenceRank) endSilence(1, rank);                    // got worse: sound again straight away
  else if ((long)(now - silenceUntil) >= 0) endSilence(0, rank);  // time is up
  else if (rank < 2) endSilence(3, rank);                         // nothing sounding any more
}

long silenceLeft(unsigned long now) {
  if (!silenced) return 0;
  long left = (long)(silenceUntil - now);
  return left > 0 ? (left + 999) / 1000 : 0;
}

void handleSilence() {
  unsigned long now = millis();
  int rank = alarmRank(now);
  if (server.hasArg("off")) {
    if (silenced) endSilence(2, rank);
    replyJson(200, "{\"ok\":true,\"silenced\":false}");
    return;
  }
  if (rank < 2) {
    replyJson(409, "{\"ok\":false,\"error\":\"nothing\"}");
    return;
  }
  unsigned long maxMs = rank >= 3 ? SILENCE_WARNING_MAX_MS : SILENCE_WATCH_MAX_MS;
  long maxMin = (long)(maxMs / 60000UL);
  long minutes = server.hasArg("min") ? server.arg("min").toInt() : maxMin;
  if (minutes < 1) minutes = 1;
  if (minutes > maxMin) minutes = maxMin;
  silenced = true;
  silenceUntil = now + (unsigned long)minutes * 60000UL;
  if (silenceUntil == 0) silenceUntil = 1;
  silenceRank = rank;
  if (!fakeNodes) today.silences++;
  addEvent(EV_SILENCE, minutes, rank);
  char out[80];
  snprintf(out, sizeof(out), "{\"ok\":true,\"silenced\":true,\"left\":%ld}", silenceLeft(now));
  replyJson(200, out);
}

void endDrill(int reason) {
  if (!drillOn) return;
  drillOn = false;
  addEvent(EV_DRILL_END, reason, 0);
}

void serviceDrill(unsigned long now) {
  if (!drillOn) return;
  if (warningRank >= 2) endDrill(2);                  // a real alert takes over
  else if (now - drillStart >= DRILL_MS) endDrill(0);
}

void handleDrill() {
  unsigned long now = millis();
  bool on = server.arg("on") == "1";
  if (!on) {   // anyone may stop a drill
    endDrill(1);
    replyJson(200, "{\"ok\":true,\"drill\":false}");
    return;
  }
  if (!requirePin()) return;
  if (warningRank >= 2) {
    replyJson(409, "{\"ok\":false,\"error\":\"real-alert\"}");
    return;
  }
  if (drillOn) {
    replyJson(200, "{\"ok\":true,\"drill\":true}");
    return;
  }
  drillOn = true;
  drillStart = now;
  if (silenced) endSilence(2, warningRank);
  addEvent(EV_DRILL_START, DRILL_MS / 1000UL, 0);
  replyJson(200, "{\"ok\":true,\"drill\":true}");
}

// ---------- Settings page and calibration ----------

void addLiveJson(const char* name, int ch, bool online, int node) {
  int spread = 0;
  int avg = recentAverage(health[ch], &spread);
  jsonAdd("\"%s\":{\"raw\":%d,\"avg\":%d,\"spread\":%d,\"online\":%s,\"sim\":%s,\"n\":%d}", name,
          health[ch].recentCount ? health[ch].recent[(health[ch].recentNext + 9) % 10] : -1, avg, spread, tf(online),
          tf(nodeSimulated(node)), health[ch].recentCount);
}

void handleSettingsGet() {
  unsigned long now = millis();
  jsonUsed = 0;
  jsonAdd("{\"ok\":true,\"saved\":%s,\"pinDefault\":%s,\"locked\":%ld,\"test\":%s,\"values\":{", tf(prefsOk),
          tf(pinIsDefault()), pinLockLeft(now), tf(fakeNodes));
  for (int i = 0; i < SETTING_COUNT; i++) jsonAdd("%s\"%s\":%d", i ? "," : "", SETTINGS[i].key, *SETTINGS[i].var);
  jsonAdd("},\"defaults\":{");
  for (int i = 0; i < SETTING_COUNT; i++) jsonAdd("%s\"%s\":%d", i ? "," : "", SETTINGS[i].key, SETTINGS[i].def);
  jsonAdd("},\"limits\":{");
  for (int i = 0; i < SETTING_COUNT; i++) jsonAdd("%s\"%s\":[%d,%d]", i ? "," : "", SETTINGS[i].key, SETTINGS[i].lo, SETTINGS[i].hi);
  jsonAdd("},\"timing\":{\"riseSteady\":%d,\"riseFast\":%d,\"riseSurge\":%d,\"pressRise\":%d,\"pressFast\":%d,\"pressSurge\":%d,",
          RISE_STEADY, RISE_FAST, RISE_SURGE, PRESS_RISE_STEADY, PRESS_FAST, PRESS_SURGE);
  jsonAdd("\"checkFast\":%d,\"checkMed\":%d,\"checkSlow\":%d,\"rainFullMin\":%lu,\"rainForgetMin\":%lu,\"rateWindowS\":%lu,\"stuckMin\":%lu},",
          CHECK_FAST_S, CHECK_MED_S, CHECK_SLOW_S, RAIN_FULL_MS / 60000UL, RAIN_FORGET_MS / 60000UL,
          RATE_WINDOW_MS / 1000UL, STUCK_MS / 60000UL);
  jsonAdd("\"live\":{");
  addLiveJson("soil", CH_SOIL, soilOnline, 1);
  jsonAdd(",");
  addLiveJson("rain", CH_RAIN, soilOnline, 1);
  jsonAdd(",");
  addLiveJson("p1", CH_P1, pressureOnline[0], 3);
  jsonAdd(",");
  addLiveJson("p2", CH_P2, pressureOnline[1], 2);
  jsonAdd(",");
  addLiveJson("water", CH_WATER, readings[0].online, 3);
  jsonAdd(",");
  addLiveJson("pointA", CH_POINT_A, readings[1].online, 4);
  jsonAdd(",");
  addLiveJson("pointB", CH_POINT_B, readings[2].online, 5);
  jsonAdd("}}");
  replyJson(200, json);
}

void sendSettingError(int code, const char* error, const char* key) {
  char out[120];
  snprintf(out, sizeof(out), "{\"ok\":false,\"error\":\"%s\",\"field\":\"%s\"}", error, key ? key : "");
  replyJson(code, out);
}

void handleSettingsPost() {
  if (!requirePin()) return;
  String action = server.arg("action");
  if (action == "pin") {
    String next = server.arg("newPin");
    if (!pinFormatOk(next)) {
      sendSettingError(400, "pin-format", "newPin");
      return;
    }
    strncpy(adminPin, next.c_str(), sizeof(adminPin) - 1);
    adminPin[sizeof(adminPin) - 1] = '\0';
    if (prefsOk) prefs.putString("pin", adminPin);
    addEvent(EV_SETTINGS, 2, 0);
    replyJson(200, prefsOk ? "{\"ok\":true}" : "{\"ok\":true,\"warning\":\"not-saved\"}");
    return;
  }
  if (action == "test") {
    bool on = server.arg("on") == "1";
    if (on != fakeNodes) {
      if (!on) stopFakeNodes();
      fakeNodes = on;
      addEvent(on ? EV_TEST_ON : EV_TEST_OFF, 0, 0);
    }
    replyJson(200, on ? "{\"ok\":true,\"test\":true}" : "{\"ok\":true,\"test\":false}");
    return;
  }
  int before[SETTING_COUNT];
  snapshotSettings(before);
  int oldMode = timingMode;
  if (action == "defaults") {
    defaultSettings(false);
  } else if (action == "calDefaults") {
    for (int i = 0; i < SETTING_COUNT; i++) {
      if (SETTINGS[i].calibration) *SETTINGS[i].var = SETTINGS[i].def;
    }
  } else {
    for (int i = 0; i < SETTING_COUNT; i++) {
      if (!server.hasArg(SETTINGS[i].key)) continue;
      String text = server.arg(SETTINGS[i].key);
      text.trim();
      char* end = nullptr;
      long v = strtol(text.c_str(), &end, 10);
      if (text.length() == 0 || (end && *end != '\0')) {
        restoreSettings(before);
        sendSettingError(400, "number", SETTINGS[i].key);
        return;
      }
      *SETTINGS[i].var = (int)clampInt((int)v, -100000, 100000);
    }
  }
  const char* bad = nullptr;
  if (!settingsValid(&bad)) {
    restoreSettings(before);
    sendSettingError(400, "invalid", bad);
    return;
  }
  if (timingMode != oldMode) timingChanged();
  saveSettings();
  updateSensors();   // show the effect straight away rather than on the next half-second tick
  addEvent(EV_SETTINGS, action == "defaults" ? 1 : (action == "calDefaults" ? 3 : 0), timingMode);
  replyJson(200, prefsOk ? "{\"ok\":true}" : "{\"ok\":true,\"warning\":\"not-saved\"}");
}

// Saves the reading a sensor gives right now as its dry / wet (or rest / full) point.
void handleCalibrate() {
  if (!requirePin()) return;
  String sensor = server.arg("sensor");
  String point = server.arg("point");
  int ch = -1, node = 0;
  bool online = false;
  if (sensor == "soil") { ch = CH_SOIL; node = 1; online = soilOnline; }
  else if (sensor == "rain") { ch = CH_RAIN; node = 1; online = soilOnline; }
  else if (sensor == "p1") { ch = CH_P1; node = 3; online = pressureOnline[0]; }
  else if (sensor == "p2") { ch = CH_P2; node = 2; online = pressureOnline[1]; }
  else if (sensor == "water") { ch = CH_WATER; node = 3; online = readings[0].online; }
  else if (sensor == "pointA") { ch = CH_POINT_A; node = 4; online = readings[1].online; }
  else if (sensor == "pointB") { ch = CH_POINT_B; node = 5; online = readings[2].online; }
  if (ch < 0) {
    sendSettingError(400, "sensor", "sensor");
    return;
  }
  if (!online) {
    sendSettingError(409, "offline", sensor.c_str());
    return;
  }
  if (nodeSimulated(node)) {
    sendSettingError(409, "simulated", sensor.c_str());
    return;
  }
  int spread = 0;
  int avg = recentAverage(health[ch], &spread);
  if (health[ch].recentCount < 4 || avg < 0) {
    sendSettingError(409, "wait", sensor.c_str());
    return;
  }
  if (spread > MIN_SPAN_RAW) {
    sendSettingError(409, "unsteady", sensor.c_str());
    return;
  }
  int before[SETTING_COUNT];
  snapshotSettings(before);
  int code = 0;   // which point: 0 dry/rest, 1 wet/full
  bool water = ch == CH_WATER || ch == CH_POINT_A || ch == CH_POINT_B;
  if (point == "dry" || point == "rest") code = 0;
  else if (point == "wet" || point == "full") code = 1;
  else {
    sendSettingError(400, "point", "point");
    return;
  }
  const char* pairKey = nullptr;
  if (ch == CH_SOIL) { (code ? SOIL_WET : SOIL_DRY) = avg; pairKey = "soilWet"; }
  else if (ch == CH_RAIN) { (code ? RAIN_WET : RAIN_DRY) = avg; pairKey = "rainWet"; }
  else if (ch == CH_P1) { (code ? PRESSURE_FULL[0] : PRESSURE_REST[0]) = avg; pairKey = "p1Full"; }
  else if (ch == CH_P2) { (code ? PRESSURE_FULL[1] : PRESSURE_REST[1]) = avg; pairKey = "p2Full"; }
  else if (water) {
    (code ? WATER_WET_RAW : WATER_DRY_RAW) = avg;
    pairKey = "wWet";
    if (WATER_DRY_RAW >= 0 && WATER_WET_RAW >= 0) {
      if (WATER_WET_RAW - WATER_DRY_RAW < MIN_SPAN_RAW) {
        restoreSettings(before);
        sendSettingError(409, "too-close", pairKey);
        return;
      }
      WET_THRESHOLD = clampInt((WATER_DRY_RAW + WATER_WET_RAW) / 2, 50, 4000);
    }
  }
  const char* bad = nullptr;
  if (!settingsValid(&bad)) {
    restoreSettings(before);
    sendSettingError(409, "too-close", bad ? bad : pairKey);
    return;
  }
  saveSettings();
  updateSensors();
  addEvent(EV_CALIBRATE, ch, code);
  char out[120];
  snprintf(out, sizeof(out), "{\"ok\":true,\"value\":%d,\"wetThr\":%d,\"waiting\":%s}", avg, WET_THRESHOLD,
           tf(water && (WATER_DRY_RAW < 0 || WATER_WET_RAW < 0)));
  replyJson(200, out);
}

void addFaultsJson(unsigned long now) {
  jsonAdd("\"sensorFault\":%s,\"faults\":[", tf(sensorFault));
  bool first = true;
  for (int c = 0; c < CH_COUNT; c++) {
    if (health[c].active == FK_NONE) continue;
    jsonAdd("%s{\"ch\":\"%s\",\"kind\":\"%s\",\"node\":%d,\"for\":%ld}", first ? "" : ",", CHANNEL_NAMES[c],
            FAULT_NAMES[health[c].active], CHANNEL_NODE[c], elapsedSec(now, health[c].activeSince));
    first = false;
  }
  jsonAdd("]");
}

// ---------- Serial ----------

void printStatus() {
  unsigned long now = millis();
  for (int id = 1; id <= REMOTE_NODES; id++) {
    int age = nodeAge(id, now);
    Serial.printf("Node %d: %s", id, nodeOnline(id) ? "online" : (age < 0 ? "waiting" : "OFFLINE"));
    if (age >= 0) Serial.printf("  last %ds ago  %d dBm  %lu packets", age, nodeView[id].rssi,
                                (unsigned long)nodeView[id].packets);
    Serial.println();
  }
  Serial.printf("Water A %d%% (rise %d %%/min)  point A %s  point B %s\n",
                signalPercent(readings[0].raw), risePerMin, readings[1].wet ? "WET" : "dry",
                readings[2].wet ? "WET" : "dry");
  Serial.printf("Rain %d%% (%d min)  soil %d%%  P1 %d%% (%d)  P2 %d%% (%d)  shakes %d\n",
                rainAmount, rainMinutes, soilPercent, pressurePercent[0], pressureRate[0],
                pressurePercent[1], pressureRate[1], vibrationCount);
  if (airOnline) Serial.printf("Air %.1f C  humidity %.1f%%\n", airTenths / 10.0f, humidityTenths / 10.0f);
  else if (dhtFault) Serial.println("DHT11 on Node 1 gives no reading");
  if (orderFault) Serial.println("Checkpoint order fault: B wet before A");
  if (unevenLoad) Serial.println("Uneven load between P1 and P2");
  if (pressureRelease) Serial.println("Sudden pressure drop: soil may have moved");
  if (degraded) Serial.println("Two or more nodes offline: monitoring degraded");
  if (fakeNodes) Serial.println("TEST MODE: nodes without hardware are simulated (/api/fake?on=0 to stop)");
  if (drillOn) Serial.println("DRILL running: alarms are practice, not a real alert");
  if (silenced) Serial.printf("Buzzer silenced for %lds more\n", silenceLeft(now));
  for (int c = 0; c < CH_COUNT; c++) {
    if (health[c].active != FK_NONE) Serial.printf("Sensor fault: %s %s\n", CHANNEL_NAMES[c], FAULT_NAMES[health[c].active]);
  }
  if (etaState == 1 || etaState == 2)
    Serial.printf("Water may reach checkpoint %c in about %lds%s\n", etaState == 1 ? 'A' : 'B', etaSeconds,
                  etaOverdue ? " (overdue)" : "");
  Serial.printf("Chance %d%%  level %s  SD %s  timing %s\n\n", chance, levelName(warningRank),
                sdOk ? "ok" : "missing", timingMode == 1 ? "field" : "demo");
}

// ---------- Setup and loop ----------

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(ALERT_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(ALERT_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  tone(BUZZER_PIN, BUZZER_HZ, 200);
  delay(300);
  noTone(BUZZER_PIN);

  analogReadResolution(12);
  pinMode(WATER_C_PIN, INPUT);
  analogSetPinAttenuation(WATER_C_PIN, ADC_11db);

  bootId = esp_random();
  if (bootId == 0) bootId = 1;
  loadSettings();
  applyTimingMode();
  resetStats(today, -1);
  memset(health, 0, sizeof(health));
  for (int id = 0; id <= REMOTE_NODES; id++) {
    nodePrev[id] = false;
    nodeEventMs[id] = 0;
  }
  Serial.printf("Timing: %s. Admin PIN %s.\n", timingMode == 1 ? "field (hours)" : "demo (minutes)",
                pinIsDefault() ? "is still the default 1234 - change it on the Settings page" : "is set");

  printMemory("at boot");
  if (startNetwork()) startEspNow();   // Wi-Fi first, before anything else takes internal RAM
  startSd();
  sdPrev = sdOk;
  addEvent(EV_BOOT, timingMode, sdOk ? 1 : 0);
  printMemory("after start-up");

  server.on("/api/levels", handleApi);
  server.on("/api/beep", handleBeep);
  server.on("/api/time", handleTime);
  server.on("/api/fake", handleFake);
  server.on("/api/history", HTTP_GET, handleHistory);
  server.on("/api/events", HTTP_GET, handleEvents);
  server.on("/api/summary", HTTP_GET, handleSummary);
  server.on("/api/silence", handleSilence);
  server.on("/api/drill", handleDrill);
  server.on("/api/settings", HTTP_GET, handleSettingsGet);
  server.on("/api/settings", HTTP_POST, handleSettingsPost);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/log.csv", handleLog);
  server.on("/favicon.ico", handleFavicon);
  server.onNotFound(handleFile);
  webEtag = String("\"") + __DATE__ + "-" + __TIME__ + "\"";
  webEtag.replace(" ", "");
  webEtag.replace(":", "");
  static const char* WANTED_HEADERS[] = {"If-None-Match"};
  server.collectHeaders(WANTED_HEADERS, 1);
  startWebServer();

  updateSensors();
  lastReadMs = millis();
  lastSerialMs = millis();
  lastLogMs = millis();
  driveAlerts(lastReadMs);
  Serial.println("Landslide hub is ready");
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  unsigned long now = millis();
  serviceNetwork(now);
  updateFakeNodes(now);
  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;
    updateSensors();
    detectEvents(now);
  }
  driveAlerts(now);
  serviceLog(now);
  sampleHistory(now);
  updateStats(now);
  if (now - lastSerialMs >= SERIAL_INTERVAL_MS) {
    lastSerialMs = now;
    printStatus();
  }
}

