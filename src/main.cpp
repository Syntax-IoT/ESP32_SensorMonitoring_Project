#include "../include/config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <time.h> // NTP / struct tm — needed for scheduled timer mode

// ─────────────────────────────────────────────────────────
// Objects
// ─────────────────────────────────────────────────────────
DHT dht(PIN_DHT22, DHT_TYPE);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ─────────────────────────────────────────────────────────
// Relay mode enum
// ─────────────────────────────────────────────────────────
enum RelayMode {
  MODE_MANUAL,
  MODE_AUTO,
  MODE_TIMER_CYCLIC,
  MODE_TIMER_SCHEDULED
};

// ─────────────────────────────────────────────────────────
// Relay state
// ─────────────────────────────────────────────────────────
struct RelayState {
  bool physicalOn = false; // actual GPIO state
  RelayMode mode = MODE_MANUAL;

  // manual
  bool manualTarget = false;

  // auto
  char autoSensorKey[16] = "co2_1";
  float setpointOn = 1000.0f;
  float setpointOff = 800.0f;

  // cyclic timer (milliseconds)
  unsigned long cyclicOnMs = 30UL * 60000UL;
  unsigned long cyclicOffMs = 10UL * 60000UL;
  unsigned long timerStart = 0; // millis() when current phase began
  bool timerPhaseOn = false;    // are we in the ON phase?

  // scheduled timer
  int schedStartHour = 22;
  int schedStartMin = 0;
  unsigned long schedOnMs = 30UL * 60000UL;
} relay;

// ─────────────────────────────────────────────────────────
// Sensor cache  (updated every publishSensors call)
// ─────────────────────────────────────────────────────────
struct SensorCache {
  float temp = NAN;
  float hum = NAN;
  float co2 = NAN; // MQ135 → CO₂ proxy (raw ADC average, or scaled)
  float nh3 = NAN; // MQ137 → NH₃ proxy
} sensorCache;

// ─────────────────────────────────────────────────────────
// Global flags / timers
// ─────────────────────────────────────────────────────────
bool sensorsWarmedUp = false;
unsigned long lastPublish = 0;
unsigned long lastReconnect = 0;
unsigned long bootTime = 0;
int reconnectAttempts = 0;

// ─────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────
void connectWiFi();
void connectMQTT();
void syncNTP();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void publishSensors();
void publishRelayState();
void applyRelayMode();
void setRelayGPIO(bool on);
float readMQAverage(int pin);
float lookupSensorValue(const char *key);

// ─────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n[BOOT] ========================");
  Serial.printf("[BOOT] Device  : %s\n", DEVICE_ID);
  Serial.printf("[BOOT] Topics  : %s | %s | %s\n", TOPIC_DATA, TOPIC_SET,
                TOPIC_RELAY);
  Serial.printf("[BOOT] Broker  : %s:%d\n", MQTT_HOST, MQTT_PORT);
  Serial.println("[BOOT] ========================");

  bootTime = millis();

  // Relay pin — start OFF (active-LOW module: HIGH = OFF)
  pinMode(PIN_RELAY, OUTPUT);
  setRelayGPIO(false);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // DHT22
  dht.begin();
  Serial.println("[SENSOR] DHT22 ready");
  Serial.printf("[SENSOR] MQ warm-up: %d s\n", ADC_WARMUP_MS / 1000);

  // Cyclic timer: start in OFF phase so the relay doesn't lurch ON at boot
  relay.timerStart = millis();
  relay.timerPhaseOn = false;

  connectWiFi();
  syncNTP();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);

  connectMQTT();
}

// ─────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────
void loop() {
  // MQ warm-up gate
  if (!sensorsWarmedUp && (millis() - bootTime >= ADC_WARMUP_MS)) {
    sensorsWarmedUp = true;
    Serial.println("[SENSOR] MQ sensors warmed up — readings now valid");
  }

  // Wi-Fi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Connection lost — reconnecting");
    connectWiFi();
  }

  // MQTT watchdog
  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastReconnect >= RECONNECT_INTERVAL_MS) {
      lastReconnect = now;
      connectMQTT();
    }
    return;
  }

  mqtt.loop();

  // Sensor publish cadence
  if (millis() - lastPublish >= SENSOR_INTERVAL_MS) {
    lastPublish = millis();
    publishSensors(); // updates sensorCache, then publishes
  }

  // Apply the active relay mode every loop tick
  // (timer modes need continuous evaluation; auto mode re-reads sensor cache)
  applyRelayMode();
}

// ─────────────────────────────────────────────────────────
// connectWiFi()
// ─────────────────────────────────────────────────────────
void connectWiFi() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(WIFI_TIMEOUT_S);

  bool ok = wm.autoConnect(
      WIFI_AP_NAME, strlen(WIFI_AP_PASSWORD) > 0 ? WIFI_AP_PASSWORD : nullptr);

  if (!ok) {
    Serial.println("[WIFI] Portal timed out — restarting");
    ESP.restart();
  }

  Serial.printf("[WIFI] Connected — IP: %s  RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  reconnectAttempts = 0;
}

// ─────────────────────────────────────────────────────────
// syncNTP()
// Needed for scheduled timer mode.  Uses pool.ntp.org.
// Adjust timezone offset (seconds) as needed.
// ─────────────────────────────────────────────────────────
void syncNTP() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("[NTP] Syncing");
  struct tm tm {};
  unsigned long deadline = millis() + 10000UL;
  while (!getLocalTime(&tm) && millis() < deadline) {
    delay(500);
    Serial.print('.');
  }
  Serial.println(
      getLocalTime(&tm) ? " OK" : " FAILED (scheduled mode may be inaccurate)");
}

// ─────────────────────────────────────────────────────────
// connectMQTT()
// ─────────────────────────────────────────────────────────
void connectMQTT() {
  if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
    Serial.println("[MQTT] Max reconnect attempts — restarting");
    ESP.restart();
  }

  Serial.printf("[MQTT] Connecting to %s:%d as %s ...\n", MQTT_HOST, MQTT_PORT,
                MQTT_CLIENT_ID);

  bool ok = (strlen(MQTT_USER) > 0)
                ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)
                : mqtt.connect(MQTT_CLIENT_ID);

  if (ok) {
    reconnectAttempts = 0;
    Serial.println("[MQTT] Connected");
    mqtt.subscribe(TOPIC_SET, 1);
    Serial.printf("[MQTT] Subscribed to %s\n", TOPIC_SET);
  } else {
    reconnectAttempts++;
    Serial.printf("[MQTT] Failed (rc=%d) — attempt %d/%d\n", mqtt.state(),
                  reconnectAttempts, MAX_RECONNECT_ATTEMPTS);
  }
}

// ─────────────────────────────────────────────────────────
// mqttCallback()
//
// Accepts the full platform command envelope:
//   {"mode":"manual",  "mode_config":{"type":"manual","state":1}}
//   {"mode":"auto",
//   "mode_config":{"sensor_key":"co2_1","setpoint_on":25,"setpoint_off":30}}
//   {"mode":"timer",
//   "mode_config":{"type":"cyclic","cyclic":{"on_duration_min":30,"off_duration_min":10}}}
//   {"mode":"timer",
//   "mode_config":{"type":"scheduled","scheduled":{"start_time":"22:00","on_duration_min":30}}}
//
// Also still accepts legacy plain strings: "ON" / "OFF" / "1" / "0"
// ─────────────────────────────────────────────────────────
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.printf("\n[MQTT] Received [%s]: %s\n", topic, msg);

  if (strcmp(topic, TOPIC_SET) != 0)
    return;

  // ── Legacy plain strings ─────────────────────────────
  if (strcasecmp(msg, "ON") == 0 || strcmp(msg, "1") == 0) {
    relay.mode = MODE_MANUAL;
    relay.manualTarget = true;
    publishRelayState();
    return;
  }
  if (strcasecmp(msg, "OFF") == 0 || strcmp(msg, "0") == 0) {
    relay.mode = MODE_MANUAL;
    relay.manualTarget = false;
    publishRelayState();
    return;
  }

  // ── JSON command ─────────────────────────────────────
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err != DeserializationError::Ok) {
    Serial.println("[MQTT] JSON parse error — ignored");
    return;
  }

  // Legacy simple relay bool: {"relay": true}
  if (doc["relay"].is<bool>() && !doc.containsKey("mode")) {
    relay.mode = MODE_MANUAL;
    relay.manualTarget = doc["relay"].as<bool>();
    publishRelayState();
    return;
  }

  const char *mode = doc["mode"] | "";
  JsonObject cfg = doc["mode_config"].as<JsonObject>();

  // ── Manual ───────────────────────────────────────────
  if (strcmp(mode, "manual") == 0) {
    relay.mode = MODE_MANUAL;
    // honour "state" from mode_config (1=ON, 0=OFF)
    relay.manualTarget = (cfg["state"].as<int>() != 0);
    Serial.printf("[RELAY] Mode=MANUAL  target=%s\n",
                  relay.manualTarget ? "ON" : "OFF");
  }

  // ── Auto ─────────────────────────────────────────────
  else if (strcmp(mode, "auto") == 0) {
    relay.mode = MODE_AUTO;
    strncpy(relay.autoSensorKey, cfg["sensor_key"] | "co2_1",
            sizeof(relay.autoSensorKey) - 1);
    relay.autoSensorKey[sizeof(relay.autoSensorKey) - 1] = '\0';
    relay.setpointOn = cfg["setpoint_on"] | 1000.0f;
    relay.setpointOff = cfg["setpoint_off"] | 800.0f;
    Serial.printf("[RELAY] Mode=AUTO  key=%s  on≥%.1f  off≤%.1f\n",
                  relay.autoSensorKey, relay.setpointOn, relay.setpointOff);
  }

  // ── Timer ────────────────────────────────────────────
  else if (strcmp(mode, "timer") == 0) {
    const char *timerType = cfg["type"] | "cyclic";

    if (strcmp(timerType, "cyclic") == 0) {
      relay.mode = MODE_TIMER_CYCLIC;
      JsonObject cyc = cfg["cyclic"].as<JsonObject>();
      relay.cyclicOnMs = (cyc["on_duration_min"] | 30) * 60000UL;
      relay.cyclicOffMs = (cyc["off_duration_min"] | 10) * 60000UL;
      // Reset phase to OFF so behaviour is deterministic on new command
      relay.timerPhaseOn = false;
      relay.timerStart = millis();
      Serial.printf("[RELAY] Mode=CYCLIC  on=%lu min  off=%lu min\n",
                    relay.cyclicOnMs / 60000UL, relay.cyclicOffMs / 60000UL);
    } else if (strcmp(timerType, "scheduled") == 0) {
      relay.mode = MODE_TIMER_SCHEDULED;
      JsonObject sched = cfg["scheduled"].as<JsonObject>();
      const char *startTime = sched["start_time"] | "00:00";
      sscanf(startTime, "%d:%d", &relay.schedStartHour, &relay.schedStartMin);
      relay.schedOnMs = (sched["on_duration_min"] | 30) * 60000UL;
      Serial.printf("[RELAY] Mode=SCHEDULED  start=%02d:%02d  on=%lu min\n",
                    relay.schedStartHour, relay.schedStartMin,
                    relay.schedOnMs / 60000UL);
    } else {
      Serial.printf("[RELAY] Unknown timer type '%s' — ignored\n", timerType);
      return;
    }
  }

  else {
    Serial.printf("[RELAY] Unknown mode '%s' — ignored\n", mode);
    return;
  }

  // Publish the new configuration immediately
  publishRelayState();
}

// ─────────────────────────────────────────────────────────
// applyRelayMode()
// Called every loop() iteration.
// Computes desired physical relay state from the active mode
// and calls setRelayGPIO() only when it needs to change.
// ─────────────────────────────────────────────────────────
void applyRelayMode() {
  bool desired = relay.physicalOn; // default = keep current state

  switch (relay.mode) {

  // ── Manual: target set directly by command ────────
  case MODE_MANUAL:
    desired = relay.manualTarget;
    break;

  // ── Auto: compare cached sensor value to setpoints ─
  case MODE_AUTO: {
    float val = lookupSensorValue(relay.autoSensorKey);
    if (!isnan(val)) {
      if (!relay.physicalOn && val >= relay.setpointOn)
        desired = true;
      else if (relay.physicalOn && val <= relay.setpointOff)
        desired = false;
      // hysteresis: if between setpoints, keep current state
    }
    break;
  }

  // ── Cyclic timer ──────────────────────────────────
  case MODE_TIMER_CYCLIC: {
    unsigned long elapsed = millis() - relay.timerStart;
    unsigned long phase =
        relay.timerPhaseOn ? relay.cyclicOnMs : relay.cyclicOffMs;
    if (elapsed >= phase) {
      relay.timerPhaseOn = !relay.timerPhaseOn;
      relay.timerStart = millis();
      Serial.printf("[RELAY] Cyclic → phase now %s\n",
                    relay.timerPhaseOn ? "ON" : "OFF");
      // publish updated state after phase flip
      desired = relay.timerPhaseOn;
      if (desired != relay.physicalOn) {
        setRelayGPIO(desired);
        publishRelayState();
        return;
      }
    }
    desired = relay.timerPhaseOn;
    break;
  }

  // ── Scheduled timer ───────────────────────────────
  case MODE_TIMER_SCHEDULED: {
    struct tm now {};
    if (!getLocalTime(&now))
      break; // NTP not synced yet

    int nowMinutes = now.tm_hour * 60 + now.tm_min;
    int startMinutes = relay.schedStartHour * 60 + relay.schedStartMin;
    int onDurMin = (int)(relay.schedOnMs / 60000UL);

    // Handle midnight wrap: treat the day as 0..1439 minutes
    int diff = nowMinutes - startMinutes;
    if (diff < 0)
      diff += 1440; // wrap into [0, 1439]

    desired = (diff < onDurMin);
    break;
  }
  }

  // Only toggle GPIO + publish when state actually changes
  if (desired != relay.physicalOn) {
    setRelayGPIO(desired);
    publishRelayState();
  }
}

// ─────────────────────────────────────────────────────────
// setRelayGPIO()
// Toggles the physical relay pin (active-LOW module).
// ─────────────────────────────────────────────────────────
void setRelayGPIO(bool on) {
  relay.physicalOn = on;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  Serial.printf("[RELAY] GPIO → %s\n", on ? "ON" : "OFF");
}

// ─────────────────────────────────────────────────────────
// publishRelayState()
// Publishes the full relay state JSON to TOPIC_RELAY.
//
// Shape matches relay_test_push.py / relay_test_sub.py:
//   {"relay":"extr_1","state":0|1,"mode":"manual|auto|timer","mode_config":{...}}
// ─────────────────────────────────────────────────────────
void publishRelayState() {
  JsonDocument doc;
  doc["relay"] = "extr_1";
  doc["state"] = relay.physicalOn ? 1 : 0;

  JsonObject cfg = doc["mode_config"].to<JsonObject>();

  switch (relay.mode) {
  case MODE_MANUAL:
    doc["mode"] = "manual";
    cfg["type"] = "manual";
    cfg["state"] = relay.manualTarget ? 1 : 0;
    break;

  case MODE_AUTO:
    doc["mode"] = "auto";
    cfg["sensor_key"] = relay.autoSensorKey;
    cfg["setpoint_on"] = relay.setpointOn;
    cfg["setpoint_off"] = relay.setpointOff;
    break;

  case MODE_TIMER_CYCLIC: {
    doc["mode"] = "timer";
    cfg["type"] = "cyclic";
    JsonObject cyc = cfg["cyclic"].to<JsonObject>();
    cyc["on_duration_min"] = relay.cyclicOnMs / 60000UL;
    cyc["off_duration_min"] = relay.cyclicOffMs / 60000UL;
    break;
  }

  case MODE_TIMER_SCHEDULED: {
    doc["mode"] = "timer";
    cfg["type"] = "scheduled";
    JsonObject sched = cfg["scheduled"].to<JsonObject>();
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", relay.schedStartHour,
             relay.schedStartMin);
    sched["start_time"] = buf;
    sched["on_duration_min"] = relay.schedOnMs / 60000UL;
    break;
  }
  }

  char buffer[384];
  size_t n = serializeJson(doc, buffer);

  if (mqtt.publish(TOPIC_RELAY, (const uint8_t *)buffer, n, true)) {
    Serial.printf("[MQTT] Relay state published → %s\n", buffer);
  } else {
    Serial.println("[MQTT] Relay publish failed");
  }
}

// ─────────────────────────────────────────────────────────
// readMQAverage()
// Oversampled ADC read for MQ sensor pins.
// ─────────────────────────────────────────────────────────
float readMQAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return (float)sum / ADC_SAMPLES;
}

// ─────────────────────────────────────────────────────────
// lookupSensorValue()
// Returns a cached sensor reading by platform key name.
// Returns NAN if the key is unknown.
// ─────────────────────────────────────────────────────────
float lookupSensorValue(const char *key) {
  if (strcmp(key, "temp_1") == 0)
    return sensorCache.temp;
  if (strcmp(key, "hum_1") == 0)
    return sensorCache.hum;
  if (strcmp(key, "co2_1") == 0)
    return sensorCache.co2;
  if (strcmp(key, "nh3_1") == 0)
    return sensorCache.nh3;
  return NAN;
}

// ─────────────────────────────────────────────────────────
// publishSensors()
// Reads all sensors, updates sensorCache, builds the
// platform-compatible JSON array, and publishes to TOPIC_DATA.
//
// JSON shape (matches sensors_push.py):
//   {
//     "timestamp": "2025-01-01T12:00:00.000Z",
//     "sensors": [
//       {"key": "temp_1",  "value": 25.0},
//       {"key": "hum_1",   "value": 50.0},
//       {"key": "co2_1",   "value": 400.0},
//       {"key": "nh3_1",   "value": 10.0}
//     ]
//   }
// ─────────────────────────────────────────────────────────
void publishSensors() {
  // ── DHT22 ────────────────────────────────────────────
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("[SENSOR] DHT22 read failed — skipping publish");
    return;
  }

  // ── MQ sensors (raw ADC average) ─────────────────────
  float co2 = readMQAverage(PIN_MQ135);
  float nh3 = readMQAverage(PIN_MQ137);

  // Update shared cache (used by auto mode)
  sensorCache.temp = temp;
  sensorCache.hum = hum;
  sensorCache.co2 = co2;
  sensorCache.nh3 = nh3;

  Serial.printf("[SENSOR] T:%.1f°C  H:%.1f%%  co2_1:%.0f  nh3_1:%.0f%s\n", temp,
                hum, co2, nh3, sensorsWarmedUp ? "" : "  [WARMING UP]");

  // ── Build ISO-8601 timestamp ──────────────────────────
  char timestamp[32];
  struct tm now {};
  if (getLocalTime(&now)) {
    // NTP synced
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d.000Z",
             now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour,
             now.tm_min, now.tm_sec);
  } else {
    // Fallback: millis-based placeholder
    unsigned long s = millis() / 1000;
    snprintf(timestamp, sizeof(timestamp), "1970-01-01T%02lu:%02lu:%02lu.000Z",
             (s / 3600) % 24, (s / 60) % 60, s % 60);
  }

  // ── Build JSON ───────────────────────────────────────
  JsonDocument doc;
  doc["timestamp"] = timestamp;

  JsonArray sensors = doc["sensors"].to<JsonArray>();

  JsonObject s0 = sensors.add<JsonObject>();
  s0["key"] = "temp_1";
  s0["value"] = round(temp * 100.0f) / 100.0f;

  JsonObject s1 = sensors.add<JsonObject>();
  s1["key"] = "hum_1";
  s1["value"] = round(hum * 100.0f) / 100.0f;

  JsonObject s2 = sensors.add<JsonObject>();
  s2["key"] = "co2_1";
  s2["value"] = round(co2 * 100.0f) / 100.0f;

  JsonObject s3 = sensors.add<JsonObject>();
  s3["key"] = "nh3_1";
  s3["value"] = round(nh3 * 100.0f) / 100.0f;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  // ── Publish → syntax/{deviceID}/data ─────────────────
  if (mqtt.publish(TOPIC_DATA, (const uint8_t *)buffer, n)) {
    Serial.printf("[MQTT] Sensor data published (%d bytes) → %s\n", (int)n,
                  TOPIC_DATA);
  } else {
    Serial.println("[MQTT] Sensor publish failed");
  }
}