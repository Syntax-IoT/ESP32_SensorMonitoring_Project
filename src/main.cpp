#include "../include/config.h"
#include "../include/logger.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <time.h>

Logger logger;

// ─────────────────────────────────────────────────────────
// Runtime flags
// ─────────────────────────────────────────────────────────
bool sensorsWarmedUp = false;
unsigned long lastPublish = 0;
unsigned long lastReconnect = 0;
unsigned long bootTime = 0;
int reconnectAttempts = 0;

// Prevent MQTT self-echo loops
char lastPublishedRelayState[384] = "";

// ─────────────────────────────────────────────────────────
// Objects
// ─────────────────────────────────────────────────────────
#ifdef HAS_DHT22
DHT dht(PIN_DHT22, DHT_TYPE);
#endif

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ─────────────────────────────────────────────────────────
// Relay mode enum + state
// ─────────────────────────────────────────────────────────
enum RelayMode {
  MODE_MANUAL,
  MODE_AUTO,
  MODE_TIMER_CYCLIC,
  MODE_TIMER_SCHEDULED
};

struct RelayState {
  bool physicalOn = false;
  RelayMode mode = MODE_MANUAL;
  bool manualTarget = false;

  char autoSensorKey[16] = "co2_1";
  float setpointOn = 1000.0f;
  float setpointOff = 800.0f;

  unsigned long cyclicOnMs = 30UL * 60000UL;
  unsigned long cyclicOffMs = 10UL * 60000UL;
  unsigned long timerStart = 0;
  bool timerPhaseOn = false;

  int schedStartHour = 22;
  int schedStartMin = 0;
  unsigned long schedOnMs = 30UL * 60000UL;
} relay;

// ────────────────────s─────────────────────────────────────
// Sensor cache
// ─────────────────────────────────────────────────────────
struct SensorCache {
  float temp = NAN;
  float hum = NAN;
  float co2 = NAN;
  float nh3 = NAN;
} sensorCache;

// ─────────────────────────────────────────────────────────
// Sensor health tracking
// Tracks consecutive failures per sensor
// ─────────────────────────────────────────────────────────
struct SensorHealth {
  int failCount = 0;
  bool wasOffline = false; // prevents spamming the error log
  bool everSucceeded = false;
} dhtHealth, mq135Health, mq137Health;

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
// LED status blinker (non-blocking)
// ─────────────────────────────────────────────────────────
enum LedPattern { LED_FAST_BLINK, LED_SLOW_PULSE };

struct LedState {
  LedPattern pattern = LED_FAST_BLINK;
  bool pinState = false;
  unsigned long lastToggle = 0;
} led;

void updateLed() {
  unsigned long now = millis();
  switch (led.pattern) {

  case LED_FAST_BLINK:
    if (now - led.lastToggle >= 200) {
      led.lastToggle = now;
      led.pinState = !led.pinState;
      digitalWrite(PIN_STATUS_LED, led.pinState ? HIGH : LOW);
    }
    break;

  case LED_SLOW_PULSE:
    if (led.pinState) {
      if (now - led.lastToggle >= 2000) {
        led.lastToggle = now;
        led.pinState = false;
        digitalWrite(PIN_STATUS_LED, LOW);
      }
    } else {
      if (now - led.lastToggle >= 200) {
        led.lastToggle = now;
        led.pinState = true;
        digitalWrite(PIN_STATUS_LED, HIGH);
      }
    }
    break;
  }
}

void setLedPattern(LedPattern p) {
  if (led.pattern == p)
    return;
  led.pattern = p;
  led.pinState = false;
  led.lastToggle = millis();
  digitalWrite(PIN_STATUS_LED, LOW);
}

// ─────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  logger.info(CAT_SYSTEM, "Boot", logger.fmt("Device=%s", DEVICE_ID));
  logger.info(CAT_SYSTEM, "Topics",
              logger.fmt("data=%s | set=%s | relay=%s", TOPIC_DATA, TOPIC_SET,
                         TOPIC_RELAY));
  logger.info(CAT_SYSTEM, "Broker", logger.fmt("%s:%d", MQTT_HOST, MQTT_PORT));

  bootTime = millis();

  // Relay — OFF immediately
  pinMode(PIN_RELAY, OUTPUT);
  setRelayGPIO(false);
  logger.info(CAT_RELAY, "Init", "Relay forced OFF at boot");

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // DHT22
#ifdef HAS_DHT22
  dht.begin();
  logger.info(CAT_SENSOR, "DHT22 ready", logger.fmt("GPIO %d", PIN_DHT22));
#else
  logger.warn(CAT_SENSOR, "DHT22 disabled", "HAS_DHT22 not defined in config");
#endif

#ifdef HAS_MQ135
  logger.info(
      CAT_SENSOR, "MQ135 enabled",
      logger.fmt("GPIO %d warm-up %ds", PIN_MQ135, ADC_WARMUP_MS / 1000));
#else
  logger.warn(CAT_SENSOR, "MQ135 disabled", "HAS_MQ135 not defined in config");
#endif

#ifdef HAS_MQ137
  logger.info(
      CAT_SENSOR, "MQ137 enabled",
      logger.fmt("GPIO %d warm-up %ds", PIN_MQ137, ADC_WARMUP_MS / 1000));
#else
  logger.warn(CAT_SENSOR, "MQ137 disabled", "HAS_MQ137 not defined in config");
#endif

  relay.timerStart = millis();
  relay.timerPhaseOn = false;

  connectWiFi();
  syncNTP();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);
  mqtt.setBufferSize(512);

  connectMQTT();
}

// ─────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────
void loop() {
  // MQ warm-up
  if (!sensorsWarmedUp && (millis() - bootTime >= ADC_WARMUP_MS)) {
    sensorsWarmedUp = true;
    logger.info(CAT_SENSOR, "MQ warm-up complete", "Values now reliable");
  }

  // Wi-Fi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    setLedPattern(LED_FAST_BLINK); // ← lost connection
    logger.warn(CAT_WIFI, "Connection lost", "Attempting reconnect");
    connectWiFi();
  } else {
    setLedPattern(LED_SLOW_PULSE); // ← connected
  }

  updateLed();

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

  if (millis() - lastPublish >= SENSOR_INTERVAL_MS) {
    lastPublish = millis();
    publishSensors();
  }

  applyRelayMode();
}

// ─────────────────────────────────────────────────────────
// connectWiFi()
// ─────────────────────────────────────────────────────────
void connectWiFi() {
  logger.info(CAT_WIFI, "Connecting", logger.fmt("AP=%s", WIFI_AP_NAME));

  WiFiManager wm;
  wm.setConfigPortalTimeout(WIFI_TIMEOUT_S);

  bool ok = wm.autoConnect(
      WIFI_AP_NAME, strlen(WIFI_AP_PASSWORD) > 0 ? WIFI_AP_PASSWORD : nullptr);

  if (!ok) {
    logger.error(CAT_WIFI, "Portal timeout", "Restarting ESP32");
    ESP.restart();
  }

  reconnectAttempts = 0;
  logger.info(CAT_WIFI, "Connected",
              logger.fmt("IP=%s RSSI=%ddBm", WiFi.localIP().toString().c_str(),
                         WiFi.RSSI()));
}

void syncNTP() {

  // Egypt timezone with DST support
  configTzTime("EET-2EEST,M4.5.5/0,M10.5.4/24", "pool.ntp.org",
               "time.nist.gov");

  logger.info(CAT_NTP, "Syncing", "pool.ntp.org");

  struct tm tm {};
  unsigned long deadline = millis() + 10000UL;

  while (!getLocalTime(&tm) && millis() < deadline)
    delay(500);

  if (getLocalTime(&tm)) {
    logger.info(CAT_NTP, "Sync OK",
                logger.fmt("%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900,
                           tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
                           tm.tm_sec));
  } else {
    logger.warn(CAT_NTP, "Sync failed",
                "Scheduled timer mode may be inaccurate");
  }
}

// ─────────────────────────────────────────────────────────
// connectMQTT()
// ─────────────────────────────────────────────────────────
void connectMQTT() {
  if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
    logger.error(CAT_MQTT, "Max reconnect attempts reached", "Restarting");
    ESP.restart();
  }

  logger.info(CAT_MQTT, "Connecting",
              logger.fmt("%s:%d as %s (attempt %d/%d)", MQTT_HOST, MQTT_PORT,
                         MQTT_CLIENT_ID, reconnectAttempts + 1,
                         MAX_RECONNECT_ATTEMPTS));

  bool ok = (strlen(MQTT_USER) > 0)
                ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)
                : mqtt.connect(MQTT_CLIENT_ID);

  if (ok) {
    reconnectAttempts = 0;
    logger.info(CAT_MQTT, "Connected",
                logger.fmt("broker=%s:%d", MQTT_HOST, MQTT_PORT));

    // Subscribe to relay command topic
    bool subOk = mqtt.subscribe(TOPIC_SET, 1);
    if (subOk) {
      logger.info(CAT_MQTT, "Subscribed",
                  logger.fmt("topic=%s qos=1", TOPIC_SET));
    } else {
      logger.error(CAT_MQTT, "Subscribe failed",
                   logger.fmt("topic=%s rc=%d", TOPIC_SET, mqtt.state()));
    }

    // Sync relay state to platform on (re)connect
    publishRelayState();

  } else {
    reconnectAttempts++;
    // Map PubSubClient error codes to readable strings
    const char *reason = "unknown";
    switch (mqtt.state()) {
    case -4:
      reason = "connection timeout";
      break;
    case -3:
      reason = "connection lost";
      break;
    case -2:
      reason = "connect failed";
      break;
    case -1:
      reason = "disconnected";
      break;
    case 1:
      reason = "bad protocol";
      break;
    case 2:
      reason = "client ID rejected";
      break;
    case 3:
      reason = "server unavailable";
      break;
    case 4:
      reason = "bad credentials";
      break;
    case 5:
      reason = "not authorised";
      break;
    }
    logger.error(CAT_MQTT, "Connect failed",
                 logger.fmt("reason=%s rc=%d attempt=%d/%d", reason,
                            mqtt.state(), reconnectAttempts,
                            MAX_RECONNECT_ATTEMPTS));
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  logger.info(CAT_MQTT, "Message received",
              logger.fmt("topic=%s payload=%s", topic, msg));

  // Ignore echoed retained messages
  if (strcmp(msg, lastPublishedRelayState) == 0) {
    logger.debug(CAT_MQTT, "Ignoring self-echoed message", msg);
    return;
  }

  if (strcmp(topic, TOPIC_SET) != 0) {
    logger.warn(CAT_MQTT, "Unexpected topic", logger.fmt("topic=%s", topic));
    return;
  }

  // ─────────────────────────────────────────────────────
  // Legacy plain string commands
  // ─────────────────────────────────────────────────────
  if (strcasecmp(msg, "ON") == 0 || strcmp(msg, "1") == 0) {
    relay.mode = MODE_MANUAL;
    relay.manualTarget = true;

    logger.info(CAT_RELAY, "Legacy command", "manual ON");

    applyRelayMode();
    publishRelayState();

    return;
  }

  if (strcasecmp(msg, "OFF") == 0 || strcmp(msg, "0") == 0) {
    relay.mode = MODE_MANUAL;
    relay.manualTarget = false;

    logger.info(CAT_RELAY, "Legacy command", "manual OFF");

    applyRelayMode();
    publishRelayState();

    return;
  }

  // ─────────────────────────────────────────────────────
  // Parse JSON
  // ─────────────────────────────────────────────────────
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err != DeserializationError::Ok) {
    logger.error(CAT_MQTT, "JSON parse failed",
                 logger.fmt("error=%s payload=%s", err.c_str(), msg));
    return;
  }

  // ─────────────────────────────────────────────────────
  // Legacy JSON
  // ─────────────────────────────────────────────────────
  if (doc["relay"].is<bool>() && !doc.containsKey("mode")) {
    relay.mode = MODE_MANUAL;
    relay.manualTarget = doc["relay"].as<bool>();
    logger.info(CAT_RELAY, "Legacy JSON", relay.manualTarget ? "ON" : "OFF");

    applyRelayMode();
    publishRelayState();
    return;
  }

  const char *mode = doc["mode"] | "";

  JsonObject cfg = doc["mode_config"].as<JsonObject>();

  // ─────────────────────────────────────────────────────
  // MANUAL
  // ─────────────────────────────────────────────────────
  if (strcmp(mode, "manual") == 0) {

    relay.mode = MODE_MANUAL;

    relay.manualTarget = (cfg["state"].as<int>() != 0);

    logger.info(CAT_RELAY, "Mode set: MANUAL",
                logger.fmt("target=%s", relay.manualTarget ? "ON" : "OFF"));
  }

  // ─────────────────────────────────────────────────────
  // AUTO
  // ─────────────────────────────────────────────────────
  else if (strcmp(mode, "auto") == 0) {

    relay.mode = MODE_AUTO;

    strncpy(relay.autoSensorKey, cfg["sensor_key"] | "co2_1",
            sizeof(relay.autoSensorKey) - 1);

    relay.autoSensorKey[sizeof(relay.autoSensorKey) - 1] = '\0';

    relay.setpointOn = cfg["setpoint_on"] | 1000.0f;

    relay.setpointOff = cfg["setpoint_off"] | 800.0f;

    logger.info(CAT_RELAY, "Mode set: AUTO",
                logger.fmt("sensor=%s on>=%.1f off<=%.1f", relay.autoSensorKey,
                           relay.setpointOn, relay.setpointOff));
  }

  // ─────────────────────────────────────────────────────
  // TIMER
  // ─────────────────────────────────────────────────────
  else if (strcmp(mode, "timer") == 0) {

    const char *timerType = cfg["type"] | "cyclic";

    // CYCLIC
    if (strcmp(timerType, "cyclic") == 0) {

      relay.mode = MODE_TIMER_CYCLIC;

      JsonObject cyc = cfg["cyclic"].as<JsonObject>();

      relay.cyclicOnMs = (cyc["on_duration_min"] | 30) * 60000UL;

      relay.cyclicOffMs = (cyc["off_duration_min"] | 10) * 60000UL;

      // Start immediately ON
      relay.timerPhaseOn = true;

      relay.timerStart = millis();

      logger.info(CAT_RELAY, "Mode set: TIMER CYCLIC",
                  logger.fmt("on=%lum off=%lum", relay.cyclicOnMs / 60000UL,
                             relay.cyclicOffMs / 60000UL));
    }

    // SCHEDULED
    else if (strcmp(timerType, "scheduled") == 0) {

      relay.mode = MODE_TIMER_SCHEDULED;

      JsonObject sched = cfg["scheduled"].as<JsonObject>();

      const char *start = sched["start_time"] | "00:00";

      sscanf(start, "%d:%d", &relay.schedStartHour, &relay.schedStartMin);

      relay.schedOnMs = (sched["on_duration_min"] | 30) * 60000UL;

      logger.info(CAT_RELAY, "Mode set: TIMER SCHEDULED",
                  logger.fmt("start=%02d:%02d on=%lum", relay.schedStartHour,
                             relay.schedStartMin, relay.schedOnMs / 60000UL));
    }

    else {

      logger.error(CAT_RELAY, "Unknown timer type", timerType);

      return;
    }
  }

  else {

    logger.error(CAT_RELAY, "Unknown mode", mode);

    return;
  }

  // APPLY IMMEDIATELY
  applyRelayMode();

  // PUBLISH ONCE ONLY
  publishRelayState();
}

void applyRelayMode() {

  bool desired = relay.physicalOn;

  switch (relay.mode) {

  // ── MANUAL ────────────────────────────────────────────
  case MODE_MANUAL:
    desired = relay.manualTarget;
    break;

  // ── AUTO ──────────────────────────────────────────────
  case MODE_AUTO: {

    float val = lookupSensorValue(relay.autoSensorKey);

    if (isnan(val)) {
      logger.warn(CAT_RELAY, "Auto mode: sensor unavailable",
                  logger.fmt("key=%s — relay holds current state",
                             relay.autoSensorKey));
      return;
    }

    if (!relay.physicalOn && val >= relay.setpointOn) {

      desired = true;

      logger.info(CAT_RELAY, "Auto threshold crossed → ON",
                  logger.fmt("key=%s val=%.1f setpoint_on=%.1f",
                             relay.autoSensorKey, val, relay.setpointOn));
      publishRelayState();
    } else if (relay.physicalOn && val <= relay.setpointOff) {

      desired = false;

      logger.info(CAT_RELAY, "Auto threshold crossed → OFF",
                  logger.fmt("key=%s val=%.1f setpoint_off=%.1f",
                             relay.autoSensorKey, val, relay.setpointOff));
      publishRelayState();
    }

    break;
  }

  // ── CYCLIC TIMER ──────────────────────────────────────
  case MODE_TIMER_CYCLIC: {

    unsigned long elapsed = millis() - relay.timerStart;

    unsigned long phaseDuration =
        relay.timerPhaseOn ? relay.cyclicOnMs : relay.cyclicOffMs;

    if (elapsed >= phaseDuration) {

      relay.timerPhaseOn = !relay.timerPhaseOn;
      relay.timerStart = millis();

      logger.info(
          CAT_RELAY, "Cyclic timer phase flipped",
          logger.fmt("new phase=%s", relay.timerPhaseOn ? "ON" : "OFF"));
      publishRelayState();
    }

    desired = relay.timerPhaseOn;

    break;
  }

  // ── SCHEDULED TIMER ───────────────────────────────────
  case MODE_TIMER_SCHEDULED: {

    struct tm now {};

    if (!getLocalTime(&now)) {

      logger.warn(CAT_RELAY, "Scheduled timer: NTP unavailable",
                  "Relay holds current state");

      return;
    }

    int nowMin = now.tm_hour * 60 + now.tm_min;

    int startMin = relay.schedStartHour * 60 + relay.schedStartMin;

    int diff = nowMin - startMin;

    if (diff < 0)
      diff += 1440;

    desired = (diff < (int)(relay.schedOnMs / 60000UL));

    break;
  }
  }

  // ── Apply physical state ──────────────────────────────
  if (desired != relay.physicalOn) {

    logger.info(CAT_RELAY, "State changed",
                logger.fmt("%s → %s", relay.physicalOn ? "ON" : "OFF",
                           desired ? "ON" : "OFF"));

    setRelayGPIO(desired);
    publishRelayState();
  }
}
// ─────────────────────────────────────────────────────────
// setRelayGPIO()
// Active-LOW relay module: LOW = ON, HIGH = OFF
// ─────────────────────────────────────────────────────────
void setRelayGPIO(bool on) {
  relay.physicalOn = on;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  logger.debug(CAT_RELAY, "GPIO set",
               logger.fmt("pin=%d level=%s physical=%s", PIN_RELAY,
                          on ? "LOW" : "HIGH", on ? "ON" : "OFF"));
}

// ─────────────────────────────────────────────────────────
// publishRelayState()
// ─────────────────────────────────────────────────────────
// void publishRelayState() {
//   JsonDocument doc;
//   doc["relay"] = RELAY_KEY;
//   doc["state"] = relay.physicalOn ? 1 : 0;

//   JsonObject cfg = doc["mode_config"].to<JsonObject>();

//   switch (relay.mode) {
//   case MODE_MANUAL:
//     doc["mode"] = "manual";
//     cfg["type"] = "manual";
//     cfg["state"] = relay.manualTarget ? 1 : 0;
//     break;
//   case MODE_AUTO:
//     doc["mode"] = "auto";
//     cfg["sensor_key"] = relay.autoSensorKey;
//     cfg["setpoint_on"] = relay.setpointOn;
//     cfg["setpoint_off"] = relay.setpointOff;
//     break;
//   case MODE_TIMER_CYCLIC: {
//     doc["mode"] = "timer";
//     cfg["type"] = "cyclic";
//     JsonObject cyc = cfg["cyclic"].to<JsonObject>();
//     cyc["on_duration_min"] = relay.cyclicOnMs / 60000UL;
//     cyc["off_duration_min"] = relay.cyclicOffMs / 60000UL;
//     break;
//   }
//   case MODE_TIMER_SCHEDULED: {
//     doc["mode"] = "timer";
//     cfg["type"] = "scheduled";
//     JsonObject sched = cfg["scheduled"].to<JsonObject>();
//     char buf[6];
//     snprintf(buf, sizeof(buf), "%02d:%02d", relay.schedStartHour,
//              relay.schedStartMin);
//     sched["start_time"] = buf;
//     sched["on_duration_min"] = relay.schedOnMs / 60000UL;
//     break;
//   }
//   }

//   char buffer[384];
//   size_t n = serializeJson(doc, buffer);
//   strncpy(lastPublishedRelayState, buffer,
//           sizeof(lastPublishedRelayState) - 1); // NEW

//   lastPublishedRelayState[sizeof(lastPublishedRelayState) - 1] = '\0'; // NEW

//   if (mqtt.publish(TOPIC_RELAY, (const uint8_t *)buffer, n, true)) {
//     logger.info(CAT_MQTT, "Relay state published",
//                 logger.fmt("topic=%s payload=%s", TOPIC_RELAY, buffer));
//   } else {
//     logger.error(CAT_MQTT, "Relay publish failed",
//                  logger.fmt("topic=%s rc=%d", TOPIC_RELAY, mqtt.state()));
//   }
// }

void publishRelayState() {
  JsonDocument doc;

  // Root level fields
  doc["relay"] = RELAY_KEY;
  doc["state"] = relay.physicalOn ? 1 : 0;

  // Set the "mode" string and build "mode_config"
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
    // Nested cyclic object
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

  // Serialize to buffer
  char buffer[384];
  size_t n = serializeJson(doc, buffer);

  // Copy to lastPublishedRelayState for tracking
  strncpy(lastPublishedRelayState, buffer, sizeof(lastPublishedRelayState) - 1);
  lastPublishedRelayState[sizeof(lastPublishedRelayState) - 1] = '\0';

  // Publish via MQTT
  if (mqtt.publish(TOPIC_RELAY, (const uint8_t *)buffer, n, true)) {
    logger.info(CAT_MQTT, "Relay state published",
                logger.fmt("topic=%s payload=%s", TOPIC_RELAY, buffer));
  } else {
    logger.error(CAT_MQTT, "Relay publish failed",
                 logger.fmt("topic=%s rc=%d", TOPIC_RELAY, mqtt.state()));
  }
}

// ─────────────────────────────────────────────────────────
// readMQAverage()
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

void publishSensors() {
  bool anySensor = false;

  // ── DHT22 ─────────────────────────────────────────────
#ifdef HAS_DHT22
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (!isnan(temp) && !isnan(hum)) {
    // Success
    if (dhtHealth.wasOffline) {
      logger.info(CAT_SENSOR, "DHT22 recovered",
                  logger.fmt("T=%.1f H=%.1f (was offline for %d reads)", temp,
                             hum, dhtHealth.failCount));
      dhtHealth.wasOffline = false;
    } else if (!dhtHealth.everSucceeded) {
      logger.info(CAT_SENSOR, "DHT22 first read OK",
                  logger.fmt("T=%.1f H=%.1f", temp, hum));
    } else {
      logger.debug(CAT_SENSOR, "DHT22 OK",
                   logger.fmt("T=%.1f H=%.1f", temp, hum));
    }
    dhtHealth.failCount = 0;
    dhtHealth.everSucceeded = true;
    sensorCache.temp = temp;
    sensorCache.hum = hum;

  } else {
    // Failure
    dhtHealth.failCount++;
    sensorCache.temp = NAN;
    sensorCache.hum = NAN;

    if (dhtHealth.failCount < DHT_MAX_FAILS) {
      logger.debug(CAT_SENSOR, "DHT22 read failed",
                   logger.fmt("fail %d/%d — skipping this tick",
                              dhtHealth.failCount, DHT_MAX_FAILS));
    } else if (!dhtHealth.wasOffline) {
      // Just crossed the threshold — log once
      dhtHealth.wasOffline = true;
      logger.error(
          CAT_SENSOR, "DHT22 offline",
          logger.fmt("%d consecutive failures — check wiring on GPIO %d",
                     dhtHealth.failCount, PIN_DHT22));
    }
    // Don't return — other sensors may still be fine
  }
#endif

  // ── MQ135 ─────────────────────────────────────────────
#ifdef HAS_MQ135
  float co2 = readMQAverage(PIN_MQ135);

  // MQ sensors on ADC return 0 when completely disconnected
  // (pin floats to GND through internal pull-down).
  // A reading of 0 is physically impossible for a powered sensor.
  if (co2 < 30.0f) {
    mq135Health.failCount++;
    sensorCache.co2 = NAN;

    if (!mq135Health.wasOffline) {
      mq135Health.wasOffline = true;
      logger.error(CAT_SENSOR, "MQ135 offline or disconnected",
                   logger.fmt("ADC reading=%.0f on GPIO %d", co2, PIN_MQ135));
    }
  } else {
    if (mq135Health.wasOffline) {
      logger.info(CAT_SENSOR, "MQ135 recovered", logger.fmt("ADC=%.0f", co2));
      mq135Health.wasOffline = false;
    } else if (!mq135Health.everSucceeded) {
      logger.info(
          CAT_SENSOR, "MQ135 first read OK",
          logger.fmt("ADC=%.0f%s", co2, sensorsWarmedUp ? "" : " [warming]"));
    } else {
      logger.debug(
          CAT_SENSOR, "MQ135 OK",
          logger.fmt("ADC=%.0f%s", co2, sensorsWarmedUp ? "" : " [warming]"));
    }
    mq135Health.failCount = 0;
    mq135Health.everSucceeded = true;
    sensorCache.co2 = co2;
  }
#else
  sensorCache.co2 = NAN;
#endif

  // ── MQ137 ─────────────────────────────────────────────
#ifdef HAS_MQ137
  float nh3 = readMQAverage(PIN_MQ137);

  if (nh3 < 30.0f) {
    mq137Health.failCount++;
    sensorCache.nh3 = NAN;

    if (!mq137Health.wasOffline) {
      mq137Health.wasOffline = true;
      logger.error(CAT_SENSOR, "MQ137 offline or disconnected",
                   logger.fmt("ADC reading=%.0f on GPIO %d", nh3, PIN_MQ137));
    }
  } else {
    if (mq137Health.wasOffline) {
      logger.info(CAT_SENSOR, "MQ137 recovered", logger.fmt("ADC=%.0f", nh3));
      mq137Health.wasOffline = false;
    } else if (!mq137Health.everSucceeded) {
      logger.info(
          CAT_SENSOR, "MQ137 first read OK",
          logger.fmt("ADC=%.0f%s", nh3, sensorsWarmedUp ? "" : " [warming]"));
    } else {
      logger.debug(
          CAT_SENSOR, "MQ137 OK",
          logger.fmt("ADC=%.0f%s", nh3, sensorsWarmedUp ? "" : " [warming]"));
    }
    mq137Health.failCount = 0;
    mq137Health.everSucceeded = true;
    sensorCache.nh3 = nh3;
  }
#else
  sensorCache.nh3 = NAN;
#endif

  // ── Timestamp ─────────────────────────────────────────
  char timestamp[32];
  struct tm now {};
  if (getLocalTime(&now)) {
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d.000Z",
             now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour,
             now.tm_min, now.tm_sec);
  } else {
    unsigned long s = millis() / 1000;
    snprintf(timestamp, sizeof(timestamp), "1970-01-01T%02lu:%02lu:%02lu.000Z",
             (s / 3600) % 24, (s / 60) % 60, s % 60);
  }

  // ── Build JSON — only valid readings enter the array ──
  JsonDocument doc;
  doc["timestamp"] = timestamp;
  JsonArray arr = doc["sensors"].to<JsonArray>();

#ifdef HAS_DHT22
  {
    JsonObject s = arr.add<JsonObject>();

    bool connected = !isnan(sensorCache.temp);

    s["key"] = "temp_1";
    s["connected"] = connected;

    if (connected) {

      s["value"] = round(sensorCache.temp * 100.0f) / 100.0f;

      anySensor = true;

    } else {

      s["value"] = nullptr;
    }
  }

  // hum_1
  {
    JsonObject s = arr.add<JsonObject>();

    bool connected = !isnan(sensorCache.hum);

    s["key"] = "hum_1";
    s["connected"] = connected;

    if (connected) {

      s["value"] = round(sensorCache.hum * 100.0f) / 100.0f;

      anySensor = true;

    } else {

      s["value"] = nullptr;
    }
  }
#endif

#ifdef HAS_MQ135
  {
    JsonObject s = arr.add<JsonObject>();

    bool connected = !isnan(sensorCache.co2);

    s["key"] = "co2_1";
    s["connected"] = connected;

    if (connected) {

      s["value"] = round(sensorCache.co2 * 100.0f) / 100.0f;

      anySensor = true;

    } else {

      s["value"] = nullptr;
    }
  }
#endif

#ifdef HAS_MQ137
  {
    JsonObject s = arr.add<JsonObject>();

    bool connected = !isnan(sensorCache.nh3);

    s["key"] = "nh3_1";
    s["connected"] = connected;

    if (connected) {

      s["value"] = round(sensorCache.nh3 * 100.0f) / 100.0f;

      anySensor = true;

    } else {

      s["value"] = nullptr;
    }
  }
#endif

  // ── Publish ───────────────────────────────────────────
  if (!anySensor) {
    logger.error(CAT_SENSOR, "All sensors failed",
                 "No data published this tick");
    // return;
  }

  char buffer[384];
  size_t n = serializeJson(doc, buffer);

  if (mqtt.publish(TOPIC_DATA, (const uint8_t *)buffer, n)) {
    logger.info(CAT_MQTT, "Sensors published",
                logger.fmt("topic=%s bytes=%d sensors=%d", TOPIC_DATA, (int)n,
                           (int)arr.size()));
  } else {
    logger.error(CAT_MQTT, "Sensor publish failed",
                 logger.fmt("topic=%s rc=%d bufsize=%d", TOPIC_DATA,
                            mqtt.state(), (int)n));
  }
}