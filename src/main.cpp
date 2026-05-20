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

// ─────────────────────────────────────────────────────────
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

float mq135_r0 = MQ135_R0_DEFAULT;
float mq137_r0 = MQ137_R0_DEFAULT;
bool mqCalibrated = false;

// ─────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────
void connectWiFi();
void connectMQTT();
void syncNTP();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void publishSensors();
void publishRelayState();
void publishRelayDefaultState();
void applyRelayMode();
void setRelayGPIO(bool on);
float readMQAverage(int pin);
float lookupSensorValue(const char *key);
void factoryResetAndRestart();

// ═════════════════════════════════════════════════════════════════════════════
//                              MQ MATH HELPERS
// ═════════════════════════════════════════════════════════════════════════════

float adcToVout(float adc) { return (adc / MQ_ADC_MAX) * MQ_VREF; }

float adcToRs(float adc) {
  float v = adcToVout(adc);
  if (v <= 0.05f)
    return -1.0f;
  return MQ_RL * (MQ_VCC - v) / v;
}

float rsToPpm(float rs, float r0, float a, float b) {
  if (rs <= 0.0f || r0 <= 0.0f)
    return NAN;
  float ratio = rs / r0;
  if (ratio <= 0.0f)
    return NAN;
  float ppm = a * powf(ratio, b);
  if (isnan(ppm) || ppm < 0.0f || ppm > 100000.0f)
    return NAN;
  return ppm;
}

float computeR0FromBaseline(float rs, float baselinePpm, float a, float b) {
  if (rs <= 0.0f || baselinePpm <= 0.0f)
    return -1.0f;
  return rs * powf(a / baselinePpm, 1.0f / b);
}

float adcToPpm(float adc, float r0, float a, float b) {
  return rsToPpm(adcToRs(adc), r0, a, b);
}

// ═════════════════════════════════════════════════════════════════════════════
//                                 LED STATUS
// ═════════════════════════════════════════════════════════════════════════════

void ledTaskFn(void *pv) {
  for (;;) {
    wifi_mode_t mode = WiFi.getMode();
    bool apActive = (mode & WIFI_MODE_AP) != 0;
    bool staLinked = (WiFi.status() == WL_CONNECTED);

    if (apActive) {
      digitalWrite(PIN_STATUS_LED, HIGH);
      vTaskDelay(pdMS_TO_TICKS(120));
      digitalWrite(PIN_STATUS_LED, LOW);
      vTaskDelay(pdMS_TO_TICKS(120));
    } else if (!staLinked) {
      digitalWrite(PIN_STATUS_LED, HIGH);
      vTaskDelay(pdMS_TO_TICKS(600));
      digitalWrite(PIN_STATUS_LED, LOW);
      vTaskDelay(pdMS_TO_TICKS(600));
    } else {
      digitalWrite(PIN_STATUS_LED, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//                          SERIAL COMMAND TASK
// ═════════════════════════════════════════════════════════════════════════════

void factoryResetAndRestart() {
  Serial.println();
  Serial.println("──────────────────────────────────────────────");
  Serial.println("  FACTORY RESET — Erasing Wi-Fi credentials");
  Serial.println("──────────────────────────────────────────────");
  WiFiManager wm;
  wm.resetSettings();
  Serial.println("  Done. Restarting…");
  delay(500);
  ESP.restart();
}

void serialTaskFn(void *pv) {
  String buf;
  buf.reserve(32);
  for (;;) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        buf.trim();
        if (buf.length() > 0) {
          if (buf.equalsIgnoreCase("reset") ||
              buf.equalsIgnoreCase("factory-reset") ||
              buf.equalsIgnoreCase("wipe")) {
            factoryResetAndRestart();
          } else {
            Serial.printf(
                "Unknown command: '%s'. Type 'reset' to factory reset.\n",
                buf.c_str());
          }
        }
        buf = "";
      } else if (buf.length() < 30) {
        buf += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//                       BRANDED WiFiManager CAPTIVE PORTAL
// ═════════════════════════════════════════════════════════════════════════════

static const char PORTAL_HEAD[] PROGMEM = R"HTML(
<title>Syntax IoT · Device Setup</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#060E09;--card:#0c1a14;--p:#27A050;--pb:#34c764;--a:#F59E0B;--t:#e9f5ee;--m:#94a3a0}
*{box-sizing:border-box}
html,body{background:var(--bg)!important;color:var(--t)!important;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Tahoma,sans-serif!important;margin:0}
body{padding:14px 12px 28px}
h1,h2,h3,h4{color:var(--t)!important}
a,a:link{color:var(--pb)!important}
.c,.msg{background:var(--card)!important;border:1px solid rgba(39,160,80,.18)!important;border-radius:14px!important;padding:16px!important;margin:10px auto!important;max-width:480px!important}
button,input[type=submit],input[type=button]{background:var(--p)!important;color:#fff!important;border:0!important;border-radius:10px!important;padding:13px!important;font-size:16px!important;font-weight:600!important;cursor:pointer!important;width:100%!important}
button:hover,input[type=submit]:hover{background:var(--pb)!important}
input[type=text],input[type=password],input[type=number],select{background:rgba(255,255,255,.05)!important;border:1px solid rgba(39,160,80,.22)!important;border-radius:9px!important;color:var(--t)!important;padding:12px!important;font-size:16px!important;width:100%!important}
input:focus{border-color:var(--pb)!important;outline:0!important}
.sx-hdr{max-width:480px;margin:6px auto 14px;text-align:center;padding:22px 14px;background:linear-gradient(180deg,#0e2018,var(--bg));border:1px solid rgba(39,160,80,.18);border-radius:18px}
.sx-hdr svg{width:62px;height:62px;filter:drop-shadow(0 0 14px rgba(39,160,80,.45))}
.sx-hdr h1{margin:10px 0 2px;font-size:26px;letter-spacing:.5px;color:var(--pb)!important;font-weight:800}
.sx-hdr h1 .iot{color:var(--t)!important;font-weight:300;margin-left:6px}
.sx-hdr p{margin:6px 0 0;color:var(--m)!important;font-size:13px;letter-spacing:.4px}
.sx-leg{max-width:480px;margin:0 auto 14px;background:var(--card);border:1px solid rgba(39,160,80,.18);border-radius:14px;padding:14px 16px}
.sx-leg h3{margin:0 0 10px;font-size:11px;color:var(--a)!important;letter-spacing:1.5px;text-transform:uppercase}
.sx-row{display:flex;align-items:center;gap:11px;padding:5px 0;font-size:13px;color:var(--m)}
.sx-dot{width:13px;height:13px;border-radius:50%;background:var(--pb);box-shadow:0 0 9px var(--pb);flex-shrink:0}
.sx-dot.f{animation:fb .26s infinite}
.sx-dot.s{animation:sb 1.3s infinite}
@keyframes fb{50%{opacity:.12}}
@keyframes sb{50%{opacity:.12}}
</style>
<script>
document.addEventListener('DOMContentLoaded',function(){
 var h=document.createElement('div');h.className='sx-hdr';
 h.innerHTML='<svg viewBox="0 0 64 64" xmlns="http://www.w3.org/2000/svg">'+
 '<path d="M48 14 C48 14 16 14 16 26 C16 38 48 28 48 40 C48 52 16 52 16 52" '+
 'stroke="#27A050" stroke-width="10" fill="none" stroke-linecap="round"/>'+
 '<path d="M48 18 C48 18 22 18 22 28 C22 38 54 30 54 40 C54 50 22 50 22 50" '+
 'stroke="#34c764" stroke-width="6" fill="none" stroke-linecap="round" opacity=".5"/>'+
 '</svg>'+
 '<h1>Syntax<span class="iot">IoT</span></h1>'+
 '<p>Smart IoT Solutions · Device Setup</p>';
 var l=document.createElement('div');l.className='sx-leg';
 l.innerHTML='<h3>LED Indicator Guide</h3>'+
 '<div class="sx-row"><span class="sx-dot f"></span>Fast blink &nbsp;—&nbsp; searching / setup mode</div>'+
 '<div class="sx-row"><span class="sx-dot s"></span>Slow blink &nbsp;—&nbsp; connecting to Wi-Fi</div>'+
 '<div class="sx-row"><span class="sx-dot"></span>Solid ON &nbsp;&nbsp;&nbsp;—&nbsp; connected &amp; running</div>';
 document.body.insertBefore(l,document.body.firstChild);
 document.body.insertBefore(h,document.body.firstChild);
});
</script>
)HTML";

// ─────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("═══════════════════════════════════════════════════════");
  Serial.println("  Syntax IoT — Poultry Farm Monitoring");
  Serial.println("  Type 'reset' at any time to wipe Wi-Fi credentials");
  Serial.println("═══════════════════════════════════════════════════════");

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

  xTaskCreatePinnedToCore(ledTaskFn, "led", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(serialTaskFn, "serial", 4096, NULL, 1, NULL, 1);

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

  if (WiFi.status() != WL_CONNECTED) {
    logger.warn(CAT_WIFI, "Connection lost", "Attempting reconnect");
    connectWiFi();
  }

  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastReconnect >= RECONNECT_INTERVAL_MS) {
      lastReconnect = now;
      connectMQTT();
    }
  } else {
    mqtt.loop();

    if (millis() - lastPublish >= SENSOR_INTERVAL_MS) {
      lastPublish = millis();
      publishSensors();
    }
  }

  applyRelayMode();
}

// ═════════════════════════════════════════════════════════════════════════════
//                       CHANNEL-AGNOSTIC WIFI DISCOVERY
// ═════════════════════════════════════════════════════════════════════════════

void configureWifiCountry() {
  wifi_country_t country = {};
  country.cc[0] = '0';
  country.cc[1] = '1';
  country.cc[2] = 0;
  country.schan = 1;
  country.nchan = 13;
  country.policy = WIFI_COUNTRY_POLICY_MANUAL;
  esp_wifi_set_country(&country);
  logger.info(CAT_WIFI, "Country set", "01 (channels 1-13 enabled)");
}

bool readSavedCredentials(String &ssid, String &pass) {
  wifi_config_t conf;
  if (esp_wifi_get_config(WIFI_IF_STA, &conf) != ESP_OK)
    return false;
  ssid = String((char *)conf.sta.ssid);
  pass = String((char *)conf.sta.password);
  return ssid.length() > 0;
}

bool scanAndConnect(const String &ssid, const String &pass,
                    uint32_t timeoutMs) {
  logger.info(CAT_WIFI, "Active scan",
              logger.fmt("hunting SSID=%s across ch 1-13", ssid.c_str()));

  int n = WiFi.scanNetworks(false, false, false, 300);
  if (n <= 0) {
    logger.warn(CAT_WIFI, "Scan empty", logger.fmt("n=%d", n));
    WiFi.scanDelete();
    return false;
  }

  int bestIdx = -1, bestRssi = -999;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == ssid && WiFi.RSSI(i) > bestRssi) {
      bestRssi = WiFi.RSSI(i);
      bestIdx = i;
    }
  }

  if (bestIdx < 0) {
    logger.warn(
        CAT_WIFI, "Target not in scan",
        logger.fmt("%d nets visible, '%s' not among them", n, ssid.c_str()));
    WiFi.scanDelete();
    return false;
  }

  int channel = WiFi.channel(bestIdx);
  uint8_t bssid[6];
  memcpy(bssid, WiFi.BSSID(bestIdx), 6);

  logger.info(CAT_WIFI, "Target located",
              logger.fmt("ch=%d rssi=%ddBm bssid=%02X:%02X:%02X:%02X:%02X:%02X",
                         channel, bestRssi, bssid[0], bssid[1], bssid[2],
                         bssid[3], bssid[4], bssid[5]));

  WiFi.scanDelete();
  WiFi.begin(ssid.c_str(), pass.c_str(), channel, bssid);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    logger.info(CAT_WIFI, "Connected via scan",
                logger.fmt("IP=%s ch=%d rssi=%ddBm",
                           WiFi.localIP().toString().c_str(), channel,
                           WiFi.RSSI()));
    return true;
  }

  logger.warn(CAT_WIFI, "Scan-connect timed out",
              logger.fmt("ch=%d after %lums", channel, timeoutMs));
  WiFi.disconnect(true, false);
  return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//                                connectWiFi()
// ═════════════════════════════════════════════════════════════════════════════

void connectWiFi() {
  logger.info(CAT_WIFI, "Connecting",
              logger.fmt("portal fallback=%s", WIFI_AP_NAME));

  WiFi.mode(WIFI_STA);
  configureWifiCountry();

  // ── Path A: saved credentials via scan ────────────────────────────────────
  String savedSSID, savedPass;
  if (readSavedCredentials(savedSSID, savedPass)) {
    logger.info(CAT_WIFI, "Saved credentials present",
                logger.fmt("SSID=%s", savedSSID.c_str()));

    for (int attempt = 1; attempt <= 3; attempt++) {
      logger.info(CAT_WIFI, "Scan-connect attempt",
                  logger.fmt("%d of 3", attempt));
      if (scanAndConnect(savedSSID, savedPass, 15000)) {
        reconnectAttempts = 0;
        return;
      }
      delay(1500);
    }
    logger.warn(CAT_WIFI, "Saved credentials failed",
                "Trying hardcoded fallback");
  }

  // ── Path B: hardcoded fallback credentials ────────────────────────────────
  // غيّر FALLBACK_SSID و FALLBACK_PASSWORD في أعلى الكود
  // ─────────────────────────────────────────────────────────────────────────
  if (strlen(FALLBACK_SSID) > 0) {
    logger.info(CAT_WIFI, "Trying hardcoded credentials",
                logger.fmt("SSID=%s", FALLBACK_SSID));

    String fbSSID = String(FALLBACK_SSID);
    String fbPass = String(FALLBACK_PASSWORD);

    for (int attempt = 1; attempt <= 3; attempt++) {
      logger.info(CAT_WIFI, "Hardcoded attempt",
                  logger.fmt("%d of 3", attempt));
      if (scanAndConnect(fbSSID, fbPass, 20000)) {
        reconnectAttempts = 0;
        // حفظ الـ credentials في NVS عشان المرة الجاية تشتغل من Path A
        WiFiManager wm;
        wm.setConfigPortalTimeout(1);
        logger.info(CAT_WIFI, "Credentials saved to NVS", "");
        return;
      }
      delay(2000);
    }
    logger.warn(CAT_WIFI, "Hardcoded credentials failed",
                "Falling back to portal");
  }

  // ── Path C: WiFiManager captive portal ────────────────────────────────────
  WiFiManager wm;
  wm.setConfigPortalTimeout(WIFI_TIMEOUT_S);
  wm.setConnectTimeout(15);
  wm.setConnectRetries(2);
  wm.setTitle("Syntax IoT");
  wm.setCustomHeadElement(PORTAL_HEAD);

  bool ok = wm.autoConnect(
      WIFI_AP_NAME, strlen(WIFI_AP_PASSWORD) > 0 ? WIFI_AP_PASSWORD : nullptr);

  if (!ok) {
    logger.error(CAT_WIFI, "Portal timeout", "Restarting ESP32");
    ESP.restart();
  }

  reconnectAttempts = 0;
  logger.info(CAT_WIFI, "Connected via portal",
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

    bool subOk = mqtt.subscribe(TOPIC_SET, 1);
    if (subOk) {
      logger.info(CAT_MQTT, "Subscribed",
                  logger.fmt("topic=%s qos=1", TOPIC_SET));
    } else {
      logger.error(CAT_MQTT, "Subscribe failed",
                   logger.fmt("topic=%s rc=%d", TOPIC_SET, mqtt.state()));
    }

    publishRelayState();

  } else {
    reconnectAttempts++;
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
      relay.mode = MODE_MANUAL;
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

void publishRelayDefaultState() {
  JsonDocument doc;
  relay.mode = MODE_MANUAL;

  // Root level fields
  doc["relay"] = RELAY_KEY;
  doc["state"] = relay.physicalOn ? 1 : 0;
  doc["mode"] = "manual";

  // Set the "mode" string and build "mode_config"
  JsonObject cfg = doc["mode_config"].to<JsonObject>();

  cfg["type"] = "manual";
  cfg["state"] = relay.manualTarget ? 1 : 0;

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

#ifdef HAS_DHT22
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (!isnan(temp) && !isnan(hum)) {
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
    dhtHealth.failCount++;
    sensorCache.temp = NAN;
    sensorCache.hum = NAN;

    if (dhtHealth.failCount < DHT_MAX_FAILS) {
      logger.debug(CAT_SENSOR, "DHT22 read failed",
                   logger.fmt("fail %d/%d — skipping this tick",
                              dhtHealth.failCount, DHT_MAX_FAILS));
    } else if (!dhtHealth.wasOffline) {
      dhtHealth.wasOffline = true;
      logger.error(
          CAT_SENSOR, "DHT22 offline",
          logger.fmt("%d consecutive failures — check wiring on GPIO %d",
                     dhtHealth.failCount, PIN_DHT22));
    }
  }
#endif

#ifdef HAS_MQ135
  float co2Adc = readMQAverage(PIN_MQ135);

  if (co2Adc < 30.0f) {
    mq135Health.failCount++;
    sensorCache.co2 = NAN;
    if (!mq135Health.wasOffline) {
      mq135Health.wasOffline = true;
      logger.error(
          CAT_SENSOR, "MQ135 offline or disconnected",
          logger.fmt("ADC reading=%.0f on GPIO %d", co2Adc, PIN_MQ135));
    }
  } else {
    if (sensorsWarmedUp && !mqCalibrated && MQ_AUTO_CALIBRATE) {
      float rs = adcToRs(co2Adc);
      if (rs > 0.0f) {
        float newR0 =
            computeR0FromBaseline(rs, MQ135_BASELINE_PPM, MQ135_A, MQ135_B);
        if (newR0 > 0.0f)
          mq135_r0 = newR0;
        logger.info(CAT_SENSOR, "MQ135 R0 auto-calibrated",
                    logger.fmt("Rs=%.2fkΩ R0=%.2fkΩ baseline=%.0fppm", rs,
                               mq135_r0, MQ135_BASELINE_PPM));
      }
    }
    float ppmCo2 = adcToPpm(co2Adc, mq135_r0, MQ135_A, MQ135_B);
    if (mq135Health.wasOffline) {
      logger.info(CAT_SENSOR, "MQ135 recovered",
                  logger.fmt("ADC=%.0f ppm=%.1f", co2Adc, ppmCo2));
      mq135Health.wasOffline = false;
    } else if (!mq135Health.everSucceeded) {
      logger.info(CAT_SENSOR, "MQ135 first read OK",
                  logger.fmt("ADC=%.0f ppm=%.1f%s", co2Adc, ppmCo2,
                             sensorsWarmedUp ? "" : " [warming]"));
    } else {
      logger.debug(CAT_SENSOR, "MQ135 OK",
                   logger.fmt("ADC=%.0f ppm=%.1f%s", co2Adc, ppmCo2,
                              sensorsWarmedUp ? "" : " [warming]"));
    }
    mq135Health.failCount = 0;
    mq135Health.everSucceeded = true;
    sensorCache.co2 = ppmCo2;
  }
#else
  sensorCache.co2 = NAN;
#endif

#ifdef HAS_MQ137
  float nh3Adc = readMQAverage(PIN_MQ137);

  if (nh3Adc < 30.0f) {
    mq137Health.failCount++;
    sensorCache.nh3 = NAN;
    if (!mq137Health.wasOffline) {
      mq137Health.wasOffline = true;
      logger.error(
          CAT_SENSOR, "MQ137 offline or disconnected",
          logger.fmt("ADC reading=%.0f on GPIO %d", nh3Adc, PIN_MQ137));
    }
  } else {
    if (sensorsWarmedUp && !mqCalibrated && MQ_AUTO_CALIBRATE) {
      float rs = adcToRs(nh3Adc);
      if (rs > 0.0f) {
        float newR0 =
            computeR0FromBaseline(rs, MQ137_BASELINE_PPM, MQ137_A, MQ137_B);
        if (newR0 > 0.0f)
          mq137_r0 = newR0;
        logger.info(CAT_SENSOR, "MQ137 R0 auto-calibrated",
                    logger.fmt("Rs=%.2fkΩ R0=%.2fkΩ baseline=%.1fppm", rs,
                               mq137_r0, MQ137_BASELINE_PPM));
      }
      mqCalibrated = true;
    }
    float ppmNh3 = adcToPpm(nh3Adc, mq137_r0, MQ137_A, MQ137_B);
    if (mq137Health.wasOffline) {
      logger.info(CAT_SENSOR, "MQ137 recovered",
                  logger.fmt("ADC=%.0f ppm=%.1f", nh3Adc, ppmNh3));
      mq137Health.wasOffline = false;
    } else if (!mq137Health.everSucceeded) {
      logger.info(CAT_SENSOR, "MQ137 first read OK",
                  logger.fmt("ADC=%.0f ppm=%.1f%s", nh3Adc, ppmNh3,
                             sensorsWarmedUp ? "" : " [warming]"));
    } else {
      logger.debug(CAT_SENSOR, "MQ137 OK",
                   logger.fmt("ADC=%.0f ppm=%.1f%s", nh3Adc, ppmNh3,
                              sensorsWarmedUp ? "" : " [warming]"));
    }
    mq137Health.failCount = 0;
    mq137Health.everSucceeded = true;
    sensorCache.nh3 = ppmNh3;
  }
#else
  sensorCache.nh3 = NAN;
#endif

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

  JsonDocument doc;
  doc["timestamp"] = timestamp;
  JsonArray arr = doc["sensors"].to<JsonArray>();

#ifdef HAS_DHT22
  {
    JsonObject s = arr.add<JsonObject>();
    bool connected = !isnan(sensorCache.temp);
    s["key"] = "temp_1";
    s["connected"] = connected;
    s["value"] =
        connected ? (round(sensorCache.temp * 100.0f) / 100.0f) : (float)NULL;
    if (connected)
      anySensor = true;
  }
  {
    JsonObject s = arr.add<JsonObject>();
    bool connected = !isnan(sensorCache.hum);
    s["key"] = "hum_1";
    s["connected"] = connected;
    s["value"] =
        connected ? (round(sensorCache.hum * 100.0f) / 100.0f) : (float)NULL;
    if (connected)
      anySensor = true;
  }
#endif

#ifdef HAS_MQ135
  {
    JsonObject s = arr.add<JsonObject>();
    bool connected = !isnan(sensorCache.co2);
    s["key"] = "co2_1";
    s["connected"] = connected;
    s["value"] =
        connected ? (round(sensorCache.co2 * 100.0f) / 100.0f) : (float)NULL;
    if (connected)
      anySensor = true;
  }
#endif

#ifdef HAS_MQ137
  {
    JsonObject s = arr.add<JsonObject>();
    bool connected = !isnan(sensorCache.nh3);
    s["key"] = "nh3_1";
    s["connected"] = connected;
    s["value"] =
        connected ? (round(sensorCache.nh3 * 100.0f) / 100.0f) : (float)NULL;
    if (connected)
      anySensor = true;
  }
#endif

  if (!anySensor) {
    logger.error(CAT_SENSOR, "All sensors failed",
                 "No data published this tick");
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