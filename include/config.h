#pragma once

// ─────────────────────────────────────────────────────────
// Connect your phone to it, open 192.168.4.1, enter the farm
// Wi-Fi credentials once. They are saved to flash permanently.
// ─────────────────────────────────────────────────────────
#define WIFI_AP_NAME "SyntaxIoT_Network"
#define WIFI_AP_PASSWORD "Syntax2026@"
#define WIFI_TIMEOUT_S 120 // seconds before portal gives up

// ─────────────────────────────────────────────────────────
// MQTT broker
// ─────────────────────────────────────────────────────────
#define MQTT_HOST "92.113.29.250"
#define MQTT_PORT 1883
#define MQTT_USER "" // add if your broker needs auth
#define MQTT_PASSWORD ""

// ─────────────────────────────────────────────────────────
// Device identity — change this per device before flashing
// All 3 MQTT topics are built automatically from DEVICE_ID
// ─────────────────────────────────────────────────────────
#define DEVICE_ID "20051648-94c5-4461-8646-8415aea544e1"
#define MQTT_CLIENT_ID DEVICE_ID

// ─────────────────────────────────────────────────────────
// MQTT topics  (syntax/{deviceID}/...)
// ─────────────────────────────────────────────────────────
#define TOPIC_DATA "syntax/" DEVICE_ID "/data"   // ESP32 → platform
#define TOPIC_SET "syntax/" DEVICE_ID "/set"     // platform → ESP32
#define TOPIC_RELAY "syntax/" DEVICE_ID "/relay" // ESP32 → platform

// ─────────────────────────────────────────────────────────
// Sensor presence flags
// Comment out any sensor NOT physically connected
// ─────────────────────────────────────────────────────────
#define HAS_DHT22 // temp_1 + hum_1
#define HAS_MQ135 // co2_1
#define HAS_MQ137 // nh3_1

// ─────────────────────────────────────────────────────────
// GPIO pins
// ─────────────────────────────────────────────────────────
#define PIN_DHT22 4
#define PIN_MQ135 34 // ADC1 — input only, safe for ADC
#define PIN_MQ137 35 // ADC1 — input only, safe for ADC
#define PIN_RELAY 26

// ─────────────────────────────────────────────────────────
// Status LED
// ─────────────────────────────────────────────────────────
#define PIN_STATUS_LED 2 // GPIO 2 — onboard blue LED on most ESP32 boards

// ─────────────────────────────────────────────────────────
// Sensor settings
// ─────────────────────────────────────────────────────────
#define DHT_TYPE DHT22
#define SENSOR_INTERVAL_MS 5000 // publish every 5 seconds
#define ADC_SAMPLES 20          // average 10 readings to reduce noise
#define ADC_WARMUP_MS 30000     // MQ heater warm-up: 30 seconds
#define DHT_MAX_FAILS 3         // consecutive fails before error log

// ─────────────────────────────────────────────────────────
// Alert thresholds
// ─────────────────────────────────────────────────────────
#define TEMP_MAX_C 35.0f
#define HUMIDITY_MIN_PCT 40.0f
#define HUMIDITY_MAX_PCT 80.0f
#define MQ135_ALERT 500
#define MQ137_ALERT 300

// ─────────────────────────────────────────────────────────
// Relay
// ─────────────────────────────────────────────────────────
#define RELAY_KEY "extr_1"

// ─────────────────────────────────────────────────────────
// Reconnect behaviour
// ─────────────────────────────────────────────────────────
#define RECONNECT_INTERVAL_MS 5000
#define MAX_RECONNECT_ATTEMPTS 10