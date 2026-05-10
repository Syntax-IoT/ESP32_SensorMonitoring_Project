#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────
// Log levels
// ─────────────────────────────────────────────────────────
enum LogLevel { LOG_DEBUG = 0, LOG_INFO = 1, LOG_WARN = 2, LOG_ERROR = 3 };

// ─────────────────────────────────────────────────────────
// Log categories
// ─────────────────────────────────────────────────────────
#define CAT_WIFI "WIFI"
#define CAT_MQTT "MQTT"
#define CAT_SENSOR "SENSOR"
#define CAT_RELAY "RELAY"
#define CAT_NTP "NTP"
#define CAT_SYSTEM "SYSTEM"

// ─────────────────────────────────────────────────────────
// Minimum level to print — change to LOG_INFO in production
// ─────────────────────────────────────────────────────────
#define LOG_LEVEL_SERIAL LOG_DEBUG

// ─────────────────────────────────────────────────────────
// Logger — serial monitor only
// Output goes to PlatformIO serial monitor during development.
// No MQTT publishing — only the 3 platform topics are used:
//   syntax/{deviceID}/data
//   syntax/{deviceID}/set
//   syntax/{deviceID}/relay
// ─────────────────────────────────────────────────────────
class Logger {
public:
  void log(LogLevel level, const char *category, const char *event,
           const char *detail = "") {
    if (level < LOG_LEVEL_SERIAL)
      return;
    Serial.printf("[%s][%s] %s%s%s\n", levelStr(level), category, event,
                  strlen(detail) > 0 ? " — " : "", detail);
  }

  void debug(const char *cat, const char *event, const char *detail = "") {
    log(LOG_DEBUG, cat, event, detail);
  }
  void info(const char *cat, const char *event, const char *detail = "") {
    log(LOG_INFO, cat, event, detail);
  }
  void warn(const char *cat, const char *event, const char *detail = "") {
    log(LOG_WARN, cat, event, detail);
  }
  void error(const char *cat, const char *event, const char *detail = "") {
    log(LOG_ERROR, cat, event, detail);
  }

  // Usage: logger.info(CAT_SENSOR, "DHT22 OK", logger.fmt("T=%.1f H=%.1f", t,
  // h));
  const char *fmt(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(_fmtBuf, sizeof(_fmtBuf), format, args);
    va_end(args);
    return _fmtBuf;
  }

private:
  char _fmtBuf[128];

  const char *levelStr(LogLevel l) {
    switch (l) {
    case LOG_DEBUG:
      return "DBG";
    case LOG_INFO:
      return "INF";
    case LOG_WARN:
      return "WRN";
    case LOG_ERROR:
      return "ERR";
    default:
      return "???";
    }
  }
};

extern Logger logger;