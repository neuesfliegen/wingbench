#pragma once

#include <Arduino.h>
#include <map>

typedef struct statusled_config {
  size_t pinR;
  size_t pinG;
  size_t pinB;
  bool inverted;
} statusled_config_t;

typedef struct statusled_pattern {
  size_t onTime;
  size_t offTime;
  size_t blinks;
  size_t pauseTime;
  uint8_t colorR;
  uint8_t colorG;
  uint8_t colorB;
} statusled_pattern_t;

#define STATUSLED_BLINK(r, g, b, count)                                \
  (statusled_pattern_t) {                                              \
    .onTime = 300, .offTime = 600, .blinks = count, .pauseTime = 1500, \
    .colorR = r, .colorG = g, .colorB = b                              \
  }

#define STATUSLED_BLINK_FAST(r, g, b, count)                           \
  (statusled_pattern_t) {                                              \
    .onTime = 150, .offTime = 300, .blinks = count, .pauseTime = 1000, \
    .colorR = r, .colorG = g, .colorB = b                              \
  }

#define STATUSLED_STATIC(r, g, b)                                           \
  (statusled_pattern_t) {                                                   \
    .onTime = 1000, .offTime = 0, .blinks = 1, .pauseTime = 0, .colorR = r, \
    .colorG = g, .colorB = b                                                \
  }

class StatusLED {
 private:
  statusled_config_t m_config;
  uint32_t m_analogRange = 64;

  std::map<String, statusled_pattern_t> m_statuses;
  statusled_pattern_t m_statusPattern;
  uint32_t m_statusTimeOffset = 0;
  uint32_t m_statusDuration;
  bool m_statusMuted = false;

  void setColor(uint8_t colorR, uint8_t colorG, uint8_t colorB);

 public:
  StatusLED();

  void begin(statusled_config_t config);
  void begin(size_t pinR, size_t pinG, size_t pinB, bool inverted = false);
  void tick();

  void setImmediate(statusled_pattern_t pattern);
  void setImmediate(size_t onTime,
                    size_t offTime,
                    size_t blinks,
                    size_t pauseTime,
                    uint8_t colorR,
                    uint8_t colorG,
                    uint8_t colorB);

  bool setStatus(String id);
  void addStatus(String id, statusled_pattern_t pattern);
  void addStatus(String id,
                 size_t onTime,
                 size_t offTime,
                 size_t blinks,
                 size_t pauseTime,
                 uint8_t colorR,
                 uint8_t colorG,
                 uint8_t colorB);
  void removeStatus(String id);

  void mute();
  void unmute();
};
