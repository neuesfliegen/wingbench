#include "StatusLED.hpp"

StatusLED::StatusLED() {}

void StatusLED::setColor(uint8_t colorR, uint8_t colorG, uint8_t colorB) {
  auto r = map(colorR, 0, 255, 0, m_analogRange - 1);
  auto g = map(colorG, 0, 255, 0, m_analogRange - 1);
  auto b = map(colorB, 0, 255, 0, m_analogRange - 1);

  if (m_config.inverted) {
    r = m_analogRange - 1 - r;
    g = m_analogRange - 1 - g;
    b = m_analogRange - 1 - b;
  }

  analogWrite(m_config.pinR, r);
  analogWrite(m_config.pinG, g);
  analogWrite(m_config.pinB, b);
}

void StatusLED::begin(statusled_config_t config) {
  m_config = config;
  pinMode(m_config.pinR, OUTPUT);
  pinMode(m_config.pinG, OUTPUT);
  pinMode(m_config.pinB, OUTPUT);
  // TODO: is this a good idea?
  // analogWriteRange(256);
};

void StatusLED::begin(size_t pinR, size_t pinG, size_t pinB, bool inverted) {
  begin((statusled_config_t){
      .pinR = pinR,
      .pinG = pinG,
      .pinB = pinB,
      .inverted = inverted,
  });
}

void StatusLED::tick() {
  if (m_statusMuted)
    return;

  // interval from 0 to statusDuration
  auto cycleTime = (millis() - m_statusTimeOffset) % m_statusDuration;

  // timing: (on, onTime, off, offTime) * (blinks - 1), on, onTime, pauseTime
  // ---_____---_____---__________
  auto tOn = m_statusPattern.onTime;
  auto tOff = m_statusPattern.offTime;
  auto b = m_statusPattern.blinks;
  bool ledOn = (cycleTime % (tOn + tOff) <= tOn) &&
               (cycleTime <= (((tOn + tOff) * b) - tOff));

  if (ledOn)
    setColor(m_statusPattern.colorR, m_statusPattern.colorG,
             m_statusPattern.colorB);
  else
    setColor(0, 0, 0);
}

void StatusLED::setImmediate(statusled_pattern_t p) {
  m_statusPattern = p;
  m_statusDuration =
      ((p.onTime + p.offTime) * p.blinks - p.offTime) + (p.pauseTime);
  m_statusTimeOffset = millis();
}

void StatusLED::setImmediate(size_t onTime,
                             size_t offTime,
                             size_t blinks,
                             size_t pauseTime,
                             uint8_t colorR,
                             uint8_t colorG,
                             uint8_t colorB) {
  setImmediate((statusled_pattern_t){
      .onTime = onTime,
      .offTime = offTime,
      .blinks = blinks,
      .pauseTime = pauseTime,
      .colorR = colorR,
      .colorG = colorG,
      .colorB = colorB,
  });
}

bool StatusLED::setStatus(String id) {
  if (m_statuses.find(id) == m_statuses.end())
    return false;

  auto p = m_statuses[id];
  setImmediate(p);

  return true;
};

void StatusLED::addStatus(String id, statusled_pattern_t pattern) {
  if (m_statuses.find(id) == m_statuses.end())
    m_statuses.erase(id);

  m_statuses[id] = pattern;
};

void StatusLED::addStatus(String id,
                          size_t onTime,
                          size_t offTime,
                          size_t blinks,
                          size_t pauseTime,
                          uint8_t colorR,
                          uint8_t colorG,
                          uint8_t colorB) {
  addStatus(id, (statusled_pattern_t){
                    .onTime = onTime,
                    .offTime = offTime,
                    .blinks = blinks,
                    .pauseTime = pauseTime,
                    .colorR = colorR,
                    .colorG = colorG,
                    .colorB = colorB,
                });
};

void StatusLED::removeStatus(String id) {
  m_statuses.erase(id);
};

void StatusLED::mute() {
  m_statusMuted = true;
  setColor(0, 0, 0);
}

void StatusLED::unmute() {
  m_statusMuted = false;
}
