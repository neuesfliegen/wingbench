#include <Arduino.h>
#include <Wire.h>

#include "AMS.h"

AMS::AMS()
    : m_pressureRawData{0, 0},
      m_temperatureRawData{0, 0},
      m_pressure{NAN},
      m_temperature{NAN} {};

ams_profile_t AMS::profile() {
  return m_profile;
};

void AMS::profile(ams_profile_t profile) {
  m_profile = profile;

#ifdef AMS_DEBUG
  AMS_DEBUG_SERIAL.print("AMS: profile | min = ");
  AMS_DEBUG_SERIAL.print(m_profile.pressureMin);
  AMS_DEBUG_SERIAL.print(", max = ");
  AMS_DEBUG_SERIAL.print(m_profile.pressureMax);
  AMS_DEBUG_SERIAL.print(", type = ");
  AMS_DEBUG_SERIAL.println((uint8_t)m_profile.type);
#endif
};

bool AMS::begin() {
  return begin(&AMS_DEFAULT_WIRE);
};

bool AMS::begin(TwoWire* wire) {
  return begin(wire, AMS_DEFAULT_I2CADDR);
};

bool AMS::begin(TwoWire* wire, uint8_t deviceAddress) {
  m_wire = wire;
  m_address = deviceAddress;

#ifdef AMS_DEBUG
  AMS_DEBUG_SERIAL.print("AMS: begin | addr = 0x");
  AMS_DEBUG_SERIAL.println(m_address, HEX);
#endif

  for (uint8_t i = 0; i < AMS_BEGIN_RETRY_COUNT; i++) {
    m_wire->requestFrom(m_address, 4);

    if (m_wire->available() == 4) {
#ifdef AMS_DEBUG
      AMS_DEBUG_SERIAL.print("AMS: begin | init done, retry = ");
      AMS_DEBUG_SERIAL.print(i);
      AMS_DEBUG_SERIAL.print(", addr = 0x");
      AMS_DEBUG_SERIAL.println(m_address, HEX);
#endif

      return true;
    } else {
#ifdef AMS_DEBUG
      AMS_DEBUG_SERIAL.print("AMS: begin | init failed, retry = ");
      AMS_DEBUG_SERIAL.print(i);
      AMS_DEBUG_SERIAL.print(", addr = 0x");
      AMS_DEBUG_SERIAL.println(m_address, HEX);
#endif

      delay(AMS_BEGIN_RETRY_DELAY_MS);
    }
  }

  return false;
};

/**
 * Verify if the AMS sensor is available for usage.
 * Only use for checks in setup-like contexts, as it takes a good bit of
 * time.
 * For checks if data is retrievable, use the return value of `update()`
 * instead.
 *
 * @see AMS::update
 * @return
 */
bool AMS::available() {
  // config/profile validity
  if (
      // pressure in invalid range (min > max or both equal)
      (m_profile.pressureMin >= m_profile.pressureMax))
    return false;

  // i2c availability
  m_wire->requestFrom(m_address, 4);
  return m_wire->available() == 4;
};

bool AMS::update() {
  // request 4 bytes from I2C
  if (m_wire->requestFrom(m_address, 4) != 4) {
#ifdef AMS_DEBUG
    AMS_DEBUG_SERIAL.println("AMS: update | failed");
#endif

    return false;
  }

  // read data
  m_wire->readBytes(m_pressureRawData, 2);
  m_wire->readBytes(m_temperatureRawData, 2);

  // reset cached values
  m_pressure = NAN;
  m_temperature = NAN;

#ifdef AMS_DEBUG
  AMS_DEBUG_SERIAL.println("AMS: update | success");
#endif

  return true;
}

float AMS::pressure() {
  // use cached value if present
  if (!isnan(m_pressure)) {
#ifdef AMS_DEBUG
    AMS_DEBUG_SERIAL.println("AMS: pressure | cache hit");
#endif

    return m_pressure;
  }

  // calculate data from raw values
  switch (m_profile.type) {
    case AMSSensorType::AMS_5812:
      m_pressure =
          ((256 * m_pressureRawData[0] + m_pressureRawData[1]) - 3277.0f) *
              (m_profile.pressureMax - m_profile.pressureMin) / 26214.0f +
          m_profile.pressureMin;
      break;

    case AMSSensorType::AMS_5915:
    case AMSSensorType::AMS_6915:
      m_pressure =
          (((256 * (m_pressureRawData[0] & 0x3F) + m_pressureRawData[1]) -
            1638.0f) *
           (m_profile.pressureMax - m_profile.pressureMin) / 13107) +
          m_profile.pressureMin;
      break;
  };

#ifdef AMS_DEBUG
  AMS_DEBUG_SERIAL.print("AMS: pressure | cache miss, value = ");
  AMS_DEBUG_SERIAL.print(m_pressure);
  AMS_DEBUG_SERIAL.println(" mbar");
#endif

  return m_pressure;
};

float AMS::temperature() {
  // use cached value if present
  if (!isnan(m_temperature)) {
#ifdef AMS_DEBUG
    AMS_DEBUG_SERIAL.println("AMS: temperature | cache hit");
#endif

    return m_temperature;
  }

  // calculate data from raw values
  switch (m_profile.type) {
    case AMSSensorType::AMS_5812:
      m_temperature =
          (((256 * m_temperatureRawData[0] + m_temperatureRawData[1]) - 3277) /
           238.309f) -
          25.0f;
      break;

    case AMSSensorType::AMS_5915:
    case AMSSensorType::AMS_6915:
      m_temperature =
          (((256.0f * m_temperatureRawData[0] + m_temperatureRawData[1]) *
            200.0f) /
           65536.0f) -
          50;
      break;
  };

#ifdef AMS_DEBUG
  AMS_DEBUG_SERIAL.print("AMS: temperature | cache miss, value = ");
  AMS_DEBUG_SERIAL.print(m_temperature);
  AMS_DEBUG_SERIAL.println(" deg C");
#endif

  return m_temperature;
};
