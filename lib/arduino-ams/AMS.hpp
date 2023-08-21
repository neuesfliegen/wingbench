#pragma once

#include <Arduino.h>
#include <Wire.h>

#ifndef AMS_DEBUG_SERIAL
#define AMS_DEBUG_SERIAL Serial
#endif

#define AMS_DEFAULT_WIRE Wire
#define AMS_DEFAULT_I2CADDR (0x28u)

#define AMS_BEGIN_RETRY_COUNT (10u)
#define AMS_BEGIN_RETRY_DELAY_MS (5u)

enum class AMSSensorType : uint8_t {
  /**
   * AMS 5812 pressure sensor
   * https://www.analog-micro.com/en/products/pressure-sensors/board-mount-pressure-sensors/ams5812/
   */
  AMS_5812 = 1,

  /**
   * AMS 5915 pressure sensor
   * https://www.analog-micro.com/en/products/pressure-sensors/board-mount-pressure-sensors/ams5915/
   */
  AMS_5915 = 2,

  /**
   * AMS 6915 pressure sensor
   * https://www.analog-micro.com/en/products/pressure-sensors/board-mount-pressure-sensors/ams6915/
   */
  AMS_6915 = 4,
};

typedef struct ams_profile {
  /// The type/series of sensor used
  AMSSensorType type;
  /// The minimum pressure the sensor is configured to read, in mbar.
  int16_t pressureMin;
  /// The maximum pressure the sensor is configured to read, in mbar.
  int16_t pressureMax;
} ams_profile_t;

class AMS {
 private:
  // config
  ams_profile_t m_profile;
  TwoWire* m_wire;
  uint8_t m_address;
  // data
  uint8_t m_pressureRawData[2]{};
  uint8_t m_temperatureRawData[2]{};
  float m_pressure{};
  float m_temperature{};

 public:
  AMS();

  ams_profile_t profile();
  void profile(ams_profile_t profile);

  bool begin();
  bool begin(TwoWire* wire);
  bool begin(TwoWire* wire, uint8_t deviceAddress);

  /**
   * @brief Check whether or not the sensor is available to be used.
   * @returns `true` on success, `false` if there was a communication error or
   * no valid configuration
   */
  bool available();

  /**
   * @brief Read and calculate the current values from the sensor.
   * @returns `true` on success, `false` if there was an exception.
   */
  bool update();

  /**
   * @brief Get the pressure from the most recently updated data.
   * @return The last read pressure value in mbar.
   */
  float pressure();

  /**
   * @brief Get the temperature from the most recently updated data.
   * @return The last read temperature value in deg Celsius.
   */
  float temperature();
};
