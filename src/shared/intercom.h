#pragma once

#include <Arduino.h>

#include "config.h"

typedef struct intercom_result_datapoint_pressure {
  uint32_t localTime;
  float measurements[CONFIG_PRESSURE_COUNT];
} intercom_result_datapoint_pressure_t;

typedef struct pressure_datapoint {
  uint8_t port;
  intercom_result_datapoint_pressure_t data;
} pressure_datapoint_t;
