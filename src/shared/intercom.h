#pragma once

#include <Arduino.h>

#include "config.h"

enum class intercom_request_id : uint8_t {
  TimeSync = 1,
  FlushDataset = 2,
};

typedef struct intercom_request {
  intercom_request_id request;
} intercom_request_t;

typedef struct intercom_result_timesync {
  uint32_t localTime;
} intercom_result_timesync_t;

typedef struct intercom_result_datapoint_pressure {
  uint32_t localTime;
  float measurements[CONFIG_PRESSURE_COUNT];
} intercom_result_datapoint_pressure_t;
