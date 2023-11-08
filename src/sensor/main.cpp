#include <Arduino.h>
#include <Wire.h>
#include <pico/stdlib.h>

// dependencies
#include <ArduinoJson.h>

// libs
#include <AMS.h>
#include <StatusLED.h>
#include <TurboFIFO.h>

// shared code
#include "shared/bench.h"
#include "shared/config.h"
#include "shared/intercom.h"

/*
 * Hardware / IO Configuration
 */

// Serial debug logging
#define TERMINAL_SERIAL Serial
#define TERMINAL_SERIAL_SPEED CONFIG_DEBUGLOG_BAUDRATE

// Serial intercom data transfer
#define INTERCOM_SERIAL Serial1
#define INTERCOM_SERIAL_SPEED CONFIG_INTERCOM_BAUDRATE
#define INTERCOM_SERIAL_PIN_RX (17u)
#define INTERCOM_SERIAL_PIN_TX (16u)

// Pressure sensors
#define PRESSURE_BUS Wire
#define PRESSURE_SPEED (800'000u)
#define PRESSURE_PIN_SDA (20u)
#define PRESSURE_PIN_SCL (21u)
#define PRESSURE_COUNT CONFIG_PRESSURE_COUNT
#define PRESSURE_BASE_ADDRESS (0x40)

// Status LED
#define STATUSLED_PIN_R (6u)
#define STATUSLED_PIN_G (8u)
#define STATUSLED_PIN_B (7u)

// Sensor Tick LED
#define SENSORLED_PIN LED_BUILTIN

/*
 * Logical configuration
 */

// Buffers
#define BUFSIZE_PRESSURE_DATA (256u)

/*
 * Application state
 */

// Peripherals
static AMS pressureSensors[PRESSURE_COUNT];
static StatusLED led;

// Data
static TurboFIFO<intercom_result_datapoint_pressure_t, BUFSIZE_PRESSURE_DATA>
    stateDatasetFIFO;

/*
 * Pressure sensor functions
 */

static void pressureInit() {
  // status LED
  pinMode(LED_BUILTIN, PinMode::OUTPUT);

  // sensor profile
  auto profile = ams_profile_t{
      .type = AMSSensorType::AMS_6915,
      .pressureMin = -25,
      .pressureMax = +25,
  };

  // sensors init
  PRESSURE_BUS.setSDA(PRESSURE_PIN_SDA);
  PRESSURE_BUS.setSCL(PRESSURE_PIN_SCL);
  PRESSURE_BUS.setClock(PRESSURE_SPEED);
  PRESSURE_BUS.begin();

  for (uint8_t i = 0; i < PRESSURE_COUNT; i++) {
    pressureSensors[i] = AMS();
    pressureSensors[i].profile(profile);
    auto initSuccess =
        pressureSensors[i].begin(&PRESSURE_BUS, PRESSURE_BASE_ADDRESS + i);

    // halt with status led if sensor init fails
    if (!initSuccess) {
      led.setStatus("init/sensor_fail");
      for (;;) {
        led.tick();
        delay(50);
      }
    }
  }
}

static intercom_result_datapoint_pressure_t pressureReadDatapoint() {
  intercom_result_datapoint_pressure_t point{
      .localTime = millis(),
  };

  // Read out all sensors to the current buffer
  for (uint8_t i = 0; i < PRESSURE_COUNT; i++) {
    if (pressureSensors[i].update()) {
      point.measurements[i] = pressureSensors[i].pressure();
    } else {
      // TODO: switch to NaN and enable ArduinoJSON support?
      point.measurements[i] = 0.0f;
    }
  }

  return point;
}

static void pressureTick() {
  // enable status LED before tick
  digitalWrite(SENSORLED_PIN, PinStatus::HIGH);

  // read data
  auto data = pressureReadDatapoint();
  stateDatasetFIFO.enqueue(data);

#ifdef TELEPLOT_ENABLE
  for (uint8_t i = 0; i < CONFIG_PRESSURE_COUNT; i++) {
    TERMINAL_SERIAL.printf(">sensor_%d:%.8f:%d\n", i, data.measurements[i],
                           data.localTime);
  }
#endif

  // disable status LED after tick
  digitalWrite(SENSORLED_PIN, PinStatus::LOW);
}

/*
 * Intercom
 */

static bool intercomInit() {
  // TODO: wait for master to ask for time offset
  // TODO: reply with local millis

  return true;
}

static void intercomSyncTime() {
  // TODO
}

static bool intercomFlushQueue() {
  if (stateDatasetFIFO.depth() == 0)
    return false;

  // determine how many datapoints to send
  auto queueDepth = min(BUFSIZE_PRESSURE_DATA, stateDatasetFIFO.depth());

  // allocate json document
  StaticJsonDocument<16 + (18 * PRESSURE_COUNT) * BUFSIZE_PRESSURE_DATA> doc;
  auto ds = doc.to<JsonArray>();

  for (size_t i = 0; i < queueDepth; i++) {
    // fetch datapoint and save it to the json object
    intercom_result_datapoint_pressure_t p;
    auto deqResult = stateDatasetFIFO.dequeue(&p);

    // abort if nothing is in the queue
    if (!deqResult)
      continue;

    auto dp = ds.createNestedObject();

    // copy struct data
    dp["ts"] = p.localTime;
    auto dpMeasurements = dp.createNestedArray("m");
    copyArray(p.measurements, dpMeasurements);
  }

  // stream serialized data to intercom
  // TODO: switch to messagepack
  serializeJson(ds, INTERCOM_SERIAL);
  INTERCOM_SERIAL.println();
  return true;
}

/*
 * Arduino lifecycle functions
 */

void setup() {
  // logging
  TERMINAL_SERIAL.begin(TERMINAL_SERIAL_SPEED);

// intercom
#if INTERCOM_SERIAL != Serial
  INTERCOM_SERIAL.setRX(INTERCOM_SERIAL_PIN_RX);
  INTERCOM_SERIAL.setTX(INTERCOM_SERIAL_PIN_TX);
#endif
  INTERCOM_SERIAL.begin(INTERCOM_SERIAL_SPEED);

  // status LED
  led.begin(STATUSLED_PIN_R, STATUSLED_PIN_G, STATUSLED_PIN_B);
  // yellow, constant
  led.addStatus("init", STATUSLED_STATIC(255, 255, 0));
  // red, 2 blinks
  led.addStatus("init/sensor_fail", STATUSLED_BLINK(255, 0, 0, 2));
  // red, 3 blinks
  led.addStatus("init/intercom_fail", STATUSLED_BLINK(255, 0, 0, 3));
  // blue, 2 fast blinks
  led.addStatus("operation/idle", STATUSLED_BLINK_FAST(0, 0, 255, 2));
  // green, 2 fast blinks
  led.addStatus("operation/recording", STATUSLED_BLINK_FAST(0, 255, 0, 2));

  // wait for serial terminal to connect
  led.setStatus("init");
  led.tick();
  delay(1000);

  intercomInit();
  pressureInit();

  intercomSyncTime();

  led.setStatus("operation/idle");
}

void loop() {
  BENCH("pressureTick", pressureTick());
  BENCH("statusLED", led.tick());
  delay(1000 / CONFIG_PRESSURE_MEASUREMENT_HZ);
}

void loop1() {
  // TODO; determine if idle or recording from intercom command
  // TODO: set status LED state
  // BENCH("intercomTick", intercomFlushQueue());
  delay(1000);
}
