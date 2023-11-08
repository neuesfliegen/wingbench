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
static bool stateSyncSet = false;
static int32_t stateSyncOffset = 0;
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

static void intercomSyncTime(int32_t offset) {
  TERMINAL_SERIAL.printf("syncing time = %d\n", offset);

  stateSyncSet = true;
  stateSyncOffset = offset;

  led.setStatus("operation/synced");
}

static bool intercomFlushQueue() {
  auto depth = stateDatasetFIFO.depth();

  if (depth == 0)
    return false;

  // allocate json document
  StaticJsonDocument<16 + (18 * PRESSURE_COUNT) * BUFSIZE_PRESSURE_DATA> doc;
  auto ds = doc.to<JsonArray>();

  for (size_t i = 0; i < depth; i++) {
    // fetch datapoint and save it to the json object
    intercom_result_datapoint_pressure_t p;
    auto deqResult = stateDatasetFIFO.dequeue(&p);

    // abort if nothing is in the queue
    if (!deqResult)
      continue;

    auto dp = ds.createNestedObject();

    // copy struct data
    dp["t"] = p.localTime;
    auto dpMeasurements = dp.createNestedArray("m");
    copyArray(p.measurements, dpMeasurements);
  }

  // stream serialized data to intercom
  serializeMsgPack(ds, INTERCOM_SERIAL);
  INTERCOM_SERIAL.println();
  return true;
}

static void intercomHandleCommand(char* rawInput) {
  auto rawCmd = String(rawInput);
  rawCmd.trim();

  auto cmdIndex = rawCmd.indexOf(' ', 0);
  auto cmd = rawCmd.substring(0, cmdIndex > 0 ? cmdIndex : rawCmd.length());
  auto param = rawCmd.substring(cmdIndex + 1, rawCmd.length());
  cmd.trim();
  param.trim();

  auto command = cmd.c_str();

  if (strcmp(command, "flush") == 0) {
    intercomFlushQueue();
  } else if (strcmp(command, "sync") == 0) {
    intercomSyncTime(param.toInt());
  } else {
    TERMINAL_SERIAL.println("intercom: unknown command");
  }
}

static void intercomTick() {
  static char commandBuf[64];
  static uint8_t commandBufIndex = 0;

  while (INTERCOM_SERIAL.available()) {
    char next;
    INTERCOM_SERIAL.readBytes(&next, 1);

    if (commandBufIndex == 63)
      commandBufIndex = 0;

    if (commandBufIndex > 0 && next == '\n') {
      char cmd[64];
      strncpy(cmd, commandBuf, commandBufIndex + 1);

      intercomHandleCommand(cmd);

      memset(commandBuf, 0, sizeof(commandBuf));
      commandBufIndex = 0;
      break;
    } else {
      commandBuf[commandBufIndex] = next;
      commandBufIndex++;
    }
  }
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
  // blue, constant
  led.addStatus("init", STATUSLED_STATIC(0, 0, 255));
  // red, 2 blinks
  led.addStatus("init/sensor_fail", STATUSLED_BLINK(255, 0, 0, 2));
  // violet, 2 fast blinks
  led.addStatus("operation/unsynced", STATUSLED_BLINK(255, 0, 255, 2));
  // green, 2 fast blinks
  led.addStatus("operation/synced", STATUSLED_BLINK_FAST(0, 255, 0, 2));

  // init status
  led.setStatus("init");
  led.tick();

  // init pressure sensors
  pressureInit();

  led.setStatus("operation/unsynced");
}

void loop() {
  auto start = millis();

  {
    BENCH("pressureTick", pressureTick());
    BENCH("statusLED", led.tick());
  }

  auto delta = millis() - start;
  delay((1000 / CONFIG_PRESSURE_MEASUREMENT_HZ) - min(0, delta));
}

void loop1() {
  intercomTick();
}
