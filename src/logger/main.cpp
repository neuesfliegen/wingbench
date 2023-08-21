#include <Arduino.h>
#include <SdFat.h>

// dependencies
#include <ArduinoJson.h>

// libs
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

// SD card logging
#define SDLOG_PREALLOC_SIZE CONFIG_SDLOG_PREALLOCATE_BYTES

// Serial intercom data transfer
#define INTERCOM_SERIAL_SPEED CONFIG_INTERCOM_BAUDRATE
#define INTERCOM_SENSOR1_SERIAL Serial1
#define INTERCOM_SENSOR2_SERIAL Serial2
#define INTERCOM_SENSOR3_SERIAL Serial3
#define INTERCOM_SENSOR4_SERIAL Serial4
#define INTERCOM_SENSOR5_SERIAL Serial5

// Status LED
#define STATUSLED_PIN_R (6u)
#define STATUSLED_PIN_G (8u)
#define STATUSLED_PIN_B (7u)

/*
 * Logical configuration
 */

// Buffers
#define BUFSIZE_SENSOR_DATA (512u)

/*
 * Application state
 */

// Peripherals
static StatusLED led;
SdFs stateFs;
FsFile stateLogfile;

// Data
static TurboFIFO<intercom_result_datapoint_pressure_t, BUFSIZE_SENSOR_DATA>
    stateSensor1FIFO;
static TurboFIFO<intercom_result_datapoint_pressure_t, BUFSIZE_SENSOR_DATA>
    stateSensor2FIFO;
static TurboFIFO<intercom_result_datapoint_pressure_t, BUFSIZE_SENSOR_DATA>
    stateSensor3FIFO;
static TurboFIFO<intercom_result_datapoint_pressure_t, BUFSIZE_SENSOR_DATA>
    stateSensor4FIFO;
static TurboFIFO<intercom_result_datapoint_pressure_t, BUFSIZE_SENSOR_DATA>
    stateSensor5FIFO;

/*
 * Logging / SD card operation
 */

bool loggingInit() {
  if (!stateFs.begin(SdioConfig(FIFO_SDIO))) {
    stateFs.initErrorHalt(&TERMINAL_SERIAL);
    Serial.println("a");
    return false;
  }
  if (!stateLogfile.open("log.csv", O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("b");
    return false;
  }
  if (!stateLogfile.preAllocate(512)) {
    Serial.println("c");
    stateLogfile.close();
    return false;
  }

  stateLogfile.write("Hello, World", 13);
  stateLogfile.sync();
  stateLogfile.close();

  return true;
}

/*
 * Arduino lifecycle functions
 */

void setup() {
  // logging
  TERMINAL_SERIAL.begin(TERMINAL_SERIAL_SPEED);

  // status LED
  led.begin(STATUSLED_PIN_R, STATUSLED_PIN_G, STATUSLED_PIN_B);
  // yellow, constant
  led.addStatus("init", STATUSLED_STATIC(255, 255, 0));
  // red, 3 blinks
  led.addStatus("init/intercom_fail", STATUSLED_BLINK(255, 0, 0, 3));
  // orange, 4 blinks
  led.addStatus("init/sdcard_fail", STATUSLED_BLINK(255, 127, 0, 2));
  // blue, 2 fast blinks
  led.addStatus("operation/idle", STATUSLED_BLINK_FAST(0, 0, 255, 2));
  // green, 2 fast blinks
  led.addStatus("operation/recording", STATUSLED_BLINK_FAST(0, 255, 0, 2));
  // green, fast blinking
  led.addStatus("operation/recording", (statusled_pattern_t){
                                           .onTime = 100,
                                           .offTime = 100,
                                           .blinks = 1,
                                           .pauseTime = 0,
                                           .colorR = 0,
                                           .colorG = 255,
                                           .colorB = 0,
                                       });

  // wait for serial terminal to connect
  led.setStatus("init");
  led.tick();
  delay(1000);

  if (!loggingInit()) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
  }

  led.setStatus("operation/idle");
}

void loop() {}
