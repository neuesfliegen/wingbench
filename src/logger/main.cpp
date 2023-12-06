#include <Arduino.h>
#include <SdFat.h>
#include <Wire.h>

// dependencies
#include <ArduinoJson.h>
#include <OneButton.h>

// libs
#include <StatusLED.h>
#include <TurboFIFO.h>

// shared code
#include "shared/bench.h"
#include "shared/config.h"
#include "shared/intercom.h"
#include "shared/utils.h"

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
#define INTERCOM_SERIAL_READ_BUFFER_SIZE (1024 * 32)
#define INTERCOM_SENSOR1_SERIAL Serial5
#define INTERCOM_SENSOR2_SERIAL Serial2
#define INTERCOM_SENSOR3_SERIAL Serial4
#define INTERCOM_SENSOR4_SERIAL Serial3
#define INTERCOM_SENSOR5_SERIAL Serial6

// Buttons
#define BUTTON_A_PIN (30u)
#define BUTTON_B_PIN (31u)

// Status LED
#define STATUSLED_PIN_R (36u)
#define STATUSLED_PIN_G (33u)
#define STATUSLED_PIN_B (37u)

/*
 * Logical configuration
 */

// Buffers
#define BUFSIZE_SENSOR_DATA (200)

/*
 * Application state
 */

// Peripherals
static StatusLED led;
OneButton buttonA(BUTTON_A_PIN, true, true);
OneButton buttonB(BUTTON_B_PIN, true, true);
SdFs fs;
FsFile logfile;

// Data
static HardwareSerial* intercomPorts[] = {
    &INTERCOM_SENSOR1_SERIAL, &INTERCOM_SENSOR2_SERIAL,
    &INTERCOM_SENSOR3_SERIAL, &INTERCOM_SENSOR4_SERIAL,
    &INTERCOM_SENSOR5_SERIAL};
static uint8_t intercomBuffers[ARRAYSIZE(intercomPorts)]
                              [INTERCOM_SERIAL_READ_BUFFER_SIZE];
static TurboFIFO<pressure_datapoint_t,
                 BUFSIZE_SENSOR_DATA * ARRAYSIZE(intercomPorts)>
    pressureDatapoints;

/*
 * Logging / SD card operation
 */

bool loggingInit() {
  if (!fs.begin(SdioConfig(FIFO_SDIO))) {
    fs.initErrorPrint(&TERMINAL_SERIAL);
    return false;
  }

  return true;
}

bool loggingOpenFile() {
  char filename[32];

  for (int i = 0;; i++) {
    snprintf(filename, sizeof(filename), "wingbench_%d.csv", i);

    if (!fs.exists(filename)) {
      logfile.open(filename, O_RDWR | O_CREAT | O_TRUNC);
      break;
    }
  }

  TERMINAL_SERIAL.printf("opened log file: %s\n", filename);

  if (!logfile.preAllocate(CONFIG_SDLOG_PREALLOCATE_BYTES)) {
    logfile.close();
    return false;
  }

  logfile.write("timestamp,sensor_id,");
  for (uint8_t i = 0; i < CONFIG_PRESSURE_COUNT - 1; i++) {
    logfile.printf("p_%d,", i);
  }
  logfile.printf("p_%d\n", CONFIG_PRESSURE_COUNT - 1);

  logfile.flush();

  return true;
}

void loggingFlushBuffer() {
  auto amount = pressureDatapoints.depth();
  TERMINAL_SERIAL.printf("writing %d datapoints to log...\n", amount);

  for (size_t i = 0; i < amount; i++) {
    pressure_datapoint_t d;
    if (!pressureDatapoints.dequeue(&d))
      break;

    logfile.printf("%d,%d,", d.data.localTime, d.port);

    uint8_t s = ARRAYSIZE(d.data.measurements);
    for (uint8_t j = 0; j < s - 1; j++)
      logfile.printf("%f,", d.data.measurements[j]);
    logfile.printf("%f\n", d.data.measurements[s - 1]);
  }

  logfile.flush();
  TERMINAL_SERIAL.println("flushed to sd card");
}

/*
 * Intercom operation
 */

void intercomSyncAll() {
  for (uint8_t i = 0; i < ARRAYSIZE(intercomPorts); i++)
    intercomPorts[i]->printf("sync %d\n", millis());
}

void intercomUnsyncAll() {
  for (uint8_t i = 0; i < ARRAYSIZE(intercomPorts); i++)
    intercomPorts[i]->printf("unsync");
}

void intercomFlushAndBuffer(HardwareSerial* intercom, uint8_t port) {
  // send flush command
  intercom->println("flush");

  // wait for sensor to start  transmitting data
  auto timeoutStart = millis();
  while (!intercom->available())
    if (millis() - timeoutStart > 100)
      return;

  // now deserialize all incoming data
  StaticJsonDocument<1024 * 64> doc;
  deserializeJson(doc, *intercom);

  // parse data into datapoints and save them to the global FIFO queue
  for (size_t i = 0; i < doc.size(); i++) {
    JsonObject dp = doc[i];
    JsonArray dm = dp["m"].as<JsonArray>();

    pressure_datapoint_t p = {.port = port, .data = {.localTime = dp["t"]}};
    for (uint8_t j = 0; j < dm.size(); j++)
      p.data.measurements[j] = dm[j];

    pressureDatapoints.enqueue(p);
  }
}

/*
 * Buttons
 */

void buttonInit() {
  // TODO: interrupts work correctly. OneButton does not, while following their
  // recommended way to implement the tick function.
  // Currently, two clicks trigger a click event at the end (most of the time).

  attachInterrupt(
      digitalPinToInterrupt(BUTTON_A_PIN),
      []() {
        buttonA.tick();
        Serial.printf("button a = %s\n", digitalRead(BUTTON_A_PIN) ? "1" : "0");
      },
      CHANGE);
  attachInterrupt(
      digitalPinToInterrupt(BUTTON_B_PIN),
      []() {
        buttonB.tick();
        Serial.printf("button b = %s\n", digitalRead(BUTTON_B_PIN) ? "1" : "0");
      },
      CHANGE);

  // Button A syncs all sensors
  buttonA.attachClick([]() { intercomSyncAll(); });
  buttonb.attachClick([]() {
    // TODO: implement recording start/stop with a state machine and
    // loggingOpenFile() as well as a future loggingCloseFile()
  });
}

/*
 * Helper / Debug functions
 */

void mirrorSerial(HardwareSerial* s) {
  if (TERMINAL_SERIAL.available()) {
    char c = TERMINAL_SERIAL.read();
    s->write(c);
  }
  if (s->available()) {
    char c = s->read();
    TERMINAL_SERIAL.write(c);
  }
}

/*
 * Arduino lifecycle functions
 */

void setup() {
  // logging
  TERMINAL_SERIAL.begin(TERMINAL_SERIAL_SPEED);

  // intercom
  for (uint8_t i = 0; i < ARRAYSIZE(intercomPorts); i++) {
    intercomPorts[i]->addMemoryForRead(intercomBuffers[i],
                                       INTERCOM_SERIAL_READ_BUFFER_SIZE);
    intercomPorts[i]->begin(INTERCOM_SERIAL_SPEED);
  }

  // status LED
  led.begin(STATUSLED_PIN_R, STATUSLED_PIN_G, STATUSLED_PIN_B);
  // yellow, constant
  led.addStatus("init", STATUSLED_STATIC(255, 255, 0));
  // red, constant
  led.addStatus("init/sdcard_fail", STATUSLED_STATIC(255, 0, 0));
  // green, 2 fast blinks
  led.addStatus("operation/idle", STATUSLED_BLINK_FAST(0, 100, 0, 2));
  // blue, 2 fast blinks
  led.addStatus("operation/recording", STATUSLED_BLINK_FAST(0, 0, 255, 2));

  TERMINAL_SERIAL.println("init...");

  led.setStatus("init");
  led.tick();

  // wait for serial terminal to connect
  delay(1000);

  TERMINAL_SERIAL.println("logging init...");

  if (!loggingInit()) {
    led.setStatus("init/sdcard_fail");
    for (;;) {
      led.tick();
      delay(50);
    }
  }

  TERMINAL_SERIAL.println("done");

  buttonInit();

  intercomUnsyncAll();
  intercomSyncAll();

  led.setStatus("operation/idle");

  loggingOpenFile();

  led.setStatus("operation/recording");
}

void loop() {
  // update StatusLED
  led.tick();

  // loop state
  static uint8_t currentIntercomPort = 0;

  // retrieve data
  intercomFlushAndBuffer(intercomPorts[currentIntercomPort],
                         currentIntercomPort + 1);

  // next intercom port
  currentIntercomPort = (currentIntercomPort + 1) % ARRAYSIZE(intercomPorts);

  // save data if buffer is almost full
  if (pressureDatapoints.depth() >
      BUFSIZE_SENSOR_DATA * (ARRAYSIZE(intercomPorts) - 1))
    loggingFlushBuffer();
}
