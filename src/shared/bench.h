#pragma once

#include <Arduino.h>

#ifndef BENCH_LOGGING_SERIAL
#define BENCH_LOGGING_SERIAL Serial
#endif

#ifdef BENCH_ENABLE
#define BENCH(id, statement)                                                  \
  {                                                                           \
    auto ___benchTimeA = micros();                                            \
    statement;                                                                \
    auto ___benchTimeB = micros();                                            \
    BENCH_LOGGING_SERIAL.print("BENCH(");                                     \
    BENCH_LOGGING_SERIAL.print((id));                                         \
    BENCH_LOGGING_SERIAL.print("): ");                                        \
    BENCH_LOGGING_SERIAL.print((___benchTimeB - ___benchTimeA) / 1000.0f, 2); \
    BENCH_LOGGING_SERIAL.println(" ms");                                      \
  }

#define BENCH_ITER(id, times, statement)                                      \
  {                                                                           \
    auto ___benchTimeA = micros();                                            \
    for (size_t ___benchIterCount = 0; ___benchIterCount < (times);           \
         ___benchIterCount++) {                                               \
      statement;                                                              \
    }                                                                         \
    auto ___benchTimeB = micros();                                            \
    BENCH_LOGGING_SERIAL.print("BENCH(");                                     \
    BENCH_LOGGING_SERIAL.print((id));                                         \
    BENCH_LOGGING_SERIAL.print(")[times=");                                   \
    BENCH_LOGGING_SERIAL.print((times));                                      \
    BENCH_LOGGING_SERIAL.print("]: total=");                                  \
    BENCH_LOGGING_SERIAL.print((___benchTimeB - ___benchTimeA) / 1000.0f, 2); \
    BENCH_LOGGING_SERIAL.print("ms, avg=");                                   \
    BENCH_LOGGING_SERIAL.print(                                               \
        (___benchTimeB - ___benchTimeA) / 1000.0f / (times), 2);              \
    BENCH_LOGGING_SERIAL.println("ms");                                       \
  }
#else
#define BENCH(id, statement) statement;

#define BENCH_ITER(id, times, statement)                            \
  {                                                                 \
    for (size_t ___benchIterCount = 0; ___benchIterCount < (times); \
         ___benchIterCount++) {                                     \
      statement;                                                    \
    }                                                               \
  }

#endif