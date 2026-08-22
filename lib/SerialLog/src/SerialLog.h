#pragma once

// Thread-safe Serial logging for FreeRTOS multi-task environments.
//
// Usage:
//   setup() {
//     Serial.begin(115200);
//     SerialLog::init();    // call once, before tasks start
//   }
//
//   // single-line atomic output:
//   LOG_PRINTF("count=%d\n", n);
//   LOG_PRINTLN("done");
//
//   // multi-line atomic output (hex dump, diag block, etc):
//   {
//     SerialLogGuard guard;
//     LOG_PRINTF("header: ");
//     for (...) LOG_PRINTF("%02X ", b);
//     LOG_PRINTLN();
//   }
//
// IMPORTANT: never call LOG_* from ISR context. Arduino Serial itself
// is not ISR-safe.

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace SerialLog {
    // Create the recursive mutex. Idempotent - safe to call multiple times.
    // Must be called after Serial.begin() and before any task that logs.
    void init();

    // For diagnostics / tests.
    SemaphoreHandle_t mutex();
}

// RAII guard. Acquires the recursive mutex with 10ms timeout in ctor,
// releases in dtor if acquired. On timeout, _locked stays false and Serial
// is still written to (unlocked fallback) - logs are never dropped.
class SerialLogGuard {
public:
    SerialLogGuard();
    ~SerialLogGuard();
private:
    bool _locked;
};

#define LOG_PRINT(...)   do { SerialLogGuard _slg; Serial.print(__VA_ARGS__); } while(0)
#define LOG_PRINTLN(...) do { SerialLogGuard _slg; Serial.println(__VA_ARGS__); } while(0)
#define LOG_PRINTF(...)  do { SerialLogGuard _slg; Serial.printf(__VA_ARGS__); } while(0)
#define LOG_WRITE(...)   do { SerialLogGuard _slg; Serial.write(__VA_ARGS__); } while(0)