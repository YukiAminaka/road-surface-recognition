#include "SerialLog.h"

namespace {
    SemaphoreHandle_t g_serialMutex = nullptr;
    constexpr TickType_t LOCK_TIMEOUT_TICKS = pdMS_TO_TICKS(10);
}

void SerialLog::init() {
    if (g_serialMutex == nullptr) {
        g_serialMutex = xSemaphoreCreateRecursiveMutex();
    }
}

SemaphoreHandle_t SerialLog::mutex() {
    return g_serialMutex;
}

SerialLogGuard::SerialLogGuard() : _locked(false) {
    if (g_serialMutex != nullptr) {
        _locked = (xSemaphoreTakeRecursive(g_serialMutex, LOCK_TIMEOUT_TICKS) == pdTRUE);
    }
}

SerialLogGuard::~SerialLogGuard() {
    if (_locked) {
        xSemaphoreGiveRecursive(g_serialMutex);
    }
}