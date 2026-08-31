#include "RTCDriver.h"
#include <PCF8563.h>
#include <Arduino.h>
#include <ctime>
#include <SerialLog.h>

static PCF8563 pcf_;

bool RTCDriver::begin() {
    Config defaultConfig;
    return begin(defaultConfig);
}

bool RTCDriver::begin(const Config& config) {
    config_ = config;
    mutex_ = xSemaphoreCreateMutex();
    // PCF8563 uses Wire internally
    return mutex_ != nullptr;
}

bool RTCDriver::begin(const Config& config, SemaphoreHandle_t i2cMutex) {
    config_ = config;
    mutex_ = i2cMutex;  // Use shared I2C mutex instead of creating internal one
    return mutex_ != nullptr;
}

void RTCDriver::setTimeFromGNSS(uint16_t year, uint8_t month, uint8_t day,
                                  uint8_t hourUTC, uint8_t minute, uint8_t second) {
    uint8_t hourLocal = hourUTC;
    utcToLocal(year, month, day, hourLocal, minute);

    xSemaphoreTake(mutex_, portMAX_DELAY);
    pcf_.init();
    pcf_.stopClock();
    pcf_.setYear(year % 100);
    pcf_.setMonth(month);
    pcf_.setDay(day);
    pcf_.setHour(hourLocal);
    pcf_.setMinut(minute);
    pcf_.setSecond(second);
    pcf_.startClock();
    xSemaphoreGive(mutex_);

    int8_t offsetH = config_.utcOffsetMinutes / 60;
    int8_t offsetM = abs(config_.utcOffsetMinutes % 60);
    LOG_PRINTF("RTCDriver: time set to %04u-%02u-%02u %02u:%02u:%02u (UTC%+d:%02d)\n",
                  year, month, day, hourLocal, minute, second, offsetH, offsetM);
}

RTCTime RTCDriver::getTime() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    Time t = pcf_.getTime();
    xSemaphoreGive(mutex_);
    RTCTime result;
    result.year = t.year + 2000;
    result.month = t.month;
    result.day = t.day;
    result.hour = t.hour;
    result.minute = t.minute;
    result.second = t.second;
    return result;
}

void RTCDriver::generateFilename(char* buffer, size_t bufSize, const char* extension) {
    RTCTime t = getTime();
    snprintf(buffer, bufSize, "%04u-%02u-%02u-%02u-%02u-%02u.%s",
             t.year, t.month, t.day, t.hour, t.minute, t.second, extension);
}

void RTCDriver::generateLogFilename(char* buffer, size_t bufSize) {
    RTCTime t = getTime();
    snprintf(buffer, bufSize, "%04u-%02u-%02u-%02u-%02u-%02u_log.txt",
             t.year, t.month, t.day, t.hour, t.minute, t.second);
}

uint64_t RTCDriver::getEpochMs(uint16_t extraMillis) {
    RTCTime now = getTime();
    struct tm t;
    t.tm_year = (now.year % 100) + 100;
    t.tm_mon  = now.month - 1;
    t.tm_mday = now.day;
    t.tm_hour = now.hour;
    t.tm_min  = now.minute;
    t.tm_sec  = now.second;
    t.tm_isdst = 0;
    time_t epoch = mktime(&t);
    return ((uint64_t)epoch * 1000) + extraMillis;
}

void RTCDriver::utcToLocal(uint16_t& year, uint8_t& month, uint8_t& day,
                            uint8_t& hour, uint8_t& minute) {
    int totalMinutes = hour * 60 + minute + config_.utcOffsetMinutes;

    auto daysInMonth = [](uint8_t m, uint16_t y) -> uint8_t {
        static const uint8_t days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
        if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) return 29;
        return days[m - 1];
    };

    // Handle negative offsets (west of UTC)
    while (totalMinutes < 0) {
        totalMinutes += 24 * 60;
        day--;
        if (day == 0) {
            month--;
            if (month == 0) {
                month = 12;
                year--;
            }
            day = daysInMonth(month, year);
        }
    }

    // Handle day overflow
    while (totalMinutes >= 24 * 60) {
        totalMinutes -= 24 * 60;
        day++;

        if (day > daysInMonth(month, year)) {
            day = 1;
            month++;
            if (month > 12) {
                month = 1;
                year++;
            }
        }
    }

    hour = totalMinutes / 60;
    minute = totalMinutes % 60;
}
