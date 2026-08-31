#pragma once
#include <cstdint>
#include <cstddef>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct RTCTime {
    uint16_t year;    // full year (e.g. 2025)
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

class RTCDriver {
public:
    struct Config {
        int16_t utcOffsetMinutes = 540;  // UTC+9 (JST) default
    };

    bool begin();
    bool begin(const Config& config);

    /// Initialize with shared I2C bus mutex for multi-device bus protection.
    /// The mutex must outlive this driver instance.
    /// @note Use this overload exclusively — do not mix with begin(config).
    bool begin(const Config& config, SemaphoreHandle_t i2cMutex);

    /// Set RTC from GNSS UTC time, converts to local time using configured offset
    void setTimeFromGNSS(uint16_t year, uint8_t month, uint8_t day,
                         uint8_t hourUTC, uint8_t minute, uint8_t second);

    /// Get current time from RTC (returns local time)
    RTCTime getTime();

    /// Generate timestamp-based filename in format "YYYY-MM-DD-HH-MM-SS.csv"
    /// Buffer must be at least 30 bytes
    void generateFilename(char* buffer, size_t bufSize, const char* extension = "csv");

    /// Generate timestamp-based log filename "YYYY-MM-DD-HH-MM-SS_log.txt"
    void generateLogFilename(char* buffer, size_t bufSize);

    /// Convert current RTC time to epoch milliseconds
    /// @param extraMillis additional milliseconds to add (e.g. millis() % 1000)
    uint64_t getEpochMs(uint16_t extraMillis = 0);

private:
    /// Convert UTC to local time using configured offset, handling day/month/year rollover
    void utcToLocal(uint16_t& year, uint8_t& month, uint8_t& day,
                    uint8_t& hour, uint8_t& minute);
    Config config_;
    SemaphoreHandle_t mutex_ = nullptr;
};
