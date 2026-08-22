#pragma once

struct GNSSData {
    double latitude;
    double longitude;
    long altitude;    // mm
    uint8_t siv;      // Number of satellites in view
};

struct GNSSTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;  // 0-999, from UBX NAV-PVT nano field
    bool valid;
};

class GNSSDriver {
public:
    struct Config {
        int8_t rxPin;
        int8_t txPin;
        uint32_t baudRate = 38400;
        uint16_t measurementRateMs = 50;  // 20Hz
        bool enableL5 = true;
        int maxRetries = 30;
        uint8_t minSIV = 4;
        unsigned long retryDelayMs = 1000;
        unsigned long pollIntervalMs = 500;
    };

    bool begin(const Config& config);

    /// Call in GNSS task loop to parse incoming UART data and dispatch callbacks
    void update();

    /// Thread-safe read of latest GNSS position data
    GNSSData getData() const;

    /// Thread-safe read of latest GNSS time
    GNSSTime getTime() const;

    /// Wait for valid GNSS time with timeout (blocking, use in setup only)
    bool waitForValidTime(unsigned long timeoutMs = 300000);

private:
    Config config_ = {};    
};