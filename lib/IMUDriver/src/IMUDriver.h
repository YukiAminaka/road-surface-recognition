#pragma once
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

struct IMUData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
};

class IMUDriver {
public:
    struct Config {
        uint8_t i2cAddress = 0x68;  // BMI2_I2C_PRIM_ADDR
        uint8_t accelODR = 0x09;    // BMI2_ACC_ODR_200HZ
        uint8_t gyroODR = 0x09;     // BMI2_GYR_ODR_200HZ
        int maxRetries = 5;
        unsigned long retryDelayMs = 500;
        int8_t sdaPin = 5;   // XIAO ESP32-S3 default
        int8_t sclPin = 6;   // XIAO ESP32-S3 default
    };

    bool begin();
    bool begin(const Config& config);

    /// Initialize with shared I2C bus mutex for multi-device bus protection.
    /// The mutex must outlive this driver instance.
    /// @note Use this overload exclusively — do not mix with begin(config).
    bool begin(const Config& config, SemaphoreHandle_t i2cMutex);

    /// Read latest sensor data (call from a single task)
    bool update();

    /// Get last read data (thread-safe)
    IMUData getData() const;

private:
    IMUData data_ = {};
    mutable portMUX_TYPE mutex_ = portMUX_INITIALIZER_UNLOCKED;
    SemaphoreHandle_t i2cMutex_ = nullptr;
};