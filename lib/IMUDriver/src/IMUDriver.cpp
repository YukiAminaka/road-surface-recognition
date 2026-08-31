#include "IMUDriver.h"
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Arduino.h>
#include <SerialLog.h>

static BMI270 imu_;

bool IMUDriver::begin() {
    Config defaultConfig;
    return begin(defaultConfig);
}

bool IMUDriver::begin(const Config& config, SemaphoreHandle_t i2cMutex) {
    i2cMutex_ = i2cMutex;
    return begin(config);  // Wire.begin() is idempotent; safe in single-threaded setup()
}

bool IMUDriver::begin(const Config& config) {
    Wire.begin(config.sdaPin, config.sclPin);

    int result = imu_.beginI2C(config.i2cAddress, Wire);
    if (result != BMI2_OK) {
        LOG_PRINTF("IMUDriver: init failed (err=%d), retrying...\n", result);
        for (int i = 0; i < config.maxRetries; i++) {
            delay(config.retryDelayMs);
            result = imu_.beginI2C(config.i2cAddress, Wire);
            if (result == BMI2_OK) break;
        }
    }

    if (result != BMI2_OK) {
        LOG_PRINTLN("IMUDriver: initialization failed after retries");
        return false;
    }

    imu_.setAccelODR(config.accelODR);
    imu_.setGyroODR(config.gyroODR);

    LOG_PRINTLN("IMUDriver: initialized (BMI270)");
    return true;
}

bool IMUDriver::update() {
    if (i2cMutex_) {
        // Bounded timeout: RTC I2C operations take ~3ms max.
        // Skip this sample rather than block the 100Hz loop indefinitely.
        if (xSemaphoreTake(i2cMutex_, pdMS_TO_TICKS(5)) != pdTRUE) {
            return false;
        }
    }
    imu_.getSensorData();
    if (i2cMutex_) xSemaphoreGive(i2cMutex_);

    taskENTER_CRITICAL(&mutex_);
    data_.accelX = imu_.data.accelX;
    data_.accelY = imu_.data.accelY;
    data_.accelZ = imu_.data.accelZ;
    data_.gyroX = imu_.data.gyroX;
    data_.gyroY = imu_.data.gyroY;
    data_.gyroZ = imu_.data.gyroZ;
    taskEXIT_CRITICAL(&mutex_);
    return true;
}

IMUData IMUDriver::getData() const {
    taskENTER_CRITICAL(&mutex_);
    IMUData copy = data_;
    taskEXIT_CRITICAL(&mutex_);
    return copy;
}