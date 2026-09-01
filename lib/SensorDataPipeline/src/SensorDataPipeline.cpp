#include "SensorDataPipeline.h"
#include <Arduino.h>
#include <SerialLog.h>

bool SensorDataPipeline::begin() {
    Config defaultConfig;
    return begin(defaultConfig);
}

bool SensorDataPipeline::begin(const Config& config) {
    config_ = config;

    // Check PSRAM
    if (ESP.getPsramSize() == 0) {
        LOG_PRINTLN("SensorDataPipeline: FATAL - No PSRAM detected!");
        return false;
    }
    LOG_PRINTF("SensorDataPipeline: PSRAM size: %u bytes, Free: %u bytes\n",
                  ESP.getPsramSize(), ESP.getFreePsram());

    // Allocate cloud queue in PSRAM
    cloudQueueBuffer_ = (uint8_t*)ps_malloc(config_.cloudQueueSize * sizeof(SensorData));
    if (!cloudQueueBuffer_) {
        LOG_PRINTF("SensorDataPipeline: FATAL - PSRAM alloc failed! Requested: %u bytes\n",
                      (unsigned)(config_.cloudQueueSize * sizeof(SensorData)));
        return false;
    }

    cloudQueue_ = xQueueCreateStatic(config_.cloudQueueSize, sizeof(SensorData),
                                      cloudQueueBuffer_, &cloudQueueStruct_);
    if (!cloudQueue_) {
        LOG_PRINTLN("SensorDataPipeline: FATAL - cloud queue creation failed");
        return false;
    }

    // Allocate SD queue in PSRAM
    sdQueueBuffer_ = (uint8_t*)ps_malloc(config_.sdQueueSize * config_.sdItemSize);
    if (!sdQueueBuffer_) {
        LOG_PRINTF("SensorDataPipeline: FATAL - PSRAM alloc failed for SD queue! Requested: %u bytes\n",
                      (unsigned)(config_.sdQueueSize * config_.sdItemSize));
        return false;
    }

    sdQueue_ = xQueueCreateStatic(config_.sdQueueSize, config_.sdItemSize,
                                   sdQueueBuffer_, &sdQueueStruct_);
    if (!sdQueue_) {
        LOG_PRINTLN("SensorDataPipeline: FATAL - SD queue creation failed");
        return false;
    }

    LOG_PRINTF("SensorDataPipeline: initialized. Free heap: %u bytes\n", ESP.getFreeHeap());
    return true;
}

bool SensorDataPipeline::pushCloudData(const SensorData& data) {
    if (xQueueSend(cloudQueue_, &data, (TickType_t)0) != pdTRUE) {
        cloudDropCount_++;
        if (cloudDropCount_ % 100 == 1) {
            LOG_PRINTF("SensorDataPipeline: cloud queue full, dropped %lu packets\n", cloudDropCount_);
        }
        return false;
    }
    return true;
}

bool SensorDataPipeline::pushSDData(const void* data) {
    return xQueueSend(sdQueue_, data, (TickType_t)0) == pdTRUE;
}

bool SensorDataPipeline::popCloudData(SensorData& data, TickType_t timeout) {
    return xQueueReceive(cloudQueue_, &data, timeout) == pdTRUE;
}

size_t SensorDataPipeline::cloudQueueCount() const {
    return uxQueueMessagesWaiting(cloudQueue_);
}

size_t SensorDataPipeline::sdQueueCount() const {
    return uxQueueMessagesWaiting(sdQueue_);
}

bool SensorDataPipeline::sdFlushReady() const {
    return uxQueueMessagesWaiting(sdQueue_) >= config_.sdFlushThreshold;
}

bool SensorDataPipeline::popSDData(void* buffer, TickType_t timeout) {
    return xQueueReceive(sdQueue_, buffer, timeout) == pdTRUE;
}
