#pragma once
#include <cstdint>
#include <cstddef>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Binary sensor data for cloud transmission (60 bytes, packed)
#pragma pack(push, 1)
struct SensorData {
    uint32_t data_number;
    uint64_t timestamp_ms;
    double   lat;
    double   lon;
    int32_t  alt;             // mm
    float    accx, accy, accz;
    float    gyrox, gyroy, gyroz;
    uint8_t  status;
    uint8_t  siv;
    char     reserved[2];
};
#pragma pack(pop)
static_assert(sizeof(SensorData) == 60, "SensorData must be 60 bytes");

// Binary record for SD storage (56 bytes, naturally aligned)
struct BinRecord {
    uint64_t timestamp_ms;  // 8
    double   lat, lon;      // 16
    int32_t  alt;           // 4  (mm)
    float    ax, ay, az;    // 12
    float    gx, gy, gz;    // 12
    uint8_t  siv;           // 1
    uint8_t  _pad[3];       // 3
};
static_assert(sizeof(BinRecord) == 56, "BinRecord must be 56 bytes");

// Binary file header (8 bytes)
struct BinFileHeader {
    uint8_t  magic[4];   // {'B','I','N','1'}
    uint16_t version;    // 1
    uint16_t reserved;   // 0
};
static_assert(sizeof(BinFileHeader) == 8, "BinFileHeader must be 8 bytes");

class SensorDataPipeline {
public:
    struct Config {
        size_t cloudQueueSize = 10000;   // items in cloud queue (PSRAM)
        size_t sdQueueSize = 300;        // items in SD queue
        size_t sdItemSize = 200;         // bytes per SD CSV line
        size_t batchSize = 24;           // items per cloud batch
        size_t sdFlushThreshold = 100;   // trigger SD write at this count
    };

    bool begin();
    bool begin(const Config& config);

    /// Push sensor data to cloud queue (PSRAM-backed, non-blocking)
    /// Returns true if queued, false if queue full (dropped)
    bool pushCloudData(const SensorData& data);

    /// Push data item to SD queue (non-blocking)
    bool pushSDData(const void* data);

    /// Receive one item from cloud queue (blocking)
    bool popCloudData(SensorData& data, TickType_t timeout = portMAX_DELAY);

    /// Get number of items waiting in cloud queue
    size_t cloudQueueCount() const;

    /// Get number of items waiting in SD queue
    size_t sdQueueCount() const;

    /// Check if SD queue reached flush threshold
    bool sdFlushReady() const;

    /// Receive one data item from SD queue (blocking)
    /// Caller must provide buffer of at least config().sdItemSize bytes
    bool popSDData(void* buffer, TickType_t timeout = portMAX_DELAY);

    /// Get config
    const Config& config() const { return config_; }

    /// Stats
    unsigned long cloudDropCount() const { return cloudDropCount_; }
    size_t batchPayloadSize() const { return sizeof(SensorData) * config_.batchSize; }

private:
    Config config_;
    QueueHandle_t cloudQueue_ = nullptr;
    QueueHandle_t sdQueue_ = nullptr;

    // Static queue for PSRAM
    StaticQueue_t cloudQueueStruct_;
    uint8_t* cloudQueueBuffer_ = nullptr;

    StaticQueue_t sdQueueStruct_;
    uint8_t* sdQueueBuffer_ = nullptr;

    unsigned long cloudDropCount_ = 0;
};
