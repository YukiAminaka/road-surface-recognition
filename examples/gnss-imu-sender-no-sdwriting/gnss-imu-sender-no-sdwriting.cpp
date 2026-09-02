/**************************************************
 * GNSS + IMU Sensor Data Sender (ESP32-S3)
 * ----------------------------------------
 * GNSS (NEO-F10N) 20Hz + IMU (BMI270) 200Hz
 * Cat-M1 UDP cloud send only (SD card 書き込みなし)
 *
 * gnss-imu-sender から SD カード CSV ロギングを取り除いた構成。
 * Core 0 は送信タスク (sendDataTask) のみを実行する。
 *
 **************************************************/

#include <Arduino.h>
#include <cmath>
#include <ctime>
#include <GNSSDriver.h>
#include <IMUDriver.h>
#include <RTCDriver.h>
#include <CatM1Modem.h>
#include <SensorDataPipeline.h>
#include <SerialLog.h>

// ---- Pin Configuration (XIAO ESP32-S3) ----
static constexpr int8_t GNSS_RX_PIN = 1;   // D0 = GPIO1
static constexpr int8_t GNSS_TX_PIN = 2;   // D1 = GPIO2
static constexpr int8_t MODEM_RX_PIN = 44;  // D7 = GPIO44
static constexpr int8_t MODEM_TX_PIN = 43;  // D6 = GPIO43

// ---- Operation Parameters ----
static constexpr int CLOUD_SEND_DIVISOR = 10;   // 200Hz / 10 = 20Hz cloud send
static constexpr int BATCH_SIZE = 16;  // 16 * 60 = 960 bytes (Cat-M1 UDP上限 ~1024 以内)
static constexpr unsigned long TIME_RESYNC_INTERVAL_MS = 10UL * 60 * 1000;  // 10 min
static constexpr float MOTION_THRESHOLD = 0.3f;       // m/s² - minimum accel magnitude delta to count as motion
static constexpr unsigned long IDLE_TIMEOUT_MS = 60000; // 10 minutes - time without motion before going idle
static constexpr uint8_t GNSS_MIN_SIV = 3;                    // minimum satellites required before proceeding
static constexpr unsigned long GNSS_FIX_TIMEOUT_MS = 300000;  // 5 min - restart if no fix
static constexpr float GNSS_MOVE_THRESHOLD_M = 15.0f;              // meters - GNSS position change to count as movement
static constexpr unsigned long GNSS_MOVE_CHECK_INTERVAL_MS = 5000; // check GNSS movement every 5 seconds

// ---- Driver Instances ----
static GNSSDriver gnss;
static IMUDriver imu;
static RTCDriver rtc;
static CatM1Modem modem;
static SensorDataPipeline pipeline;

// ---- State ----
static unsigned long dataNumber = 0;
static portMUX_TYPE timeBaseMutex = portMUX_INITIALIZER_UNLOCKED;
static uint64_t baseEpochMs = 0;
static unsigned long baseMillis = 0;
static float prevAccelMag = 0.0f;
static unsigned long lastMotionTimeMs = 0;
static bool isIdle = false;
static double prevGnssLat = 0.0;
static double prevGnssLon = 0.0;
static unsigned long lastGnssCheckMs = 0;
static bool gnssPositionInitialized = false;
static bool imuAvailable = false;
static SemaphoreHandle_t modemMutex = NULL;
static SemaphoreHandle_t i2cMutex = NULL;     // Wire (I2C) バス排他制御: IMU ↔ RTC

// Convert UTC date/time to Unix epoch (seconds since 1970-01-01 00:00:00 UTC).
// Pure arithmetic, no timezone dependency (unlike mktime which uses TZ setting).
static time_t utcToEpoch(int y, int m, int d, int h, int min, int sec) {
    if (m <= 2) { y--; m += 12; }
    int era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m - 3) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = (long)era * 146097L + (long)doe - 719468L;
    return days * 86400L + h * 3600L + min * 60L + sec;
}

// ---- Task: IMU + Queue Push (200Hz, Core 1) ----
void getImuTask(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(5);  // 5ms = 200Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    lastMotionTimeMs = millis();

    while (1) {
        IMUData imuData = {};
        if (imuAvailable) {
            imu.update();
            imuData = imu.getData();
        }

        uint64_t localBaseEpoch;
        unsigned long localBaseMillis;
        taskENTER_CRITICAL(&timeBaseMutex);
        localBaseEpoch = baseEpochMs;
        localBaseMillis = baseMillis;
        taskEXIT_CRITICAL(&timeBaseMutex);

        unsigned long currentMillis = millis();
        uint64_t currentEpochMs = localBaseEpoch + (currentMillis - localBaseMillis);

        // Motion detection - IMU (skip when IMU unavailable - accel values are always zero)
        if (imuAvailable) {
            float mag = sqrtf(imuData.accelX * imuData.accelX +
                              imuData.accelY * imuData.accelY +
                              imuData.accelZ * imuData.accelZ);
            float delta = fabsf(mag - prevAccelMag);
            prevAccelMag = mag;
            if (delta > MOTION_THRESHOLD) {
                lastMotionTimeMs = millis();
            }
        }

        // Motion detection - GNSS position change
        GNSSData gnssData = gnss.getData();  // thread-safe
        if (gnssData.siv >= GNSS_MIN_SIV) {
            unsigned long now = millis();
            if (now - lastGnssCheckMs >= GNSS_MOVE_CHECK_INTERVAL_MS) {
                lastGnssCheckMs = now;
                if (gnssPositionInitialized) {
                    double dLat = (gnssData.latitude - prevGnssLat) * 111139.0;
                    double dLon = (gnssData.longitude - prevGnssLon) * 111139.0 * cos(gnssData.latitude * M_PI / 180.0);
                    float dist = sqrtf((float)(dLat * dLat + dLon * dLon));
                    if (dist > GNSS_MOVE_THRESHOLD_M) {
                        lastMotionTimeMs = millis();
                    }
                }
                prevGnssLat = gnssData.latitude;
                prevGnssLon = gnssData.longitude;
                gnssPositionInitialized = true;
            }
        }

        // Idle state evaluation (considers both IMU and GNSS motion)
        {
            bool wasIdle = isIdle;
            isIdle = (millis() - lastMotionTimeMs > IDLE_TIMEOUT_MS);
            if (!wasIdle && isIdle) {
                Serial.println("IDLE: No motion detected, pausing sending");
            } else if (wasIdle && !isIdle) {
                Serial.println("ACTIVE: Motion detected, resuming sending");
            }
        }

        if (!isIdle) {
            // Push to cloud queue (every 10th cycle @ 20Hz)
            if (dataNumber % CLOUD_SEND_DIVISOR == 0) {
                SensorData packet;
                packet.data_number  = (uint32_t)dataNumber;
                packet.timestamp_ms = currentEpochMs;
                packet.lat          = gnssData.latitude;
                packet.lon          = gnssData.longitude;
                packet.alt          = (int32_t)gnssData.altitude;
                packet.accx         = imuData.accelX;
                packet.accy         = imuData.accelY;
                packet.accz         = imuData.accelZ;
                packet.gyrox        = imuData.gyroX;
                packet.gyroy        = imuData.gyroY;
                packet.gyroz        = imuData.gyroZ;
                packet.status       = 0x01;
                packet.siv          = gnssData.siv;
                packet.reserved[0]  = 0;
                packet.reserved[1]  = 0;

                pipeline.pushCloudData(packet);
            }
        }

        dataNumber++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ---- Task: GNSS Position Update (20Hz, Core 1) ----
void getGnssTask(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50ms = 20Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long lastResyncMs = millis();

    while (1) {
        gnss.update();

        // Periodic time re-sync: GNSS → baseEpoch/baseMillis
        if (millis() - lastResyncMs >= TIME_RESYNC_INTERVAL_MS) {
            GNSSTime t = gnss.getTime();
            if (t.valid) {
                unsigned long newMillis = millis();  // capture millis close to GNSS read

                // Compute epoch directly from GNSS UTC (no RTC round-trip delay)
                time_t epoch = utcToEpoch(t.year, t.month, t.day,
                                          t.hour, t.minute, t.second);
                uint64_t newEpoch = (uint64_t)epoch * 1000 + t.millisecond;

                taskENTER_CRITICAL(&timeBaseMutex);
                baseEpochMs = newEpoch;
                baseMillis = newMillis;
                taskEXIT_CRITICAL(&timeBaseMutex);

                // Write to RTC as backup only
                rtc.setTimeFromGNSS(t.year, t.month, t.day,
                                    t.hour, t.minute, t.second);

                lastResyncMs = millis();
                Serial.println("Time re-synced from GNSS");
            } else {
                // GNSS unavailable — fallback to RTC
                unsigned long newMillis = millis();
                uint64_t rtcEpoch = rtc.getEpochMs(0);

                taskENTER_CRITICAL(&timeBaseMutex);
                baseEpochMs = rtcEpoch;
                baseMillis = newMillis;
                taskEXIT_CRITICAL(&timeBaseMutex);

                Serial.println("WARNING: GNSS time invalid, re-synced from RTC");
                // Don't update lastResyncMs — retry GNSS next cycle
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ---- Helper: Handle batch send result ----
static void handleBatchSendResult(uint8_t* batchBuffer, size_t payloadSize, int& requeueCount) {
    xSemaphoreTake(modemMutex, portMAX_DELAY);
    CatM1Modem::SendResult result = modem.sendWithRetry(batchBuffer, payloadSize);
    xSemaphoreGive(modemMutex);

    if (result != CatM1Modem::SendResult::FAILED) {
        Serial.println("Send Success!");
        requeueCount = 0;
        return;
    }

    requeueCount++;
    if (requeueCount >= 3) {
        Serial.println("Max requeue cycles. Dropping batch & pausing 60s...");
        vTaskDelay(pdMS_TO_TICKS(60000));
        requeueCount = 0;
        return;
    }

    Serial.printf("Re-queuing data... (requeue #%d)\n", requeueCount);
    SensorData* requeuePtr = (SensorData*)batchBuffer;
    for (int i = 0; i < BATCH_SIZE; i++) {
        pipeline.pushCloudData(requeuePtr[i]);
    }
}

// ---- Task: Cloud Data Send (Core 0, 唯一の Core 0 タスク) ----
void sendDataTask(void* pvParameters) {
    const size_t BATCH_PAYLOAD_SIZE = pipeline.batchPayloadSize();
    static alignas(SensorData) uint8_t batchBuffer[sizeof(SensorData) * BATCH_SIZE];

    SensorData itemBuf;
    int itemsCollected = 0;
    int requeueCount = 0;

    while (1) {
        if (!pipeline.popCloudData(itemBuf)) continue;

        memcpy(&batchBuffer[itemsCollected * sizeof(SensorData)],
               &itemBuf, sizeof(SensorData));
        itemsCollected++;

        if (itemsCollected < BATCH_SIZE) continue;

        handleBatchSendResult(batchBuffer, BATCH_PAYLOAD_SIZE, requeueCount);
        itemsCollected = 0;
    }
}

void setup() {
    Serial.begin(115200);
    SerialLog::init();
    delay(2000);

    setenv("TZ", "JST-9", 1);
    tzset();

    // Create shared I2C bus mutex before initializing I2C peripherals (IMU, RTC)
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("FATAL: I2C mutex creation failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // Initialize RTC
    if (!rtc.begin(RTCDriver::Config{}, i2cMutex)) {
        Serial.println("FATAL: RTC init failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // Initialize GNSS
    GNSSDriver::Config gnssConfig;
    gnssConfig.rxPin = GNSS_RX_PIN;
    gnssConfig.txPin = GNSS_TX_PIN;
    gnssConfig.minSIV = GNSS_MIN_SIV;
    if (!gnss.begin(gnssConfig)) {
        Serial.println("FATAL: GNSS init failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // Initialize Cat-M1 Modem
    CatM1Modem::Config modemConfig;
    modemConfig.rxPin = MODEM_RX_PIN;
    modemConfig.txPin = MODEM_TX_PIN;
    modem.begin(modemConfig);

    // Initialize IMU with shared I2C mutex
    imuAvailable = imu.begin(IMUDriver::Config{}, i2cMutex);
    if (!imuAvailable) {
        Serial.println("WARNING: IMU init failed - continuing with GNSS only");
    }

    // Wait for GNSS satellite fix (>= GNSS_MIN_SIV satellites) and valid time.
    // Block here until fix is acquired; restart if timeout expires.
    Serial.printf("Waiting for GNSS fix (>= %u satellites)...\n", GNSS_MIN_SIV);
    if (!gnss.waitForValidTime(GNSS_FIX_TIMEOUT_MS)) {
        Serial.println("FATAL: GNSS fix not acquired within timeout, restarting");
        ESP.restart();
    }

    {
        GNSSTime gnssTime = gnss.getTime();
        baseMillis = millis();  // capture millis as close to GNSS read as possible

        // Compute epoch directly from GNSS UTC (no RTC round-trip delay)
        time_t epoch = utcToEpoch(gnssTime.year, gnssTime.month, gnssTime.day,
                                  gnssTime.hour, gnssTime.minute, gnssTime.second);
        baseEpochMs = (uint64_t)epoch * 1000 + gnssTime.millisecond;

        // Write to RTC as backup only
        rtc.setTimeFromGNSS(gnssTime.year, gnssTime.month, gnssTime.day,
                            gnssTime.hour, gnssTime.minute, gnssTime.second);
    }
    Serial.printf("Time base recorded: epoch=%llu, millis=%lu\n", baseEpochMs, baseMillis);

    // Create modem access mutex (protects modem UART access)
    modemMutex = xSemaphoreCreateMutex();
    if (modemMutex == NULL) {
        Serial.println("FATAL: Failed to create modem mutex");
        ESP.restart();
    }

    // Initialize modem (with retries)
    bool modemOk = false;
    for (int retry = 0; retry < 3 && !modemOk; retry++) {
        modemOk = modem.init();
        if (!modemOk) {
            Serial.printf("Modem init failed (attempt %d/3)\n", retry + 1);
            delay(5000);
        }
    }
    if (!modemOk) {
        Serial.println("WARNING: Modem init failed - will retry in sendDataTask");
    }

    // Initialize data pipeline (PSRAM-backed cloud queue)
    SensorDataPipeline::Config pipeConfig;
    pipeConfig.batchSize = BATCH_SIZE;
    if (!pipeline.begin(pipeConfig)) {
        Serial.println("FATAL: Pipeline init failed");
        ESP.restart();
    }

    // Create FreeRTOS tasks
    // Core 1: センサー系 (IMU / GNSS)、Core 0: 送信のみ
    xTaskCreateUniversal(getImuTask,   "getImuTask",   8192, NULL, 3, NULL, 1);
    xTaskCreateUniversal(getGnssTask,  "getGnssTask",  8192, NULL, 2, NULL, 1);
    xTaskCreateUniversal(sendDataTask, "sendDataTask", 8192, NULL, 1, NULL, 0);

    Serial.println("System started. All tasks running.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1));
}
