#include "GNSSDriver.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include <HardwareSerial.h>
#include <SerialLog.h>
#include <Arduino.h>

static SFE_UBLOX_GNSS_SERIAL gnss_; // シリアル通信でu-blox GNSSモジュールと通信するためのオブジェクト
static HardwareSerial gnssSerial_(1);  // UART1

// u-blox UBX protocol returns coordinates in 1e-7 degrees
static constexpr double UBX_COORD_SCALE = 10000000.0;

// Shared state (protected by mutex_)
static portMUX_TYPE mutex_ = portMUX_INITIALIZER_UNLOCKED;
static GNSSData data_ = {};
static GNSSTime time_ = {};

// Auto PVT callback - called by checkCallbacks() when new NAV-PVT arrives
static void pvtCallback(UBX_NAV_PVT_data_t *pvt) {
    taskENTER_CRITICAL(&mutex_);
    data_.latitude  = pvt->lat / UBX_COORD_SCALE;
    data_.longitude = pvt->lon / UBX_COORD_SCALE;
    data_.altitude  = pvt->height;  // mm above ellipsoid (matches former getAltitude())
    data_.siv       = pvt->numSV;
    time_.year      = pvt->year;
    time_.month     = pvt->month;
    time_.day       = pvt->day;
    time_.hour      = pvt->hour;
    time_.minute    = pvt->min;
    time_.second    = pvt->sec;
    time_.millisecond = pvt->iTOW % 1000;  // matches getMillisecond() implementation
    time_.valid     = pvt->valid.bits.validDate && pvt->valid.bits.validTime;
    taskEXIT_CRITICAL(&mutex_);
}

bool GNSSDriver::begin(const Config& config) {
    config_ = config;
    gnssSerial_.begin(config.baudRate, SERIAL_8N1, config.rxPin, config.txPin);

    int retries = 0;
    while (!gnss_.begin(gnssSerial_)) {
        if (++retries > config.maxRetries) {
            LOG_PRINTF("GNSSDriver: init failed after %d retries\n", config.maxRetries);
            return false;
        }
        delay(config.retryDelayMs);
    }

    gnss_.setUART1Output(COM_TYPE_UBX);
    gnss_.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    gnss_.setMeasurementRate(config.measurementRateMs);

    if (config.enableL5) {
        gnss_.setVal8(UBLOX_CFG_SIGNAL_GPS_L5_ENA, 1);
        gnss_.setGPSL5HealthOverride(true);
    }

    gnss_.setLNAMode(SFE_UBLOX_LNA_MODE_NORMAL);
    gnss_.softwareResetGNSSOnly();

    // Register auto PVT callback (streaming mode)
    // Module will continuously output NAV-PVT at the configured measurement rate.
    // pvtCallback is invoked from checkCallbacks() when new data arrives.
    gnss_.setAutoPVTcallbackPtr(&pvtCallback);

    LOG_PRINTLN("GNSSDriver: initialized (callback mode)");
    return true;
}

void GNSSDriver::update() {
    gnss_.checkUblox();
    gnss_.checkCallbacks();
}

GNSSData GNSSDriver::getData() const {
    GNSSData d;
    taskENTER_CRITICAL(&mutex_);
    d = data_;
    taskEXIT_CRITICAL(&mutex_);
    return d;
}

GNSSTime GNSSDriver::getTime() const {
    GNSSTime t;
    taskENTER_CRITICAL(&mutex_);
    t = time_;
    taskEXIT_CRITICAL(&mutex_);
    return t;
}

bool GNSSDriver::waitForValidTime(unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        gnss_.checkUblox();
        gnss_.checkCallbacks();

        GNSSTime t = getTime();
        GNSSData d = getData();

        if (t.valid && d.siv >= config_.minSIV) {
            return true;
        }

        LOG_PRINTF("GNSSDriver: waiting for time... SIV: %u\n", d.siv);
        delay(config_.pollIntervalMs);
    }
    return false;
}