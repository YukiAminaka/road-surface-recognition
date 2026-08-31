#include "CatM1Modem.h"
#include "CatM1Modem_Internal.h"
#include <SerialLog.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// MUST live in exactly one TU. Duplicating in another *.cpp would create two
// distinct HardwareSerial(2) objects with internal linkage (no ODR violation,
// no linker error), silently corrupting UART2 state. Keep here only.
static HardwareSerial modemSerial_(2);

bool CatM1Modem::begin(const Config& config) {
    config_ = config;
    serial_ = &modemSerial_;
    serial_->begin(config.baudRate, SERIAL_8N1, config.rxPin, config.txPin);
    LOG_PRINTLN("CatM1Modem: serial initialized");
    return true;
}

bool CatM1Modem::waitForReady(unsigned long timeoutMs) {
    // Phase 1: Passive wait — monitor UART for ALL boot URCs to finish.
    // SIM7080G sends URCs in sequence: RDY → +CFUN: 1 → +CPIN: READY → SMS DONE
    // Only after the LAST URC is the modem ready for AT commands.
    // We wait until the modem has been quiet for BOOT_QUIET_MS after the last byte.
    static constexpr unsigned long BOOT_QUIET_MS = 5000;  // 5s silence = boot complete

    unsigned long start = millis();
    unsigned long lastDataTime = start;
    bool dataDetected = false;

    while (millis() - start < timeoutMs) {
        if (serial_->available()) {
            if (!dataDetected) {
                dataDetected = true;
                LOG_PRINTLN("CatM1Modem: boot activity detected");
            }
            // Drain all available bytes at once to keep up with baud rate
            while (serial_->available()) {
                serial_->read();
            }
            lastDataTime = millis();
        } else if (dataDetected && (millis() - lastDataTime >= BOOT_QUIET_MS)) {
            // Modem has been quiet for BOOT_QUIET_MS after last URC — boot complete
            LOG_PRINTF("CatM1Modem: boot URCs finished (%lums quiet)\n", BOOT_QUIET_MS);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(dataDetected ? 1 : 50));
    }

    if (!dataDetected) {
        LOG_PRINTF("CatM1Modem: no boot activity in %lu ms\n", timeoutMs);
    }

    flushRx();

    // Phase 2: Fast AT probe (TinyGSM-style).
    // After boot URCs finish, modem should respond to AT within milliseconds.
    for (unsigned long probeStart = millis(); millis() - probeStart < AT_PROBE_TIMEOUT_MS; ) {
        if (sendATCommand("AT\r\n", AT_PROBE_RESP_MS)) {
            LOG_PRINTLN("CatM1Modem: modem responsive");
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(AT_PROBE_INTERVAL_MS));
    }

    // If initial baudRate failed, try targetBaudRate (NVM persistence case)
    if (config_.targetBaudRate != 0 && config_.targetBaudRate != config_.baudRate) {
        LOG_PRINTLN("CatM1Modem: waitForReady — no response at initial baud, trying target baud rate");
        serial_->updateBaudRate(config_.targetBaudRate);
        flushRx();
        for (int i = 0; i < NVM_BAUD_RETRIES; i++) {
            if (sendATCommand("AT\r\n", AT_PROBE_RESP_MS)) {
                LOG_PRINTLN("CatM1Modem: waitForReady — modem responded at target baud rate (NVM persisted)");
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(AT_PROBE_INTERVAL_MS));
        }
        // Restore UART to initial baud if target probe also failed
        serial_->updateBaudRate(config_.baudRate);
    }

    LOG_PRINTLN("CatM1Modem: modem not responsive after boot wait");
    return false;
}

bool CatM1Modem::init() {
    // Validate HTTP profile bounds. Fall back to NORMAL on invalid config so
    // adaptive tuning still operates within a sane range.
    {
        int initIdx = (int)config_.httpInitialProfile;
        int fastIdx = (int)config_.httpFastestProfile;
        int slowIdx = (int)config_.httpSlowestProfile;
        bool valid = (fastIdx <= slowIdx)
                  && (initIdx >= fastIdx)
                  && (initIdx <= slowIdx)
                  && (slowIdx < (int)HTTP_PROFILE_COUNT);
        if (!valid) {
            LOG_PRINTF("CatM1Modem: WARNING invalid HTTP profile config "
                          "(init=%d, fast=%d, slow=%d), falling back to NORMAL\n",
                          initIdx, fastIdx, slowIdx);
            config_.httpInitialProfile = HttpProfile::NORMAL;
            config_.httpFastestProfile = HttpProfile::FAST;
            config_.httpSlowestProfile = HttpProfile::SAFEST;
        }
        currentProfile_ = config_.httpInitialProfile;
        successStreak_  = 0;
        LOG_PRINTF("CatM1Modem: HTTP profile initial=%s (fastest=%s, slowest=%s)\n",
                      HTTP_PROFILES[(int)config_.httpInitialProfile].name,
                      HTTP_PROFILES[(int)config_.httpFastestProfile].name,
                      HTTP_PROFILES[(int)config_.httpSlowestProfile].name);
    }

    // Phase 1: Fast AT probe (TinyGSM-style: 200ms response + 100ms interval)
    bool modemReady = false;
    bool alreadyAtTarget = false;
    for (unsigned long probeStart = millis(); millis() - probeStart < AT_PROBE_TIMEOUT_MS; ) {
        if (sendATCommand("AT\r\n", AT_PROBE_RESP_MS)) {
            modemReady = true;
            break;
        }
        delay(AT_PROBE_INTERVAL_MS);
    }

    // If initial baudRate failed, try targetBaudRate (NVM persistence case)
    if (!modemReady && config_.targetBaudRate != 0 && config_.targetBaudRate != config_.baudRate) {
        LOG_PRINTLN("CatM1Modem: init — no response at initial baud, trying target baud rate");
        serial_->updateBaudRate(config_.targetBaudRate);
        for (int i = 0; i < NVM_BAUD_RETRIES; i++) {
            if (sendATCommand("AT\r\n", AT_PROBE_RESP_MS)) {
                modemReady = true;
                alreadyAtTarget = true;
                LOG_PRINTLN("CatM1Modem: init — modem responded at target baud rate (NVM persisted)");
                break;
            }
            delay(AT_PROBE_INTERVAL_MS);
        }
    }

    if (!modemReady) {
        // Restore UART to initial baud if we switched during NVM probe
        if (config_.targetBaudRate != 0 && config_.targetBaudRate != config_.baudRate) {
            serial_->updateBaudRate(config_.baudRate);
        }
        LOG_PRINTLN("CatM1Modem: modem not responding");
        return false;
    }

    // Disable echo
    vTaskDelay(pdMS_TO_TICKS(500));
    for (int i = 0; i < 3; i++) {
        if (sendATCommand("ATE0\r\n", TIMEOUT_AT_BASIC_MS)) break;
        delay(500);
    }

    // Phase 1: PDP deactivate + PDP context definition (legacy order)
    // Legacy does CNACT=0,0 → CGDCONT → CFUN=1 → CGATT=1
    sendATCommand("AT+CNACT=0,0\r\n", TIMEOUT_AT_SIM_MS);  // ERROR OK if already inactive
    {
        char cgdcontCmd[CMD_BUF_SIZE];
        snprintf(cgdcontCmd, sizeof(cgdcontCmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", config_.apn);
        sendATCommand(cgdcontCmd, TIMEOUT_AT_SIM_MS);
    }
    sendATCommand("AT+CGDCONT?\r\n", TIMEOUT_AT_BASIC_MS);

    // Phase 2: Radio on → attach (legacy order)
    configureCatM1Mode();

    if (!sendATCommand("AT+CFUN=1\r\n", TIMEOUT_AT_CFUN_MS)) {
        LOG_PRINTLN("CatM1Modem: WARNING AT+CFUN=1 failed, continuing");
    }

    if (!sendATCommand("AT+CGATT=1\r\n", TIMEOUT_AT_NETWORK_MS)) {
        LOG_PRINTLN("CatM1Modem: WARNING AT+CGATT=1 failed, continuing");
    }
    sendATCommand("AT+CGATT?\r\n", TIMEOUT_AT_BASIC_MS);

    // 3GPP PDP context activate (legacy uses this)
    sendATCommand("AT+CGACT=1,1\r\n", TIMEOUT_AT_NETWORK_MS);

    // SIM/signal diagnostics (legacy order: after attach)
    sendATCommand("AT+CPIN?\r\n", TIMEOUT_AT_SIM_MS);
    sendATCommand("AT+CSQ\r\n", TIMEOUT_AT_SIM_MS);
    sendATCommand("AT+COPS?\r\n", TIMEOUT_AT_BASIC_MS);
    sendATCommand("AT+CGNAPN\r\n", TIMEOUT_AT_BASIC_MS);

    // URC configuration for debug visibility
    sendATCommand("AT+CEREG=2\r\n", TIMEOUT_AT_BASIC_MS);
    sendATCommand("AT+CNSMOD=1\r\n", TIMEOUT_AT_BASIC_MS);

    // Wait for network registration (CEREG stat=1 or 5)
    if (!waitForRegistration(30, 2000)) {
        LOG_PRINTLN("CatM1Modem: WARNING registration timeout, continuing to PDP");
    }

    // Phase 3: APN config + PDP activation

    char apnCmd[CMD_BUF_SIZE];
    int apnLen = snprintf(apnCmd, sizeof(apnCmd), "AT+CNCFG=0,1,\"%s\",\"%s\",\"%s\",%u\r\n",
             config_.apn, config_.user, config_.pass, config_.authType);
    if (apnLen < 0 || apnLen >= (int)sizeof(apnCmd)) {
        LOG_PRINTLN("CatM1Modem: APN command too long, truncated");
        return false;
    }
    if (!sendATCommand(apnCmd, TIMEOUT_AT_SIM_MS)) {
        LOG_PRINTLN("CatM1Modem: APN config failed");
        return false;
    }
    sendATCommand("AT+CNCFG?\r\n", TIMEOUT_AT_BASIC_MS);

    if (!activatePdp()) {
        LOG_PRINTLN("CatM1Modem: PDP activation failed");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));  // PDP stabilization delay

    // Phase 4: Open socket (with result code verification)
    if (!openSocket()) {
        LOG_PRINTLN("CatM1Modem: socket open failed");
        return false;
    }

    // Phase 5: Negotiate higher baud rate for faster UART transfers
    // Skip if modem already responded at targetBaudRate (NVM-persisted baud)
    if (!alreadyAtTarget) {
        if (!negotiateBaudRate()) {
            LOG_PRINTLN("CatM1Modem: WARNING baud negotiation failed, continuing at default baud rate");
        }
    } else {
        LOG_PRINTLN("CatM1Modem: skipping baud negotiation — already at target baud rate");
    }

    LOG_PRINTLN("CatM1Modem: initialized successfully");
    return true;
}

void CatM1Modem::configureCatM1Mode() {
    sendATCommand("AT+CMEE=2\r\n", TIMEOUT_AT_BASIC_MS);   // verbose error reports
    sendATCommand("AT+CNMP=38\r\n", TIMEOUT_AT_BASIC_MS);  // LTE only
    sendATCommand("AT+CMNB=1\r\n", TIMEOUT_AT_BASIC_MS);   // CAT-M only (no NB-IoT)
    char bandCmd[64];
    int bandLen = snprintf(bandCmd, sizeof(bandCmd), "AT+CBANDCFG=\"CAT-M\",%s\r\n", config_.catmBands);
    if (bandLen >= 0 && bandLen < (int)sizeof(bandCmd)) {
        sendATCommand(bandCmd, TIMEOUT_AT_BASIC_MS);
    } else {
        LOG_PRINTLN("CatM1Modem: CBANDCFG command too long, skipped");
    }
}

void CatM1Modem::flushRx() {
    unsigned long idle = millis();
    unsigned long hard = millis();
    while ((millis() - idle < FLUSH_IDLE_MS) && (millis() - hard < FLUSH_MAX_MS)) {
        if (serial_->available()) {
            serial_->read();
            idle = millis();  // reset idle timer on each byte
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void CatM1Modem::flushRxDeep() {
    unsigned long idle = millis();
    unsigned long hard = millis();
    while ((millis() - idle < FLUSH_DEEP_IDLE_MS) && (millis() - hard < FLUSH_DEEP_MAX_MS)) {
        if (serial_->available()) {
            serial_->read();
            idle = millis();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

bool CatM1Modem::sendATCommand(const char* command, unsigned long timeoutMs) {
    // Flush residual data (URCs, leftover responses) before sending
    flushRx();

    serial_->write(command);

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "ERROR") != NULL) {
                {
                    SerialLogGuard guard;
                    LOG_PRINT("CatM1Modem: AT error: ");
                    LOG_PRINTLN(response);
                }
                return false;
            }
            if (strstr(response, "OK") != NULL) {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    LOG_PRINTLN("CatM1Modem: AT timeout");
    return false;
}

bool CatM1Modem::negotiateBaudRate() {
    if (config_.targetBaudRate == 0 || config_.targetBaudRate == config_.baudRate) {
        return true;  // no change needed
    }

    LOG_PRINTF("CatM1Modem: negotiating baud rate %lu → %lu\n",
                  (unsigned long)config_.baudRate, (unsigned long)config_.targetBaudRate);

    // Step 1: Tell modem to switch baud rate
    char cmd[CMD_BUF_SIZE];
    snprintf(cmd, sizeof(cmd), "AT+IPR=%lu\r\n", (unsigned long)config_.targetBaudRate);
    if (!sendATCommand(cmd, TIMEOUT_AT_BASIC_MS)) {
        LOG_PRINTLN("CatM1Modem: AT+IPR failed, staying at current baud rate");
        return false;
    }

    // Step 2: Switch ESP32 UART to match
    serial_->flush();  // ensure AT+IPR response fully sent at old baud
    vTaskDelay(pdMS_TO_TICKS(50));  // brief stabilization
    serial_->updateBaudRate(config_.targetBaudRate);

    // Step 3: Verify communication at new baud rate
    vTaskDelay(pdMS_TO_TICKS(100));
    if (sendATCommand("AT\r\n", TIMEOUT_AT_BASIC_MS)) {
        if (!sendATCommand("AT&W\r\n", TIMEOUT_AT_BASIC_MS)) {
            LOG_PRINTLN("CatM1Modem: WARNING AT&W failed, baud rate change is volatile");
        }
        LOG_PRINTF("CatM1Modem: baud rate negotiated to %lu\n",
                      (unsigned long)config_.targetBaudRate);
        return true;
    }

    // Step 4: Fallback — modem is at targetBaudRate, try to revert it.
    // Must send AT+IPR at targetBaudRate (where modem currently is).
    LOG_PRINTLN("CatM1Modem: verification failed at new baud, reverting modem");
    // Retry AT+IPR revert at targetBaudRate (modem's current rate)
    snprintf(cmd, sizeof(cmd), "AT+IPR=%lu\r\n", (unsigned long)config_.baudRate);
    if (sendATCommand(cmd, TIMEOUT_AT_BASIC_MS)) {
        // Modem accepted revert — switch ESP32 back
        serial_->flush();
        vTaskDelay(pdMS_TO_TICKS(50));
        serial_->updateBaudRate(config_.baudRate);
        vTaskDelay(pdMS_TO_TICKS(100));
        if (sendATCommand("AT\r\n", TIMEOUT_AT_BASIC_MS)) {
            LOG_PRINTLN("CatM1Modem: reverted to original baud rate");
            return false;
        }
    }

    // Last resort: modem unresponsive at targetBaudRate too.
    // Try baudRate in case modem didn't actually switch.
    serial_->updateBaudRate(config_.baudRate);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!sendATCommand("AT\r\n", TIMEOUT_AT_BASIC_MS)) {
        LOG_PRINTLN("CatM1Modem: WARNING baud negotiation last resort failed — UART may be inconsistent");
    }

    return false;
}
