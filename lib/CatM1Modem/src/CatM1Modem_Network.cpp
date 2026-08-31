#include "CatM1Modem.h"
#include "CatM1Modem_Internal.h"
#include <SerialLog.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

bool CatM1Modem::waitForRegistration(int maxAttempts, unsigned long intervalMs) {
    for (int i = 0; i < maxAttempts; i++) {
        serial_->write("AT+CEREG?\r\n");

        char response[RESPONSE_BUF_SIZE];
        int idx = 0;
        response[0] = '\0';
        unsigned long startTime = millis();

        while (millis() - startTime < TIMEOUT_AT_SIM_MS) {
            if (serial_->available()) {
                char c = serial_->read();
                if (idx < (int)sizeof(response) - 1) {
                    response[idx++] = c;
                    response[idx] = '\0';
                }
                if (strstr(response, "OK") != NULL) {
                    // Parse +CEREG: <n>,<stat> — stat 1=home, 5=roaming
                    char* cereg = strstr(response, "+CEREG:");
                    if (cereg) {
                        char* comma = strchr(cereg, ',');
                        if (comma) {
                            int stat = atoi(comma + 1);
                            if (stat == 1 || stat == 5) {
                                LOG_PRINTF("CatM1Modem: registered (stat=%d)\n", stat);
                                return true;
                            }
                            LOG_PRINTF("CatM1Modem: not registered yet (stat=%d), retry %d/%d\n",
                                          stat, i + 1, maxAttempts);
                        }
                    }
                    break;  // got OK but not registered, retry after delay
                }
                if (strstr(response, "ERROR") != NULL) {
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        delay(intervalMs);
    }
    return false;
}

bool CatM1Modem::waitForPdpActive(unsigned long timeoutMs) {
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
            if (strstr(response, "ACTIVE") != NULL) {
                LOG_PRINTLN("CatM1Modem: PDP context active");
                return true;
            }
            if (strstr(response, "DEACTIVE") != NULL) {
                LOG_PRINTLN("CatM1Modem: PDP context deactivated");
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Timeout — verify with AT+CNACT?
    LOG_PRINTLN("CatM1Modem: PDP URC timeout, checking with AT+CNACT?");
    return sendATCommand("AT+CNACT?\r\n", TIMEOUT_AT_SIM_MS);
}

bool CatM1Modem::activatePdp() {
    for (int attempt = 0; attempt < PDP_ACTIVATE_RETRIES; attempt++) {
        if (attempt > 0) {
            LOG_PRINTF("CatM1Modem: PDP activation retry %d/%d\n",
                          attempt + 1, PDP_ACTIVATE_RETRIES);
        }
        if (sendATCommand("AT+CNACT=0,1\r\n", TIMEOUT_AT_PDP_MS) &&
            waitForPdpActive(TIMEOUT_AT_PDP_MS)) {
            sendATCommand("AT+CNACT?\r\n", TIMEOUT_AT_BASIC_MS);
            return true;
        }
        // Deactivate before retry to reset PDP state
        sendATCommand("AT+CNACT=0,0\r\n", TIMEOUT_AT_SIM_MS);
        sendATCommand("AT+CNACT?\r\n", TIMEOUT_AT_BASIC_MS);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    LOG_PRINTLN("CatM1Modem: PDP activation failed after retries");
    return false;
}

bool CatM1Modem::isPdpActive() {
    flushRx();  // light flush; callers (ensureConnected) already do deep flush
    serial_->write("AT+CNACT?\r\n");

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();

    while (millis() - startTime < TIMEOUT_AT_SIM_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "ERROR") != NULL) return false;
            if (strstr(response, "OK") != NULL) {
                // Parse +CNACT: 0,<status>,"<ip>"  — status 1 = active
                char* cnact = strstr(response, "+CNACT:");
                if (cnact) {
                    char* comma = strchr(cnact, ',');
                    if (comma) {
                        int status = atoi(comma + 1);
                        return (status == 1);
                    }
                }
                return false;  // no +CNACT line found
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool CatM1Modem::ensureConnected() {
    // Clear any residual data from prior HTTP sessions before probing network state.
    // Without this, leftover response bytes can corrupt AT+CEREG/CNACT parsing.
    flushRxDeep();

    // Step 1: Quick registration check (3 attempts, 1s apart)
    bool registered = waitForRegistration(3, 1000);
    if (!registered) {
        LOG_PRINTLN("CatM1Modem: ensureConnected — not registered, recovering network (L2)");
        if (recoverNetwork()) return true;
        LOG_PRINTLN("CatM1Modem: ensureConnected — L2 failed, full recovery (L3)");
        if (recoverModemFull()) return true;
        LOG_PRINTLN("CatM1Modem: ensureConnected — L3 failed, deep recovery (L4)");
        return recoverModemDeep();
    }

    // Step 2: PDP context check (carrier may have deactivated during idle)
    if (!isPdpActive()) {
        LOG_PRINTLN("CatM1Modem: ensureConnected — PDP inactive, recovering network (L2)");
        if (recoverNetwork()) return true;
        LOG_PRINTLN("CatM1Modem: ensureConnected — L2 failed, full recovery (L3)");
        if (recoverModemFull()) return true;
        LOG_PRINTLN("CatM1Modem: ensureConnected — L3 failed, deep recovery (L4)");
        return recoverModemDeep();
    }

    LOG_PRINTLN("CatM1Modem: ensureConnected — network OK");
    return true;
}

bool CatM1Modem::recoverSocket() {
    return openSocket();
}

bool CatM1Modem::recoverNetwork() {
    httpConnected_ = false;
    sendATCommand("AT+CACLOSE=0\r\n", TIMEOUT_AT_BASIC_MS);
    // Also close HTTP TCP socket (cid=1) if it was open
    char closeTcpCmd[CMD_BUF_SIZE];
    snprintf(closeTcpCmd, sizeof(closeTcpCmd), "AT+CACLOSE=%u\r\n", TCP_UPLOAD_CID);
    sendATCommand(closeTcpCmd, TIMEOUT_AT_BASIC_MS);
    sendATCommand("AT+CNACT=0,0\r\n", TIMEOUT_AT_SIM_MS);
    sendATCommand("AT+CNACT?\r\n", TIMEOUT_AT_BASIC_MS);

    // Define PDP context with APN before re-attach (same as init)
    {
        char cgdcontCmd[CMD_BUF_SIZE];
        snprintf(cgdcontCmd, sizeof(cgdcontCmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", config_.apn);
        sendATCommand(cgdcontCmd, TIMEOUT_AT_SIM_MS);
    }

    // Detach and re-attach to force clean network state
    sendATCommand("AT+CGATT=0\r\n", TIMEOUT_AT_CFUN_MS);
    sendATCommand("AT+CGATT?\r\n", TIMEOUT_AT_BASIC_MS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (!sendATCommand("AT+CGATT=1\r\n", TIMEOUT_AT_NETWORK_MS)) {
        LOG_PRINTLN("CatM1Modem: L2 CGATT=1 failed");
        return false;
    }
    sendATCommand("AT+CGATT?\r\n", TIMEOUT_AT_BASIC_MS);

    // Wait for registration BEFORE PDP activation
    if (!waitForRegistration(10, 2000)) {
        LOG_PRINTLN("CatM1Modem: L2 registration failed");
        return false;
    }

    char apnCmd[CMD_BUF_SIZE];
    int apnLen = snprintf(apnCmd, sizeof(apnCmd), "AT+CNCFG=0,1,\"%s\",\"%s\",\"%s\",%u\r\n",
             config_.apn, config_.user, config_.pass, config_.authType);
    if (apnLen < 0 || apnLen >= (int)sizeof(apnCmd)) {
        LOG_PRINTLN("CatM1Modem: L2 APN command too long");
        return false;
    }
    sendATCommand(apnCmd, TIMEOUT_AT_SIM_MS);
    sendATCommand("AT+CNCFG?\r\n", TIMEOUT_AT_BASIC_MS);

    if (!activatePdp()) {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));  // PDP stabilization delay

    // Extended flush: discard delayed URCs that arrive after PDP activation
    {
        unsigned long fIdle = millis();
        unsigned long fHard = millis();
        while ((millis() - fIdle < 200) && (millis() - fHard < 2000)) {
            if (serial_->available()) {
                serial_->read();
                fIdle = millis();
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }

    return openSocket();
}

bool CatM1Modem::recoverModemFull() {
    httpConnected_ = false;

    // Deactivate PDP before reset (best-effort)
    sendATCommand("AT+CNACT=0,0\r\n", TIMEOUT_AT_SIM_MS);

    // AT+CFUN=1,1 resets modem — baud rate typically reverts to default,
    // but may persist at targetBaudRate if NVM-stored.
    serial_->write("AT+CFUN=1,1\r\n");
    serial_->flush();
    serial_->updateBaudRate(config_.baudRate);  // try default baud first
    vTaskDelay(pdMS_TO_TICKS(MODEM_RESET_WAIT_MS));

    // Probe at initial baudRate first (most likely after reset)
    bool active = false;
    bool alreadyAtTarget = false;  // true if modem responded at targetBaudRate (NVM path)
    for (int i = 0; i < MAX_AT_RETRIES; i++) {
        if (sendATCommand("AT\r\n", TIMEOUT_AT_BASIC_MS)) {
            active = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(MODEM_RETRY_DELAY_MS));
    }

    // If initial baudRate failed, try targetBaudRate (NVM persistence case)
    if (!active && config_.targetBaudRate != 0 && config_.targetBaudRate != config_.baudRate) {
        LOG_PRINTLN("CatM1Modem: no response at initial baud, trying target baud rate");
        serial_->updateBaudRate(config_.targetBaudRate);
        for (int i = 0; i < NVM_BAUD_RETRIES; i++) {
            if (sendATCommand("AT\r\n", TIMEOUT_AT_BASIC_MS)) {
                active = true;
                alreadyAtTarget = true;
                LOG_PRINTLN("CatM1Modem: modem responded at target baud rate (NVM persisted)");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(MODEM_RETRY_DELAY_MS));
        }
    }
    if (!active) return false;

    // Clean state after reset (same pattern as init)
    sendATCommand("ATZ\r\n", TIMEOUT_AT_BASIC_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    sendATCommand("ATE0\r\n", TIMEOUT_AT_BASIC_MS);

    // Radio off → configure → radio on (TinyGSM-style, same as init)
    // CFUN=1,1 reboots to CFUN=1 (full) by default, so explicitly go to
    // CFUN=0 first to clear stale state and apply band config cleanly.
    sendATCommand("AT+CFUN=0\r\n", TIMEOUT_AT_CFUN_MS);
    vTaskDelay(pdMS_TO_TICKS(500));

    configureCatM1Mode();

    if (!sendATCommand("AT+CFUN=1\r\n", TIMEOUT_AT_CFUN_MS)) return false;
    sendATCommand("AT+CFUN?\r\n", TIMEOUT_AT_BASIC_MS);

    // Define PDP context with APN before attach (same as init)
    {
        char cgdcontCmd[CMD_BUF_SIZE];
        snprintf(cgdcontCmd, sizeof(cgdcontCmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", config_.apn);
        sendATCommand(cgdcontCmd, TIMEOUT_AT_SIM_MS);
    }

    // CGATT=0 not needed — CFUN=0 already cleared attach state
    if (!sendATCommand("AT+CGATT=1\r\n", TIMEOUT_AT_NETWORK_MS)) return false;
    sendATCommand("AT+CGATT?\r\n", TIMEOUT_AT_BASIC_MS);

    // URC configuration (same as init Phase 2.5)
    sendATCommand("AT+CEREG=2\r\n", TIMEOUT_AT_BASIC_MS);
    sendATCommand("AT+CNSMOD=1\r\n", TIMEOUT_AT_BASIC_MS);

    if (!waitForRegistration(15, 2000)) {
        LOG_PRINTLN("CatM1Modem: L3 registration failed");
        return false;
    }

    char apnCmd[CMD_BUF_SIZE];
    int apnLen = snprintf(apnCmd, sizeof(apnCmd), "AT+CNCFG=0,1,\"%s\",\"%s\",\"%s\",%u\r\n",
             config_.apn, config_.user, config_.pass, config_.authType);
    if (apnLen < 0 || apnLen >= (int)sizeof(apnCmd)) {
        LOG_PRINTLN("CatM1Modem: L3 APN command too long");
        return false;
    }
    sendATCommand(apnCmd, TIMEOUT_AT_SIM_MS);
    sendATCommand("AT+CNCFG?\r\n", TIMEOUT_AT_BASIC_MS);

    if (!activatePdp()) return false;

    vTaskDelay(pdMS_TO_TICKS(3000));  // PDP stabilization delay

    // Extended flush: discard delayed URCs that arrive after PDP activation
    {
        unsigned long fIdle = millis();
        unsigned long fHard = millis();
        while ((millis() - fIdle < 200) && (millis() - fHard < 2000)) {
            if (serial_->available()) {
                serial_->read();
                fIdle = millis();
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }

    if (!openSocket()) return false;

    // Re-negotiate higher baud rate after modem reset
    // Skip if modem already responded at targetBaudRate (NVM-persisted baud)
    if (!alreadyAtTarget) {
        if (!negotiateBaudRate()) {
            LOG_PRINTLN("CatM1Modem: WARNING baud negotiation failed after L3 recovery, continuing at default baud rate");
        }
    } else {
        LOG_PRINTLN("CatM1Modem: skipping baud negotiation — already at target baud rate");
    }
    return true;
}

bool CatM1Modem::recoverModemDeep() {
    httpConnected_ = false;

    // Phase 1: PDP deactivate (best-effort)
    sendATCommand("AT+CNACT=0,0\r\n", TIMEOUT_AT_SIM_MS);
    sendATCommand("AT+CNACT?\r\n", TIMEOUT_AT_BASIC_MS);

    // Phase 2: Clear cached network info
    // AT+CFUN=0 → AT+CLRNET=0,1 → AT+CFUN=1,1
    LOG_PRINTLN("CatM1Modem: L4 clearing network info (AT+CLRNET)");
    sendATCommand("AT+CFUN=0\r\n", TIMEOUT_AT_CFUN_MS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    sendATCommand("AT+CLRNET=0,1\r\n", TIMEOUT_AT_SIM_MS);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Phase 3: Full reset with restart
    serial_->write("AT+CFUN=1,1\r\n");
    serial_->flush();
    serial_->updateBaudRate(config_.baudRate);
    vTaskDelay(pdMS_TO_TICKS(MODEM_RESET_WAIT_MS));

    // Phase 4: Probe for modem (same as recoverModemFull)
    bool active = false;
    bool alreadyAtTarget = false;
    for (int i = 0; i < MAX_AT_RETRIES; i++) {
        if (sendATCommand("AT\r\n", TIMEOUT_AT_BASIC_MS)) { active = true; break; }
        vTaskDelay(pdMS_TO_TICKS(MODEM_RETRY_DELAY_MS));
    }
    if (!active && config_.targetBaudRate != 0 && config_.targetBaudRate != config_.baudRate) {
        serial_->updateBaudRate(config_.targetBaudRate);
        for (int i = 0; i < NVM_BAUD_RETRIES; i++) {
            if (sendATCommand("AT\r\n", TIMEOUT_AT_BASIC_MS)) { active = true; alreadyAtTarget = true; break; }
            vTaskDelay(pdMS_TO_TICKS(MODEM_RETRY_DELAY_MS));
        }
    }
    if (!active) return false;

    // Phase 5: Clean init
    sendATCommand("ATZ\r\n", TIMEOUT_AT_BASIC_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    sendATCommand("ATE0\r\n", TIMEOUT_AT_BASIC_MS);
    configureCatM1Mode();

    if (!sendATCommand("AT+CFUN=1\r\n", TIMEOUT_AT_CFUN_MS)) return false;
    sendATCommand("AT+CFUN?\r\n", TIMEOUT_AT_BASIC_MS);

    // Define PDP context with APN before attach (same as init)
    {
        char cgdcontCmd[CMD_BUF_SIZE];
        snprintf(cgdcontCmd, sizeof(cgdcontCmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", config_.apn);
        sendATCommand(cgdcontCmd, TIMEOUT_AT_SIM_MS);
    }

    sendATCommand("AT+CGATT=0\r\n", TIMEOUT_AT_CFUN_MS);
    sendATCommand("AT+CGATT?\r\n", TIMEOUT_AT_BASIC_MS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (!sendATCommand("AT+CGATT=1\r\n", TIMEOUT_AT_NETWORK_MS)) return false;
    sendATCommand("AT+CGATT?\r\n", TIMEOUT_AT_BASIC_MS);

    sendATCommand("AT+CEREG=2\r\n", TIMEOUT_AT_BASIC_MS);
    sendATCommand("AT+CNSMOD=1\r\n", TIMEOUT_AT_BASIC_MS);

    if (!waitForRegistration(30, 2000)) {
        LOG_PRINTLN("CatM1Modem: L4 registration failed");
        return false;
    }

    // APN + PDP
    char apnCmd[CMD_BUF_SIZE];
    int apnLen = snprintf(apnCmd, sizeof(apnCmd), "AT+CNCFG=0,1,\"%s\",\"%s\",\"%s\",%u\r\n",
             config_.apn, config_.user, config_.pass, config_.authType);
    if (apnLen < 0 || apnLen >= (int)sizeof(apnCmd)) return false;
    sendATCommand(apnCmd, TIMEOUT_AT_SIM_MS);
    sendATCommand("AT+CNCFG?\r\n", TIMEOUT_AT_BASIC_MS);

    if (!activatePdp()) return false;
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Extended flush
    {
        unsigned long fIdle = millis();
        unsigned long fHard = millis();
        while ((millis() - fIdle < 200) && (millis() - fHard < 2000)) {
            if (serial_->available()) { serial_->read(); fIdle = millis(); }
            else vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (!openSocket()) return false;

    if (!alreadyAtTarget) {
        if (!negotiateBaudRate()) {
            LOG_PRINTLN("CatM1Modem: WARNING L4 baud negotiation failed");
        }
    }
    return true;
}
