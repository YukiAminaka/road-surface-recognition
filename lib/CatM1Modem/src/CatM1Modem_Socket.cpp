#include "CatM1Modem.h"
#include "CatM1Modem_Internal.h"
#include <SerialLog.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

bool CatM1Modem::send(const uint8_t* data, size_t length) {
    vTaskDelay(pdMS_TO_TICKS(SEND_PRE_DELAY_MS));

    // Flush receive buffer
    unsigned long flushStart = millis();
    unsigned long flushLimit = millis();
    while ((serial_->available() || (millis() - flushStart < FLUSH_IDLE_MS)) && (millis() - flushLimit < FLUSH_MAX_MS)) {
        if (serial_->available()) {
            serial_->read();
            flushStart = millis();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    char command[SEND_CMD_BUF_SIZE];
    snprintf(command, sizeof(command), "AT+CASEND=0,%u\r\n", (unsigned int)length);
    serial_->write(command);

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();
    bool promptReceived = false;

    // Wait for ">" prompt
    while (millis() - startTime < PROMPT_TIMEOUT_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if ((idx >= 2 && response[idx-2] == '>' && response[idx-1] == ' ') || (idx >= 1 && response[idx-1] == '>')) {
                promptReceived = true;
                break;
            }
            if (strstr(response, "ERROR") != NULL) {
                LOG_PRINT("CatM1Modem: error before prompt: ");
                LOG_PRINTLN(response);
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!promptReceived) {
        LOG_PRINTLN("CatM1Modem: no prompt received");
        return false;
    }

    serial_->write(data, length);

    idx = 0;
    response[0] = '\0';
    startTime = millis();
    bool sendOk = false;

    // Wait for send confirmation
    while (millis() - startTime < SEND_CONFIRM_TIMEOUT_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "OK") != NULL) {
                sendOk = true;
                break;
            }
            if (strstr(response, "ERROR") != NULL) {
                LOG_PRINTLN("CatM1Modem: send ERROR");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!sendOk && strstr(response, "ERROR") == NULL) {
        LOG_PRINTLN("CatM1Modem: send TIMEOUT");
        return false;
    }

    return sendOk;
}

CatM1Modem::SendResult CatM1Modem::sendWithRetry(const uint8_t* data, size_t length) {
    int consecutiveFailures = 0;
    bool recoveryUsed = false;

    while (consecutiveFailures < RETRY_MAX_FAILURES) {
        if (send(data, length)) {
            LOG_PRINTLN("CatM1Modem: send success");
            return recoveryUsed ? SendResult::RECOVERED : SendResult::SUCCESS;
        }

        consecutiveFailures++;
        LOG_PRINTF("CatM1Modem: send failed. Fail count: %d\n", consecutiveFailures);

        if (consecutiveFailures >= RETRY_MAX_FAILURES) {
            break;
        }

        if (consecutiveFailures == RETRY_L1_THRESHOLD) {
            LOG_PRINTLN("CatM1Modem: Recovery L1: Reset Socket...");
            recoverSocket();
            recoveryUsed = true;
        } else if (consecutiveFailures == RETRY_L2_THRESHOLD) {
            LOG_PRINTLN("CatM1Modem: Recovery L2: Reconnect Network...");
            if (!recoverNetwork()) {
                LOG_PRINTLN("CatM1Modem: L2 failed. Escalating to L3...");
                recoverModemFull();
            }
            recoveryUsed = true;
        } else {
            vTaskDelay(pdMS_TO_TICKS(RETRY_INTERVAL_MS));
        }
    }

    LOG_PRINTF("CatM1Modem: giving up after %d consecutive failures\n", consecutiveFailures);
    return SendResult::FAILED;
}

bool CatM1Modem::openConnection(uint8_t cid, const char* type, const char* host, uint16_t port) {
    char closeCmd[CMD_BUF_SIZE];
    snprintf(closeCmd, sizeof(closeCmd), "AT+CACLOSE=%u\r\n", cid);
    sendATCommand(closeCmd, TIMEOUT_AT_BASIC_MS);

    char openCmd[CMD_BUF_SIZE];
    int openLen = snprintf(openCmd, sizeof(openCmd), "AT+CAOPEN=%u,0,\"%s\",\"%s\",%u,1\r\n",
             cid, type, host, port);
    if (openLen < 0 || openLen >= (int)sizeof(openCmd)) {
        LOG_PRINTF("CatM1Modem: CAOPEN command too long (cid=%u)\n", cid);
        return false;
    }

    flushRx();
    serial_->write(openCmd);

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();

    while (millis() - startTime < TIMEOUT_AT_SOCKET_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "ERROR") != NULL) {
                LOG_PRINTF("CatM1Modem: CAOPEN cid=%u error: %s\n", cid, response);
                return false;
            }
            char* caopen = strstr(response, "+CAOPEN:");
            if (caopen && strstr(response, "OK") != NULL) {
                char* comma = strchr(caopen, ',');
                if (comma) {
                    int result = atoi(comma + 1);
                    if (result == 0) {
                        LOG_PRINTF("CatM1Modem: cid=%u connected (%s:%u)\n", cid, host, port);
                        return true;
                    }
                    LOG_PRINTF("CatM1Modem: CAOPEN cid=%u failed (result=%d)\n", cid, result);
                    return false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    LOG_PRINTF("CatM1Modem: CAOPEN cid=%u timeout\n", cid);
    return false;
}

bool CatM1Modem::sendOnCid(uint8_t cid, const uint8_t* data, size_t length) {
    vTaskDelay(pdMS_TO_TICKS(SEND_PRE_DELAY_MS));

    // Flush receive buffer
    unsigned long flushStart = millis();
    unsigned long flushLimit = millis();
    while ((serial_->available() || (millis() - flushStart < FLUSH_IDLE_MS)) && (millis() - flushLimit < FLUSH_MAX_MS)) {
        if (serial_->available()) {
            serial_->read();
            flushStart = millis();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    char command[SEND_CMD_BUF_SIZE];
    snprintf(command, sizeof(command), "AT+CASEND=%u,%u\r\n", cid, (unsigned int)length);
    serial_->write(command);

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();
    bool promptReceived = false;

    // Wait for ">" prompt
    while (millis() - startTime < PROMPT_TIMEOUT_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if ((idx >= 2 && response[idx-2] == '>' && response[idx-1] == ' ') || (idx >= 1 && response[idx-1] == '>')) {
                promptReceived = true;
                break;
            }
            if (strstr(response, "ERROR") != NULL) {
                LOG_PRINTF("CatM1Modem: sendOnCid cid=%u ERROR before prompt: %s\n", cid, response);
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!promptReceived) {
        LOG_PRINTF("CatM1Modem: sendOnCid cid=%u no prompt (timeout %lums)\n", cid, PROMPT_TIMEOUT_MS);
        return false;
    }

    serial_->write(data, length);

    idx = 0;
    response[0] = '\0';
    startTime = millis();
    bool sendOk = false;

    while (millis() - startTime < SEND_CONFIRM_TIMEOUT_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "OK") != NULL) {
                sendOk = true;
                break;
            }
            if (strstr(response, "ERROR") != NULL) {
                // Diagnostic: strstr matches "ERROR" inside "+CME ERROR" BEFORE the
                // trailing ": <code>\r\n" arrives. Keep reading briefly so the dump
                // captures the full error code (or proves it is genuinely absent).
                unsigned long errStart = millis();
                unsigned long errIdle = millis();
                while ((millis() - errIdle < 30) && (millis() - errStart < 200)) {
                    if (serial_->available()) {
                        char c2 = serial_->read();
                        if (idx < (int)sizeof(response) - 1) {
                            response[idx++] = c2;
                            response[idx] = '\0';
                        }
                        errIdle = millis();
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }
                {
                    SerialLogGuard guard;
                    LOG_PRINTF("CatM1Modem: sendOnCid cid=%u ERROR after data (len=%d): ", cid, idx);
                    for (int i = 0; i < idx; i++) {
                        LOG_PRINTF("%02X ", (unsigned char)response[i]);
                    }
                    LOG_PRINT("\n  text: ");
                    for (int i = 0; i < idx; i++) {
                        char ch = response[i];
                        LOG_PRINT((ch >= 0x20 && ch < 0x7F) ? ch : '.');
                    }
                    LOG_PRINTLN();
                }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return sendOk;
}

bool CatM1Modem::queryUnackedBytes(uint8_t cid, unsigned long& txLen,
                                   unsigned long& ackLen, unsigned long& nackLen) {
    // AT+CAACK=<cid> returns "+CAACK: <txlen>,<acklen>,<nacklen>".
    // nacklen approaching the SIM7080G internal TCP send buffer cap (~8KB)
    // indicates Cat-M1 uplink is not draining fast enough — root cause for
    // chunk-N "+CME ERROR" responses to AT+CASEND when buffer is full.
    txLen = ackLen = nackLen = 0;

    flushRx();
    char cmd[SEND_CMD_BUF_SIZE];
    snprintf(cmd, sizeof(cmd), "AT+CAACK=%u\r\n", cid);
    serial_->write(cmd);

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();

    while (millis() - startTime < TIMEOUT_AT_BASIC_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "OK") != NULL || strstr(response, "ERROR") != NULL) break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    const char* p = strstr(response, "+CAACK:");
    if (!p) return false;
    p += 7;  // skip "+CAACK:"
    while (*p == ' ') p++;

    // Parse "<tx>,<ack>,<nack>" — strtoul handles leading whitespace
    char* end = nullptr;
    txLen = strtoul(p, &end, 10);
    if (!end || *end != ',') return false;
    ackLen = strtoul(end + 1, &end, 10);
    if (!end || *end != ',') return false;
    nackLen = strtoul(end + 1, nullptr, 10);
    return true;
}

void CatM1Modem::waitForSendBufferDrain(uint8_t cid) {
    unsigned long start = millis();
    bool loggedWait = false;
    while (millis() - start < BUFFER_DRAIN_TIMEOUT_MS) {
        unsigned long tx = 0, ack = 0, nack = 0;
        if (!queryUnackedBytes(cid, tx, ack, nack)) {
            // CAACK failed — modem AT in transient state. Fall back to fixed
            // pacing once (don't loop on a broken interface) and proceed.
            vTaskDelay(pdMS_TO_TICKS(CHUNK_PACING_MS));
            return;
        }
        if (nack < currentWatermarkBytes()) return;
        if (!loggedWait) {
            LOG_PRINTF("CatM1Modem: waiting for TCP send buffer drain (nack=%lu)\n", nack);
            loggedWait = true;
        }
        vTaskDelay(pdMS_TO_TICKS(BUFFER_DRAIN_POLL_MS));
    }
    LOG_PRINTLN("CatM1Modem: buffer drain timeout, proceeding anyway");
}

bool CatM1Modem::checkCidConnected(uint8_t cid) {
    // AT+CASTATE? returns "+CASTATE: <cid>,<state>" for each active CID.
    // AT+CASTATE=<n> is NOT a valid read form on SIM7080G — use ? form only.
    flushRx();
    serial_->write("AT+CASTATE?\r\n");

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();

    while (millis() - startTime < TIMEOUT_AT_BASIC_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "OK") != NULL || strstr(response, "ERROR") != NULL) break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Scan for "+CASTATE: <cid>,<state>" matching the requested cid
    const char* p = response;
    while ((p = strstr(p, "+CASTATE:")) != NULL) {
        p += 9;  // skip "+CASTATE:"
        while (*p == ' ') p++;
        if (atoi(p) == (int)cid) {
            const char* comma = strchr(p, ',');
            if (comma) return atoi(comma + 1) == 1;
        }
    }
    return false;
}

bool CatM1Modem::openSocket() {
    sendATCommand("AT+CACLOSE=0\r\n", TIMEOUT_AT_BASIC_MS);

    if (!sendATCommand("AT+CACID=0\r\n", TIMEOUT_AT_BASIC_MS)) {
        LOG_PRINTLN("CatM1Modem: AT+CACID=0 failed");
        return false;
    }

    char openCmd[CMD_BUF_SIZE];
    int openLen = snprintf(openCmd, sizeof(openCmd), "AT+CAOPEN=0,0,\"%s\",\"%s\",%u,1\r\n",
             config_.protocol, config_.host, config_.port);
    if (openLen < 0 || openLen >= (int)sizeof(openCmd)) {
        LOG_PRINTLN("CatM1Modem: CAOPEN command too long");
        return false;
    }

    // Send CAOPEN and parse +CAOPEN: <cid>,<result> for result=0
    flushRx();
    serial_->write(openCmd);

    char response[RESPONSE_BUF_SIZE];
    int idx = 0;
    response[0] = '\0';
    unsigned long startTime = millis();

    while (millis() - startTime < TIMEOUT_AT_SOCKET_MS) {
        if (serial_->available()) {
            char c = serial_->read();
            if (idx < (int)sizeof(response) - 1) {
                response[idx++] = c;
                response[idx] = '\0';
            }
            if (strstr(response, "ERROR") != NULL) {
                LOG_PRINT("CatM1Modem: CAOPEN error: ");
                LOG_PRINTLN(response);
                return false;
            }
            // Check for +CAOPEN: <cid>,<result> followed by OK
            char* caopen = strstr(response, "+CAOPEN:");
            if (caopen && strstr(response, "OK") != NULL) {
                char* comma = strchr(caopen, ',');
                if (comma) {
                    int result = atoi(comma + 1);
                    if (result == 0) {
                        LOG_PRINTLN("CatM1Modem: socket opened (result=0)");
                        return true;
                    }
                    LOG_PRINTF("CatM1Modem: CAOPEN failed (result=%d)\n", result);
                    return false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    LOG_PRINTLN("CatM1Modem: CAOPEN timeout");
    return false;
}

bool CatM1Modem::tcpSend(const uint8_t* data, size_t length) {
    LOG_PRINTF("CatM1Modem: TCP send %u bytes to %s:%u\n",
                  (unsigned int)length, config_.tcpHost, config_.tcpPort);

    // Open TCP connection on cid=1 (coexists with UDP on cid=0)
    if (!openConnection(TCP_UPLOAD_CID, "TCP", config_.tcpHost, config_.tcpPort)) {
        LOG_PRINTLN("CatM1Modem: TCP connect failed");
        return false;
    }

    // Send data in chunks
    size_t offset = 0;
    int chunkNum = 0;
    bool ok = true;

    while (offset < length) {
        size_t chunkSize = length - offset;
        if (chunkSize > TCP_CHUNK_SIZE) chunkSize = TCP_CHUNK_SIZE;

        if (!sendOnCid(TCP_UPLOAD_CID, data + offset, chunkSize)) {
            LOG_PRINTF("CatM1Modem: TCP chunk %d failed at %u/%u\n",
                          chunkNum, (unsigned int)offset, (unsigned int)length);
            ok = false;
            break;
        }

        offset += chunkSize;
        chunkNum++;

        if (chunkNum % 10 == 0) {
            LOG_PRINTF("CatM1Modem: TCP send %u/%u bytes\n",
                          (unsigned int)offset, (unsigned int)length);
        }
    }

    if (ok) {
        LOG_PRINTF("CatM1Modem: TCP send complete (%d chunks, %u bytes)\n",
                      chunkNum, (unsigned int)length);

        // Wait for TCP send buffer to flush before closing.
        // AT+CASEND OK only means data is accepted into the modem buffer,
        // not that it has been delivered over TCP.  Closing too early can
        // cause the remote end to miss trailing bytes (e.g. CRC in SORACOM
        // Binary Format v1 → "Checksum is not matched").
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Close TCP connection
    char closeCmd[CMD_BUF_SIZE];
    snprintf(closeCmd, sizeof(closeCmd), "AT+CACLOSE=%u\r\n", TCP_UPLOAD_CID);
    sendATCommand(closeCmd, TIMEOUT_AT_BASIC_MS);

    return ok;
}
