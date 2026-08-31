#include "CatM1Modem.h"
#include "CatM1Modem_Internal.h"
#include <SerialLog.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>

bool CatM1Modem::httpOpen() {
    // If connection appears alive but has been idle too long, proactively
    // close and reopen.  SORACOM/carrier may silently drop idle PDP contexts
    // or TCP connections, leaving httpConnected_ stale.
    if (httpConnected_) {
        unsigned long idleMs = millis() - httpLastActivityMs_;
        if (idleMs > HTTP_CONN_IDLE_TIMEOUT_MS) {
            LOG_PRINTF("CatM1Modem: HTTP connection idle %lu ms, recycling\n", idleMs);
            httpClose();
        } else {
            return true;
        }
    }

    flushRxDeep();  // clear any residual bytes before opening new connection
    if (!openConnection(TCP_UPLOAD_CID, "TCP", config_.httpHost, config_.httpPort)) {
        LOG_PRINTLN("CatM1Modem: HTTP persistent connect failed");
        return false;
    }
    httpConnected_ = true;
    httpLastActivityMs_ = millis();
    LOG_PRINTLN("CatM1Modem: HTTP persistent connection opened");
    return true;
}

void CatM1Modem::httpClose() {
    if (!httpConnected_) return;
    char closeCmd[CMD_BUF_SIZE];
    snprintf(closeCmd, sizeof(closeCmd), "AT+CACLOSE=%u\r\n", TCP_UPLOAD_CID);
    sendATCommand(closeCmd, TIMEOUT_AT_BASIC_MS);
    httpConnected_ = false;
    LOG_PRINTLN("CatM1Modem: HTTP persistent connection closed");
}

bool CatM1Modem::httpPost(const char* contentType, const uint8_t* body, size_t bodyLen) {
    size_t chunksTotal     = 0;
    size_t chunksWithRetry = 0;
    LOG_PRINTF("CatM1Modem: HTTP POST %u bytes to %s:%u%s\n",
                  (unsigned int)bodyLen, config_.httpHost, config_.httpPort, config_.httpPath);

    // Use persistent connection if httpOpen() was called, otherwise open per-call
    bool ownConnection = false;
    if (!httpConnected_) {
        if (!openConnection(TCP_UPLOAD_CID, "TCP", config_.httpHost, config_.httpPort)) {
            LOG_PRINTLN("CatM1Modem: HTTP connect failed");
            return false;
        }
        ownConnection = true;
    }

    const char* connHeader = ownConnection ? "close" : "keep-alive";

    // Build and send HTTP request header
    char header[512];
    int headerLen = snprintf(header, sizeof(header),
        "POST %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %u\r\n"
        "Connection: %s\r\n"
        "\r\n",
        config_.httpPath, config_.httpHost, contentType, (unsigned int)bodyLen, connHeader);

    if (headerLen < 0 || headerLen >= (int)sizeof(header)) {
        LOG_PRINTLN("CatM1Modem: HTTP header too long or encoding error");
        goto fail;
    }

    // Send header in chunks (with exponential backoff retry)
    {
        size_t offset = 0;
        while (offset < (size_t)headerLen) {
            size_t chunkSize = (size_t)headerLen - offset;
            if (chunkSize > currentChunkSize()) chunkSize = currentChunkSize();
            bool chunkOk = false;
            int  retryCount = 0;
            for (int r = 0; r < CHUNK_MAX_RETRIES; r++) {
                if (sendOnCid(TCP_UPLOAD_CID, (const uint8_t*)header + offset, chunkSize)) {
                    chunkOk = true;
                    retryCount = r;
                    break;
                }
                LOG_PRINTF("CatM1Modem: header chunk retry %d at %u/%u\n",
                              r + 1, (unsigned int)offset, (unsigned int)headerLen);
                {
                    unsigned long tx = 0, ack = 0, nack = 0;
                    if (queryUnackedBytes(TCP_UPLOAD_CID, tx, ack, nack)) {
                        LOG_PRINTF("CatM1Modem: CAACK cid=%u tx=%lu ack=%lu nack=%lu\n",
                                      TCP_UPLOAD_CID, tx, ack, nack);
                    }
                }
                if (r < CHUNK_MAX_RETRIES - 1) {
                    unsigned long delay = CHUNK_RETRY_BASE_MS << r;  // exponential
                    if (delay > CHUNK_RETRY_MAX_MS) delay = CHUNK_RETRY_MAX_MS;
                    vTaskDelay(pdMS_TO_TICKS(delay));
                }
            }
            chunksTotal++;
            if (chunkOk && retryCount > 0) chunksWithRetry++;
            if (!chunkOk) {
                LOG_PRINTLN("CatM1Modem: HTTP header send failed");
                goto fail;
            }
            offset += chunkSize;
        }
    }

    // Send body in chunks (exponential backoff + unconditional inter-chunk pacing)
    {
        size_t offset = 0;
        int chunkNum = 0;
        while (offset < bodyLen) {
            size_t chunkSize = bodyLen - offset;
            if (chunkSize > currentChunkSize()) chunkSize = currentChunkSize();

            // CAACK-driven adaptive pacing: query SIM7080G's internal TCP send buffer
            // occupancy (nack bytes) via AT+CAACK and wait if it would overflow on next
            // current-profile chunk. Replaces unconditional CHUNK_PACING_MS to handle
            // the case where Cat-M1 uplink drain rate fluctuates with RF/heat/load.
            // CAACK query itself takes ~30-100ms, providing natural minimum pacing.
            if (offset > 0) {
                waitForSendBufferDrain(TCP_UPLOAD_CID);
            }

            bool chunkOk = false;
            int  retryCount = 0;
            for (int r = 0; r < CHUNK_MAX_RETRIES; r++) {
                if (sendOnCid(TCP_UPLOAD_CID, body + offset, chunkSize)) {
                    chunkOk = true;
                    retryCount = r;
                    break;
                }
                LOG_PRINTF("CatM1Modem: chunk %d retry %d at %u/%u\n",
                              chunkNum, r + 1, (unsigned int)offset, (unsigned int)bodyLen);
                {
                    unsigned long tx = 0, ack = 0, nack = 0;
                    if (queryUnackedBytes(TCP_UPLOAD_CID, tx, ack, nack)) {
                        LOG_PRINTF("CatM1Modem: CAACK cid=%u tx=%lu ack=%lu nack=%lu\n",
                                      TCP_UPLOAD_CID, tx, ack, nack);
                    }
                }
                if (!checkCidConnected(TCP_UPLOAD_CID)) {
                    LOG_PRINTF("CatM1Modem: chunk %d cid=%u dead, aborting retries\n",
                                  chunkNum, TCP_UPLOAD_CID);
                    break;
                }
                if (r < CHUNK_MAX_RETRIES - 1) {
                    unsigned long backoff = CHUNK_RETRY_BASE_MS << r;  // exponential
                    if (backoff > CHUNK_RETRY_MAX_MS) backoff = CHUNK_RETRY_MAX_MS;
                    vTaskDelay(pdMS_TO_TICKS(backoff));
                }
            }
            chunksTotal++;
            if (chunkOk && retryCount > 0) chunksWithRetry++;
            if (!chunkOk) {
                LOG_PRINTF("CatM1Modem: HTTP body chunk %d failed at %u/%u (after %d retries)\n",
                              chunkNum, (unsigned int)offset, (unsigned int)bodyLen, CHUNK_MAX_RETRIES);
                goto fail;
            }

            offset += chunkSize;
            chunkNum++;
            if (chunkNum % 10 == 0) {
                LOG_PRINTF("CatM1Modem: HTTP send %u/%u bytes\n",
                              (unsigned int)offset, (unsigned int)bodyLen);
            }
        }
        LOG_PRINTF("CatM1Modem: HTTP body sent (%d chunks, %u bytes)\n",
                      chunkNum, (unsigned int)bodyLen);
    }

    // Allow modem TCP stack to flush send buffer before reading response.
    // AT+CASEND OK means data accepted into buffer, NOT delivered over TCP.
    // Cat-M1 uplink latency requires a brief wait to avoid partial delivery.
    vTaskDelay(pdMS_TO_TICKS(HTTP_TCP_FLUSH_DELAY_MS));

    // Read response and check status code
    {
        // Dynamic timeout: base + body transfer time at pessimistic throughput
        unsigned long httpRespTimeout = HTTP_RESP_BASE_TIMEOUT_MS
                                      + (bodyLen * 1000UL) / HTTP_RESP_BYTES_PER_SEC;
        if (httpRespTimeout > HTTP_RESP_MAX_TIMEOUT_MS)
            httpRespTimeout = HTTP_RESP_MAX_TIMEOUT_MS;
        LOG_PRINTF("CatM1Modem: HTTP response timeout set to %lu ms\n", httpRespTimeout);

        char response[HTTP_RESPONSE_BUF_SIZE];
        int idx = 0;
        int lineStart = 0;        // start of current incomplete line
        response[0] = '\0';
        unsigned long startTime = millis();
        bool gotStatus = false;
        bool httpOk = false;

        while (millis() - startTime < httpRespTimeout) {
            if (serial_->available()) {
                char c = serial_->read();

                // Buffer full — compact by discarding completed lines
                if (idx >= (int)sizeof(response) - 1) {
                    if (lineStart > 0) {
                        int remain = idx - lineStart;
                        memmove(response, response + lineStart, remain);
                        idx = remain;
                        lineStart = 0;
                        response[idx] = '\0';
                    } else {
                        // Single line exceeds buffer — discard oldest half
                        int half = idx / 2;
                        memmove(response, response + half, idx - half);
                        idx -= half;
                        lineStart = 0;
                        response[idx] = '\0';
                    }
                }

                response[idx++] = c;
                response[idx] = '\0';

                // On newline, check if completed line is a URC (starts with '+')
                if (c == '\n') {
                    // Check if line from lineStart is a URC
                    int ls = lineStart;
                    // Skip leading \r\n
                    while (ls < idx && (response[ls] == '\r' || response[ls] == '\n')) ls++;
                    if (ls < idx && response[ls] == '+') {
                        // URC detected — discard this line
                        idx = lineStart;
                        response[idx] = '\0';
                    } else {
                        lineStart = idx;  // advance to next line
                    }
                }

                if (!gotStatus) {
                    char* http = strstr(response, "HTTP/1.");
                    if (http) {
                        char* space = strchr(http, ' ');
                        if (space && strlen(space + 1) >= 3) {
                            int statusCode = atoi(space + 1);
                            if (statusCode >= 200 && statusCode < 300) {
                                httpOk = true;
                            }
                            LOG_PRINTF("CatM1Modem: HTTP response %d\n", statusCode);
                            gotStatus = true;
                        }
                    }
                }
                if (gotStatus) break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!gotStatus) {
            LOG_PRINTLN("CatM1Modem: HTTP response timeout");
            httpOk = false;
            // Connection likely dead — mark persistent connection as broken
            if (!ownConnection) httpConnected_ = false;
        }

        // Always drain remaining response data to prevent leftover bytes from
        // polluting subsequent AT command parsing. On keep-alive connections
        // the server may still be sending response body after we got the status line.
        {
            unsigned long drainStart = millis();
            unsigned long drainLimit = millis();
            while ((millis() - drainStart < DRAIN_IDLE_MS) && (millis() - drainLimit < DRAIN_HARD_LIMIT_MS)) {
                if (serial_->available()) {
                    serial_->read();
                    drainStart = millis();  // reset idle timer on activity
                } else {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
        }

        if (ownConnection || !httpOk) {
            char closeCmd[CMD_BUF_SIZE];
            snprintf(closeCmd, sizeof(closeCmd), "AT+CACLOSE=%u\r\n", TCP_UPLOAD_CID);
            sendATCommand(closeCmd, TIMEOUT_AT_BASIC_MS);
            httpConnected_ = false;
        } else if (httpOk) {
            httpLastActivityMs_ = millis();
        }

        recordPostResult(httpOk, chunksTotal, chunksWithRetry);
        return httpOk;
    }

fail:
    if (ownConnection) {
        char closeCmd[CMD_BUF_SIZE];
        snprintf(closeCmd, sizeof(closeCmd), "AT+CACLOSE=%u\r\n", TCP_UPLOAD_CID);
        sendATCommand(closeCmd, TIMEOUT_AT_BASIC_MS);
    } else {
        // Persistent connection is broken — mark as closed
        httpConnected_ = false;
    }
    recordPostResult(false, chunksTotal, chunksWithRetry);
    return false;
}

size_t CatM1Modem::currentChunkSize() const {
    return HTTP_PROFILES[(int)currentProfile_].chunkSize;
}

size_t CatM1Modem::currentWatermarkBytes() const {
    return HTTP_PROFILES[(int)currentProfile_].watermarkBytes;
}

void CatM1Modem::recordPostResult(bool success, size_t total, size_t retried) {
    HttpProfile prev = currentProfile_;
    int idx = (int)currentProfile_;
    // Integer comparison to avoid double arithmetic on ESP32 hot path:
    //   errorRate >= 10% ⇔ retried * 10 >= total
    bool highError = (total > 0) && (retried * 10 >= total);
    bool zeroError = (total > 0) && (retried == 0);

    if (!success || highError) {
        idx = std::min(idx + 1, (int)config_.httpSlowestProfile);
        successStreak_ = 0;
    } else if (zeroError) {
        if (++successStreak_ >= 2) {
            idx = std::max(idx - 1, (int)config_.httpFastestProfile);
            successStreak_ = 0;
        }
    } else {
        successStreak_ = 0;
    }

    currentProfile_ = (HttpProfile)idx;
    if (prev != currentProfile_) {
        // Use integer-percent format to match the integer comparison above.
        unsigned ratePct = total > 0 ? (unsigned)((retried * 100) / total) : 0;
        LOG_PRINTF("CatM1Modem: HTTP profile %s -> %s (chunks=%u, retried=%u, errRate=%u%%)\n",
                      HTTP_PROFILES[(int)prev].name,
                      HTTP_PROFILES[(int)currentProfile_].name,
                      (unsigned)total, (unsigned)retried, ratePct);
    }
}
