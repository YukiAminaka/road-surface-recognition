#include "CatM1Modem.h"
#include "CatM1Modem_Internal.h"
#include <SerialLog.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void CatM1Modem::logModemStatus() {
    SerialLogGuard guard;
    LOG_PRINTLN("CatM1Modem: --- status ---");

    // Send each query and print the full response (not just OK/ERROR)
    const char* commands[] = {
        "AT+COPS?\r\n",    // 接続キャリア
        "AT+CGNAPN\r\n",   // 実効APN
        "AT+CSQ\r\n",      // 信号品質
        "AT+CASTATE?\r\n", // ソケット状態
        "AT+CNACT?\r\n",   // PDP状態
        "AT+CEREG?\r\n",   // 登録状態
        "AT+CACFG?\r\n",   // TCP送信バッファ等のソケット設定 (chunk 6固定失敗の原因切り分け用)
    };

    // AT+CACFG? returns ~600-800 bytes (per-cid × multiple params); a 256B
    // buffer would fill before "OK" arrives and trigger a 2s timeout while
    // holding modemMutex. Use a function-local 1KB buffer for diag only.
    static constexpr size_t DIAG_RESPONSE_BUF_SIZE = 1024;

    for (const char* cmd : commands) {
        flushRx();
        serial_->write(cmd);

        char response[DIAG_RESPONSE_BUF_SIZE];
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
                if (strstr(response, "OK") != NULL || strstr(response, "ERROR") != NULL) {
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Print response lines that contain '+' (informational URC/result lines)
        char* line = response;
        while (line && *line) {
            char* nl = strchr(line, '\n');
            if (nl) *nl = '\0';
            // Print lines starting with '+' (e.g. +COPS:, +CSQ:, +CNACT:)
            char* trimmed = line;
            while (*trimmed == '\r' || *trimmed == ' ') trimmed++;
            if (*trimmed == '+') {
                LOG_PRINT("  ");
                LOG_PRINTLN(trimmed);
            }
            if (nl) {
                *nl = '\n';
                line = nl + 1;
            } else {
                break;
            }
        }
    }

    LOG_PRINTLN("CatM1Modem: --- end status ---");
}
