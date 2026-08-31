#pragma once
#include <cstdint>
#include <cstddef>
#include <HardwareSerial.h>

/// HTTP POST adaptive tuning profile.
/// Lower index = more aggressive (larger chunks, higher in-flight buffer occupancy).
/// Profile parameters are defined in HTTP_PROFILES[] (CatM1Modem_Internal.h).
enum class HttpProfile : uint8_t {
    FAST    = 0,   // max in-flight 5555B — requires good RF
    NORMAL  = 1,   // max in-flight 5120B — field-tested baseline
    SAFE    = 2,   // max in-flight 3840B
    SAFEST  = 3,   // max in-flight 2560B — most conservative
};

class CatM1Modem {
public:
    enum class SendResult { SUCCESS, RECOVERED, FAILED };

    struct Config {
        int8_t rxPin;
        int8_t txPin;
        uint32_t baudRate = 115200;
        uint32_t targetBaudRate = 460800;  // negotiated after init
        uint8_t uartNum = 2;        // UART2
        const char* apn = "soracom.io";
        const char* user = "sora";
        const char* pass = "sora";
        uint8_t authType = 1;           // 0=none, 1=PAP, 2=CHAP
        const char* host = "funnel.soracom.io";
        uint16_t port = 23080;
        const char* protocol = "UDP";
        // TCP upload settings (raw binary, e.g. Soracom Beam)
        const char* tcpHost = "beam.soracom.io";
        uint16_t tcpPort = 23080;
        // HTTP upload settings (manual HTTP over raw TCP)
        const char* httpHost = "uni.soracom.io";
        uint16_t httpPort = 8888;
        const char* httpPath = "/";
        const char* catmBands = "1,3,18,19,26";  // AT+CBANDCFG bands (KDDI+docomo, 700MHz帯除外)
        // HTTP POST adaptive tuning (used by httpPost only; tcpSend unaffected).
        HttpProfile httpInitialProfile  = HttpProfile::NORMAL;  // startup value
        HttpProfile httpFastestProfile  = HttpProfile::FAST;    // upper bound (cannot exceed)
        HttpProfile httpSlowestProfile  = HttpProfile::SAFEST;  // lower bound (cannot exceed)
    };

    bool begin(const Config& config);

    /// Passively wait for modem to finish cold boot, then verify with gentle AT probe.
    /// SIM7080G sends URCs ("RDY", "+CPIN: READY") during boot — sending AT commands
    /// before boot completes can interfere with the startup sequence.
    /// Returns true if modem is responsive.
    bool waitForReady(unsigned long timeoutMs = 60000);

    /// Initialize modem: AT check, network attach, APN, socket open
    bool init();

    /// Send binary data over the open socket
    bool send(const uint8_t* data, size_t length);

    /// Send with automatic 3-level retry/recovery
    /// Returns SUCCESS on first-try success, RECOVERED if recovery was needed, FAILED after max retries
    SendResult sendWithRetry(const uint8_t* data, size_t length);

    /// Send binary data via raw TCP socket (supports large payloads >4KB)
    bool tcpSend(const uint8_t* data, size_t length);

    /// Open persistent TCP connection for HTTP (call before multiple httpPost calls)
    /// @note Not thread-safe. All HTTP methods (httpOpen/httpClose/httpPost) must be
    ///       called from the same task or externally serialized.
    bool httpOpen();
    /// Close persistent TCP connection for HTTP
    void httpClose();

    /// Send HTTP POST over raw TCP (no size limit, uses Content-Length).
    /// If httpOpen() was called beforehand, reuses that connection with keep-alive.
    /// Otherwise opens and closes a connection per call.
    /// @note Not thread-safe — see httpOpen().
    bool httpPost(const char* contentType, const uint8_t* body, size_t bodyLen);

    /// Log modem status for debugging (COPS, APN, socket state, signal quality)
    void logModemStatus();

    /// Verify network health (registration + PDP) and recover if needed.
    /// Call before each upload cycle to handle idle PDP expiry.
    /// Uses tiered recovery: L2 (recoverNetwork) → L3 (recoverModemFull) → L4 (recoverModemDeep).
    bool ensureConnected();

    /// Recovery levels
    bool recoverSocket();    // L1: Close and reopen socket
    bool recoverNetwork();   // L2: Reconnect PDP + socket
    bool recoverModemFull(); // L3: Full modem reset + reconnect
    bool recoverModemDeep(); // L4: Clear network info + full reset

private:
    bool isPdpActive();
    bool sendATCommand(const char* command, unsigned long timeoutMs = 10000);
    bool waitForRegistration(int maxAttempts, unsigned long intervalMs);
    bool openSocket();
    bool openConnection(uint8_t cid, const char* type, const char* host, uint16_t port);
    bool sendOnCid(uint8_t cid, const uint8_t* data, size_t length);
    bool checkCidConnected(uint8_t cid);
    /// Query AT+CAACK=<cid> to read sent/ack/nack byte counters.
    /// @param[out] txLen  total bytes handed to modem via CASEND for this cid
    /// @param[out] ackLen bytes acknowledged by remote TCP peer
    /// @param[out] nackLen bytes still pending in modem TCP send buffer (un-ACKed)
    /// @return true if parsed successfully. Used for diagnosing buffer-fill issues.
    bool queryUnackedBytes(uint8_t cid, unsigned long& txLen,
                           unsigned long& ackLen, unsigned long& nackLen);
    /// Block until SIM7080G TCP send buffer drains below current profile watermark.
    /// Polls AT+CAACK every BUFFER_DRAIN_POLL_MS, gives up after BUFFER_DRAIN_TIMEOUT_MS.
    /// Falls back to fixed CHUNK_PACING_MS delay if CAACK query itself fails.
    /// Used between body chunks to prevent +CME ERROR from buffer overflow.
    void waitForSendBufferDrain(uint8_t cid);
    bool waitForPdpActive(unsigned long timeoutMs);
    bool activatePdp();  // AT+CNACT=0,1 with retry
    void flushRx();
    void flushRxDeep();  // longer idle/hard timeout for post-HTTP cleanup
    bool negotiateBaudRate();
    void configureCatM1Mode();  // CAT-M1 fixed mode + debug URC settings
    /// Return current chunkSize from HTTP_PROFILES[currentProfile_].
    size_t currentChunkSize() const;
    /// Return current watermarkBytes from HTTP_PROFILES[currentProfile_].
    size_t currentWatermarkBytes() const;
    /// Update currentProfile_ based on POST result.
    /// success=true with errorRate=0% for 2 consecutive calls upgrades one step;
    /// success=false OR errorRate>=10% downgrades one step. Bounded by Config.
    void   recordPostResult(bool success, size_t chunksTotal, size_t chunksWithRetry);

    HardwareSerial* serial_ = nullptr;
    Config config_ = {};
    bool httpConnected_ = false;           // persistent HTTP TCP connection state
    unsigned long httpLastActivityMs_ = 0; // millis() of last successful httpPost
    HttpProfile currentProfile_ = HttpProfile::NORMAL;  // active HTTP profile (set in init())
    uint8_t     successStreak_  = 0;                    // consecutive 0%-error POST count
};