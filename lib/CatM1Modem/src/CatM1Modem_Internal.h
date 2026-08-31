#pragma once
// Private header for CatM1Modem implementation files.
// Not part of the library's public API. Do not include from outside lib/CatM1Modem/src/.

#include <cstddef>
#include <cstdint>
#include <climits>

// --- Timeout constants (milliseconds) ---
static constexpr unsigned long TIMEOUT_AT_BASIC_MS     = 2000;
static constexpr unsigned long TIMEOUT_AT_SIM_MS       = 5000;
static constexpr unsigned long TIMEOUT_AT_CFUN_MS      = 10000;
static constexpr unsigned long TIMEOUT_AT_NETWORK_MS   = 75000;
static constexpr unsigned long TIMEOUT_AT_SOCKET_MS    = 60000;

// --- Retry constants ---
static constexpr int           MAX_AT_RETRIES          = 10;
static constexpr unsigned long RETRY_DELAY_MS          = 1000;
static constexpr unsigned long MODEM_RESET_WAIT_MS     = 20000;
static constexpr unsigned long MODEM_RETRY_DELAY_MS    = 2000;

// --- AT probe constants (TinyGSM-style fast probing) ---
static constexpr unsigned long AT_PROBE_RESP_MS        = 500;    // per-attempt response timeout (SIM7080G needs 300-400ms at boot)
static constexpr unsigned long AT_PROBE_INTERVAL_MS    = 100;    // gap between attempts
static constexpr unsigned long AT_PROBE_TIMEOUT_MS     = 10000;  // overall probe window

// --- PDP activation constants ---
static constexpr int           PDP_ACTIVATE_RETRIES    = 3;      // AT+CNACT retry count
static constexpr unsigned long TIMEOUT_AT_PDP_MS       = 90000;  // per-attempt PDP timeout (SIMCom: 60-150s)

// --- Send operation constants ---
static constexpr unsigned long SEND_PRE_DELAY_MS       = 20;
static constexpr unsigned long FLUSH_IDLE_MS           = 20;
static constexpr unsigned long FLUSH_MAX_MS            = 500;
static constexpr unsigned long FLUSH_DEEP_IDLE_MS      = 100;   // deep flush: wait longer for slow Cat-M1 data arrival
static constexpr unsigned long FLUSH_DEEP_MAX_MS       = 3000;  // deep flush: hard cap
static constexpr unsigned long DRAIN_IDLE_MS           = 2000;  // httpPost response drain (Cat-M1 inter-packet gap can exceed 500ms on slow links)
static constexpr unsigned long DRAIN_HARD_LIMIT_MS     = 10000; // absolute drain time cap
static constexpr unsigned long PROMPT_TIMEOUT_MS       = 3000;
static constexpr unsigned long SEND_CONFIRM_TIMEOUT_MS = 10000;

// --- Buffer sizes ---
static constexpr size_t        RESPONSE_BUF_SIZE       = 256;
static constexpr size_t        CMD_BUF_SIZE            = 128;
static constexpr size_t        SEND_CMD_BUF_SIZE       = 64;

// --- sendWithRetry constants ---
static constexpr int           RETRY_L1_THRESHOLD      = 3;
static constexpr int           RETRY_L2_THRESHOLD      = 6;
static constexpr int           RETRY_MAX_FAILURES      = 10;
static constexpr unsigned long RETRY_INTERVAL_MS       = 100;

// --- Baud rate recovery constants ---
static constexpr int           NVM_BAUD_RETRIES          = 3;      // quick NVM-path probe; intentionally lower than MAX_AT_RETRIES

// --- TCP upload constants ---
static constexpr uint8_t       TCP_UPLOAD_CID           = 1;      // TCP cid (separate from UDP cid=0)
static constexpr size_t        TCP_CHUNK_SIZE            = 1024;  // Reduced from 1459 (TCP MSS) on 2026-05-17. Field test confirmed chunk-6 (8754B = 6×1459B) stalls were structural SIM7080G ~8KB TCP send buffer overflow, not RF. 1024B chunks let 8 chunks fit in the buffer, giving Cat-M1 uplink more drain headroom.
static constexpr int           CHUNK_MAX_RETRIES         = 5;       // per-chunk retry; max backoff = 300+600+1200+2000+2000=6100ms (well under Soracom Beam TCP idle timeout ~60s)
static constexpr unsigned long CHUNK_RETRY_BASE_MS       = 300;    // initial backoff; doubles each retry (exponential)
static constexpr unsigned long CHUNK_RETRY_MAX_MS        = 2000;   // cap for exponential backoff (lower cap to keep per-chunk retry budget under 10s)
static constexpr unsigned long CHUNK_PACING_MS           = 300;    // fallback inter-chunk delay used only when waitForSendBufferDrain() cannot query CAACK (modem AT state transient). Normal path uses CAACK-driven adaptive pacing.

// --- CAACK-driven adaptive pacing constants ---
// Replaces unconditional CHUNK_PACING_MS with buffer-aware throttling.
// Watermark is now per-profile (see HTTP_PROFILES[].watermarkBytes,
// driven by CatM1Modem::currentWatermarkBytes()).
static constexpr unsigned long BUFFER_DRAIN_POLL_MS        = 100;   // re-query CAACK every 100ms while waiting
static constexpr unsigned long BUFFER_DRAIN_TIMEOUT_MS     = 5000;  // give up waiting (let chunk send + retry handle it)

// --- HTTP persistent connection constants ---
static constexpr unsigned long HTTP_CONN_IDLE_TIMEOUT_MS  = 60000;   // close & reopen if idle > 60s (SORACOM may drop idle PDP)

// --- HTTP response constants ---
static constexpr unsigned long HTTP_RESP_BASE_TIMEOUT_MS  = 15000;   // base: server processing time
static constexpr unsigned long HTTP_RESP_BYTES_PER_SEC    = 10000;   // Cat-M1 pessimistic uplink throughput
static constexpr unsigned long HTTP_RESP_MAX_TIMEOUT_MS   = 120000;  // hard cap: 2 minutes
static constexpr unsigned long HTTP_TCP_FLUSH_DELAY_MS    = 2000;    // post-body TCP flush wait
static constexpr size_t        HTTP_RESPONSE_BUF_SIZE     = 1024;    // HTTP response buffer

// --- HTTP POST adaptive profile table ---
// Index = HttpProfile value. Each profile sets the chunk size for AT+CASEND
// and the CAACK watermark used by waitForSendBufferDrain().
// max in-flight = chunkSize + watermarkBytes must stay <= ~5555B to avoid
// SIM7080G ~8KB TCP send buffer overflow (CAACK reading lag, see
// memory/project_sim7080g_buffer_headroom.md).
struct HttpProfileParams {
    size_t      chunkSize;
    size_t      watermarkBytes;
    const char* name;
};

static constexpr HttpProfileParams HTTP_PROFILES[] = {
    {1459, 4096, "FAST"  },  // 5555B in-flight — requires good RF
    {1024, 4096, "NORMAL"},  // 5120B in-flight — field-tested baseline
    { 768, 3072, "SAFE"  },  // 3840B in-flight
    { 512, 2048, "SAFEST"},  // 2560B in-flight — most conservative
};
static constexpr size_t HTTP_PROFILE_COUNT = sizeof(HTTP_PROFILES)/sizeof(HTTP_PROFILES[0]);

// Compile-time guard: ensure exponential backoff never overflows unsigned long (32-bit on ESP32).
// CHUNK_RETRY_BASE_MS << (CHUNK_MAX_RETRIES - 1) must fit in ULONG_MAX.
static_assert(CHUNK_RETRY_BASE_MS <= (ULONG_MAX >> (CHUNK_MAX_RETRIES - 1)),
              "CHUNK_RETRY_BASE_MS << (CHUNK_MAX_RETRIES-1) overflows unsigned long");
              