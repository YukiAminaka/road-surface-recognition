#include "SDCSVWriter.h"
#include <Arduino.h>
#include <SerialLog.h>

#ifdef SDCSVWRITER_ENABLE_LZ4
#include <lz4frame.h>

// LZ4 frame preferences (固定設定)
static const LZ4F_preferences_t kLZ4Prefs = {
    /* frameInfo */ {
        /* blockSizeID         */ LZ4F_max64KB,
        /* blockMode           */ LZ4F_blockLinked,
        /* contentChecksumFlag */ LZ4F_contentChecksumEnabled,
        /* frameType           */ LZ4F_frame,
        /* contentSize         */ 0,
        /* dictID              */ 0,
        /* blockChecksumFlag   */ LZ4F_noBlockChecksum,
    },
    /* compressionLevel */ 0,
    // autoFlush=0: 64KB ブロックが埋まるまで cctx 内にバッファして圧縮率を最大化。
    // 旧 autoFlush=1 では 56B のレコード毎にミニブロックが吐かれて
    // ブロックヘッダオーバーヘッドが圧縮率を悪化させていた。
    /* autoFlush        */ 0,
    /* favorDecSpeed    */ 0,
};

// LZ4F_compressBound の上限値(1 回の compressUpdate / End あたり)
static constexpr size_t kLZ4MaxInputPerUpdate = 64 * 1024;
#endif  // SDCSVWRITER_ENABLE_LZ4

SDCSVWriter::~SDCSVWriter() {
#ifdef SDCSVWRITER_ENABLE_LZ4
    if (ready_ && mode_ == WriteMode::LZ4 && lz4FrameOpen_) {
        // Close current frame before closing file
        size_t n = LZ4F_compressEnd(
            static_cast<LZ4F_cctx*>(lz4Cctx_),
            lz4OutBuf_, lz4OutBufCap_, NULL);
        if (!LZ4F_isError(n) && n > 0) {
            file_.write(lz4OutBuf_, n);
        }
        lz4FrameOpen_ = false;
    }
    lz4Cleanup_();
#endif
    if (ready_) {
        file_.flush();
        file_.close();
        ready_ = false;
    }
}

bool SDCSVWriter::begin(int csPin, const char* filename) {
    if (!initSD(csPin)) {
        return false;
    }

    file_ = sd_.open(filename, O_RDWR | O_CREAT | O_AT_END);
    if (!file_) {
        LOG_PRINTF("SDCSVWriter: file open failed: %s\r\n", filename);
        ready_ = false;
        return false;
    }

    LOG_PRINTF("SDCSVWriter: opened %s for writing\r\n", filename);
    lineCount_ = 0;
    lastFlushMs_ = millis();
    ready_ = true;
    return true;
}

bool SDCSVWriter::initSD(int csPin) {
    if (!sd_.begin(csPin)) {
        LOG_PRINTLN("SDCSVWriter: SD card init failed!");
        sdInitialized_ = false;
        ready_ = false;
        return false;
    }
    LOG_PRINTLN("SDCSVWriter: SD card initialized");
    sdInitialized_ = true;
    return true;
}

bool SDCSVWriter::initSD(const SPIPins& pins) {
    spi_.begin(pins.sck, pins.miso, pins.mosi, pins.cs);
    SdSpiConfig config(pins.cs, SHARED_SPI, SD_SCK_MHZ(4), &spi_);
    if (!sd_.begin(config)) {
        {
            SerialLogGuard guard;
            LOG_PRINTLN("SDCSVWriter: SD card init failed!");
            LOG_PRINTF("  Check wiring: MOSI=GPIO%d, MISO=GPIO%d, SCK=GPIO%d, CS=GPIO%d\r\n",
                          pins.mosi, pins.miso, pins.sck, pins.cs);
        }
        sdInitialized_ = false;
        ready_ = false;
        return false;
    }
    LOG_PRINTLN("SDCSVWriter: SD card initialized");
    sdInitialized_ = true;
    return true;
}

bool SDCSVWriter::openWithHeader(const char* dirPath, const char* filename,
                                 const char* header) {
    if (!sdInitialized_) {
        LOG_PRINTLN("SDCSVWriter: SD not initialized");
        return false;
    }

    if (!mkdirRecursive(dirPath)) {
        LOG_PRINTF("SDCSVWriter: mkdir failed: %s\r\n", dirPath);
        return false;
    }

    // Build full path: dirPath/filename
    char fullPath[128];
    int pathLen = snprintf(fullPath, sizeof(fullPath), "%s/%s", dirPath, filename);
    if (pathLen < 0 || pathLen >= (int)sizeof(fullPath)) {
        LOG_PRINTF("SDCSVWriter: path too long: %s/%s\r\n", dirPath, filename);
        ready_ = false;
        return false;
    }
    strncpy(currentPath_, fullPath, sizeof(currentPath_) - 1);
    currentPath_[sizeof(currentPath_) - 1] = '\0';

    file_ = sd_.open(fullPath, O_RDWR | O_CREAT | O_AT_END);
    if (!file_) {
        LOG_PRINTF("SDCSVWriter: file open failed: %s\r\n", fullPath);
        ready_ = false;
        return false;
    }

    // Write header if the file is new (empty)
    if (file_.fileSize() == 0) {
        file_.println(header);
    }

    LOG_PRINTF("SDCSVWriter: opened %s for writing\r\n", fullPath);
    lineCount_ = 0;
    lastFlushMs_ = millis();
    ready_ = true;
    return true;
}

bool SDCSVWriter::rotate(const char* dirPath, const char* filename,
                         const char* header) {
    if (ready_) {
        file_.flush();
        file_.close();
        ready_ = false;
    }
    return openWithHeader(dirPath, filename, header);
}

bool SDCSVWriter::openBinary(const char* dirPath, const char* filename) {
    if (!sdInitialized_) {
        LOG_PRINTLN("SDCSVWriter: SD not initialized");
        return false;
    }

    if (!mkdirRecursive(dirPath)) {
        LOG_PRINTF("SDCSVWriter: mkdir failed: %s\r\n", dirPath);
        return false;
    }

    char fullPath[128];
    int pathLen = snprintf(fullPath, sizeof(fullPath), "%s/%s", dirPath, filename);
    if (pathLen < 0 || pathLen >= (int)sizeof(fullPath)) {
        LOG_PRINTF("SDCSVWriter: path too long: %s/%s\r\n", dirPath, filename);
        ready_ = false;
        return false;
    }
    strncpy(currentPath_, fullPath, sizeof(currentPath_) - 1);
    currentPath_[sizeof(currentPath_) - 1] = '\0';

    file_ = sd_.open(fullPath, O_RDWR | O_CREAT | O_AT_END);
    if (!file_) {
        LOG_PRINTF("SDCSVWriter: file open failed: %s\r\n", fullPath);
        ready_ = false;
        return false;
    }

    LOG_PRINTF("SDCSVWriter: opened %s for binary writing\r\n", fullPath);
    lineCount_ = 0;
    lastFlushMs_ = millis();
    ready_ = true;
    return true;
}

bool SDCSVWriter::rotateBinary(const char* dirPath, const char* filename) {
    if (ready_) {
        file_.flush();
        file_.close();
        ready_ = false;
    }
    return openBinary(dirPath, filename);
}

bool SDCSVWriter::writeRaw(const void* data, size_t len, bool count) {
    if (!ready_) {
        return false;
    }
#ifdef SDCSVWRITER_ENABLE_LZ4
    if (mode_ == WriteMode::LZ4) {
        if (!lz4FrameOpen_) return false;
        if (!lz4WriteCompressed_(data, len)) return false;
        if (count) lineCount_++;
        return true;
    }
#endif
    size_t written = file_.write(static_cast<const uint8_t*>(data), len);
    if (written == len) {
        if (count) lineCount_++;
        return true;
    }
    return false;
}

bool SDCSVWriter::saveRawFile(const char* dirPath, const char* filename,
                              const uint8_t* data, uint32_t length) {
    if (!sdInitialized_) {
        LOG_PRINTLN("SDCSVWriter: SD not initialized");
        return false;
    }

    if (!mkdirRecursive(dirPath)) {
        LOG_PRINTF("SDCSVWriter: mkdir failed: %s\r\n", dirPath);
        return false;
    }

    char fullPath[128];
    int pathLen = snprintf(fullPath, sizeof(fullPath), "%s/%s", dirPath, filename);
    if (pathLen < 0 || pathLen >= (int)sizeof(fullPath)) {
        LOG_PRINTF("SDCSVWriter: path too long: %s/%s\r\n", dirPath, filename);
        return false;
    }

    File f = sd_.open(fullPath, O_RDWR | O_CREAT | O_TRUNC);
    if (!f) {
        LOG_PRINTF("SDCSVWriter: file open failed: %s\r\n", fullPath);
        return false;
    }

    size_t written = f.write(data, length);
    f.close();

    if (written != length) {
        LOG_PRINTF("SDCSVWriter: write error: %u/%u bytes\r\n", (unsigned)written, (unsigned)length);
        return false;
    }

    LOG_PRINTF("SDCSVWriter: saved %s (%u bytes)\r\n", fullPath, (unsigned)length);
    return true;
}

bool SDCSVWriter::mkdirRecursive(const char* path) {
    char buf[128];
    size_t len = strlen(path);
    if (len == 0 || len >= sizeof(buf)) {
        return false;
    }
    memcpy(buf, path, len + 1);

    for (size_t i = 1; i <= len; i++) {
        if (buf[i] == '/' || buf[i] == '\0') {
            char saved = buf[i];
            buf[i] = '\0';
            if (sd_.exists(buf)) {
                // Verify it is actually a directory, not a file.
                // Some SD cards create a file instead of a directory.
                File f = sd_.open(buf);
                if (f) {
                    bool isDir = f.isDirectory();
                    f.close();
                    if (!isDir) {
                        LOG_PRINTF("SDCSVWriter: %s exists as file, replacing with directory\r\n", buf);
                        sd_.remove(buf);
                        if (!sd_.mkdir(buf)) {
                            return false;
                        }
                    }
                }
            } else {
                if (!sd_.mkdir(buf)) {
                    return false;
                }
            }
            buf[i] = saved;
        }
    }
    return true;
}

bool SDCSVWriter::writeLine(const char* csvLine) {
    if (!ready_) {
        return false;
    }
    size_t expectedLen = strlen(csvLine) + 2;  // +2 for \r\n
    if (file_.println(csvLine) != expectedLen) {
        return false;
    }
    lineCount_++;
    return true;
}

void SDCSVWriter::flushIfNeeded() {
    if (!ready_) {
        return;
    }

    const unsigned long now = millis();
    const bool earlyPhase = (lineCount_ <= kEarlyFlushThreshold);
    const bool periodicLine = (lineCount_ % kPeriodicFlushInterval == 0);
    const bool timeoutElapsed = ((now - lastFlushMs_) > kFlushTimeoutMs);

    if (!(earlyPhase || periodicLine || timeoutElapsed)) return;

#ifdef SDCSVWRITER_ENABLE_LZ4
    // LZ4 モードでは LZ4 フレームは閉じず、cctx 内にバッファされた未完了
    // ブロックを LZ4F_flush で吐き出してから SD に sync する。フレーム境界は
    // ファイルローテーション (rotateBinaryLZ4) と終了 (destructor) でのみ
    // 生成する。ブロックヘッダ ~4B/回 の犠牲で耐久性窓を 1 ブロックぶんに
    // 抑える(autoFlush=0 のままだと最大 ~64KB 入力ぶん未エミットになる)。
    if (mode_ == WriteMode::LZ4) {
        lz4FlushBlock_();
        file_.flush();
        lastFlushMs_ = now;
        return;
    }
#endif

    file_.flush();
    lastFlushMs_ = now;
}

void SDCSVWriter::flush() {
    if (!ready_) {
        return;
    }
#ifdef SDCSVWRITER_ENABLE_LZ4
    if (mode_ == WriteMode::LZ4) {
        // cctx 内にバッファされた未エミットブロックを吐き出してから SD に sync。
        // フレームは閉じない(圧縮率維持のため)。
        lz4FlushBlock_();
        file_.flush();
        lastFlushMs_ = millis();
        return;
    }
#endif
    file_.flush();
    lastFlushMs_ = millis();
}

bool SDCSVWriter::isReady() const {
    return ready_;
}

void SDCSVWriter::printCardInfo() {
    if (!sdInitialized_) {
        LOG_PRINTLN("SDCSVWriter: SD not initialized");
        return;
    }
    uint32_t clusterCount = sd_.clusterCount();
    uint32_t freeClusterCount = sd_.freeClusterCount();
    uint32_t sectorsPerCluster = sd_.sectorsPerCluster();
    uint32_t totalMB = (uint32_t)((uint64_t)clusterCount * sectorsPerCluster / 2048);
    uint32_t freeMB = (uint32_t)((uint64_t)freeClusterCount * sectorsPerCluster / 2048);
    LOG_PRINTF("SDCSVWriter: SD card %lu MB total, %lu MB free (%lu%% used)\r\n",
                  (unsigned long)totalMB, (unsigned long)freeMB,
                  totalMB > 0 ? (unsigned long)((totalMB - freeMB) * 100 / totalMB) : 0UL);
}

// ===========================================================================
// LZ4 mode methods
// ===========================================================================

#ifdef SDCSVWRITER_ENABLE_LZ4

bool SDCSVWriter::openBinaryLZ4(const char* dirPath, const char* filename) {
    if (!sdInitialized_) {
        LOG_PRINTLN("SDCSVWriter: SD not initialized");
        return false;
    }

    if (!mkdirRecursive(dirPath)) {
        LOG_PRINTF("SDCSVWriter: mkdir failed: %s\r\n", dirPath);
        return false;
    }

    char fullPath[128];
    int pathLen = snprintf(fullPath, sizeof(fullPath), "%s/%s", dirPath, filename);
    if (pathLen < 0 || pathLen >= (int)sizeof(fullPath)) {
        LOG_PRINTF("SDCSVWriter: path too long: %s/%s\r\n", dirPath, filename);
        ready_ = false;
        return false;
    }
    strncpy(currentPath_, fullPath, sizeof(currentPath_) - 1);
    currentPath_[sizeof(currentPath_) - 1] = '\0';

    file_ = sd_.open(fullPath, O_RDWR | O_CREAT | O_AT_END);
    if (!file_) {
        LOG_PRINTF("SDCSVWriter: file open failed: %s\r\n", fullPath);
        ready_ = false;
        return false;
    }

    // Allocate LZ4 cctx (once; reused across rotateBinaryLZ4 calls)
    if (lz4Cctx_ == nullptr) {
        LZ4F_cctx* ctx = nullptr;
        LZ4F_errorCode_t err = LZ4F_createCompressionContext(&ctx, LZ4F_VERSION);
        if (LZ4F_isError(err) || ctx == nullptr) {
            LOG_PRINTF("SDCSVWriter: LZ4 cctx alloc failed: %s\r\n",
                          LZ4F_getErrorName(err));
            file_.close();
            ready_ = false;
            return false;
        }
        lz4Cctx_ = ctx;
    }

    // Allocate output buffer in PSRAM
    if (lz4OutBuf_ == nullptr) {
        lz4OutBufCap_ = LZ4F_compressBound(kLZ4MaxInputPerUpdate, &kLZ4Prefs);
        if (lz4OutBufCap_ < LZ4F_HEADER_SIZE_MAX) {
            lz4OutBufCap_ = LZ4F_HEADER_SIZE_MAX;
        }
        lz4OutBuf_ = (uint8_t*)ps_malloc(lz4OutBufCap_);
        if (lz4OutBuf_ == nullptr) {
            LOG_PRINTF("SDCSVWriter: LZ4 out buf alloc failed (%u bytes)\r\n",
                          (unsigned)lz4OutBufCap_);
            file_.close();
            lz4Cleanup_();
            ready_ = false;
            return false;
        }
    }

    // Begin first frame
    size_t n = LZ4F_compressBegin(static_cast<LZ4F_cctx*>(lz4Cctx_),
                                  lz4OutBuf_, lz4OutBufCap_, &kLZ4Prefs);
    if (LZ4F_isError(n)) {
        LOG_PRINTF("SDCSVWriter: LZ4F_compressBegin failed: %s\r\n",
                      LZ4F_getErrorName(n));
        file_.close();
        ready_ = false;
        return false;
    }
    if (n > 0) {
        size_t w = file_.write(lz4OutBuf_, n);
        if (w != n) {
            LOG_PRINTF("SDCSVWriter: short write on frame header: %u/%u\r\n",
                          (unsigned)w, (unsigned)n);
            file_.close();
            ready_ = false;
            return false;
        }
    }

    LOG_PRINTF("SDCSVWriter: opened %s for LZ4 binary writing\r\n", fullPath);
    mode_ = WriteMode::LZ4;
    lz4FrameOpen_ = true;
    lineCount_ = 0;
    lastFlushMs_ = millis();
    ready_ = true;
    return true;
}

bool SDCSVWriter::rotateBinaryLZ4(const char* dirPath, const char* filename) {
    if (ready_ && mode_ == WriteMode::LZ4 && lz4FrameOpen_) {
        // Close current frame on old file
        size_t n = LZ4F_compressEnd(static_cast<LZ4F_cctx*>(lz4Cctx_),
                                    lz4OutBuf_, lz4OutBufCap_, NULL);
        if (!LZ4F_isError(n) && n > 0) {
            file_.write(lz4OutBuf_, n);
        }
        lz4FrameOpen_ = false;
        file_.flush();
        file_.close();
        ready_ = false;
    } else if (ready_) {
        file_.flush();
        file_.close();
        ready_ = false;
    }
    return openBinaryLZ4(dirPath, filename);
}

bool SDCSVWriter::lz4FlushBlock_() {
    if (!lz4FrameOpen_) return false;
    // cctx 内にバッファされた未完了ブロックがあれば吐き出す。
    // フレームヘッダ/endmark は生成しない(=フレームは閉じない)ので
    // LZ4F_compressUpdate を続けて呼び出せる。
    size_t n = LZ4F_flush(static_cast<LZ4F_cctx*>(lz4Cctx_),
                          lz4OutBuf_, lz4OutBufCap_, NULL);
    if (LZ4F_isError(n)) {
        // lz4frame API 仕様: LZ4F_isError を返した cctx は次回 LZ4F_compressBegin
        // までは indeterminate。fail-closed にして以降の書込みを止める。
        LOG_PRINTF("SDCSVWriter: LZ4F_flush failed: %s\r\n",
                      LZ4F_getErrorName(n));
        ready_ = false;
        lz4FrameOpen_ = false;
        return false;
    }
    if (n > 0) {
        size_t w = file_.write(lz4OutBuf_, n);
        if (w != n) {
            // 部分書込みでファイルは中途半端な状態。cctx 自体は coherent でも
            // SD 側がブロック途中で切れたので、フレーム全体が破損扱い。
            LOG_PRINTF("SDCSVWriter: short write on lz4 flush: %u/%u\r\n",
                          (unsigned)w, (unsigned)n);
            ready_ = false;
            lz4FrameOpen_ = false;
            return false;
        }
    }
    return true;
}

bool SDCSVWriter::lz4WriteCompressed_(const void* data, size_t len) {
    // BinRecord 1 件は 56B、BinFileHeader は 8B → 常に kLZ4MaxInputPerUpdate(=64KB)以下
    if (len > kLZ4MaxInputPerUpdate) {
        LOG_PRINTF("SDCSVWriter: LZ4 write too large (%u > %u)\r\n",
                      (unsigned)len, (unsigned)kLZ4MaxInputPerUpdate);
        return false;
    }
    size_t n = LZ4F_compressUpdate(static_cast<LZ4F_cctx*>(lz4Cctx_),
                                   lz4OutBuf_, lz4OutBufCap_,
                                   data, len, NULL);
    if (LZ4F_isError(n)) {
        LOG_PRINTF("SDCSVWriter: LZ4F_compressUpdate failed: %s\r\n",
                      LZ4F_getErrorName(n));
        ready_ = false;
        lz4FrameOpen_ = false;
        return false;
    }
    if (n > 0) {
        size_t w = file_.write(lz4OutBuf_, n);
        if (w != n) {
            LOG_PRINTF("SDCSVWriter: short write on compressed data: %u/%u\r\n",
                          (unsigned)w, (unsigned)n);
            ready_ = false;
            lz4FrameOpen_ = false;
            return false;
        }
    }
    return true;
}

void SDCSVWriter::lz4Cleanup_() {
    if (lz4Cctx_) {
        LZ4F_freeCompressionContext(static_cast<LZ4F_cctx*>(lz4Cctx_));
        lz4Cctx_ = nullptr;
    }
    if (lz4OutBuf_) {
        free(lz4OutBuf_);
        lz4OutBuf_ = nullptr;
    }
    lz4OutBufCap_ = 0;
    lz4FrameOpen_ = false;
    mode_ = WriteMode::RAW;
}

#else  // SDCSVWRITER_ENABLE_LZ4 not defined

bool SDCSVWriter::openBinaryLZ4(const char* /*dirPath*/, const char* /*filename*/) {
    LOG_PRINTLN("SDCSVWriter: LZ4 mode not compiled in (define SDCSVWRITER_ENABLE_LZ4)");
    return false;
}

bool SDCSVWriter::rotateBinaryLZ4(const char* /*dirPath*/, const char* /*filename*/) {
    LOG_PRINTLN("SDCSVWriter: LZ4 mode not compiled in (define SDCSVWRITER_ENABLE_LZ4)");
    return false;
}

bool SDCSVWriter::lz4FlushBlock_()           { return false; }
bool SDCSVWriter::lz4WriteCompressed_(const void*, size_t) { return false; }
void SDCSVWriter::lz4Cleanup_()               { /* nothing to clean up */ }

#endif  // SDCSVWRITER_ENABLE_LZ4