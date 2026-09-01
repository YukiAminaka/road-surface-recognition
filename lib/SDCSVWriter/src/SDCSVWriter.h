#pragma once
#include <cstdint>
#include <SPI.h>
#include <SdFat.h>

class SDCSVWriter {
public:
    struct SPIPins {
        int8_t mosi;
        int8_t miso;
        int8_t sck;
        int8_t cs;
    };

    ~SDCSVWriter();

    /// Initialize SD card and open file for writing
    /// @param csPin  SPI chip-select pin for SD card
    /// @param filename  File path to open (e.g. "2025-01-01.csv")
    /// @return true if SD init and file open succeeded
    bool begin(int csPin, const char* filename);

    /// Initialize only the SD card (without opening a file)
    /// @param csPin  SPI chip-select pin for SD card (uses default SPI bus)
    /// @return true if SD init succeeded
    bool initSD(int csPin);

    /// Initialize SD card with explicit SPI pin configuration
    /// @param pins  SPI pin mapping (MOSI, MISO, SCK, CS)
    /// @return true if SD init succeeded
    bool initSD(const SPIPins& pins);

    /// Create directory path, open file, and write header if file is new
    /// @param dirPath   Directory path to create (e.g. "2025/01/15")
    /// @param filename  File name within dirPath (e.g. "data.csv")
    /// @param header    CSV header line to write if file is new
    /// @return true if directory creation and file open succeeded
    /// @note SD must be initialized first via begin() or initSD()
    bool openWithHeader(const char* dirPath, const char* filename,
                        const char* header);

    /// Flush, close current file, and open a new file with header
    /// @param dirPath   Directory path to create (e.g. "2025/01/15")
    /// @param filename  File name within dirPath (e.g. "data.csv")
    /// @param header    CSV header line to write if file is new
    /// @return true if rotation succeeded
    bool rotate(const char* dirPath, const char* filename,
                const char* header);

    /// Create directory path and open binary file for writing
    /// @param dirPath   Directory path to create (e.g. "2025/01/15")
    /// @param filename  File name within dirPath (e.g. "data.bin")
    /// @return true if directory creation and file open succeeded
    bool openBinary(const char* dirPath, const char* filename);

    /// Create directory path and open binary file for LZ4 streaming write.
    /// 1ファイル = 1 LZ4フレームで構成し、フレーム境界はファイル
    /// ローテーション (rotateBinaryLZ4) と終了 (destructor) でのみ生成する。
    /// flushIfNeeded() / flush() はフレームを閉じず、cctx 内バッファを
    /// LZ4F_flush で吐き出してから SD に sync する(辞書連結は維持される)。
    /// Build flag SDCSVWRITER_ENABLE_LZ4 must be defined; otherwise this stub
    /// returns false without side effects.
    /// @param dirPath   Directory path to create (e.g. "2025/01/15")
    /// @param filename  File name within dirPath (recommend "...bin.lz4" suffix)
    /// @return true if directory creation, file open, and LZ4 cctx setup succeeded
    bool openBinaryLZ4(const char* dirPath, const char* filename);

    /// Flush current LZ4 frame, close current file, open a new LZ4-mode file
    /// @return true if rotation succeeded
    bool rotateBinaryLZ4(const char* dirPath, const char* filename);

    /// Flush, close current file, and open a new binary file
    /// @param dirPath   Directory path to create (e.g. "2025/01/15")
    /// @param filename  File name within dirPath (e.g. "data.bin")
    /// @return true if rotation succeeded
    bool rotateBinary(const char* dirPath, const char* filename);

    /// Write raw bytes to the currently open file
    /// @param data   Pointer to data
    /// @param len    Number of bytes to write
    /// @param count  If true, increment lineCount (use false for headers)
    /// @return true if write succeeded
    bool writeRaw(const void* data, size_t len, bool count = true);

    /// Create directory path and save raw binary data to file
    /// @param dirPath   Directory path to create (e.g. "2025/01/15")
    /// @param filename  File name within dirPath (e.g. "photo.jpg")
    /// @param data      Raw data to write
    /// @param length    Data length in bytes
    /// @return true if save succeeded
    /// @note SD must be initialized first via begin() or initSD()
    bool saveRawFile(const char* dirPath, const char* filename,
                     const uint8_t* data, uint32_t length);

    /// Write a single CSV line (appends newline automatically)
    /// @return true if write succeeded
    bool writeLine(const char* csvLine);

    /// Flush file to SD card if conditions are met (periodic flush logic)
    /// Call after writeLine() batch
    void flushIfNeeded();

    /// Force flush
    void flush();

    /// Whether SD and file are ready for writing
    bool isReady() const;

    /// Number of lines written since begin()
    unsigned long lineCount() const { return lineCount_; }

    /// Access the underlying SdFat instance (for file listing/reading/deletion)
    SdFat& sd() { return sd_; }

    /// Print SD card free/total space diagnostics to Serial
    void printCardInfo();

    /// Full path of the currently open CSV file (empty string if none)
    const char* currentFilePath() const { return currentPath_; }

    /// Create each directory component in path one by one
    bool mkdirRecursive(const char* path);

private:
    static constexpr unsigned long kEarlyFlushThreshold = 100;   // flush every write for first N lines
    static constexpr unsigned long kPeriodicFlushInterval = 200;  // flush every N lines thereafter
    static constexpr unsigned long kFlushTimeoutMs = 200;         // flush at least every N milliseconds

    SdFat sd_;
    SPIClass spi_;
    File file_;
    bool sdInitialized_ = false;
    bool ready_ = false;
    unsigned long lineCount_ = 0;
    unsigned long lastFlushMs_ = 0;
    char currentPath_[128] = {};

    // ---- LZ4 mode state (active only when SDCSVWRITER_ENABLE_LZ4 build flag is set) ----
    enum class WriteMode : uint8_t { RAW, LZ4 };
    WriteMode mode_ = WriteMode::RAW;

    // Opaque pointer to LZ4F_cctx (lz4frame.h is NOT included from this header
    // to keep other build environments free of the lz4 dependency).
    // Cast to LZ4F_cctx* in the .cpp file.
    void*    lz4Cctx_ = nullptr;
    uint8_t* lz4OutBuf_ = nullptr;     // ps_malloc, output staging
    size_t   lz4OutBufCap_ = 0;
    bool     lz4FrameOpen_ = false;

    // Internal helpers (no-op stubs when SDCSVWRITER_ENABLE_LZ4 is undefined)
    bool lz4FlushBlock_();
    bool lz4WriteCompressed_(const void* data, size_t len);
    void lz4Cleanup_();
};
