// =============================================================================
//  drivers/sd_log — implementation (framework SD over VSPI)
// =============================================================================

#include "drivers/sd_log.h"
#include <FS.h>
#include <SD.h>
#include "config.h"
#include "comms/params.h"

namespace sd_log {

static File s_file;
static bool s_ok = false;
static uint32_t s_since_flush = 0;

// 12-byte header: magic "SROTLOG1" + record size.
static const char MAGIC[8] = { 'S', 'R', 'O', 'T', 'L', 'O', 'G', '1' };

bool begin(SPIClass* spi) {
    if (!SD.begin(params::pinOr(g_params.pin_sdcs, PIN_SD_CS, true), *spi)) { s_ok = false; return false; }

    // Find the next free log file name.
    char path[16];
    for (int i = 0; i < 1000; ++i) {
        snprintf(path, sizeof(path), "/log%03d.bin", i);
        if (!SD.exists(path)) break;
    }
    s_file = SD.open(path, FILE_WRITE);
    if (!s_file) { s_ok = false; return false; }

    uint16_t recsz = sizeof(Record);
    s_file.write((const uint8_t*)MAGIC, sizeof(MAGIC));
    s_file.write((const uint8_t*)&recsz, sizeof(recsz));
    s_file.flush();
    s_ok = true;
    return true;
}

void logRecord(const Record& r) {
    if (!s_ok) return;
    s_file.write((const uint8_t*)&r, sizeof(Record));
    if (++s_since_flush >= 20) { s_file.flush(); s_since_flush = 0; }
}

void flush() {
    if (s_ok) s_file.flush();
}

bool healthy() { return s_ok; }

}  // namespace sd_log
