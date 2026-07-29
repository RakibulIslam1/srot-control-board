#pragma once

// =============================================================================
//  drivers/sd_log — binary flight logger on the SD card (VSPI, CS15)
//
//  Packed fixed-width records appended at the logging rate. Shares VSPI with the
//  LoRa radio (CS arbitration). A new numbered file is opened each boot.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>

namespace sd_log {

#pragma pack(push, 1)
struct Record {
    uint32_t t_ms;
    float    qw, qx, qy, qz;   // orientation
    float    gx, gy, gz;       // gyro rad/s
    float    depth;            // m
    int16_t  thr[8];           // DShot values
    uint8_t  mode;
    uint8_t  armed;
    float    pm1, pm2;         // battery V
};
#pragma pack(pop)

bool begin(SPIClass* spi);
void logRecord(const Record& r);
void flush();
bool healthy();

}  // namespace sd_log
