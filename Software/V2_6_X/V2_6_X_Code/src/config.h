#pragma once
// Everything you would realistically change: which board this is, which drivers to build, and the
// timings and thresholds of the flight cycle.
// Values fixed by the board, a datasheet or a protocol live in constants.h

#include <Arduino.h>

// Which implementation gets compiled for each part
#define CHIPSAT_DRIVER_LIBRARY 1
#define CHIPSAT_DRIVER_OWN     2

// Each one can be overridden with -DCHIPSAT_x_DRIVER=n
#ifndef CHIPSAT_SYSTEM_DRIVER
#define CHIPSAT_SYSTEM_DRIVER CHIPSAT_DRIVER_LIBRARY  // arduino core + ST HAL
#endif
#ifndef CHIPSAT_I2C_DRIVER
#define CHIPSAT_I2C_DRIVER    CHIPSAT_DRIVER_LIBRARY  // Wire
#endif
#ifndef CHIPSAT_UART_DRIVER
#define CHIPSAT_UART_DRIVER   CHIPSAT_DRIVER_LIBRARY  // HardwareSerial
#endif
#ifndef CHIPSAT_IMU_DRIVER
#define CHIPSAT_IMU_DRIVER    CHIPSAT_DRIVER_LIBRARY  // SparkFun BNO08x 1.0.6
#endif
#ifndef CHIPSAT_GPS_DRIVER
#define CHIPSAT_GPS_DRIVER    CHIPSAT_DRIVER_OWN      // ours, or SparkFun u-blox GNSS 2.2.28
#endif
#ifndef CHIPSAT_ENV_DRIVER
#define CHIPSAT_ENV_DRIVER    CHIPSAT_DRIVER_LIBRARY  // Adafruit BME280 2.3.0
#endif
#ifndef CHIPSAT_GAUGE_DRIVER
#define CHIPSAT_GAUGE_DRIVER  CHIPSAT_DRIVER_OWN      // ours, or Adafruit MAX1704X 1.0.3
#endif
#ifndef CHIPSAT_RADIO_DRIVER
#define CHIPSAT_RADIO_DRIVER  CHIPSAT_DRIVER_LIBRARY  // RadioLib 7.1.2
#endif

// Which Wio-E5 is on the board, from the module label: "Wio-E5-LE" or plain "Wio-E5" (HP)
#define CHIPSAT_RADIO_MODULE_LE 1
#define CHIPSAT_RADIO_MODULE_HP 2
#ifndef CHIPSAT_RADIO_MODULE
#define CHIPSAT_RADIO_MODULE    CHIPSAT_RADIO_MODULE_HP
#endif

// Console log level: 0 off, 1 error, 2 warn, 3 info, 4 debug
#ifndef CHIPSAT_LOG_LEVEL
#define CHIPSAT_LOG_LEVEL 3
#endif

namespace ChipSatConfig
{

constexpr uint8_t kChipSatId = 4;   // change for each board

constexpr uint32_t kConsoleBaud = 115200;

// LED pin levels. Nobody has checked which one actually lights it
constexpr uint8_t kLedLevelAtBoot = LOW;
constexpr uint8_t kLedLevelDuringTx = LOW;
constexpr uint8_t kLedLevelAfterTx = HIGH;

constexpr uint32_t kFirstTxIntervalMs = 5000;   // counted from reset

// How hard every device tries at boot before setup gives up
constexpr uint8_t  kInitAttempts = 5;
constexpr uint16_t kInitRetryDelayMs = 500;

// IMU
constexpr uint16_t kImuReportIntervalMs = 50;   // library overflows above 65 ms
constexpr uint8_t  kImuEnableAttempts = 5;
constexpr uint16_t kImuEnableSpacingMs = 50;
constexpr uint16_t kImuEnableRetryDelayMs = 75;
constexpr uint16_t kImuBootSettleMs = 1000;
constexpr uint16_t kImuWakeSettleMs = 150;
constexpr uint16_t kImuFreshWaitCycleMs = 500;
constexpr uint16_t kImuFreshWaitWakeMs = 3000;
constexpr uint32_t kImuFreshPollMs = 2;

// IMU health checks
constexpr uint32_t kImuStaleMs = 3000;          // IMU data older than this is sent as invalid
constexpr uint8_t  kImuStuckCycles = 3;         // cycles with a silent report before a soft reset
constexpr uint8_t  kImuMaxSoftResets = 3;       // per boot
constexpr uint8_t  kImuMaxServicePasses = 16;   // I2C reads per IMU update
constexpr uint32_t kImuSoftResetSettleMs = 300;

// GPS
constexpr uint32_t kGpsBaud = 9600;
constexpr uint16_t kGpsPvtWaitMs = 1200;
constexpr uint16_t kGpsAckWaitMs = 300;     // a config reply is 10 bytes, about 10 ms on the wire
constexpr uint8_t  kGpsDynamicModel = 8;    // airborne <4g
constexpr uint16_t kGpsMeasurementIntervalMs = 1000;
constexpr uint16_t kGpsNavigationRate = 1;
constexpr uint8_t  kGpsOperateModeFull = 0;

// A fix needs this many satellites before we believe the position. Three fix the position, a fourth
// fixes the clock, so below four the receiver is still converging
constexpr uint8_t  kGpsMinSatellites = 4;

// Throw away the receiver's stored position at boot, so it can never report a stale location from a
// previous run. Keeps the almanac and ephemeris, so a warm start is still fast
constexpr bool     kGpsClearStoredPosition = false;
constexpr uint16_t kGpsResetSettleMs = 500;   // how long the receiver gets to restart afterwards

// BME280
constexpr float kSeaLevelPressureHpa = 1013.25f;   // altitude is measured against this

// Radio
constexpr float    kRadioFrequencyMhz = 915.0f;
constexpr float    kRadioBandwidthKhz = 125.0f;
constexpr uint8_t  kRadioSpreadingFactor = 9;
constexpr uint8_t  kRadioCodingRate = 7;   // 4/7
constexpr uint8_t  kRadioSyncWord = 0x12;
constexpr uint16_t kRadioPreambleLength = 8;
constexpr float    kRadioCurrentLimitMa = 140.0f;   // SetPaConfig resets this, 60 mA LP / 140 mA HP (DS_SX1261-2 Rev 2.2 Table 5-2)

} // namespace ChipSatConfig
