#pragma once
// Everything you would realistically change: which board this is, which drivers to build, and the
// timings and thresholds of the flight cycle.
// Values fixed by the board, a datasheet or a protocol live in constants.h

#include <Arduino.h>

// Which implementation gets compiled for each part
#define CHIPSAT_DRIVER_LIBRARY 1
#define CHIPSAT_DRIVER_OWN     2

// Each one can be overridden with -DCHIPSAT_x_DRIVER=n. System, I2C and UART are library only, no own version
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
#define CHIPSAT_IMU_DRIVER    CHIPSAT_DRIVER_OWN      // ours, or SparkFun BNO08x 1.0.6
#endif
#ifndef CHIPSAT_GPS_DRIVER
#define CHIPSAT_GPS_DRIVER    CHIPSAT_DRIVER_OWN      // ours, or SparkFun u-blox GNSS 2.2.28
#endif
#ifndef CHIPSAT_ENV_DRIVER
#define CHIPSAT_ENV_DRIVER    CHIPSAT_DRIVER_OWN      // ours, or Adafruit BME280 2.3.0
#endif
#ifndef CHIPSAT_GAUGE_DRIVER
#define CHIPSAT_GAUGE_DRIVER  CHIPSAT_DRIVER_OWN      // ours, or Adafruit MAX1704X 1.0.3
#endif
#ifndef CHIPSAT_RADIO_DRIVER
#define CHIPSAT_RADIO_DRIVER  CHIPSAT_DRIVER_OWN      // ours, or RadioLib 7.1.2
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

constexpr uint8_t kChipSatId = 64;   // change for each board

constexpr uint32_t kConsoleBaud = 115200;

// LED pin levels. Nobody has checked which one actually lights it
constexpr uint8_t kLedLevelAtBoot = LOW;
constexpr uint8_t kLedLevelDuringTx = LOW;
constexpr uint8_t kLedLevelAfterTx = HIGH;

// How often a packet goes out
// Battery data is read and sent every packet either way
constexpr bool     kTxFromBattery = false;
constexpr uint32_t kTxIntervalMs = 0; // override battery percent table

// Resets the board if the code hangs. Longest stretch with no wait in it is ~1.8 s (GPS setup with no
// receiver), by reading, not measured. 32 s at most on this chip
constexpr uint32_t kWatchdogTimeoutMs = 5000;
static_assert(kWatchdogTimeoutMs >= 1 && kWatchdogTimeoutMs <= 32000, "IWatchdog ignores anything outside this");

// At boot a packet goes out between sensor starts if the last one is older than this
constexpr uint32_t kBootKeepAliveMs = 1000;

// Start attempts for the library drivers. Ours try once and keep retrying in the background
constexpr uint8_t  kInitAttempts = 5;
constexpr uint16_t kInitRetryDelayMs = 500;

// IMU
constexpr uint16_t kImuReportIntervalMs = 50;   // all four reports. The library driver overflows above 65 ms
constexpr uint8_t  kImuEnableAttempts = 5;       // tries per report when turning them on
constexpr uint16_t kImuEnableSpacingMs = 50;     // pause after a report turned on
constexpr uint16_t kImuEnableRetryDelayMs = 75;  // pause after one that didn't
constexpr uint16_t kImuBootSettleMs = 1000;      // after a reset. Ours stops early when the IMU says it's up
constexpr uint32_t kImuRetryMs = 10000;   // how often a lost IMU gets another try, own driver only
constexpr uint8_t  kImuInitAttempts = 1;   // at boot, after that the background retry takes over
constexpr uint16_t kImuWakeSettleMs = 150;       // after wake, before the reports are turned on again
constexpr uint16_t kImuFreshWaitCycleMs = 500;   // longest a cycle waits for new IMU data
constexpr uint16_t kImuFreshWaitWakeMs = 3000;   // the same after a wake
constexpr uint32_t kImuFreshPollMs = 2;          // between checks while waiting

// Own driver only. It waits for the IMU's own answers instead of fixed settles
constexpr uint16_t kImuResetWaitMs = 1500;    // for reset complete. The library waited 300 + 200 + our 1000
constexpr uint16_t kImuReplyWaitMs = 250;     // product id and each enable's reply, the first took 170 ms on the bench
constexpr uint16_t kImuPollMs = 5;            // between reads while waiting for an answer

// IMU health checks
constexpr uint32_t kImuStaleMs = 1000;          // IMU data older than this is sent as invalid
constexpr uint8_t  kImuStuckCycles = 3;         // cycles with a silent report before a soft reset
constexpr uint8_t  kImuMaxSoftResets = 3;       // per boot
constexpr uint8_t  kImuMaxServicePasses = 16;   // I2C reads per IMU update
constexpr uint32_t kImuSoftResetSettleMs = 300;   // after a soft reset, library driver only

// GPS
constexpr uint16_t kGpsPvtWaitMs = 1200;      // library driver only, it polls
constexpr uint16_t kGpsPvtMaxAgeMs = 1500;    // older positions go out invalid
constexpr uint8_t  kGpsInitAttempts = 1;      // read() resends the settings later, so boot doesn't wait
constexpr uint32_t kGpsReconfigMs = 5000;     // settings are resent at most this often while they look lost
constexpr uint16_t kGpsNmeaLostBytes = 20;    // more non-UBX bytes between reads means NMEA is back on, settings lost
constexpr uint16_t kGpsAckWaitMs = 300;     // a config reply is 10 bytes, about 10 ms on the wire
constexpr uint8_t  kGpsDynamicModel = 8;    // airborne <4g
constexpr uint16_t kGpsMeasurementIntervalMs = 1000;   // one position a second
constexpr uint16_t kGpsNavigationRate = 1;              // a solution every measurement

// A fix needs this many satellites before we believe the position. Three fix the position, a fourth
// fixes the clock, so below four the receiver is still converging
constexpr uint8_t  kGpsMinSatellites = 4;

// Throw away the receiver's stored position at boot, so it can never report a stale location from a
// previous run. Keeps the almanac and ephemeris, so a warm start is still fast
constexpr bool     kGpsClearStoredPosition = false;
constexpr uint16_t kGpsResetSettleMs = 500;   // how long the receiver gets to restart afterwards

// BME280
constexpr uint8_t  kEnvOversampling = 0x05;   // 0 off, 1 x1, 2 x2, 3 x4, 4 x8, 5 x16. x16 takes up to 113 ms
constexpr uint16_t kEnvMeasureTimeoutMs = 100;   // on top of that
constexpr uint16_t kEnvMeasurePollMs = 5;
constexpr uint16_t kEnvCalibrationWaitMs = 50;   // the chip copies its calibration out after a reset
constexpr uint16_t kEnvResetSettleMs = 10;
constexpr uint32_t kEnvRetryMs = 10000;   // how often read() tries again to start a BME280 that failed

// Altitude is measured against this. 1013.25 hPa is the standard atmosphere, which is what to use
// unless someone can set the day's actual sea level pressure before launch
constexpr float kSeaLevelPressureHpa = 1013.25f;

// Fuel gauge
constexpr uint32_t kGaugeRetryMs = 10000;   // how often read() tries again to start a MAX17048 that failed

// Radio
constexpr float    kRadioFrequencyMhz = 915.0f;
constexpr float    kRadioBandwidthKhz = 125.0f;
constexpr uint8_t  kRadioSpreadingFactor = 9;
constexpr uint8_t  kRadioCodingRate = 7;   // 4/7
constexpr uint8_t  kRadioSyncWord = 0x12;
constexpr uint16_t kRadioPreambleLength = 8;
constexpr float    kRadioCurrentLimitMa = 140.0f;   // 0 to 140. applyPaConfig overwrites it, see Radio.h

} // namespace ChipSatConfig
