#pragma once
// Board and flight settings

#include <Arduino.h>

// Which implementation gets compiled for each part. Only the library versions exist right now
#define CHIPSAT_DRIVER_LIBRARY 1
#define CHIPSAT_DRIVER_OWN     2

#define CHIPSAT_SYSTEM_DRIVER CHIPSAT_DRIVER_LIBRARY  // arduino core + ST HAL
#define CHIPSAT_I2C_DRIVER    CHIPSAT_DRIVER_LIBRARY  // Wire
#define CHIPSAT_UART_DRIVER   CHIPSAT_DRIVER_LIBRARY  // HardwareSerial
#define CHIPSAT_IMU_DRIVER    CHIPSAT_DRIVER_LIBRARY  // SparkFun BNO08x 1.0.6
#define CHIPSAT_GPS_DRIVER    CHIPSAT_DRIVER_LIBRARY  // SparkFun u-blox GNSS 2.2.28
#define CHIPSAT_ENV_DRIVER    CHIPSAT_DRIVER_LIBRARY  // Adafruit BME280 2.3.0
#define CHIPSAT_GAUGE_DRIVER  CHIPSAT_DRIVER_LIBRARY  // Adafruit MAX1704X 1.0.3
#define CHIPSAT_RADIO_DRIVER  CHIPSAT_DRIVER_LIBRARY  // RadioLib 7.1.2

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
constexpr uint32_t kConsoleRxPin = PB7;
constexpr uint32_t kConsoleTxPin = PB6;
constexpr uint32_t kLedPin = PA9;
constexpr uint32_t kI2cSdaPin = PA15;
constexpr uint32_t kI2cSclPin = PB15;
constexpr uint32_t kGpsRxPin = PC1;   // (rx, tx) is right, the core swaps the LPUART pins itself
constexpr uint32_t kGpsTxPin = PC0;

// LED pin levels. Nobody has checked which one actually lights it
constexpr uint8_t kLedLevelAtBoot = LOW;
constexpr uint8_t kLedLevelDuringTx = LOW;
constexpr uint8_t kLedLevelAfterTx = HIGH;

constexpr uint32_t kFirstTxIntervalMs = 5000;   // counted from reset
constexpr uint32_t kGpsHaltPollMs = 1000;
constexpr uint32_t kRadioHaltPollMs = 10;

constexpr uint8_t  kInitAttempts = 5;
constexpr uint16_t kInitRetryDelayMs = 500;

// IMU
constexpr uint8_t  kImuAddress = 0x4A;
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
constexpr uint8_t  kGpsDynamicModel = 8;   // airborne <4g
constexpr uint16_t kGpsMeasurementIntervalMs = 1000;
constexpr uint16_t kGpsNavigationRate = 1;
constexpr uint8_t  kGpsOperateModeFull = 0;
constexpr uint8_t  kGpsConfigLayers = 0x03;   // RAM + BBR
constexpr uint32_t kUbxCfgRateMeas = 0x30210001;
constexpr uint32_t kUbxCfgRateNav = 0x30210002;
constexpr uint32_t kUbxCfgPmOperateMode = 0x20D00001;
constexpr uint32_t kUbxCfgNavspgDynmodel = 0x20110021;

// BME280
constexpr uint8_t kEnvAddress = 0x77;
constexpr float   kSeaLevelPressureHpa = 1013.25f;

// Radio
constexpr float    kRadioFrequencyMhz = 915.0f;
constexpr float    kRadioBandwidthKhz = 125.0f;
constexpr uint8_t  kRadioSpreadingFactor = 9;
constexpr uint8_t  kRadioCodingRate = 7;   // 4/7
constexpr uint8_t  kRadioSyncWord = 0x12;
constexpr uint16_t kRadioPreambleLength = 8;
constexpr float    kRadioTcxoVoltage = 1.7f;   // Seeed radio_driver.c TCXO_CTRL_1_7V, same as Meshtastic's wio-e5 variant
constexpr bool     kRadioUseLdo = false;   // use the DC-DC
constexpr float    kRadioCurrentLimitMa = 140.0f;   // SetPaConfig resets this, 60 mA LP / 140 mA HP (DS_SX1261-2 Rev 2.2 Table 5-2)

#if CHIPSAT_RADIO_MODULE == CHIPSAT_RADIO_MODULE_LE
// LE module, low power PA at 14 dBm
// SX1261 +14 dBm row of DS_SX1261-2 Rev 2.2 Table 13-21, same as RadioLib's own LP case
constexpr const char *kRadioModuleName = "le";
constexpr int8_t   kRadioPowerDbm = 14;
constexpr int8_t   kPaPowerDbm = 14;
constexpr uint8_t  kPaDutyCycle = 0x04;
constexpr uint8_t  kPaDeviceSel = 0x01;             // low power PA
constexpr uint8_t  kPaHpMax = 0x00;
constexpr uint8_t  kPaLut = 0x01;
#elif CHIPSAT_RADIO_MODULE == CHIPSAT_RADIO_MODULE_HP
// HP module, high power PA at +17 dBm. Draws a lot more current than LE
// SX1262 +17 dBm row of the same table, SetTxParams +22 (hence kPaPowerDbm)
// not RadioLib's own 0x04 / 0x07 row for 22 dBm
constexpr const char *kRadioModuleName = "hp";
constexpr int8_t   kRadioPowerDbm = 20;
constexpr int8_t   kPaPowerDbm = 22;
constexpr uint8_t  kPaDutyCycle = 0x02;
constexpr uint8_t  kPaDeviceSel = 0x00;             // high power PA
constexpr uint8_t  kPaHpMax = 0x03;
constexpr uint8_t  kPaLut = 0x01;
#else
#error "CHIPSAT_RADIO_MODULE has to be CHIPSAT_RADIO_MODULE_LE or CHIPSAT_RADIO_MODULE_HP"
#endif

} // namespace ChipSatConfig
