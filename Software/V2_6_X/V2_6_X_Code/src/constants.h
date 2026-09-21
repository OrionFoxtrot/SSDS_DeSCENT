#pragma once
// Fixed by the board, a datasheet or a protocol. Named so the code reads, but changing one means
// the hardware or the message format changed, not that you tuned something.
// Settings you would realistically change live in config.h

#include <Arduino.h>
#include "config.h"

namespace ChipSatConstants
{

// Pins, set by the V2.5.x board layout
constexpr uint32_t kConsoleRxPin = PB7;
constexpr uint32_t kConsoleTxPin = PB6;
constexpr uint32_t kLedPin = PA9;
constexpr uint32_t kI2cSdaPin = PA15;
constexpr uint32_t kI2cSclPin = PB15;
constexpr uint32_t kGpsRxPin = PC1;   // (rx, tx) is right, the core swaps the LPUART pins itself
constexpr uint32_t kGpsTxPin = PC0;

// I2C addresses, set by the parts themselves
constexpr uint8_t kImuAddress = 0x4A;
constexpr uint8_t kEnvAddress = 0x77;
constexpr uint8_t kGaugeAddress = 0x36;

// How often we poll while halted. Nothing depends on these, the board is not coming back
constexpr uint32_t kGpsHaltPollMs = 1000;
constexpr uint32_t kRadioHaltPollMs = 10;

// MAX17048 registers. All 16 bit, big endian
constexpr uint8_t kGaugeRegSoc = 0x04;
constexpr uint8_t kGaugeRegVersion = 0x08;
constexpr uint8_t kGaugeRegStatus = 0x1A;
constexpr uint16_t kGaugeVersionMask = 0xFFF0;
constexpr uint16_t kGaugeVersionValue = 0x0010;
constexpr uint8_t kGaugeStatusResetIndicator = 0x01;   // high byte, set once after a real power-up

// BME280 registers
constexpr uint8_t kEnvRegCalib1 = 0x88;     // 26 bytes, temperature and pressure constants plus H1
constexpr uint8_t kEnvRegCalib2 = 0xE1;     // 7 bytes, the humidity constants
constexpr uint8_t kEnvRegChipId = 0xD0;
constexpr uint8_t kEnvRegReset = 0xE0;
constexpr uint8_t kEnvRegStatus = 0xF3;
constexpr uint8_t kEnvRegCtrlHum = 0xF2;
constexpr uint8_t kEnvRegCtrlMeas = 0xF4;
constexpr uint8_t kEnvRegConfig = 0xF5;
constexpr uint8_t kEnvRegData = 0xF7;       // 8 bytes: pressure, temperature, humidity
constexpr uint8_t kEnvCalib1Length = 26;
constexpr uint8_t kEnvCalib2Length = 7;
constexpr uint8_t kEnvDataLength = 8;
constexpr uint8_t kEnvChipIdValue = 0x60;
constexpr uint8_t kEnvResetCommand = 0xB6;
constexpr uint8_t kEnvStatusImUpdate = 0x01;   // bit 0, copying calibration out of its own memory
constexpr uint8_t kEnvStatusMeasuring = 0x08;  // bit 3, a conversion is running
constexpr uint8_t kEnvModeSleep = 0x00;
constexpr uint8_t kEnvModeForced = 0x01;
constexpr uint8_t kEnvFilterOff = 0x00;

// UBX framing: B5 62, class, id, length (little endian), payload, two checksum bytes
constexpr uint8_t kUbxSync1 = 0xB5;
constexpr uint8_t kUbxSync2 = 0x62;
constexpr uint8_t kUbxClassNav = 0x01;
constexpr uint8_t kUbxIdNavPvt = 0x07;
constexpr uint8_t kUbxClassAck = 0x05;
constexpr uint8_t kUbxIdAck = 0x01;
constexpr uint8_t kUbxIdNak = 0x00;
constexpr uint8_t kUbxClassCfg = 0x06;
constexpr uint8_t kUbxIdValset = 0x8A;
constexpr uint8_t kUbxIdReset = 0x04;

// The configuration keys we write, from the M10 interface description
constexpr uint32_t kUbxCfgRateMeas = 0x30210001;
constexpr uint32_t kUbxCfgRateNav = 0x30210002;
constexpr uint32_t kUbxCfgPmOperateMode = 0x20D00001;
constexpr uint32_t kUbxCfgNavspgDynmodel = 0x20110021;
constexpr uint8_t kUbxConfigLayers = 0x03;   // RAM + battery backed, so a receiver reset keeps them

// NAV-PVT payload is 92 bytes. The fields we read, by offset
constexpr uint16_t kNavPvtLength = 92;
constexpr uint8_t kNavPvtFixType = 20;
constexpr uint8_t kNavPvtNumSatellites = 23;
constexpr uint8_t kNavPvtLon = 24;
constexpr uint8_t kNavPvtLat = 28;
constexpr uint8_t kNavPvtHeightMsl = 36;
constexpr uint8_t kNavPvtFlags3 = 78;
constexpr uint16_t kNavPvtInvalidLlh = 0x0001;   // bit 0 of flags3

// Clearing the receiver's stored position: bit 4 of the battery-backed mask. 0xFFFF would be a cold
// start and would cost minutes of acquisition. Reset mode 0x02 restarts navigation, not the chip
constexpr uint16_t kUbxBbrMaskPositionOnly = 0x0010;
constexpr uint8_t kUbxResetModeGnssOnly = 0x02;

// What the position fields can physically hold. Outside these is corruption, not a poor fix
constexpr int32_t kMaxLatitudeE7 = 900000000;
constexpr int32_t kMaxLongitudeE7 = 1800000000;
constexpr int32_t kMinAltitudeMm = -500000;     // -500 m, lower than any land on earth
constexpr int32_t kMaxAltitudeMm = 120000000;   // 120 km, higher than the mission goes

// Radio settings fixed by the module rather than chosen: its TCXO runs at 1.7 V (Seeed
// radio_driver.c TCXO_CTRL_1_7V) and it has an SMPS, so the DC-DC is used rather than the LDO
constexpr float kRadioTcxoVoltage = 1.7f;
constexpr bool kRadioUseLdo = false;

// Power amplifier settings, from DS_SX1261-2 Rev 2.2 Table 13-21. One row per module variant
#if CHIPSAT_RADIO_MODULE == CHIPSAT_RADIO_MODULE_LE
// LE module, SX1261 +14 dBm row, the same values RadioLib uses for its own low power case
constexpr const char *kRadioModuleName = "le";
constexpr int8_t   kRadioPowerDbm = 14;
constexpr int8_t   kPaPowerDbm = 14;
constexpr uint8_t  kPaDutyCycle = 0x04;
constexpr uint8_t  kPaDeviceSel = 0x01;   // low power PA
constexpr uint8_t  kPaHpMax = 0x00;
constexpr uint8_t  kPaLut = 0x01;
#elif CHIPSAT_RADIO_MODULE == CHIPSAT_RADIO_MODULE_HP
// HP module, SX1262 +17 dBm row, SetTxParams +22 (hence kPaPowerDbm). Not RadioLib's 0x04 / 0x07
// row for 22 dBm. Draws a lot more current than LE
constexpr const char *kRadioModuleName = "hp";
constexpr int8_t   kRadioPowerDbm = 20;
constexpr int8_t   kPaPowerDbm = 22;
constexpr uint8_t  kPaDutyCycle = 0x02;
constexpr uint8_t  kPaDeviceSel = 0x00;   // high power PA
constexpr uint8_t  kPaHpMax = 0x03;
constexpr uint8_t  kPaLut = 0x01;
#else
#error "CHIPSAT_RADIO_MODULE has to be CHIPSAT_RADIO_MODULE_LE or CHIPSAT_RADIO_MODULE_HP"
#endif

} // namespace ChipSatConstants
