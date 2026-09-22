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

// Wire timeouts in a row before the bus gets started again
constexpr uint8_t kI2cStuckTransfers = 3;

// I2C addresses, set by the parts themselves
constexpr uint8_t kImuAddress = 0x4A;
constexpr uint8_t kEnvAddress = 0x77;
constexpr uint8_t kGaugeAddress = 0x36;

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

// The BME280's operating range (datasheet rev 1.23, key parameters). A reading outside it is a bad
// part or bad calibration, not weather
constexpr float kEnvMinPressurePa = 30000.0f;
constexpr float kEnvMaxPressurePa = 110000.0f;
constexpr float kEnvMinTemperatureC = -40.0f;
constexpr float kEnvMaxTemperatureC = 85.0f;
constexpr float kEnvMinHumidityPercent = 0.0f;
constexpr float kEnvMaxHumidityPercent = 100.0f;

// BNO085 SHTP. Every packet starts with length (2 bytes, bit 15 = continuation), channel, sequence.
// Channel numbers are fixed on the BNO085 (datasheet 1000-3927 v1.17 section 1.3.1)
constexpr uint8_t  kShtpHeaderLength = 4;
constexpr uint16_t kShtpContinuation = 0x8000;
constexpr uint8_t  kShtpChannelCommand = 0;     // the advertisement comes on this one
constexpr uint8_t  kShtpChannelExecutable = 1;
constexpr uint8_t  kShtpChannelControl = 2;
constexpr uint8_t  kShtpChannelReports = 3;
constexpr uint8_t  kShtpChunkLength = 32;       // Wire's own buffer size, so it never reallocs
constexpr uint16_t kShtpPacketMax = 384;        // SparkFun's receive buffer, the advertisement fits

// Executable channel, one byte each way (Figure 1-27)
constexpr uint8_t kImuCommandReset = 1;
constexpr uint8_t kImuCommandOn = 2;
constexpr uint8_t kImuCommandSleep = 3;
constexpr uint8_t kImuResetComplete = 1;

// Own driver, background restart
constexpr uint8_t kImuRestartReads = 8;           // packets read per restart step, while there are any
constexpr uint8_t kImuLostAfterEmptyWaits = 3;    // fresh waits in a row with no report at all, then restarted

// SH-2 report ids and lengths on the control and report channels
constexpr uint8_t kSh2SetFeature = 0xFD;
constexpr uint8_t kSh2SetFeatureLength = 17;
constexpr uint8_t kSh2GetFeatureResponse = 0xFC;
constexpr uint8_t kSh2GetFeatureResponseLength = 17;
constexpr uint8_t kSh2ProductIdRequest = 0xF9;
constexpr uint8_t kSh2ProductIdResponse = 0xF8;
constexpr uint8_t kSh2ProductIdResponseLength = 16;
constexpr uint8_t kSh2CommandResponse = 0xF1;
constexpr uint8_t kSh2CommandResponseLength = 16;
constexpr uint8_t kSh2UnsolicitedInit = 0x84;   // command byte of the SH-2 init message after a reset
constexpr uint8_t kSh2InitSystem = 1;           // its r[1], sh2.c:433-434
constexpr uint8_t kShtpTagReportLengths = 0x81; // in the advertisement, report id then length
constexpr uint8_t kSh2Timebase = 0xFB;
constexpr uint8_t kSh2TimestampRebase = 0xFA;
constexpr uint8_t kSh2TimeReportLength = 5;
constexpr uint8_t kSh2LinearAcceleration = 0x04;
constexpr uint8_t kSh2Gyroscope = 0x02;         // calibrated
constexpr uint8_t kSh2Magnetometer = 0x03;      // calibrated
constexpr uint8_t kSh2RotationVector = 0x05;
constexpr uint8_t kSh2ReportLength = 10;        // id, sequence, status, delay, x y z
constexpr uint8_t kSh2RotationVectorLength = 14;   // plus real and accuracy

// Fixed point: value / 2^Q (sh2_SensorValue.c)
constexpr uint8_t kSh2LinearAccelerationQ = 8;  // m/s^2
constexpr uint8_t kSh2GyroscopeQ = 9;           // rad/s
constexpr uint8_t kSh2MagnetometerQ = 4;        // uT
constexpr uint8_t kSh2RotationVectorQ = 14;

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
constexpr uint32_t kUbxCfgUart1OutprotNmea = 0x10740002;
constexpr uint32_t kUbxCfgMsgoutNavPvtUart1 = 0x20910007;   // NAV-PVT every n solutions, 0 off
constexpr uint8_t kUbxConfigLayers = 0x03;   // RAM + battery backed, so a receiver reset keeps them
constexpr uint8_t kGpsOperateModeFull = 0;   // CFG-PM-OPERATEMODE, full power
constexpr uint32_t kGpsBaud = 9600;   // the receiver's factory setting, we never change it

// NAV-PVT payload is 92 bytes. The fields we read, by offset
constexpr uint16_t kNavPvtLength = 92;
constexpr uint8_t kNavPvtFixType = 20;
constexpr uint8_t kNavPvtFlags = 21;
constexpr uint8_t kNavPvtNumSatellites = 23;
constexpr uint8_t kNavPvtLon = 24;
constexpr uint8_t kNavPvtLat = 28;
constexpr uint8_t kNavPvtHeightMsl = 36;
constexpr uint8_t kNavPvtFlags3 = 78;
constexpr uint16_t kNavPvtInvalidLlh = 0x0001;   // bit 0 of flags3
constexpr uint8_t kNavPvtGnssFixOk = 0x01;   // bit 0 of flags, the fix is within the DOP and accuracy masks
constexpr uint8_t kNavPvtFix3d = 3;
constexpr uint8_t kNavPvtFixGnssDeadReckoning = 4;   // 5 is time only, its position means nothing

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

// Our own radio driver only, the library path doesn't use these. DS = DS_SX1261-2 Rev 2.2
constexpr uint32_t kRadioSwitchPin1 = PA4;   // RF switch, both low when idle, PA5 high to transmit
constexpr uint32_t kRadioSwitchPin2 = PA5;
constexpr uint8_t  kRadioTcxoVoltageCode = 0x01;   // 1.7 V, DS Table 13-35
constexpr uint32_t kRadioTcxoDelaySteps = 320;     // 5 ms in 15.625 us steps, RadioLib's default

constexpr uint16_t kRadioRegVersion = 0x0320;   // "SX1261..." string, not in the DS, RadioLib reads it
constexpr uint16_t kRadioRegIqPolarity = 0x0736;
constexpr uint16_t kRadioRegSyncWord = 0x0740;   // 2 bytes
constexpr uint16_t kRadioRegTxModulation = 0x0889;
constexpr uint16_t kRadioRegTxClamp = 0x08D8;
constexpr uint16_t kRadioRegOcp = 0x08E7;
constexpr uint8_t  kRadioVersionLength = 16;

constexpr uint8_t  kRadioStandbyRc = 0x00;
constexpr uint8_t  kRadioStandbyTries = 5;       // after a reset, begin gives up after this many
constexpr uint32_t kRadioStandbyRetryMs = 10;   // RadioLib waits 10 ms between its tries
constexpr uint8_t  kRadioFallbackStandbyRc = 0x20;
constexpr uint8_t  kRadioPacketTypeLora = 0x01;
constexpr uint8_t  kRadioRegulatorDcDc = 0x01;
constexpr uint8_t  kRadioCalibrateAll = 0x7F;
constexpr uint8_t  kRadioRamp200Us = 0x04;
constexpr uint8_t  kRadioHeaderExplicit = 0x00;
constexpr uint8_t  kRadioCrcOn = 0x01;
constexpr uint8_t  kRadioIqStandard = 0x00;
constexpr uint8_t  kRadioSyncControlBits = 0x44;   // how RadioLib 7.1.2 turns 0x12 into 0x1424, SX126x.cpp:934
constexpr uint16_t kRadioIrqTxDone = 0x0001;
constexpr uint16_t kRadioIrqTimeout = 0x0200;
constexpr uint16_t kRadioIrqAll = 0x43FF;
constexpr uint8_t  kRadioStatusCommandMask = 0x0E;   // bits 3:1, DS Table 13-76
constexpr uint8_t  kRadioStatusCommandTimeout = 0x06;
constexpr uint8_t  kRadioStatusCommandError = 0x08;
constexpr uint8_t  kRadioStatusCommandFailed = 0x0A;

// Power amplifier settings, from DS_SX1261-2 Rev 2.2 Table 13-21. One row per module variant
#if CHIPSAT_RADIO_MODULE == CHIPSAT_RADIO_MODULE_LE
// LE module, SX1261 +14 dBm row, the same values RadioLib uses for its own low power case
constexpr const char *kRadioModuleName = "le";
constexpr int8_t   kRadioPowerDbm = 14;
constexpr bool     kImuSleepsDuringTx = false;
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
constexpr bool     kImuSleepsDuringTx = true;
constexpr int8_t   kPaPowerDbm = 22;
constexpr uint8_t  kPaDutyCycle = 0x02;
constexpr uint8_t  kPaDeviceSel = 0x00;   // high power PA
constexpr uint8_t  kPaHpMax = 0x03;
constexpr uint8_t  kPaLut = 0x01;
#else
#error "CHIPSAT_RADIO_MODULE has to be CHIPSAT_RADIO_MODULE_LE or CHIPSAT_RADIO_MODULE_HP"
#endif

} // namespace ChipSatConstants
