// LilyGO T-Beam binary telemetry receiver
//
// Decodes the 55-byte packed ChipSat telemetry packet discussed for the
// Wio-E5 transmitter. All multibyte values are little-endian.

#include <Arduino.h>
#include <RadioLib.h>
#include "boards.h"

#define hexPrint false
#define loudSetup false

// -----------------------------------------------------------------------------
// LoRa settings
// These must match the transmitting ChipSat.
// -----------------------------------------------------------------------------
constexpr float kFrequencyMHz = 915.0;
constexpr float kBandwidthKHz = 125.0;
constexpr uint8_t kSpreadingFactor = 9;
constexpr uint8_t kCodingRate = 7;
constexpr uint8_t kSyncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
constexpr int8_t kOutputPowerDbm = 20;  // Receiver does not use this, but begin() requires it.
constexpr uint16_t kPreambleLength = 8;
constexpr uint8_t kGain = 1;

// -----------------------------------------------------------------------------
// Telemetry packet layout: 55 bytes total
// -----------------------------------------------------------------------------
constexpr size_t kTelemetryPacketLength = 55;
constexpr size_t kCrcProtectedLength = 53;  // Bytes 0 through 52.

namespace PacketOffset {
constexpr size_t kGpsLatitudeE7 = 0;             // int32_t, degrees x 1e7
constexpr size_t kGpsLongitudeE7 = 4;            // int32_t, degrees x 1e7
constexpr size_t kGpsAltitudeMslMm = 8;          // int32_t, millimeters
constexpr size_t kEnvironmentalAltitudeCm = 12;  // int32_t, meters x 100

constexpr size_t kLinearAccelerationX = 16;  // int16_t, m/s^2 x 100
constexpr size_t kLinearAccelerationY = 18;
constexpr size_t kLinearAccelerationZ = 20;

constexpr size_t kGyroscopeX = 22;  // int16_t, deg/s x 10
constexpr size_t kGyroscopeY = 24;
constexpr size_t kGyroscopeZ = 26;

constexpr size_t kMagnetometerX = 28;  // int16_t, uT x 10
constexpr size_t kMagnetometerY = 30;
constexpr size_t kMagnetometerZ = 32;

constexpr size_t kQuaternionI = 34;  // int16_t, normalized value x 32767
constexpr size_t kQuaternionJ = 36;
constexpr size_t kQuaternionK = 38;
constexpr size_t kQuaternionReal = 40;

constexpr size_t kTemperature = 42;     // int16_t, deg C x 100
constexpr size_t kPressure = 44;        // uint16_t, 0.1 hPa per count
constexpr size_t kHumidity = 46;        // uint16_t, percent RH x 100
constexpr size_t kPacketCounter = 48;   // uint16_t
constexpr size_t kCSID = 50;            // uint8_t, ChipSat ID
constexpr size_t kCellPercentage = 51;  // uint8_t, percent x 2
constexpr size_t kSensorValidity = 52;  // uint8_t, validity/freshness bitmask
constexpr size_t kCrc16 = 53;           // uint16_t, CRC-16-CCITT-FALSE
}  // namespace PacketOffset

namespace SensorValidityMask {
constexpr uint8_t kLinearAcceleration = 1U << 0;
constexpr uint8_t kGyroscope = 1U << 1;
constexpr uint8_t kMagnetometer = 1U << 2;
constexpr uint8_t kQuaternion = 1U << 3;
constexpr uint8_t kGps = 1U << 4;
constexpr uint8_t kStateOfCharge = 1U << 5;
constexpr uint8_t kEnvironmental = 1U << 6;
constexpr uint8_t kAllDataFresh = 1U << 7;
}  // namespace SensorValidityMask

struct DecodedTelemetry {
  double latitudeDeg;
  double longitudeDeg;
  float gpsAltitudeM;
  float environmentalAltitudeM;

  float linearAccelerationX;
  float linearAccelerationY;
  float linearAccelerationZ;

  float gyroscopeX;
  float gyroscopeY;
  float gyroscopeZ;

  float magnetometerX;
  float magnetometerY;
  float magnetometerZ;

  float quaternionI;
  float quaternionJ;
  float quaternionK;
  float quaternionReal;

  float temperatureC;
  float pressureHpa;
  float humidityPercent;

  uint16_t packetCounter;
  uint8_t CSID;
  float cellPercentage;

  uint8_t sensorValidity;
  bool linearAccelerationValid;
  bool gyroscopeValid;
  bool magnetometerValid;
  bool quaternionValid;
  bool gpsValid;
  bool stateOfChargeValid;
  bool environmentalValid;
  bool allDataFresh;

  uint16_t receivedCrc;
  uint16_t calculatedCrc;
};

SX1276 radio = new Module(
  RADIO_CS_PIN,
  RADIO_DI0_PIN,
  RADIO_RST_PIN,
  RADIO_DIO1_PIN);

volatile bool receivedFlag = false;
volatile bool receiveInterruptEnabled = true;

// -----------------------------------------------------------------------------
// Interrupt callback
// -----------------------------------------------------------------------------
void setFlag() {
  if (receiveInterruptEnabled) {
    receivedFlag = true;
  }
}

// -----------------------------------------------------------------------------
// Little-endian integer readers
// These do not depend on the ESP32's native endianness or struct packing.
// -----------------------------------------------------------------------------
uint16_t readUint16LE(const uint8_t* data, size_t offset) {
  return static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
}

int16_t readInt16LE(const uint8_t* data, size_t offset) {
  return static_cast<int16_t>(readUint16LE(data, offset));
}

uint32_t readUint32LE(const uint8_t* data, size_t offset) {
  return static_cast<uint32_t>(data[offset]) | (static_cast<uint32_t>(data[offset + 1]) << 8) | (static_cast<uint32_t>(data[offset + 2]) << 16) | (static_cast<uint32_t>(data[offset + 3]) << 24);
}

int32_t readInt32LE(const uint8_t* data, size_t offset) {
  return static_cast<int32_t>(readUint32LE(data, offset));
}

// -----------------------------------------------------------------------------
// CRC-16-CCITT-FALSE
// Polynomial: 0x1021
// Initial value: 0xFFFF
// Final XOR: 0x0000
// Input/output reflection: false
// -----------------------------------------------------------------------------
uint16_t calculateCrc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t byteIndex = 0; byteIndex < length; ++byteIndex) {
    crc ^= static_cast<uint16_t>(data[byteIndex]) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

// -----------------------------------------------------------------------------
// Convert the compact packet values back into engineering units.
// -----------------------------------------------------------------------------
bool decodeTelemetryPacket(
  const uint8_t* packet,
  size_t packetLength,
  DecodedTelemetry& decoded) {
  if (packetLength != kTelemetryPacketLength) {
    return false;
  }

  decoded.latitudeDeg =
    static_cast<double>(readInt32LE(packet, PacketOffset::kGpsLatitudeE7)) / 10000000.0;
  decoded.longitudeDeg =
    static_cast<double>(readInt32LE(packet, PacketOffset::kGpsLongitudeE7)) / 10000000.0;
  decoded.gpsAltitudeM =
    readInt32LE(packet, PacketOffset::kGpsAltitudeMslMm) / 1000.0f;
  decoded.environmentalAltitudeM =
    readInt32LE(packet, PacketOffset::kEnvironmentalAltitudeCm) / 100.0f;

  decoded.linearAccelerationX =
    readInt16LE(packet, PacketOffset::kLinearAccelerationX) / 100.0f;
  decoded.linearAccelerationY =
    readInt16LE(packet, PacketOffset::kLinearAccelerationY) / 100.0f;
  decoded.linearAccelerationZ =
    readInt16LE(packet, PacketOffset::kLinearAccelerationZ) / 100.0f;

  decoded.gyroscopeX =
    readInt16LE(packet, PacketOffset::kGyroscopeX) / 10.0f;
  decoded.gyroscopeY =
    readInt16LE(packet, PacketOffset::kGyroscopeY) / 10.0f;
  decoded.gyroscopeZ =
    readInt16LE(packet, PacketOffset::kGyroscopeZ) / 10.0f;

  decoded.magnetometerX =
    readInt16LE(packet, PacketOffset::kMagnetometerX) / 10.0f;
  decoded.magnetometerY =
    readInt16LE(packet, PacketOffset::kMagnetometerY) / 10.0f;
  decoded.magnetometerZ =
    readInt16LE(packet, PacketOffset::kMagnetometerZ) / 10.0f;

  decoded.quaternionI =
    readInt16LE(packet, PacketOffset::kQuaternionI) / 32767.0f;
  decoded.quaternionJ =
    readInt16LE(packet, PacketOffset::kQuaternionJ) / 32767.0f;
  decoded.quaternionK =
    readInt16LE(packet, PacketOffset::kQuaternionK) / 32767.0f;
  decoded.quaternionReal =
    readInt16LE(packet, PacketOffset::kQuaternionReal) / 32767.0f;

  decoded.temperatureC =
    readInt16LE(packet, PacketOffset::kTemperature) / 100.0f;
  decoded.pressureHpa =
    readUint16LE(packet, PacketOffset::kPressure) / 10.0f;
  decoded.humidityPercent =
    readUint16LE(packet, PacketOffset::kHumidity) / 100.0f;

  decoded.packetCounter =
    readUint16LE(packet, PacketOffset::kPacketCounter);
  decoded.CSID = packet[PacketOffset::kCSID];
  decoded.cellPercentage =
    packet[PacketOffset::kCellPercentage] / 2.0f;

  decoded.sensorValidity = packet[PacketOffset::kSensorValidity];
  decoded.linearAccelerationValid =
    (decoded.sensorValidity & SensorValidityMask::kLinearAcceleration) != 0;
  decoded.gyroscopeValid =
    (decoded.sensorValidity & SensorValidityMask::kGyroscope) != 0;
  decoded.magnetometerValid =
    (decoded.sensorValidity & SensorValidityMask::kMagnetometer) != 0;
  decoded.quaternionValid =
    (decoded.sensorValidity & SensorValidityMask::kQuaternion) != 0;
  decoded.gpsValid =
    (decoded.sensorValidity & SensorValidityMask::kGps) != 0;
  decoded.stateOfChargeValid =
    (decoded.sensorValidity & SensorValidityMask::kStateOfCharge) != 0;
  decoded.environmentalValid =
    (decoded.sensorValidity & SensorValidityMask::kEnvironmental) != 0;
  decoded.allDataFresh =
    (decoded.sensorValidity & SensorValidityMask::kAllDataFresh) != 0;

  decoded.receivedCrc = readUint16LE(packet, PacketOffset::kCrc16);
  decoded.calculatedCrc =
    calculateCrc16CcittFalse(packet, kCrcProtectedLength);

  return true;
}

void printRawPacket(const uint8_t* packet, size_t length) {
  // Line 1: raw packet as uppercase, space-separated hexadecimal bytes.
  if (hexPrint) {
    for (size_t i = 0; i < length; ++i) {
      if (packet[i] < 0x10) {
        Serial.print('0');
      }
      Serial.print(packet[i], HEX);

      if (i + 1 < length) {
        Serial.print(' ');
      }
    }

    Serial.println();
  }
}

// Line 2: decoded packet fields in transmitted order, followed by RSSI and SNR.
//
// latitude,longitude,gpsAltitude,environmentalAltitude,
// linearAccelerationX,linearAccelerationY,linearAccelerationZ,
// gyroscopeX,gyroscopeY,gyroscopeZ,
// magnetometerX,magnetometerY,magnetometerZ,
// quaternionI,quaternionJ,quaternionK,quaternionReal,
// temperature,pressure,humidity,packetCounter,CSID,cellPercentage,
// sensorValidity,linearAccelerationValid,gyroscopeValid,magnetometerValid,
// quaternionValid,gpsValid,stateOfChargeValid,environmentalValid,allDataFresh,
// receivedCRC,RSSI,SNR
void printCsvTelemetry(
  const DecodedTelemetry& data,
  float rssiDbm,
  float snrDb) {
  Serial.print(data.latitudeDeg, 7);
  Serial.print(',');
  Serial.print(data.longitudeDeg, 7);
  Serial.print(',');
  Serial.print(data.gpsAltitudeM, 3);
  Serial.print(',');
  Serial.print(data.environmentalAltitudeM, 2);
  Serial.print(',');

  Serial.print(data.linearAccelerationX, 2);
  Serial.print(',');
  Serial.print(data.linearAccelerationY, 2);
  Serial.print(',');
  Serial.print(data.linearAccelerationZ, 2);
  Serial.print(',');

  Serial.print(data.gyroscopeX, 1);
  Serial.print(',');
  Serial.print(data.gyroscopeY, 1);
  Serial.print(',');
  Serial.print(data.gyroscopeZ, 1);
  Serial.print(',');

  Serial.print(data.magnetometerX, 1);
  Serial.print(',');
  Serial.print(data.magnetometerY, 1);
  Serial.print(',');
  Serial.print(data.magnetometerZ, 1);
  Serial.print(',');

  Serial.print(data.quaternionI, 5);
  Serial.print(',');
  Serial.print(data.quaternionJ, 5);
  Serial.print(',');
  Serial.print(data.quaternionK, 5);
  Serial.print(',');
  Serial.print(data.quaternionReal, 5);
  Serial.print(',');

  Serial.print(data.temperatureC, 2);
  Serial.print(',');
  Serial.print(data.pressureHpa, 1);
  Serial.print(',');
  Serial.print(data.humidityPercent, 2);
  Serial.print(',');

  Serial.print(data.packetCounter);
  Serial.print(',');
  Serial.print(data.CSID);
  Serial.print(',');
  Serial.print(data.cellPercentage, 1);
  Serial.print(',');
  Serial.print(data.sensorValidity);
  Serial.print(',');
  Serial.print(data.linearAccelerationValid ? 1 : 0);
  Serial.print(',');
  Serial.print(data.gyroscopeValid ? 1 : 0);
  Serial.print(',');
  Serial.print(data.magnetometerValid ? 1 : 0);
  Serial.print(',');
  Serial.print(data.quaternionValid ? 1 : 0);
  Serial.print(',');
  Serial.print(data.gpsValid ? 1 : 0);
  Serial.print(',');
  Serial.print(data.stateOfChargeValid ? 1 : 0);
  Serial.print(',');
  Serial.print(data.environmentalValid ? 1 : 0);
  Serial.print(',');
  Serial.print(data.allDataFresh ? 1 : 0);
  Serial.print(',');
  Serial.print(data.receivedCrc);
  Serial.print(',');

  Serial.print(rssiDbm, 1);
  Serial.print(',');
  Serial.println(snrDb, 1);
}

void updateDisplay(
  const DecodedTelemetry& data,
  bool crcValid,
  float rssiDbm,
  float snrDb) {
#ifdef HAS_DISPLAY
  if (!u8g2) {
    return;
  }

  char line[32];

  u8g2->clearBuffer();

  // The 5x8 font allows six readable rows on the T-Beam's 128x64 display.
  u8g2->setFont(u8g2_font_5x8_tf);

  snprintf(
    line,
    sizeof(line),
    "CSID %u P%u %s",
    data.CSID,
    data.packetCounter,
    crcValid ? "OK" : "BAD");
  u8g2->drawStr(0, 8, line);

  snprintf(line, sizeof(line), "Lat %.5f", data.latitudeDeg);
  u8g2->drawStr(0, 18, line);

  snprintf(line, sizeof(line), "Lon %.5f", data.longitudeDeg);
  u8g2->drawStr(0, 28, line);

  snprintf(
    line,
    sizeof(line),
    "T %.1fC  SOC %.1f%%",
    data.temperatureC,
    data.cellPercentage);
  u8g2->drawStr(0, 38, line);

  snprintf(line, sizeof(line), "R %.0f  S %.1f", rssiDbm, snrDb);
  u8g2->drawStr(0, 48, line);

  // L=linear acceleration, G=gyroscope, M=magnetometer, Q=quaternion,
  // P=GPS position, S=state of charge, E=environmental, F=all data fresh.
  snprintf(
    line,
    sizeof(line),
    "L%d G%d M%d Q%d P%d S%d E%d F%d",
    data.linearAccelerationValid ? 1 : 0,
    data.gyroscopeValid ? 1 : 0,
    data.magnetometerValid ? 1 : 0,
    data.quaternionValid ? 1 : 0,
    data.gpsValid ? 1 : 0,
    data.stateOfChargeValid ? 1 : 0,
    data.environmentalValid ? 1 : 0,
    data.allDataFresh ? 1 : 0);
  u8g2->drawStr(0, 61, line);

  u8g2->sendBuffer();
#else
  (void)data;
  (void)crcValid;
  (void)rssiDbm;
  (void)snrDb;
#endif
}

void alertFailure(int state, const __FlashStringHelper* operation) {
  if (state == RADIOLIB_ERR_NONE) {
    if(loudSetup){
      Serial.println(F("success!"));
    }
    return;
  }

  Serial.print(operation);
  Serial.print(F(" failed, RadioLib code "));
  Serial.println(state);

#ifdef HAS_DISPLAY
  if (u8g2) {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_6x12_tf);
    u8g2->drawStr(0, 12, "Radio init failed");
    u8g2->sendBuffer();
  }
#endif

  while (true) {
    delay(1000);
  }
}

void initRadio() {
  if(loudSetup){
    Serial.print(F("Initializing LoRa radio ... "));
  }
  int state = radio.begin(
    kFrequencyMHz,
    kBandwidthKHz,
    kSpreadingFactor,
    kCodingRate,
    kSyncWord,
    kOutputPowerDbm,
    kPreambleLength,
    kGain);
  alertFailure(state, F("radio.begin"));

  // LoRa PHY CRC. This is separate from the CRC-16 stored in bytes 53-54.
  if(loudSetup){
    Serial.print(F("Enabling LoRa PHY CRC ... "));
  }
  state = radio.setCRC(true);
  alertFailure(state, F("radio.setCRC"));
}

void beginListening() {
  const int state = radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Could not restart receive mode. RadioLib code "));
    Serial.println(state);
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial){};
  initBoard();

  // The T-Beam radio power rail needs a brief startup delay.
  delay(1500);

  initRadio();
  radio.setPacketReceivedAction(setFlag);

  if(loudSetup){
    Serial.print(F("Expected telemetry packet length: "));
    Serial.print(kTelemetryPacketLength);
    Serial.println(F(" bytes"));
    

    Serial.print(F("Estimated time on air: "));
    Serial.print(radio.getTimeOnAir(kTelemetryPacketLength));
    Serial.println(F(" us"));
  }
  Serial.println(
    "Latitude_deg,Longitude_deg,GPS_altitude_MSL_m,Environmental_altitude_m,"
    "Linear_acceleration_X_m_s2,Linear_acceleration_Y_m_s2,Linear_acceleration_Z_m_s2,"
    "Gyroscope_X_deg_s,Gyroscope_Y_deg_s,Gyroscope_Z_deg_s,"
    "Magnetometer_X_uT,Magnetometer_Y_uT,Magnetometer_Z_uT,"
    "Quaternion_I,Quaternion_J,Quaternion_K,Quaternion_Real,"
    "Temperature_degC,Pressure_hPa,Humidity_percentRH,"
    "Packet_counter,CSID,Cell_percentage_percent,Sensor_validity_bitmask,"
    "Linear_acceleration_valid,Gyroscope_valid,Magnetometer_valid,Quaternion_valid,"
    "GPS_valid,State_of_charge_valid,Environmental_valid,All_data_fresh,"
    "Received_CRC16_decimal,RSSI_dBm,SNR_dB"
  );

#ifdef HAS_DISPLAY
  if (u8g2) {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_6x12_tf);
    u8g2->drawStr(0, 12, "Waiting for packet");
    u8g2->drawStr(0, 28, "Expected: 55 bytes");
    u8g2->sendBuffer();
  }
#endif

  beginListening();
}

void loop() {
  if (!receivedFlag) {
    return;
  }

  receiveInterruptEnabled = false;
  receivedFlag = false;

  // With explicit LoRa headers, RadioLib reads the actual payload length here.
  const size_t receivedLength = radio.getPacketLength();
  uint8_t packet[255] = { 0 };

  if (receivedLength == 0 || receivedLength > sizeof(packet)) {
    Serial.print(F("Invalid reported packet length: "));
    Serial.println(receivedLength);
    receiveInterruptEnabled = true;
    beginListening();
    return;
  }

  const int state = radio.readData(packet, receivedLength);

  if (state == RADIOLIB_ERR_NONE) {
    const float rssiDbm = radio.getRSSI();
    const float snrDb = radio.getSNR();

    if (receivedLength != kTelemetryPacketLength) {
      Serial.print(F("Packet length error: expected "));
      Serial.print(kTelemetryPacketLength);
      Serial.print(F(" bytes, but received "));
      Serial.print(receivedLength);
      Serial.println(F(" bytes. Packet was not decoded."));
    } else {
      DecodedTelemetry decoded = {};

      if (decodeTelemetryPacket(packet, receivedLength, decoded)) {
        const bool crcValid =
          decoded.receivedCrc == decoded.calculatedCrc;

        // Exactly two serial lines for each successfully received telemetry packet.
        printRawPacket(packet, receivedLength);
        printCsvTelemetry(decoded, rssiDbm, snrDb);

        updateDisplay(decoded, crcValid, rssiDbm, snrDb);
      }
    }
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println(F("LoRa PHY CRC error: corrupted packet discarded."));
  } else {
    Serial.print(F("Radio receive failed. RadioLib code "));
    Serial.println(state);
  }

  receiveInterruptEnabled = true;
  beginListening();
}