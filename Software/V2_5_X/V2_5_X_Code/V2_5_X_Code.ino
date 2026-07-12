#include "Sensors.h"
#include "Telemetry.h"
#include <RadioLib.h>

ChipSatSensors::Sensors sensors;

ChipSatTelemetry::TelemetryPacket transmitPacket;

uint16_t packetCounter = 0;

#define Print_rxPin PB7
#define Print_txPin PB6
HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);

// Radio Setup
#define blinkypin PA9
STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = { PC_3, PC_4, PC_5, RADIOLIB_NC, RADIOLIB_NC };

static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, LOW } },
  { STM32WLx::MODE_TX_HP, { LOW, HIGH } },  // for LoRa-E5 mini (HP)
  //{STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini (LP)
  END_OF_MODE_TABLE,
};

void setup() {
  Print_tx_rx.begin(115200);

  bool allSensorsFound = sensors.begin(
    &Print_tx_rx,  // Debug output; use nullptr to disable
    9600,          // MAX-M10S UART baud rate
    50             // BNO08x report interval in milliseconds
  );

  if (!allSensorsFound) {
    Print_tx_rx.println("One or more sensors failed to initialize");
  }

  // Start Radio Setup
  pinMode(blinkypin, OUTPUT);
  digitalWrite(blinkypin, LOW);  // turn the LED off by making the voltage LOW
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  Print_tx_rx.print(F("[STM32WL] Initializing ... "));
  int state = radio.begin(915.0);

  //radio.setOutputPower(14); // FOR LP = 14(?)
  // radio.setOutputPower(20);  // For HP = 20-22
  radio.setOutputPower(22);


  if (state == RADIOLIB_ERR_NONE) {
    Print_tx_rx.println(F("success!"));
  } else {
    Print_tx_rx.print(F("failed, code "));
    Print_tx_rx.println(state);
    while (true) { delay(10); }
  }

  // set appropriate TCXO voltage for Nucleo WL55JC1
  state = radio.setTCXO(1.7);
  if (state == RADIOLIB_ERR_NONE) {
    Print_tx_rx.println(F("success!"));
  } else {
    Print_tx_rx.print(F("failed, code "));
    Print_tx_rx.println(state);
    while (true) { delay(10); }
  }
}

int transmissionState = -1;

void loop() {
  // Must run frequently to collect all four BNO08x report types.
  sensors.service();

  static uint32_t previousReadMs = 0;

  if (millis() - previousReadMs >= 5000) {
    const bool freshIMUOkay = sensors.waitForFreshIMUData();
    const bool gpsReadOkay = sensors.readGPS();
    const bool stateOfChargeReadOkay = sensors.readStateOfCharge();
    const bool environmentalReadOkay = sensors.readEnvironmental();

    const bool allDataFresh =
      freshIMUOkay &&
      gpsReadOkay &&
      stateOfChargeReadOkay &&
      environmentalReadOkay;

    const ChipSatSensors::SensorData &data = sensors.data;

    spitOutAllData(data, Print_tx_rx);

    sensors.sleepForTransmit();

    // Sensors off, go quiet for transmit:

    ChipSatTelemetry::encodePacket(
      data,
      packetCounter,
      allDataFresh,
      transmitPacket);

    packetCounter++;

    ChipSatTelemetry::printPacketHex(
      transmitPacket,
      Print_tx_rx);

    ChipSatTelemetry::printSensorValidity(
      transmitPacket.sensorValidity,
      Print_tx_rx);

    digitalWrite(blinkypin, HIGH);
    transmissionState = radio.transmit(
      reinterpret_cast<uint8_t *>(&transmitPacket),
      sizeof(transmitPacket));

    if (transmissionState == RADIOLIB_ERR_NONE) {
      Print_tx_rx.println(F("Telemetry packet transmitted"));
    } else {
      Print_tx_rx.print(F("Transmission failed, code "));
      Print_tx_rx.println(transmissionState);
    }
    digitalWrite(blinkypin, LOW);

    // Transmit Over, Go loud for sensors:
    sensors.wakeAfterTransmit();

    previousReadMs = millis();
    Print_tx_rx.println("=============== End of Cycle ================");
  }
}

void spitOutAllData(
  const ChipSatSensors::SensorData &data,
  Stream &output) {
  output.println();
  output.println(F("========== SENSOR DATA =========="));

  // IMU
  output.println(F("[IMU]"));

  output.print(F("Linear Accel X: "));
  output.print(data.imu.linearAccelerationMps2.x, 3);
  output.println(F(" m/s^2"));

  output.print(F("Linear Accel Y: "));
  output.print(data.imu.linearAccelerationMps2.y, 3);
  output.println(F(" m/s^2"));

  output.print(F("Linear Accel Z: "));
  output.print(data.imu.linearAccelerationMps2.z, 3);
  output.println(F(" m/s^2"));

  output.print(F("Gyro X: "));
  output.print(data.imu.gyroscopeRadPerSec.x, 4);
  output.println(F(" rad/s"));

  output.print(F("Gyro Y: "));
  output.print(data.imu.gyroscopeRadPerSec.y, 4);
  output.println(F(" rad/s"));

  output.print(F("Gyro Z: "));
  output.print(data.imu.gyroscopeRadPerSec.z, 4);
  output.println(F(" rad/s"));

  output.print(F("Mag X: "));
  output.print(data.imu.magnetometerMicroTesla.x, 2);
  output.println(F(" uT"));

  output.print(F("Mag Y: "));
  output.print(data.imu.magnetometerMicroTesla.y, 2);
  output.println(F(" uT"));

  output.print(F("Mag Z: "));
  output.print(data.imu.magnetometerMicroTesla.z, 2);
  output.println(F(" uT"));

  output.print(F("Quaternion I: "));
  output.println(data.imu.orientation.i, 5);

  output.print(F("Quaternion J: "));
  output.println(data.imu.orientation.j, 5);

  output.print(F("Quaternion K: "));
  output.println(data.imu.orientation.k, 5);

  output.print(F("Quaternion Real: "));
  output.println(data.imu.orientation.real, 5);

  // GPS
  output.println(F("[GPS]"));

  output.print(F("Latitude E7: "));
  output.println(data.gps.latitudeE7);

  output.print(F("Longitude E7: "));
  output.println(data.gps.longitudeE7);

  output.print(F("Altitude MSL: "));
  output.print(data.gps.altitudeMSLmm);
  output.println(F(" mm"));

  output.print(F("Latitude: "));
  output.print(data.gps.latitudeE7 / 10000000.0, 7);
  output.println(F(" deg"));

  output.print(F("Longitude: "));
  output.print(data.gps.longitudeE7 / 10000000.0, 7);
  output.println(F(" deg"));

  output.print(F("Altitude MSL: "));
  output.print(data.gps.altitudeMSLmm / 1000.0f, 3);
  output.println(F(" m"));

  // State of charge
  output.println(F("[STATE OF CHARGE]"));

  output.print(F("Cell Percentage: "));
  output.print(data.stateOfCharge.cellPercentage, 2);
  output.println(F(" %"));

  // Environmental sensor
  output.println(F("[ENVIRONMENTAL]"));

  output.print(F("Temperature: "));
  output.print(data.environmental.temperatureC, 2);
  output.println(F(" C"));

  output.print(F("Pressure: "));
  output.print(data.environmental.pressurePa / 100.0f, 2);
  output.println(F(" hPa"));

  output.print(F("Humidity: "));
  output.print(data.environmental.humidityPercent, 2);
  output.println(F(" %"));

  output.print(F("Environmental Altitude: "));
  output.print(data.environmental.altitudeM, 2);
  output.println(F(" m"));

  output.println(F("================================="));
  output.println();
}


void spitOutSomeData(
  const ChipSatSensors::SensorData &data,
  Stream &output) {
  Print_tx_rx.print("Temperature: ");
  Print_tx_rx.println(data.environmental.temperatureC);

  Print_tx_rx.print("State of charge: ");
  Print_tx_rx.println(data.stateOfCharge.cellPercentage);

  Print_tx_rx.print("Latitude E7: ");
  Print_tx_rx.println(data.gps.latitudeE7);

  Print_tx_rx.print("Linear accel X: ");
  Print_tx_rx.println(data.imu.linearAccelerationMps2.x);
}


/*
  if (millis() - previousReadMs >= 1000) {
    previousReadMs = millis();

    // Read everything first. The latest values remain stored in sensors.data
    // while the IMU and GPS are asleep.
    sensors.readSlowSensors();
    sensors.service();

    const ChipSatSensors::SensorData &data = sensors.data;


    sensors.sleepForTransmit();  // GPS indefinete off.

    // Replace this print with your packet creation and radio.transmit(...) call.
    spitOutAllData(data, Print_tx_rx);

    // Resume sensor collection after the transmission finishes.
    sensors.wakeAfterTransmit();
  }
  */