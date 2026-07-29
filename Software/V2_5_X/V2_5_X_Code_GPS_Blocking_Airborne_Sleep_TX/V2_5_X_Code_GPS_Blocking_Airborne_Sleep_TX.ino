#include "Sensors.h"
#include "Telemetry.h"
#include <RadioLib.h>

// Things for me to change:
#define waitForGPSInhibit false 
#define HP_WIO 1  // 1: HP WIO. 0: LE WIO

//ALSO DONT FORGET TO CHANGE CSID IN TELEMETRY.h

ChipSatSensors::Sensors sensors;

ChipSatTelemetry::TelemetryPacket transmitPacket;

uint16_t packetCounter = 0;

#define Print_rxPin PB7
#define Print_txPin PB6
HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);

// Radio Setup
#define blinkypin PA9
STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = {
  PA4,
  PA5,
  RADIOLIB_NC,
  RADIOLIB_NC,
  RADIOLIB_NC
};

#if HP_WIO

// Standard Wio-E5, high-power PA
static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, LOW } },
  { STM32WLx::MODE_TX_HP, { LOW, HIGH } },
  END_OF_MODE_TABLE,
};

constexpr int8_t TX_POWER_DBM = 10;

#else

// Wio-E5-LE, low-power PA
static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, LOW } },
  { STM32WLx::MODE_TX_LP, { HIGH, HIGH } },
  END_OF_MODE_TABLE,
};

constexpr int8_t TX_POWER_DBM = 14;
#endif

void setup() {
  Print_tx_rx.begin(115200);
  printResetCause();


  bool allSensorsFound = sensors.begin(
    &Print_tx_rx,  // Debug output; use nullptr to disable
    9600,          // MAX-M10S UART baud rate
    50             // BNO08x report interval in milliseconds
  );

  if (!allSensorsFound) {
    Print_tx_rx.println("One or more sensors failed to initialize");
  }

  // Do not initialize or use the high-power radio until the GPS has acquired
  // its first valid fix. This call intentionally blocks with no timeout.
  if (!sensors.gpsReady()) {
    Print_tx_rx.println(F("GPS initialization failed; radio will remain disabled"));
    while (true) { delay(1000); }
  }

  if (waitForGPSInhibit) {
    sensors.waitForGPSFix();
    Print_tx_rx.println(F("Initial GPS lock complete; continuing radio setup"));
  }

  // Start Radio Setup only after the initial GPS acquisition is complete.
  pinMode(blinkypin, OUTPUT);
  digitalWrite(blinkypin, LOW);  // turn the LED off by making the voltage LOW

  configRadio();

  // delay(9999999);
}

int transmissionState = -1;

void loop() {
  // Must run frequently to collect all four BNO08x report types.
  sensors.service();

  static uint32_t previousReadMs = 0;

  if (millis() - previousReadMs >= 5000) {
    const bool freshIMUOkay = sensors.waitForFreshIMUData();
    const bool gpsReadOkay = sensors.readGPS(1200);
    const bool stateOfChargeReadOkay = sensors.readStateOfCharge();
    const bool environmentalReadOkay = sensors.readEnvironmental();

    const bool allDataFresh =
      freshIMUOkay && gpsReadOkay && stateOfChargeReadOkay && environmentalReadOkay;

    const ChipSatSensors::SensorData &data = sensors.data;

    spitOutAllData(data, Print_tx_rx);

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

    // Put both the MAX-M10S and BNO08x into their low-power states immediately
    // before transmission. Do not transmit if either sleep command fails.
    const bool sensorsAsleepForTransmit = sensors.sleepForTransmit();

    if (sensorsAsleepForTransmit) {
      digitalWrite(blinkypin, LOW);
      delay(1000);
      transmissionState = radio.transmit(
        reinterpret_cast<uint8_t *>(&transmitPacket),
        sizeof(transmitPacket));
      delay(1000);
      digitalWrite(blinkypin, HIGH);

      if (transmissionState == RADIOLIB_ERR_NONE) {
        Print_tx_rx.println(F("Telemetry packet transmitted"));
      } else {
        Print_tx_rx.print(F("Transmission failed, code "));
        Print_tx_rx.println(transmissionState);
      }
    } else {
      Print_tx_rx.println(F("Transmission skipped: GPS or IMU did not enter sleep"));
    }

    // Wake both sensors after the radio current has ended. Airborne mode and
    // normal acquisition/tracking operation are reapplied to the MAX-M10S.
    if (!sensors.wakeAfterTransmit(500)) {
      Print_tx_rx.println(F("Warning: GPS or IMU failed to wake after transmit cycle"));
    }

    previousReadMs = millis();
    Print_tx_rx.println("=============== End of Cycle ================");
  }
}


void configRadio() {
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  Print_tx_rx.print(F("[STM32WL] Initializing ... "));

  int state = radio.begin(
    915.0,  // Frequency in MHz
    125.0,  // Bandwidth in kHz
    9,      // Spreading factor
    7,      // Coding rate 4/7
    0x12,   // Private LoRa sync word
    TX_POWER_DBM,     // Requested output power in dBm Dont change me  here
    8,      // Preamble length
    1.7,    // TCXO voltage setting
    false   // false = use internal DC-DC regulator
  );
  radio.autoLDRO();

  if (state != RADIOLIB_ERR_NONE) {
    Print_tx_rx.print(F("Radio initialization failed, code "));
    Print_tx_rx.println(state);

    while (true) {
      delay(10);
    }
  }

  Print_tx_rx.println(F("success!"));


  // RadioLib defaults the PA overcurrent protection to 60 mA.
  // This is too low for 22 dBm operation on the Wio-E5-HP.
  state = radio.setCurrentLimit(140.0);

  if (state != RADIOLIB_ERR_NONE) {
    Print_tx_rx.print(F("Current limit configuration failed, code "));
    Print_tx_rx.println(state);

    while (true) {
      delay(10);
    }
  }

  Print_tx_rx.print(F("Radio current limit: "));
  Print_tx_rx.print(radio.getCurrentLimit());
  Print_tx_rx.println(F(" mA"));

  state = radio.setOutputPower(TX_POWER_DBM);


  if (state != RADIOLIB_ERR_NONE) {
    Print_tx_rx.print(F("TX power configuration failed, code "));
    Print_tx_rx.println(state);

    while (true) {
      delay(10);
    }
  }

  Print_tx_rx.print(F("TX power configured with val: "));
  Print_tx_rx.println(String(TX_POWER_DBM));
}

int16_t configureOptimal20dBm() {
  int16_t state;

  // Important:
  // This selects the HP path and programs SetTxParams to +22 dBm.
  // ST's optimized +20 dBm configuration requires SetTxParams = +22.
  state = radio.setOutputPower(22);
  if (state != RADIOLIB_ERR_NONE) {
    return state;
  }

  // RadioLib argument order:
  // setPaConfig(paDutyCycle, deviceSel, hpMax, paLut)
  //
  // ST optimized +20 dBm HP configuration:
  // paDutyCycle = 0x03
  // deviceSel   = 0x00
  // hpMax       = 0x05
  // paLut       = 0x01
  state = radio.setPaConfig(
    0x03,
    0x00,
    0x05,
    0x01);

  if (state != RADIOLIB_ERR_NONE) {
    return state;
  }

  // OCP is protection, not a means of setting TX current.
  // Do not clamp it to 100 mA while characterizing the PA.
  state = radio.setCurrentLimit(140.0);
  return state;
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

void printResetCause() {
  Print_tx_rx.println("Reset cause:");

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
    Print_tx_rx.println("  BOR/POR power-related reset");
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    Print_tx_rx.println("  Software reset");
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    Print_tx_rx.println("  Independent watchdog reset");
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
    Print_tx_rx.println("  Window watchdog reset");
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    Print_tx_rx.println("  NRST/system reset flag");
  }

  Print_tx_rx.print("Raw RCC->CSR: 0x");
  Print_tx_rx.println(RCC->CSR, HEX);

  __HAL_RCC_CLEAR_RESET_FLAGS();
}

/*
  if (millis() - previousReadMs >= 1000) {
    previousReadMs = millis();

    // Read everything first. The latest values remain stored in sensors.data
    // while the IMU is asleep; the GPS remains in continuous tracking.
    sensors.readSlowSensors();
    sensors.service();

    const ChipSatSensors::SensorData &data = sensors.data;


    sensors.sleepIMU();  // GPS remains powered and tracking.

    // Replace this print with your packet creation and radio.transmit(...) call.
    spitOutAllData(data, Print_tx_rx);

    // Resume sensor collection after the transmission finishes.
    sensors.wakeIMU();
  }
  */
/* Marks Old Configuration
static const uint32_t rfswitch_pins[] = { PC_3, PC_4, PC_5, RADIOLIB_NC, RADIOLIB_NC };

static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, LOW } },
  { STM32WLx::MODE_TX_HP, { LOW, HIGH } },  // for LoRa-E5 mini (HP)
  //{STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini (LP)
  END_OF_MODE_TABLE,
};
*/