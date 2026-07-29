#include "Debug.h"
#include "Sensors.h"
#include "Telemetry.h"
#include <RadioLib.h>

// Things for me to change:
// Project-wide debug level. Change only this value.
//   0 = No prints
//   1 = Boot/init prints only
//   2 = sensor data only
//   3 = All prints
//   4 = Packet/telemetry prints only
//   5 = Errors, resets, warnings, and failures only
//   6 = Radio/TX status and cadence only
namespace ChipSatDebug {
uint8_t level = 3;
}

#define waitForGPSInhibit false
#define HP_WIO 0        // 1: HP WIO. 0: LE WIO
#define HP_OPTIMIZED 0  // 1: HP WIO. 0: LE WIO

constexpr uint8_t CHIPSAT_ID = 9;  // Change this for each ChipSat

// battery percent based tx duty (s);
#define _75to100 10  //10s tx duty
#define _50to75 30   // 30s tx duty
#define _35to50 60   // 60s tx duty
#define _20to35 120  // 120s tx duty
#define _sub20 180   // 180s tx duty

ChipSatSensors::Sensors sensors;
ChipSatTelemetry::TelemetryPacket transmitPacket;
uint16_t packetCounter = 0;

uint32_t txIntervalFromSoc(float socPercent) {
  if (socPercent > 75.0f) {
    return 10UL * 1000UL;
  } else if (socPercent >= 50.0f) {
    return 30UL * 1000UL;
  } else if (socPercent >= 35.0f) {
    return 60UL * 1000UL;
  } else if (socPercent >= 20.0f) {
    return 120UL * 1000UL;  //120
  } else {
    return 180UL * 1000UL;  //180
  }
}

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

constexpr int8_t TX_POWER_DBM = 20;

#else

// Wio-E5-LE, low-power PA
static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, LOW } },
  { STM32WLx::MODE_TX_LP, { LOW, HIGH } }, //should be high high
  END_OF_MODE_TABLE,
};

constexpr int8_t TX_POWER_DBM = 14;
#endif

void setup() {
  pinMode(blinkypin, OUTPUT);
  digitalWrite(blinkypin, LOW);  // turn the LED off by making the voltage LOW

  Print_tx_rx.begin(115200);

  Print_tx_rx.println("Boot Complete");
  printResetCause();

  bool allSensorsFound = sensors.begin(
    &Print_tx_rx,  // Debug output; use nullptr to disable
    9600,          // MAX-M10S UART baud rate
    50             // BNO08x report interval in milliseconds
  );

  if (!allSensorsFound && ChipSatDebug::bootOrErrors()) {
    Print_tx_rx.println(F("One or more sensors failed to initialize"));
  }

  // Do not initialize or use the high-power radio until the GPS has acquired
  // its first valid fix. This call intentionally blocks with no timeout.
  if (!sensors.gpsReady()) {
    if (ChipSatDebug::bootOrErrors()) {
      Print_tx_rx.println(F("GPS initialization failed; radio will remain disabled"));
    }
    while (true) { delay(1000); }
  }

  if (waitForGPSInhibit) {
    sensors.waitForGPSFix();
    if (ChipSatDebug::boot()) {
      Print_tx_rx.println(F("Initial GPS lock complete; continuing radio setup"));
    }
  }

  configRadio();

  int radioState = 99;
  if (HP_OPTIMIZED == 1) {  // 14, 17, 20, 22
    radioState = configureOptimized17dBm();
  } else if (HP_OPTIMIZED == 0){
    radioState = configureOptimized14dBmLP();
  } 
  if (radioState != RADIOLIB_ERR_NONE) {
    if (ChipSatDebug::bootOrErrors()) {
      Print_tx_rx.print(F("Radio optimized PA configuration failed, code "));
      Print_tx_rx.println(radioState);
    }

    while (true) {
      delay(10);
    }
  }

  Print_tx_rx.print("Setup Complete. Final Radio State: ");
  Print_tx_rx.println(radioState);
}

int transmissionState = -1;

void loop() {
  // Print_tx_rx.println("Helloworld2");
  // Keep collecting BNO08x reports continuously between transmissions.
  sensors.service();

  static uint32_t previousReadMs = 0;
  static uint32_t txIntervalMs = 5000UL;  // First sensor read and TX after 5 seconds.

  // Only enter the full sensor-read and transmit cycle when TX is due.
  if (millis() - previousReadMs >= txIntervalMs) {

    // Read all my sensors:
    const bool freshIMUOkay = sensors.waitForFreshIMUData();
    const bool gpsReadOkay = sensors.readGPS(1200);
    const bool stateOfChargeReadOkay = sensors.readStateOfCharge();
    const bool environmentalReadOkay = sensors.readEnvironmental();

    // Form the Sensor Data Coherently
    const bool allDataFresh =
      freshIMUOkay && gpsReadOkay && stateOfChargeReadOkay && environmentalReadOkay;

    const ChipSatSensors::SensorData &data = sensors.data;

    // printSensorDataCompact(data, Print_tx_rx);
    printSensorDataReadable(data, Print_tx_rx);

    // Encode the Packet

    ChipSatTelemetry::encodePacket(data, packetCounter, CHIPSAT_ID, allDataFresh, transmitPacket);
    packetCounter++;
    ChipSatTelemetry::printPacketHex(transmitPacket, Print_tx_rx);
    ChipSatTelemetry::printSensorValidity(transmitPacket.sensorValidity, Print_tx_rx);

    // Go dark for transmit. GPS does NOT sleep
    const bool sensorsAsleepForTransmit = sensors.sleepForTransmit();


    if (sensorsAsleepForTransmit) {  // run the TX
      digitalWrite(blinkypin, LOW);

      transmissionState = radio.transmit(
        reinterpret_cast<uint8_t *>(&transmitPacket),
        sizeof(transmitPacket));

      digitalWrite(blinkypin, HIGH);

      if (transmissionState == RADIOLIB_ERR_NONE) {
        if (ChipSatDebug::radio()) {
          Print_tx_rx.println(F("Telemetry packet transmitted"));
        }
      } else if (ChipSatDebug::radioOrErrors()) {
        Print_tx_rx.print(F("Transmission failed, code "));
        Print_tx_rx.println(transmissionState);
      }
    } else if (ChipSatDebug::errors()) {
      Print_tx_rx.println(F("Transmission skipped: GPS or IMU did not enter sleep"));
    }

    // Wake both sensors after the radio current has ended. Airborne mode and
    // normal acquisition/tracking operation are reapplied to the MAX-M10S.
    if (!sensors.wakeAfterTransmit(500) && ChipSatDebug::errors()) {
      Print_tx_rx.println(F("Warning: GPS or IMU failed to wake after transmit cycle"));
    }

    // Use the latest valid SoC reading to set the NEXT transmit interval.
    // If the fuel-gauge read failed, retain the previous interval.
    if (stateOfChargeReadOkay) {
      txIntervalMs = txIntervalFromSoc(data.stateOfCharge.cellPercentage);
    }

    previousReadMs = millis();

    if (ChipSatDebug::radio()) {
      Print_tx_rx.print(F("Next TX interval: "));
      Print_tx_rx.print(txIntervalMs / 1000UL);
      Print_tx_rx.println(F(" s"));
      Print_tx_rx.println(F("=============== End of Cycle ================"));
    }
  }
}

// Helpers and setup:

void configRadio() {
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  if (ChipSatDebug::boot()) {
    Print_tx_rx.print(F("[STM32WL] Initializing ... "));
  }

  int state = radio.begin(
    915.0,         // Frequency in MHz
    125.0,         // Bandwidth in kHz
    9,             // Spreading factor
    7,             // Coding rate 4/7
    0x12,          // Private LoRa sync word
    TX_POWER_DBM,  // Requested output power in dBm Dont change me  here
    8,             // Preamble length
    1.7,           // TCXO voltage setting
    false          // false = use internal DC-DC regulator
  );
  radio.autoLDRO();

  if (state != RADIOLIB_ERR_NONE) {
    if (ChipSatDebug::bootOrErrors()) {
      Print_tx_rx.print(F("Radio initialization failed, code "));
      Print_tx_rx.println(state);
    }

    while (true) {
      delay(10);
    }
  }

  if (ChipSatDebug::boot()) {
    Print_tx_rx.println(F("success!"));
  }


  // RadioLib defaults the PA overcurrent protection to 60 mA.
  // This is too low for 22 dBm operation on the Wio-E5-HP.
  state = radio.setCurrentLimit(140.0);

  if (state != RADIOLIB_ERR_NONE) {
    if (ChipSatDebug::bootOrErrors()) {
      Print_tx_rx.print(F("Current limit configuration failed, code "));
      Print_tx_rx.println(state);
    }

    while (true) {
      delay(10);
    }
  }

  if (ChipSatDebug::boot()) {
    Print_tx_rx.print(F("Radio current limit: "));
    Print_tx_rx.print(radio.getCurrentLimit());
    Print_tx_rx.println(F(" mA"));
  }

  state = radio.setOutputPower(TX_POWER_DBM);


  if (state != RADIOLIB_ERR_NONE) {
    if (ChipSatDebug::bootOrErrors()) {
      Print_tx_rx.print(F("TX power configuration failed, code "));
      Print_tx_rx.println(state);
    }

    while (true) {
      delay(10);
    }
  }

  if (ChipSatDebug::boot()) {
    Print_tx_rx.print(F("TX power configured with val: "));
    Print_tx_rx.println(TX_POWER_DBM);
  }
}


// Optimal Configurations

int16_t configureOptimized14dBmLP() {
  int16_t state = radio.setOutputPower(14);
  if (state != RADIOLIB_ERR_NONE) return state;

  return radio.setPaConfig(
    0x04,  // paDutyCycle
    0x01,  // deviceSel: low-power PA
    0x00,  // hpMax: unused for LP
    0x01   // paLut
  );
}
int16_t configureOptimized22dBm() {
  int16_t state = radio.setOutputPower(22);
  if (state != RADIOLIB_ERR_NONE) return state;

  return radio.setPaConfig(
    0x04,  // paDutyCycle
    0x00,  // deviceSel: HP PA
    0x07,  // hpMax
    0x01   // paLut
  );
}

int16_t configureOptimized20dBm() {
  int16_t state = radio.setOutputPower(22);
  if (state != RADIOLIB_ERR_NONE) return state;

  return radio.setPaConfig(
    0x03,
    0x00,
    0x05,
    0x01);
}

int16_t configureOptimized17dBm() {
  int16_t state = radio.setOutputPower(22);
  if (state != RADIOLIB_ERR_NONE) return state;

  return radio.setPaConfig(
    0x02,
    0x00,
    0x03,
    0x01);
}

int16_t configureOptimized14dBm() {
  int16_t state = radio.setOutputPower(14);
  if (state != RADIOLIB_ERR_NONE) return state;

  return radio.setPaConfig(
    0x02,
    0x00,
    0x02,
    0x01);
}


void printSensorDataCompact(
  const ChipSatSensors::SensorData &data,
  Stream &output) {
  if (!ChipSatDebug::sensorData()) {
    return;
  }

  static bool headerPrinted = false;
  if (!headerPrinted) {
    output.println(F("SENSOR,ax_mps2,ay_mps2,az_mps2,gx_radps,gy_radps,gz_radps,mx_uT,my_uT,mz_uT,qi,qj,qk,qr,lat_e7,lon_e7,gps_alt_mm,soc_pct,temp_C,pressure_hPa,humidity_pct,env_alt_m,imu_valid,gps_valid,soc_valid,env_valid"));
    headerPrinted = true;
  }

  output.print(F("SENSOR,"));
  output.print(data.imu.linearAccelerationMps2.x, 3);
  output.print(',');
  output.print(data.imu.linearAccelerationMps2.y, 3);
  output.print(',');
  output.print(data.imu.linearAccelerationMps2.z, 3);
  output.print(',');
  output.print(data.imu.gyroscopeRadPerSec.x, 4);
  output.print(',');
  output.print(data.imu.gyroscopeRadPerSec.y, 4);
  output.print(',');
  output.print(data.imu.gyroscopeRadPerSec.z, 4);
  output.print(',');
  output.print(data.imu.magnetometerMicroTesla.x, 2);
  output.print(',');
  output.print(data.imu.magnetometerMicroTesla.y, 2);
  output.print(',');
  output.print(data.imu.magnetometerMicroTesla.z, 2);
  output.print(',');
  output.print(data.imu.orientation.i, 5);
  output.print(',');
  output.print(data.imu.orientation.j, 5);
  output.print(',');
  output.print(data.imu.orientation.k, 5);
  output.print(',');
  output.print(data.imu.orientation.real, 5);
  output.print(',');
  output.print(data.gps.latitudeE7);
  output.print(',');
  output.print(data.gps.longitudeE7);
  output.print(',');
  output.print(data.gps.altitudeMSLmm);
  output.print(',');
  output.print(data.stateOfCharge.cellPercentage, 2);
  output.print(',');
  output.print(data.environmental.temperatureC, 2);
  output.print(',');
  output.print(data.environmental.pressurePa / 100.0f, 2);
  output.print(',');
  output.print(data.environmental.humidityPercent, 2);
  output.print(',');
  output.print(data.environmental.altitudeM, 2);
  output.print(',');
  output.print(data.imu.valid);
  output.print(',');
  output.print(data.gps.valid);
  output.print(',');
  output.print(data.stateOfCharge.valid);
  output.print(',');
  output.println(data.environmental.valid);
}


void printSensorDataReadable(
  const ChipSatSensors::SensorData &data,
  Stream &output) {
  if (!ChipSatDebug::sensorData()) {
    return;
  }

  output.println();
  output.println(F("==================== SENSOR DATA ===================="));

  output.println(F("IMU"));
  output.print(F("  Overall status:             "));
  output.println(data.imu.valid ? F("VALID") : F("INVALID"));

  output.print(F("  Linear acceleration [m/s^2]: X="));
  output.print(data.imu.linearAccelerationMps2.x, 3);
  output.print(F(", Y="));
  output.print(data.imu.linearAccelerationMps2.y, 3);
  output.print(F(", Z="));
  output.println(data.imu.linearAccelerationMps2.z, 3);
  output.print(F("    Valid: "));
  output.print(data.imu.linearAccelerationValid ? F("YES") : F("NO"));
  output.print(F(" | Updated: "));
  output.print(data.imu.linearAccelerationUpdatedMs);
  output.println(F(" ms"));

  output.print(F("  Gyroscope [rad/s]:           X="));
  output.print(data.imu.gyroscopeRadPerSec.x, 4);
  output.print(F(", Y="));
  output.print(data.imu.gyroscopeRadPerSec.y, 4);
  output.print(F(", Z="));
  output.println(data.imu.gyroscopeRadPerSec.z, 4);
  output.print(F("    Valid: "));
  output.print(data.imu.gyroscopeValid ? F("YES") : F("NO"));
  output.print(F(" | Updated: "));
  output.print(data.imu.gyroscopeUpdatedMs);
  output.println(F(" ms"));

  output.print(F("  Magnetometer [uT]:           X="));
  output.print(data.imu.magnetometerMicroTesla.x, 2);
  output.print(F(", Y="));
  output.print(data.imu.magnetometerMicroTesla.y, 2);
  output.print(F(", Z="));
  output.println(data.imu.magnetometerMicroTesla.z, 2);
  output.print(F("    Valid: "));
  output.print(data.imu.magnetometerValid ? F("YES") : F("NO"));
  output.print(F(" | Updated: "));
  output.print(data.imu.magnetometerUpdatedMs);
  output.println(F(" ms"));

  output.print(F("  Quaternion [i, j, k, real]:  "));
  output.print(data.imu.orientation.i, 5);
  output.print(F(", "));
  output.print(data.imu.orientation.j, 5);
  output.print(F(", "));
  output.print(data.imu.orientation.k, 5);
  output.print(F(", "));
  output.println(data.imu.orientation.real, 5);
  output.print(F("    Valid: "));
  output.print(data.imu.orientationValid ? F("YES") : F("NO"));
  output.print(F(" | Updated: "));
  output.print(data.imu.orientationUpdatedMs);
  output.println(F(" ms"));

  output.println(F("GPS"));
  output.print(F("  Status:                     "));
  output.println(data.gps.valid ? F("VALID 3D FIX") : F("INVALID / NO 3D FIX"));
  output.print(F("  Latitude:                   "));
  output.print(static_cast<double>(data.gps.latitudeE7) / 10000000.0, 7);
  output.print(F(" deg  [raw E7: "));
  output.print(data.gps.latitudeE7);
  output.println(']');
  output.print(F("  Longitude:                  "));
  output.print(static_cast<double>(data.gps.longitudeE7) / 10000000.0, 7);
  output.print(F(" deg  [raw E7: "));
  output.print(data.gps.longitudeE7);
  output.println(']');
  output.print(F("  Altitude MSL:               "));
  output.print(data.gps.altitudeMSLmm / 1000.0f, 3);
  output.print(F(" m  [raw mm: "));
  output.print(data.gps.altitudeMSLmm);
  output.println(']');
  output.print(F("  Updated:                    "));
  output.print(data.gps.updatedMs);
  output.println(F(" ms"));

  output.println(F("POWER"));
  output.print(F("  State of charge:            "));
  output.print(data.stateOfCharge.cellPercentage, 2);
  output.println(F(" %"));
  output.print(F("  Status:                     "));
  output.println(data.stateOfCharge.valid ? F("VALID") : F("INVALID"));
  output.print(F("  Updated:                    "));
  output.print(data.stateOfCharge.updatedMs);
  output.println(F(" ms"));

  output.println(F("ENVIRONMENT"));
  output.print(F("  Temperature:                "));
  output.print(data.environmental.temperatureC, 2);
  output.println(F(" C"));
  output.print(F("  Pressure:                   "));
  output.print(data.environmental.pressurePa / 100.0f, 2);
  output.println(F(" hPa"));
  output.print(F("  Humidity:                   "));
  output.print(data.environmental.humidityPercent, 2);
  output.println(F(" %RH"));
  output.print(F("  Pressure altitude:          "));
  output.print(data.environmental.altitudeM, 2);
  output.println(F(" m"));
  output.print(F("  Status:                     "));
  output.println(data.environmental.valid ? F("VALID") : F("INVALID"));
  output.print(F("  Updated:                    "));
  output.print(data.environmental.updatedMs);
  output.println(F(" ms"));

  output.println(F("====================================================="));
  output.println();
}


void printResetCause() {
  if (!ChipSatDebug::errors()) {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return;
  }

  Print_tx_rx.println(F("Reset cause:"));

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
    Print_tx_rx.println(F("  BOR/POR power-related reset"));
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    Print_tx_rx.println(F("  Software reset"));
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    Print_tx_rx.println(F("  Independent watchdog reset"));
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
    Print_tx_rx.println(F("  Window watchdog reset"));
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    Print_tx_rx.println(F("  NRST/system reset flag"));
  }

  Print_tx_rx.print(F("Raw RCC->CSR: 0x"));
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

    // Print compact data only when sensor-data debug is selected.
    printSensorDataCompact(data, Print_tx_rx);

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

/*
Boring Forward Declerations(Cpp req):

void configRadio();
int16_t configureOptimized22dBm();
int16_t configureOptimized20dBm();
int16_t configureOptimized17dBm();
int16_t configureOptimized14dBm();
void printSensorDataCompact(
  const ChipSatSensors::SensorData &data,
  Stream &output);
void printResetCause();

*/