#include "includes/SystemManager.hpp"
#include "includes/Constants.hpp"

extern "C" char *sbrk(int i);

SystemManager::SystemManager(SoftwareSerial &debugSerial,
                             std::array<uint32_t, 5> rfswitch_pins,
                             std::array<Module::RfSwitchMode_t, 5> rfswitch_table,
                             uint8_t gpsRxPin,
                             uint8_t gpsTxPin) : debug(debugSerial),
                                                 radio(rfswitch_pins, rfswitch_table),
                                                 imu(),
                                                 bme(),
                                                 gps(gpsRxPin, gpsTxPin)
{
}

void SystemManager::begin()
{
  debug.begin(9600);
  debug.println("In the process");
  // Initialize sensors
  imu.begin();
  bme.begin();
  lastWakeTime_ = millis();
  gps.begin(9600);

  // Initialize radio
  if (!radio.begin(915.0))
  {
    debug.println("Radio init failed!");
    while (1)
      delay(10); // halt if radio fails --> should this be here?
  }
  radio.setTCXO(1.7);

  debug.println("System initialized successfully.");
}

void SystemManager::setImuBmeSleep(bool sleep)
{
  if (sleep)
  {
    imu.sleep();
    bme.sleep();
    imuBmeAwake_ = false;
    debug.println("IMU/BME → sleep");
  }
  else
  {
    imu.wake();
    bme.wake();
    imuBmeAwake_ = true;
    lastWakeTime_ = millis();
    debug.println("IMU/BME → awake");
  }
}

void SystemManager::applyDutyCycle()
{
  uint32_t now = millis();
  uint32_t half = DUTY_CYCLE_PERIOD_MS / 2;

  if (now - lastToggle_ >= half)
  {
    lastToggle_ = now;
    setImuBmeSleep(imuBmeAwake_);
  }
}

void SystemManager::transmitData()
{
  memset(txBuffer, 0, ChipSatPacket::PACKET_SIZE);
  ChipSatPacket::serialize(payload_, txBuffer);

  int state = radio.transmit(txBuffer, ChipSatPacket::PACKET_SIZE);
  ChipSatPacket::deserializeAndPrint(txBuffer);
  radio.interpretState(state);
}

void SystemManager::loop()
{
  // extern char *sbrk(int);
  // char *heapEnd = (char *)sbrk(0);
  // char stackDummy;
  // debug.print("Free mem: ");
  // debug.println(&stackDummy - heapEnd);

  payload_.gpsData = gps.readData();
  applyDutyCycle();

  if (imuBmeAwake_)
  {
    uint32_t now = millis();
    uint32_t elapsed = now - lastWakeTime_;
    debug.print("Time since wake: ");
    debug.println(elapsed);

    if (now - lastWakeTime_ >= IMU_WARMUP_MS)
    {
      payload_.bmeData = bme.readData();
      payload_.imuData = imu.readData();
    }
  }
  // payload_.bmeData = bme.readData();
  // payload_.imuData = imu.readData();
  // if (state_ == SensorState::FALLING)
  // {
  //   applyDutyCycle();

  //   if (imuBmeAwake_)
  //   {
  //     uint32_t now = millis();

  //     payload_.bmeData = bme.readData();
  //     payload_.imuData = imu.readData();

  //     float gx = payload_.imuData.gyroX / 100.0f; // back to rad/s
  //     float gy = payload_.imuData.gyroY / 100.0f;
  //     float gz = payload_.imuData.gyroZ / 100.0f;
  //     float ax = payload_.imuData.linX / 100.0f; // back to m/s²
  //     float ay = payload_.imuData.linY / 100.0f;
  //     float az = payload_.imuData.linZ / 100.0f;
  //     float alt = payload_.gpsData.altitude;

  //     float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  //     float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);

  //     bool stillFalling;
  //     if (!altInitialized_)
  //     {
  //       stillFalling = true; // assume falling until we have two readings to compare
  //       altInitialized_ = true;
  //     }
  //     else
  //     {
  //       stillFalling = (lastAlt_ - alt) > ALT_DROP_THRESHOLD;
  //     }
  //     lastAlt_ = alt;

  //     bool accelStable = accelMag < GROUND_ACCEL_THRESHOLD;
  //     bool gyroStable = gyroMag < GROUND_GYRO_THRESHOLD;
  //     bool altStable = !stillFalling;

  //     if (accelStable && gyroStable && altStable)
  //     {
  //       if (groundSince_ == 0)
  //         groundSince_ = now;
  //       if (now - groundSince_ >= GROUND_CONFIRM_MS)
  //       {
  //         state_ = SensorState::GROUND_OPS;
  //         setImuBmeSleep(true);
  //         debug.println("STATE → GROUND_OPS");
  //       }
  //     }
  //     else
  //     {
  //       groundSince_ = 0;
  //     }
  //   }
  //}

  debug.println("Transmitting payload:");
  transmitData();

  // optional (for testing?)
  digitalWrite(PA9, HIGH);
  delay(1000);
  digitalWrite(PA9, LOW);
  // delay(1000);

  // BME.setSampling(MODE::SLEEP/MODE::FORCED/MODE::NORMAL)
  // IMU.sleep() or
}

// ======================================================
//              CODE GRAVEYARD
// ======================================================

// ### Old setup (without duty cycles) ###
// DataPacket SystemManager::collectData()
// {
//   DataPacket data;

//   data.gpsData = gps.readData();
//   data.imuData = imu.readData();
//   data.bmeData = bme.readData();
//   printPacket(data);

//   return data;
// }

// void SystemManager::transmitData(const DataPacket &data)
// {
//   uint8_t buffer[PACKET_SIZE];
//   ChipSatPacket::serialize(data, buffer);

//   int state = radio.transmit(buffer, PACKET_SIZE);
//   radio.interpretState(state);
// }

// void SystemManager::inloop()
// {
//   static DataPacket data;

//   data.gpsData = gps.readData();
//   data.imuData = imu.readData();
//   data.bmeData = bme.readData();

//   debug.println("Transmitting payload:");
//   // debug.println(payload); TODO: finish serial debugger print

//   transmitData(data);

//   // optional (for testing?)
//   digitalWrite(PA9, HIGH);
//   delay(1000);
//   digitalWrite(PA9, LOW);
//   delay(1000);
// }

// ### Old helper print function ###
// void SystemManager::printPacket(DataPacket &data)
// {
//   debug.println("=== Sensor Data ===");
//   debug.printf("GPS: %ld, %ld, %ld @ %u\n",
//                data.gpsData.latitude, data.gpsData.longitude,
//                data.gpsData.altitude, data.gpsData.time);

//   debug.printf("Gyro: %d, %d, %d\n",
//                data.imuData.gyroX, data.imuData.gyroY, data.imuData.gyroZ);

//   debug.printf("LinAcc: %d, %d, %d\n",
//                data.imuData.linX, data.imuData.linY, data.imuData.linZ);

//   debug.printf("Mag: %d, %d, %d\n",
//                data.imuData.magX, data.imuData.magY, data.imuData.magZ);

//   debug.printf("BME: T=%d P=%d H=%d\n",
//                data.bmeData.temp, data.bmeData.pressure, data.bmeData.humidity);

//   // 2. Raw Hex Dump (The "Is it actually packed?" test)
//   uint8_t *raw = (uint8_t *)&data;
//   debug.print("Raw Bytes (Size: ");
//   debug.print(sizeof(DataPacket));
//   debug.print("): ");

//   for (size_t i = 0; i < sizeof(DataPacket); i++)
//   {
//     if (raw[i] < 0x10)
//       debug.print("0");
//     debug.print(raw[i], HEX);
//     debug.print(" ");
//   }
//   debug.println("\n==================");
// }

// ### OLD old VERSION ###
// void SystemManager::transmitData(const DataPacket &payload)
// {
//   int state = radio.transmit((uint8_t *)&payload, sizeof(DataPacket));
//   radio.interpretState(state); // prints success/error
// }

// // IMU 50% duty → sample every 2 seconds
// if (now - lastImuRead >= 2000)
// {
//   payload_.imuData = imu.readData();
//   lastImuRead = now;
// }

// // BME 50% duty → sample every 2 seconds
// if (now - lastBmeRead >= 2000)
// {
//   payload_.bmeData = bme.readData();
//   lastBmeRead = now;
// }