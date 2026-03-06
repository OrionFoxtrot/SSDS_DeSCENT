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
  // auto freeRam = []()
  // {
  //   char stack_dummy = 0;
  //   return &stack_dummy - sbrk(0);
  // };
  // debug.print("Free RAM: ");
  // debug.println(freeRam());
  // debug.println();

  payload_.gpsData = gps.readData();
  payload_.bmeData = bme.readData();
  payload_.imuData = imu.readData();

  debug.println("Transmitting payload:");
  transmitData();

  // optional (for testing?)
  digitalWrite(PA9, HIGH);
  delay(1000);
  digitalWrite(PA9, LOW);
  // delay(1000);

  // DUTY CYCLE
  // uint32_t now = millis(); // put logic if this overflows

  // // WRAP THIS IN IF STATEMENT FOR STATE CYCLE (use a flag)
  // // IMU/BME duty cycle toggle
  // if (now - lastIMUToggle >= (imuOn ? imuOnTime : imuOffTime))
  // {
  //   imuOn = !imuOn;
  //   lastIMUToggle = now;

  //   digitalWrite(IMU_EN_PIN, imuOn ? HIGH : LOW);
  //   digitalWrite(BME_EN_PIN, !imuOn ? HIGH : LOW);
  // }

  // // IMU sampling
  // if (imuOn && now - lastIMURead >= imuSampleInterval)
  // {
  //   payload_.imuData = imu.readData();
  //   lastIMURead = now;
  // }

  // // BME sampling
  // if (!imuOn && now - lastBMERead >= bmeSampleInterval)
  // {
  //   payload_.bmeData = bme.readData();
  //   lastBMERead = now;
  // }

  // // GPS 100%
  // payload_.gpsData = gps.readData();

  // // Transmit packet
  // if (now - lastTx >= PACKET_INTERVAL_MS)
  // {
  //   debug.println("Transmitting payload:");

  //   transmitData(payload_);
  //   lastTx = now;

  //   digitalWrite(PA9, HIGH);
  //   delay(1000);
  //   digitalWrite(PA9, LOW);
  //   delay(1000);
  // }
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