#include "includes/SystemManager.hpp"
#include "includes/Constants.hpp"

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

void SystemManager::printPacket(DataPacket &data)
{
  debug.println("=== Sensor Data ===");
  debug.println("GPS: %ld, %ld, %ld @ %d\n",
                data.gpsData.latitude, data.gpsData.longitude,
                data.gpsData.altitude, data.gpsData.time);

  debug.println("Gyro: %d, %d, %d\n",
                data.imuData.gyroX, data.imuData.gyroY, data.imuData.gyroZ);

  debug.println("LinAcc: %d, %d, %d\n",
                data.imuData.linX, data.imuData.linY, data.imuData.linZ);

  debug.println("Mag: %d, %d, %d\n",
                data.imuData.magX, data.imuData.magY, data.imuData.magZ);

  debug.println("BME: T=%d P=%d H=%d\n",
                data.bmeData.temp, data.bmeData.pressure, data.bmeData.humidity);

  // 2. Raw Hex Dump (The "Is it actually packed?" test)
  uint8_t *raw = (uint8_t *)&data;
  debug.print("Raw Bytes: ");
  for (size_t i = 0; i < sizeof(DataPacket); i++)
  {
    debug.print(raw[i], HEX);
    debug.print(" ");
  }
  debug.println();
}

DataPacket SystemManager::collectData()
{
  DataPacket data;

  data.gpsData = gps.readData();
  data.imuData = imu.readData();
  data.bmeData = bme.readData();
  printPacket(data);

  return data;
}

void SystemManager::transmitData(const DataPacket &payload)
{
  int state = radio.transmit((uint8_t *)&payload, sizeof(DataPacket));
  radio.interpretState(state); // prints success/error
}

void SystemManager::inloop()
{
  DataPacket payload = collectData();

  debug.println("Transmitting payload:");
  // debug.println(payload); TODO: finish serial debugger print

  transmitData(payload);

  // optional (for testing?)
  digitalWrite(PA9, HIGH);
  delay(1000);
  digitalWrite(PA9, LOW);
  delay(1000);
}