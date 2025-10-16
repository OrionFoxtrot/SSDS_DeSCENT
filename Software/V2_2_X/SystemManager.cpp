#include "SystemManager.hpp"
#include "Constants.hpp"

SystemManager::SystemManager(SoftwareSerial &debugSerial,
                             const uint32_t *rfswitch_pins,
                             const Module::RfSwitchMode_t *rfswitch_table,
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

String SystemManager::collectData()
{
  String data = "";

  // GPS
  String gpsData = gps.readData();
  data += gpsData + ";";

  // IMU
  String imuData = imu.readData();
  data += imuData + ";";

  // BME
  String bmeData = bme.readData();
  data += bmeData;

  return data;
}

void SystemManager::transmitData(String payload)
{
  int state = radio.transmit(payload);
  radio.interpretState(state); // prints success/error
}

void SystemManager::inloop()
{
  String payload = collectData();

  debug.println("Transmitting payload:");
  debug.println(payload);

  transmitData(payload);

  // optional (for testing?)
  digitalWrite(PA9, HIGH);
  delay(1000);
  digitalWrite(PA9, LOW);
  delay(1000);
}