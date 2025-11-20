// include the library
#include <RadioLib.h>
#include <Wire.h>

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "SparkFun_BNO080_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO080



#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#define Print_rxPin PB7
#define Print_txPin PB6

#define GPS_rxPin PC1
#define GPS_txPin PC0

#define SEALEVELPRESSURE_HPA (1013.25)

TinyGPSPlus gps;
Adafruit_BME280 bme;  // I2C
BNO080 myIMU;


SoftwareSerial Print_tx_rx = SoftwareSerial(Print_rxPin, Print_txPin);
SoftwareSerial GPS_tx_rx = SoftwareSerial(GPS_rxPin, GPS_txPin);


// no need to configure pins, signals are routed to the radio internally
STM32WLx radio = new STM32WLx_Module();

// set RF switch configuration for Nucleo WL55JC1
// NOTE: other boards may be different!
//       Some boards may not have either LP or HP.
//       For those, do not set the LP/HP entry in the table.
static const uint32_t rfswitch_pins[] = { PC_3, PC_4, PC_5, RADIOLIB_NC, RADIOLIB_NC };


static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, HIGH, LOW } },
  { STM32WLx::MODE_TX_LP, { HIGH, HIGH, HIGH } },
  { STM32WLx::MODE_TX_HP, { HIGH, LOW, HIGH } },
  END_OF_MODE_TABLE,
};
/*
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW, LOW}},
  {STM32WLx::MODE_RX,    {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {LOW, HIGH}},
  END_OF_MODE_TABLE,
};
*/


void setup() {

  // Serial:
  Print_tx_rx.begin(9600);
  GPS_tx_rx.begin(9600);

  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, LOW);  // turn the LED off by making the voltage LOW


  // --- START RF LoRa ---
  // this has to be done prior to calling begin()
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // initialize STM32WL with default settings, except frequency
  Print_tx_rx.print(F("[STM32WL] Initializing ... "));
  int state = radio.begin(915.0);
  radio.setOutputPower(14);
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
  // --- End RF LoRa ---

  // --- START IMU  ---
  Wire.begin();
  myIMU.begin(0x4A);
  Wire.setClock(400000);  //Increase I2C data rate to 400kHz

  myIMU.enableLinearAccelerometer(50);  //Send data update every 50ms

  Print_tx_rx.println(F("Linear Accelerometer enabled"));
  Print_tx_rx.println(F("Output in form x, y, z, in m/s^2"));
  // --- End IMU ---

  // --- Start BME280 ---
  unsigned status;
  status = bme.begin();
  if (!status) {
    Print_tx_rx.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Print_tx_rx.print("SensorID was: 0x");
    Print_tx_rx.println(bme.sensorID(), 16);
    Print_tx_rx.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Print_tx_rx.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Print_tx_rx.print("        ID of 0x60 represents a BME 280.\n");
    Print_tx_rx.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  // --- End BME280 ---
}



// counter to keep track of transmitted packets
int count = 0;
String gpsInfo = "No GPS";
String iMUInfo = "No IMU";
String bMEInfo = "No BME";

String transmitString = "";

void loop() {

  //GPS STUFF
  while (GPS_tx_rx.available() > 0) {
    if (gps.encode(GPS_tx_rx.read())) {
      gpsInfo = displayInfo();
    }
  }
  Print_tx_rx.println("GPS INFO: " + gpsInfo);

  if (myIMU.dataAvailable() == true) {
    iMUInfo = getIMUInfo();
  }
  Print_tx_rx.println("IMU INFO: " + iMUInfo);

  bMEInfo = getBMEInfo();
  Print_tx_rx.println("BME INFO: " + bMEInfo);


  transmitString = gpsInfo + ';' + iMUInfo + ';' + bMEInfo;
  Print_tx_rx.println("SIZE OF PACKET: " + String(sizeof(transmitString)));

  Print_tx_rx.println("[STM32WL] Transmitting packet: " + transmitString);

  int state = radio.transmit(transmitString);

  radioStateInterpreter(state);





  // wait for a second before transmitting again
  Print_tx_rx.println("Looping...");

  digitalWrite(PA9, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(PA9, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);
  count += 1;
}

String displayInfo() {

  String str = "";

  //Print_tx_rx.print(F("Location: "));

  if (gps.location.isValid()) {
    //Print_tx_rx.print(gps.location.lat(), 6);
    //Print_tx_rx.print(F(","));
    //Print_tx_rx.print(gps.location.lng(), 6);
    str = str + "Location: " + gps.location.lat() + ", " + gps.location.lng();
  } else {
    //Print_tx_rx.print(F("INVALID"));
    str = str + "Location: " + "INVALID";
  }

  //Print_tx_rx.print(F("  Date: "));
  str += "  Date: ";

  if (gps.date.isValid()) {
    //Print_tx_rx.print(gps.date.month());
    //Print_tx_rx.print(F("/"));
    //Print_tx_rx.print(gps.date.day());
    //Print_tx_rx.print(F("/"));
    //Print_tx_rx.print(gps.date.year());

    str += String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
  } else {
    //Print_tx_rx.print(F("INVALID"));
    str += "INVALID";
  }

  //Print_tx_rx.print(F(" Time: "));
  str += " Time: ";
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) {
      //Print_tx_rx.print(F("0"));
      str += "0";
    }
    //Print_tx_rx.print(gps.time.hour());
    //Print_tx_rx.print(F(":"));
    str += gps.time.hour() + ":";
    if (gps.time.minute() < 10) {
      //Print_tx_rx.print(F("0"));
      str += "0";
    }
    //Print_tx_rx.print(gps.time.minute());
    //Print_tx_rx.print(F(":"));
    str += gps.time.minute() + ":";
    if (gps.time.second() < 10) {
      //Print_tx_rx.print(F("0"));
      str += "0";
    }
    //Print_tx_rx.print(gps.time.second());
    //Print_tx_rx.print(F("."));
    str += gps.time.second();
    if (gps.time.centisecond() < 10) {
      //Print_tx_rx.print(F("0"));
      str += "0";
    }
    //Print_tx_rx.print(gps.time.centisecond());
    str += gps.time.centisecond();
  } else {
    //Print_tx_rx.print(F("INVALID"));
    str += "INVALID";
  }

  //Print_tx_rx.println();
  //Print_tx_rx.println(sizeof(str));
  return (str);
}


String getIMUInfo() {
  /*
  String Structure:
  int(IMU_X), int(IMU_Y), int(IMU_Z) 
  */
  String str = "";

  float x = myIMU.getLinAccelX();
  float y = myIMU.getLinAccelY();
  float z = myIMU.getLinAccelZ();
  byte linAccuracy = myIMU.getLinAccelAccuracy();

  str = str + String(x) + ',' + String(y) + ',' + String(z);
  return (str);
  /*
  Print_tx_rx.print(x, 2);
  Print_tx_rx.print(F(","));
  Print_tx_rx.print(y, 2);
  Print_tx_rx.print(F(","));
  Print_tx_rx.print(z, 2);
  Print_tx_rx.print(F(","));
  Print_tx_rx.print(linAccuracy);

  Print_tx_rx.println();*/
}


String getBMEInfo() {
  /*
  String Structure:
  int(Temperature in C), int(Pressure in hPa), int(altitude in m), int(humidity in %)
  */
  String str = "";

  //Print_tx_rx.print("Temperature = ");
  //Print_tx_rx.print(bme.readTemperature());
  //Print_tx_rx.println(" °C");
  str = str + String(int(bme.readTemperature()));

  //Print_tx_rx.print("Pressure = ");

  //Print_tx_rx.print(bme.readPressure() / 100.0F);
  //Print_tx_rx.println(" hPa");
  str = str + ',' + String(int(bme.readPressure() / 100.0F));

  //Print_tx_rx.print("Approx. Altitude = ");
  //Print_tx_rx.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  //Print_tx_rx.println(" m");
  str = str + ',' + String(int(bme.readAltitude(SEALEVELPRESSURE_HPA)));


  //Print_tx_rx.print("Humidity = ");
  //Print_tx_rx.print(bme.readHumidity());
  //Print_tx_rx.println(" %");
  str = str + ',' + String(int(bme.readHumidity()));

  //Print_tx_rx.println();
  return (str);
}


void radioStateInterpreter(int state) {
  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Print_tx_rx.println(F("success!"));

    // print measured data rate
    Print_tx_rx.print(F("[STM32WL] Datarate:\t"));
    Print_tx_rx.print(radio.getDataRate());
    Print_tx_rx.println(F(" bps"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Print_tx_rx.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Print_tx_rx.println(F("timeout!"));

  } else {
    // some other error occurred
    Print_tx_rx.print(F("failed, code "));
    Print_tx_rx.println(state);
  }
}