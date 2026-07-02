
#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
#include <SoftwareSerial.h>

// IMU
#define BNO08X_INT -1 // not used
#define BNO08X_RST -1 // not used
#define BNO08X_ADDR 0x4A  // Alternate address if ADR jumper is closed

// UART
#define Print_rxPin PB7
#define Print_txPin PB6
#define GPS_rxPin PC1
#define GPS_txPin PC0

// BME
#define SEALEVELPRESSURE_HPA (1013.25)

// UART Definition
HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);
SoftwareSerial GPS_tx_rx = SoftwareSerial(GPS_rxPin, GPS_txPin);

// Object Creation
BNO08x myIMU;
TinyGPSPlus gps;
Adafruit_BME280 bme;  // I2C


#define blinky PA9

void setup() {

  // UART Start
  Print_tx_rx.begin(115200);
  GPS_tx_rx.begin(9600);
  while (!Print_tx_rx) delay(10);  

  //Pin Definition
  pinMode(blinky, OUTPUT);



  

  // --- START BNO085 ---
  Print_tx_rx.println();

  Wire.begin();
  Wire.flush();
  while (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Print_tx_rx.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    delay(2500); // Try to initialize IMU
  }
  Print_tx_rx.println("BNO08x found!");

  setReports();

  Print_tx_rx.println("BNO initialized");
  // ---  END BNO085 ---


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
    //while (1) delay(10);
  }
  Print_tx_rx.println("BME initialized");
  // --- End BME280 ---

  // --- START GPS ---

}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Print_tx_rx.println("Setting desired reports");
  
  if (myIMU.enableLinearAccelerometer() == true) {
    Print_tx_rx.println(F("Accelerometer enabled"));
    Print_tx_rx.println(F("Output in form x, y, z, in m/s^2"));
  } else {
    Print_tx_rx.println("Could not enable accelerometer");
  }
}


String gpsInfo = "No GPS";
String iMUInfo = "No IMU";
String bMEInfo = "No BME";

String transmitString = "";

int count = 0;
// String gpsInfo = "INVALID: Cannot read";
int procMax = 1000;
int procChar = 0;

void loop() {
  digitalWrite(blinky, LOW);
  while (GPS_tx_rx.available() > 0) {
     gps.encode(GPS_tx_rx.read());
     if(++procChar >= procMax){
        break;
     }
  }
  displayInfo(&gpsInfo);
  Print_tx_rx.println("GPS INFO: " + gpsInfo);

  bMEInfo = getBMEInfo();
  Print_tx_rx.println("BME INFO: " + bMEInfo);

  iMUInfo = getIMUInfo();
  Print_tx_rx.println("IMU INFO: " + iMUInfo);
  


  
  Print_tx_rx.println("Looping...");

  digitalWrite(blinky, HIGH);
    
  delay(1000);
  
}
void displayInfo(String *str) {

  *str = "";
  
  Print_tx_rx.print(F("Location: "));
  
  if (gps.location.isValid()) {
    Print_tx_rx.print(gps.location.lat(), 6);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(gps.location.lng(), 6);
    *str = *str + "Location: " + gps.location.lat() + ", " + gps.location.lng();
  } else {
    Print_tx_rx.print(F("INVALID"));
    *str = *str + "Location: " + "INVALID";
  }

  Print_tx_rx.print(F("  Date: "));
  *str += "  Date: ";

  if (gps.date.isValid()) {
    Print_tx_rx.print(gps.date.month());
    Print_tx_rx.print(F("/"));
    Print_tx_rx.print(gps.date.day());
    Print_tx_rx.print(F("/"));
    Print_tx_rx.print(gps.date.year());

    *str += String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
  } else {
    Print_tx_rx.print(F("INVALID\n"));
    *str += "INVALID";
  }

  Print_tx_rx.print(F(" Time: "));
  *str += " Time: ";
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) {
      Print_tx_rx.print(F("0"));
      *str += "0";
    }
    Print_tx_rx.print(gps.time.hour());
    Print_tx_rx.print(F(":"));
    *str += gps.time.hour() + ":";
    if (gps.time.minute() < 10) {
      Print_tx_rx.print(F("0"));
      *str += "0";
    }
    Print_tx_rx.print(gps.time.minute());
    Print_tx_rx.print(F(":"));
    *str += gps.time.minute() + ":";
    if (gps.time.second() < 10) {
      Print_tx_rx.print(F("0"));
      *str += "0";
    }
    Print_tx_rx.print(gps.time.second());
    Print_tx_rx.print(F("."));
    *str += gps.time.second();
    if (gps.time.centisecond() < 10) {   
      Print_tx_rx.print(F("0"));
      *str += "0";
    }
    Print_tx_rx.print(gps.time.centisecond());
    *str += gps.time.centisecond();
  } else {
    Print_tx_rx.print(F("INVALID"));
    *str += "INVALID";
  }

  Print_tx_rx.print("\n");
  //Print_tx_rx.println(sizeof(str));
}




String getIMUInfo() {
  /*
  String Structure:
  int(IMU_X), int(IMU_Y), int(IMU_Z) 
  */
  String str = "";
  float x = 0 ;
  float y = 0;
  float z = 0;

  if (myIMU.wasReset()) {
    Print_tx_rx.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {
    digitalWrite(blinky, HIGH);

    x = myIMU.getLinAccelX();
    y = myIMU.getLinAccelY();
    z = myIMU.getLinAccelZ();
    byte linAccuracy = myIMU.getLinAccelAccuracy();



    // Print_tx_rx.print(x, 2);
    // Print_tx_rx.print(F(","));
    // Print_tx_rx.print(y, 2);
    // Print_tx_rx.print(F(","));
    // Print_tx_rx.print(z, 2);

    // Print_tx_rx.println();

  }
  str = str + String(int(x)) + ',' + String(int(y)) + ',' + String(int(z));
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


