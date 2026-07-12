/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2652

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <SoftwareSerial.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

#define rxPin PB7
#define txPin PB6

HardwareSerial Print_tx_rx =  HardwareSerial(rxPin, txPin);



void setup() {
    Print_tx_rx.begin(115200);
    while(!Print_tx_rx);    // time to get serial running
    Print_tx_rx.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    //status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    Print_tx_rx.println('hello world');
    status = bme.begin(0x77);
    
    if (!status) {
        Print_tx_rx.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Print_tx_rx.print("SensorID was: 0x"); Print_tx_rx.println(bme.sensorID(),16);
        Print_tx_rx.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Print_tx_rx.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Print_tx_rx.print("        ID of 0x60 represents a BME 280.\n");
        Print_tx_rx.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    Print_tx_rx.println("-- Default Test --");
    delayTime = 1000;

    Print_tx_rx.println();

    pinMode(PA9, OUTPUT);
    Print_tx_rx.println("I have Risen");
    digitalWrite(PA9, HIGH);
}


void loop() { 
    printValues();
    delay(delayTime);
}


void printValues() {
    Print_tx_rx.print("Temperature = ");
    Print_tx_rx.print(bme.readTemperature());
    Print_tx_rx.println(" °C");

    Print_tx_rx.print("Pressure = ");

    Print_tx_rx.print(bme.readPressure() / 100.0F);
    Print_tx_rx.println(" hPa");

    Print_tx_rx.print("Approx. Altitude = ");
    Print_tx_rx.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Print_tx_rx.println(" m");

    Print_tx_rx.print("Humidity = ");
    Print_tx_rx.print(bme.readHumidity());
    Print_tx_rx.println(" %");

    Print_tx_rx.println();
}


/*

16:45:18.148 -> I2C device found at address 0x4A  !
16:45:18.215 -> I2C device found at address 0x76  !
16:45:18.249 -> done
16:45:18.249 -> 
16:45:23.197 -> Scanning...
16:45:23.197 -> I2C device found at address 0x4A  !
16:45:23.231 -> I2C device found at address 0x76  !


*/ 