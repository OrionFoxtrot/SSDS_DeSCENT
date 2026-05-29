#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

#define Print_rxPin PB7
#define Print_txPin PB6


#define GPS_rxPin PC1
#define GPS_txPin PC0
#include <SoftwareSerial.h>

//static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(GPS_rxPin, GPS_txPin);
HardwareSerial Print_tx_rx =  HardwareSerial(Print_rxPin, Print_txPin);

void setup()
{
  Print_tx_rx.begin(115200);
  ss.begin(GPSBaud);

  Print_tx_rx.println(F("DeviceExample.ino"));
  Print_tx_rx.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Print_tx_rx.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Print_tx_rx.println(F("by Mikal Hart"));
  Print_tx_rx.println();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      String s = getInfoString();
      Print_tx_rx.println(s);
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Print_tx_rx.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

 
String getInfoString()
{
  String info = "Location: ";

  // Location
  if (gps.location.isValid())
  {
    info += String(gps.location.lat(), 6);
    info += ",";
    info += String(gps.location.lng(), 6);
  }
  else
  {
    info += "INVALID";
  }

  info += "  Date/Time: ";

  // Date
  if (gps.date.isValid())
  {
    info += String(gps.date.month());
    info += "/";
    info += String(gps.date.day());
    info += "/";
    info += String(gps.date.year());
  }
  else
  {
    info += "INVALID";
  }

  info += " ";

  // Time
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) info += "0";
    info += String(gps.time.hour());

    info += ":";

    if (gps.time.minute() < 10) info += "0";
    info += String(gps.time.minute());

    info += ":";

    if (gps.time.second() < 10) info += "0";
    info += String(gps.time.second());

    info += ".";

    if (gps.time.centisecond() < 10) info += "0";
    info += String(gps.time.centisecond());
  }
  else
  {
    info += "INVALID";
  }

  return info;
}
