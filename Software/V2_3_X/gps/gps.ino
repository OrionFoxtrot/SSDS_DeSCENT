#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

#define Print_rxPin PB7
#define Print_txPin PB6
#include <SoftwareSerial.h>

#define GPS_rxPin PC1
#define GPS_txPin PC0

//static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(GPS_rxPin, GPS_txPin);
SoftwareSerial Print_tx_rx =  SoftwareSerial(Print_rxPin, Print_txPin);

void setup()
{
  Print_tx_rx.begin(9600);
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
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Print_tx_rx.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  Print_tx_rx.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Print_tx_rx.print(gps.location.lat(), 6);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(gps.location.lng(), 6);
  }
  else
  {
    Print_tx_rx.print(F("INVALID"));
  }

  Print_tx_rx.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Print_tx_rx.print(gps.date.month());
    Print_tx_rx.print(F("/"));
    Print_tx_rx.print(gps.date.day());
    Print_tx_rx.print(F("/"));
    Print_tx_rx.print(gps.date.year());
  }
  else
  {
    Print_tx_rx.print(F("INVALID"));
  }

  Print_tx_rx.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Print_tx_rx.print(F("0"));
    Print_tx_rx.print(gps.time.hour());
    Print_tx_rx.print(F(":"));
    if (gps.time.minute() < 10) Print_tx_rx.print(F("0"));
    Print_tx_rx.print(gps.time.minute());
    Print_tx_rx.print(F(":"));
    if (gps.time.second() < 10) Print_tx_rx.print(F("0"));
    Print_tx_rx.print(gps.time.second());
    Print_tx_rx.print(F("."));
    if (gps.time.centisecond() < 10) Print_tx_rx.print(F("0"));
    Print_tx_rx.print(gps.time.centisecond());
  }
  else
  {
    Print_tx_rx.print(F("INVALID"));
  }

  Print_tx_rx.println();
}
