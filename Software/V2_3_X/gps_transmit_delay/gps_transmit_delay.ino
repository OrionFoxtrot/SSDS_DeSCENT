// include the library
#include <RadioLib.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPSPlus.h>

// this should work now for new v23x boards

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
SoftwareSerial GPS_tx_rx = SoftwareSerial(GPS_rxPin, GPS_txPin);
SoftwareSerial Print_tx_rx =  SoftwareSerial(Print_rxPin, Print_txPin);

// no need to configure pins, signals are routed to the radio internally
STM32WLx radio = new STM32WLx_Module();

// set RF switch configuration for Nucleo WL55JC1
// NOTE: other boards may be different!
//       Some boards may not have either LP or HP.
//       For those, do not set the LP/HP entry in the table.
static const uint32_t rfswitch_pins[] =
                         {PC_3,  PC_4,  PC_5, RADIOLIB_NC, RADIOLIB_NC};



// EDIT ME

static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE, {LOW, LOW}},
  {STM32WLx::MODE_RX, {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {LOW, HIGH}}, // for LoRa-E5 mini (HP)
  //{STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini (LP)
  END_OF_MODE_TABLE,
};


// Dont think this works:

// FOR HIGH POWERED START
// static const Module::RfSwitchMode_t rfswitch_table[] = {
//   {STM32WLx::MODE_IDLE, {LOW, LOW}},
//   {STM32WLx::MODE_RX, {HIGH, LOW}},
//   {STM32WLx::MODE_TX_HP, {LOW, HIGH}}, // for LoRa-E5 mini
//   END_OF_MODE_TABLE,
// };
// FOR HIGH POWERED END

/*
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW, LOW}},
  {STM32WLx::MODE_RX,    {HIGH, LOW}},
  {STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini
  END_OF_MODE_TABLE,
};

*/


void setup() {
  Print_tx_rx.begin(9600);

  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, LOW);   // turn the LED off by making the voltage LOW


  // set RF switch control configuration
  // this has to be done prior to calling begin()
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // initialize STM32WL with default settings, except frequency
  Print_tx_rx.print(F("[STM32WL] Initializing ... "));
  int state = radio.begin(915.0);

  // EDIT ME:
  //radio.setOutputPower(14); // FOR LP = 14(?)
  radio.setOutputPower(22); // For HP = 20-22


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

  Print_tx_rx.begin(9600);
  GPS_tx_rx.begin(GPSBaud);
}



// counter to keep track of transmitted packets
int count = 0;
String s = "";

void loop() {
  s = "INVALID (NO T/R X From UART)";
  while (GPS_tx_rx.available() > 0){
    if (gps.encode(GPS_tx_rx.read())){
      s = getInfoString();
      Print_tx_rx.println(s); 
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Print_tx_rx.println(F("No GPS detected: check wiring."));
    while(true);
  }
  Print_tx_rx.print(F("[STM32WL] Transmitting packet ... "));
  int state = radio.transmit(s);
  //Print_tx_rx.println(s);


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

  // wait for a second before transmitting again
  Print_tx_rx.println("Looping...");
  digitalWrite(PA9, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(PA9, LOW);   // turn the LED off by making the voltage LOW
  delay(5000);
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

