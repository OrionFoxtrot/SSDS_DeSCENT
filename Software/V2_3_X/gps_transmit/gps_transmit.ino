// include the library
#include <RadioLib.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// #define Print_rxPin PB6
// #define Print_txPin PB7
#define Print_rxPin PB7
#define Print_txPin PB6
#include <SoftwareSerial.h>

#define GPS_rxPin PC1
#define GPS_txPin PC0

TinyGPSPlus gps;

HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);
SoftwareSerial GPS_tx_rx = SoftwareSerial(GPS_rxPin, GPS_txPin);


// no need to configure pins, signals are routed to the radio internally
STM32WLx radio = new STM32WLx_Module();

// set RF switch configuration for Nucleo WL55JC1
// NOTE: other boards may be different!
//       Some boards may not have either LP or HP.
//       For those, do not set the LP/HP entry in the table.
// static const uint32_t rfswitch_pins[] = { PC_3, PC_4, PC_5, RADIOLIB_NC, RADIOLIB_NC };

static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE, {LOW, LOW}},
  {STM32WLx::MODE_RX, {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {LOW, HIGH}}, // for LoRa-E5 mini (HP)
  //{STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini (LP)
  END_OF_MODE_TABLE,
};


// static const Module::RfSwitchMode_t rfswitch_table[] = {
//   { STM32WLx::MODE_IDLE, { LOW, LOW, LOW } },
//   { STM32WLx::MODE_RX, { HIGH, HIGH, LOW } },
//   //{ STM32WLx::MODE_TX_LP, { HIGH, HIGH, HIGH } },
//   { STM32WLx::MODE_TX_HP, { HIGH, LOW, HIGH } },
//   END_OF_MODE_TABLE,
// };
/*
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW, LOW}},
  {STM32WLx::MODE_RX,    {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {LOW, HIGH}},
  END_OF_MODE_TABLE,
};
*/


void setup() {
  Print_tx_rx.begin(115200);
  GPS_tx_rx.begin(9600);

  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, LOW);  // turn the LED off by making the voltage LOW


  // set RF switch control configuration
  // this has to be done prior to calling begin()
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // initialize STM32WL with default settings, except frequency
  Print_tx_rx.print(F("[STM32WL] Initializing ... "));
  int state = radio.begin(915.0,125);
  
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
}



// counter to keep track of transmitted packets
int count = 0;
String gpsInfo = "INVALID: Cannot read";
int procMax = 100;
int procChar = 0;
void loop() {
  procChar = 0;
  //GPS STUFF
  while (GPS_tx_rx.available() > 0) {
     gps.encode(GPS_tx_rx.read());
     if(++procChar >= procMax){
        break;
     }
  }
  displayInfo(&gpsInfo);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Print_tx_rx.println(F("No GPS detected: check wiring."));
    while (true)
      ;
  }

  Print_tx_rx.print(F("[STM32WL] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  //String str = "Hello World! #" + String(count++);
  Print_tx_rx.println("chars: " + procChar);
  Print_tx_rx.println("str: " + gpsInfo);
  int state = radio.transmit(gpsInfo);

  // you can also transmit byte array up to 256 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

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
  Print_tx_rx.print("Looping...");
  digitalWrite(PA9, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(PA9, LOW);   // turn the LED off by making the voltage LOW
  delay(20000);
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
