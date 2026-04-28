// include the library
#include <RadioLib.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>



#define Print_rxPin PB7
#define Print_txPin PB6
#include <SoftwareSerial.h>

SoftwareSerial soft_tx_rx =  SoftwareSerial(Print_rxPin, Print_txPin);


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
  soft_tx_rx.begin(9600);

  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, LOW);   // turn the LED off by making the voltage LOW


  // set RF switch control configuration
  // this has to be done prior to calling begin()
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // initialize STM32WL with default settings, except frequency
  soft_tx_rx.print(F("[STM32WL] Initializing ... "));
  int state = radio.begin(915.0);
  

  // EDIT ME:
  //radio.setOutputPower(14); // FOR LP = 14(?)
  radio.setOutputPower(22); // For HP = 20-22


  if (state == RADIOLIB_ERR_NONE) {
    soft_tx_rx.println(F("success!"));
  } else {
    soft_tx_rx.print(F("failed, code "));
    soft_tx_rx.println(state);
    while (true) { delay(10); }
  }

  // set appropriate TCXO voltage for Nucleo WL55JC1
  state = radio.setTCXO(1.7);
  if (state == RADIOLIB_ERR_NONE) {
    soft_tx_rx.println(F("success!"));
  } else {
    soft_tx_rx.print(F("failed, code "));
    soft_tx_rx.println(state);
    while (true) { delay(10); }
  }
}



// counter to keep track of transmitted packets
int count = 0;

void loop() {
  soft_tx_rx.print(F("[STM32WL] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  String str = "Hello World! #" + String(count++);
  int state = radio.transmit(str);

  // you can also transmit byte array up to 256 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    soft_tx_rx.println(F("success!"));

    // print measured data rate
    soft_tx_rx.print(F("[STM32WL] Datarate:\t"));
    soft_tx_rx.print(radio.getDataRate());
    soft_tx_rx.println(F(" bps"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    soft_tx_rx.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    soft_tx_rx.println(F("timeout!"));

  } else {
    // some other error occurred
    soft_tx_rx.print(F("failed, code "));
    soft_tx_rx.println(state);

  }

  // wait for a second before transmitting again
  soft_tx_rx.print("Looping...");
  digitalWrite(PA9, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(PA9, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);
}

