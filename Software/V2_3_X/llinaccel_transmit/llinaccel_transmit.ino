
#include <Wire.h>
#include <RadioLib.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;


#define BNO08X_INT PB3 // not used
#define BNO08X_RST PB4 // not used
#define BNO08X_ADDR 0x4A  // Alternate address if ADR jumper is closed

#define Print_rxPin PB7
#define Print_txPin PB6

int count = 0;

HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);

STM32WLx radio = new STM32WLx_Module();
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

#define blinky PA9
#define GPS_RESET PB5
void setup() {
  Print_tx_rx.begin(115200);
  pinMode(blinky, OUTPUT);
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET,HIGH);
  while (!Print_tx_rx) delay(10);  
  
  Print_tx_rx.println();
  Print_tx_rx.println("BNO08x Read Example");

  Wire.begin();
  Wire.flush();
  while (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Print_tx_rx.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    delay(2500); // Try to initialize IMU
  }
  Print_tx_rx.println("BNO08x found!");

  setReports();

  // set RF switch control configuration
  // this has to be done prior to calling begin()
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // initialize STM32WL with default settings, except frequency
  Print_tx_rx.print(F("[STM32WL] Initializing ... "));
  int state = radio.begin(915.0);
  // radio.begin(915.0)
  // radio.begin()

  

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

  Print_tx_rx.println("Reading events");
  delay(100);
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

void loop() {
  

  if (myIMU.wasReset()) {
    Print_tx_rx.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {
    digitalWrite(blinky, HIGH);

    float x = myIMU.getLinAccelX();
    float y = myIMU.getLinAccelY();
    float z = myIMU.getLinAccelZ();

    Print_tx_rx.print(x, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(y, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(z, 2);

    Print_tx_rx.println();

  }
  delay(10);
  transmit_sm();
  digitalWrite(blinky, LOW);
}

void transmit_sm(){
  Print_tx_rx.print(F("[STM32WL] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  String str = String(count++);
  int state = radio.transmit(str);

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
  delay(30000); // Delay Time
}
