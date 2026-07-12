// include the library
#include <RadioLib.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_MAX1704X.h"
#include <Adafruit_Sensor.h>

#include <Adafruit_BME280.h>
Adafruit_MAX17048 maxlipo;
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;




#define Print_rxPin PB7
#define Print_txPin PB6
HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);

#define SCL_Line PB15
#define SDA_Line PA15
TwoWire I2C_Line(SDA_Line, SCL_Line);


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C

// no need to configure pins, signals are routed to the radio internally
STM32WLx radio = new STM32WLx_Module();

// set RF switch configuration for Nucleo WL55JC1
// NOTE: other boards may be different!
//       Some boards may not have either LP or HP.
//       For those, do not set the LP/HP entry in the table.
static const uint32_t rfswitch_pins[] = { PC_3, PC_4, PC_5, RADIOLIB_NC, RADIOLIB_NC };



// EDIT ME

static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, LOW } },
  { STM32WLx::MODE_TX_HP, { LOW, HIGH } },  // for LoRa-E5 mini (HP)
  //{STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini (LP)
  END_OF_MODE_TABLE,
};

// GPS UART pins
// MCU RX pin connects to GPS TX
// MCU TX pin connects to GPS RX
#define GPS_rxPin PC1
#define GPS_txPin PC0
HardwareSerial GPSSerial(GPS_rxPin, GPS_txPin);



void setup() {
  delay(2000);
  Print_tx_rx.begin(115200);
  Print_tx_rx.println("Boot");
  GPSSerial.begin(9600);
  delay(2000);

  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, LOW);  // turn the LED off by making the voltage LOW


  // set RF switch control configuration
  // this has to be done prior to calling begin()
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // initialize STM32WL with default settings, except frequency
  Print_tx_rx.print(F("[STM32WL] Initializing ... "));
  int state = radio.begin(915.0);


  // EDIT ME:
  //radio.setOutputPower(14); // FOR LP = 14(?)
  // radio.setOutputPower(20);  // For HP = 20-22
  radio.setOutputPower(22);


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
  Print_tx_rx.println("--- End of Transmit Stuff ---");

  I2C_Line.begin();
  I2C_Line.setTimeout(50);
  I2C_Line.setClock(100000);
  while (!maxlipo.begin(&I2C_Line)) {
    Print_tx_rx.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }
  Print_tx_rx.print(F("Found MAX17048"));
  Print_tx_rx.print(F(" with Chip ID: 0x"));
  Print_tx_rx.println(maxlipo.getChipID(), HEX);

  //BME
  unsigned status;
  status = bme.begin(0x77, &I2C_Line);
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

  //GPS START
  if (myGNSS.begin(GPSSerial) == false) {
    Print_tx_rx.println("GPS not detected. Check baud rate / wiring.");
    while (1) {
      delay(1000);
    }
  }
  bool ok = myGNSS.powerOffWithInterrupt(
    0,                             // forever
    VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX  // allow UART RX to wake it
  );

  if (ok)
    Print_tx_rx.println("Standby command ACKed.");
  else
    Print_tx_rx.println("Standby command failed / no ACK.");

  Print_tx_rx.println("Do NOT talk to GPS now. Measure current drop.");

  //GPS END
}



// counter to keep track of transmitted packets
unsigned long count = 0;
char msg[80];
float cellVoltage = 1.11;
float cellPercent = 1.1;
int temp = 1.1;
void loop() {

  Print_tx_rx.println("-----Reading Cell Start-----");
  cellVoltage = maxlipo.cellVoltage();
  if (isnan(cellVoltage)) {
    Print_tx_rx.println("Failed to read cell voltage, check battery is connected!");
    delay(2000);
    return;
  }
  Print_tx_rx.print(F("Batt Voltage: "));
  Print_tx_rx.print(cellVoltage, 3);
  Print_tx_rx.println(" V");
  Print_tx_rx.print(F("Batt Percent: "));
  Print_tx_rx.print(maxlipo.cellPercent(), 1);
  Print_tx_rx.println(" %");
  cellPercent = maxlipo.cellPercent();
  Print_tx_rx.println("-----Reading Cell End-----");
  temp = int(bme.readTemperature());


  uint16_t voltage_centi = cellVoltage * 100 + 0.5;  // 3.72 V -> 372
  uint16_t percent_tenth = cellPercent * 10 + 0.5;   // 84.6 % -> 846

  snprintf(msg, sizeof(msg),
           "Counter: %lu, Voltage: %u.%02u V, Battery: %u.%u %%, Temp: %d",
           count++,
           (unsigned)voltage_centi / 100,
           (unsigned)voltage_centi % 100,
           (unsigned)percent_tenth / 10,
           (unsigned)percent_tenth % 10,
           temp);

  Print_tx_rx.println("-----Message Form Start-----");
  Print_tx_rx.print("Message = ");
  Print_tx_rx.println(msg);
  Print_tx_rx.println("-----Message Form End-----");

  Print_tx_rx.print(F("[STM32WL] Transmitting packet ... "));

  int state = radio.transmit(msg);


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
  Print_tx_rx.println("------------------------------------------------");
  digitalWrite(PA9, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(2000);              // wait for a second
  digitalWrite(PA9, LOW);   // turn the LED off by making the voltage LOW
  // delay(1.8e+6);
  delay(2000);
}
