#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;

// Debug UART to your serial adapter
#define Print_rxPin PB7
#define Print_txPin PB6
HardwareSerial Print_tx_rx(Print_rxPin, Print_txPin);

// GPS UART pins
// MCU RX pin connects to GPS TX
// MCU TX pin connects to GPS RX
#define GPS_rxPin PC1
#define GPS_txPin PC0
HardwareSerial GPSSerial(GPS_rxPin, GPS_txPin);

void setup()
{
  Print_tx_rx.begin(115200);
  delay(1000);

  Print_tx_rx.println();
  Print_tx_rx.println("MAX-M10S UART sleep test");

  // Change this if your GPS UART baud is different
  GPSSerial.begin(9600);
  delay(500);

  Print_tx_rx.println("Trying GPS begin over UART...");

  if (myGNSS.begin(GPSSerial) == false)
  {
    Print_tx_rx.println("GPS not detected. Check baud rate / wiring.");
    while (1)
    {
      delay(1000);
    }
  }

  Print_tx_rx.println("GPS detected.");
  Print_tx_rx.println("Measure current now: GPS should be awake.");
  delay(10000);

  Print_tx_rx.println("Sending GPS standby command for 20 seconds...");

  bool ok = myGNSS.powerOffWithInterrupt(
    20000,                              // sleep duration in ms
    VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX   // allow UART RX to wake it
  );

  if (ok)
    Print_tx_rx.println("Standby command ACKed.");
  else
    Print_tx_rx.println("Standby command failed / no ACK.");

  Print_tx_rx.println("Do NOT talk to GPS now. Measure current drop.");
  Print_tx_rx.println("Waiting 25 seconds...");

  // Important: do not call myGNSS.getPVT(), checkUblox(), etc. here.
  delay(25000);

  Print_tx_rx.println("GPS should be awake again now.");
  Print_tx_rx.println("Trying to read position/status...");

  // Re-open serial just in case
  GPSSerial.begin(9600);
  delay(500);

  if (myGNSS.getPVT(2000))
  {
    Print_tx_rx.println("GPS responded after sleep.");
    Print_tx_rx.print("Fix type: ");
    Print_tx_rx.println(myGNSS.getFixType());
    Print_tx_rx.print("SIV: ");
    Print_tx_rx.println(myGNSS.getSIV());
  }
  else
  {
    Print_tx_rx.println("GPS did not respond after sleep. Try reset or longer wait.");
  }
}

void loop()
{
  // Do nothing. This is a one-shot test.
}