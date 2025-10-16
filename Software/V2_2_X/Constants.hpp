#define Print_rxPin PB7
#define Print_txPin PB6
#define GPS_rxPin PC1
#define GPS_txPin PC0
#include <SoftwareSerial.h>

SoftwareSerial Print_tx_rx = SoftwareSerial(Print_rxPin, Print_txPin);
SoftwareSerial GPS_tx_rx = SoftwareSerial(GPS_rxPin, GPS_txPin);