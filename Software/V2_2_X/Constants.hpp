#pragma once
#define Print_rxPin PB7
#define Print_txPin PB6
#define GPS_rxPin PC1
#define GPS_txPin PC0
#include <SoftwareSerial.h>

extern SoftwareSerial Print_tx_rx;
extern SoftwareSerial GPS_tx_rx;