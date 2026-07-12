#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#define Print_rxPin PB7
#define Print_txPin PB6
#define GPS_rxPin PC1
#define GPS_txPin PC0

#include <HardwareSerial.h>
#include <SoftwareSerial.h>

extern SoftwareSerial Print_tx_rx;
extern HardwareSerial GPS_tx_rx;

// tbd
constexpr uint32_t DUTY_CYCLE_PERIOD_MS = 20000;
#endif