#include "SystemManager.hpp"
#include <SoftwareSerial.h>

#define PRINT_RX PB7
#define PRINT_TX PB6
// SoftwareSerial PrintSerial(PRINT_RX, PRINT_TX);

#define GPS_RX PC1
#define GPS_TX PC0

// LoRa RF switch pins/table
const uint32_t rfswitch_pins[] = {PC_3, PC_4, PC_5, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE, {LOW, LOW, LOW}},
    {STM32WLx::MODE_RX, {HIGH, HIGH, LOW}},
    {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},
    {STM32WLx::MODE_TX_HP, {HIGH, LOW, HIGH}},
    END_OF_MODE_TABLE,
};

// Create system manager instance
SystemManager systemManager(PrintSerial, rfswitch_pins, rfswitch_table, GPS_RX, GPS_TX);

void setup()
{
  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, LOW);

  systemManager.begin();
}

void loop()
{
  systemManager.loop();
}
