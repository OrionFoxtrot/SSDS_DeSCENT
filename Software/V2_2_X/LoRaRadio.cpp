#include "LoRaRadio.hpp"
#include "Constants.hpp"
using namespace std;
#include <array>


LoRaRadio::LoRaRadio(std::array<uint32_t, 5> rfswitch_pins, std::array<Module::RfSwitchMode_t, 5> rfswitch_table)
  : module(new STM32WLx_Module()),
    radio(module),
    _rfswitch_pins(rfswitch_pins),
    _rfswitch_table(rfswitch_table)
{
  // Convert std::array to C-style array for setRfSwitchTable
  uint32_t pins[5];
  std::copy(_rfswitch_pins.begin(), _rfswitch_pins.end(), pins);
  radio.setRfSwitchTable(pins, _rfswitch_table.data());
}

bool LoRaRadio::begin(float freq, float power)
{
  // initialize STM32WL with default settings, except frequency
  // radio.setRfSwitchTable(_rfswitch_pins, _rfswitch_table);
  int state = radio.begin(freq);
  radio.setOutputPower(power);

  if (state == RADIOLIB_ERR_NONE)
  {
    Print_tx_rx.println(F("success!"));
  }
  else
  {
    Print_tx_rx.print(F("failed, code "));
    Print_tx_rx.println(state);
    while (true)
    {
      delay(10);
    }
  }
  return true;
}

bool LoRaRadio::setTCXO(float voltage)
{
  // set appropriate TCXO voltage for Nucleo WL55JC1
  int state = radio.setTCXO(voltage);
  if (state == RADIOLIB_ERR_NONE)
  {
    Print_tx_rx.println(F("success!"));
  }
  else
  {
    Print_tx_rx.print(F("failed, code "));
    Print_tx_rx.println(state);
    while (true)
    {
      delay(10);
    }
  }
}

int LoRaRadio::transmit(String payload) //deleted constants and reference
{
  int state = radio.transmit(payload);
  return state;
}

void LoRaRadio::interpretState(int state)
{
  SoftwareSerial debug = Print_tx_rx; 
  switch (state)
  {
  case RADIOLIB_ERR_NONE:
    debug.println(F("[LoRaRadio] Packet transmitted successfully!"));
    debug.print(F("[LoRaRadio] Data rate: "));
    debug.print(radio.getDataRate());
    debug.println(F(" bps"));
    break;

  case RADIOLIB_ERR_PACKET_TOO_LONG:
    debug.println(F("[LoRaRadio] Packet too long!"));
    break;

  case RADIOLIB_ERR_TX_TIMEOUT:
    debug.println(F("[LoRaRadio] Transmission timeout!"));
    break;

  default:
    debug.print(F("[LoRaRadio] Transmission failed, code: "));
    debug.println(state);
    break;
  }
}
