#include "LoRaRadio.hpp"
#include "Constants.hpp"

LoRaRadio::LoRaRadio(const uint32_t *rfswitch_pins, const Module::RfSwitchMode_t *rfswitch_table){
  radio = new STM32WLx_Module();
  _rfswitch_pins = rfswitch_pins;
  _rfswitch_table = rfswitch_table;
}

bool LoRaRadio::begin(float freq, float power)
{
  // initialize STM32WL with default settings, except frequency
  radio.setRfSwitchTable(_rfswitch_pins, _rfswitch_table);
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
