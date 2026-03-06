#include "includes/communication/LoRaRadio.hpp"
#include "includes/Constants.hpp"
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
    Print_tx_rx.println(F("success! (begin)"));
  }
  else
  {
    Print_tx_rx.print(F("failed (begin), code "));
    Print_tx_rx.println(state);
    while (true)
    {
      delay(10);
    }
  }
  return true;
}

void LoRaRadio::setTCXO(float voltage)
{
  // set appropriate TCXO voltage for Nucleo WL55JC1

  int state = radio.setTCXO(voltage);
  Print_tx_rx.print(F("state before if: "));
  Print_tx_rx.println(state);
  if (state == RADIOLIB_ERR_NONE)
  {
    Print_tx_rx.println(F("success! (second)"));
  }
  else
  {
    Print_tx_rx.print(F("failed (second), code "));
    Print_tx_rx.println(state);
    while (true)
    {
      delay(10);
    }
  }
}

// int LoRaRadio::transmit(uint8_t *payload, size_t length) // deleted constants and reference
// {
//   int state = radio.transmit(payload, length);
//   return state;
// }
volatile bool txDone = false;

void onTxDone()
{
  txDone = true;
}

int LoRaRadio::transmit(uint8_t *payload, size_t length)
{
  // Start non-blocking transmission
  int state = radio.startTransmit(payload, length);
  if (state != RADIOLIB_ERR_NONE)
  {
    return state;
  }

  // Attach DIO1 interrupt
  radio.setDio1Action(onTxDone);

  // Wait for completion or timeout
  unsigned long start = millis();
  while (!txDone && (millis() - start < 5000))
  {
    delay(1);
  }

  if (!txDone)
  {
    state = RADIOLIB_ERR_TX_TIMEOUT;
  }

  txDone = false; // Reset flag
  return state;
}

void LoRaRadio::interpretState(int state)
{
  // SoftwareSerial &debug = Print_tx_rx;
  delay(5);
  switch (state)
  {
  case RADIOLIB_ERR_NONE:
    Print_tx_rx.println(F("[LoRaRadio] Packet transmitted successfully!"));
    Print_tx_rx.print(F("[LoRaRadio] Data rate: "));
    Print_tx_rx.print(radio.getDataRate());
    Print_tx_rx.println(F(" bps"));
    break;

  case RADIOLIB_ERR_PACKET_TOO_LONG:
    Print_tx_rx.println(F("[LoRaRadio] Packet too long!"));
    break;

  case RADIOLIB_ERR_TX_TIMEOUT:
    Print_tx_rx.println(F("[LoRaRadio] Transmission timeout!"));
    break;

  default:
    Print_tx_rx.print(F("[LoRaRadio] Transmission failed, code: "));
    Print_tx_rx.println(state);
    break;
  }

  // Print_tx_rx.flush();
  // delay(5);
}
