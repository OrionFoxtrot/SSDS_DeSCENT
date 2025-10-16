// LoRaRadio.h
#pragma once
#include <RadioLib.h>


class LoRaRadio
{
public:
  LoRaRadio(const uint32_t *rfswitch_pins, const Module::RfSwitchMode_t *rfswitch_table);
  bool begin(float freq, float power = 14.0);
  bool setTCXO(float voltage);
  int transmit(String payload);
  void interpretState(int state);

private:
  STM32WLx radio;
  const uint32_t *_rfswitch_pins;
  const Module::RfSwitchMode_t *_rfswitch_table;
};
