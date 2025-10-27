// LoRaRadio.h
#pragma once
#include <RadioLib.h>


class LoRaRadio
{
public:
  LoRaRadio(std::array<uint32_t, 5> rfswitch_pins, std::array<Module::RfSwitchMode_t, 5> rfswitch_table);
  bool begin(float freq, float power = 14.0);
  bool setTCXO(float voltage);
  int transmit(String payload);
  void interpretState(int state);

private:
  STM32WLx_Module* module;  
  STM32WLx radio;
  std::array<uint32_t, 5> _rfswitch_pins; 
  std::array<Module::RfSwitchMode_t, 5> _rfswitch_table;  
  // const uint32_t *_rfswitch_pins;
  // const Module::RfSwitchMode_t *_rfswitch_table;
};
