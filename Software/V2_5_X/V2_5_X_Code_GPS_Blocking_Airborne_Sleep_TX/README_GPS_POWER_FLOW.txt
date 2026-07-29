GPS and transmit power flow
===========================

1. sensors.begin() configures the MAX-M10S for normal 1 Hz operation and
   Airborne <4g. The configuration is written to RAM and BBR.
2. setup() calls waitForGPSFix() and blocks indefinitely until a valid 3D fix.
3. The LoRa radio is initialized only after that initial fix.
4. Each loop cycle reads and stores fresh sensor data.
5. Immediately before radio.transmit(), sleepForTransmit() puts the MAX-M10S
   into software standby and puts the BNO08x into sleep mode.
6. Transmission is skipped if either sensor fails to enter sleep.
7. After the radio call, wakeAfterTransmit() wakes both sensors.
8. wakeGPS() reapplies UBX-only UART output, Airborne <4g, and normal 1 Hz
   full-power acquisition/tracking. No PSM cyclic tracking is used.
9. The GPS may briefly need to reacquire after each wake. Until readGPS()
   confirms a new valid 3D fix, the GPS validity bit is false.
