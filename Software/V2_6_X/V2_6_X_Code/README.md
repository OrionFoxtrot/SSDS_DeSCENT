# V2_6_X_Code

Flight software for the DeSCENT ChipSats (Wio-E5 boards)

## Before flashing
Configure src/config.h - most importantly, look at the id `kChipSatId` and radio module `CHIPSAT_RADIO_MODULE`

## Build
Generic WLE5JCIx, upload method SWD

## Notes

- IMU data older than 3 s goes out marked invalid
- a report quiet for 3 cycles gets the IMU soft reset, 3 times a boot at most
- our own IMU report handler, the library only keeps the last report per I2C read
- packets print as `I PKT` lines too and descent_decode.py reads them
- no watchdog yet so a failed I2C write to the IMU still hangs the board
