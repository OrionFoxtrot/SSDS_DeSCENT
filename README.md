# SSDS DeSCENT

This repository contains hardware, software, documentation, and test data for the Space Systems Design Studio DeSCENT ChipSat effort.

DeSCENT, the Demonstration of Suborbital ChipSats Ejected from a New Shepard Test Flight, develops small ChipSat payloads, a release/deployer system, and the supporting test software used to validate power, sensing, telemetry, and recovery workflows.

## Repository contents

```text
Documentation/                         Mission, requirements, ConOps, verification, and programming documents
Hardware/                              ChipSat PCB files, CAD files, component datasheets, and hardware notes
Software/                              Arduino and MATLAB test scripts for ChipSat and ground station work
Descent_V2_GPS_1575_600_20.cal         RF calibration file for the V2 GPS path
Descent_V2_LoRa_915_400_10.cal         RF calibration file for the V2 LoRa path
LICENSE                                MIT license
```

## Documentation

The `Documentation/` folder stores project level references and mission documents, including:

- `SSDS-DESCENT-ERD-001.pdf` — engineering requirements document
- `SSDS-DeSCENT-ConOps.pdf` — concept of operations
- `DeSCENT_VCRM.xlsx` — verification compliance and requirements matrix
- `V2_Programming_Instructions.pdf` — programming instructions for the V2 ChipSat hardware
- `Alpha ChipSat Specsheet.pdf` and `DeSCENT ChipSat Specsheet.pdf` — ChipSat specification sheets

## Hardware

The `Hardware/` folder contains the electrical and mechanical design files for the ChipSat hardware and deployer. It includes:

- Altium project artifacts for V1.90 ChipSat hardware
- V2 ChipSat hardware folders
- 2025 CAD and deployer CAD files
- component datasheets for the Wio E5, BNO085, BME280, W25Q16JV flash, L6920 regulator, and related parts
- a hardware specific README describing V1 and V2 board families

V1 ChipSats use a dedicated MCU and LoRa radio. V2 ChipSats use the Seeed Studio Wio E5 STM32WL module as the combined MCU and LoRa radio platform.

## Software

The `Software/` folder contains prototype and test scripts used during board bring up, sensor verification, telemetry testing, and power characterization.

Key software areas include:

- `V1_Breadboard_Test_Scripts/` — early V1 tests for blink, GPS, IMU, BME280, LoRa, and combined LoRa GPS operation
- `V2_1_0_Test_Scripts/` and `V2_1_1_Test_Scripts/` — Wio E5 tests for blink, LoRa, GPS, BME280, IMU, flash, I2C scanning, and combined function checks
- `Multi_V_I_Sensor_Logger/` and `voltage_current_logger/` — INA260 voltage and current logging sketches
- `Power_Testing/` — CSV data, MATLAB plotting scripts, and figures for charge, drain, and solar assisted power tests
- `T_Beam_Scripts/` — LoRa receiver and T-Beam support scripts for ground station or telemetry testing

Most Arduino sketches are standalone tests. Confirm the target board, pin definitions, power rails, and connected sensors before flashing a script to flight like hardware.

## Typical setup

1. Install the Arduino IDE or another compatible Arduino build environment.
2. Install the board support package needed for the target board, such as STM32 support for Wio E5 based tests or ESP32 support for T-Beam receiver tests.
3. Install the libraries used by the relevant sketch. Common libraries include:
   - RadioLib
   - Adafruit Sensor
   - Adafruit BME280
   - Adafruit INA260
   - SparkFun BNO080 or BNO08x libraries, depending on the sketch
   - TinyGPSPlus
4. Open the desired `.ino` file from `Software/`.
5. Review the pin definitions and serial ports at the top of the sketch.
6. Compile and upload to the target board.
7. Use the serial monitor or logger output to verify sensor readings, radio status, or power measurements.

## Power test workflow

Power test scripts and data are stored in `Software/Power_Testing/`. The Arduino logger records voltage and current data, while the MATLAB scripts read CSV files and generate plots for battery, solar cell, and post regulator rails.

For a new power test:

1. Program the relevant INA260 logger sketch.
2. Save the serial output as a CSV file.
3. Update the `name` variable in the MATLAB plotting script to match the CSV filename.
4. Run the MATLAB script to generate voltage and current plots.

## Notes for contributors

- Keep board version information in folder names or file headers when adding new tests.
- Add a short comment block at the top of new sketches describing the target hardware, required libraries, expected sensor addresses, and serial baud rate.
- Avoid committing generated CAD history, temporary build products, or local editor metadata unless the file is intentionally being archived.
- Use clear names for test data files so the hardware revision, test condition, and date can be reconstructed later.

## Safety and usage

This repository contains research and prototype hardware artifacts. Scripts and design files may assume a specific board revision, wiring setup, or lab configuration. Verify connections and expected power levels before programming or powering hardware.

## License

This repository is licensed under the MIT License. See `LICENSE` for details.
