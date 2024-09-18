# Chemisense
![Chemisense Module](Chemisense_v1_Module.jpg)

Performs resistance measuring of 16 channels and ambient sensing with data logging via:
- [Arduino MKR WiFi 1010 (MCU)](https://store-usa.arduino.cc/products/arduino-mkr-wifi-1010)
- [TI ADS124S06 (ADC)](https://www.ti.com/product/ADS124S06)
- [TI CD74HC4067 (MUX)](https://www.ti.com/product/CD74HC4067)
- [TI INA190A](https://www.ti.com/product/INA190)
- [Bosch BME680 Gas Sensor](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/)
- [Adafruit 4682 Micro-SD Card Breakout](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4682/12822319)

Schematic and PCB were designed using KiCad and are available by secure link.

## Usage
### Initialization (Power-up)
The LED will be blue during the following (and pulse green afterward):
- Initialize serial port (USB)
- Initialize IO
- Initialize ADC
- Initialize MUX and enable
- Initialize SD card
- Initialize gas sensors

### Operation
See serial output at init (or inspect setup()) for user inputs.

### Data Logging
All channels and gas data are logged in CSV format with headers (first row) "key,value". Reference [Neurotech-Hub/Chemisensor-MATLAB](https://github.com/Neurotech-Hub/Chemisensor-MATLAB) for reading these data in MATLAB.

### Calibration
Calibration utilizes one or many log files and fits a linear equation against known calibration values (calibration die). This relationship has been verified to be linear. See: [Chemisensor MATLAB/run_logSensitivity.m](https://github.com/Neurotech-Hub/Chemisensor-MATLAB/blob/main/run_logSensitivity.m).

Note, variables `MATLAB_SLOPE` and `MATLAB_INTERCEPT` should be set to 0 for these files, otherwise, previous calibrations will be applied to the saved data.

The calibration die has been measured using a precision meter; these values are stored on the original SD card and are hardcoded as the variable:

`const float spec_k[16] = { 1.001, 4.695, 10.021, 0.0, 1.002, 4.68, 9.955, 0.0, 0.999, 4.682, 10.016, 0.0, 0.999, 4.685, 9.95, 0.0 };  // measured values, also in B00.txt`

### Charging
This module currently uses a very simple battery switch while utilizing the Arduino's charging module. **Therefore, to charge the battery, the power switch must be ON.**

## Custom Dependencies
Clone this to Arduino/libraries:
- [Neurotech-Hub/ADS124S06-Arduino](https://github.com/Neurotech-Hub/ADS124S06-Arduino)
