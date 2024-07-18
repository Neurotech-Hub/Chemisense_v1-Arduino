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
Two buttons provide a non-serial UI:
1. Black: read all 16 channels and send results to the serial port.
2. Red: log all data to the SD card.

Serial commands (newline terminated):
- 0-15: reads an individual channel
- 55: log data
- 99: perform calibration

### Data Logging
All channels and gas data are logged in CSV format with headers (first row) "key,value". Reference [Neurotech-Hub/Chemisensor-MATLAB](https://github.com/Neurotech-Hub/Chemisensor-MATLAB) for reading these data in MATLAB.

### Calibration
The calibration utilizes a daughterboard with a known configuration of four 0Ω "shunt" resistors. It reads those channels and removes any offset in the ADC. The offset is saved on SD card in "CAL.TXT, " referenced during initialization.

## Custom Dependencies
Clone this to Arduino/libraries:
- [Neurotech-Hub/ADS124S06-Arduino](https://github.com/Neurotech-Hub/ADS124S06-Arduino)
