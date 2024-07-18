/*
By Matt Gaidica, PhD

Notes: The native SD library is having trouble with CS assertion so it is being handled manually.
*/

#include <ADS124S06.h>
#include <light_CD74HC4067.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <SPI.h>
#include <SD.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define MUX_DELAY 5  // ms

#define CS_BME_0 13
#define CS_BME_1 14
#define CS_SD 12
#define CS_ADS A5

#define MUX_ADC_EN A6
#define MUX_ADC_0 0
#define MUX_ADC_1 1
#define MUX_ADC_2 2
#define MUX_ADC_3 3

#define MUX_CUR_EN A4
#define MUX_CUR_0 A0
#define MUX_CUR_1 A1
#define MUX_CUR_2 A2
#define MUX_CUR_3 A3

#define SWITCH_0 4
#define SWITCH_1 5
#define SD_DET 6
#define FAN_PWM 7

#define LED_DIM 25
#define LED_BRIGHT 255

char fileName[13];  // Buffer to hold file name in the format "00000.TXT"

const float VREF = 2.5;             // Reference voltage
const long ADC_RANGE = 8388607;     // 24-bit ADC range adjusted for unipolar measurement (2^23 - 1)
const float currentSource = 10e-6;  // Current source value (10µA)
const float fixedResistor = 10000;  // Fixed resistor value (10kOhm)
const float gain = 25;              // Sensor gain
uint8_t ADCstatus[1];
float spec_k[16] = { 1, 4.7, 10, 0, 1, 4.7, 10, 0, 1, 4.7, 10, 0, 1, 4.7, 10, 0 };
float ADC_OFFSET = 411;  // default
bool sdInitialized = false;
bool error = false;
int fadeLED = 0;
bool fadeDir = false;

float temperature_die;
uint32_t pressure_die;
float humidity_die;
uint32_t gas_resistance_die;
float altitude_die;

float temperature_topside;
uint32_t pressure_topside;
float humidity_topside;
uint32_t gas_resistance_topside;
float altitude_topside;

int32_t adcValues[16];
float measuredRValues[16];

SPISettings SPI_SETTINGS_ADS(500000, MSBFIRST, SPI_MODE1);
ADS124S06 ads(CS_ADS, SPI_SETTINGS_ADS);
CD74HC4067 mux_adc(MUX_ADC_0, MUX_ADC_1, MUX_ADC_2, MUX_ADC_3);
CD74HC4067 mux_cur(MUX_CUR_0, MUX_CUR_1, MUX_CUR_2, MUX_CUR_3);
Adafruit_BME680 bme_die(CS_BME_0);
Adafruit_BME680 bme_topside(CS_BME_1);

void setup() {
  WiFiDrv::pinMode(25, OUTPUT);  //define GREEN LED
  WiFiDrv::pinMode(26, OUTPUT);  //define RED LED
  WiFiDrv::pinMode(27, OUTPUT);  //define BLUE LED
  RGBLED('B', LED_DIM);
  Serial.begin(9600);
  delay(2000);  // wait for serial
  Serial.println("\n\n***Hello, Chemisensor.***");

  // Init GPIO
  pinMode(CS_BME_0, OUTPUT);
  digitalWrite(CS_BME_0, HIGH);
  pinMode(CS_BME_1, OUTPUT);
  digitalWrite(CS_BME_1, HIGH);
  pinMode(CS_SD, OUTPUT);
  digitalWrite(CS_SD, HIGH);
  pinMode(CS_ADS, OUTPUT);
  digitalWrite(CS_ADS, HIGH);

  pinMode(MUX_ADC_EN, OUTPUT);
  digitalWrite(MUX_ADC_EN, HIGH);
  pinMode(MUX_ADC_0, OUTPUT);
  pinMode(MUX_ADC_2, OUTPUT);
  pinMode(MUX_ADC_1, OUTPUT);
  pinMode(MUX_ADC_3, OUTPUT);

  pinMode(MUX_CUR_EN, OUTPUT);
  digitalWrite(MUX_CUR_EN, HIGH);
  pinMode(MUX_CUR_0, OUTPUT);
  pinMode(MUX_CUR_1, OUTPUT);
  pinMode(MUX_CUR_2, OUTPUT);
  pinMode(MUX_CUR_3, OUTPUT);

  pinMode(SWITCH_0, INPUT_PULLUP);
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(SD_DET, INPUT_PULLUP);
  pinMode(FAN_PWM, OUTPUT);
  analogWrite(FAN_PWM, 255);  // 0-255

  // Initialize the ADS124S06 (and SPI)
  ads.begin();
  ads.adcStartupRoutine();
  configureADS();
  // ads.startConversions();

  mux_adc.channel(0);  // Set the channel for mux_adc
  mux_cur.channel(0);  // Set the channel for mux_cur
  enableMUX();

  // Try initializing SD card up to 5 times
  digitalWrite(CS_SD, LOW);
  for (int attempt = 0; attempt < 5; attempt++) {
    if (SD.begin(CS_SD, SPI_HALF_SPEED)) {
      sdInitialized = true;
      break;
    } else {
      Serial.print("SD card initialization attempt ");
      Serial.print(attempt + 1);
      Serial.println(" failed.");
      delay(200);  // Wait for 200ms before the next attempt
    }
  }

  if (sdInitialized) {
    Serial.println("SD card initialized.");
    readCalibrationValues();
  } else {
    Serial.println("SD card initialization failed after 5 attempts.");
    error = true;
  }
  digitalWrite(CS_SD, HIGH);

  if (!bme_die.begin() || !bme_topside.begin()) {
    Serial.println("Could not find both BME680 sensors, check wiring!");
    error = true;
  } else {
    configureBME(bme_die);
    Serial.println("\nBME near die:");
    readBME_die(true);
    configureBME(bme_topside);
    Serial.println("\nBME topside:");
    readBME_topside(true);
  }

  if (sdInitialized) {
    attachInterrupt(digitalPinToInterrupt(SD_DET), sdCardRemoved, FALLING);
  }

  Serial.println("\n-----\nCommands:\n0-15 (black button): read channels\n55 (red button): log data\n99: calibrate\n-----");
  RGBLED('-', 0);
}

void loop() {
  if (Serial.available() > 0) {
    RGBLED('-', 0);
    RGBLED('B', LED_DIM);
    String input = Serial.readStringUntil('\n');  // Read the input until newline character
    int channel = input.toInt();                  // Convert the input to an integer
    if (channel >= 0 && channel <= 15) {
      mux_adc.channel(channel);  // Set the channel for mux_adc
      mux_cur.channel(channel);  // Set the channel for mux_cur

      Serial.print("\nChannel set to: ");
      Serial.println(channel);
      ads.sendSTART();
      delay(MUX_DELAY);
      int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);  // DIRECT
      float R_v = calcADS_R(adcValue, true);
    } else if (channel == 99) {
      Serial.println("\nRunning calibration...");
      if (runCalibration()) {
        readCalibrationValues();
      }
    } else if (channel == 55) {
      logData();
    } else {
      Serial.println("Invalid command.");
    }
    RGBLED('-', 0);
  }

  if (digitalRead(SWITCH_0) == LOW) {
    Serial.println("Logging...");
    logData();
    while (digitalRead(SWITCH_0) == LOW) {
      // wait for button to release
    }
  }

  if (digitalRead(SWITCH_1) == LOW) {
    sampleAllChannels();
    for (int i = 0; i < 16; i++) {
      Serial.print(i);
      Serial.print(",");
      Serial.println(measuredRValues[i], 4);
    }
    while (digitalRead(SWITCH_1) == LOW) {
      // wait for button to release
    }
  }

  if (error) {
    while (1) {
      RGBLED('R', LED_DIM);
      delay(200);
      RGBLED('-', 0);
      delay(200);
    }
  }

  if (fadeLED == 0 || fadeLED == 100) {
    fadeDir = !fadeDir;
  }
  if (fadeDir) {
    fadeLED++;
  } else {
    fadeLED--;
  }
  RGBLED('G', fadeLED);
  delay(10);
}

void logData() {
  RGBLED('R', LED_DIM);
  sampleAllChannels();
  readBME_die(false);
  readBME_topside(false);

  int nextFileNumber = getNextFileNumber();
  snprintf(fileName, sizeof(fileName), "%05d.TXT", nextFileNumber);
  Serial.print("Creating file: ");
  Serial.println(fileName);

  digitalWrite(CS_SD, LOW);
  File dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile) {
    // Write the header
    dataFile.println("key,value");

    // // Write the measuredRValues array
    for (int i = 0; i < 16; i++) {
      dataFile.print(i);
      dataFile.print(",");
      dataFile.println(measuredRValues[i], 4);
    }

    // // write sensors
    dataFile.print("temperature_die,");
    dataFile.println(temperature_die, 2);
    dataFile.print("pressure_die,");
    dataFile.println(pressure_die);
    dataFile.print("humidity_die,");
    dataFile.println(humidity_die, 2);
    dataFile.print("gas_resistance_die,");
    dataFile.println(gas_resistance_die);
    dataFile.print("altitude_die,");
    dataFile.println(altitude_die, 2);

    dataFile.print("temperature_topside,");
    dataFile.println(temperature_topside, 2);
    dataFile.print("pressure_topside,");
    dataFile.println(pressure_topside);
    dataFile.print("humidity_topside,");
    dataFile.println(humidity_topside, 2);
    dataFile.print("gas_resistance_topside,");
    dataFile.println(gas_resistance_topside);
    dataFile.print("altitude_topside,");
    dataFile.println(altitude_topside, 2);

    dataFile.close();
    Serial.println("Data logged successfully.");
  } else {
    Serial.println("Error opening file.");
  }
  digitalWrite(CS_SD, HIGH);
  RGBLED('-', 0);
}

int getNextFileNumber() {
  int fileNumber = 0;
  digitalWrite(CS_SD, LOW);
  while (fileNumber < 100000) {
    snprintf(fileName, sizeof(fileName), "%05d.TXT", fileNumber);
    if (!SD.exists(fileName)) {
      break;
    }
    fileNumber++;
  }
  digitalWrite(CS_SD, HIGH);
  return fileNumber;
}

void sdCardRemoved() {
  detachInterrupt(digitalPinToInterrupt(SD_DET));  // Detach the interrupt
  error = true;
}

void sampleAllChannels() {
  // Sample ADC values for each channel
  for (int channel = 0; channel < 16; channel++) {
    // Set the channel for mux_adc and mux_cur
    mux_adc.channel(channel);
    mux_cur.channel(channel);

    ads.sendSTART();
    delay(MUX_DELAY);
    // Read the conversion result
    int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);
    float R = calcADS_R(adcValue, false);

    // Store the values in arrays
    adcValues[channel] = adcValue;
    measuredRValues[channel] = R;
  }
}

bool readCalibrationValues() {
  bool success = false;

  digitalWrite(CS_SD, LOW);
  File dataFile = SD.open("CAL.TXT");

  if (dataFile) {
    int32_t adcValue;
    float measuredRValue;
    float sumADC = 0;
    int count = 0;

    while (dataFile.available()) {
      // Read each line of the file
      String line = dataFile.readStringUntil('\n');

      // Skip the header line
      if (line.startsWith("channel")) {
        continue;
      }

      // Parse the line to extract values
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);

      int channel = line.substring(0, firstComma).toInt();
      adcValue = line.substring(firstComma + 1, secondComma).toInt();
      measuredRValue = line.substring(secondComma + 1).toFloat();

      // Check if spec_k is 0 for the current channel
      if (spec_k[channel] == 0) {
        sumADC += adcValue;
        count++;
      }
    }

    // Calculate the mean ADC value for spec_k == 0
    if (count > 0) {
      ADC_OFFSET = sumADC / count;
    }

    dataFile.close();
    Serial.print("Updated ADC_OFFSET: ");
    Serial.println(ADC_OFFSET);
    success = true;
  } else {
    Serial.println("Failed to open CAL.TXT for reading.");
  }
  digitalWrite(CS_SD, HIGH);
  return success;
}

bool runCalibration() {
  bool success = false;

  sampleAllChannels();  // fill buffers

  // Write the sampled data to the SD card
  digitalWrite(CS_SD, LOW);
  // Ensure the calibration file is overwritten
  if (SD.exists("CAL.TXT")) {
    SD.remove("CAL.TXT");
  }

  File dataFile = SD.open("CAL.TXT", FILE_WRITE);

  if (dataFile) {
    // Write the CSV header
    dataFile.println("channel,measured_ADC_value,measured_R_value");

    for (int channel = 0; channel < 16; channel++) {
      // Write the data to the SD card
      dataFile.print(channel);
      dataFile.print(',');
      dataFile.print(adcValues[channel]);
      dataFile.print(',');
      dataFile.print(measuredRValues[channel], 4);
      dataFile.println();
    }

    dataFile.close();
    Serial.println("Calibration data saved to CAL.TXT.");
    success = true;
  } else {
    Serial.println("Failed to open CAL.TXT for writing.");
  }
  digitalWrite(CS_SD, HIGH);
  return success;
}

// Initialize register values according to datasheet recommendations
void configureADS() {
  // Set the internal reference voltage
  ads.writeSingleRegister(REG_ADDR_REF, ADS_REFSEL_INT | ADS_REFINT_ON_ALWAYS);

  // Set the IDAC current to 10µA on the AIN0 pin
  ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10);
  ads.writeSingleRegister(REG_ADDR_IDACMUX, ADS_IDAC1_A0 | ADS_IDAC2_OFF);

  // Set the input multiplexer to measure AIN1 referenced to AINCOM
  ads.writeSingleRegister(REG_ADDR_INPMUX, ADS_P_AIN1 | ADS_N_AINCOM);

  // Gain setting (PGA): Disable PGA, ensure gain=1
  ads.writeSingleRegister(REG_ADDR_PGA, PGA_DEFAULT);

  // Single-shot conversion (must send start)
  ads.writeSingleRegister(REG_ADDR_DATARATE, ADS_CONVMODE_SS | ADS_DR_800);
}

void configureBME(Adafruit_BME680 bme) {
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms
}

void readBME_die(bool debug) {
  if (!bme_die.performReading()) {
    Serial.println("Failed to perform BME reading.");
    return;
  }
  temperature_die = bme_die.temperature;                      // C
  pressure_die = bme_die.pressure / 100.0;                    // hPa
  humidity_die = bme_die.humidity;                            // %
  altitude_die = bme_die.readAltitude(SEALEVELPRESSURE_HPA);  // m
  gas_resistance_die = bme_die.gas_resistance / 1000.0;       // kOhms

  if (debug) {
    Serial.print("Temperature: ");
    Serial.print(temperature_die);
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(pressure_die);
    Serial.println(" hPa");

    Serial.print("Humidity: ");
    Serial.print(humidity_die);
    Serial.println(" %");

    Serial.print("Altitude: ");
    Serial.print(altitude_die);
    Serial.println(" m");

    Serial.print("Gas Resistance: ");
    Serial.print(gas_resistance_die);
    Serial.println(" kOhms");
  }
}

void readBME_topside(bool debug) {
  if (!bme_topside.performReading()) {
    Serial.println("Failed to perform BME reading.");
    return;
  }
  temperature_topside = bme_topside.temperature;                      // C
  pressure_topside = bme_topside.pressure / 100.0;                    // hPa
  humidity_topside = bme_topside.humidity;                            // %
  altitude_topside = bme_topside.readAltitude(SEALEVELPRESSURE_HPA);  // m
  gas_resistance_topside = bme_topside.gas_resistance / 1000.0;       // kOhms

  if (debug) {
    Serial.print("Temperature: ");
    Serial.print(temperature_topside);
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(pressure_topside);
    Serial.println(" hPa");

    Serial.print("Humidity: ");
    Serial.print(humidity_topside);
    Serial.println(" %");

    Serial.print("Altitude: ");
    Serial.print(altitude_topside);
    Serial.println(" m");

    Serial.print("Gas Resistance: ");
    Serial.print(gas_resistance_topside);
    Serial.println(" kOhms");
  }
}

float calcADS_R(int32_t adcValue, bool debug) {
  // Calculate the voltage before the gain stage
  float bufferVoltage = ((adcValue - ADC_OFFSET) * VREF) / ADC_RANGE;

  // Calculate the voltage after the gain stage
  float adcVoltage = bufferVoltage / gain;
  // Serial.print("Pre-buffer (V): ");
  // Serial.println(adcVoltage, 6);

  float R_eq = adcVoltage / currentSource;
  // Serial.print("Equivalent Resistance (R_eq): ");
  // Serial.println(R_eq, 3);

  float R = 1 / ((1 / R_eq) - (1 / fixedResistor));

  // fix negative, can occur at low R's due to offset
  if (R < 0) {
    R = 0;
  }

  if (debug) {
    Serial.print("ADC Value: ");
    Serial.print(adcValue);
    Serial.print(" (-");
    Serial.print(ADC_OFFSET, 2);
    Serial.println(")");

    Serial.print("ADC Node (V): ");
    Serial.println(bufferVoltage, 6);

    Serial.print("R (kOhm): ");
    Serial.println(R / 1000.0, 3);
    Serial.println("");
  }

  return R;
}

void enableMUX() {
  digitalWrite(MUX_ADC_EN, LOW);
  digitalWrite(MUX_CUR_EN, LOW);
}

void disableMUX() {
  digitalWrite(MUX_ADC_EN, HIGH);
  digitalWrite(MUX_CUR_EN, HIGH);
}

void RGBLED(char RGB, int state) {
  switch (RGB) {
    case 'R':
      WiFiDrv::analogWrite(25, state);  // RED
      break;
    case 'G':
      WiFiDrv::analogWrite(26, state);  // GREEN
      break;
    case 'B':
      WiFiDrv::analogWrite(27, state);  // BLUE
      break;
    default:  // not RGB, turn off
      WiFiDrv::analogWrite(25, LOW);
      WiFiDrv::analogWrite(26, LOW);
      WiFiDrv::analogWrite(27, LOW);
      break;
  }
}