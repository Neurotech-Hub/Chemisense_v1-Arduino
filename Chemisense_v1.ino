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
#define MUX_DELAY 5 // ms

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

const float VREF = 2.5;             // Reference voltage
const long ADC_RANGE = 8388607;     // 24-bit ADC range adjusted for unipolar measurement (2^23 - 1)
const float currentSource = 10e-6;  // Current source value (10 microamperes)
const float fixedResistor = 10000;  // Fixed resistor value (10k ohms)
const float gain = 25;              // Sensor gain
uint8_t ADCstatus[1];
float spec_k[16] = { 1, 4.7, 10, 0, 1, 4.7, 10, 0, 1, 4.7, 10, 0, 1, 4.7, 10, 0 };
float ADC_OFFSET = 411;  // default
bool sdInitialized = false;

SPISettings SPI_SETTINGS_ADS(500000, MSBFIRST, SPI_MODE1);
ADS124S06 ads(CS_ADS, SPI_SETTINGS_ADS);
CD74HC4067 mux_adc(MUX_ADC_0, MUX_ADC_1, MUX_ADC_2, MUX_ADC_3);
CD74HC4067 mux_cur(MUX_CUR_0, MUX_CUR_1, MUX_CUR_2, MUX_CUR_3);
Adafruit_BME680 bme_sensor(CS_BME_0);
Adafruit_BME680 bme_board(CS_BME_1);

void setup() {
  WiFiDrv::pinMode(25, OUTPUT);  //define GREEN LED
  WiFiDrv::pinMode(26, OUTPUT);  //define RED LED
  WiFiDrv::pinMode(27, OUTPUT);  //define BLUE LED
  RGBLED('R', LED_DIM);
  Serial.begin(9600);
  delay(2000);  // wait for serial

  // Init GPIO
  pinMode(CS_BME_0, OUTPUT);
  pinMode(CS_BME_1, OUTPUT);
  pinMode(CS_SD, OUTPUT);
  pinMode(CS_ADS, OUTPUT);

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
  setupRegisters();
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
  }
  digitalWrite(CS_SD, HIGH);

  // mux_cur.disable();
  // mux_adc.disable();

  // if (!bme_sensor.begin() || !bme_board.begin()) {
  //   Serial.println("Could not find both BME680 sensors, check wiring!");
  //   while (1)
  //     ;
  // }

  // Set up oversampling and filter initialization
  // bme_sensor.setTemperatureOversampling(BME680_OS_8X);
  // bme_sensor.setHumidityOversampling(BME680_OS_2X);
  // bme_sensor.setPressureOversampling(BME680_OS_4X);
  // bme_sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
  // bme_sensor.setGasHeater(320, 150);  // 320*C for 150 ms
  // bme_board.setTemperatureOversampling(BME680_OS_8X);
  // bme_board.setHumidityOversampling(BME680_OS_2X);
  // bme_board.setPressureOversampling(BME680_OS_4X);
  // bme_board.setIIRFilterSize(BME680_FILTER_SIZE_3);
  // bme_board.setGasHeater(320, 150);  // 320*C for 150 ms

  RGBLED('-', 0);
  RGBLED('G', LED_DIM);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the input until newline character
    int channel = input.toInt();                  // Convert the input to an integer
    if (channel >= 0 && channel <= 15) {
      mux_adc.channel(channel);  // Set the channel for mux_adc
      mux_cur.channel(channel);  // Set the channel for mux_cur

      Serial.print("Channel set to: ");
      Serial.println(channel);
      ads.sendSTART();
      delay(MUX_DELAY);
      int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT); // DIRECT
      float R_v = equivalentResistance(adcValue);
    } else {
      Serial.println("Running calibration...");
      if (runCalibration()) {
        readCalibrationValues();
      }
    }
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
  int32_t adcValues[16];
  float measuredRValues[16];

  // Sample ADC values for each channel
  for (int channel = 0; channel < 16; channel++) {
    // Set the channel for mux_adc and mux_cur
    mux_adc.channel(channel);
    mux_cur.channel(channel);

    Serial.print("Sampling channel ");
    Serial.println(channel);

    ads.sendSTART();
    delay(MUX_DELAY);
    // Read the conversion result
    int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);
    float R = equivalentResistance(adcValue);

    // Store the values in arrays
    adcValues[channel] = adcValue;
    measuredRValues[channel] = R;
  }

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
void setupRegisters() {
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

void printBME_sensor() {
  if (!bme_sensor.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme_sensor.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme_sensor.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme_sensor.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme_sensor.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme_sensor.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
}

float equivalentResistance(int32_t adcValue) {
  Serial.print("ADC Value: ");
  Serial.print(adcValue);
  Serial.print(" (-");
  Serial.print(ADC_OFFSET, 2);
  Serial.println(")");

  // Calculate the voltage before the gain stage
  float bufferVoltage = ((adcValue - ADC_OFFSET) * VREF) / ADC_RANGE;
  Serial.print("ADC Node (V): ");
  Serial.println(bufferVoltage, 6);

  // Calculate the voltage after the gain stage
  float adcVoltage = bufferVoltage / gain;
  // Serial.print("Pre-buffer (V): ");
  // Serial.println(adcVoltage, 6);

  float R_eq = adcVoltage / currentSource;
  // Serial.print("Equivalent Resistance (R_eq): ");
  // Serial.println(R_eq, 3);

  float R = 1 / ((1 / R_eq) - (1 / fixedResistor));
  if (R < 0) {
    R = 0;
  }
  Serial.print("Rv (kOhm): ");
  Serial.println(R / 1000.0, 3);
  Serial.println("");

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