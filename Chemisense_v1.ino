#include <ADS124S06.h>
// #include "HC4067.h"
#include <light_CD74HC4067.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <SPI.h>

#define SEALEVELPRESSURE_HPA (1013.25)

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

SPISettings SPI_SETTINGS_ADS(1000000, MSBFIRST, SPI_MODE1);
ADS124S06 ads(CS_ADS, SPI_SETTINGS_ADS);
// HC4067 mux_adc(MUX_ADC_0, MUX_ADC_1, MUX_ADC_2, MUX_ADC_3, MUX_ADC_EN);
// HC4067 mux_cur(MUX_CUR_0, MUX_CUR_1, MUX_CUR_2, MUX_CUR_3, MUX_CUR_EN);
CD74HC4067 mux_adc(MUX_ADC_0, MUX_ADC_1, MUX_ADC_2, MUX_ADC_3);
CD74HC4067 mux_cur(MUX_CUR_0, MUX_CUR_1, MUX_CUR_2, MUX_CUR_3);
Adafruit_BME680 bme_sensor(CS_BME_0);
Adafruit_BME680 bme_board(CS_BME_1);

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

  // Initialize the ADS124S06 (and SPI)
  ads.begin();
  ads.adcStartupRoutine();
  setupRegisters();
  ads.startConversions();
  delay(10);

  mux_adc.channel(0);  // Set the channel for mux_adc
  mux_cur.channel(0);  // Set the channel for mux_cur
  enableMUX();

  RGBLED('-', 0);
  RGBLED('G', LED_DIM);
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

float equivalentResistance(int32_t adcValue) {
  // Calculate the voltage before the gain stage
  float bufferVoltage = (adcValue * VREF) / ADC_RANGE;
  Serial.print("Buffer Voltage (before gain adjustment): ");
  Serial.println(bufferVoltage, 10);

  // Calculate the voltage after the gain stage
  float adcVoltage = bufferVoltage / gain;
  Serial.print("ADC Voltage (after gain adjustment): ");
  Serial.println(adcVoltage, 10);

  float R_eq = adcVoltage / currentSource;
  Serial.print("Equivalent Resistance (R_eq): ");
  Serial.println(R_eq, 10);

  float R_v = 1 / ((1 / R_eq) - (1 / fixedResistor));
  Serial.print("Rv: ");
  Serial.println(R_v, 3);

  return R_v;
}

void enableMUX() {
  digitalWrite(MUX_ADC_EN, LOW);
  digitalWrite(MUX_CUR_EN, LOW);
}

void disableMUX() {
  digitalWrite(MUX_ADC_EN, HIGH);
  digitalWrite(MUX_CUR_EN, HIGH);
}

void loop() {
  // int channel = 7;
  // // for (int channel = 0; channel < 16; channel++) {
  // mux_adc.channel(channel);  // Set the channel for mux_adc
  // mux_cur.channel(channel);  // Set the channel for mux_cur
  // delay(100);

  // // Read the conversion result
  // int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);
  // float R_v = equivalentResistance(adcValue);
  // // Print the result
  // // Serial.print("Channel: ");
  // // Serial.print(channel);
  // // Serial.print(", ADC: ");
  // // Serial.print(adcValue);
  // // Serial.print(", R_v: ");
  // // Serial.println(R_v, 4);
  // delay(1000);
  // // }
  // Serial.println("");

  // delay(1000);

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the input until newline character
    int channel = input.toInt();                  // Convert the input to an integer
    if (channel >= 0 && channel <= 15) {
      mux_adc.channel(channel);  // Set the channel for mux_adc
      mux_cur.channel(channel);  // Set the channel for mux_cur

      Serial.print("Channel set to: ");
      Serial.println(channel);

      mux_adc.channel(channel);  // Set the channel for mux_adc
      mux_cur.channel(channel);  // Set the channel for mux_cur
      delay(100);
      int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);
      float R_v = equivalentResistance(adcValue);
    } else {
      Serial.println("Invalid channel. Enter a number between 0 and 15.");
    }
  }
}
