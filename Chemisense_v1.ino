#include "ADS124S06.h"
#include "HC4067.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

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

ADS124S06 ads(CS_ADS);
HC4067 mux_adc(MUX_ADC_0, MUX_ADC_1, MUX_ADC_2, MUX_ADC_3, MUX_ADC_EN);
HC4067 mux_cur(MUX_CUR_0, MUX_CUR_1, MUX_CUR_2, MUX_CUR_3, MUX_CUR_EN);
Adafruit_BME680 bme_sensor(CS_BME_0);
Adafruit_BME680 bme_board(CS_BME_1);

// Initialize register values according to datasheet recommendations
void setupRegisters() {
  // Initialize the ADS124S06 with the desired settings
  ads.restoreRegisterDefaults();

  // Set the internal reference voltage
  ads.writeSingleRegister(REG_ADDR_REF, ADS_REFSEL_INT | ADS_REFINT_ON_ALWAYS);

  // Set the IDAC current to 10µA on the AIN0 pin
  ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10);
  ads.writeSingleRegister(REG_ADDR_IDACMUX, ADS_IDAC1_A0 | ADS_IDAC2_OFF);

  // Set the input multiplexer to measure AIN1 referenced to AINCOM
  ads.writeSingleRegister(REG_ADDR_INPMUX, ADS_P_AIN1 | ADS_N_AINCOM);

  // Other configurations can be added here
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
  // Start serial communication for debugging
  Serial.begin(9600);

  // Init GPIO
  pinMode(CS_BME_0, OUTPUT);
  pinMode(CS_BME_1, OUTPUT);
  pinMode(CS_SD, OUTPUT);
  pinMode(CS_ADS, OUTPUT);

  pinMode(MUX_ADC_EN, OUTPUT);
  pinMode(MUX_ADC_0, OUTPUT);
  pinMode(MUX_ADC_2, OUTPUT);
  pinMode(MUX_ADC_1, OUTPUT);
  pinMode(MUX_ADC_3, OUTPUT);

  pinMode(MUX_CUR_EN, OUTPUT);
  pinMode(MUX_CUR_0, OUTPUT);
  pinMode(MUX_CUR_1, OUTPUT);
  pinMode(MUX_CUR_2, OUTPUT);
  pinMode(MUX_CUR_3, OUTPUT);

  pinMode(SWITCH_0, INPUT_PULLUP);
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(SD_DET, INPUT_PULLUP);
  pinMode(FAN_PWM, OUTPUT);

  // Initialize the ADS124S06
  ads.begin();
  setupRegisters();

  if (!bme_sensor.begin() || !bme_board.begin()) {
    Serial.println("Could not find both BME680 sensors, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme_sensor.setTemperatureOversampling(BME680_OS_8X);
  bme_sensor.setHumidityOversampling(BME680_OS_2X);
  bme_sensor.setPressureOversampling(BME680_OS_4X);
  bme_sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme_sensor.setGasHeater(320, 150);  // 320*C for 150 ms
  bme_board.setTemperatureOversampling(BME680_OS_8X);
  bme_board.setHumidityOversampling(BME680_OS_2X);
  bme_board.setPressureOversampling(BME680_OS_4X);
  bme_board.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme_board.setGasHeater(320, 150);  // 320*C for 150 ms

  // Start conversions
  ads.startConversions();
  mux_adc.disable();
  mux_adc.setChannel(0);
  mux_cur.disable();
  mux_cur.setChannel(0);
}

void loop() {
  uint8_t status[1];
  int32_t result;

  // Wait for data ready (DRDY) indication
  // Assuming a function or a way to wait until data is ready
  // Placeholder delay - replace with actual DRDY check if available
  delay(10);

  // Read the conversion result
  result = ads.readConvertedData(status, DIRECT);

  // Print the result
  Serial.print("ADC Result: ");
  Serial.println(result);

  // Wait before reading again
  delay(1000);
}
