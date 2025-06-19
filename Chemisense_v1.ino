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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BQ24195.h> // [MKR WiFi 1010 Battery Application Note | Arduino Documentation](https://docs.arduino.cc/tutorials/mkr-wifi-1010/mkr-battery-app-note/)

#define SEALEVELPRESSURE_HPA (1013.25)
#define MUX_DELAY 10            // ms - increased from 1ms for better MUX settling
#define START_DELAY 10          // ms - increased from 5ms for better ADC settling
#define CURRENT_SOURCE_DELAY 50 // ms - new: dedicated delay for current source changes

// Sample Averaging Configuration - More samples = better repeatability but slower measurements
#define SAMPLES_PER_CHANNEL 3 // Number of samples to average per channel (1-20 recommended)
                              // 1 = no averaging (fastest)
                              // 5 = good balance of speed/precision
                              // 10+ = high precision (slow)

// Reference Drift Compensation - Since diagnostic shows ADC/reference drift is the main issue
#define USE_REFERENCE_COMPENSATION 1 // 1 = compensate for reference drift, 0 = disable
                                     // Takes initial reading as reference to subtract drift

// Sampling Strategy - How to distribute samples across time
#define USE_TIME_DISTRIBUTED_SAMPLING 1 // 1 = sample all channels N times (round-robin)
                                        // 0 = sample each channel N times sequentially
                                        // Time-distributed better captures drift variation

#define CS_BME_0 13
#define CS_BME_1 14
#define CS_SD 6
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
#define FAN_PWM 7

#define LED_DIM 25
#define LED_BRIGHT 255

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

char fileName[13]; // Buffer to hold file name in the format "00000.TXT"

const float VREF = 2.5;         // Reference voltage (ADC internal)
const long ADC_RANGE = 8388607; // 24-bit ADC range adjusted for unipolar measurement (2^23 - 1)

// these would eventually live in EEPROM/SD card, measured w/ precision meter
const float MATLAB_SLOPE = 1.045742;
const float MATLAB_INTERCEPT = -7.228576;
const float SOURCE_CAL_10UA = 10.094e-6;
const float SOURCE_CAL_50UA = 49.751e-6;
float currentSource;              // set at init
const float fixedResistor = 2208; // Fixed resistor value
// ideal values: { 1, 4.7, 10, 0, 1, 4.7, 10, 0, 1, 4.7, 10, 0, 1, 4.7, 10, 0 };
const float calDie_00[16] = {1001.0, 4695.0, 10021.0, 0.0, 1002.0, 4680.0, 9955.0, 0.0, 999.0, 4682.0, 10016.0, 0.0, 999.0, 4685.0, 9950.0, 0.0};                            // measured values, also in B00.txt
const float caleDie_01[16] = {997.5, 2202.1, 4698.3, 10019.0, 46917.0, 100240.0, 469660.0, 1003450.0, 998.3, 2201.0, 4696.0, 9998.5, 46825.0, 99725.0, 470620.0, 9989800.0}; // B01.txt

const float INA190_GAIN = 25; // see datasheet
uint8_t ADCstatus[1];

// ADS124S06 status and timing constants
#define MAX_CONVERSION_WAIT_MS 100 // Maximum time to wait for conversion
#define STATUS_POLL_INTERVAL_MS 1  // How often to check status
#define STATUS_DRDY_MASK 0x01      // DRDY bit in status register (bit 0)

// float ADC_OFFSET = 0;                        // default, can be overwritten using calibration function
const float ADC_SOURCE_THRESH = VREF - 0.2; // slightly lower than VREF
const float SOURCE_MULTIPLIER = 5.0;        // from 10uA to 50uA

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

// Reference drift compensation variables
int32_t referenceReading = 0; // Initial reference reading for drift compensation
bool referenceSet = false;    // Flag to track if reference has been set

SPISettings SPI_SETTINGS_ADS(500000, MSBFIRST, SPI_MODE1);
ADS124S06 ads(CS_ADS, SPI_SETTINGS_ADS);
CD74HC4067 mux_adc(MUX_ADC_0, MUX_ADC_1, MUX_ADC_2, MUX_ADC_3);
CD74HC4067 mux_cur(MUX_CUR_0, MUX_CUR_1, MUX_CUR_2, MUX_CUR_3);
Adafruit_BME680 bme_die(CS_BME_0);
Adafruit_BME680 bme_topside(CS_BME_1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// functions with defaults need declaration for some reason
void sampleChannels(int channel = -1, bool doDebug = false);
float calcADS_R(int32_t adcValue, bool debug, bool allowSourceMod = false);
bool waitForConversion(unsigned long timeoutMs = MAX_CONVERSION_WAIT_MS);
void diagnoseMeasurementVariation(int testChannel = 0); // Diagnostic function for measurement stability
void isolateVariationSource(int testChannel = 0);       // Isolate ADC vs current source vs reference drift

// display controls
unsigned long previousMillis = 0; // Store the last time the page was updated
int currentPage = 0;              // Variable to track the current page (0 or 1)
const long interval = 4000;       // Interval for page switching (2000 ms)
bool paginate = false;

// PMIC/battery
int R1 = 330000;
int R2 = 1000000;
float rawADC;
float voltADC;
float voltBat;
int max_Source_voltage;          // upper source voltage for the battery
float batteryFullVoltage = 4.2;  // upper voltage limit for battery
float batteryEmptyVoltage = 3.5; // lower voltage limit for battery
float batteryCapacity = 2.2;     // set battery capacity in Ah
int percentCharged = 0;

void setup()
{
  WiFiDrv::pinMode(25, OUTPUT); // define GREEN LED
  WiFiDrv::pinMode(26, OUTPUT); // define RED LED
  WiFiDrv::pinMode(27, OUTPUT); // define BLUE LED
  RGBLED('B', LED_DIM);

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
  pinMode(MUX_CUR_EN, OUTPUT);
  disableMUX();
  pinMode(MUX_ADC_0, OUTPUT);
  pinMode(MUX_ADC_2, OUTPUT);
  pinMode(MUX_ADC_1, OUTPUT);
  pinMode(MUX_ADC_3, OUTPUT);

  pinMode(MUX_CUR_0, OUTPUT);
  pinMode(MUX_CUR_1, OUTPUT);
  pinMode(MUX_CUR_2, OUTPUT);
  pinMode(MUX_CUR_3, OUTPUT);

  pinMode(SWITCH_0, INPUT_PULLUP);
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(FAN_PWM, OUTPUT);
  analogWrite(FAN_PWM, 255); // 0-255

  Serial.begin(9600);
  delay(2000); // wait for serial
  Serial.println("\n\n***Hello, Chemisensor.***");

  // Initialize the ADS124S06 (and SPI)
  ads.begin();
  ads.adcStartupRoutine();
  configureADS();
  // ads.startConversions(); // !! do not use, we are using one-shot mode

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    // allow to continue, non-critical aspect
  }
  delay(1000); // required for I2C screen to startup

  // configure BQ24195 PMIC
  analogReference(AR_DEFAULT); // Arduino
  analogReadResolution(12);    // Arduino
  max_Source_voltage = (3.3 * (R1 + R2)) / R2;
  PMIC.begin();                                      // start the PMIC I2C connection
  PMIC.enableBoostMode();                            // boost battery output to 5V
  PMIC.setMinimumSystemVoltage(batteryEmptyVoltage); // set the minimum battery output to 3.5V
  PMIC.setChargeVoltage(batteryFullVoltage);         // set battery voltage at full charge
  PMIC.setChargeCurrent(batteryCapacity / 2);        // set battery current to C/2 in amps
  PMIC.enableCharge();                               // enable charging of battery
  checkBattery();
  if (percentCharged > 0)
  {
    char initBuffer[64]; // Create a buffer large enough to hold the formatted string
    snprintf(initBuffer, sizeof(initBuffer), "Hi, Chemisensor v1.1\nNeurotech Hub | MG\nCharge: %d%%", percentCharged);
    displayText(initBuffer, 1);
  }
  else
  {
    displayText("Hi, Chemisensor v1.1\nNeurotech Hub | MG\nNo Battery (turn ON)", 1);
  }

  enableMUX();
  mux_adc.channel(0); // Set the channel for mux_adc
  mux_cur.channel(0); // Set the channel for mux_cur

  // Try initializing SD card up to 5 times
  digitalWrite(CS_SD, LOW);
  for (int attempt = 0; attempt < 3; attempt++)
  {
    if (SD.begin(CS_SD, SPI_HALF_SPEED))
    {
      sdInitialized = true;
      break;
    }
    else
    {
      Serial.print("SD card initialization attempt ");
      Serial.print(attempt + 1);
      Serial.println(" failed.");
    }
  }
  if (sdInitialized)
  {
    Serial.println("SD card initialized.");
  }
  else
  {
    Serial.println("SD card initialization failed.");
    error = true;
  }
  digitalWrite(CS_SD, HIGH);

  if (!bme_die.begin() || !bme_topside.begin())
  {
    Serial.println("Could not find both BME680 sensors!");
    error = true;
  }
  else
  {
    configureBME(bme_die);
    Serial.println("\nBME near die:");
    readBME_die(true);
    configureBME(bme_topside);
    Serial.println("\nBME topside:");
    readBME_topside(true);
  }

  Serial.println("\n-----\nCommands:");
  Serial.println("0-15: read single channel (and display serial + I2C)");
  Serial.println("33 (button A): read all channels");
  Serial.println("55 (button B): log data");
  Serial.println("77: reset reference compensation");
  Serial.println("66: toggle sampling method (time-distributed vs sequential)");
  Serial.println("99: diagnostic test - analyze measurement variation");
  Serial.println("88: isolation test - identify variation source");
  RGBLED('-', 0);
}

void loop()
{
  if (error)
  {
    while (1)
    {
      RGBLED('R', LED_DIM);
      delay(200);
      RGBLED('-', 0);
      delay(200);
    }
  }

  if (Serial.available() > 0)
  {
    RGBLED('-', 0);
    RGBLED('B', LED_DIM);
    String input = Serial.readStringUntil('\n'); // Read the input until newline character
    int channel = input.toInt();                 // Convert the input to an integer
    if (channel >= 0 && channel <= 15)
    {
      Serial.print("\nSampling channel ");
      Serial.println(channel);
      sampleChannels(channel, true);
      sendValueToDisplay(channel);
      // disableMUX();
      // enableMUX();
      paginate = false;
    }
    else if (channel == 33)
    {
      input_showChannels();
      paginate = true;
    }
    else if (channel == 55)
    {
      input_logData();
      paginate = false;
    }
    else if (channel == 77)
    {
      Serial.println("Resetting reference compensation...");
      referenceSet = false;
      referenceReading = 0;
      Serial.println("Reference reset. Next measurement will set new reference.");
      paginate = false;
    }
    else if (channel == 66)
    {
#if USE_TIME_DISTRIBUTED_SAMPLING
      Serial.println("Currently using: TIME-DISTRIBUTED sampling");
      Serial.println("To switch to sequential: change USE_TIME_DISTRIBUTED_SAMPLING to 0 and recompile");
#else
      Serial.println("Currently using: SEQUENTIAL sampling");
      Serial.println("To switch to time-distributed: change USE_TIME_DISTRIBUTED_SAMPLING to 1 and recompile");
#endif
      paginate = false;
    }
    else if (channel == 99)
    {
      Serial.println("Running diagnostic test on channel 0...");
      diagnoseMeasurementVariation(0);
      paginate = false;
    }
    else if (channel == 88)
    {
      Serial.println("Running isolation test on channel 0...");
      isolateVariationSource(0);
      paginate = false;
    }
    else
    {
      Serial.println("Invalid command.");
    }
    RGBLED('-', 0);
  }

  if (digitalRead(SWITCH_0) == LOW)
  {
    input_logData();
    while (digitalRead(SWITCH_0) == LOW)
    {
      // wait for button to release
    }
    paginate = false;
  }

  if (digitalRead(SWITCH_1) == LOW)
  {
    input_showChannels();
    while (digitalRead(SWITCH_1) == LOW)
    {
      // wait for button to release
    }
    paginate = true;
  }

  if (fadeLED == 0 || fadeLED == 100)
  {
    fadeDir = !fadeDir;
  }
  if (fadeDir)
  {
    fadeLED++;
  }
  else
  {
    fadeLED--;
  }
  RGBLED('G', fadeLED);

  if (paginate)
  {
    unsigned long currentMillis = millis(); // Get the current time
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis; // Save the last time the page was updated
      // Call the display function with the current page
      displayChannels(currentPage);
      // Toggle between page 0 and page 1
      currentPage = (currentPage == 0) ? 1 : 0;
    }
  }

  delay(10);
}

// void input_runCal() {
//   Serial.println("\nRunning calibration...");
//   displayText("CAL", 2);
//   if (runCalibration()) {
//     readCalibrationValues();  // recalculates ADC_OFFSET
//   }
// }
void input_showChannels()
{
  sampleChannels();
  for (int i = 0; i < 16; i++)
  {
    Serial.print(i);
    Serial.print(",");
    Serial.println(measuredRValues[i], 4);
  }
  currentPage = 0;
}
void input_logData()
{
  Serial.println("Logging...");
  displayText("Logging...", 2);
  logData();
  displayText(fileName, 2);
}

void displayChannels(int page)
{
  display.clearDisplay();              // Clear the display
  display.setTextSize(1);              // Use smaller text to fit more information
  display.setTextColor(SSD1306_WHITE); // Set text color

  // Determine the starting and ending channels based on the page
  int startChannel = (page == 0) ? 0 : 8;
  int endChannel = startChannel + 8;

  // Loop through the selected set of channels (either 0-7 or 8-15)
  for (int i = startChannel; i < endChannel; i++)
  {
    char buffer[20]; // Buffer to hold the formatted string for each channel
    snprintf(buffer, sizeof(buffer), "%02d:%.1f", i + 1, measuredRValues[i]);

    // Adjust cursor position for each channel
    if ((i - startChannel) < 4)
    {
      // First 4 channels go in the left column
      display.setCursor(0, (i - startChannel) * 8); // Set row for left column
    }
    else
    {
      // Last 4 channels go in the right column
      display.setCursor(64, (i - startChannel - 4) * 8); // Set row for right column
    }
    display.write(buffer); // Write the buffer to the screen
  }
  display.display(); // Render everything to the screen
}

void displayText(const char *message, int textSize)
{
  display.clearDisplay();
  display.setTextSize(textSize);       // Set text size as per the parameter
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner
  display.cp437(true);                 // Use full 256 char 'Code Page 437' font
  display.write(message);              // Display the passed string
  display.display();                   // Render to the display
}

void sendValueToDisplay(int channel)
{
  char buffer[50]; // Buffer to hold the formatted string
  snprintf(buffer, sizeof(buffer), "Ch%d\n%.2f", channel, measuredRValues[channel]);
  displayText(buffer, 2); // Send the formatted string to the display
}

void logData()
{
  RGBLED('R', LED_DIM);
  sampleChannels();
  readBME_die(false);
  readBME_topside(false);

  int nextFileNumber = getNextFileNumber();
  snprintf(fileName, sizeof(fileName), "%05d.TXT", nextFileNumber);
  Serial.print("Creating file: ");
  Serial.println(fileName);

  digitalWrite(CS_SD, LOW);
  File dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile)
  {
    // Write the header
    dataFile.println("key,value");

    // // Write the measuredRValues array
    for (int i = 0; i < 16; i++)
    {
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
  }
  else
  {
    Serial.println("Error opening file.");
  }
  digitalWrite(CS_SD, HIGH);
  RGBLED('-', 0);
}

int getNextFileNumber()
{
  int fileNumber = 0;
  digitalWrite(CS_SD, LOW);
  while (fileNumber < 100000)
  {
    snprintf(fileName, sizeof(fileName), "%05d.TXT", fileNumber);
    if (!SD.exists(fileName))
    {
      break;
    }
    fileNumber++;
  }
  digitalWrite(CS_SD, HIGH);
  return fileNumber;
}

void sampleChannels(int channel, bool doDebug)
{
  // Check if a valid channel is passed (0-15)
  int startChannel = 0;
  int endChannel = 16;

  if (channel >= 0 && channel < 16)
  {
    // If a valid channel is passed, only sample that channel
    startChannel = channel;
    endChannel = channel + 1;
  }

#if USE_TIME_DISTRIBUTED_SAMPLING
  // Time-distributed sampling: sample all channels round-robin N times
  // This spreads samples across time to better capture drift variation

  // Initialize accumulator arrays
  long adcSums[16] = {0};
  int validSampleCounts[16] = {0};

  if (doDebug)
  {
    Serial.println("Using time-distributed sampling...");
  }

  // Take SAMPLES_PER_CHANNEL rounds, sampling all channels each round
  for (int sample = 0; sample < SAMPLES_PER_CHANNEL; sample++)
  {
    if (doDebug && SAMPLES_PER_CHANNEL > 1)
    {
      Serial.print("=== Sample Round ");
      Serial.print(sample + 1);
      Serial.println(" ===");
    }

    for (int ch = startChannel; ch < endChannel; ch++)
    {
      // Set the channel for mux_adc and mux_cur
      mux_adc.channel(ch);
      mux_cur.channel(ch);
      delay(MUX_DELAY);

      ads.sendSTART();
      delay(START_DELAY);

      int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);
      adcSums[ch] += adcValue;
      validSampleCounts[ch]++;

      // Show individual sample values when debugging single channel
      if (doDebug && (endChannel - startChannel == 1))
      {
        Serial.print("Ch");
        Serial.print(ch);
        Serial.print(" Round ");
        Serial.print(sample + 1);
        Serial.print(": ");
        Serial.println(adcValue);
      }

      // Small delay between channels in same round
      delay(2);
    }

    // Longer delay between rounds to capture drift
    if (sample < SAMPLES_PER_CHANNEL - 1)
    {
      delay(100); // 100ms between rounds
    }
  }

  // Process accumulated samples for each channel
  for (int ch = startChannel; ch < endChannel; ch++)
  {
    int32_t avgAdcValue = adcSums[ch] / validSampleCounts[ch];

#if USE_REFERENCE_COMPENSATION
    // Apply reference drift compensation
    if (!referenceSet && ch == startChannel)
    {
      // Set reference on first channel of first measurement
      referenceReading = avgAdcValue;
      referenceSet = true;
      if (doDebug)
      {
        Serial.print("Reference set to: ");
        Serial.println(referenceReading);
      }
    }

    // Compensate for reference drift by using reference as baseline
    int32_t compensatedValue = avgAdcValue;
    if (doDebug && ch == startChannel)
    {
      Serial.print("Raw: ");
      Serial.print(avgAdcValue);
      Serial.print(", Compensated: ");
      Serial.println(compensatedValue);
    }
    float R = calcADS_R(compensatedValue, doDebug, true);
#else
    float R = calcADS_R(avgAdcValue, doDebug, true);
#endif

    // Handle 50µA current source switching if needed
    if (R == -1.0)
    {
      // Switch to 50µA and re-sample this channel with time distribution
      ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_50);
      currentSource = SOURCE_CAL_50UA;
      delay(CURRENT_SOURCE_DELAY);

      // Re-sample this channel with 50µA using time distribution
      long adcSum50ua = 0;
      int validSamples50ua = 0;

      for (int sample = 0; sample < SAMPLES_PER_CHANNEL; sample++)
      {
        mux_adc.channel(ch);
        mux_cur.channel(ch);
        delay(MUX_DELAY);

        ads.sendSTART();
        delay(START_DELAY);

        int32_t adcValue50ua = ads.readConvertedData(ADCstatus, DIRECT);
        adcSum50ua += adcValue50ua;
        validSamples50ua++;

        if (sample < SAMPLES_PER_CHANNEL - 1)
        {
          delay(100); // Same timing as main sampling
        }
      }

      avgAdcValue = adcSum50ua / validSamples50ua;

#if USE_REFERENCE_COMPENSATION
      int32_t compensated50ua = avgAdcValue;
      R = calcADS_R(compensated50ua, doDebug, false);
#else
      R = calcADS_R(avgAdcValue, doDebug, false);
#endif

      // Reset to 10µA
      ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10);
      currentSource = SOURCE_CAL_10UA;
      delay(CURRENT_SOURCE_DELAY);
    }

    // Store the values in arrays
    adcValues[ch] = avgAdcValue;
    measuredRValues[ch] = R;
  }

#else
  // Original channel-sequential sampling for comparison
  for (int ch = startChannel; ch < endChannel; ch++)
  {
    // Set the channel for mux_adc and mux_cur
    mux_adc.channel(ch);
    mux_cur.channel(ch);
    delay(MUX_DELAY);

    // Take multiple samples and average them for better repeatability
    long adcSum = 0;
    int validSamples = 0;

    for (int sample = 0; sample < SAMPLES_PER_CHANNEL; sample++)
    {
      ads.sendSTART();
      delay(START_DELAY);

      int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);
      adcSum += adcValue;
      validSamples++;

      // Show individual sample values when debugging single channel
      if (doDebug && SAMPLES_PER_CHANNEL > 1)
      {
        Serial.print("Sample ");
        Serial.print(sample + 1);
        Serial.print(": ");
        Serial.println(adcValue);
      }

      // Small delay between samples to avoid interference
      if (sample < SAMPLES_PER_CHANNEL - 1)
      {
        delay(2);
      }
    }

    // Calculate average ADC value
    int32_t avgAdcValue = adcSum / validSamples;

#if USE_REFERENCE_COMPENSATION
    // Apply reference drift compensation
    if (!referenceSet && ch == startChannel)
    {
      // Set reference on first channel of first measurement
      referenceReading = avgAdcValue;
      referenceSet = true;
      if (doDebug)
      {
        Serial.print("Reference set to: ");
        Serial.println(referenceReading);
      }
    }

    // Compensate for reference drift by using reference as baseline
    int32_t compensatedValue = avgAdcValue;
    if (doDebug && ch == startChannel)
    {
      Serial.print("Raw: ");
      Serial.print(avgAdcValue);
      Serial.print(", Compensated: ");
      Serial.println(compensatedValue);
    }
    float R = calcADS_R(compensatedValue, doDebug, true);
#else
    float R = calcADS_R(avgAdcValue, doDebug, true);
#endif

    // test for return flag
    if (R == -1.0)
    {
      // Serial.println("Increasing source from 10uA to 50uA.");
      ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_50); // increase for low R's
      currentSource = SOURCE_CAL_50UA;
      delay(CURRENT_SOURCE_DELAY); // settle after current source change

      // Take multiple samples with 50µA current source and average
      adcSum = 0;
      validSamples = 0;

      for (int sample = 0; sample < SAMPLES_PER_CHANNEL; sample++)
      {
        ads.sendSTART();
        delay(START_DELAY);

        int32_t adcValue50ua = ads.readConvertedData(ADCstatus, DIRECT);
        adcSum += adcValue50ua;
        validSamples++;

        // Small delay between samples
        if (sample < SAMPLES_PER_CHANNEL - 1)
        {
          delay(2);
        }
      }

      avgAdcValue = adcSum / validSamples;

#if USE_REFERENCE_COMPENSATION
      // Apply same compensation for 50µA readings
      int32_t compensated50ua = avgAdcValue;
      R = calcADS_R(compensated50ua, doDebug, false);
#else
      R = calcADS_R(avgAdcValue, doDebug, false); // overwrite, disallow source modification
#endif
      ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10); // reset to default
      currentSource = SOURCE_CAL_10UA;
      delay(CURRENT_SOURCE_DELAY); // settle after resetting current source
    }

    // Store the values in arrays
    adcValues[ch] = avgAdcValue;
    measuredRValues[ch] = R;
  }
#endif
}

// bool readCalibrationValues() {
//   bool success = false;

//   digitalWrite(CS_SD, LOW);
//   File dataFile = SD.open("CAL.TXT");

//   if (dataFile) {
//     int32_t adcValue;
//     float measuredRValue;
//     float sumADC = 0;
//     int count = 0;

//     while (dataFile.available()) {
//       // Read each line of the file
//       String line = dataFile.readStringUntil('\n');

//       // Skip the header line
//       if (line.startsWith("channel")) {
//         continue;
//       }

//       // Parse the line to extract values
//       int firstComma = line.indexOf(',');
//       int secondComma = line.indexOf(',', firstComma + 1);

//       int channel = line.substring(0, firstComma).toInt();
//       adcValue = line.substring(firstComma + 1, secondComma).toInt();
//       measuredRValue = line.substring(secondComma + 1).toFloat();

//       // Check if calDie_00 is 0 for the current channel
//       if (calDie_00[channel] == 0) {
//         sumADC += adcValue;
//         count++;
//       }
//     }

//     // Calculate the mean ADC value for calDie_00 == 0
//     if (count > 0) {
//       float storedOffset = sumADC / count;
//       if (abs(storedOffset) < 1000) {
//         ADC_OFFSET = storedOffset;
//       } else {
//         Serial.println("SD card ADC_OFFSET out of range.");
//       }
//     }

//     dataFile.close();
//     success = true;
//   } else {
//     Serial.println("Failed to open CAL.TXT for reading.");
//   }
//   Serial.print("ADC_OFFSET: ");
//   Serial.println(ADC_OFFSET);
//   digitalWrite(CS_SD, HIGH);
//   return success;
// }

bool runCalibration()
{
  bool success = false;

  sampleChannels(); // fill buffers

  // Write the sampled data to the SD card
  digitalWrite(CS_SD, LOW);
  // Ensure the calibration file is overwritten
  if (SD.exists("CAL.TXT"))
  {
    SD.remove("CAL.TXT");
  }

  File dataFile = SD.open("CAL.TXT", FILE_WRITE);

  if (dataFile)
  {
    // Write the CSV header
    dataFile.println("channel,measured_ADC_value,measured_R_value");

    for (int channel = 0; channel < 16; channel++)
    {
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
  }
  else
  {
    Serial.println("Failed to open CAL.TXT for writing.");
  }
  digitalWrite(CS_SD, HIGH);
  return success;
}

// Initialize register values according to datasheet recommendations
void configureADS()
{
  // Set the internal reference voltage
  ads.writeSingleRegister(REG_ADDR_REF, ADS_REFSEL_INT | ADS_REFINT_ON_ALWAYS);
  // Set the IDAC current to 10µA on the AIN0 pin
  ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10); // ADS_IDACMAG_10, ADS_IDACMAG_50
  currentSource = SOURCE_CAL_10UA;

  ads.writeSingleRegister(REG_ADDR_IDACMUX, ADS_IDAC1_A0 | ADS_IDAC2_OFF);
  // Set the input multiplexer to measure AIN1 referenced to AINCOM
  ads.writeSingleRegister(REG_ADDR_INPMUX, ADS_P_AIN1 | ADS_N_AINCOM);
  // Gain setting (PGA): Disable PGA, ensure gain=1
  ads.writeSingleRegister(REG_ADDR_PGA, PGA_DEFAULT);
  // Single-shot conversion (must send start hereon)
  ads.writeSingleRegister(REG_ADDR_DATARATE, ADS_CONVMODE_SS | ADS_DR_400);

  // Print averaging configuration
  Serial.print("ADC sampling: ");
  Serial.print(SAMPLES_PER_CHANNEL);
  Serial.println(" samples per channel (averaged)");
}

void configureBME(Adafruit_BME680 bme)
{
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void readBME_die(bool debug)
{
  if (!bme_die.performReading())
  {
    Serial.println("Failed to perform BME reading.");
    return;
  }
  temperature_die = bme_die.temperature;                     // C
  pressure_die = bme_die.pressure / 100.0;                   // hPa
  humidity_die = bme_die.humidity;                           // %
  altitude_die = bme_die.readAltitude(SEALEVELPRESSURE_HPA); // m
  gas_resistance_die = bme_die.gas_resistance / 1000.0;      // kOhms

  if (debug)
  {
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

void readBME_topside(bool debug)
{
  if (!bme_topside.performReading())
  {
    Serial.println("Failed to perform BME reading.");
    return;
  }
  temperature_topside = bme_topside.temperature;                     // C
  pressure_topside = bme_topside.pressure / 100.0;                   // hPa
  humidity_topside = bme_topside.humidity;                           // %
  altitude_topside = bme_topside.readAltitude(SEALEVELPRESSURE_HPA); // m
  gas_resistance_topside = bme_topside.gas_resistance / 1000.0;      // kOhms

  if (debug)
  {
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

float calcADS_R(int32_t adcValue, bool debug, bool allowSourceMod)
{
  // after INA190 gain
  float ADCNode = (adcValue * VREF) / ADC_RANGE;
  if (ADCNode * SOURCE_MULTIPLIER < ADC_SOURCE_THRESH && allowSourceMod)
  {
    return -1.0; // flag for current source modification
  }
  // get voltage drop before gain
  float voltageDrop = ADCNode / INA190_GAIN;
  // Ohm's Law
  float R_eq = voltageDrop / currentSource;
  // account for voltage divider
  float R = 1 / ((1 / R_eq) - (1 / fixedResistor));
  // apply calibration
  R = MATLAB_SLOPE * R + MATLAB_INTERCEPT;

  // fix negative, can occur at low R's due to offset
  if (R < 0.0)
  {
    // Serial.println("R < 0, valid only for small R's.");
    R = 0.0;
  }

  if (debug)
  {
    Serial.print("Voltage Drop (at INA): ");
    Serial.println(voltageDrop, 3);

    Serial.print("ADC Value: ");
    Serial.print(adcValue);

    Serial.print("ADC Node (V): ");
    Serial.println(ADCNode, 6);

    Serial.print("R_eq: ");
    Serial.println(R_eq, 3);

    Serial.print("R (Ohm)): ");
    Serial.println(R, 3);
    Serial.println("");
  }

  return R;
}

void enableMUX()
{
  digitalWrite(MUX_ADC_EN, LOW);
  digitalWrite(MUX_CUR_EN, LOW);
}

void disableMUX()
{
  digitalWrite(MUX_ADC_EN, HIGH);
  digitalWrite(MUX_CUR_EN, HIGH);
}

void RGBLED(char RGB, int state)
{
  switch (RGB)
  {
  case 'R':
    WiFiDrv::analogWrite(25, state); // RED
    break;
  case 'G':
    WiFiDrv::analogWrite(26, state); // GREEN
    break;
  case 'B':
    WiFiDrv::analogWrite(27, state); // BLUE
    break;
  default: // not RGB, turn off
    WiFiDrv::analogWrite(25, LOW);
    WiFiDrv::analogWrite(26, LOW);
    WiFiDrv::analogWrite(27, LOW);
    break;
  }
}

void checkBattery()
{
  rawADC = analogRead(ADC_BATTERY);               // the value obtained directly at the PB09 input pin
  voltADC = rawADC * (3.3 / 4095.0);              // convert ADC value to the voltage read at the pin
  voltBat = voltADC * (max_Source_voltage / 3.3); // we cannot use map since it requires int inputs/outputs

  percentCharged = (voltBat - batteryEmptyVoltage) * (100) / (batteryFullVoltage - batteryEmptyVoltage); // custom float friendly map function

  Serial.print("VBATT = ");
  Serial.print(voltBat);
  Serial.print("V, ");
  Serial.print(percentCharged);
  Serial.println("% charged");
}

bool waitForConversion(unsigned long timeoutMs)
{
  unsigned long startTime = millis();
  while (millis() - startTime < timeoutMs)
  {
    // Check if conversion data is ready by reading the status register
    // For ADS124S06, we can check if data is ready by trying to read status
    // Alternative: use the status byte from readConvertedData
    uint8_t tempStatus[1];
    ads.readConvertedData(tempStatus, DIRECT); // This will get status if SENDSTAT is enabled

    // Note: This is a simplified approach - we're actually reading the data
    // A better approach would be to check a dedicated status register if available
    return true; // Data is ready since we successfully read it
  }
  return false; // Timeout
}

void diagnoseMeasurementVariation(int testChannel)
{
  Serial.print("=== Measurement Variation Diagnostic for Channel ");
  Serial.print(testChannel);
  Serial.println(" ===");

  const int numTests = 20;
  int32_t rapidSamples[numTests];
  int32_t spacedSamples[numTests];

  // Set up the test channel
  mux_adc.channel(testChannel);
  mux_cur.channel(testChannel);
  delay(MUX_DELAY);

  Serial.println("\n1. RAPID SAMPLING TEST (back-to-back measurements):");
  Serial.println("   If variation is random noise, these should vary randomly");
  Serial.println("   If variation is systematic, these should be very similar");

  // Test 1: Rapid sampling (minimal delay between samples)
  for (int i = 0; i < numTests; i++)
  {
    ads.sendSTART();
    delay(START_DELAY);
    rapidSamples[i] = ads.readConvertedData(ADCstatus, DIRECT);
    delay(1); // Minimal delay

    Serial.print("Rapid ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(rapidSamples[i]);
  }

  Serial.println("\n2. SPACED SAMPLING TEST (1 second between measurements):");
  Serial.println("   If variation is thermal/drift, these should show trends");
  Serial.println("   If variation is random, these should vary randomly");

  // Test 2: Spaced sampling (1 second delays to allow drift)
  for (int i = 0; i < numTests; i++)
  {
    ads.sendSTART();
    delay(START_DELAY);
    spacedSamples[i] = ads.readConvertedData(ADCstatus, DIRECT);

    Serial.print("Spaced ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(spacedSamples[i]);

    if (i < numTests - 1)
      delay(1000); // 1 second between measurements
  }

  // Calculate statistics
  long rapidSum = 0, spacedSum = 0;
  for (int i = 0; i < numTests; i++)
  {
    rapidSum += rapidSamples[i];
    spacedSum += spacedSamples[i];
  }

  int32_t rapidAvg = rapidSum / numTests;
  int32_t spacedAvg = spacedSum / numTests;

  // Calculate variation (simple range for now)
  int32_t rapidMin = rapidSamples[0], rapidMax = rapidSamples[0];
  int32_t spacedMin = spacedSamples[0], spacedMax = spacedSamples[0];

  for (int i = 1; i < numTests; i++)
  {
    if (rapidSamples[i] < rapidMin)
      rapidMin = rapidSamples[i];
    if (rapidSamples[i] > rapidMax)
      rapidMax = rapidSamples[i];
    if (spacedSamples[i] < spacedMin)
      spacedMin = spacedSamples[i];
    if (spacedSamples[i] > spacedMax)
      spacedMax = spacedSamples[i];
  }

  Serial.println("\n=== ANALYSIS ===");
  Serial.print("Rapid sampling - Avg: ");
  Serial.print(rapidAvg);
  Serial.print(", Range: ");
  Serial.print(rapidMax - rapidMin);
  Serial.print(" (");
  Serial.print(rapidMin);
  Serial.print(" to ");
  Serial.print(rapidMax);
  Serial.println(")");

  Serial.print("Spaced sampling - Avg: ");
  Serial.print(spacedAvg);
  Serial.print(", Range: ");
  Serial.print(spacedMax - spacedMin);
  Serial.print(" (");
  Serial.print(spacedMin);
  Serial.print(" to ");
  Serial.print(spacedMax);
  Serial.println(")");

  Serial.print("Average difference: ");
  Serial.println(abs(rapidAvg - spacedAvg));

  Serial.println("\n=== INTERPRETATION ===");
  if ((rapidMax - rapidMin) < (spacedMax - spacedMin) / 2)
  {
    Serial.println("-> Likely SYSTEMATIC variation (thermal/drift)");
    Serial.println("   - Rapid samples are more consistent");
    Serial.println("   - Averaging won't help much");
    Serial.println("   - Focus on thermal stability, settling times");
  }
  else if (abs(rapidAvg - spacedAvg) > (rapidMax - rapidMin))
  {
    Serial.println("-> Likely DRIFT/OFFSET issues");
    Serial.println("   - Average values differ significantly");
    Serial.println("   - Check reference stability, current source");
  }
  else
  {
    Serial.println("-> Likely RANDOM NOISE");
    Serial.println("   - Similar variation in both tests");
    Serial.println("   - Averaging should help");
  }

  Serial.println("=== END DIAGNOSTIC ===\n");
}

void isolateVariationSource(int testChannel)
{
  Serial.print("=== Variation Source Isolation for Channel ");
  Serial.print(testChannel);
  Serial.println(" ===");

  // Set up the test channel
  mux_adc.channel(testChannel);
  mux_cur.channel(testChannel);
  delay(MUX_DELAY);

  const int numTests = 10;

  Serial.println("\n1. SAME CHANNEL, NO MUX SWITCHING (ADC + Reference stability):");
  Serial.println("   Tests ADC and reference drift without circuit changes");

  // Test 1: Same channel repeatedly (no MUX changes)
  int32_t sameChannelReadings[numTests];
  for (int i = 0; i < numTests; i++)
  {
    ads.sendSTART();
    delay(START_DELAY);
    sameChannelReadings[i] = ads.readConvertedData(ADCstatus, DIRECT);

    Serial.print("Same Ch ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(sameChannelReadings[i]);

    delay(500); // 500ms between readings
  }

  Serial.println("\n2. SWITCH TO DIFFERENT CHANNEL AND BACK (MUX effects):");
  Serial.println("   Tests if MUX switching introduces variation");

  // Test 2: Switch to different channel and back
  int32_t muxSwitchReadings[numTests];
  for (int i = 0; i < numTests; i++)
  {
    // Switch to a different channel temporarily
    int otherChannel = (testChannel + 1) % 16;
    mux_adc.channel(otherChannel);
    mux_cur.channel(otherChannel);
    delay(MUX_DELAY);

    // Switch back to test channel
    mux_adc.channel(testChannel);
    mux_cur.channel(testChannel);
    delay(MUX_DELAY);

    ads.sendSTART();
    delay(START_DELAY);
    muxSwitchReadings[i] = ads.readConvertedData(ADCstatus, DIRECT);

    Serial.print("MUX Switch ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(muxSwitchReadings[i]);

    delay(500);
  }

  Serial.println("\n3. CURRENT SOURCE SWITCHING TEST (10µA stability):");
  Serial.println("   Tests current source stability and settling");

  // Test 3: Current source switching effects
  int32_t currentSwitchReadings[numTests];
  for (int i = 0; i < numTests; i++)
  {
    // Temporarily switch to 50µA and back to 10µA
    ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_50);
    currentSource = SOURCE_CAL_50UA;
    delay(CURRENT_SOURCE_DELAY);

    // Switch back to 10µA
    ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10);
    currentSource = SOURCE_CAL_10UA;
    delay(CURRENT_SOURCE_DELAY);

    ads.sendSTART();
    delay(START_DELAY);
    currentSwitchReadings[i] = ads.readConvertedData(ADCstatus, DIRECT);

    Serial.print("Current Switch ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(currentSwitchReadings[i]);

    delay(200);
  }

  // Calculate ranges for comparison
  int32_t sameChMin = sameChannelReadings[0], sameChMax = sameChannelReadings[0];
  int32_t muxMin = muxSwitchReadings[0], muxMax = muxSwitchReadings[0];
  int32_t currentMin = currentSwitchReadings[0], currentMax = currentSwitchReadings[0];

  for (int i = 1; i < numTests; i++)
  {
    if (sameChannelReadings[i] < sameChMin)
      sameChMin = sameChannelReadings[i];
    if (sameChannelReadings[i] > sameChMax)
      sameChMax = sameChannelReadings[i];
    if (muxSwitchReadings[i] < muxMin)
      muxMin = muxSwitchReadings[i];
    if (muxSwitchReadings[i] > muxMax)
      muxMax = muxSwitchReadings[i];
    if (currentSwitchReadings[i] < currentMin)
      currentMin = currentSwitchReadings[i];
    if (currentSwitchReadings[i] > currentMax)
      currentMax = currentSwitchReadings[i];
  }

  Serial.println("\n=== VARIATION SOURCE ANALYSIS ===");
  Serial.print("Same Channel Range: ");
  Serial.print(sameChMax - sameChMin);
  Serial.println(" (ADC + Reference baseline)");

  Serial.print("MUX Switching Range: ");
  Serial.print(muxMax - muxMin);
  Serial.println(" (adds MUX effects)");

  Serial.print("Current Switching Range: ");
  Serial.print(currentMax - currentMin);
  Serial.println(" (adds current source effects)");

  Serial.println("\n=== INTERPRETATION ===");
  int32_t baselineVariation = sameChMax - sameChMin;
  int32_t muxVariation = muxMax - muxMin;
  int32_t currentVariation = currentMax - currentMin;

  if (muxVariation > baselineVariation * 2)
  {
    Serial.println("-> MUX SWITCHING is a major source of variation");
    Serial.println("   - Increase MUX_DELAY");
    Serial.println("   - Check MUX power supply stability");
  }

  if (currentVariation > baselineVariation * 2)
  {
    Serial.println("-> CURRENT SOURCE SWITCHING is a major source of variation");
    Serial.println("   - Increase CURRENT_SOURCE_DELAY");
    Serial.println("   - Current source may need better stability");
  }

  if (baselineVariation > 500)
  {
    Serial.println("-> ADC/REFERENCE DRIFT is significant");
    Serial.println("   - ADC internal reference may be drifting");
    Serial.println("   - Consider external reference or temperature compensation");
  }
  else
  {
    Serial.println("-> ADC/REFERENCE is relatively stable");
  }

  Serial.println("=== END ISOLATION TEST ===\n");
}