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

// Command definitions
#define MAX_CMD_LENGTH 64
#define MAX_CHANNELS 16
#define MAX_TOKENS 10

// Command strings
const char *CMD_INIT = "init";     // Initialize sampling configuration
const char *CMD_START = "start";   // Start continuous sampling
const char *CMD_STOP = "stop";     // Stop continuous sampling
const char *CMD_SAMPLE = "sample"; // Sample specific channel(s)
const char *CMD_SHOW = "show";     // Show all channel values
const char *CMD_LOG = "log";       // Log data to SD card
const char *CMD_RESET = "reset";   // Reset reference compensation
const char *CMD_DIAG = "diag";     // Run diagnostic test
const char *CMD_TEST = "test";     // Run isolation test
const char *CMD_HELP = "help";     // Show command help

// Sampling configuration structure
struct SamplingConfig
{
  uint8_t numSamples;                   // Samples to average per channel (1-255)
  uint32_t rateLimit;                   // Minimum ms between outputs (0 = fast as possible)
  bool isRunning;                       // Sampling state
  uint8_t activeChannels[MAX_CHANNELS]; // 1 = active, 0 = inactive
  uint8_t numActiveChannels;            // Count of active channels
};

// Global sampling configuration
SamplingConfig samplingConfig = {
    .numSamples = 1,       // Default to 10 samples
    .rateLimit = 0,        // Default to fast as possible
    .isRunning = false,    // Start stopped
    .activeChannels = {0}, // All channels inactive
    .numActiveChannels = 0};

// Debug timing flag
bool debugTiming = false; // Hard-coded for timing analysis

// No legacy commands needed - using new text commands only

#define SEALEVELPRESSURE_HPA (1013.25)
#define MUX_DELAY 25            // ms - reduced from 50ms to optimize speed
#define START_DELAY 8           // ms - reduced from 15ms to optimize speed
#define CURRENT_SOURCE_DELAY 85 // ms - optimized based on consistent performance

// Sampling Configuration
#define ROUND_DELAY_MS 10  // ms - reduced from 25ms to minimize drift effects between rounds
#define CHANNEL_DELAY_MS 2 // ms - keeping this minimal delay between channels

// Reference compensation is critical given observed drift
#define REFERENCE_CHECK_INTERVAL 5000 // ms - how often to update reference
#define REFERENCE_CHANNEL 0           // use channel 0 (known resistor) as reference

// Reference and sampling configuration
// Using time-distributed sampling with reference drift compensation for optimal stability

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

// Command parsing functions
void parseChannels(const char *channelStr)
{
  // Reset active channels
  memset(samplingConfig.activeChannels, 0, MAX_CHANNELS);
  samplingConfig.numActiveChannels = 0;

  // Handle 'all' keyword
  if (strncmp(channelStr, "all", 3) == 0)
  {
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
      samplingConfig.activeChannels[i] = 1;
    }
    samplingConfig.numActiveChannels = MAX_CHANNELS;
    return;
  }

  // Parse comma-separated list
  char *str = strdup(channelStr); // Make a copy since strtok modifies string
  char *token = strtok(str, ",");

  while (token != NULL && samplingConfig.numActiveChannels < MAX_CHANNELS)
  {
    int channel = atoi(token);
    if (channel >= 0 && channel < MAX_CHANNELS)
    {
      samplingConfig.activeChannels[channel] = 1;
      samplingConfig.numActiveChannels++;
    }
    token = strtok(NULL, ",");
  }

  free(str);
}

void parseInitCommand(const char *cmd)
{
  char cmdCopy[MAX_CMD_LENGTH];
  strncpy(cmdCopy, cmd, MAX_CMD_LENGTH - 1);
  cmdCopy[MAX_CMD_LENGTH - 1] = '\0';

  char *savePtr2;
  char *token = strtok_r(cmdCopy, ";", &savePtr2);
  while (token != NULL)
  {
    // Skip leading spaces only (don't modify the string)
    while (isspace(*token))
      token++;

    if (debugTiming)
    {
      Serial.print("DEBUG: Token after trimming: '");
      Serial.print(token);
      Serial.println("'");
    }

    if (strncmp(token, "rate_limit=", 11) == 0)
    {
      char *value = token + 11;
      if (strcmp(value, "min") == 0)
      {
        samplingConfig.rateLimit = 0; // Run as fast as possible
      }
      else
      {
        // Convert seconds to milliseconds
        samplingConfig.rateLimit = atol(value) * 1000;
      }
    }
    else if (strncmp(token, "sample_average=", 14) == 0)
    {
      // Use strtol for more robust parsing
      // Find the '=' character and point to what comes after it
      char *equalsPos = strchr(token, '=');
      char *value = equalsPos ? equalsPos + 1 : token + 14;
      char *endptr;
      long samples = strtol(value, &endptr, 10);

      if (debugTiming)
      {
        Serial.print("DEBUG: Parsing '");
        Serial.print(value);
        Serial.print("' -> ");
        Serial.print(samples);
        Serial.print(", endptr='");
        Serial.print(*endptr);
        Serial.println("'");
      }

      // Check if parsing was successful
      if (endptr == value || *endptr != '\0')
      {
        Serial.println("Warning: Invalid sample_average value");
        samples = 0;
      }

      if (samples > 0 && samples <= 255)
      {
        samplingConfig.numSamples = (uint8_t)samples;
        if (debugTiming)
        {
          Serial.print("DEBUG: Set numSamples to ");
          Serial.println(samplingConfig.numSamples);
        }
      }
      else
      {
        Serial.println("Warning: sample_average out of range (1-255)");
      }
    }
    else if (strncmp(token, "channels=", 9) == 0)
    {
      parseChannels(token + 9);
    }
    else if (strcmp(token, "start") == 0)
    {
      samplingConfig.isRunning = true;
    }

    token = strtok_r(NULL, ";", &savePtr2);
  }

  // Print final configuration once
  Serial.println("\n=== Configuration ===");
  Serial.print("Rate limit: ");
  Serial.print(samplingConfig.rateLimit);
  Serial.println("ms");
  Serial.print("Sample average: ");
  Serial.println(samplingConfig.numSamples);
  Serial.print("Active channels: ");
  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    if (samplingConfig.activeChannels[i])
    {
      Serial.print(i);
      Serial.print(" ");
    }
  }
  Serial.println();
  Serial.print("Sampling: ");
  Serial.println(samplingConfig.isRunning ? "started" : "stopped");
  Serial.println("==================\n");
}

void processCommand(const char *cmd)
{
  // Trim leading/trailing whitespace
  while (isspace(*cmd))
    cmd++;

  // Extract the base command (everything before first space)
  char baseCmd[MAX_CMD_LENGTH];
  int i = 0;
  while (cmd[i] && !isspace(cmd[i]) && i < MAX_CMD_LENGTH - 1)
  {
    baseCmd[i] = tolower(cmd[i]); // Convert to lowercase for case-insensitive comparison
    i++;
  }
  baseCmd[i] = '\0';

  // Get the parameters (everything after first space)
  const char *params = cmd[i] ? &cmd[i + 1] : "";
  while (isspace(*params))
    params++; // Skip leading spaces in parameters

  // Process commands
  if (strcmp(baseCmd, CMD_INIT) == 0)
  {
    parseInitCommand(params);
  }
  else if (strcmp(baseCmd, CMD_START) == 0)
  {
    samplingConfig.isRunning = true;
    Serial.println("Sampling started");
  }
  else if (strcmp(baseCmd, CMD_STOP) == 0)
  {
    samplingConfig.isRunning = false;
    Serial.println("Sampling stopped");
  }
  else if (strcmp(baseCmd, CMD_SAMPLE) == 0)
  {
    // Sample specific channels: "sample 0" or "sample 0,1,2" or "sample all"
    if (*params)
    {
      // Clear any previous channel configuration
      memset(samplingConfig.activeChannels, 0, MAX_CHANNELS);
      samplingConfig.numActiveChannels = 0;

      // Parse the channels
      parseChannels(params);

      // Take one set of samples with current configuration
      bool first = true;
      for (int ch = 0; ch < MAX_CHANNELS; ch++)
      {
        if (samplingConfig.activeChannels[ch])
        {
          sampleChannels(ch, false);
          // Output in format: CH:VALUE,CH:VALUE,...
          if (!first)
            Serial.print(",");
          Serial.print(ch);
          Serial.print(":");
          Serial.print(measuredRValues[ch], 4);
          first = false;
        }
      }
      if (!first)
        Serial.println();
    }
    else
    {
      Serial.println("Error: Please specify channel(s) to sample (e.g., 'sample 0' or 'sample 0,1,2' or 'sample all')");
    }
  }
  else if (strcmp(baseCmd, CMD_SHOW) == 0)
  {
    input_showChannels();
    paginate = true;
  }
  else if (strcmp(baseCmd, CMD_LOG) == 0)
  {
    input_logData();
    paginate = false;
  }
  else if (strcmp(baseCmd, CMD_RESET) == 0)
  {
    Serial.println("Resetting reference compensation...");
    referenceSet = false;
    referenceReading = 0;
    Serial.println("Reference reset. Next measurement will set new reference.");
  }
  else if (strcmp(baseCmd, CMD_DIAG) == 0)
  {
    diagnoseMeasurementVariation(0);
  }
  else if (strcmp(baseCmd, CMD_TEST) == 0)
  {
    isolateVariationSource(0);
  }
  else if (strcmp(baseCmd, CMD_HELP) == 0)
  {
    showHelp();
  }
  else
  {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}

void showHelp()
{
  Serial.println("\n=== Available Commands ===");
  Serial.println("Configuration:");
  Serial.println("  init rate_limit=min;sample_average=N;channels=0,1,2,3;start");
  Serial.println("  Example 1: init rate_limit=min;sample_average=10;channels=0,1,2;start");
  Serial.println("  Example 2: init rate_limit=1;sample_average=10;channels=0,1,2;start  (1 sec between updates)");
  Serial.println("\nBasic Commands:");
  Serial.println("  start - Begin continuous sampling");
  Serial.println("  stop - Stop continuous sampling");
  Serial.println("  sample [channels] - Sample specific channels (e.g., 'sample 0,1,2' or 'sample all')");
  Serial.println("  show - Show all channel values");
  Serial.println("  log - Log data to SD card");
  Serial.println("\nMaintenance Commands:");
  Serial.println("  reset - Reset reference compensation");
  Serial.println("  diag - Run diagnostic test");
  Serial.println("  test - Run isolation test");
  Serial.println("  help - Show this help message");
  Serial.println("\nOutput Format:");
  Serial.println("  CH:VALUE,CH:VALUE,... (e.g., 0:123.45,1:234.56)");
  Serial.println("======================\n");
}

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

  Serial.println("\nType 'help' for available commands.");
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

  static uint32_t lastSampleTime = 0;
  static char cmdBuffer[MAX_CMD_LENGTH];
  static uint8_t cmdIndex = 0;

  // Handle serial commands
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (cmdIndex > 0)
      {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    }
    else if (cmdIndex < MAX_CMD_LENGTH - 1)
    {
      cmdBuffer[cmdIndex++] = c;
    }
  }

  // Handle continuous sampling if enabled
  if (samplingConfig.isRunning)
  {
    uint32_t now = millis();
    uint32_t elapsed = now - lastSampleTime;

    if (elapsed >= samplingConfig.rateLimit)
    {
      if (debugTiming)
      {
        Serial.print("LOOP_START: ");
        Serial.println(now);
      }

      // Sample all active channels
      bool first = true;
      for (int ch = 0; ch < MAX_CHANNELS; ch++)
      {
        if (samplingConfig.activeChannels[ch])
        {
          if (debugTiming)
          {
            Serial.print("SAMPLE_START_CH");
            Serial.print(ch);
            Serial.print(": ");
            Serial.println(millis());
          }

          sampleChannels(ch, false);

          if (debugTiming)
          {
            Serial.print("SAMPLE_END_CH");
            Serial.print(ch);
            Serial.print(": ");
            Serial.println(millis());
          }

          // Output in format: CH:VALUE,CH:VALUE,...
          if (!first)
            Serial.print(",");
          Serial.print(ch);
          Serial.print(":");
          Serial.print(measuredRValues[ch], 4);
          first = false;
        }
      }
      if (!first)
        Serial.println(); // Only print newline if we output something

      if (debugTiming)
      {
        Serial.print("LOOP_END: ");
        Serial.println(millis());
      }

      lastSampleTime = now;
    }
  }

  // Handle button inputs (preserved from original code)
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
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      displayChannels(currentPage);
      currentPage = (currentPage == 0) ? 1 : 0;
    }
  }

  delay(1);
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
  uint32_t startTime = millis();
  if (debugTiming)
  {
    Serial.print("SAMPLE_FUNC_START: ");
    Serial.println(startTime);
  }

  // Check if a valid channel is passed (0-15)
  int startChannel = 0;
  int endChannel = 16;

  if (channel >= 0 && channel < 16)
  {
    // If a valid channel is passed, only sample that channel
    startChannel = channel;
    endChannel = channel + 1;
  }

  // Initialize accumulator arrays
  long adcSums[16] = {0};
  int validSampleCounts[16] = {0};

  // Take numSamples rounds, sampling all channels each round
  if (debugTiming)
  {
    Serial.print("DEBUG: Starting sampling loop with numSamples=");
    Serial.println(samplingConfig.numSamples);
  }

  for (int sample = 0; sample < samplingConfig.numSamples; sample++)
  {
    if (debugTiming)
    {
      Serial.print("ROUND_START_");
      Serial.print(sample);
      Serial.print(": ");
      Serial.println(millis());
    }

    for (int ch = startChannel; ch < endChannel; ch++)
    {
      if (debugTiming)
      {
        Serial.print("CHANNEL_START_");
        Serial.print(ch);
        Serial.print(": ");
        Serial.println(millis());
      }

      // Set the channel for mux_adc and mux_cur
      mux_adc.channel(ch);
      mux_cur.channel(ch);
      delay(MUX_DELAY);

      if (debugTiming)
      {
        Serial.print("MUX_DELAY_END: ");
        Serial.println(millis());
      }

      ads.sendSTART();
      delay(START_DELAY);

      if (debugTiming)
      {
        Serial.print("START_DELAY_END: ");
        Serial.println(millis());
      }

      int32_t adcValue = ads.readConvertedData(ADCstatus, DIRECT);
      adcSums[ch] += adcValue;
      validSampleCounts[ch]++;

      if (debugTiming)
      {
        Serial.print("ADC_READ_END: ");
        Serial.println(millis());
      }

      // Small delay between channels in same round
      delay(CHANNEL_DELAY_MS);

      if (debugTiming)
      {
        Serial.print("CHANNEL_DELAY_END: ");
        Serial.println(millis());
      }
    }

    // Longer delay between rounds to capture drift
    if (sample < samplingConfig.numSamples - 1)
    {
      delay(ROUND_DELAY_MS);
      if (debugTiming)
      {
        Serial.print("ROUND_DELAY_END: ");
        Serial.println(millis());
      }
    }
  }

  // Process accumulated samples for each channel
  for (int ch = startChannel; ch < endChannel; ch++)
  {
    if (debugTiming)
    {
      Serial.print("PROCESS_START_CH");
      Serial.print(ch);
      Serial.print(": ");
      Serial.println(millis());
    }

    int32_t avgAdcValue = adcSums[ch] / validSampleCounts[ch];

    // Apply reference drift compensation
    if (!referenceSet && ch == startChannel)
    {
      // Set reference on first channel of first measurement
      referenceReading = avgAdcValue;
      referenceSet = true;
    }

    // Compensate for reference drift by using reference as baseline
    int32_t compensatedValue = avgAdcValue;
    float R = calcADS_R(compensatedValue, doDebug, true);

    if (debugTiming)
    {
      Serial.print("CALC_R_END_CH");
      Serial.print(ch);
      Serial.print(": ");
      Serial.println(millis());
    }

    // Handle 50µA current source switching if needed
    if (R == -1.0)
    {
      if (debugTiming)
      {
        Serial.print("CURRENT_SWITCH_START_CH");
        Serial.print(ch);
        Serial.print(": ");
        Serial.println(millis());
      }

      // Switch to 50µA and re-sample this channel with time distribution
      ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_50);
      currentSource = SOURCE_CAL_50UA;
      delay(CURRENT_SOURCE_DELAY);

      if (debugTiming)
      {
        Serial.print("CURRENT_SWITCH_50UA_END: ");
        Serial.println(millis());
      }

      // Re-sample this channel with 50µA using time distribution
      long adcSum50ua = 0;
      int validSamples50ua = 0;

      for (int sample = 0; sample < samplingConfig.numSamples; sample++)
      {
        if (debugTiming)
        {
          Serial.print("RESAMPLE_ROUND_");
          Serial.print(sample);
          Serial.print(": ");
          Serial.println(millis());
        }

        mux_adc.channel(ch);
        mux_cur.channel(ch);
        delay(MUX_DELAY);

        ads.sendSTART();
        delay(START_DELAY);

        int32_t adcValue50ua = ads.readConvertedData(ADCstatus, DIRECT);
        adcSum50ua += adcValue50ua;
        validSamples50ua++;

        if (sample < samplingConfig.numSamples - 1)
        {
          delay(ROUND_DELAY_MS);
        }
      }

      avgAdcValue = adcSum50ua / validSamples50ua;
      int32_t compensated50ua = avgAdcValue;
      R = calcADS_R(compensated50ua, doDebug, false);

      if (debugTiming)
      {
        Serial.print("RESAMPLE_CALC_END: ");
        Serial.println(millis());
      }

      // Reset to 10µA
      ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10);
      currentSource = SOURCE_CAL_10UA;
      delay(CURRENT_SOURCE_DELAY);

      if (debugTiming)
      {
        Serial.print("CURRENT_SWITCH_10UA_END: ");
        Serial.println(millis());
      }
    }

    // Store the values in arrays
    adcValues[ch] = avgAdcValue;
    measuredRValues[ch] = R;

    if (debugTiming)
    {
      Serial.print("PROCESS_END_CH");
      Serial.print(ch);
      Serial.print(": ");
      Serial.println(millis());
    }
  }

  if (debugTiming)
  {
    Serial.print("SAMPLE_FUNC_END: ");
    Serial.println(millis());
  }
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
  Serial.print(samplingConfig.numSamples);
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
    R = 0.0;
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
  // Force test channel to 0 since that's our only valid reference
  testChannel = 0;

  Serial.print("=== Variation Source Isolation for Channel ");
  Serial.print(testChannel);
  Serial.println(" (Fixed Resistor) ===");

  // Set up the test channel
  mux_adc.channel(testChannel);
  mux_cur.channel(testChannel);
  delay(MUX_DELAY);

  const int numTests = 10;
  const int testDelay = 100; // Consistent delay between all measurements

  Serial.println("\n1. RAPID BASELINE TEST (minimal delay):");
  // Test 1: Same channel with minimal delays
  int32_t rapidReadings[numTests];
  for (int i = 0; i < numTests; i++)
  {
    ads.sendSTART();
    delay(START_DELAY);
    rapidReadings[i] = ads.readConvertedData(ADCstatus, DIRECT);
    Serial.print("Rapid ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(rapidReadings[i]);
    delay(testDelay);
  }

  Serial.println("\n2. DRIFT BASELINE TEST (500ms delay):");
  // Test 2: Same channel with longer delays to see drift
  int32_t driftReadings[numTests];
  for (int i = 0; i < numTests; i++)
  {
    ads.sendSTART();
    delay(START_DELAY);
    driftReadings[i] = ads.readConvertedData(ADCstatus, DIRECT);
    Serial.print("Drift ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(driftReadings[i]);
    delay(500); // Longer delay to observe drift
  }

  Serial.println("\n3. MUX SWITCHING TEST:");
  int32_t muxSwitchReadings[numTests];
  for (int i = 0; i < numTests; i++)
  {
    // Switch to channel 1 temporarily
    mux_adc.channel(1);
    mux_cur.channel(1);
    delay(MUX_DELAY);

    // Switch back to test channel 0
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
    delay(testDelay);
  }

  Serial.println("\n4. CURRENT SOURCE SWITCHING TEST:");
  int32_t currentSwitchReadings[numTests];
  for (int i = 0; i < numTests; i++)
  {
    ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_50);
    currentSource = SOURCE_CAL_50UA;
    delay(CURRENT_SOURCE_DELAY);

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
    delay(testDelay);
  }

  // Calculate ranges
  int32_t rapidMin = rapidReadings[0], rapidMax = rapidReadings[0];
  int32_t driftMin = driftReadings[0], driftMax = driftReadings[0];
  int32_t muxMin = muxSwitchReadings[0], muxMax = muxSwitchReadings[0];
  int32_t currentMin = currentSwitchReadings[0], currentMax = currentSwitchReadings[0];

  for (int i = 1; i < numTests; i++)
  {
    if (rapidReadings[i] < rapidMin)
      rapidMin = rapidReadings[i];
    if (rapidReadings[i] > rapidMax)
      rapidMax = rapidReadings[i];
    if (driftReadings[i] < driftMin)
      driftMin = driftReadings[i];
    if (driftReadings[i] > driftMax)
      driftMax = driftReadings[i];
    if (muxSwitchReadings[i] < muxMin)
      muxMin = muxSwitchReadings[i];
    if (muxSwitchReadings[i] > muxMax)
      muxMax = muxSwitchReadings[i];
    if (currentSwitchReadings[i] < currentMin)
      currentMin = currentSwitchReadings[i];
    if (currentSwitchReadings[i] > currentMax)
      currentMax = currentSwitchReadings[i];
  }

  Serial.println("\n=== VARIATION ANALYSIS ===");
  Serial.print("Rapid Sample Range: ");
  Serial.print(rapidMax - rapidMin);
  Serial.println(" (minimal delay baseline)");

  Serial.print("Drift Sample Range: ");
  Serial.print(driftMax - driftMin);
  Serial.println(" (with 500ms delay)");

  Serial.print("MUX Switching Range: ");
  Serial.print(muxMax - muxMin);
  Serial.println(" (adds MUX effects)");

  Serial.print("Current Switching Range: ");
  Serial.print(currentMax - currentMin);
  Serial.println(" (adds current source effects)");

  Serial.println("\n=== INTERPRETATION ===");
  int32_t rapidVariation = rapidMax - rapidMin;
  int32_t driftVariation = driftMax - driftMin;
  int32_t muxVariation = muxMax - muxMin;
  int32_t currentVariation = currentMax - currentMin;

  if (driftVariation > rapidVariation * 1.5)
  {
    Serial.println("-> THERMAL/REFERENCE DRIFT is significant");
    Serial.println("   - Consider using reference compensation");
    Serial.println("   - Try to minimize measurement time");
  }

  if (muxVariation > rapidVariation * 1.5)
  {
    Serial.println("-> MUX SWITCHING adds noise");
    Serial.println("   - Consider increasing MUX_DELAY");
  }

  if (currentVariation > rapidVariation * 1.5)
  {
    Serial.println("-> CURRENT SOURCE SWITCHING adds noise");
    Serial.println("   - Consider increasing CURRENT_SOURCE_DELAY");
  }

  Serial.println("=== END ISOLATION TEST ===\n");
}