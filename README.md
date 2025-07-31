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

## Quick Start
1. Power on the device (LED will pulse green when ready)
2. Connect via serial terminal at 9600 baud
3. Type `help` to see available commands
4. Try a single measurement: `sample 0,1,2`
5. Start continuous measurements: `init rate_limit=min;sample_average=10;channels=0,1,2;start`

## Detailed Usage
### Power-Up Sequence
When powered on, the LED will show:
- **Blue**: During initialization
- **Pulsing Green**: Ready for commands
- **Pulsing Red**: Error condition

The initialization sequence includes:
1. Serial port (USB)
2. IO configuration
3. ADC setup
4. MUX enabling
5. SD card check
6. Gas sensors setup

### Command Interface
The device accepts commands through the serial port (9600 baud). There are two types of sampling commands:

#### 1. Single Sample Commands
```
sample 0              # Sample only channel 0
sample 0,1,2         # Sample channels 0, 1, and 2
sample all           # Sample all channels (0-15)
```
- Takes one measurement using current averaging settings
- Returns results immediately
- Format: `CH:VALUE,CH:VALUE,...` (e.g., `0:123.45,1:234.56`)

#### 2. Continuous Sampling Commands
```
# Full configuration with immediate start:
init rate_limit=min;sample_average=10;channels=0,1,2;start

# Or configure and start separately:
init rate_limit=1;sample_average=10;channels=0,1,2
start

# Stop continuous sampling:
stop
```

Configuration Parameters:
- `rate_limit`: Time between measurements
  - `min`: As fast as possible
  - `1`: One second between measurements
  - `0.5`: Half second between measurements
  - etc.
- `sample_average`: Number of samples to average (1-255)
  - Higher values: More stable readings, slower updates
  - Lower values: Faster updates, more noise
- `channels`: Which channels to measure
  - Individual: `0,1,2`
  - All: `all`

#### 3. Utility Commands
```
show     # Display current values for all channels
log      # Save all channel values to SD card
help     # Show command help
```

### Output Formats
All measurements are output in the format:
```
CH:VALUE,CH:VALUE,...

Example:
0:123.4567,1:234.5678,2:345.6789
```
- CH: Channel number (0-15)
- VALUE: Resistance in ohms (4 decimal places)

### Data Logging
The `log` command saves data to the SD card:
- Files are numbered sequentially (00000.TXT, 00001.TXT, etc.)
- CSV format with "key,value" header
- Includes all channel measurements and gas sensor data
- Use [Neurotech-Hub/Chemisensor-MATLAB](https://github.com/Neurotech-Hub/Chemisensor-MATLAB) for data analysis

### Performance Notes
The device uses optimized timing for stability:
- MUX switching: 50ms
- ADC conversion: 15ms
- Current source: 85ms
- Round delay: 25ms
- Channel delay: 2ms

These timings balance measurement speed and accuracy. For best results:
1. Allow system to warm up for 3-5 minutes
2. Use `sample_average=3` or higher for stable readings (at the expense of longer sampling duration)

### Timing Analysis & Performance Bottlenecks

Based on actual system measurements, here are the precise timing breakdowns and performance impacts:

#### Single Channel, Single Sample Timing (Best Case)
```
MUX_DELAY: 26ms (25ms configured + 1ms overhead)
START_DELAY: 8ms (8ms configured)
ADC_READ: 1ms (ADC conversion time)
CHANNEL_DELAY: 2ms (2ms configured)
PROCESSING: 1ms (calculations and storage)
TOTAL: ~38ms per measurement
Theoretical Rate: ~26 Hz
```

#### Multi-Channel Impact
When measuring multiple channels, timing scales linearly:
- **2 channels**: ~76ms per measurement cycle (~13 Hz)
- **4 channels**: ~152ms per measurement cycle (~6.6 Hz)
- **8 channels**: ~304ms per measurement cycle (~3.3 Hz)
- **16 channels**: ~608ms per measurement cycle (~1.6 Hz)

#### Averaging Impact
The `sample_average` parameter has a significant impact:
- **Each additional sample adds**: 38ms + 10ms (ROUND_DELAY) = 48ms
- **sample_average=10**: 38ms + (9 × 48ms) = 470ms per measurement (~2.1 Hz)
- **sample_average=20**: 38ms + (19 × 48ms) = 950ms per measurement (~1.1 Hz)

#### Current Source Switching Impact
If a channel triggers current source switching (low resistance), additional delays occur:
- **50µA switch**: 85ms
- **Re-sampling**: 38ms × numSamples
- **10µA switch back**: 85ms
- **Total additional**: 170ms + (38ms × numSamples)

#### Main Loop Overhead
- **Loop processing**: 2ms between measurements
- **Serial output**: Variable (depends on baud rate and data length)

#### Performance Optimization Recommendations

**For Maximum Speed:**
- Use `sample_average=1` for fastest updates
- Limit to 1-2 channels for real-time monitoring
- Avoid channels that trigger current source switching

**For Maximum Stability:**
- Use `sample_average=10` or higher
- Allow 3-5 minute warm-up period
- Use multiple channels for comprehensive monitoring

**For Balanced Performance:**
- Use `sample_average=3-5` for good stability/speed balance
- Monitor 4-8 channels simultaneously
- Consider the trade-off between update rate and measurement quality

#### Real-World Performance Examples

| Configuration | Channels | Sample Avg | Expected Rate | Use Case |
|---------------|----------|------------|---------------|----------|
| `rate_limit=min;sample_average=1;channels=0` | 1 | 1 | ~26 Hz | High-speed single sensor |
| `rate_limit=min;sample_average=1;channels=0,1,2` | 3 | 1 | ~8.7 Hz | Fast multi-sensor |
| `rate_limit=min;sample_average=10;channels=0,1,2` | 3 | 10 | ~2.1 Hz | Stable multi-sensor |
| `rate_limit=min;sample_average=10;channels=all` | 16 | 10 | ~0.4 Hz | Comprehensive monitoring |

### Multi-Channel Sampling Rate Optimization

#### Current Timing Variables (in order of impact)

**1. MUX_DELAY (LARGEST IMPACT)**
```cpp
#define MUX_DELAY 25  // Currently 25ms
```
- **Impact**: 26ms per channel (25ms + 1ms overhead)
- **Multiplier**: × number of channels
- **Optimization**: Reduce to 10-15ms for faster switching
- **Risk**: May cause MUX settling issues

**2. START_DELAY**
```cpp
#define START_DELAY 8  // Currently 8ms
```
- **Impact**: 8ms per channel
- **Multiplier**: × number of channels
- **Optimization**: Reduce to 3-5ms
- **Risk**: ADC may not be ready for conversion

**3. CHANNEL_DELAY_MS**
```cpp
#define CHANNEL_DELAY_MS 2  // Currently 2ms
```
- **Impact**: 2ms per channel
- **Multiplier**: × number of channels
- **Optimization**: Reduce to 0-1ms
- **Risk**: Minimal risk, safe to reduce

**4. ROUND_DELAY_MS (Averaging Impact)**
```cpp
#define ROUND_DELAY_MS 10  // Currently 10ms
```
- **Impact**: 10ms × (numSamples - 1)
- **Multiplier**: × (sample_average - 1)
- **Optimization**: Reduce to 5ms or eliminate
- **Risk**: May reduce measurement stability

#### Performance Impact Calculations

**Current Multi-Channel Timing:**
```
Per Channel: 26ms (MUX) + 8ms (START) + 2ms (CHANNEL) = 36ms
Per Measurement Cycle: 36ms × number_of_channels
```

**Optimized Multi-Channel Timing (aggressive):**
```
Per Channel: 15ms (MUX) + 5ms (START) + 1ms (CHANNEL) = 21ms
Per Measurement Cycle: 21ms × number_of_channels
```

#### Specific Optimization Strategies

**1. Reduce MUX_DELAY (Highest Impact)**
```cpp
#define MUX_DELAY 15  // Reduce from 25ms to 15ms
```
- **Savings**: 11ms per channel
- **4 channels**: 44ms saved per cycle
- **8 channels**: 88ms saved per cycle
- **16 channels**: 176ms saved per cycle

**2. Reduce START_DELAY (Medium Impact)**
```cpp
#define START_DELAY 5  // Reduce from 8ms to 5ms
```
- **Savings**: 3ms per channel
- **4 channels**: 12ms saved per cycle
- **8 channels**: 24ms saved per cycle
- **16 channels**: 48ms saved per cycle

**3. Eliminate CHANNEL_DELAY_MS (Low Impact)**
```cpp
#define CHANNEL_DELAY_MS 0  // Reduce from 2ms to 0ms
```
- **Savings**: 2ms per channel
- **4 channels**: 8ms saved per cycle
- **8 channels**: 16ms saved per cycle
- **16 channels**: 32ms saved per cycle

#### Expected Performance Improvements

| Configuration | Current Rate | Optimized Rate | Improvement |
|---------------|--------------|----------------|-------------|
| 4 channels, 1 sample | ~6.6 Hz | ~11.9 Hz | +80% |
| 8 channels, 1 sample | ~3.3 Hz | ~5.9 Hz | +79% |
| 16 channels, 1 sample | ~1.6 Hz | ~2.9 Hz | +81% |

#### Implementation Recommendations

**Conservative Optimization (Low Risk):**
```cpp
#define MUX_DELAY 20        // Reduce by 5ms
#define START_DELAY 6       // Reduce by 2ms
#define CHANNEL_DELAY_MS 1  // Reduce by 1ms
```

**Aggressive Optimization (Higher Risk):**
```cpp
#define MUX_DELAY 15        // Reduce by 10ms
#define START_DELAY 5       // Reduce by 3ms
#define CHANNEL_DELAY_MS 0  // Eliminate
```

**Maximum Speed (Experimental):**
```cpp
#define MUX_DELAY 10        // Reduce by 15ms
#define START_DELAY 3       // Reduce by 5ms
#define CHANNEL_DELAY_MS 0  // Eliminate
```

#### Testing Strategy

1. **Start with conservative optimization**
2. **Test with known stable resistors**
3. **Monitor for measurement drift**
4. **Gradually reduce delays if stable**
5. **Use diagnostic commands to verify accuracy**

#### Alternative Approaches

**1. Parallel Sampling (Hardware Modification)**
- Use multiple ADCs in parallel
- Eliminates sequential channel switching
- Requires significant hardware changes

**2. Selective Channel Monitoring**
- Only sample channels with changing values
- Implement adaptive sampling rates
- Requires software intelligence

**3. Interleaved Sampling**
- Sample different channel groups at different rates
- Prioritize critical channels
- Complex but effective for mixed requirements

### Current Source Switching Optimization (MAJOR IMPACT)

#### Current Source Switching Logic

The system automatically switches between 10µA and 50µA current sources based on resistance values:

```cpp
const float ADC_SOURCE_THRESH = VREF - 0.2; // 2.3V threshold
const float SOURCE_MULTIPLIER = 5.0;        // 10µA to 50µA multiplier

// In calcADS_R():
if (ADCNode * SOURCE_MULTIPLIER < ADC_SOURCE_THRESH && allowSourceMod)
{
  return -1.0; // Triggers current source switching
}
```

**When triggered, current source switching adds:**
- **50µA switch delay**: 85ms
- **Re-sampling**: 38ms × numSamples
- **10µA switch back**: 85ms
- **Total additional**: 170ms + (38ms × numSamples)

#### Current Source Switching Impact

**For single sample (sample_average=1):**
- **Additional time**: 170ms + 38ms = 208ms
- **Total measurement**: 38ms + 208ms = 246ms
- **Rate impact**: 26 Hz → 4 Hz (85% reduction!)

**For 10 samples (sample_average=10):**
- **Additional time**: 170ms + (38ms × 10) = 550ms
- **Total measurement**: 470ms + 550ms = 1020ms
- **Rate impact**: 2.1 Hz → 1 Hz (52% reduction!)

#### Optimization Strategies

**1. Disable Current Source Switching (LARGEST IMPACT)**
```cpp
// Option A: Set threshold to 0 (never switch)
const float ADC_SOURCE_THRESH = 0.0;

// Option B: Modify calcADS_R() to never return -1.0
if (false && ADCNode * SOURCE_MULTIPLIER < ADC_SOURCE_THRESH && allowSourceMod)
{
  return -1.0; // Never triggered
}
```

**Impact**: Eliminates 170ms + re-sampling time per triggered measurement

**2. Use Fixed Current Source**
```cpp
// Always use 50µA for better low-resistance measurement
ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_50);
currentSource = SOURCE_CAL_50UA;
```

**Impact**: Eliminates switching delays, but may reduce accuracy for high-resistance measurements

**3. Adjust Threshold**
```cpp
// Make switching less likely
const float ADC_SOURCE_THRESH = VREF - 0.5; // More conservative threshold
```

**Impact**: Reduces frequency of switching while maintaining some dynamic range

#### Performance Impact of Disabling Current Source Switching

| Configuration | With Switching | Without Switching | Improvement |
|---------------|----------------|-------------------|-------------|
| 1 channel, 1 sample | ~4 Hz | ~26 Hz | +550% |
| 4 channels, 1 sample | ~1 Hz | ~6.6 Hz | +560% |
| 8 channels, 1 sample | ~0.5 Hz | ~3.3 Hz | +560% |

#### Implementation Recommendations

**For Maximum Speed (Recommended for known resistance ranges):**
```cpp
// Disable current source switching entirely
const float ADC_SOURCE_THRESH = 0.0;
```

**For Balanced Performance:**
```cpp
// Use fixed 50µA current source
// Modify configureADS() to always use 50µA
ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_50);
currentSource = SOURCE_CAL_50UA;
```

### Charging
**Important:** The power switch must be ON to charge the battery. This is due to the simple battery switch design utilizing the Arduino's charging module.

## Dependencies
### Required Libraries
Install these libraries in your Arduino IDE (Sketch → Include Library → Manage Libraries):

**Core Arduino Libraries (usually pre-installed):**
- `SPI` - SPI communication
- `SD` - SD card operations
- `Wire` - I2C communication
- `WiFiNINA` - WiFi functionality for MKR WiFi 1010

**Third-Party Libraries:**
- `ADS124S06` - [Neurotech-Hub/ADS124S06-Arduino](https://github.com/Neurotech-Hub/ADS124S06-Arduino) (clone to Arduino/libraries)
- `light_CD74HC4067` - CD74HC4067 multiplexer control
- `Adafruit_Sensor` - Adafruit sensor library
- `Adafruit_BME680` - BME680 gas sensor library
- `Adafruit_GFX` - Graphics library for displays
- `Adafruit_SSD1306` - SSD1306 OLED display library
- `BQ24195` - Battery management IC library

### Installation Instructions
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for and install the Adafruit libraries listed above
4. For the custom ADS124S06 library:
   - Download from [Neurotech-Hub/ADS124S06-Arduino](https://github.com/Neurotech-Hub/ADS124S06-Arduino)
   - Extract to your Arduino/libraries folder
5. For other libraries not in Library Manager, download and install manually

## Hardware Notes
- The I2C screen is modified with a 3.3V line for battery operation
- Measurement stability includes thermal and reference drift compensation