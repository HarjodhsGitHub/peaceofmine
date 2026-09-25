#include <Arduino.h>
#include <SPI.h>
#include <ArduinoJson.h>

// ==============================================================================
// PIN DEFINITIONS & HARDWARE CONSTANTS
// ==============================================================================
constexpr uint8_t PIN_ADC_CS   = D7;  // GPIO17 on XIAO ESP32-C6
constexpr uint8_t PIN_ADC_SCK  = D8;  // GPIO19 on XIAO ESP32-C6
constexpr uint8_t PIN_ADC_MISO = D9;  // GPIO20 on XIAO ESP32-C6
constexpr uint8_t PIN_ADC_MOSI = D10; // GPIO18 (bus initialization only)

// 8 MHz SPI clock guarantees fast, clean transfers over prototype jumpers
static SPISettings adcSPISettings(8000000, MSBFIRST, SPI_MODE0);

constexpr float V_REF = 3.3f;                   // ADS7042 AVDD rail
constexpr float ATTENUATION = 6.8f / (4.7f + 6.8f); // 0.591304f (4.7k / 6.8k divider)

// Number of samples per burst: 256 samples covers ~20 full cycles of 6.7 kHz
constexpr uint16_t BURST_SIZE = 256;

// ==============================================================================
// ADS7042 LOW-LEVEL READ (UNBUFFERED TIMING)
// ==============================================================================
inline uint16_t readADS7042() {
  SPI.beginTransaction(adcSPISettings);
  digitalWrite(PIN_ADC_CS, LOW); // CS falling edge captures input and starts conversion
  
  uint16_t raw = SPI.transfer16(0x0000); // 16 clock cycles
  
  digitalWrite(PIN_ADC_CS, HIGH); // CS high starts acquisition phase
  SPI.endTransaction();

  // Crucial for unbuffered 2.78k divider: Hold CS high for >= 4 us
  // to allow the internal 15 pF sampling cap to settle to 12-bit precision.
  delayMicroseconds(4);

  // Extract 12-bit payload: bits [13:2]
  return (raw >> 2) & 0x0FFF;
}

// ==============================================================================
// SETUP & OFFSET CALIBRATION
// ==============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2500); // Allow USB-CDC to connect

  pinMode(PIN_ADC_CS, OUTPUT);
  digitalWrite(PIN_ADC_CS, HIGH);

  SPI.begin(PIN_ADC_SCK, PIN_ADC_MISO, PIN_ADC_MOSI, PIN_ADC_CS);

  // Mandatory ADS7042 Power-Up Offset Calibration (datasheet section 10.4.1.1):
  // Provide 16 SCLK clocks during the first CS low frame to zero internal offsets.
  delay(10);
  SPI.beginTransaction(adcSPISettings);
  digitalWrite(PIN_ADC_CS, LOW);
  SPI.transfer16(0x0000);
  digitalWrite(PIN_ADC_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(10);
}

// ==============================================================================
// MAIN LOOP: 50 Hz TELEMETRY (20 ms PERIOD)
// ==============================================================================
void loop() {
  static uint32_t previousPrintMs = 0;
  static uint32_t sequence = 0;
  static JsonDocument message;

  const uint32_t now = millis();

  // 50 Hz execution interval (every 20 ms)
  if (now - previousPrintMs >= 20) {
    previousPrintMs = now;

    // 1. High-Speed Burst Capture
    uint16_t samples[BURST_SIZE];
    uint32_t sumCodes = 0;
    uint16_t rawMin = 4095;
    uint16_t rawMax = 0;

    const uint32_t burstStartUs = micros();
    for (uint16_t i = 0; i < BURST_SIZE; i++) {
      samples[i] = readADS7042();
      sumCodes += samples[i];
      if (samples[i] < rawMin) rawMin = samples[i];
      if (samples[i] > rawMax) rawMax = samples[i];
    }
    const uint32_t burstEndUs = micros();

    // 2. Compute Mean DC Voltages
    const float meanCode = static_cast<float>(sumCodes) / BURST_SIZE;
    const float adcMeanV = (meanCode * V_REF) / 4096.0f;
    const float opampMeanV = adcMeanV / ATTENUATION;

    // 3. Frequency & Cycle-Averaged Vpp Extraction
    // Use hysteresis midpoint crossing to reject ADC noise on the 3-5 mV target
    const uint16_t midCode = (rawMax + rawMin) / 2;
    const uint16_t hysteresis = 15; // ~12 mV noise deadband
    const float dtSampleUs = static_cast<float>(burstEndUs - burstStartUs) / BURST_SIZE;

    int32_t firstCrossIdx = -1;
    int32_t lastCrossIdx = -1;
    uint16_t cycleCount = 0;
    bool armed = (samples[0] < (midCode - hysteresis));

    uint32_t sumCycleVpp = 0;
    uint16_t localCycleMax = 0;
    uint16_t localCycleMin = 4095;

    for (uint16_t i = 0; i < BURST_SIZE; i++) {
      if (samples[i] > localCycleMax) localCycleMax = samples[i];
      if (samples[i] < localCycleMin) localCycleMin = samples[i];

      // Detect rising edge crossing through midpoint
      if (armed && (samples[i] > (midCode + hysteresis))) {
        if (firstCrossIdx < 0) {
          firstCrossIdx = i;
        } else {
          // Accumulate Vpp of completed cycle
          sumCycleVpp += (localCycleMax - localCycleMin);
          cycleCount++;
        }
        lastCrossIdx = i;
        localCycleMax = samples[i];
        localCycleMin = samples[i];
        armed = false; // Rearm below midpoint
      } else if (!armed && (samples[i] < (midCode - hysteresis))) {
        armed = true;
      }
    }

    // Calculate Frequency (Hz)
    float freqHz = 0.0f;
    if (cycleCount >= 2 && lastCrossIdx > firstCrossIdx) {
      const float elapsedUs = (lastCrossIdx - firstCrossIdx) * dtSampleUs;
      if (elapsedUs > 0.0f) {
        freqHz = (static_cast<float>(cycleCount) * 1000000.0f) / elapsedUs;
      }
    }

    // Calculate Peak-to-Peak (Averaged across captured cycles to suppress noise)
    float adcVppV = 0.0f;
    if (cycleCount > 0) {
      const float avgVppCodes = static_cast<float>(sumCycleVpp) / cycleCount;
      adcVppV = (avgVppCodes * V_REF) / 4096.0f;
    } else {
      // Fallback if frequency was undetectable (e.g., pure DC)
      adcVppV = ((rawMax - rawMin) * V_REF) / 4096.0f;
    }
    const float opampVppV = adcVppV / ATTENUATION;

    // 4. Construct JSON Document & Stream to Raspberry Pi
    message["seq"] = sequence++;
    message["uptime_ms"] = now;
    message["freq_hz"] = serialized(String(freqHz, 1));
    message["adc_mean_v"] = serialized(String(adcMeanV, 3));
    message["adc_vpp_v"] = serialized(String(adcVppV, 3));
    message["opamp_mean_v"] = serialized(String(opampMeanV, 3));
    message["opamp_vpp_v"] = serialized(String(opampVppV, 3));

    serializeJson(message, Serial);
    Serial.println(); // Newline delimiter for Python's readline()
  }
}