#include <Arduino.h>
#include <SPI.h>

// ==============================================================================
// PIN DEFINITIONS & SPI SETTINGS
// ==============================================================================
constexpr uint8_t PIN_ADC_CS   = D7;  // D7 on XIAO ESP32-C6 (GPIO18)
constexpr uint8_t PIN_ADC_SCK  = D8;  // D8 on XIAO ESP32-C6 (GPIO19)
constexpr uint8_t PIN_ADC_MISO = D9;  // D9 on XIAO ESP32-C6 (GPIO20)
constexpr uint8_t PIN_ADC_MOSI = D10;  // D10 / unused by ADS7042, but defined for bus init

// ADS7042 allows up to 16 MHz. We use 4 MHz for breadboard/jumper robustness.
// SPI Mode 0 (CPOL=0, CPHA=0): Master samples MISO on rising edge of SCLK.
static SPISettings adcSPISettings(4000000, MSBFIRST, SPI_MODE0);

constexpr float V_REF = 3.3f;          // ADC full-scale reference (AVDD)
constexpr float ATTENUATION = 0.5913f; // 6.8k / (4.7k + 6.8k) divider factor

// ==============================================================================
// LOW-LEVEL ADC READ
// ==============================================================================
// ADS7042 SPI Frame Format (16 clocks):
// Bits [15:14] = 00 (Leading zeros)
// Bits [13:2]  = D11..D0 (12-bit conversion code)
// Bits [1:0]   = 00 (Trailing zeros)
inline uint16_t readADS7042_Raw(bool &framingValid) {
  SPI.beginTransaction(adcSPISettings);
  
  digitalWrite(PIN_ADC_CS, LOW); // Falling edge samples input and initiates conversion
  
  // Read 2 bytes (16 clock pulses)
  uint16_t raw = SPI.transfer16(0x0000);
  
  digitalWrite(PIN_ADC_CS, HIGH); // CS high triggers acquisition phase (tACQ >= 200ns)
  SPI.endTransaction();

  // Validate timing/framing: Bits 15 and 14 must always be zero
  framingValid = ((raw & 0xC000) == 0);

  return raw;
}

inline uint16_t extract12BitCode(uint16_t raw) {
  return (raw >> 2) & 0x0FFF;
}

// ==============================================================================
// SETUP & POWER-UP CALIBRATION
// ==============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2500); // Allow USB-CDC to connect

  pinMode(PIN_ADC_CS, OUTPUT);
  digitalWrite(PIN_ADC_CS, HIGH);

  // Initialize hardware SPI on XIAO C6 pins
  SPI.begin(PIN_ADC_SCK, PIN_ADC_MISO, PIN_ADC_MOSI, PIN_ADC_CS);

  Serial.println("\n--- TI ADS7042 Hardware Verification Test ---");

  // Mandatory Offset Calibration (Section 10.4.1.1):
  // The first CS falling edge after power-up enters offset calibration.
  // It requires >= 16 SCLK falling edges. The SDO output remains 0 during this cycle.
  delay(10); // Settle supplies
  SPI.beginTransaction(adcSPISettings);
  digitalWrite(PIN_ADC_CS, LOW);
  SPI.transfer16(0x0000); // 16 clocks
  digitalWrite(PIN_ADC_CS, HIGH);
  SPI.endTransaction();
  
  delayMicroseconds(2); // Provide ample tACQ (> 200 ns) after calibration
  Serial.println("[OK] Power-up offset calibration cycle issued.");
  Serial.println("Starting burst analysis (every 500 ms)...");
}

// ==============================================================================
// MAIN LOOP: BURST DIAGNOSTICS
// ==============================================================================
void loop() {
  constexpr uint16_t BURST_SIZE = 128; // Capture ~19 cycles of 6.7 kHz
  uint16_t rawBuffer[BURST_SIZE];
  uint16_t codeBuffer[BURST_SIZE];
  bool framingErrors = false;

  // 1. High-speed burst acquisition
  for (uint16_t i = 0; i < BURST_SIZE; i++) {
    bool valid = false;
    rawBuffer[i] = readADS7042_Raw(valid);
    codeBuffer[i] = extract12BitCode(rawBuffer[i]);
    if (!valid) framingErrors = true;
    
    // Maintain minimum tACQ = 200 ns before next read
    delayMicroseconds(5); 
  }

  // 2. Compute Statistics over the burst
  uint16_t minCode = 4095;
  uint16_t maxCode = 0;
  uint32_t sumCode = 0;

  for (uint16_t i = 0; i < BURST_SIZE; i++) {
    if (codeBuffer[i] < minCode) minCode = codeBuffer[i];
    if (codeBuffer[i] > maxCode) maxCode = codeBuffer[i];
    sumCode += codeBuffer[i];
  }

  float avgCode = (float)sumCode / BURST_SIZE;
  float vMin = (minCode * V_REF) / 4096.0f;
  float vMax = (maxCode * V_REF) / 4096.0f;
  float vMean = (avgCode * V_REF) / 4096.0f;
  float vPP   = vMax - vMin;

  // Calculate upstream voltage (reconstructed before the resistor divider)
  float upstreamMean = vMean / ATTENUATION;
  float upstreamPP   = vPP / ATTENUATION;

  // 3. Print Diagnostic Output
  Serial.println("--------------------------------------------------");
  if (framingErrors) {
    Serial.println("[WARNING] Framing Error detected! (Leading bits were not 00)");
    Serial.println("           Check SCLK/MISO wiring integrity or ground noise.");
  } else {
    Serial.println("[PASS] SPI Framing synchronized (Leading bits are 00).");
  }

  // Print a sample raw word in Binary/Hex for inspection
  Serial.printf("Raw Sample [0]: 0x%04X | Binary: ", rawBuffer[0]);
  for (int b = 15; b >= 0; b--) {
    Serial.print((rawBuffer[0] >> b) & 1);
    if (b == 14 || b == 2) Serial.print(" "); // Group: [15:14] [13:2] [1:0]
  }
  Serial.println();

  // Print ADC-level and upstream signal measurements
  Serial.printf("ADC Input (Pin AINP):  Mean: %.3f V | Vpp: %.3f V | Span: [%.3f V - %.3f V]\n",
                vMean, vPP, vMin, vMax);
  Serial.printf("ADC Codes:             Mean: %.1f   | Min: %u      | Max: %u\n",
                avgCode, minCode, maxCode);
  Serial.printf("Upstream Op-Amp (Est): Mean: %.3f V | Vpp: %.3f V (Expected: ~2.5V DC, ~0.60Vpp)\n",
                upstreamMean, upstreamPP);

  delay(500); // Repeat every 500 ms
}