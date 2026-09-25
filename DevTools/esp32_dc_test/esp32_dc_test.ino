#include <Arduino.h>
#include <Wire.h>

// ==============================================================================
// PIN DEFINITIONS & I2C SETTINGS
// ==============================================================================
constexpr uint8_t PIN_I2C_SDA  = D4;  // GPIO22 on XIAO ESP32-C6 -> LV1
constexpr uint8_t PIN_I2C_SCL  = D5;  // GPIO23 on XIAO ESP32-C6 -> LV2
constexpr uint8_t PIN_TEST_OUT = D3;  // GPIO21 -> Connect directly to ADS1115 AIN0

constexpr uint8_t ADS1115_ADDR = 0x48; // ADDR pin tied to GND

// ADS1115 Registers (Section 8.1)
constexpr uint8_t REG_CONVERSION = 0x00;
constexpr uint8_t REG_CONFIG     = 0x01;

// LSB size for FSR = ±6.144V (Section 7.3.3, Table 7-1)
constexpr float LSB_VOLTS = 0.0001875f; // 187.5 µV per count

// ==============================================================================
// I2C BUS SCANNER
// ==============================================================================
bool scanI2C(uint8_t targetAddr) {
  Wire.beginTransmission(targetAddr);
  uint8_t error = Wire.endTransmission();
  return (error == 0);
}

// ==============================================================================
// ADS1115 CONFIGURATION
// ==============================================================================
void configureADS1115() {
  // Config Register Bitfields (Section 8.1.3, Table 8-3):
  // Bit 15:    OS = 0 (No effect in continuous mode)
  // Bits 14:12: MUX[2:0] = 100b (Single-ended AIN0 vs GND)
  // Bits 11:9:  PGA[2:0] = 000b (FSR = ±6.144V, safe for up to 5V inputs)
  // Bit 8:     MODE = 0b (Continuous-conversion mode)
  //   -> MSB = 0b01000000 = 0x40
  //
  // Bits 7:5:  DR[2:0] = 100b (128 SPS)
  // Bit 4:     COMP_MODE = 0b (Traditional)
  // Bit 3:     COMP_POL = 0b (Active low)
  // Bit 2:     COMP_LAT = 0b (Non-latching)
  // Bits 1:0:  COMP_QUE[1:0] = 11b (Disable comparator)
  //   -> LSB = 0b10000011 = 0x83

  uint8_t configMSB = 0x40;
  uint8_t configLSB = 0x83;

  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(REG_CONFIG);
  Wire.write(configMSB);
  Wire.write(configLSB);
  Wire.endTransmission();

  // Point the Address Pointer register back to the Conversion Register (0x00)
  // Once set, subsequent reads can directly stream 2 bytes (Section 7.5.3)
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(REG_CONVERSION);
  Wire.endTransmission();
}

// ==============================================================================
// READ 16-BIT CONVERSION VALUE
// ==============================================================================
int16_t readADS1115_Raw() {
  Wire.requestFrom(ADS1115_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return (int16_t)((msb << 8) | lsb); // 16-bit two's complement (Section 7.5.4)
  }
  return 0;
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2500);

  Serial.println("\n--- TI ADS1115 Hardware Test (XIAO ESP32-C6) ---");

  // Configure test signal generator pin
  pinMode(PIN_TEST_OUT, OUTPUT);
  digitalWrite(PIN_TEST_OUT, LOW);

  // Initialize hardware I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000); // 100 kHz standard mode

  // Verify connection through level shifter
  Serial.print("Checking I2C address 0x48... ");
  if (scanI2C(ADS1115_ADDR)) {
    Serial.println("[OK] ADS1115 detected via level shifter!");
  } else {
    Serial.println("[FAIL] Device not responding at 0x48.");
    Serial.println(" -> Check level shifter LV (3.3V), HV (5V), and GND wiring.");
    Serial.println(" -> Ensure ADDR pin is firmly tied to GND.");
    while (1) delay(100);
  }

  // Initialize ADC configuration
  configureADS1115();
  delay(20); // Wait for first conversion to settle (128 SPS = ~8 ms)

  Serial.println("Setup complete. Toggling D3 between 0V and 3.3V every 2.5s...");
  Serial.println("---------------------------------------------------------------");
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================
void loop() {
  static uint32_t lastToggleTime = 0;
  static uint32_t lastSampleTime = 0;
  static bool testPinState = false;

  uint32_t now = millis();

  // 1. Toggle Test Signal Pin D3 every 2500 ms (Non-blocking)
  if (now - lastToggleTime >= 2500) {
    lastToggleTime = now;
    testPinState = !testPinState;
    digitalWrite(PIN_TEST_OUT, testPinState ? HIGH : LOW);
    
    Serial.println("\n---------------------------------------------------------------");
    Serial.printf(">>> SWITCHING TEST PIN D3 TO: %s (Nominal: %s) <<<\n",
                  testPinState ? "HIGH" : "LOW",
                  testPinState ? "3.30 V" : "0.00 V");
    Serial.println("---------------------------------------------------------------");
  }

  // 2. Poll ADS1115 every 100 ms (10 Hz sample rate)
  if (now - lastSampleTime >= 100) {
    lastSampleTime = now;

    int16_t rawCode = readADS1115_Raw();
    float measuredVolts = rawCode * LSB_VOLTS;

    Serial.printf("[D3 = %-4s] Raw Code: %6d | Voltage: %6.3f V (%7.1f mV)\n",
                  testPinState ? "HIGH" : "LOW",
                  rawCode,
                  measuredVolts,
                  measuredVolts * 1000.0f);
  }
}