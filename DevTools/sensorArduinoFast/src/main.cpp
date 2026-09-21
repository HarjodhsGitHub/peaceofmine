#include <Arduino.h>
#include <avr/interrupt.h>

namespace {

// The ATmega328P ADC needs 13 ADC clocks per conversion.  With a /16
// prescaler at 16 MHz it therefore free-runs at about 76.9 kS/s.  ADLAR lets
// the ISR read only ADCH, which keeps each conversion handling as short as
// possible.
constexpr uint8_t kAdcMidpoint = 128;

// Both quantities use Q8 fixed point.  The DC tracker is intentionally much
// slower than the envelope tracker so the 60 kHz signal is not absorbed as DC.
volatile int32_t dcQ8 = static_cast<int32_t>(kAdcMidpoint) << 8;
volatile uint16_t envelopeQ8 = 0;

ISR(ADC_vect) {
  const int32_t sampleQ8 = static_cast<int32_t>(ADCH) << 8;
  const int32_t dcError = sampleQ8 - dcQ8;
  dcQ8 += dcError >> 12;  // About a 3 Hz DC-tracking cutoff.

  const int32_t signedMagnitude = sampleQ8 - dcQ8;
  const uint16_t magnitudeQ8 = signedMagnitude < 0
      ? static_cast<uint16_t>(-signedMagnitude)
      : static_cast<uint16_t>(signedMagnitude);

  // Rectify, then apply a one-pole low-pass filter (~48 Hz envelope cutoff).
  envelopeQ8 += (static_cast<int32_t>(magnitudeQ8) - envelopeQ8) >> 8;
}

uint8_t readPeakAmplitude() {
  noInterrupts();
  const uint16_t averageRectifiedQ8 = envelopeQ8;
  interrupts();

  // For a sine wave: peak = mean(abs(sine)) * pi / 2.
  // 402 / 65536 is pi / (2 * 256), converting Q8 to ADC counts.
  const uint16_t peak =
      (static_cast<uint32_t>(averageRectifiedQ8) * 402U + 32768U) >> 16;
  return peak > 255U ? 255U : static_cast<uint8_t>(peak);
}

void startFastAdcOnA0() {
  ADMUX = _BV(REFS0) | _BV(ADLAR);  // AVcc reference, left-adjusted ADC0/A0.
  ADCSRB = 0;                       // Free-running trigger source.
  DIDR0 = _BV(ADC0D);               // Disable A0's digital input buffer.
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2);
  ADCSRA |= _BV(ADSC);              // Start the first conversion.
}

}  // namespace

void setup() {
  Serial.begin(115200);
  startFastAdcOnA0();
}

void loop() {
  // Printing is deliberately infrequent; sampling and filtering remain in the
  // ADC ISR at full speed.  Each line is the estimated sine peak in 8-bit ADC
  // counts (0..255).
  static uint32_t previousPrintMs = 0;
  const uint32_t now = millis();
  if (now - previousPrintMs >= 50) {
    previousPrintMs = now;
    Serial.println(readPeakAmplitude());
  }
}
