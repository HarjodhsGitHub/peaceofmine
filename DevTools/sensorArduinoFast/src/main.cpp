#include <Arduino.h>
#include <ArduinoJson.h>
#include <avr/interrupt.h>

namespace {

// The ATmega328P ADC needs 13 ADC clocks per conversion.  With a /16
// prescaler at 16 MHz it therefore free-runs at about 76.9 kS/s.  ADLAR lets
// the ISR read only ADCH, which keeps each conversion handling as short as
// possible.
constexpr uint8_t kAdcMidpoint = 128;

// Both quantities use Q8 fixed point.  The DC tracker is intentionally much
// slower than the envelope tracker so the 60 kHz signal is not absorbed as DC.
uint16_t dcQ8 = static_cast<uint16_t>(kAdcMidpoint) << 8;
volatile uint16_t envelopeQ8 = 0;

ISR(ADC_vect) {
  const uint16_t sampleQ8 = static_cast<uint16_t>(ADCH) << 8;
  // Unsigned differences avoid costly 32-bit shifts on the 8-bit AVR.
  // Round downward updates up to preserve signed arithmetic-shift behavior.
  if (sampleQ8 >= dcQ8) {
    dcQ8 += (sampleQ8 - dcQ8) >> 12;
  } else {
    dcQ8 -= ((dcQ8 - sampleQ8 - 1U) >> 12) + 1U;
  }
  const uint16_t magnitudeQ8 = sampleQ8 >= dcQ8
      ? sampleQ8 - dcQ8 : dcQ8 - sampleQ8;
  const uint16_t envelope = envelopeQ8;
  if (magnitudeQ8 >= envelope) {
    envelopeQ8 = envelope + ((magnitudeQ8 - envelope) >> 8);
  } else {
    envelopeQ8 = envelope - (((envelope - magnitudeQ8 - 1U) >> 8) + 1U);
  }
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
  // Reuse the document; serialization stays outside the ADC interrupt.
  static JsonDocument message;
  static uint32_t sequence = 0;
  static uint32_t previousPrintMs = 0;
  const uint32_t now = millis();
  if (now - previousPrintMs >= 10) {
    previousPrintMs = now;
    message["v"] = 1;
    message["seq"] = sequence++;
    message["uptime_ms"] = now;
    message["amplitude_adc"] = readPeakAmplitude();
    if (!message.overflowed()) {
      serializeJson(message, Serial);
      Serial.println();
    }
  }
}
