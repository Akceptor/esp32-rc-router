#include "hal/adc_input.h"

#if defined(ARDUINO)
#include <Arduino.h>

Esp32AdcInput::Esp32AdcInput() : pin_(0) {}

bool Esp32AdcInput::begin(uint8_t pin) {
  pin_ = pin;
  pinMode(pin_, INPUT);
  return true;
}

uint16_t Esp32AdcInput::readRaw() {
  return static_cast<uint16_t>(analogRead(pin_));
}

uint32_t Esp32AdcInput::readAveragedMv(uint8_t samples) {
  if (samples == 0) {
    samples = 1;
  }
  uint64_t total = 0;
  for (uint8_t i = 0; i < samples; i++) {
    total += analogReadMilliVolts(pin_);
  }
  return static_cast<uint32_t>(total / samples);
}

#endif  // defined(ARDUINO)

MockAdcInput::MockAdcInput() : pin_(0), raw_(0), mv_(0), read_count_(0) {}

bool MockAdcInput::begin(uint8_t pin) {
  pin_ = pin;
  return true;
}

uint16_t MockAdcInput::readRaw() {
  read_count_++;
  return raw_;
}

uint32_t MockAdcInput::readAveragedMv(uint8_t samples) {
  (void)samples;
  read_count_++;
  return mv_;
}

void MockAdcInput::setMv(uint32_t mv) { mv_ = mv; }
void MockAdcInput::setRaw(uint16_t raw) { raw_ = raw; }
uint32_t MockAdcInput::readCount() const { return read_count_; }
