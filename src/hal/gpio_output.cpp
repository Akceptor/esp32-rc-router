#include "hal/gpio_output.h"

#include <string.h>

#if defined(ARDUINO)
#include <Arduino.h>

Esp32GpioOutput::Esp32GpioOutput() {
  memset(freq_hz_, 0, sizeof(freq_hz_));
  memset(resolution_bits_, 0, sizeof(resolution_bits_));
  memset(attached_, 0, sizeof(attached_));
  memset(pin_, 0, sizeof(pin_));
}

bool Esp32GpioOutput::attachPwm(uint8_t pin, uint8_t ledc_channel, uint32_t freq_hz,
                                 uint8_t resolution_bits) {
  if (ledc_channel >= kMaxChannels) {
    return false;
  }
  // Arduino-ESP32 v3.x replaced ledcSetup()+ledcAttachPin() with a single pin+channel call.
  ledcAttachChannel(pin, freq_hz, resolution_bits, ledc_channel);
  freq_hz_[ledc_channel] = freq_hz;
  resolution_bits_[ledc_channel] = resolution_bits;
  attached_[ledc_channel] = true;
  pin_[ledc_channel] = pin;
  return true;
}

bool Esp32GpioOutput::attachDigital(uint8_t pin) {
  pinMode(pin, OUTPUT);
  return true;
}

void Esp32GpioOutput::writePulseUs(uint8_t ledc_channel, uint16_t pulse_us) {
  if (ledc_channel >= kMaxChannels || !attached_[ledc_channel]) {
    return;
  }
  uint32_t freq = freq_hz_[ledc_channel] == 0 ? 50 : freq_hz_[ledc_channel];
  uint8_t bits = resolution_bits_[ledc_channel] == 0 ? 16 : resolution_bits_[ledc_channel];
  uint32_t period_us = 1000000UL / freq;
  uint32_t max_duty = (1UL << bits);
  uint32_t duty = static_cast<uint32_t>((static_cast<uint64_t>(pulse_us) * max_duty) / period_us);
  if (duty >= max_duty) {
    duty = max_duty - 1;
  }
  ledcWriteChannel(ledc_channel, duty);
}

void Esp32GpioOutput::writeDigital(uint8_t pin, bool level) {
  digitalWrite(pin, level ? HIGH : LOW);
}

void Esp32GpioOutput::detach(uint8_t ledc_channel) {
  if (ledc_channel >= kMaxChannels) {
    return;
  }
  ledcDetach(pin_[ledc_channel]);
  attached_[ledc_channel] = false;
}

#endif  // defined(ARDUINO)

MockGpioOutput::MockGpioOutput() {
  memset(pulse_us_, 0, sizeof(pulse_us_));
  memset(pwm_attached_, 0, sizeof(pwm_attached_));
  memset(pwm_freq_, 0, sizeof(pwm_freq_));
  memset(write_count_, 0, sizeof(write_count_));
  memset(digital_level_, 0, sizeof(digital_level_));
  memset(digital_attached_, 0, sizeof(digital_attached_));
}

bool MockGpioOutput::attachPwm(uint8_t pin, uint8_t ledc_channel, uint32_t freq_hz,
                                uint8_t resolution_bits) {
  (void)pin;
  (void)resolution_bits;
  if (ledc_channel >= kMaxChannels) {
    return false;
  }
  pwm_attached_[ledc_channel] = true;
  pwm_freq_[ledc_channel] = freq_hz;
  return true;
}

bool MockGpioOutput::attachDigital(uint8_t pin) {
  if (pin >= kMaxPins) {
    return false;
  }
  digital_attached_[pin] = true;
  return true;
}

void MockGpioOutput::writePulseUs(uint8_t ledc_channel, uint16_t pulse_us) {
  if (ledc_channel >= kMaxChannels) {
    return;
  }
  pulse_us_[ledc_channel] = pulse_us;
  write_count_[ledc_channel]++;
}

void MockGpioOutput::writeDigital(uint8_t pin, bool level) {
  if (pin >= kMaxPins) {
    return;
  }
  digital_level_[pin] = level;
}

void MockGpioOutput::detach(uint8_t ledc_channel) {
  if (ledc_channel >= kMaxChannels) {
    return;
  }
  pwm_attached_[ledc_channel] = false;
}

uint16_t MockGpioOutput::lastPulseUs(uint8_t ledc_channel) const {
  return ledc_channel < kMaxChannels ? pulse_us_[ledc_channel] : 0;
}

bool MockGpioOutput::lastDigital(uint8_t pin) const {
  return pin < kMaxPins ? digital_level_[pin] : false;
}

bool MockGpioOutput::pwmAttached(uint8_t ledc_channel) const {
  return ledc_channel < kMaxChannels ? pwm_attached_[ledc_channel] : false;
}

uint32_t MockGpioOutput::pwmFreq(uint8_t ledc_channel) const {
  return ledc_channel < kMaxChannels ? pwm_freq_[ledc_channel] : 0;
}

uint32_t MockGpioOutput::writeCount(uint8_t ledc_channel) const {
  return ledc_channel < kMaxChannels ? write_count_[ledc_channel] : 0;
}
