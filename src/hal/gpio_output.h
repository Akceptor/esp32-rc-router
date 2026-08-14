#pragma once
#include <stdint.h>

class IGpioOutput {
 public:
  virtual ~IGpioOutput() {}
  virtual bool attachPwm(uint8_t pin, uint8_t ledc_channel, uint32_t freq_hz,
                         uint8_t resolution_bits) = 0;
  virtual bool attachDigital(uint8_t pin) = 0;
  virtual void writePulseUs(uint8_t ledc_channel, uint16_t pulse_us) = 0;
  virtual void writeDigital(uint8_t pin, bool level) = 0;
  virtual void detach(uint8_t ledc_channel) = 0;
};

#if defined(ARDUINO)
class Esp32GpioOutput : public IGpioOutput { /* LEDC impl */
 public:
  Esp32GpioOutput();
  bool attachPwm(uint8_t pin, uint8_t ledc_channel, uint32_t freq_hz,
                 uint8_t resolution_bits) override;
  bool attachDigital(uint8_t pin) override;
  void writePulseUs(uint8_t ledc_channel, uint16_t pulse_us) override;
  void writeDigital(uint8_t pin, bool level) override;
  void detach(uint8_t ledc_channel) override;

 private:
  static const uint8_t kMaxChannels = 16;
  uint32_t freq_hz_[kMaxChannels];
  uint8_t resolution_bits_[kMaxChannels];
  bool attached_[kMaxChannels];
  uint8_t pin_[kMaxChannels];  // remembers which GPIO each ledc_channel is attached to, since the
                               // Arduino-ESP32 v3.x LEDC API (ledcAttachChannel/ledcWriteChannel/
                               // ledcDetach) addresses detach by pin, not by channel.
};
#endif

class MockGpioOutput : public IGpioOutput {
 public:
  MockGpioOutput();
  bool attachPwm(uint8_t pin, uint8_t ledc_channel, uint32_t freq_hz,
                 uint8_t resolution_bits) override;
  bool attachDigital(uint8_t pin) override;
  void writePulseUs(uint8_t ledc_channel, uint16_t pulse_us) override;
  void writeDigital(uint8_t pin, bool level) override;
  void detach(uint8_t ledc_channel) override;

  uint16_t lastPulseUs(uint8_t ledc_channel) const;
  bool lastDigital(uint8_t pin) const;
  bool pwmAttached(uint8_t ledc_channel) const;
  uint32_t pwmFreq(uint8_t ledc_channel) const;
  uint32_t writeCount(uint8_t ledc_channel) const;

 private:
  static const uint8_t kMaxChannels = 16;
  static const uint8_t kMaxPins = 40;
  uint16_t pulse_us_[kMaxChannels];
  bool pwm_attached_[kMaxChannels];
  uint32_t pwm_freq_[kMaxChannels];
  uint32_t write_count_[kMaxChannels];
  bool digital_level_[kMaxPins];
  bool digital_attached_[kMaxPins];
};
