#pragma once
#include <stdint.h>
#include "hal/gpio_output.h"
#include "protocols/protocol_types.h"
#include "config/config_types.h"

class PwmManager {
 public:
  explicit PwmManager(IGpioOutput& gpio);

  bool begin(const PWMPinConfig cfg[PWM_PIN_COUNT]);
  void setConfig(uint8_t idx, const PWMPinConfig& cfg);
  void update(uint32_t now_ms, const RCFrame& frame, bool link_valid);
  void setFailsafeMode(FailsafeMode mode);

  uint16_t currentPulseUs(uint8_t idx) const;
  bool currentDigital(uint8_t idx) const;
  bool outputActive(uint8_t idx) const;

  static uint8_t ledcChannelFor(uint8_t idx) { return idx; }

 private:
  bool attachOutput(uint8_t idx);
  void detachOutput(uint8_t idx);
  void writeOutput(uint8_t idx, uint16_t pulse_us, bool digital_level, bool is_switch);
  uint32_t intervalMsFor(const PWMPinConfig& cfg) const;

  IGpioOutput& gpio_;
  PWMPinConfig cfg_[PWM_PIN_COUNT];

  uint16_t current_pulse_us_[PWM_PIN_COUNT];
  bool current_digital_[PWM_PIN_COUNT];
  bool output_active_[PWM_PIN_COUNT];
  uint32_t last_write_ms_[PWM_PIN_COUNT];
  bool has_written_[PWM_PIN_COUNT];

  FailsafeMode failsafe_mode_;
};
