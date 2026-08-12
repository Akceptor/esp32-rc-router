#pragma once
#include <stdint.h>
#include "hal/gpio_output.h"

enum class LedPattern : uint8_t {
  OFF = 0, SOLID = 1, SLOW_BLINK = 2, FAST_BLINK = 3, DOUBLE_BLINK = 4, TRIPLE_BLINK = 5
};

class StatusLed {
 public:
  StatusLed(IGpioOutput& gpio, uint8_t pin, bool active_high);
  bool begin();
  void setPattern(LedPattern pattern);
  LedPattern pattern() const;
  void update(uint32_t now_ms);
  bool level() const;

 private:
  struct Phase { uint16_t duration_ms; bool on; };
  static const uint8_t kMaxPhases = 6;

  void loadPhases();
  void applyLevel(bool on);

  IGpioOutput& gpio_;
  uint8_t pin_;
  bool active_high_;
  LedPattern pattern_;
  bool level_;
  Phase phases_[kMaxPhases];
  uint8_t phase_count_;
  uint32_t period_ms_;
};
