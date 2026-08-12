#include "hal/status_led.h"

StatusLed::StatusLed(IGpioOutput& gpio, uint8_t pin, bool active_high)
    : gpio_(gpio),
      pin_(pin),
      active_high_(active_high),
      pattern_(LedPattern::OFF),
      level_(false),
      phase_count_(0),
      period_ms_(0) {}

bool StatusLed::begin() {
  bool ok = gpio_.attachDigital(pin_);
  loadPhases();
  applyLevel(false);
  return ok;
}

void StatusLed::setPattern(LedPattern pattern) {
  pattern_ = pattern;
  loadPhases();
}

LedPattern StatusLed::pattern() const { return pattern_; }

void StatusLed::loadPhases() {
  phase_count_ = 0;
  period_ms_ = 0;

  switch (pattern_) {
    case LedPattern::OFF:
    case LedPattern::SOLID:
      // Handled directly in update(); no phase table needed.
      break;

    case LedPattern::SLOW_BLINK:
      // 1000 ms period, 50% duty.
      phases_[0] = {500, true};
      phases_[1] = {500, false};
      phase_count_ = 2;
      break;

    case LedPattern::FAST_BLINK:
      // 200 ms period, 50% duty.
      phases_[0] = {100, true};
      phases_[1] = {100, false};
      phase_count_ = 2;
      break;

    case LedPattern::DOUBLE_BLINK:
      // Two 80 ms pulses, 800 ms of cumulative off-time (80 between the
      // pulses + 720 idle), 960 ms total period, 2 rising edges/period.
      phases_[0] = {80, true};
      phases_[1] = {80, false};
      phases_[2] = {80, true};
      phases_[3] = {720, false};
      phase_count_ = 4;
      break;

    case LedPattern::TRIPLE_BLINK:
      // Three 80 ms pulses, 800 ms of cumulative off-time (2x80 between
      // pulses + 640 idle), 1040 ms total period, 3 rising edges/period.
      phases_[0] = {80, true};
      phases_[1] = {80, false};
      phases_[2] = {80, true};
      phases_[3] = {80, false};
      phases_[4] = {80, true};
      phases_[5] = {640, false};
      phase_count_ = 6;
      break;
  }

  for (uint8_t i = 0; i < phase_count_; i++) {
    period_ms_ += phases_[i].duration_ms;
  }
}

void StatusLed::update(uint32_t now_ms) {
  if (pattern_ == LedPattern::OFF) {
    applyLevel(false);
    return;
  }
  if (pattern_ == LedPattern::SOLID) {
    applyLevel(true);
    return;
  }
  if (period_ms_ == 0 || phase_count_ == 0) {
    applyLevel(false);
    return;
  }

  uint32_t phase_elapsed = now_ms % period_ms_;
  uint32_t acc = 0;
  bool on = false;
  for (uint8_t i = 0; i < phase_count_; i++) {
    acc += phases_[i].duration_ms;
    if (phase_elapsed < acc) {
      on = phases_[i].on;
      break;
    }
  }
  applyLevel(on);
}

bool StatusLed::level() const { return level_; }

void StatusLed::applyLevel(bool on) {
  level_ = on;
  bool physical = active_high_ ? on : !on;
  gpio_.writeDigital(pin_, physical);
}
