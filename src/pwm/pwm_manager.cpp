#include "pwm/pwm_manager.h"

PwmManager::PwmManager(IGpioOutput& gpio)
    : gpio_(gpio), cfg_(), failsafe_mode_(FailsafeMode::HOLD_LAST) {
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    current_pulse_us_[i] = RC_PULSE_MID_US;
    current_digital_[i] = false;
    output_active_[i] = false;
    last_write_ms_[i] = 0;
    has_written_[i] = false;
  }
}

uint32_t PwmManager::intervalMsFor(const PWMPinConfig& cfg) const {
  if (cfg.update_rate_hz == 0) return 20;
  uint32_t interval = 1000u / cfg.update_rate_hz;
  if (interval == 0) interval = 1;
  return interval;
}

bool PwmManager::attachOutput(uint8_t idx) {
  const PWMPinConfig& c = cfg_[idx];
  if (c.mode == PwmMode::SERVO) {
    return gpio_.attachPwm(c.pin, ledcChannelFor(idx), c.update_rate_hz == 0 ? 50 : c.update_rate_hz, 16);
  } else if (c.mode == PwmMode::SWITCH) {
    return gpio_.attachDigital(c.pin);
  }
  return true;
}

void PwmManager::detachOutput(uint8_t idx) {
  const PWMPinConfig& c = cfg_[idx];
  if (c.mode == PwmMode::SERVO) {
    gpio_.detach(ledcChannelFor(idx));
  } else if (c.mode == PwmMode::SWITCH) {
    gpio_.writeDigital(c.pin, false);
  }
  output_active_[idx] = false;
}

bool PwmManager::begin(const PWMPinConfig cfg[PWM_PIN_COUNT]) {
  bool all_ok = true;
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    cfg_[i] = cfg[i];
    has_written_[i] = false;
    output_active_[i] = false;
    if (cfg_[i].mode != PwmMode::DISABLED) {
      bool ok = attachOutput(i);
      output_active_[i] = ok;
      if (!ok) all_ok = false;
    }
  }
  return all_ok;
}

void PwmManager::setConfig(uint8_t idx, const PWMPinConfig& cfg) {
  if (idx >= PWM_PIN_COUNT) return;
  const PWMPinConfig& old = cfg_[idx];
  bool mode_changed = (old.mode != cfg.mode);
  bool pin_changed = (old.pin != cfg.pin);

  if (mode_changed || pin_changed) {
    if (old.mode != PwmMode::DISABLED) detachOutput(idx);
    cfg_[idx] = cfg;
    has_written_[idx] = false;
    if (cfg_[idx].mode != PwmMode::DISABLED) {
      output_active_[idx] = attachOutput(idx);
    } else {
      output_active_[idx] = false;
    }
  } else {
    cfg_[idx] = cfg;
  }
}

void PwmManager::setFailsafeMode(FailsafeMode mode) { failsafe_mode_ = mode; }

void PwmManager::writeOutput(uint8_t idx, uint16_t pulse_us, bool digital_level, bool is_switch) {
  current_pulse_us_[idx] = pulse_us;
  current_digital_[idx] = digital_level;
  if (is_switch) {
    gpio_.writeDigital(cfg_[idx].pin, digital_level);
  } else {
    gpio_.writePulseUs(ledcChannelFor(idx), pulse_us);
  }
}

void PwmManager::update(uint32_t now_ms, const RCFrame& frame, bool link_valid) {
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    const PWMPinConfig& c = cfg_[i];
    if (c.mode == PwmMode::DISABLED) continue;

    if (!link_valid && failsafe_mode_ == FailsafeMode::STOP_PWM) {
      if (output_active_[i]) detachOutput(i);
      continue;
    }
    if (!output_active_[i]) {
      // Recovering from a prior STOP_PWM detach.
      if (!attachOutput(i)) continue;
      output_active_[i] = true;
      has_written_[i] = false;
    }

    if (!link_valid && failsafe_mode_ == FailsafeMode::HOLD_LAST) {
      // Leave last written value untouched.
      continue;
    }

    uint32_t interval = intervalMsFor(c);
    if (has_written_[i] && (now_ms - last_write_ms_[i]) < interval) continue;

    uint16_t source_value;
    if (!link_valid && failsafe_mode_ == FailsafeMode::FAILSAFE_VALUES) {
      source_value = c.failsafe_us;
    } else {
      source_value = frame.channels[c.source_channel];
    }

    if (c.mode == PwmMode::SERVO) {
      int32_t out = source_value;
      if (c.invert) {
        out = (int32_t)RC_PULSE_MIN_US + (int32_t)RC_PULSE_MAX_US - out;
      }
      uint16_t pulse = clampPulseUs(out);
      writeOutput(i, pulse, current_digital_[i], false);
    } else if (c.mode == PwmMode::SWITCH) {
      bool on = (source_value >= c.switch_threshold_us);
      bool level = c.switch_active_high ? on : !on;
      writeOutput(i, current_pulse_us_[i], level, true);
    }

    last_write_ms_[i] = now_ms;
    has_written_[i] = true;
  }
}

uint16_t PwmManager::currentPulseUs(uint8_t idx) const {
  if (idx >= PWM_PIN_COUNT) return RC_PULSE_MID_US;
  return current_pulse_us_[idx];
}

bool PwmManager::currentDigital(uint8_t idx) const {
  if (idx >= PWM_PIN_COUNT) return false;
  return current_digital_[idx];
}

bool PwmManager::outputActive(uint8_t idx) const {
  if (idx >= PWM_PIN_COUNT) return false;
  return output_active_[idx];
}
