#include "config/config_types.h"

#include <stddef.h>
#include <string.h>

namespace {

bool clampU8(uint8_t& value, uint8_t lo, uint8_t hi) {
  if (value < lo) { value = lo; return true; }
  if (value > hi) { value = hi; return true; }
  return false;
}

bool clampU16(uint16_t& value, uint16_t lo, uint16_t hi) {
  if (value < lo) { value = lo; return true; }
  if (value > hi) { value = hi; return true; }
  return false;
}

bool clampF32(float& value, float lo, float hi) {
  if (value < lo) { value = lo; return true; }
  if (value > hi) { value = hi; return true; }
  return false;
}

bool ensureTerminated(char* str, size_t cap) {
  if (cap == 0) return false;
  if (str[cap - 1] != '\0') {
    str[cap - 1] = '\0';
    return true;
  }
  return false;
}

}  // namespace

void configLoadDefaults(RouterConfig& cfg) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.version = CONFIG_VERSION;

  cfg.receivers[0].enabled = true;
  cfg.receivers[0].protocol = ProtocolType::CRSF;
  cfg.receivers[0].priority = 0;
  cfg.receivers[0].baud = 420000;
  cfg.receivers[0].rx_pin = 16;
  cfg.receivers[0].tx_pin = 17;
  cfg.receivers[0].inverted = false;

  cfg.receivers[1].enabled = true;
  cfg.receivers[1].protocol = ProtocolType::SBUS;
  cfg.receivers[1].priority = 1;
  cfg.receivers[1].baud = 100000;
  cfg.receivers[1].rx_pin = 18;
  cfg.receivers[1].tx_pin = 19;
  cfg.receivers[1].inverted = true;

  cfg.selection.rssi_threshold_percent = 50;
  cfg.selection.lq_threshold_percent = 50;
  cfg.selection.hysteresis_percent = 10;
  cfg.selection.switch_delay_ms = 200;
  cfg.selection.min_active_time_ms = 500;
  cfg.selection.link_timeout_ms = 300;

  cfg.output.protocol = ProtocolType::CRSF;
  cfg.output.baud = 420000;
  cfg.output.tx_pin = 23;
  cfg.output.rx_pin = 22;
  cfg.output.inverted = false;
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    cfg.output.channel_map[i] = i;
  }

  static const uint8_t kPwmPins[PWM_PIN_COUNT] = {25, 26, 27, 32};
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    cfg.pwm[i].mode = PwmMode::SERVO;
    cfg.pwm[i].pin = kPwmPins[i];
    cfg.pwm[i].source_channel = i;
    cfg.pwm[i].update_rate_hz = 50;
    cfg.pwm[i].invert = false;
    cfg.pwm[i].switch_threshold_us = RC_PULSE_MID_US;
    cfg.pwm[i].switch_active_high = true;
    cfg.pwm[i].failsafe_us = RC_PULSE_MID_US;
  }

  cfg.voltage.enabled = false;
  cfg.voltage.adc_pin = 33;
  cfg.voltage.divider_ratio = 11.0f;
  cfg.voltage.calibration_factor = 1.0f;
  cfg.voltage.telemetry_override = false;
  cfg.voltage.cell_count = 3;

  strncpy(cfg.network.ssid, "RC-Router", sizeof(cfg.network.ssid) - 1);
  cfg.network.password[0] = '\0';
  cfg.network.ap_mode = true;
  cfg.network.use_dhcp = true;
  cfg.network.static_ip = 0;
  cfg.network.gateway = 0;
  cfg.network.netmask = 0;
  strncpy(cfg.network.hostname, "rc-router", sizeof(cfg.network.hostname) - 1);

  cfg.system.log_level = 2;  // LogLevel::INFO
  cfg.system.serial_console = true;
  cfg.system.failsafe_mode = FailsafeMode::HOLD_LAST;
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    cfg.system.failsafe_channels[i] = RC_PULSE_MID_US;
  }

  cfg.crc32 = configCrc32(cfg);
}

uint32_t configCrc32(const RouterConfig& cfg) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&cfg);
  size_t len = offsetof(RouterConfig, crc32);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; bit++) {
      uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

bool configValidate(RouterConfig& cfg) {
  bool changed = false;

  changed |= clampU8(cfg.selection.rssi_threshold_percent, 0, 100);
  changed |= clampU8(cfg.selection.lq_threshold_percent, 0, 100);
  changed |= clampU8(cfg.selection.hysteresis_percent, 0, 100);
  changed |= clampU16(cfg.selection.switch_delay_ms, 0, 5000);
  changed |= clampU16(cfg.selection.min_active_time_ms, 0, 10000);
  changed |= clampU16(cfg.selection.link_timeout_ms, 50, 2000);

  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    if (cfg.output.channel_map[i] >= RC_CHANNEL_COUNT) {
      cfg.output.channel_map[i] = i;
      changed = true;
    }
  }

  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    changed |= clampU16(cfg.pwm[i].update_rate_hz, 50, 333);
    if (cfg.pwm[i].source_channel >= RC_CHANNEL_COUNT) {
      cfg.pwm[i].source_channel = 0;
      changed = true;
    }
    uint16_t clamped = clampPulseUs(cfg.pwm[i].failsafe_us);
    if (clamped != cfg.pwm[i].failsafe_us) {
      cfg.pwm[i].failsafe_us = clamped;
      changed = true;
    }
  }

  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    uint16_t clamped = clampPulseUs(cfg.system.failsafe_channels[i]);
    if (clamped != cfg.system.failsafe_channels[i]) {
      cfg.system.failsafe_channels[i] = clamped;
      changed = true;
    }
  }

  changed |= clampF32(cfg.voltage.divider_ratio, 1.0f, 100.0f);
  changed |= clampF32(cfg.voltage.calibration_factor, 0.5f, 2.0f);
  changed |= clampU8(cfg.voltage.cell_count, 1, 12);

  changed |= ensureTerminated(cfg.network.ssid, sizeof(cfg.network.ssid));
  changed |= ensureTerminated(cfg.network.password, sizeof(cfg.network.password));
  changed |= ensureTerminated(cfg.network.hostname, sizeof(cfg.network.hostname));

  return !changed;
}
