#include "telemetry/voltage_monitor.h"
#include <math.h>

const float VoltageMonitor::kSmoothingAlpha = 0.25f;

VoltageMonitor::VoltageMonitor(IAdcInput& adc)
    : adc_(adc),
      config_(),
      pin_mv_(0),
      smoothed_volts_(0.0f),
      has_sample_(false),
      last_sample_ms_(0),
      has_sampled_once_(false) {}

bool VoltageMonitor::begin(const VoltageConfig& cfg) {
  config_ = cfg;
  has_sampled_once_ = false;
  has_sample_ = false;
  pin_mv_ = 0;
  smoothed_volts_ = 0.0f;
  return adc_.begin(config_.adc_pin);
}

void VoltageMonitor::setConfig(const VoltageConfig& cfg) {
  bool pin_changed = (config_.adc_pin != cfg.adc_pin);
  config_ = cfg;
  if (pin_changed) adc_.begin(config_.adc_pin);
}

void VoltageMonitor::update(uint32_t now_ms) {
  if (!config_.enabled) return;
  if (has_sampled_once_ && (now_ms - last_sample_ms_) < kSampleIntervalMs) return;

  pin_mv_ = adc_.readAveragedMv(kSamples);
  float raw_volts = (pin_mv_ / 1000.0f) * config_.divider_ratio * config_.calibration_factor;

  if (!has_sample_) {
    smoothed_volts_ = raw_volts;   // seed the filter with the first sample
    has_sample_ = true;
  } else {
    smoothed_volts_ = kSmoothingAlpha * raw_volts + (1.0f - kSmoothingAlpha) * smoothed_volts_;
  }

  last_sample_ms_ = now_ms;
  has_sampled_once_ = true;
}

float VoltageMonitor::voltage() const {
  if (!config_.enabled) return 0.0f;
  return smoothed_volts_;
}

uint32_t VoltageMonitor::pinMillivolts() const { return pin_mv_; }

float VoltageMonitor::perCellFromVoltage(float total_volts, uint8_t cell_count) {
  if (cell_count == 0) return 0.0f;
  return total_volts / (float)cell_count;
}

float VoltageMonitor::perCellVoltage() const {
  return perCellFromVoltage(voltage(), config_.cell_count);
}

uint8_t VoltageMonitor::percentFromCellVoltage(float cell_volts) {
  const float kEmptyV = 3.3f;
  const float kFullV = 4.2f;
  float pct = (cell_volts - kEmptyV) / (kFullV - kEmptyV) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (uint8_t)(pct + 0.5f);
}

float VoltageMonitor::calibrate(float actual_voltage) {
  if (config_.calibration_factor <= 0.0f) config_.calibration_factor = 1.0f;
  float raw_volts_without_factor = smoothed_volts_ / config_.calibration_factor;

  if (raw_volts_without_factor < 0.1f) {
    return config_.calibration_factor;   // guard against divide-by-zero / bogus reading
  }

  float new_factor = actual_voltage / raw_volts_without_factor;
  if (new_factor < 0.5f) new_factor = 0.5f;
  if (new_factor > 2.0f) new_factor = 2.0f;

  config_.calibration_factor = new_factor;
  // Re-derive the smoothed volts value under the new factor so voltage() reflects
  // the calibration immediately, without waiting for the next 100 ms sample.
  smoothed_volts_ = raw_volts_without_factor * new_factor;
  return new_factor;
}

bool VoltageMonitor::buildBattery(BatteryTelemetry& out) const {
  if (!config_.enabled) {
    out.voltage_dv = 0;
    out.current_da = 0;
    out.used_capacity_mah = 0;
    out.remaining_percent = 0;
    return false;
  }

  float v = voltage();
  out.voltage_dv = (uint16_t)(v * 10.0f + 0.5f);
  out.current_da = 0;
  out.used_capacity_mah = 0;

  if (config_.cell_count > 0) {
    float cell_v = perCellFromVoltage(v, config_.cell_count);
    out.remaining_percent = percentFromCellVoltage(cell_v);
  } else {
    out.remaining_percent = 0;
  }
  return true;
}
