#pragma once
#include <stdint.h>
#include "hal/adc_input.h"
#include "protocols/protocol_types.h"
#include "config/config_types.h"

class VoltageMonitor {
 public:
  explicit VoltageMonitor(IAdcInput& adc);

  bool begin(const VoltageConfig& cfg);
  void setConfig(const VoltageConfig& cfg);

  // Samples at a 100 ms interval. Each sample is `readAveragedMv(16)` (a 16-sample
  // hardware-level average taken by the ADC HAL in one call), producing a raw volts
  // value. On top of that, an exponential smoothing filter (alpha = 0.25) is applied
  // to the *volts* value across successive 100 ms samples, so short-lived ADC noise
  // and load transients settle out over roughly half a second without adding latency
  // comparable to a long moving average.
  void update(uint32_t now_ms);

  float voltage() const;             // (pinMillivolts()/1000) * divider_ratio * calibration_factor, smoothed
  uint32_t pinMillivolts() const;
  float perCellVoltage() const;

  // new_factor = actual / (raw_volts_without_factor); raw_volts_without_factor is the
  // measured voltage with calibration_factor divided back out, i.e. what the pin+divider
  // alone imply. The result is clamped to [0.5, 2.0] and written into config_.
  // Guards against divide-by-zero: if the measured (uncalibrated) volts are below
  // 0.1 V, the call is a no-op and the existing factor is returned unchanged.
  // The caller (WebServerManager) is responsible for copying the returned factor
  // into RouterConfig.voltage.calibration_factor and calling ConfigManager::save();
  // this class only updates its own in-memory config_ copy.
  float calibrate(float actual_voltage);

  bool telemetryOverrideEnabled() const { return config_.telemetry_override; }
  bool buildBattery(BatteryTelemetry& out) const;
  const VoltageConfig& config() const { return config_; }

 private:
  static float perCellFromVoltage(float total_volts, uint8_t cell_count);
  static uint8_t percentFromCellVoltage(float cell_volts);

  IAdcInput& adc_;
  VoltageConfig config_;
  uint32_t pin_mv_;
  float smoothed_volts_;
  bool has_sample_;
  uint32_t last_sample_ms_;
  bool has_sampled_once_;

  static const float kSmoothingAlpha;
  static const uint8_t kSamples = 16;
  static const uint32_t kSampleIntervalMs = 100;
};
