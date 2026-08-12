#include <unity.h>
#include "hal/adc_input.h"
#include "telemetry/voltage_monitor.h"
#include "config/config_types.h"

static VoltageConfig makeCfg(bool enabled, uint8_t cell_count, float ratio, float factor) {
  VoltageConfig c;
  c.enabled = enabled;
  c.adc_pin = 33;
  c.divider_ratio = ratio;
  c.calibration_factor = factor;
  c.telemetry_override = true;
  c.cell_count = cell_count;
  return c;
}

void test_known_mv_ratio_factor_gives_expected_volts(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 3, 11.0f, 1.0f));
  adc.setMv(1000);   // 1.0 V at the pin

  vm.update(0);
  // First sample seeds the exponential filter directly (see implementation note),
  // so after one update the smoothed value equals the raw computed value.
  float expected = (1000 / 1000.0f) * 11.0f * 1.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.05f, expected, vm.voltage());
}

void test_sampling_respects_100ms_interval(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 3, 11.0f, 1.0f));
  adc.setMv(1000);

  vm.update(0);
  uint32_t after_first = adc.readCount();

  vm.update(50);    // within the 100 ms window
  TEST_ASSERT_EQUAL_UINT32(after_first, adc.readCount());

  vm.update(100);   // interval elapsed
  TEST_ASSERT_TRUE(adc.readCount() > after_first);
}

void test_calibrate_computes_factor(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 3, 10.0f, 1.0f));
  adc.setMv(1200);   // raw_volts_without_factor = 1.2 * 10.0 = 12.0 V
  vm.update(0);

  float new_factor = vm.calibrate(12.6f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.6f / 12.0f, new_factor);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, new_factor, vm.config().calibration_factor);
}

void test_calibrate_clamps_both_ends(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 3, 10.0f, 1.0f));
  adc.setMv(1200);   // raw_volts_without_factor = 12.0 V
  vm.update(0);

  float too_high = vm.calibrate(30.0f);   // would need factor 2.5
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, too_high);

  vm.setConfig(makeCfg(true, 3, 10.0f, 1.0f));
  vm.update(0);
  float too_low = vm.calibrate(1.0f);     // would need factor ~0.083
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, too_low);
}

void test_calibrate_near_zero_reading_is_noop(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 3, 10.0f, 1.23f));
  adc.setMv(1);    // 0.01 V raw -> below the 0.1 V guard
  vm.update(0);

  float result = vm.calibrate(12.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.23f, result);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.23f, vm.config().calibration_factor);
}

void test_build_battery_3s_11_1v(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 3, 1.0f, 1.0f));
  adc.setMv(11100);
  vm.update(0);

  BatteryTelemetry batt;
  TEST_ASSERT_TRUE(vm.buildBattery(batt));
  TEST_ASSERT_EQUAL_UINT16(111, batt.voltage_dv);
  TEST_ASSERT_EQUAL_UINT16(0, batt.current_da);
  TEST_ASSERT_EQUAL_UINT32(0, batt.used_capacity_mah);
  // per-cell = 3.7 V -> between the 0%@3.3V and 100%@4.2V LiPo curve
  float expected_pct = (3.7f - 3.3f) / (4.2f - 3.3f) * 100.0f;
  TEST_ASSERT_UINT8_WITHIN(2, (uint8_t)expected_pct, batt.remaining_percent);
}

void test_build_battery_3s_12_6v_full(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 3, 1.0f, 1.0f));
  adc.setMv(12600);
  vm.update(0);

  BatteryTelemetry batt;
  TEST_ASSERT_TRUE(vm.buildBattery(batt));
  TEST_ASSERT_EQUAL_UINT16(126, batt.voltage_dv);
  TEST_ASSERT_EQUAL_UINT8(100, batt.remaining_percent);
}

void test_disabled_config_returns_zero(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(false, 3, 1.0f, 1.0f));
  adc.setMv(12600);
  vm.update(0);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, vm.voltage());
  BatteryTelemetry batt;
  TEST_ASSERT_FALSE(vm.buildBattery(batt));
}

void test_per_cell_voltage_4s(void) {
  MockAdcInput adc;
  VoltageMonitor vm(adc);
  vm.begin(makeCfg(true, 4, 1.0f, 1.0f));
  adc.setMv(16800);   // 16.8 V total, 4S -> 4.2 V/cell
  vm.update(0);

  TEST_ASSERT_FLOAT_WITHIN(0.02f, 4.2f, vm.perCellVoltage());
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_known_mv_ratio_factor_gives_expected_volts);
  RUN_TEST(test_sampling_respects_100ms_interval);
  RUN_TEST(test_calibrate_computes_factor);
  RUN_TEST(test_calibrate_clamps_both_ends);
  RUN_TEST(test_calibrate_near_zero_reading_is_noop);
  RUN_TEST(test_build_battery_3s_11_1v);
  RUN_TEST(test_build_battery_3s_12_6v_full);
  RUN_TEST(test_disabled_config_returns_zero);
  RUN_TEST(test_per_cell_voltage_4s);
  return UNITY_END();
}
