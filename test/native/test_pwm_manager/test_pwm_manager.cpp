#include <unity.h>
#include "hal/gpio_output.h"
#include "pwm/pwm_manager.h"
#include "protocols/protocol_types.h"
#include "config/config_types.h"

static PWMPinConfig servoCfg(uint8_t pin, uint8_t src_ch, bool invert, uint16_t rate_hz) {
  PWMPinConfig c;
  c.mode = PwmMode::SERVO;
  c.pin = pin;
  c.source_channel = src_ch;
  c.update_rate_hz = rate_hz;
  c.invert = invert;
  c.switch_threshold_us = 1500;
  c.switch_active_high = true;
  c.failsafe_us = 1500;
  return c;
}

static PWMPinConfig switchCfg(uint8_t pin, uint8_t src_ch, uint16_t threshold, bool active_high) {
  PWMPinConfig c;
  c.mode = PwmMode::SWITCH;
  c.pin = pin;
  c.source_channel = src_ch;
  c.update_rate_hz = 50;
  c.invert = false;
  c.switch_threshold_us = threshold;
  c.switch_active_high = active_high;
  c.failsafe_us = 900;
  return c;
}

static RCFrame frameWith(uint8_t ch, uint16_t val) {
  RCFrame f;
  rcFrameInit(f);
  f.channels[ch] = val;
  f.valid = true;
  return f;
}

void test_servo_output_writes_mapped_channel(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = servoCfg(25, 2, false, 50);
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);

  RCFrame f = frameWith(2, 1700);
  pwm.update(0, f, true);

  TEST_ASSERT_EQUAL_UINT16(1700, pwm.currentPulseUs(0));
  TEST_ASSERT_EQUAL_UINT16(1700, gpio.lastPulseUs(PwmManager::ledcChannelFor(0)));
}

void test_invert_mirrors_pulse(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = servoCfg(25, 0, true, 50);
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);

  RCFrame f = frameWith(0, 1200);
  pwm.update(0, f, true);

  uint16_t expected = RC_PULSE_MIN_US + RC_PULSE_MAX_US - 1200;
  TEST_ASSERT_EQUAL_UINT16(expected, pwm.currentPulseUs(0));
}

void test_update_rate_throttles_writes(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = servoCfg(25, 0, false, 50);   // 50 Hz -> 20 ms period
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);

  uint8_t ch = PwmManager::ledcChannelFor(0);
  RCFrame f = frameWith(0, 1500);
  pwm.update(0, f, true);
  uint32_t after_first = gpio.writeCount(ch);

  pwm.update(5, f, true);   // 5 ms later, well under 20 ms
  TEST_ASSERT_EQUAL_UINT32(after_first, gpio.writeCount(ch));

  pwm.update(20, f, true);  // 20 ms after the first write
  TEST_ASSERT_EQUAL_UINT32(after_first + 1, gpio.writeCount(ch));
}

void test_switch_crosses_threshold_both_directions(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = switchCfg(26, 1, 1500, true);
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);

  pwm.update(0, frameWith(1, 1600), true);
  TEST_ASSERT_TRUE(pwm.currentDigital(0));
  TEST_ASSERT_TRUE(gpio.lastDigital(26));

  pwm.update(20, frameWith(1, 1400), true);
  TEST_ASSERT_FALSE(pwm.currentDigital(0));
  TEST_ASSERT_FALSE(gpio.lastDigital(26));
}

void test_active_low_polarity_inverts(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = switchCfg(26, 1, 1500, false);   // active low
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);

  pwm.update(0, frameWith(1, 1600), true);   // above threshold, but active-low -> level false
  TEST_ASSERT_FALSE(pwm.currentDigital(0));
}

void test_disabled_writes_nothing(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) cfg[i].mode = PwmMode::DISABLED;
  pwm.begin(cfg);

  pwm.update(0, frameWith(0, 1700), true);
  TEST_ASSERT_EQUAL_UINT32(0, gpio.writeCount(PwmManager::ledcChannelFor(0)));
  TEST_ASSERT_FALSE(pwm.outputActive(0));
}

void test_failsafe_values_on_link_loss(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = servoCfg(25, 0, false, 50);
  cfg[0].failsafe_us = 1100;
  cfg[1] = switchCfg(26, 1, 1500, true);
  cfg[1].failsafe_us = 1200;   // below threshold -> off
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);
  pwm.setFailsafeMode(FailsafeMode::FAILSAFE_VALUES);

  RCFrame f;
  rcFrameInit(f);
  f.channels[0] = 1700;
  f.channels[1] = 1700;
  f.valid = true;
  pwm.update(0, f, true);

  pwm.update(20, f, false);   // link lost
  TEST_ASSERT_EQUAL_UINT16(1100, pwm.currentPulseUs(0));
  TEST_ASSERT_FALSE(pwm.currentDigital(1));
}

void test_stop_pwm_detaches(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = servoCfg(25, 0, false, 50);
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);
  pwm.setFailsafeMode(FailsafeMode::STOP_PWM);

  RCFrame f = frameWith(0, 1700);
  pwm.update(0, f, true);
  TEST_ASSERT_TRUE(pwm.outputActive(0));

  pwm.update(20, f, false);
  TEST_ASSERT_FALSE(pwm.outputActive(0));
  TEST_ASSERT_FALSE(gpio.pwmAttached(PwmManager::ledcChannelFor(0)));
}

void test_hold_last_holds(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = servoCfg(25, 0, false, 50);
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);
  pwm.setFailsafeMode(FailsafeMode::HOLD_LAST);

  RCFrame f = frameWith(0, 1650);
  pwm.update(0, f, true);
  pwm.update(20, f, false);

  TEST_ASSERT_EQUAL_UINT16(1650, pwm.currentPulseUs(0));
  TEST_ASSERT_TRUE(pwm.outputActive(0));
}

void test_runtime_mode_change_servo_to_switch_reattaches(void) {
  MockGpioOutput gpio;
  PwmManager pwm(gpio);
  PWMPinConfig cfg[PWM_PIN_COUNT];
  cfg[0] = servoCfg(25, 0, false, 50);
  cfg[1].mode = PwmMode::DISABLED;
  cfg[2].mode = PwmMode::DISABLED;
  cfg[3].mode = PwmMode::DISABLED;
  pwm.begin(cfg);

  RCFrame f = frameWith(0, 1650);
  pwm.update(0, f, true);
  TEST_ASSERT_TRUE(gpio.pwmAttached(PwmManager::ledcChannelFor(0)));

  PWMPinConfig new_cfg = switchCfg(25, 0, 1500, true);
  pwm.setConfig(0, new_cfg);

  TEST_ASSERT_FALSE(gpio.pwmAttached(PwmManager::ledcChannelFor(0)));
  pwm.update(20, frameWith(0, 1700), true);
  TEST_ASSERT_TRUE(pwm.currentDigital(0));
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_servo_output_writes_mapped_channel);
  RUN_TEST(test_invert_mirrors_pulse);
  RUN_TEST(test_update_rate_throttles_writes);
  RUN_TEST(test_switch_crosses_threshold_both_directions);
  RUN_TEST(test_active_low_polarity_inverts);
  RUN_TEST(test_disabled_writes_nothing);
  RUN_TEST(test_failsafe_values_on_link_loss);
  RUN_TEST(test_stop_pwm_detaches);
  RUN_TEST(test_hold_last_holds);
  RUN_TEST(test_runtime_mode_change_servo_to_switch_reattaches);
  return UNITY_END();
}
