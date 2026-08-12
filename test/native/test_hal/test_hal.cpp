#include <unity.h>
#include "hal/uart_port.h"
#include "hal/gpio_output.h"
#include "hal/adc_input.h"
#include "hal/status_led.h"

void setUp(void) {}
void tearDown(void) {}

void test_mock_uart_port_inject_rx_read_write(void) {
  MockUartPort uart;
  TEST_ASSERT_FALSE(uart.begun());
  TEST_ASSERT_TRUE(uart.begin(420000, UART_CONFIG_8N1, 16, 17, false));
  TEST_ASSERT_TRUE(uart.begun());
  TEST_ASSERT_EQUAL_UINT32(420000, uart.lastBaud());
  TEST_ASSERT_FALSE(uart.lastInverted());

  const uint8_t rx_data[3] = {0x01, 0x02, 0x03};
  uart.injectRx(rx_data, sizeof(rx_data));
  TEST_ASSERT_EQUAL_size_t(3, uart.available());

  uint8_t out[3] = {0, 0, 0};
  size_t n = uart.read(out, sizeof(out));
  TEST_ASSERT_EQUAL_size_t(3, n);
  TEST_ASSERT_EQUAL_UINT8(0x01, out[0]);
  TEST_ASSERT_EQUAL_UINT8(0x03, out[2]);
  TEST_ASSERT_EQUAL_size_t(0, uart.available());

  const uint8_t tx_data[2] = {0xAA, 0xBB};
  size_t written = uart.write(tx_data, sizeof(tx_data));
  TEST_ASSERT_EQUAL_size_t(2, written);
  TEST_ASSERT_EQUAL_size_t(2, uart.txSize());
  TEST_ASSERT_EQUAL_UINT8(0xAA, uart.txData()[0]);
  uart.clearTx();
  TEST_ASSERT_EQUAL_size_t(0, uart.txSize());
}

void test_mock_gpio_output_records_pulse(void) {
  MockGpioOutput gpio;
  TEST_ASSERT_TRUE(gpio.attachPwm(25, 0, 50, 16));
  TEST_ASSERT_TRUE(gpio.pwmAttached(0));
  TEST_ASSERT_EQUAL_UINT32(50, gpio.pwmFreq(0));

  gpio.writePulseUs(0, 1500);
  TEST_ASSERT_EQUAL_UINT16(1500, gpio.lastPulseUs(0));
  TEST_ASSERT_EQUAL_UINT32(1, gpio.writeCount(0));

  gpio.writePulseUs(0, 1600);
  TEST_ASSERT_EQUAL_UINT16(1600, gpio.lastPulseUs(0));
  TEST_ASSERT_EQUAL_UINT32(2, gpio.writeCount(0));

  TEST_ASSERT_TRUE(gpio.attachDigital(2));
  gpio.writeDigital(2, true);
  TEST_ASSERT_TRUE(gpio.lastDigital(2));
  gpio.writeDigital(2, false);
  TEST_ASSERT_FALSE(gpio.lastDigital(2));
}

void test_status_led_solid_level_true(void) {
  MockGpioOutput gpio;
  StatusLed led(gpio, 2, true);
  TEST_ASSERT_TRUE(led.begin());
  led.setPattern(LedPattern::SOLID);
  led.update(0);
  TEST_ASSERT_TRUE(led.level());
  led.update(12345);
  TEST_ASSERT_TRUE(led.level());
}

void test_status_led_slow_blink_toggles_at_500ms(void) {
  MockGpioOutput gpio;
  StatusLed led(gpio, 2, true);
  TEST_ASSERT_TRUE(led.begin());
  led.setPattern(LedPattern::SLOW_BLINK);

  led.update(0);
  TEST_ASSERT_TRUE(led.level());
  led.update(499);
  TEST_ASSERT_TRUE(led.level());
  led.update(500);
  TEST_ASSERT_FALSE(led.level());
  led.update(999);
  TEST_ASSERT_FALSE(led.level());
  led.update(1000);
  TEST_ASSERT_TRUE(led.level());
}

void test_status_led_fast_blink_toggles_at_100ms(void) {
  MockGpioOutput gpio;
  StatusLed led(gpio, 2, true);
  TEST_ASSERT_TRUE(led.begin());
  led.setPattern(LedPattern::FAST_BLINK);

  led.update(0);
  TEST_ASSERT_TRUE(led.level());
  led.update(99);
  TEST_ASSERT_TRUE(led.level());
  led.update(100);
  TEST_ASSERT_FALSE(led.level());
  led.update(199);
  TEST_ASSERT_FALSE(led.level());
  led.update(200);
  TEST_ASSERT_TRUE(led.level());
}

void test_status_led_double_blink_two_rising_edges_per_960ms(void) {
  MockGpioOutput gpio;
  StatusLed led(gpio, 2, true);
  TEST_ASSERT_TRUE(led.begin());
  led.setPattern(LedPattern::DOUBLE_BLINK);

  bool prev = false;
  int rising_edges = 0;
  for (uint32_t t = 0; t < 960; t++) {
    led.update(t);
    bool cur = led.level();
    if (cur && !prev) {
      rising_edges++;
    }
    prev = cur;
  }
  TEST_ASSERT_EQUAL_INT(2, rising_edges);
}

void test_status_led_off_stays_low(void) {
  MockGpioOutput gpio;
  StatusLed led(gpio, 2, true);
  TEST_ASSERT_TRUE(led.begin());
  led.setPattern(LedPattern::OFF);
  led.update(0);
  TEST_ASSERT_FALSE(led.level());
  led.update(5000);
  TEST_ASSERT_FALSE(led.level());
  TEST_ASSERT_FALSE(gpio.lastDigital(2));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_mock_uart_port_inject_rx_read_write);
  RUN_TEST(test_mock_gpio_output_records_pulse);
  RUN_TEST(test_status_led_solid_level_true);
  RUN_TEST(test_status_led_slow_blink_toggles_at_500ms);
  RUN_TEST(test_status_led_fast_blink_toggles_at_100ms);
  RUN_TEST(test_status_led_double_blink_two_rising_edges_per_960ms);
  RUN_TEST(test_status_led_off_stays_low);
  return UNITY_END();
}
