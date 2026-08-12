#include <unity.h>
#include <string.h>
#include "hal/uart_port.h"
#include "output/output_manager.h"
#include "protocols/protocol_types.h"
#include "protocols/crsf/crsf_parser.h"
#include "protocols/sbus/sbus_parser.h"
#include "config/config_types.h"

static OutputConfig makeCrsfCfg() {
  OutputConfig cfg;
  cfg.protocol = ProtocolType::CRSF;
  cfg.baud = 420000;
  cfg.tx_pin = 23;
  cfg.rx_pin = 22;
  cfg.inverted = false;
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) cfg.channel_map[i] = i;
  return cfg;
}

static OutputConfig makeSbusCfg() {
  OutputConfig cfg = makeCrsfCfg();
  cfg.protocol = ProtocolType::SBUS;
  cfg.baud = 100000;      // will be forced anyway
  cfg.inverted = false;   // will be forced to true
  return cfg;
}

static RCFrame makeFrame(uint16_t base) {
  RCFrame f;
  rcFrameInit(f);
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    f.channels[i] = clampPulseUs(base + i * 10);
  }
  f.valid = true;
  f.timestamp_ms = 0;
  return f;
}

void test_crsf_output_produces_26_byte_frame(void) {
  MockUartPort uart;
  OutputManager om(uart);
  OutputConfig cfg = makeCrsfCfg();
  TEST_ASSERT_TRUE(om.begin(cfg));

  RCFrame f = makeFrame(1000);
  uart.clearTx();
  om.update(1000, f, true);

  TEST_ASSERT_EQUAL_UINT32(26, uart.txSize());
  TEST_ASSERT_EQUAL_HEX8(0xC8, uart.txData()[0]);
  TEST_ASSERT_EQUAL_UINT32(1, om.framesSent());
  TEST_ASSERT_EQUAL_UINT32(26, om.lastFrameSize());
}

void test_crsf_rate_limiting(void) {
  MockUartPort uart;
  OutputManager om(uart);
  OutputConfig cfg = makeCrsfCfg();
  om.begin(cfg);

  RCFrame f = makeFrame(1000);
  om.update(1000, f, true);
  TEST_ASSERT_EQUAL_UINT32(1, om.framesSent());

  uart.clearTx();
  om.update(1001, f, true);   // only 1 ms later, interval is 4 ms
  TEST_ASSERT_EQUAL_UINT32(0, uart.txSize());
  TEST_ASSERT_EQUAL_UINT32(1, om.framesSent());

  om.update(1004, f, true);   // 4 ms after the first send
  TEST_ASSERT_EQUAL_UINT32(26, uart.txSize());
  TEST_ASSERT_EQUAL_UINT32(2, om.framesSent());
}

void test_switch_to_sbus_reconfigures_uart(void) {
  MockUartPort uart;
  OutputManager om(uart);
  om.begin(makeCrsfCfg());

  OutputConfig sbus_cfg = makeSbusCfg();
  om.setConfig(sbus_cfg);

  TEST_ASSERT_TRUE(uart.begun());
  TEST_ASSERT_EQUAL_UINT32(100000, uart.lastBaud());
  TEST_ASSERT_TRUE(uart.lastInverted());

  RCFrame f = makeFrame(1200);
  uart.clearTx();
  om.update(2000, f, true);
  TEST_ASSERT_EQUAL_UINT32(25, uart.txSize());
}

void test_channel_map_applied(void) {
  MockUartPort uart;
  OutputManager om(uart);
  OutputConfig cfg = makeCrsfCfg();
  // out_ch 0 pulls from in_ch 6
  uint8_t map[RC_CHANNEL_COUNT];
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) map[i] = i;
  map[0] = 6;
  cfg.channel_map[0] = 6;
  for (uint8_t i = 1; i < RC_CHANNEL_COUNT; i++) cfg.channel_map[i] = i;
  om.begin(cfg);
  om.mapper().setMap(cfg.channel_map);

  RCFrame f = makeFrame(1000);
  uint16_t expected_ch6 = f.channels[6];

  uart.clearTx();
  om.update(3000, f, true);

  CrsfParser parser;
  parser.push(uart.txData(), uart.txSize(), 3000);
  TEST_ASSERT_TRUE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT16(expected_ch6, parser.frame().channels[0]);
}

void test_failsafe_values_mode(void) {
  MockUartPort uart;
  OutputManager om(uart);
  OutputConfig cfg = makeCrsfCfg();
  om.begin(cfg);

  uint16_t fs_values[RC_CHANNEL_COUNT];
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) fs_values[i] = 1300;
  om.setFailsafe(FailsafeMode::FAILSAFE_VALUES, fs_values);

  RCFrame f = makeFrame(1000);
  om.update(5000, f, true);   // establish a good frame first

  uart.clearTx();
  om.update(5004, f, false);  // link lost

  CrsfParser parser;
  parser.push(uart.txData(), uart.txSize(), 5004);
  TEST_ASSERT_TRUE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT16(1300, parser.frame().channels[3]);
}

void test_hold_last_mode(void) {
  MockUartPort uart;
  OutputManager om(uart);
  OutputConfig cfg = makeCrsfCfg();
  om.begin(cfg);
  uint16_t fs_values[RC_CHANNEL_COUNT];
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) fs_values[i] = 1300;
  om.setFailsafe(FailsafeMode::HOLD_LAST, fs_values);

  RCFrame f = makeFrame(1400);
  om.update(6000, f, true);

  uart.clearTx();
  om.update(6004, f, false);

  CrsfParser parser;
  parser.push(uart.txData(), uart.txSize(), 6004);
  TEST_ASSERT_TRUE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT16(f.channels[3], parser.frame().channels[3]);
}

void test_stop_pwm_mode_sends_one_flagged_frame_then_silence(void) {
  MockUartPort uart;
  OutputManager om(uart);
  OutputConfig cfg = makeSbusCfg();
  om.begin(cfg);
  uint16_t fs_values[RC_CHANNEL_COUNT];
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) fs_values[i] = 1300;
  om.setFailsafe(FailsafeMode::STOP_PWM, fs_values);

  RCFrame f = makeFrame(1400);
  om.update(7000, f, true);

  uart.clearTx();
  om.update(7014, f, false);   // link just lost -> one final flagged frame
  TEST_ASSERT_EQUAL_UINT32(25, uart.txSize());
  SbusParser parser;
  parser.push(uart.txData(), uart.txSize(), 7014);
  TEST_ASSERT_TRUE(parser.linkQuality().failsafe);

  uart.clearTx();
  om.update(7028, f, false);   // still down -> silence
  TEST_ASSERT_EQUAL_UINT32(0, uart.txSize());
  om.update(7042, f, false);
  TEST_ASSERT_EQUAL_UINT32(0, uart.txSize());
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_crsf_output_produces_26_byte_frame);
  RUN_TEST(test_crsf_rate_limiting);
  RUN_TEST(test_switch_to_sbus_reconfigures_uart);
  RUN_TEST(test_channel_map_applied);
  RUN_TEST(test_failsafe_values_mode);
  RUN_TEST(test_hold_last_mode);
  RUN_TEST(test_stop_pwm_mode_sends_one_flagged_frame_then_silence);
  return UNITY_END();
}
