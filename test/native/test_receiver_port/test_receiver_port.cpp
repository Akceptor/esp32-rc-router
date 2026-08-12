#include <unity.h>
#include <string.h>
#include "receiver/receiver_port.h"
#include "hal/uart_port.h"
#include "protocols/crsf/crsf_generator.h"

void setUp(void) {}
void tearDown(void) {}

static ReceiverPortConfig makeCfg(ProtocolType proto, uint32_t baud, int8_t rx, int8_t tx,
                                   bool inverted, uint8_t priority) {
  ReceiverPortConfig cfg;
  cfg.enabled = true;
  cfg.protocol = proto;
  cfg.priority = priority;
  cfg.baud = baud;
  cfg.rx_pin = rx;
  cfg.tx_pin = tx;
  cfg.inverted = inverted;
  return cfg;
}

static void test_crsf_bytes_produce_frame_and_alive(void) {
  MockUartPort uart;
  ReceiverPort port(0, uart);
  ReceiverPortConfig cfg = makeCfg(ProtocolType::CRSF, 420000, 16, 17, false, 0);
  TEST_ASSERT_TRUE(port.begin(cfg));

  CrsfGenerator gen;
  RCFrame frame;
  rcFrameInit(frame);
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) frame.channels[i] = 1500;
  frame.valid = true;
  uint8_t buf[64];
  size_t n = gen.buildRcFrame(frame, buf, sizeof(buf));
  uart.injectRx(buf, n);

  port.update(1000);

  TEST_ASSERT_TRUE(port.getFrame().valid);
  TEST_ASSERT_TRUE(port.isAlive(1000, 500));
}

static void test_is_alive_false_after_timeout(void) {
  MockUartPort uart;
  ReceiverPort port(0, uart);
  ReceiverPortConfig cfg = makeCfg(ProtocolType::CRSF, 420000, 16, 17, false, 0);
  port.begin(cfg);

  CrsfGenerator gen;
  RCFrame frame;
  rcFrameInit(frame);
  frame.valid = true;
  uint8_t buf[64];
  size_t n = gen.buildRcFrame(frame, buf, sizeof(buf));
  uart.injectRx(buf, n);
  port.update(1000);
  TEST_ASSERT_TRUE(port.isAlive(1000, 500));

  TEST_ASSERT_FALSE(port.isAlive(2000, 500));
}

static void test_protocol_switch_to_sbus_rebegins_uart(void) {
  MockUartPort uart;
  ReceiverPort port(1, uart);
  ReceiverPortConfig cfg_crsf = makeCfg(ProtocolType::CRSF, 420000, 18, 19, false, 1);
  port.begin(cfg_crsf);
  TEST_ASSERT_EQUAL_UINT32(420000, uart.lastBaud());

  ReceiverPortConfig cfg_sbus = makeCfg(ProtocolType::SBUS, 100000, 18, 19, false, 1);
  port.setConfig(cfg_sbus);

  TEST_ASSERT_EQUAL_UINT32(100000, uart.lastBaud());
  TEST_ASSERT_TRUE(uart.lastInverted());
  TEST_ASSERT_TRUE(port.protocol() == ProtocolType::SBUS);
}

static void test_get_frame_invalid_before_data(void) {
  MockUartPort uart;
  ReceiverPort port(0, uart);
  ReceiverPortConfig cfg = makeCfg(ProtocolType::CRSF, 420000, 16, 17, false, 0);
  port.begin(cfg);

  TEST_ASSERT_FALSE(port.getFrame().valid);
}

static void test_write_telemetry_returns_0_for_sbus_and_n_for_crsf(void) {
  MockUartPort uart_sbus;
  ReceiverPort port_sbus(0, uart_sbus);
  ReceiverPortConfig cfg_sbus = makeCfg(ProtocolType::SBUS, 100000, 16, 17, true, 0);
  port_sbus.begin(cfg_sbus);
  uint8_t payload[4] = {1, 2, 3, 4};
  TEST_ASSERT_EQUAL_UINT32(0, port_sbus.writeTelemetry(payload, sizeof(payload)));

  MockUartPort uart_crsf;
  ReceiverPort port_crsf(1, uart_crsf);
  ReceiverPortConfig cfg_crsf = makeCfg(ProtocolType::CRSF, 420000, 18, 19, false, 0);
  port_crsf.begin(cfg_crsf);
  TEST_ASSERT_EQUAL_UINT32(4, port_crsf.writeTelemetry(payload, sizeof(payload)));
  TEST_ASSERT_EQUAL_UINT32(4, uart_crsf.txSize());
}

static void test_bytes_read_accounting(void) {
  MockUartPort uart;
  ReceiverPort port(0, uart);
  ReceiverPortConfig cfg = makeCfg(ProtocolType::CRSF, 420000, 16, 17, false, 0);
  port.begin(cfg);

  CrsfGenerator gen;
  RCFrame frame;
  rcFrameInit(frame);
  frame.valid = true;
  uint8_t buf[64];
  size_t n = gen.buildRcFrame(frame, buf, sizeof(buf));
  uart.injectRx(buf, n);

  port.update(500);

  TEST_ASSERT_EQUAL_UINT32(n, port.bytesRead());
}

static void test_telemetry_pops_through_from_parser(void) {
  MockUartPort uart;
  ReceiverPort port(0, uart);
  ReceiverPortConfig cfg = makeCfg(ProtocolType::CRSF, 420000, 16, 17, false, 0);
  port.begin(cfg);

  // CRSF battery sensor frame: SYNC, LEN, TYPE=0x08, payload(8), CRC8.
  uint8_t frame[12];
  frame[0] = 0xC8;
  frame[1] = 10;
  frame[2] = 0x08;
  uint8_t payload[8] = {0x04, 0xB0, 0x00, 0x05, 0x00, 0x00, 0x64, 88};
  memcpy(&frame[3], payload, 8);
  uint8_t crc = 0;
  for (int i = 2; i < 11; ++i) {
    crc ^= frame[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  frame[11] = crc;
  uart.injectRx(frame, sizeof(frame));

  port.update(200);

  TelemetryPacket pkt;
  TEST_ASSERT_TRUE(port.popTelemetry(pkt));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_crsf_bytes_produce_frame_and_alive);
  RUN_TEST(test_is_alive_false_after_timeout);
  RUN_TEST(test_protocol_switch_to_sbus_rebegins_uart);
  RUN_TEST(test_get_frame_invalid_before_data);
  RUN_TEST(test_write_telemetry_returns_0_for_sbus_and_n_for_crsf);
  RUN_TEST(test_bytes_read_accounting);
  RUN_TEST(test_telemetry_pops_through_from_parser);
  return UNITY_END();
}
