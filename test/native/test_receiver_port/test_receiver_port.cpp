#include <unity.h>
#include <string.h>
#include "receiver/receiver_port.h"
#include "hal/uart_port.h"
#include "protocols/crsf/crsf_generator.h"
#include "protocols/mavlink/mavlink_parser.h"

void setUp(void) {}
void tearDown(void) {}

// --- SBUS frame builder (adapted from test/native/test_sbus_parser/test_sbus_parser.cpp) ---
// Packs 16 channel raw values (11-bit, 0..2047) LSB-first into the 22 SBUS data bytes.
static void packSbusChannels(const uint16_t raw[16], uint8_t out[22]) {
  memset(out, 0, 22);
  uint32_t bitpos = 0;
  for (int ch = 0; ch < 16; ch++) {
    uint32_t v = raw[ch] & 0x7FFu;
    for (int b = 0; b < 11; b++) {
      if (v & (1u << b)) {
        uint32_t bit = bitpos + b;
        out[bit / 8] |= (uint8_t)(1u << (bit % 8));
      }
    }
    bitpos += 11;
  }
}

static void buildSbusFrame(const uint16_t raw[16], uint8_t flags, uint8_t out[25]) {
  out[0] = 0x0F;
  uint8_t data[22];
  packSbusChannels(raw, data);
  memcpy(&out[1], data, 22);
  out[23] = flags;
  out[24] = 0x00;
}

// --- MAVLINK frame builder (adapted from test/native/test_mavlink_parser/test_mavlink_parser.cpp) ---
static size_t buildMavFrame(uint32_t msgid, const uint8_t* payload, uint8_t len, uint8_t* out) {
  size_t idx = 0;
  out[idx++] = MAVLINK_STX_V2;
  out[idx++] = len;
  out[idx++] = 0;  // incompat_flags
  out[idx++] = 0;  // compat_flags
  out[idx++] = 0;  // seq
  out[idx++] = MAVLINK_SYSTEM_ID;
  out[idx++] = MAVLINK_COMPONENT_ID;
  out[idx++] = (uint8_t)(msgid & 0xFF);
  out[idx++] = (uint8_t)((msgid >> 8) & 0xFF);
  out[idx++] = (uint8_t)((msgid >> 16) & 0xFF);
  for (uint8_t i = 0; i < len; ++i) {
    out[idx++] = payload[i];
  }
  uint16_t crc = mavlinkCrc16(&out[1], (size_t)9 + len, mavlinkCrcExtra(msgid));
  out[idx++] = (uint8_t)(crc & 0xFF);
  out[idx++] = (uint8_t)((crc >> 8) & 0xFF);
  return idx;
}

static void writeU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void writeU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void buildRcChannelsPayload(const uint16_t chan_us[18], uint8_t chancount, uint8_t rssi,
                                    uint8_t out_payload[42]) {
  writeU32(&out_payload[0], 12345);  // time_boot_ms
  for (int i = 0; i < 18; ++i) {
    writeU16(&out_payload[4 + i * 2], chan_us[i]);
  }
  out_payload[40] = chancount;
  out_payload[41] = rssi;
}

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

static void test_sbus_bytes_produce_decoded_frame_through_port(void) {
  MockUartPort uart;
  ReceiverPort port(0, uart);
  ReceiverPortConfig cfg = makeCfg(ProtocolType::SBUS, 100000, 16, 17, true, 0);
  TEST_ASSERT_TRUE(port.begin(cfg));

  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  raw[0] = 172;
  raw[1] = 1811;
  uint8_t frame[25];
  buildSbusFrame(raw, 0x00, frame);
  uart.injectRx(frame, sizeof(frame));

  port.update(1000);

  TEST_ASSERT_TRUE(port.getFrame().valid);
  TEST_ASSERT_EQUAL_UINT16(988, port.getFrame().channels[0]);
  TEST_ASSERT_EQUAL_UINT16(2012, port.getFrame().channels[1]);
}

static void test_mavlink_bytes_produce_decoded_frame_through_port(void) {
  MockUartPort uart;
  ReceiverPort port(0, uart);
  ReceiverPortConfig cfg = makeCfg(ProtocolType::MAVLINK, 57600, 16, 17, false, 0);
  TEST_ASSERT_TRUE(port.begin(cfg));

  uint16_t chan_us[18];
  for (int i = 0; i < 18; ++i) chan_us[i] = 1000 + i * 10;
  uint8_t payload[42];
  buildRcChannelsPayload(chan_us, 16, 200, payload);
  uint8_t frame[64];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_RC_CHANNELS, payload, 42, frame);
  uart.injectRx(frame, n);

  port.update(1000);

  TEST_ASSERT_TRUE(port.getFrame().valid);
  for (int i = 0; i < 16; ++i) {
    TEST_ASSERT_EQUAL_UINT16(1000 + i * 10, port.getFrame().channels[i]);
  }
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
  RUN_TEST(test_sbus_bytes_produce_decoded_frame_through_port);
  RUN_TEST(test_mavlink_bytes_produce_decoded_frame_through_port);
  return UNITY_END();
}
