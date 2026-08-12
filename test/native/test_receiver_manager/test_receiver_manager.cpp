#include <unity.h>
#include <string.h>
#include "receiver/receiver_manager.h"
#include "receiver/receiver_port.h"
#include "hal/uart_port.h"

void setUp(void) {}
void tearDown(void) {}

static void packChannels11Bit(const uint16_t chan_us[16], uint8_t out[22]) {
  uint32_t bitbuf = 0;
  int bitcount = 0;
  size_t outidx = 0;
  for (int i = 0; i < 16; ++i) {
    int32_t raw = 172 + ((int32_t)chan_us[i] - 988) * 1639 / 1024;
    if (raw < 0) raw = 0;
    if (raw > 2047) raw = 2047;
    uint32_t rawu = (uint32_t)raw & 0x7FF;
    bitbuf |= (rawu << bitcount);
    bitcount += 11;
    while (bitcount >= 8) {
      out[outidx++] = (uint8_t)(bitbuf & 0xFF);
      bitbuf >>= 8;
      bitcount -= 8;
    }
  }
  if (bitcount > 0 && outidx < 22) {
    out[outidx++] = (uint8_t)(bitbuf & 0xFF);
  }
}

static uint8_t testCrsfCrc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static size_t buildCrsfRcFrame(const uint16_t ch_us[16], uint8_t* out) {
  uint8_t payload[22];
  packChannels11Bit(ch_us, payload);
  out[0] = 0xC8;   // CRSF_SYNC
  out[1] = 24;     // len = type(1) + payload(22) + crc(1)
  out[2] = 0x16;   // CRSF_TYPE_RC_CHANNELS_PACKED
  memcpy(&out[3], payload, 22);
  out[25] = testCrsfCrc8(&out[2], 23);
  return 26;
}

static size_t buildCrsfLinkStatsFrame(uint8_t lq, int8_t rssi_dbm, uint8_t* out) {
  uint8_t rssi_mag = (uint8_t)(-(int16_t)rssi_dbm);
  uint8_t payload[10] = {rssi_mag, rssi_mag, lq, 0, 0, 0, 0, rssi_mag, lq, 0};
  out[0] = 0xC8;
  out[1] = 12;     // len = type(1) + payload(10) + crc(1)
  out[2] = 0x14;   // CRSF_TYPE_LINK_STATISTICS
  memcpy(&out[3], payload, 10);
  out[13] = testCrsfCrc8(&out[2], 11);
  return 14;
}

// Injects a LINK_STATISTICS frame followed by an RC_CHANNELS_PACKED frame carrying
// `ch`, with the given link quality percentage and RSSI (negative dBm).
static void feedCrsf(MockUartPort& uart, const uint16_t ch[16], uint8_t lq, int8_t rssi_dbm) {
  uint8_t buf[64];
  size_t n1 = buildCrsfLinkStatsFrame(lq, rssi_dbm, buf);
  size_t n2 = buildCrsfRcFrame(ch, buf + n1);
  uart.injectRx(buf, n1 + n2);
}

static void feedSbus(MockUartPort& uart, const uint16_t ch[16]) {
  uint8_t payload[22];
  packChannels11Bit(ch, payload);
  uint8_t frame[25];
  frame[0] = 0x0F;  // SBUS_HEADER
  memcpy(&frame[1], payload, 22);
  frame[23] = 0x00;  // flags: no frame_lost, no failsafe
  frame[24] = 0x00;  // SBUS_FOOTER
  uart.injectRx(frame, 25);
}

static ReceiverPortConfig makeCfg(ProtocolType proto, int8_t rx, int8_t tx, uint8_t priority) {
  ReceiverPortConfig cfg;
  cfg.enabled = true;
  cfg.protocol = proto;
  cfg.priority = priority;
  cfg.baud = (proto == ProtocolType::CRSF) ? 420000 : 100000;
  cfg.rx_pin = rx;
  cfg.tx_pin = tx;
  cfg.inverted = (proto == ProtocolType::SBUS);
  return cfg;
}

static SelectionConfig makeSelCfg() {
  SelectionConfig cfg;
  cfg.rssi_threshold_percent = 30;
  cfg.lq_threshold_percent = 30;
  cfg.hysteresis_percent = 10;
  cfg.switch_delay_ms = 500;
  cfg.min_active_time_ms = 1000;
  cfg.link_timeout_ms = 300;
  return cfg;
}

static void fillChannels(uint16_t ch[16], uint16_t us) {
  for (int i = 0; i < 16; ++i) ch[i] = us;
}

static void test_starts_no_link(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::SBUS, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  mgr.begin(makeSelCfg());

  TEST_ASSERT_TRUE(mgr.state() == SelectionState::NO_LINK);
  TEST_ASSERT_FALSE(mgr.hasValidLink());
  TEST_ASSERT_EQUAL_INT8(-1, mgr.activeIndex());
}

static void test_single_alive_port_becomes_active(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::SBUS, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  mgr.begin(makeSelCfg());

  uint16_t ch[16];
  fillChannels(ch, 1500);
  feedCrsf(uart_a, ch, 90, -60);

  mgr.update(100);

  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());
  TEST_ASSERT_TRUE(mgr.hasValidLink());
  TEST_ASSERT_EQUAL_UINT32(0, mgr.switchCount());
}

static void test_better_challenger_does_not_switch_before_delay(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::CRSF, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.min_active_time_ms = 0;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);

  feedCrsf(uart_a, ch, 40, -90);
  mgr.update(0);
  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());

  for (uint32_t t = 50; t <= 400; t += 50) {
    feedCrsf(uart_a, ch, 40, -90);
    feedCrsf(uart_b, ch, 100, -40);
    mgr.update(t);
  }

  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());  // still on original active
  TEST_ASSERT_EQUAL_UINT32(0, mgr.switchCount());
}

static void test_does_switch_after_delay(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::CRSF, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.min_active_time_ms = 0;
  sel.switch_delay_ms = 300;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);

  feedCrsf(uart_a, ch, 40, -90);
  mgr.update(0);
  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());

  for (uint32_t t = 50; t <= 400; t += 50) {
    feedCrsf(uart_a, ch, 40, -90);
    feedCrsf(uart_b, ch, 100, -40);
    mgr.update(t);
  }

  TEST_ASSERT_EQUAL_INT8(1, mgr.activeIndex());
  TEST_ASSERT_EQUAL_UINT32(1, mgr.switchCount());
}

static void test_does_not_switch_below_hysteresis(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::CRSF, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.min_active_time_ms = 0;
  sel.hysteresis_percent = 50;
  sel.switch_delay_ms = 100;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);

  feedCrsf(uart_a, ch, 80, -60);
  mgr.update(0);
  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());

  for (uint32_t t = 50; t <= 500; t += 50) {
    feedCrsf(uart_a, ch, 80, -60);
    feedCrsf(uart_b, ch, 90, -55);  // only marginally better, below 50% margin
    mgr.update(t);
  }

  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());
  TEST_ASSERT_EQUAL_UINT32(0, mgr.switchCount());
}

static void test_dead_active_switches_immediately(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::CRSF, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.min_active_time_ms = 5000;  // very long hold, must be ignored on dead-active path
  sel.link_timeout_ms = 200;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);

  feedCrsf(uart_a, ch, 90, -60);
  mgr.update(0);
  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());

  // Port A goes silent; port B comes alive well before min_active_time_ms would allow.
  feedCrsf(uart_b, ch, 90, -60);
  mgr.update(250);  // now_ms - last_frame(a)=250 > link_timeout_ms(200) => a is dead

  TEST_ASSERT_EQUAL_INT8(1, mgr.activeIndex());
  TEST_ASSERT_EQUAL_UINT32(1, mgr.switchCount());
}

static void test_min_active_time_blocks_early_quality_switch(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::CRSF, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.min_active_time_ms = 10000;
  sel.switch_delay_ms = 100;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);

  feedCrsf(uart_a, ch, 40, -90);
  mgr.update(0);
  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());

  for (uint32_t t = 50; t <= 500; t += 50) {
    feedCrsf(uart_a, ch, 40, -90);
    feedCrsf(uart_b, ch, 100, -40);
    mgr.update(t);
  }

  // Both alive, port B clearly better and stable well past switch_delay_ms, but
  // min_active_time_ms(10000) has not elapsed on the current active -> no switch.
  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());
  TEST_ASSERT_EQUAL_UINT32(0, mgr.switchCount());
}

static void test_both_dead_no_link(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::SBUS, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.link_timeout_ms = 200;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);
  feedCrsf(uart_a, ch, 90, -60);
  feedSbus(uart_b, ch);
  mgr.update(0);
  TEST_ASSERT_TRUE(mgr.hasValidLink());

  mgr.update(1000);  // neither port fed again; both exceed link_timeout_ms

  TEST_ASSERT_TRUE(mgr.state() == SelectionState::NO_LINK);
  TEST_ASSERT_FALSE(mgr.hasValidLink());
  TEST_ASSERT_EQUAL_INT8(-1, mgr.activeIndex());
}

static void test_switch_count_increments_once_per_switch(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::CRSF, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.min_active_time_ms = 0;
  sel.switch_delay_ms = 100;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);

  feedCrsf(uart_a, ch, 40, -90);
  mgr.update(0);

  for (uint32_t t = 50; t <= 300; t += 50) {
    feedCrsf(uart_a, ch, 40, -90);
    feedCrsf(uart_b, ch, 100, -40);
    mgr.update(t);
  }
  TEST_ASSERT_EQUAL_UINT32(1, mgr.switchCount());

  // Keep feeding stable conditions; switchCount must not keep incrementing.
  for (uint32_t t = 350; t <= 600; t += 50) {
    feedCrsf(uart_a, ch, 40, -90);
    feedCrsf(uart_b, ch, 100, -40);
    mgr.update(t);
  }
  TEST_ASSERT_EQUAL_UINT32(1, mgr.switchCount());
}

static void test_flapping_does_not_cause_repeated_switches(void) {
  MockUartPort uart_a, uart_b;
  ReceiverPort port_a(0, uart_a), port_b(1, uart_b);
  port_a.begin(makeCfg(ProtocolType::CRSF, 16, 17, 0));
  port_b.begin(makeCfg(ProtocolType::CRSF, 18, 19, 1));

  ReceiverManager mgr(port_a, port_b);
  SelectionConfig sel = makeSelCfg();
  sel.min_active_time_ms = 0;
  sel.switch_delay_ms = 300;
  mgr.begin(sel);

  uint16_t ch[16];
  fillChannels(ch, 1500);

  feedCrsf(uart_a, ch, 40, -90);
  mgr.update(0);

  // Challenger quality flaps above/below the hysteresis margin every other tick,
  // each dip resetting the "stayed better" timer, so switch_delay_ms is never
  // satisfied uninterrupted within this short window.
  uint32_t t = 50;
  for (int cycle = 0; cycle < 6; ++cycle) {
    feedCrsf(uart_a, ch, 40, -90);
    feedCrsf(uart_b, ch, 100, -40);  // clearly better
    mgr.update(t);
    t += 50;
    feedCrsf(uart_a, ch, 40, -90);
    feedCrsf(uart_b, ch, 45, -88);  // dips back near active's quality
    mgr.update(t);
    t += 50;
  }

  TEST_ASSERT_EQUAL_INT8(0, mgr.activeIndex());
  TEST_ASSERT_EQUAL_UINT32(0, mgr.switchCount());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_starts_no_link);
  RUN_TEST(test_single_alive_port_becomes_active);
  RUN_TEST(test_better_challenger_does_not_switch_before_delay);
  RUN_TEST(test_does_switch_after_delay);
  RUN_TEST(test_does_not_switch_below_hysteresis);
  RUN_TEST(test_dead_active_switches_immediately);
  RUN_TEST(test_min_active_time_blocks_early_quality_switch);
  RUN_TEST(test_both_dead_no_link);
  RUN_TEST(test_switch_count_increments_once_per_switch);
  RUN_TEST(test_flapping_does_not_cause_repeated_switches);
  return UNITY_END();
}
