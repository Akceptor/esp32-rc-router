#include <unity.h>
#include <string.h>
#include "protocols/mavlink/mavlink_generator.h"
#include "protocols/mavlink/mavlink_parser.h"

void setUp(void) {}
void tearDown(void) {}

static void test_round_trip_channels_exact(void) {
  MavlinkGenerator gen;
  MavlinkParser parser;
  RCFrame frame;
  rcFrameInit(frame);
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) {
    frame.channels[i] = (uint16_t)(1000 + i * 50);
  }
  frame.valid = true;

  uint8_t buf[280];
  size_t n = gen.buildRcFrame(frame, buf, sizeof(buf));
  TEST_ASSERT_GREATER_THAN_UINT32(0, n);

  size_t frames = parser.push(buf, n, 100);

  TEST_ASSERT_EQUAL_UINT32(1, frames);
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) {
    TEST_ASSERT_EQUAL_UINT16(1000 + i * 50, parser.frame().channels[i]);
  }
}

static void test_header_bytes_exact(void) {
  MavlinkGenerator gen;
  RCFrame frame;
  rcFrameInit(frame);
  uint8_t buf[280];
  size_t n = gen.buildRcFrame(frame, buf, sizeof(buf));

  TEST_ASSERT_GREATER_THAN_UINT32(9, n);
  TEST_ASSERT_EQUAL_UINT8(MAVLINK_STX_V2, buf[0]);
  uint32_t msgid = (uint32_t)buf[7] | ((uint32_t)buf[8] << 8) | ((uint32_t)buf[9] << 16);
  TEST_ASSERT_EQUAL_UINT32(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, msgid);
}

static void test_seq_increments_and_wraps(void) {
  MavlinkGenerator gen;
  RCFrame frame;
  rcFrameInit(frame);
  uint8_t buf[280];

  size_t n0 = gen.buildRcFrame(frame, buf, sizeof(buf));
  uint8_t seq0 = buf[4];
  TEST_ASSERT_EQUAL_UINT8(0, seq0);
  (void)n0;

  uint8_t last_seq = seq0;
  for (int i = 0; i < 255; ++i) {
    gen.buildRcFrame(frame, buf, sizeof(buf));
    uint8_t seq = buf[4];
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(last_seq + 1), seq);
    last_seq = seq;
  }
  // After 256 total frames built, sequence has wrapped back to 0.
  TEST_ASSERT_EQUAL_UINT8(255, last_seq);
  gen.buildRcFrame(frame, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT8(0, buf[4]);
}

static void test_zero_trimming_shortens_len(void) {
  uint8_t payload[10] = {1, 2, 3, 0, 0, 0, 0, 0, 0, 0};
  size_t trimmed = mavlinkTrimTrailingZeros(payload, 10);
  TEST_ASSERT_EQUAL_UINT32(3, trimmed);

  uint8_t all_nonzero[4] = {9, 9, 9, 9};
  TEST_ASSERT_EQUAL_UINT32(4, mavlinkTrimTrailingZeros(all_nonzero, 4));

  uint8_t all_zero[5] = {0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_UINT32(0, mavlinkTrimTrailingZeros(all_zero, 5));
}

static void test_out_cap_too_small_returns_zero(void) {
  MavlinkGenerator gen;
  RCFrame frame;
  rcFrameInit(frame);
  uint8_t buf[4];
  size_t n = gen.buildRcFrame(frame, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT32(0, n);
}

static void test_battery_round_trip_through_telemetry_fifo(void) {
  MavlinkGenerator gen;
  MavlinkParser parser;
  BatteryTelemetry batt;
  batt.voltage_dv = 1250;   // 12.50 V
  batt.current_da = 85;     // 8.5 A
  batt.used_capacity_mah = 1200;
  batt.remaining_percent = 63;

  uint8_t buf[280];
  size_t n = gen.buildBatteryTelemetry(batt, buf, sizeof(buf));
  TEST_ASSERT_GREATER_THAN_UINT32(0, n);

  parser.push(buf, n, 900);

  TelemetryPacket pkt;
  bool got = parser.popTelemetry(pkt);
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL_INT((int)TelemetryKind::BATTERY, (int)pkt.kind);
  TEST_ASSERT_EQUAL_UINT8(63, pkt.data[35]);
  uint16_t voltage_mv = (uint16_t)pkt.data[5] | ((uint16_t)pkt.data[6] << 8);
  TEST_ASSERT_EQUAL_UINT16(12500, voltage_mv);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_round_trip_channels_exact);
  RUN_TEST(test_header_bytes_exact);
  RUN_TEST(test_seq_increments_and_wraps);
  RUN_TEST(test_zero_trimming_shortens_len);
  RUN_TEST(test_out_cap_too_small_returns_zero);
  RUN_TEST(test_battery_round_trip_through_telemetry_fifo);
  return UNITY_END();
}
