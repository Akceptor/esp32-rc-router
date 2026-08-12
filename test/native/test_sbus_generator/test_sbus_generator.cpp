// test/native/test_sbus_generator/test_sbus_generator.cpp
#include <unity.h>
#include "protocols/sbus/sbus_generator.h"
#include "protocols/sbus/sbus_parser.h"

void setUp(void) {}
void tearDown(void) {}

static void test_round_trip_through_parser(void) {
  SbusGenerator gen;
  RCFrame f;
  rcFrameInit(f);
  uint16_t values[RC_CHANNEL_COUNT] = {988, 1000, 1200, 1500, 1700, 1900, 2012, 988,
                                        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) f.channels[i] = values[i];
  f.valid = true;

  uint8_t out[32];
  size_t n = gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT32(25, n);

  SbusParser parser;
  size_t decoded = parser.push(out, n, 100);
  TEST_ASSERT_EQUAL_UINT32(1, decoded);
  TEST_ASSERT_TRUE(parser.hasFrame());

  const RCFrame& rt = parser.frame();
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    int32_t diff = (int32_t)rt.channels[i] - (int32_t)values[i];
    if (diff < 0) diff = -diff;
    TEST_ASSERT_TRUE(diff <= 1);
  }
}

static void test_exact_header_footer_bytes(void) {
  SbusGenerator gen;
  RCFrame f;
  rcFrameInit(f);
  f.valid = true;

  uint8_t out[32];
  size_t n = gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT32(25, n);
  TEST_ASSERT_EQUAL_HEX8(0x0F, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, out[24]);
}

static void test_flags_byte_reflects_setters(void) {
  SbusGenerator gen;
  RCFrame f;
  rcFrameInit(f);
  f.valid = true;

  uint8_t out[32];

  gen.setFailsafe(false);
  gen.setFrameLost(false);
  gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT8(0x00, out[23] & 0x0C);

  gen.setFrameLost(true);
  gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT8(0x04, out[23] & 0x04);

  gen.setFailsafe(true);
  gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT8(0x08, out[23] & 0x08);
  TEST_ASSERT_EQUAL_UINT8(0x04, out[23] & 0x04);
}

static void test_out_cap_24_returns_zero(void) {
  SbusGenerator gen;
  RCFrame f;
  rcFrameInit(f);
  f.valid = true;

  uint8_t out[24];
  size_t n = gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT32(0, n);
}

static void test_battery_returns_zero(void) {
  SbusGenerator gen;
  BatteryTelemetry batt;
  batt.voltage_dv = 1200;
  batt.current_da = 100;
  batt.used_capacity_mah = 500;
  batt.remaining_percent = 50;

  uint8_t out[32];
  size_t n = gen.buildBatteryTelemetry(batt, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT32(0, n);  // SBUS is unidirectional, no telemetry channel
}

static void test_max_frame_size_is_25(void) {
  SbusGenerator gen;
  TEST_ASSERT_EQUAL_UINT32(25, gen.maxFrameSize());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_round_trip_through_parser);
  RUN_TEST(test_exact_header_footer_bytes);
  RUN_TEST(test_flags_byte_reflects_setters);
  RUN_TEST(test_out_cap_24_returns_zero);
  RUN_TEST(test_battery_returns_zero);
  RUN_TEST(test_max_frame_size_is_25);
  return UNITY_END();
}
