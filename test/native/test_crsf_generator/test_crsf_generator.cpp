// test/native/test_crsf_generator/test_crsf_generator.cpp
#include <unity.h>
#include <string.h>
#include "protocols/crsf/crsf_generator.h"
#include "protocols/crsf/crsf_parser.h"

void setUp(void) {}
void tearDown(void) {}

static void test_build_rc_frame_exact_bytes(void) {
  CrsfGenerator gen;
  RCFrame f;
  rcFrameInit(f);
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) f.channels[i] = 1500;
  f.valid = true;
  f.timestamp_ms = 42;

  uint8_t out[64];
  size_t n = gen.buildRcFrame(f, out, sizeof(out));

  TEST_ASSERT_EQUAL_UINT32(26, n);
  TEST_ASSERT_EQUAL_HEX8(0xC8, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0x18, out[1]);  // 24 = type(1)+payload(22)+crc(1)
  TEST_ASSERT_EQUAL_HEX8(0x16, out[2]);

  uint8_t expected_crc = crsfCrc8(&out[2], 23);
  TEST_ASSERT_EQUAL_HEX8(expected_crc, out[25]);
}

static void test_round_trip_channels_within_1us(void) {
  CrsfGenerator gen;
  RCFrame f;
  rcFrameInit(f);
  uint16_t values[RC_CHANNEL_COUNT] = {988, 1000, 1200, 1500, 1700, 1900, 2012, 988,
                                        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) f.channels[i] = values[i];
  f.valid = true;

  uint8_t out[64];
  size_t n = gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_TRUE(n > 0);

  CrsfParser parser;
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

static void test_out_cap_too_small_returns_zero(void) {
  CrsfGenerator gen;
  RCFrame f;
  rcFrameInit(f);
  f.valid = true;
  uint8_t out[10];  // too small for a 26-byte frame
  size_t n = gen.buildRcFrame(f, out, sizeof(out));
  TEST_ASSERT_EQUAL_UINT32(0, n);
}

static void test_max_frame_size_is_64(void) {
  CrsfGenerator gen;
  TEST_ASSERT_EQUAL_UINT32(64, gen.maxFrameSize());
}

static void test_battery_telemetry_byte_layout(void) {
  CrsfGenerator gen;
  BatteryTelemetry batt;
  batt.voltage_dv = 1230;         // 12.30 V
  batt.current_da = 550;          // 5.5 A
  batt.used_capacity_mah = 1200;  // mAh
  batt.remaining_percent = 67;

  uint8_t out[16];
  size_t n = gen.buildBatteryTelemetry(batt, out, sizeof(out));

  // addr(1)+len(1)+type(1)+payload(8: voltage2+current2+capacity3+percent1)+crc(1) = 12.
  // NOTE: the brief's literal test code asserted 11 here, which contradicts its own
  // out[1]==0x0A (len=10 meaning 10 bytes follow the length byte: type+8 payload+crc)
  // and the payload field offsets checked below (out[3]..out[10], 8 bytes). Corrected
  // to the protocol-correct value of 12, consistent with the CRC window convention and
  // the round-trip test (which passes using the generator's actual returned length).
  TEST_ASSERT_EQUAL_UINT32(12, n);
  TEST_ASSERT_EQUAL_HEX8(0xC8, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0x0A, out[1]);  // len = 10: type(1)+payload(8)+crc(1)... see below
  TEST_ASSERT_EQUAL_HEX8(0x08, out[2]);

  uint16_t voltage_be = (uint16_t)((out[3] << 8) | out[4]);
  TEST_ASSERT_EQUAL_UINT16(1230, voltage_be);

  uint16_t current_be = (uint16_t)((out[5] << 8) | out[6]);
  TEST_ASSERT_EQUAL_UINT16(550, current_be);

  uint32_t capacity_be = ((uint32_t)out[7] << 16) | ((uint32_t)out[8] << 8) | out[9];
  TEST_ASSERT_EQUAL_UINT32(1200, capacity_be);

  TEST_ASSERT_EQUAL_UINT8(67, out[10 - 1 + 1]);  // remaining percent byte, see impl layout
}

static void test_round_trip_battery_through_parser(void) {
  CrsfGenerator gen;
  BatteryTelemetry batt;
  batt.voltage_dv = 1650;
  batt.current_da = 220;
  batt.used_capacity_mah = 900;
  batt.remaining_percent = 40;

  uint8_t out[16];
  size_t n = gen.buildBatteryTelemetry(batt, out, sizeof(out));
  TEST_ASSERT_TRUE(n > 0);

  CrsfParser parser;
  size_t decoded = parser.push(out, n, 55);
  TEST_ASSERT_EQUAL_UINT32(0, decoded);  // battery telemetry is not an RC frame

  TelemetryPacket pkt;
  bool got = parser.popTelemetry(pkt);
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL(TelemetryKind::BATTERY, pkt.kind);
  TEST_ASSERT_EQUAL_UINT8((uint8_t)n, pkt.length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(out, pkt.data, n);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_build_rc_frame_exact_bytes);
  RUN_TEST(test_round_trip_channels_within_1us);
  RUN_TEST(test_out_cap_too_small_returns_zero);
  RUN_TEST(test_max_frame_size_is_64);
  RUN_TEST(test_battery_telemetry_byte_layout);
  RUN_TEST(test_round_trip_battery_through_parser);
  return UNITY_END();
}
