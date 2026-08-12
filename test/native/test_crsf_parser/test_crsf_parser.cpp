// test/native/test_crsf_parser/test_crsf_parser.cpp
#include <unity.h>
#include <string.h>
#include "protocols/crsf/crsf_parser.h"

// Packs 16 channel values (11-bit each, 0..2047) LSB-first into 22 bytes,
// matching the CRSF RC_CHANNELS_PACKED wire layout.
static void packChannels11(const uint16_t raw[16], uint8_t out[22]) {
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

static void buildRcChannelsFrame(const uint16_t raw[16], uint8_t out[26]) {
  out[0] = 0xC8;  // sync
  out[1] = 24;    // length: type(1) + payload(22) + crc(1) = 24
  out[2] = 0x16;  // type
  uint8_t payload[22];
  packChannels11(raw, payload);
  memcpy(&out[3], payload, 22);
  uint8_t crc = crsfCrc8(&out[2], 23);  // type + payload
  out[25] = crc;
}

void setUp(void) {}
void tearDown(void) {}

static void test_valid_rc_frame_decodes_known_channels(void) {
  CrsfParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;  // midpoint-ish raw value
  raw[0] = 172;   // -> 988 us
  raw[1] = 1811;  // -> 2012 us
  raw[2] = 992;   // -> approx mid

  uint8_t frame[26];
  buildRcChannelsFrame(raw, frame);

  size_t n = parser.push(frame, sizeof(frame), 1000);
  TEST_ASSERT_EQUAL_UINT32(1, n);
  TEST_ASSERT_TRUE(parser.hasFrame());
  const RCFrame& f = parser.frame();
  TEST_ASSERT_TRUE(f.valid);
  TEST_ASSERT_EQUAL_UINT16(988, f.channels[0]);
  TEST_ASSERT_EQUAL_UINT16(2012, f.channels[1]);
  TEST_ASSERT_EQUAL_UINT32(1000, f.timestamp_ms);
  TEST_ASSERT_EQUAL_UINT32(1, parser.framesDecoded());
}

static void test_bad_crc_increments_errors_no_frame(void) {
  CrsfParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  uint8_t frame[26];
  buildRcChannelsFrame(raw, frame);
  frame[25] ^= 0xFF;  // corrupt CRC

  size_t n = parser.push(frame, sizeof(frame), 1000);
  TEST_ASSERT_EQUAL_UINT32(0, n);
  TEST_ASSERT_FALSE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT32(1, parser.crcErrors());
}

static void test_split_across_two_pushes_still_decodes(void) {
  CrsfParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 1000;
  uint8_t frame[26];
  buildRcChannelsFrame(raw, frame);

  size_t n1 = parser.push(frame, 10, 500);
  TEST_ASSERT_EQUAL_UINT32(0, n1);
  size_t n2 = parser.push(frame + 10, sizeof(frame) - 10, 600);
  TEST_ASSERT_EQUAL_UINT32(1, n2);
  TEST_ASSERT_TRUE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT32(600, parser.frame().timestamp_ms);
}

static void test_garbage_before_sync_skipped(void) {
  CrsfParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 900;
  uint8_t frame[26];
  buildRcChannelsFrame(raw, frame);

  uint8_t buf[5 + 26];
  buf[0] = 0x01;
  buf[1] = 0x02;
  buf[2] = 0xFF;
  buf[3] = 0x00;
  buf[4] = 0xC8;  // false sync byte with no valid follow-up length would be handled too,
                  // but here we just prepend true garbage before the real frame.
  memcpy(&buf[5], frame, 26);

  size_t n = parser.push(buf, sizeof(buf), 700);
  TEST_ASSERT_EQUAL_UINT32(1, n);
  TEST_ASSERT_TRUE(parser.hasFrame());
}

static void test_link_statistics_updates_rssi_lq(void) {
  CrsfParser parser;
  uint8_t frame[14];
  frame[0] = 0xC8;
  frame[1] = 12;    // type(1) + payload(10) + crc(1)
  frame[2] = 0x14;  // LINK_STATISTICS
  frame[3] = 70;    // uplink_rssi_ant1 raw (dBm = -70)
  frame[4] = 0;     // uplink_rssi_ant2
  frame[5] = 80;    // uplink_link_quality (percent)
  frame[6] = 0;     // uplink_snr
  frame[7] = 0;     // active_antenna
  frame[8] = 0;     // rf_mode
  frame[9] = 0;     // uplink_tx_power
  frame[10] = 0;    // downlink_rssi
  frame[11] = 0;    // downlink_link_quality
  frame[12] = 0;    // downlink_snr
  uint8_t crc = crsfCrc8(&frame[2], 10);
  frame[13] = crc;

  size_t n = parser.push(frame, sizeof(frame), 100);
  TEST_ASSERT_EQUAL_UINT32(0, n);  // link stats is not an RC frame
  const LinkQuality& lq = parser.linkQuality();
  TEST_ASSERT_EQUAL_UINT8(80, lq.lq_percent);
  TEST_ASSERT_EQUAL_INT16(-70, lq.rssi_dbm);
  TEST_ASSERT_FALSE(lq.failsafe);
}

static void test_lq_zero_sets_failsafe(void) {
  CrsfParser parser;
  uint8_t frame[14];
  frame[0] = 0xC8;
  frame[1] = 12;
  frame[2] = 0x14;
  frame[3] = 120;
  frame[4] = 0;
  frame[5] = 0;  // lq = 0
  frame[6] = 0;
  frame[7] = 0;
  frame[8] = 0;
  frame[9] = 0;
  frame[10] = 0;
  frame[11] = 0;
  frame[12] = 0;
  uint8_t crc = crsfCrc8(&frame[2], 10);
  frame[13] = crc;

  parser.push(frame, sizeof(frame), 200);
  TEST_ASSERT_TRUE(parser.linkQuality().failsafe);
  TEST_ASSERT_EQUAL_UINT8(0, parser.linkQuality().lq_percent);
}

static void test_tick_timeout_invalidates(void) {
  CrsfParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  uint8_t frame[26];
  buildRcChannelsFrame(raw, frame);

  parser.push(frame, sizeof(frame), 1000);
  TEST_ASSERT_TRUE(parser.frame().valid);

  parser.tick(1400);  // 400 ms elapsed, within timeout
  TEST_ASSERT_TRUE(parser.linkQuality().valid);

  parser.tick(1600);  // 600 ms elapsed, past 500 ms timeout
  TEST_ASSERT_FALSE(parser.linkQuality().valid);
  TEST_ASSERT_TRUE(parser.linkQuality().failsafe);
}

static void test_telemetry_frame_lands_in_pop_and_fifo_empties(void) {
  CrsfParser parser;
  uint8_t frame[11];
  frame[0] = 0xC8;
  frame[1] = 9;     // type(1) + payload(7) + crc(1)
  frame[2] = 0x08;  // BATTERY_SENSOR
  frame[3] = 0x00;  // voltage hi
  frame[4] = 0x64;  // voltage lo -> 100 (0.1V units big-endian) = 10.0V
  frame[5] = 0x00;  // current hi
  frame[6] = 0x0A;  // current lo
  frame[7] = 0x00;  // capacity byte0
  frame[8] = 0x00;  // capacity byte1
  frame[9] = 0x05;  // capacity byte2 / remaining percent depending on layout
  uint8_t crc = crsfCrc8(&frame[2], 7);
  frame[10] = crc;

  size_t n = parser.push(frame, sizeof(frame), 300);
  TEST_ASSERT_EQUAL_UINT32(0, n);

  TelemetryPacket pkt;
  bool got = parser.popTelemetry(pkt);
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL(TelemetryKind::BATTERY, pkt.kind);
  TEST_ASSERT_EQUAL_UINT32(300, pkt.timestamp_ms);
  TEST_ASSERT_EQUAL_UINT8(11, pkt.length);  // raw frame bytes copied verbatim

  bool got2 = parser.popTelemetry(pkt);
  TEST_ASSERT_FALSE(got2);  // FIFO now empty
}

static void test_oversized_length_rejected_no_desync(void) {
  CrsfParser parser;
  uint8_t bad[3];
  bad[0] = 0xC8;
  bad[1] = 200;  // invalid length, must be 2..62
  bad[2] = 0x16;

  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  uint8_t good[26];
  buildRcChannelsFrame(raw, good);

  uint8_t buf[3 + 26];
  memcpy(buf, bad, 3);
  memcpy(buf + 3, good, 26);

  size_t n = parser.push(buf, sizeof(buf), 400);
  TEST_ASSERT_EQUAL_UINT32(1, n);  // the parser recovers and decodes the valid frame after
  TEST_ASSERT_TRUE(parser.hasFrame());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_valid_rc_frame_decodes_known_channels);
  RUN_TEST(test_bad_crc_increments_errors_no_frame);
  RUN_TEST(test_split_across_two_pushes_still_decodes);
  RUN_TEST(test_garbage_before_sync_skipped);
  RUN_TEST(test_link_statistics_updates_rssi_lq);
  RUN_TEST(test_lq_zero_sets_failsafe);
  RUN_TEST(test_tick_timeout_invalidates);
  RUN_TEST(test_telemetry_frame_lands_in_pop_and_fifo_empties);
  RUN_TEST(test_oversized_length_rejected_no_desync);
  return UNITY_END();
}
