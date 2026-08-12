// test/native/test_sbus_parser/test_sbus_parser.cpp
#include <unity.h>
#include <string.h>
#include "protocols/sbus/sbus_parser.h"

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

static size_t countByteOccurrences(const uint8_t* data, size_t len, uint8_t value) {
  size_t count = 0;
  for (size_t i = 0; i < len; i++) {
    if (data[i] == value) count++;
  }
  return count;
}

void setUp(void) {}
void tearDown(void) {}

static void test_known_frame_decodes_expected_channels(void) {
  SbusParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  raw[0] = 172;
  raw[1] = 1811;

  uint8_t frame[25];
  buildSbusFrame(raw, 0x00, frame);

  size_t n = parser.push(frame, sizeof(frame), 1000);
  TEST_ASSERT_EQUAL_UINT32(1, n);
  TEST_ASSERT_TRUE(parser.hasFrame());
  const RCFrame& f = parser.frame();
  TEST_ASSERT_EQUAL_UINT16(988, f.channels[0]);
  TEST_ASSERT_EQUAL_UINT16(2012, f.channels[1]);
  TEST_ASSERT_TRUE(f.valid);
}

static void test_frame_lost_flag_drops_lq(void) {
  SbusParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  uint8_t frame_ok[25];
  uint8_t frame_lost[25];
  buildSbusFrame(raw, 0x00, frame_ok);
  buildSbusFrame(raw, 0x04, frame_lost);  // bit2 = frame_lost

  uint32_t t = 0;
  for (int i = 0; i < 32; i++) {
    parser.push(frame_ok, sizeof(frame_ok), t);
    t += 14;
  }
  uint8_t lq_before = parser.linkQuality().lq_percent;
  TEST_ASSERT_EQUAL_UINT8(100, lq_before);

  for (int i = 0; i < 16; i++) {
    parser.push(frame_lost, sizeof(frame_lost), t);
    t += 14;
  }
  uint8_t lq_after = parser.linkQuality().lq_percent;
  TEST_ASSERT_TRUE(lq_after < lq_before);
  TEST_ASSERT_TRUE(parser.linkQuality().frame_lost);
}

static void test_failsafe_flag_sets_failsafe(void) {
  SbusParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  uint8_t frame[25];
  buildSbusFrame(raw, 0x08, frame);  // bit3 = failsafe

  parser.push(frame, sizeof(frame), 500);
  TEST_ASSERT_TRUE(parser.linkQuality().failsafe);
}

static void test_wrong_footer_rejected_and_resyncs(void) {
  SbusParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 1000;

  uint8_t bad_frame[25];
  buildSbusFrame(raw, 0x00, bad_frame);
  bad_frame[24] = 0x55;  // corrupt footer

  uint8_t good_frame[25];
  buildSbusFrame(raw, 0x00, good_frame);

  uint8_t buf[25 + 25];
  memcpy(buf, bad_frame, 25);
  memcpy(buf + 25, good_frame, 25);

  size_t n = parser.push(buf, sizeof(buf), 900);
  TEST_ASSERT_EQUAL_UINT32(1, n);  // bad frame dropped, good frame decoded after resync
  TEST_ASSERT_TRUE(parser.hasFrame());
  // This bad frame's own payload legitimately contains 0x0F bytes (verified
  // independently: raw=1000 repeated 16x packs to two spurious 0x0F bytes in
  // the 22-byte data region), so a naive resync that trusts the first 0x0F
  // sighting would attempt multiple false realignments while hunting for the
  // real boundary. Those internal attempts must not inflate crcErrors() --
  // exactly one CRC error should be recorded for this one corruption event.
  TEST_ASSERT_EQUAL_UINT32(1, parser.crcErrors());
}

// Adversarial: the corrupted frame's payload is chosen so it contains
// SEVERAL 0x0F bytes before the real next frame arrives. A resync that
// blindly jumps to the first byte equal to SBUS_HEADER and treats it as a
// legitimate frame start (rather than requiring the footer 24 bytes later to
// actually validate) would either mis-decode garbage or, at minimum, report
// an inflated/unbounded crcErrors() count as it thrashes through each false
// candidate. This verifies the count stays sane (one error per corruption
// episode) and the real subsequent frame still decodes correctly.
static void test_resync_survives_payload_with_multiple_header_byte_values(void) {
  SbusParser parser;
  // Raw channel values independently verified (via offline simulation of the
  // exact LSB-first 11-bit packing) to place 0x0F at three separate offsets
  // within the 22-byte data region of the corrupted frame.
  uint16_t raw_adv[16] = {1215, 1972, 1795, 827,  1610, 605,  727, 717,
                           271,  1301, 892,  2044, 16,   402,  1404, 120};
  uint8_t bad_frame[25];
  buildSbusFrame(raw_adv, 0x00, bad_frame);
  TEST_ASSERT_TRUE(3 <= countByteOccurrences(&bad_frame[1], 22, 0x0F));
  bad_frame[24] = 0x99;  // corrupt footer

  uint16_t raw_good[16];
  for (int i = 0; i < 16; i++) raw_good[i] = 992;
  uint8_t good_frame[25];
  buildSbusFrame(raw_good, 0x00, good_frame);

  uint8_t buf[25 + 25];
  memcpy(buf, bad_frame, 25);
  memcpy(buf + 25, good_frame, 25);

  size_t n = parser.push(buf, sizeof(buf), 700);
  TEST_ASSERT_EQUAL_UINT32(1, n);
  TEST_ASSERT_TRUE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT32(1, parser.crcErrors());
  TEST_ASSERT_EQUAL_UINT32(1, parser.framesDecoded());
  const RCFrame& f = parser.frame();
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    TEST_ASSERT_EQUAL_UINT16(1500, f.channels[i]);
  }
}

static void test_split_push_works(void) {
  SbusParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 1500 > 1811 ? 1811 : 1000;
  uint8_t frame[25];
  buildSbusFrame(raw, 0x00, frame);

  size_t n1 = parser.push(frame, 12, 300);
  TEST_ASSERT_EQUAL_UINT32(0, n1);
  size_t n2 = parser.push(frame + 12, 25 - 12, 314);
  TEST_ASSERT_EQUAL_UINT32(1, n2);
  TEST_ASSERT_TRUE(parser.hasFrame());
}

static void test_all_min_and_max_raw_clamp(void) {
  SbusParser parser;
  uint16_t raw_min[16];
  uint16_t raw_max[16];
  for (int i = 0; i < 16; i++) {
    raw_min[i] = 0;
    raw_max[i] = 2047;
  }

  uint8_t frame_min[25];
  uint8_t frame_max[25];
  buildSbusFrame(raw_min, 0x00, frame_min);
  buildSbusFrame(raw_max, 0x00, frame_max);

  parser.push(frame_min, sizeof(frame_min), 10);
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    TEST_ASSERT_EQUAL_UINT16(988, parser.frame().channels[i]);
  }

  parser.push(frame_max, sizeof(frame_max), 20);
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    TEST_ASSERT_EQUAL_UINT16(2012, parser.frame().channels[i]);
  }
}

static void test_garbage_prefix_skipped(void) {
  SbusParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  uint8_t frame[25];
  buildSbusFrame(raw, 0x00, frame);

  uint8_t buf[4 + 25];
  buf[0] = 0xAA;
  buf[1] = 0xBB;
  buf[2] = 0xCC;
  buf[3] = 0xDD;
  memcpy(buf + 4, frame, 25);

  size_t n = parser.push(buf, sizeof(buf), 600);
  TEST_ASSERT_EQUAL_UINT32(1, n);
  TEST_ASSERT_TRUE(parser.hasFrame());
}

static void test_tick_timeout_invalidates(void) {
  SbusParser parser;
  uint16_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = 992;
  uint8_t frame[25];
  buildSbusFrame(raw, 0x00, frame);

  parser.push(frame, sizeof(frame), 1000);
  TEST_ASSERT_TRUE(parser.frame().valid);

  parser.tick(1600);
  TEST_ASSERT_FALSE(parser.linkQuality().valid);
  TEST_ASSERT_TRUE(parser.linkQuality().failsafe);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_known_frame_decodes_expected_channels);
  RUN_TEST(test_frame_lost_flag_drops_lq);
  RUN_TEST(test_failsafe_flag_sets_failsafe);
  RUN_TEST(test_wrong_footer_rejected_and_resyncs);
  RUN_TEST(test_resync_survives_payload_with_multiple_header_byte_values);
  RUN_TEST(test_split_push_works);
  RUN_TEST(test_all_min_and_max_raw_clamp);
  RUN_TEST(test_garbage_prefix_skipped);
  RUN_TEST(test_tick_timeout_invalidates);
  return UNITY_END();
}
