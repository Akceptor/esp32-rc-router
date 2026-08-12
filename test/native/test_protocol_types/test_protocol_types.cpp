#include <unity.h>
#include "protocols/protocol_types.h"

void setUp(void) {}
void tearDown(void) {}

void test_rcFrameInit_gives_all_mid_and_invalid(void) {
  RCFrame f;
  f.valid = true;
  f.timestamp_ms = 999;
  rcFrameInit(f);
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    TEST_ASSERT_EQUAL_UINT16(RC_PULSE_MID_US, f.channels[i]);
  }
  TEST_ASSERT_FALSE(f.valid);
}

void test_clampPulseUs_clamps_below_above_and_in_range(void) {
  TEST_ASSERT_EQUAL_UINT16(RC_PULSE_MIN_US, clampPulseUs(0));
  TEST_ASSERT_EQUAL_UINT16(RC_PULSE_MIN_US, clampPulseUs(-500));
  TEST_ASSERT_EQUAL_UINT16(RC_PULSE_MIN_US, clampPulseUs(500));
  TEST_ASSERT_EQUAL_UINT16(RC_PULSE_MAX_US, clampPulseUs(3000));
  TEST_ASSERT_EQUAL_UINT16(1500, clampPulseUs(1500));
  TEST_ASSERT_EQUAL_UINT16(RC_PULSE_MIN_US, clampPulseUs(RC_PULSE_MIN_US));
  TEST_ASSERT_EQUAL_UINT16(RC_PULSE_MAX_US, clampPulseUs(RC_PULSE_MAX_US));
}

void test_rcframe_sizeof_sanity(void) {
  RCFrame f;
  TEST_ASSERT_TRUE(sizeof(f.channels) == RC_CHANNEL_COUNT * sizeof(uint16_t));
  TEST_ASSERT_TRUE(sizeof(RCFrame) >= sizeof(f.channels) + sizeof(f.timestamp_ms) + sizeof(bool));
}

void test_linkQualityInit_zeroes(void) {
  LinkQuality lq;
  lq.rssi_percent = 50;
  lq.lq_percent = 50;
  lq.rssi_dbm = -70;
  lq.frame_lost = true;
  lq.failsafe = true;
  lq.last_frame_ms = 1234;
  lq.frames_received = 10;
  lq.crc_errors = 3;
  lq.valid = true;

  linkQualityInit(lq);

  TEST_ASSERT_EQUAL_UINT8(0, lq.rssi_percent);
  TEST_ASSERT_EQUAL_UINT8(0, lq.lq_percent);
  TEST_ASSERT_EQUAL_INT16(0, lq.rssi_dbm);
  TEST_ASSERT_FALSE(lq.frame_lost);
  TEST_ASSERT_FALSE(lq.failsafe);
  TEST_ASSERT_EQUAL_UINT32(0, lq.last_frame_ms);
  TEST_ASSERT_EQUAL_UINT32(0, lq.frames_received);
  TEST_ASSERT_EQUAL_UINT32(0, lq.crc_errors);
  TEST_ASSERT_FALSE(lq.valid);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_rcFrameInit_gives_all_mid_and_invalid);
  RUN_TEST(test_clampPulseUs_clamps_below_above_and_in_range);
  RUN_TEST(test_rcframe_sizeof_sanity);
  RUN_TEST(test_linkQualityInit_zeroes);
  return UNITY_END();
}
