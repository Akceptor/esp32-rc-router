#include <unity.h>
#include "output/channel_mapper.h"

void setUp(void) {}
void tearDown(void) {}

static void test_identity_passthrough(void) {
  ChannelMapper mapper;
  RCFrame in;
  rcFrameInit(in);
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) in.channels[i] = (uint16_t)(1000 + i);
  in.valid = true;
  in.timestamp_ms = 42;

  RCFrame out;
  rcFrameInit(out);
  mapper.apply(in, out);

  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) {
    TEST_ASSERT_EQUAL_UINT16(in.channels[i], out.channels[i]);
  }
}

static void test_output1_remaps_to_channel7(void) {
  ChannelMapper mapper;
  uint8_t map[RC_CHANNEL_COUNT];
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) map[i] = (uint8_t)i;
  map[1] = 6;  // output channel index 1 (0-based) <- input channel index 6 ("channel 7")
  mapper.setMap(map);

  RCFrame in;
  rcFrameInit(in);
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) in.channels[i] = (uint16_t)(1000 + i * 10);

  RCFrame out;
  rcFrameInit(out);
  mapper.apply(in, out);

  TEST_ASSERT_EQUAL_UINT16(in.channels[6], out.channels[1]);
  TEST_ASSERT_EQUAL_UINT8(6, mapper.mapping(1));
}

static void test_duplicate_source_channels_allowed(void) {
  ChannelMapper mapper;
  uint8_t map[RC_CHANNEL_COUNT];
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) map[i] = 2;  // every output reads input channel 2
  mapper.setMap(map);

  RCFrame in;
  rcFrameInit(in);
  in.channels[2] = 1777;

  RCFrame out;
  rcFrameInit(out);
  mapper.apply(in, out);

  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) {
    TEST_ASSERT_EQUAL_UINT16(1777, out.channels[i]);
  }
}

static void test_invalid_index_falls_back_to_identity(void) {
  ChannelMapper mapper;
  uint8_t map[RC_CHANNEL_COUNT];
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) map[i] = (uint8_t)i;
  map[4] = 20;  // out of range (>= RC_CHANNEL_COUNT)
  mapper.setMap(map);

  TEST_ASSERT_EQUAL_UINT8(4, mapper.mapping(4));  // falls back to identity for this slot
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) {
    if (i != 4) {
      TEST_ASSERT_EQUAL_UINT8((uint8_t)i, mapper.mapping((uint8_t)i));
    }
  }
}

static void test_apply_preserves_timestamp_and_valid(void) {
  ChannelMapper mapper;
  RCFrame in;
  rcFrameInit(in);
  in.timestamp_ms = 98765;
  in.valid = true;

  RCFrame out;
  rcFrameInit(out);
  mapper.apply(in, out);

  TEST_ASSERT_EQUAL_UINT32(98765, out.timestamp_ms);
  TEST_ASSERT_TRUE(out.valid);

  in.valid = false;
  mapper.apply(in, out);
  TEST_ASSERT_FALSE(out.valid);
}

static void test_full_reverse_map(void) {
  ChannelMapper mapper;
  uint8_t map[RC_CHANNEL_COUNT];
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) map[i] = (uint8_t)(RC_CHANNEL_COUNT - 1 - i);
  mapper.setMap(map);

  RCFrame in;
  rcFrameInit(in);
  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) in.channels[i] = (uint16_t)(1000 + i);

  RCFrame out;
  rcFrameInit(out);
  mapper.apply(in, out);

  for (int i = 0; i < RC_CHANNEL_COUNT; ++i) {
    TEST_ASSERT_EQUAL_UINT16(in.channels[RC_CHANNEL_COUNT - 1 - i], out.channels[i]);
  }
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_identity_passthrough);
  RUN_TEST(test_output1_remaps_to_channel7);
  RUN_TEST(test_duplicate_source_channels_allowed);
  RUN_TEST(test_invalid_index_falls_back_to_identity);
  RUN_TEST(test_apply_preserves_timestamp_and_valid);
  RUN_TEST(test_full_reverse_map);
  return UNITY_END();
}
