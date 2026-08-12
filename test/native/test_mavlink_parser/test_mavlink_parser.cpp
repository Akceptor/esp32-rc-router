#include <unity.h>
#include <string.h>
#include "protocols/mavlink/mavlink_parser.h"

// Builds a valid MAVLink v2 frame into `out`, returns total bytes written.
// Non-signed, sysid/compid = MAVLINK_SYSTEM_ID/MAVLINK_COMPONENT_ID, seq = 0.
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

void setUp(void) {}
void tearDown(void) {}

static void test_rc_channels_decodes_expected_us(void) {
  MavlinkParser parser;
  uint16_t chan_us[18];
  for (int i = 0; i < 18; ++i) chan_us[i] = 1000 + i * 10;
  uint8_t payload[42];
  buildRcChannelsPayload(chan_us, 16, 200, payload);
  uint8_t frame[64];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_RC_CHANNELS, payload, 42, frame);

  size_t frames = parser.push(frame, n, 1000);

  TEST_ASSERT_EQUAL_UINT32(1, frames);
  TEST_ASSERT_TRUE(parser.hasFrame());
  for (int i = 0; i < 16; ++i) {
    TEST_ASSERT_EQUAL_UINT16(1000 + i * 10, parser.frame().channels[i]);
  }
  TEST_ASSERT_EQUAL_UINT32(1000, parser.frame().timestamp_ms);
  TEST_ASSERT_TRUE(parser.frame().valid);
  uint8_t expected_pct = (uint8_t)((uint16_t)200 * 100 / 254);
  TEST_ASSERT_EQUAL_UINT8(expected_pct, parser.linkQuality().rssi_percent);
}

static void test_bad_crc_increments_crc_errors(void) {
  MavlinkParser parser;
  uint16_t chan_us[18];
  for (int i = 0; i < 18; ++i) chan_us[i] = 1500;
  uint8_t payload[42];
  buildRcChannelsPayload(chan_us, 16, 100, payload);
  uint8_t frame[64];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_RC_CHANNELS, payload, 42, frame);
  frame[n - 1] ^= 0xFF;  // corrupt checksum high byte

  size_t frames = parser.push(frame, n, 500);

  TEST_ASSERT_EQUAL_UINT32(0, frames);
  TEST_ASSERT_EQUAL_UINT32(1, parser.crcErrors());
  TEST_ASSERT_FALSE(parser.hasFrame());
}

static void test_heartbeat_sets_alive_and_lq_100(void) {
  MavlinkParser parser;
  uint8_t payload[9];
  writeU32(&payload[0], 0);  // custom_mode
  payload[4] = 2;            // type
  payload[5] = 3;            // autopilot
  payload[6] = 0x81;         // base_mode
  payload[7] = 4;            // system_status
  payload[8] = 3;            // mavlink_version
  uint8_t frame[32];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_HEARTBEAT, payload, 9, frame);

  parser.push(frame, n, 2000);

  TEST_ASSERT_TRUE(parser.heartbeatAlive(2000, 1500));
  TEST_ASSERT_EQUAL_UINT32(2000, parser.lastHeartbeatMs());
  TEST_ASSERT_EQUAL_UINT8(0x81, parser.baseMode());
  TEST_ASSERT_EQUAL_UINT8(4, parser.systemStatus());
  TEST_ASSERT_EQUAL_UINT8(100, parser.linkQuality().lq_percent);
  TEST_ASSERT_FALSE(parser.linkQuality().failsafe);
}

static void test_heartbeat_timeout_drops_lq_and_sets_failsafe(void) {
  MavlinkParser parser;
  uint8_t payload[9];
  memset(payload, 0, sizeof(payload));
  uint8_t frame[32];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_HEARTBEAT, payload, 9, frame);

  parser.push(frame, n, 0);
  TEST_ASSERT_EQUAL_UINT8(100, parser.linkQuality().lq_percent);

  parser.tick(3500);

  TEST_ASSERT_EQUAL_UINT8(0, parser.linkQuality().lq_percent);
  TEST_ASSERT_TRUE(parser.linkQuality().failsafe);
  TEST_ASSERT_FALSE(parser.heartbeatAlive(3500, 1500));
}

static void test_rc_channels_override_zero_holds_previous(void) {
  MavlinkParser parser;
  uint16_t chan_us[18];
  for (int i = 0; i < 18; ++i) chan_us[i] = 1600;
  uint8_t payload[42];
  buildRcChannelsPayload(chan_us, 16, 100, payload);
  uint8_t frame[64];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_RC_CHANNELS, payload, 42, frame);
  parser.push(frame, n, 100);
  TEST_ASSERT_EQUAL_UINT16(1600, parser.frame().channels[3]);

  uint8_t ov_payload[38];
  memset(ov_payload, 0, sizeof(ov_payload));
  ov_payload[0] = 1;  // target_system
  ov_payload[1] = 1;  // target_component
  writeU16(&ov_payload[2 + 0 * 2], 1700);
  writeU16(&ov_payload[2 + 3 * 2], 0);  // channel 4 (index 3) held
  uint8_t ov_frame[64];
  size_t n2 = buildMavFrame(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, ov_payload, 38, ov_frame);

  size_t frames = parser.push(ov_frame, n2, 150);

  TEST_ASSERT_EQUAL_UINT32(1, frames);
  TEST_ASSERT_EQUAL_UINT16(1700, parser.frame().channels[0]);
  TEST_ASSERT_EQUAL_UINT16(1600, parser.frame().channels[3]);  // held
}

static void test_split_push_works(void) {
  MavlinkParser parser;
  uint16_t chan_us[18];
  for (int i = 0; i < 18; ++i) chan_us[i] = 1234;
  uint8_t payload[42];
  buildRcChannelsPayload(chan_us, 16, 254, payload);
  uint8_t frame[64];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_RC_CHANNELS, payload, 42, frame);

  size_t split = n / 2;
  size_t frames1 = parser.push(frame, split, 10);
  TEST_ASSERT_EQUAL_UINT32(0, frames1);
  TEST_ASSERT_FALSE(parser.hasFrame());

  size_t frames2 = parser.push(frame + split, n - split, 20);
  TEST_ASSERT_EQUAL_UINT32(1, frames2);
  TEST_ASSERT_TRUE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT16(1234, parser.frame().channels[5]);
}

static void test_battery_status_goes_to_telemetry_fifo(void) {
  MavlinkParser parser;
  uint8_t payload[36];
  memset(payload, 0, sizeof(payload));
  payload[35] = 77;  // battery_remaining percent
  uint8_t frame[64];
  size_t n = buildMavFrame(MAVLINK_MSG_ID_BATTERY_STATUS, payload, 36, frame);

  parser.push(frame, n, 500);

  TelemetryPacket pkt;
  bool got = parser.popTelemetry(pkt);
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL_INT((int)TelemetryKind::BATTERY, (int)pkt.kind);
  TEST_ASSERT_EQUAL_UINT8(36, pkt.length);
  TEST_ASSERT_EQUAL_UINT8(77, pkt.data[35]);
  TEST_ASSERT_EQUAL_UINT32(500, pkt.timestamp_ms);
}

static void test_signed_frame_is_skipped_cleanly(void) {
  MavlinkParser parser;
  uint16_t chan_us[18];
  for (int i = 0; i < 18; ++i) chan_us[i] = 1900;
  uint8_t payload[42];
  buildRcChannelsPayload(chan_us, 16, 100, payload);

  uint8_t frame[80];
  size_t idx = 0;
  frame[idx++] = MAVLINK_STX_V2;
  frame[idx++] = 42;
  frame[idx++] = MAVLINK_INCOMPAT_FLAG_SIGNED;  // incompat_flags: signed
  frame[idx++] = 0;
  frame[idx++] = 0;  // seq
  frame[idx++] = MAVLINK_SYSTEM_ID;
  frame[idx++] = MAVLINK_COMPONENT_ID;
  uint32_t msgid = MAVLINK_MSG_ID_RC_CHANNELS;
  frame[idx++] = (uint8_t)(msgid & 0xFF);
  frame[idx++] = (uint8_t)((msgid >> 8) & 0xFF);
  frame[idx++] = (uint8_t)((msgid >> 16) & 0xFF);
  for (uint8_t i = 0; i < 42; ++i) frame[idx++] = payload[i];
  uint16_t crc = mavlinkCrc16(&frame[1], (size_t)9 + 42, mavlinkCrcExtra(msgid));
  frame[idx++] = (uint8_t)(crc & 0xFF);
  frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);
  for (int i = 0; i < MAVLINK_SIGNATURE_LEN; ++i) frame[idx++] = 0xAA;  // dummy signature

  size_t frames = parser.push(frame, idx, 700);

  TEST_ASSERT_EQUAL_UINT32(0, frames);
  TEST_ASSERT_FALSE(parser.hasFrame());
  TEST_ASSERT_EQUAL_UINT32(0, parser.crcErrors());
  TEST_ASSERT_EQUAL_UINT32(0, parser.framesDecoded());

  // Stream must resync: a following well-formed frame still decodes.
  uint8_t frame2[64];
  size_t n2 = buildMavFrame(MAVLINK_MSG_ID_RC_CHANNELS, payload, 42, frame2);
  size_t frames2 = parser.push(frame2, n2, 800);
  TEST_ASSERT_EQUAL_UINT32(1, frames2);
  TEST_ASSERT_TRUE(parser.hasFrame());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_rc_channels_decodes_expected_us);
  RUN_TEST(test_bad_crc_increments_crc_errors);
  RUN_TEST(test_heartbeat_sets_alive_and_lq_100);
  RUN_TEST(test_heartbeat_timeout_drops_lq_and_sets_failsafe);
  RUN_TEST(test_rc_channels_override_zero_holds_previous);
  RUN_TEST(test_split_push_works);
  RUN_TEST(test_battery_status_goes_to_telemetry_fifo);
  RUN_TEST(test_signed_frame_is_skipped_cleanly);
  return UNITY_END();
}
