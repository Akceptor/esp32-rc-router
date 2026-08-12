#include "protocols/mavlink/mavlink_generator.h"
#include <string.h>
#include <stdint.h>

static const size_t MAVLINK_MAX_FRAME_SIZE = 280;  // STX(1)+header(9)+payload(255)+crc(2)+sig(13)

size_t mavlinkTrimTrailingZeros(uint8_t* payload, size_t len) {
  while (len > 0 && payload[len - 1] == 0) {
    len--;
  }
  return len;
}

MavlinkGenerator::MavlinkGenerator()
    : target_system_(1), target_component_(1), seq_(0) {}

void MavlinkGenerator::setTarget(uint8_t sysid, uint8_t compid) {
  target_system_ = sysid;
  target_component_ = compid;
}

size_t MavlinkGenerator::maxFrameSize() const {
  return MAVLINK_MAX_FRAME_SIZE;
}

size_t MavlinkGenerator::buildFrame(uint32_t msgid, const uint8_t* payload, uint8_t payload_len,
                                     uint8_t* out, size_t out_cap) {
  size_t total = (size_t)1 + 9 + payload_len + 2;
  if (out == nullptr || out_cap < total) {
    return 0;
  }
  size_t idx = 0;
  out[idx++] = MAVLINK_STX_V2;
  out[idx++] = payload_len;
  out[idx++] = 0;  // incompat_flags (unsigned)
  out[idx++] = 0;  // compat_flags
  out[idx++] = seq_;
  out[idx++] = MAVLINK_SYSTEM_ID;
  out[idx++] = MAVLINK_COMPONENT_ID;
  out[idx++] = (uint8_t)(msgid & 0xFF);
  out[idx++] = (uint8_t)((msgid >> 8) & 0xFF);
  out[idx++] = (uint8_t)((msgid >> 16) & 0xFF);
  for (uint8_t i = 0; i < payload_len; ++i) {
    out[idx++] = payload[i];
  }
  uint16_t crc = mavlinkCrc16(&out[1], (size_t)9 + payload_len, mavlinkCrcExtra(msgid));
  out[idx++] = (uint8_t)(crc & 0xFF);
  out[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  seq_ = (uint8_t)(seq_ + 1);  // wraps 255 -> 0 naturally
  return idx;
}

size_t MavlinkGenerator::buildRcFrame(const RCFrame& frame, uint8_t* out, size_t out_cap) {
  uint8_t payload[38];
  payload[0] = target_system_;
  payload[1] = target_component_;
  for (uint8_t i = 0; i < 18; ++i) {
    uint16_t v = (i < RC_CHANNEL_COUNT) ? frame.channels[i] : 0;
    payload[2 + i * 2] = (uint8_t)(v & 0xFF);
    payload[3 + i * 2] = (uint8_t)((v >> 8) & 0xFF);
  }
  uint8_t len = (uint8_t)mavlinkTrimTrailingZeros(payload, sizeof(payload));
  return buildFrame(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, payload, len, out, out_cap);
}

size_t MavlinkGenerator::buildBatteryTelemetry(const BatteryTelemetry& batt, uint8_t* out,
                                                size_t out_cap) {
  uint8_t payload[36];
  memset(payload, 0, sizeof(payload));

  payload[0] = 0;  // id
  payload[1] = 0;  // battery_function: MAV_BATTERY_FUNCTION_UNKNOWN
  payload[2] = 0;  // type: MAV_BATTERY_TYPE_UNKNOWN

  int16_t temperature = INT16_MAX;  // sentinel: unknown
  payload[3] = (uint8_t)(temperature & 0xFF);
  payload[4] = (uint8_t)((temperature >> 8) & 0xFF);

  uint16_t voltage_mv = (uint16_t)(batt.voltage_dv * 10);  // decivolts -> millivolts
  uint16_t voltages[10];
  voltages[0] = voltage_mv;
  for (int i = 1; i < 10; ++i) {
    voltages[i] = 0xFFFF;  // sentinel: cell not present
  }
  for (int i = 0; i < 10; ++i) {
    payload[5 + i * 2] = (uint8_t)(voltages[i] & 0xFF);
    payload[6 + i * 2] = (uint8_t)((voltages[i] >> 8) & 0xFF);
  }

  int16_t current_ca = (int16_t)(batt.current_da * 10);  // deciamps -> centiamps
  payload[25] = (uint8_t)(current_ca & 0xFF);
  payload[26] = (uint8_t)((current_ca >> 8) & 0xFF);

  int32_t current_consumed = -1;  // sentinel: unknown
  payload[27] = (uint8_t)(current_consumed & 0xFF);
  payload[28] = (uint8_t)((current_consumed >> 8) & 0xFF);
  payload[29] = (uint8_t)((current_consumed >> 16) & 0xFF);
  payload[30] = (uint8_t)((current_consumed >> 24) & 0xFF);

  int32_t energy_consumed = -1;  // sentinel: unknown
  payload[31] = (uint8_t)(energy_consumed & 0xFF);
  payload[32] = (uint8_t)((energy_consumed >> 8) & 0xFF);
  payload[33] = (uint8_t)((energy_consumed >> 16) & 0xFF);
  payload[34] = (uint8_t)((energy_consumed >> 24) & 0xFF);

  payload[35] = batt.remaining_percent;

  uint8_t len = (uint8_t)mavlinkTrimTrailingZeros(payload, sizeof(payload));
  return buildFrame(MAVLINK_MSG_ID_BATTERY_STATUS, payload, len, out, out_cap);
}
