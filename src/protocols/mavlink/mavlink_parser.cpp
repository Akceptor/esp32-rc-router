#include "protocols/mavlink/mavlink_parser.h"
#include <string.h>

uint8_t mavlinkCrcExtra(uint32_t msgid) {
  switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: return 50;
    case MAVLINK_MSG_ID_SYS_STATUS: return 124;
    case MAVLINK_MSG_ID_RC_CHANNELS: return 118;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: return 124;
    case MAVLINK_MSG_ID_BATTERY_STATUS: return 154;
    default: return 0;
  }
}

void mavlinkCrcAccumulate(uint8_t data, uint16_t& crc) {
  uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
  tmp ^= (uint8_t)(tmp << 4);
  crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

uint16_t mavlinkCrc16(const uint8_t* buf, size_t len, uint8_t crc_extra) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    mavlinkCrcAccumulate(buf[i], crc);
  }
  mavlinkCrcAccumulate(crc_extra, crc);
  return crc;
}

MavlinkParser::MavlinkParser() {
  reset();
}

void MavlinkParser::reset() {
  state_ = State::WAIT_STX;
  header_idx_ = 0;
  payload_len_ = 0;
  payload_idx_ = 0;
  crc_idx_ = 0;
  sig_idx_ = 0;
  incompat_flags_ = 0;
  seq_ = 0;
  sysid_ = 0;
  compid_ = 0;
  msgid_ = 0;

  rcFrameInit(frame_);
  linkQualityInit(lq_);
  has_frame_ = false;

  last_heartbeat_ms_ = 0;
  heartbeat_seen_ = false;
  base_mode_ = 0;
  system_status_ = 0;

  telemetry_head_ = 0;
  telemetry_count_ = 0;

  crc_errors_ = 0;
  frames_decoded_ = 0;
}

uint16_t MavlinkParser::readPayloadU16(uint32_t offset) const {
  return (uint16_t)payload_[offset] | ((uint16_t)payload_[offset + 1] << 8);
}

size_t MavlinkParser::push(const uint8_t* data, size_t len, uint32_t now_ms) {
  size_t frames = 0;
  for (size_t i = 0; i < len; ++i) {
    if (processByte(data[i], now_ms)) {
      frames++;
    }
  }
  updateHeartbeatFreshness(now_ms);
  return frames;
}

bool MavlinkParser::processByte(uint8_t b, uint32_t now_ms) {
  switch (state_) {
    case State::WAIT_STX:
      if (b == MAVLINK_STX_V2) {
        state_ = State::WAIT_LEN;
      }
      return false;

    case State::WAIT_LEN:
      payload_len_ = b;
      header_buf_[0] = b;
      header_idx_ = 1;
      state_ = State::WAIT_HEADER;
      return false;

    case State::WAIT_HEADER:
      header_buf_[header_idx_++] = b;
      if (header_idx_ == 9) {
        incompat_flags_ = header_buf_[1];
        seq_ = header_buf_[3];
        sysid_ = header_buf_[4];
        compid_ = header_buf_[5];
        msgid_ = (uint32_t)header_buf_[6] | ((uint32_t)header_buf_[7] << 8) |
                 ((uint32_t)header_buf_[8] << 16);
        payload_idx_ = 0;
        if (payload_len_ == 0) {
          state_ = State::WAIT_CRC;
          crc_idx_ = 0;
        } else {
          state_ = State::WAIT_PAYLOAD;
        }
      }
      return false;

    case State::WAIT_PAYLOAD:
      payload_[payload_idx_++] = b;
      if (payload_idx_ >= payload_len_) {
        state_ = State::WAIT_CRC;
        crc_idx_ = 0;
      }
      return false;

    case State::WAIT_CRC:
      crc_buf_[crc_idx_++] = b;
      if (crc_idx_ == 2) {
        if (incompat_flags_ & MAVLINK_INCOMPAT_FLAG_SIGNED) {
          sig_idx_ = 0;
          state_ = State::WAIT_SIGNATURE;
          return false;
        }
        return finishFrame(now_ms);
      }
      return false;

    case State::WAIT_SIGNATURE:
      sig_idx_++;
      if (sig_idx_ >= MAVLINK_SIGNATURE_LEN) {
        // Signed frames are rejected outright: consume the signature to
        // resync the stream, but never decode or count them.
        state_ = State::WAIT_STX;
      }
      return false;
  }
  return false;
}

bool MavlinkParser::finishFrame(uint32_t now_ms) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < 9; ++i) {
    mavlinkCrcAccumulate(header_buf_[i], crc);
  }
  for (uint8_t i = 0; i < payload_len_; ++i) {
    mavlinkCrcAccumulate(payload_[i], crc);
  }
  mavlinkCrcAccumulate(mavlinkCrcExtra(msgid_), crc);

  uint16_t received = (uint16_t)crc_buf_[0] | ((uint16_t)crc_buf_[1] << 8);
  state_ = State::WAIT_STX;

  if (crc != received) {
    crc_errors_++;
    return false;
  }
  return handleCompleteFrame(now_ms);
}

bool MavlinkParser::handleCompleteFrame(uint32_t now_ms) {
  if (msgid_ == MAVLINK_MSG_ID_HEARTBEAT) {
    decodeHeartbeat(now_ms);
    return false;
  }
  if (msgid_ == MAVLINK_MSG_ID_RC_CHANNELS) {
    decodeRcChannels(now_ms);
    frames_decoded_++;
    return true;
  }
  if (msgid_ == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
    decodeRcChannelsOverride(now_ms);
    frames_decoded_++;
    return true;
  }
  if (msgid_ == MAVLINK_MSG_ID_BATTERY_STATUS || msgid_ == MAVLINK_MSG_ID_SYS_STATUS) {
    pushTelemetry(TelemetryKind::BATTERY, payload_, payload_len_, now_ms);
    return false;
  }
  pushTelemetry(TelemetryKind::PASSTHROUGH, payload_, payload_len_, now_ms);
  return false;
}

void MavlinkParser::decodeHeartbeat(uint32_t now_ms) {
  if (payload_len_ >= 7) {
    base_mode_ = payload_[6];
  }
  if (payload_len_ >= 8) {
    system_status_ = payload_[7];
  }
  last_heartbeat_ms_ = now_ms;
  heartbeat_seen_ = true;
  updateHeartbeatFreshness(now_ms);
}

void MavlinkParser::decodeRcChannels(uint32_t now_ms) {
  if (payload_len_ < 42) {
    return;  // malformed for this dialect subset, ignore safely
  }
  const uint32_t offset = 4;  // skip time_boot_ms (u32)
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; ++i) {
    uint16_t v = readPayloadU16(offset + i * 2);
    if (v != 0 && v != 0xFFFF) {
      frame_.channels[i] = clampPulseUs((int32_t)v);
    }
  }
  uint8_t rssi = payload_[41];
  uint16_t rssi_pct = (uint16_t)((uint32_t)rssi * 100 / 254);
  if (rssi_pct > 100) rssi_pct = 100;
  lq_.rssi_percent = (uint8_t)rssi_pct;
  lq_.rssi_dbm = (int16_t)(-120 + (int32_t)lq_.rssi_percent * 70 / 100);

  frame_.timestamp_ms = now_ms;
  frame_.valid = true;
  lq_.last_frame_ms = now_ms;
  lq_.frames_received++;
  lq_.valid = true;
  has_frame_ = true;
}

void MavlinkParser::decodeRcChannelsOverride(uint32_t now_ms) {
  if (payload_len_ < 2) {
    return;
  }
  const uint32_t offset = 2;  // skip target_system, target_component
  uint8_t available_bytes = (uint8_t)(payload_len_ - 2);
  uint8_t channel_count = (uint8_t)(available_bytes / 2);
  if (channel_count > 18) channel_count = 18;

  for (uint8_t i = 0; i < channel_count && i < RC_CHANNEL_COUNT; ++i) {
    uint16_t v = readPayloadU16(offset + i * 2);
    if (v != 0 && v != 0xFFFF) {
      frame_.channels[i] = clampPulseUs((int32_t)v);
    }
  }
  frame_.timestamp_ms = now_ms;
  frame_.valid = true;
  lq_.last_frame_ms = now_ms;
  lq_.frames_received++;
  lq_.valid = true;
  has_frame_ = true;
}

void MavlinkParser::updateHeartbeatFreshness(uint32_t now_ms) {
  if (!heartbeat_seen_) {
    lq_.lq_percent = 0;
    lq_.failsafe = true;
    return;
  }
  uint32_t age = now_ms - last_heartbeat_ms_;
  if (age <= 1500) {
    lq_.lq_percent = 100;
    lq_.failsafe = false;
  } else if (age >= 3000) {
    lq_.lq_percent = 0;
    lq_.failsafe = true;
  } else {
    uint32_t span = age - 1500;
    lq_.lq_percent = (uint8_t)(100 - (span * 100) / 1500);
    lq_.failsafe = false;
  }
}

void MavlinkParser::pushTelemetry(TelemetryKind kind, const uint8_t* data, uint8_t len,
                                   uint32_t now_ms) {
  if (telemetry_count_ == MAVLINK_TELEMETRY_RING) {
    // Ring full: drop the oldest entry to make room (documented overwrite policy).
    telemetry_head_ = (uint8_t)((telemetry_head_ + 1) % MAVLINK_TELEMETRY_RING);
    telemetry_count_--;
  }
  uint8_t tail = (uint8_t)((telemetry_head_ + telemetry_count_) % MAVLINK_TELEMETRY_RING);
  TelemetryPacket& pkt = telemetry_ring_[tail];
  pkt.kind = kind;
  uint8_t copy_len = len;
  if (copy_len > TELEMETRY_MAX_PAYLOAD) copy_len = TELEMETRY_MAX_PAYLOAD;
  memcpy(pkt.data, data, copy_len);
  pkt.length = copy_len;
  pkt.timestamp_ms = now_ms;
  telemetry_count_++;
}

bool MavlinkParser::popTelemetry(TelemetryPacket& out) {
  if (telemetry_count_ == 0) {
    return false;
  }
  out = telemetry_ring_[telemetry_head_];
  telemetry_head_ = (uint8_t)((telemetry_head_ + 1) % MAVLINK_TELEMETRY_RING);
  telemetry_count_--;
  return true;
}

void MavlinkParser::tick(uint32_t now_ms) {
  updateHeartbeatFreshness(now_ms);
}

uint32_t MavlinkParser::crcErrors() const {
  return crc_errors_;
}

uint32_t MavlinkParser::framesDecoded() const {
  return frames_decoded_;
}

bool MavlinkParser::hasFrame() const {
  return has_frame_;
}

const RCFrame& MavlinkParser::frame() const {
  return frame_;
}

const LinkQuality& MavlinkParser::linkQuality() const {
  return lq_;
}

uint32_t MavlinkParser::lastHeartbeatMs() const {
  return last_heartbeat_ms_;
}

bool MavlinkParser::heartbeatAlive(uint32_t now_ms, uint16_t timeout_ms) const {
  if (!heartbeat_seen_) return false;
  return (now_ms - last_heartbeat_ms_) < timeout_ms;
}

uint8_t MavlinkParser::baseMode() const {
  return base_mode_;
}

uint8_t MavlinkParser::systemStatus() const {
  return system_status_;
}
