// src/protocols/crsf/crsf_parser.cpp
#include "protocols/crsf/crsf_parser.h"
#include <string.h>

uint8_t crsfCrc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0xD5);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

static uint16_t crsfRawToUs(uint16_t raw) {
  // CRSF 11-bit raw range 172..1811 maps to 988..2012 us.
  // us = ((raw - 172) * 1024) / 1639 + 988, integer math with rounding via
  // adding half the divisor before dividing.
  int32_t r = (int32_t)raw;
  if (r < 172) r = 172;
  if (r > 1811) r = 1811;
  int32_t numerator = (r - 172) * 1024;
  int32_t us = (numerator + 1639 / 2) / 1639 + 988;
  return clampPulseUs(us);
}

CrsfParser::CrsfParser() { reset(); }

void CrsfParser::reset() {
  buf_len_ = 0;
  rcFrameInit(frame_);
  linkQualityInit(link_);
  has_frame_ = false;
  telem_head_ = 0;
  telem_count_ = 0;
  crc_errors_ = 0;
  frames_decoded_ = 0;
  last_frame_ms_ = 0;
}

bool CrsfParser::hasFrame() const { return has_frame_; }
const RCFrame& CrsfParser::frame() const { return frame_; }
const LinkQuality& CrsfParser::linkQuality() const { return link_; }
uint32_t CrsfParser::crcErrors() const { return crc_errors_; }
uint32_t CrsfParser::framesDecoded() const { return frames_decoded_; }

void CrsfParser::shiftBufLeft(size_t n) {
  if (n >= buf_len_) {
    buf_len_ = 0;
    return;
  }
  memmove(buf_, buf_ + n, buf_len_ - n);
  buf_len_ -= n;
}

size_t CrsfParser::push(const uint8_t* data, size_t len, uint32_t now_ms) {
  size_t decoded = 0;
  for (size_t i = 0; i < len; i++) {
    if (buf_len_ < CRSF_MAX_FRAME_SIZE) {
      buf_[buf_len_++] = data[i];
    } else {
      // Should not normally happen: a well-formed candidate never exceeds
      // CRSF_MAX_FRAME_SIZE bytes (length byte is capped at 62). Guard
      // against overflow by dropping the oldest byte to make room.
      shiftBufLeft(1);
      buf_[buf_len_++] = data[i];
    }
    decoded += drainBuffer(now_ms);
  }
  return decoded;
}

size_t CrsfParser::drainBuffer(uint32_t now_ms) {
  size_t decoded = 0;
  while (buf_len_ > 0) {
    uint8_t sync = buf_[0];
    if (!(sync == CRSF_SYNC || sync == CRSF_ADDR_TRANSMITTER || sync == CRSF_ADDR_RECEIVER)) {
      // Not a plausible sync byte: drop it and re-examine the next one.
      shiftBufLeft(1);
      continue;
    }
    if (buf_len_ < 2) {
      break;  // need the length byte
    }
    uint8_t length = buf_[1];
    if (length < 2 || length > 62) {
      // Invalid length: discard only the sync byte so the byte that failed
      // as a length gets a fresh chance to be reinterpreted as a sync
      // marker on the next iteration.
      shiftBufLeft(1);
      continue;
    }
    size_t total_needed = 2 + (size_t)length;
    if (buf_len_ < total_needed) {
      break;  // need more data bytes
    }

    // Real CRSF computes the CRC over the full Type+Payload span
    // (length - 1 bytes) uniformly for all frame types.
    uint8_t computed_crc = crsfCrc8(&buf_[2], length - 1u);
    uint8_t received_crc = buf_[total_needed - 1];

    if (computed_crc == received_crc) {
      size_t before = frames_decoded_;
      handleCompleteFrame(buf_, total_needed, now_ms);
      if (frames_decoded_ != before) decoded++;
      shiftBufLeft(total_needed);
    } else {
      crc_errors_++;
      // Discard only the sync byte; retry from the next position so a
      // later, still-buffered byte can be reinterpreted as a fresh sync.
      shiftBufLeft(1);
    }
  }
  return decoded;
}

void CrsfParser::handleCompleteFrame(const uint8_t* raw_frame, size_t frame_len,
                                      uint32_t now_ms) {
  uint8_t type = raw_frame[2];
  const uint8_t* payload = &raw_frame[3];

  if (type == CRSF_TYPE_RC_CHANNELS_PACKED) {
    decodeRcChannels(payload, now_ms);
  } else if (type == CRSF_TYPE_LINK_STATISTICS) {
    decodeLinkStatistics(payload);
  } else {
    enqueueTelemetry(type, raw_frame, frame_len, now_ms);
  }
}

void CrsfParser::decodeRcChannels(const uint8_t* payload, uint32_t now_ms) {
  uint16_t raw[RC_CHANNEL_COUNT];
  uint32_t bitpos = 0;
  for (int ch = 0; ch < RC_CHANNEL_COUNT; ch++) {
    uint32_t v = 0;
    for (int b = 0; b < 11; b++) {
      uint32_t bit = bitpos + b;
      uint8_t byte = payload[bit / 8];
      if (byte & (1u << (bit % 8))) v |= (1u << b);
    }
    raw[ch] = (uint16_t)v;
    bitpos += 11;
  }

  for (int ch = 0; ch < RC_CHANNEL_COUNT; ch++) {
    frame_.channels[ch] = crsfRawToUs(raw[ch]);
  }
  frame_.timestamp_ms = now_ms;
  frame_.valid = true;
  has_frame_ = true;
  frames_decoded_++;
  last_frame_ms_ = now_ms;

  link_.valid = true;
  link_.last_frame_ms = now_ms;
  link_.frames_received++;
}

void CrsfParser::decodeLinkStatistics(const uint8_t* payload) {
  uint8_t rssi_ant1 = payload[0];
  uint8_t lq = payload[2];

  link_.rssi_dbm = (int16_t)(-(int16_t)rssi_ant1);
  link_.lq_percent = lq;

  int32_t dbm = link_.rssi_dbm;
  int32_t pct;
  if (dbm <= -120) {
    pct = 0;
  } else if (dbm >= -50) {
    pct = 100;
  } else {
    pct = ((dbm + 120) * 100) / 70;
  }
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  link_.rssi_percent = (uint8_t)pct;

  link_.failsafe = (lq == 0);
  link_.valid = true;
}

void CrsfParser::enqueueTelemetry(uint8_t type, const uint8_t* full_frame, size_t frame_len,
                                   uint32_t now_ms) {
  TelemetryPacket pkt;
  pkt.kind = (type == CRSF_TYPE_BATTERY_SENSOR) ? TelemetryKind::BATTERY
                                                 : TelemetryKind::PASSTHROUGH;
  size_t copy_len = frame_len;
  if (copy_len > TELEMETRY_MAX_PAYLOAD) copy_len = TELEMETRY_MAX_PAYLOAD;
  memcpy(pkt.data, full_frame, copy_len);
  pkt.length = (uint8_t)copy_len;
  pkt.timestamp_ms = now_ms;

  size_t write_index = (telem_head_ + telem_count_) % CRSF_TELEMETRY_QUEUE_SIZE;
  telem_queue_[write_index] = pkt;
  if (telem_count_ < CRSF_TELEMETRY_QUEUE_SIZE) {
    telem_count_++;
  } else {
    // ring full: overwrite oldest, advance head
    telem_head_ = (telem_head_ + 1) % CRSF_TELEMETRY_QUEUE_SIZE;
  }
}

bool CrsfParser::popTelemetry(TelemetryPacket& out) {
  if (telem_count_ == 0) return false;
  out = telem_queue_[telem_head_];
  telem_head_ = (telem_head_ + 1) % CRSF_TELEMETRY_QUEUE_SIZE;
  telem_count_--;
  return true;
}

void CrsfParser::tick(uint32_t now_ms) {
  if (link_.last_frame_ms == 0 && frames_decoded_ == 0) return;
  if (now_ms - last_frame_ms_ > 500) {
    frame_.valid = false;
    link_.valid = false;
    link_.failsafe = true;
  }
}
