// src/protocols/sbus/sbus_parser.cpp
#include "protocols/sbus/sbus_parser.h"

uint16_t sbusRawToUs(uint16_t raw) {
  int32_t r = (int32_t)raw;
  if (r < 172) r = 172;
  if (r > 1811) r = 1811;
  int32_t numerator = (r - 172) * 1024;
  int32_t us = (numerator + 1639 / 2) / 1639 + 988;
  return clampPulseUs(us);
}

SbusParser::SbusParser() { reset(); }

void SbusParser::reset() {
  state_ = State::WAIT_HEADER;
  buf_len_ = 0;
  crc_error_counted_ = false;
  rcFrameInit(frame_);
  linkQualityInit(link_);
  has_frame_ = false;
  for (size_t i = 0; i < SBUS_LQ_WINDOW; i++) lost_window_[i] = false;
  lq_index_ = 0;
  lq_filled_ = 0;
  crc_errors_ = 0;
  frames_decoded_ = 0;
  last_frame_ms_ = 0;
}

bool SbusParser::hasFrame() const { return has_frame_; }
const RCFrame& SbusParser::frame() const { return frame_; }
const LinkQuality& SbusParser::linkQuality() const { return link_; }
uint32_t SbusParser::crcErrors() const { return crc_errors_; }
uint32_t SbusParser::framesDecoded() const { return frames_decoded_; }

bool SbusParser::popTelemetry(TelemetryPacket& out) {
  (void)out;
  return false;  // SBUS carries no telemetry channel in this parser
}

size_t SbusParser::push(const uint8_t* data, size_t len, uint32_t now_ms) {
  size_t decoded = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (state_ == State::WAIT_HEADER) {
      if (b == SBUS_HEADER) {
        buf_[0] = b;
        buf_len_ = 1;
        state_ = State::WAIT_BODY;
      }
      continue;
    }

    // WAIT_BODY
    buf_[buf_len_++] = b;
    if (buf_len_ == SBUS_FRAME_SIZE) {
      if (isValidFrameCandidate()) {
        size_t before = frames_decoded_;
        decodeFrame(now_ms);
        if (frames_decoded_ != before) decoded++;
        state_ = State::WAIT_HEADER;
        buf_len_ = 0;
        crc_error_counted_ = false;  // episode resolved; next mismatch is new
      } else {
        // Candidate rejected. A byte merely equal to SBUS_HEADER is never
        // trusted as a new frame start on its own: we shift the window so
        // that byte becomes buf_[0], but we do NOT decode from it yet. It
        // only gets accepted once buf_ fills back up to SBUS_FRAME_SIZE and
        // isValidFrameCandidate() passes on *that* alignment (i.e. the byte
        // 24 positions later really is the footer, plus the flag-byte
        // plausibility check below). Ordinary SBUS channel payloads can
        // legitimately contain 0x0F bytes by coincidence, so a single
        // corruption event can require several such rejected candidates
        // before the true frame boundary is found; count that whole hunt as
        // one CRC error, not one per rejected candidate.
        if (!crc_error_counted_) {
          crc_errors_++;
          crc_error_counted_ = true;
        }
        bool resynced = false;
        for (size_t k = 1; k < buf_len_; k++) {
          if (buf_[k] == SBUS_HEADER) {
            size_t remaining = buf_len_ - k;
            for (size_t m = 0; m < remaining; m++) buf_[m] = buf_[k + m];
            buf_len_ = remaining;
            resynced = true;
            break;
          }
        }
        if (!resynced) {
          buf_len_ = 0;
          state_ = State::WAIT_HEADER;
          crc_error_counted_ = false;  // fully lost sync; next episode is new
        }
      }
    }
  }
  return decoded;
}

bool SbusParser::isValidFrameCandidate() const {
  if (buf_[SBUS_FRAME_SIZE - 1] != SBUS_FOOTER) return false;
  // SBUS flag byte (buf_[23]) reserved bits 4-7 are always zero on real
  // frames (only bits 2/3 -- frame_lost/failsafe -- are used here). Requiring
  // them to be zero is a second, independent check beyond the footer byte,
  // which meaningfully narrows the odds that a false resync alignment (one
  // that isn't the true frame boundary) is coincidentally accepted just
  // because a 0x00 byte happens to sit 24 positions later.
  uint8_t flags = buf_[23];
  if ((flags & 0xF0) != 0) return false;
  return true;
}

void SbusParser::decodeFrame(uint32_t now_ms) {
  const uint8_t* data = &buf_[1];
  uint16_t raw[RC_CHANNEL_COUNT];
  uint32_t bitpos = 0;
  for (int ch = 0; ch < RC_CHANNEL_COUNT; ch++) {
    uint32_t v = 0;
    for (int b = 0; b < 11; b++) {
      uint32_t bit = bitpos + b;
      uint8_t byte = data[bit / 8];
      if (byte & (1u << (bit % 8))) v |= (1u << b);
    }
    raw[ch] = (uint16_t)v;
    bitpos += 11;
  }

  for (int ch = 0; ch < RC_CHANNEL_COUNT; ch++) {
    frame_.channels[ch] = sbusRawToUs(raw[ch]);
  }
  frame_.timestamp_ms = now_ms;
  frame_.valid = true;
  has_frame_ = true;
  frames_decoded_++;
  last_frame_ms_ = now_ms;

  uint8_t flags = buf_[23];
  bool frame_lost = (flags & 0x04) != 0;
  bool failsafe = (flags & 0x08) != 0;

  lost_window_[lq_index_] = frame_lost;
  lq_index_ = (lq_index_ + 1) % SBUS_LQ_WINDOW;
  if (lq_filled_ < SBUS_LQ_WINDOW) lq_filled_++;

  recomputeLinkQuality();

  link_.frame_lost = frame_lost;
  link_.failsafe = failsafe;
  link_.rssi_dbm = 0;  // SBUS carries no RSSI information
  link_.valid = true;
  link_.last_frame_ms = now_ms;
  link_.frames_received++;
}

void SbusParser::recomputeLinkQuality() {
  size_t lost_count = 0;
  for (size_t i = 0; i < lq_filled_; i++) {
    if (lost_window_[i]) lost_count++;
  }
  size_t denom = (lq_filled_ == 0) ? 1 : lq_filled_;
  int32_t pct = 100 - (int32_t)((lost_count * 100) / denom);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  link_.lq_percent = (uint8_t)pct;
  link_.rssi_percent = link_.lq_percent;  // SBUS has no RSSI, mirror LQ
}

void SbusParser::tick(uint32_t now_ms) {
  if (last_frame_ms_ == 0 && frames_decoded_ == 0) return;
  if (now_ms - last_frame_ms_ > 500) {
    frame_.valid = false;
    link_.valid = false;
    link_.failsafe = true;
  }
}
