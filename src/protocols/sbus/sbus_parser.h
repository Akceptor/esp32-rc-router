// src/protocols/sbus/sbus_parser.h
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "protocols/protocol_types.h"

static const uint8_t SBUS_HEADER = 0x0F;
static const uint8_t SBUS_FOOTER = 0x00;
static const size_t SBUS_FRAME_SIZE = 25;
static const size_t SBUS_LQ_WINDOW = 32;

// SBUS raw range is 0..2047 (11-bit), but valid stick travel is 172..1811 mapping
// to 988..2012 us, same scale as CRSF. Values outside 172..1811 are clamped.
uint16_t sbusRawToUs(uint16_t raw);

class SbusParser {
 public:
  SbusParser();
  void reset();
  size_t push(const uint8_t* data, size_t len, uint32_t now_ms);
  bool hasFrame() const;
  const RCFrame& frame() const;
  const LinkQuality& linkQuality() const;
  bool popTelemetry(TelemetryPacket& out);
  void tick(uint32_t now_ms);
  uint32_t crcErrors() const;
  uint32_t framesDecoded() const;

 private:
  enum class State : uint8_t { WAIT_HEADER = 0, WAIT_BODY = 1 };

  void decodeFrame(uint32_t now_ms);
  void recomputeLinkQuality();
  bool isValidFrameCandidate() const;

  State state_;
  uint8_t buf_[SBUS_FRAME_SIZE];
  size_t buf_len_;
  bool crc_error_counted_;

  RCFrame frame_;
  LinkQuality link_;
  bool has_frame_;

  bool lost_window_[SBUS_LQ_WINDOW];
  size_t lq_index_;
  size_t lq_filled_;

  uint32_t crc_errors_;
  uint32_t frames_decoded_;
  uint32_t last_frame_ms_;
};
