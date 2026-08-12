// src/protocols/crsf/crsf_parser.h
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "protocols/protocol_types.h"

static const uint8_t CRSF_SYNC = 0xC8;
static const uint8_t CRSF_ADDR_FC = 0xC8;
static const uint8_t CRSF_ADDR_TRANSMITTER = 0xEE;
static const uint8_t CRSF_ADDR_RECEIVER = 0xEA;

static const uint8_t CRSF_TYPE_RC_CHANNELS_PACKED = 0x16;
static const uint8_t CRSF_TYPE_LINK_STATISTICS = 0x14;
static const uint8_t CRSF_TYPE_BATTERY_SENSOR = 0x08;

static const size_t CRSF_MAX_FRAME_SIZE = 64;
static const size_t CRSF_TELEMETRY_QUEUE_SIZE = 8;

// CRC8 DVB-S2, polynomial 0xD5, table-free bit-by-bit implementation.
uint8_t crsfCrc8(const uint8_t* data, size_t len);

class CrsfParser {
 public:
  CrsfParser();
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
  // Consumes as many complete frames as possible from the front of buf_,
  // decoding valid ones and discarding one byte at a time on any framing
  // failure (bad length / bad CRC) so a later byte gets a fresh chance to
  // be reinterpreted as a sync marker. Returns the number of RC channel
  // frames decoded during this drain.
  size_t drainBuffer(uint32_t now_ms);
  void shiftBufLeft(size_t n);
  void handleCompleteFrame(const uint8_t* raw_frame, size_t frame_len, uint32_t now_ms);
  void decodeRcChannels(const uint8_t* payload, uint32_t now_ms);
  void decodeLinkStatistics(const uint8_t* payload);
  void enqueueTelemetry(uint8_t type, const uint8_t* full_frame, size_t frame_len,
                         uint32_t now_ms);

  uint8_t buf_[CRSF_MAX_FRAME_SIZE];
  size_t buf_len_;

  RCFrame frame_;
  LinkQuality link_;
  bool has_frame_;

  TelemetryPacket telem_queue_[CRSF_TELEMETRY_QUEUE_SIZE];
  size_t telem_head_;
  size_t telem_count_;

  uint32_t crc_errors_;
  uint32_t frames_decoded_;
  uint32_t last_frame_ms_;
};
