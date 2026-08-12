#pragma once
#include <stddef.h>
#include <stdint.h>
#include "protocols/protocol_types.h"

// MAVLink v2 framing is self-contained here (hand-rolled state machine + CRC16-MCRF4XX),
// so this module is native-testable without depending on generated mavlink-c headers.
// `lib_deps` MAY add `mavlink/c_library_v2` later for extended message decoding (e.g. full
// GPS/attitude dialects), but the RC-relevant subset (HEARTBEAT, RC_CHANNELS,
// RC_CHANNELS_OVERRIDE, BATTERY_STATUS, SYS_STATUS) never needs it.

static const uint8_t MAVLINK_STX_V2 = 0xFD;

static const uint32_t MAVLINK_MSG_ID_HEARTBEAT = 0;
static const uint32_t MAVLINK_MSG_ID_SYS_STATUS = 1;
static const uint32_t MAVLINK_MSG_ID_RC_CHANNELS = 65;
static const uint32_t MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;
static const uint32_t MAVLINK_MSG_ID_BATTERY_STATUS = 147;

static const uint8_t MAVLINK_INCOMPAT_FLAG_SIGNED = 0x01;
static const uint8_t MAVLINK_SIGNATURE_LEN = 13;
static const uint8_t MAVLINK_MAX_PAYLOAD_LEN = 255;
static const uint8_t MAVLINK_SYSTEM_ID = 1;
static const uint8_t MAVLINK_COMPONENT_ID = 1;

static const uint8_t MAVLINK_TELEMETRY_RING = 8;

// Per-message CRC_EXTRA byte, folded into the CRC16-MCRF4XX after header+payload.
// Only messages this router understands have a documented CRC_EXTRA; unknown
// message ids default to 0, which is a deliberate simplification (this module does
// not carry the full MAVLink dialect table) — it is internally consistent for any
// frame this codebase itself builds via MavlinkGenerator, which is all this parser
// needs to guarantee for the "PASSTHROUGH" telemetry path.
uint8_t mavlinkCrcExtra(uint32_t msgid);

// One step of the CRC16-MCRF4XX (X.25) accumulator, MAVLink's checksum algorithm.
void mavlinkCrcAccumulate(uint8_t data, uint16_t& crc);

// Full CRC16-MCRF4XX over `buf[0..len)` followed by `crc_extra`, seeded at 0xFFFF.
uint16_t mavlinkCrc16(const uint8_t* buf, size_t len, uint8_t crc_extra);

class MavlinkParser {
 public:
  MavlinkParser();

  void reset();

  // Feed bytes; returns number of complete RC frames (RC_CHANNELS or
  // RC_CHANNELS_OVERRIDE) decoded during this call.
  size_t push(const uint8_t* data, size_t len, uint32_t now_ms);

  bool hasFrame() const;
  const RCFrame& frame() const;
  const LinkQuality& linkQuality() const;
  bool popTelemetry(TelemetryPacket& out);

  // Ages heartbeat-derived link quality; call periodically even with no new bytes.
  void tick(uint32_t now_ms);

  uint32_t crcErrors() const;
  uint32_t framesDecoded() const;

  uint32_t lastHeartbeatMs() const;
  bool heartbeatAlive(uint32_t now_ms, uint16_t timeout_ms) const;
  uint8_t baseMode() const;
  uint8_t systemStatus() const;

 private:
  enum class State : uint8_t {
    WAIT_STX,
    WAIT_LEN,
    WAIT_HEADER,
    WAIT_PAYLOAD,
    WAIT_CRC,
    WAIT_SIGNATURE
  };

  bool processByte(uint8_t b, uint32_t now_ms);
  bool finishFrame(uint32_t now_ms);
  bool handleCompleteFrame(uint32_t now_ms);
  void decodeHeartbeat(uint32_t now_ms);
  void decodeRcChannels(uint32_t now_ms);
  void decodeRcChannelsOverride(uint32_t now_ms);
  void updateHeartbeatFreshness(uint32_t now_ms);
  void pushTelemetry(TelemetryKind kind, const uint8_t* data, uint8_t len, uint32_t now_ms);
  uint16_t readPayloadU16(uint32_t offset) const;

  State state_;
  uint8_t header_buf_[9];  // len, incompat, compat, seq, sysid, compid, msgid0..2
  uint8_t header_idx_;
  uint8_t payload_len_;
  uint8_t payload_[MAVLINK_MAX_PAYLOAD_LEN];
  uint8_t payload_idx_;
  uint8_t crc_buf_[2];
  uint8_t crc_idx_;
  uint8_t sig_idx_;

  uint8_t incompat_flags_;
  uint8_t seq_;
  uint8_t sysid_;
  uint8_t compid_;
  uint32_t msgid_;

  RCFrame frame_;
  LinkQuality lq_;
  bool has_frame_;

  uint32_t last_heartbeat_ms_;
  bool heartbeat_seen_;
  uint8_t base_mode_;
  uint8_t system_status_;

  TelemetryPacket telemetry_ring_[MAVLINK_TELEMETRY_RING];
  uint8_t telemetry_head_;
  uint8_t telemetry_count_;

  uint32_t crc_errors_;
  uint32_t frames_decoded_;
};
