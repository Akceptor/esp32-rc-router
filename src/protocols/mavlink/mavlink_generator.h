#pragma once
#include <stddef.h>
#include <stdint.h>
#include "protocols/protocol_types.h"
#include "protocols/mavlink/mavlink_parser.h"

// Truncates trailing zero bytes from `payload` (MAVLink v2 wire-size optimization),
// returns the new length. `payload` contents are unchanged; only the reported
// length shrinks. A payload of all zero bytes trims to length 0.
size_t mavlinkTrimTrailingZeros(uint8_t* payload, size_t len);

class MavlinkGenerator {
 public:
  MavlinkGenerator();

  void setTarget(uint8_t sysid, uint8_t compid);

  // Returns bytes written, 0 on failure/insufficient capacity.
  size_t buildRcFrame(const RCFrame& frame, uint8_t* out, size_t out_cap);
  size_t buildBatteryTelemetry(const BatteryTelemetry& batt, uint8_t* out, size_t out_cap);
  size_t maxFrameSize() const;

 private:
  size_t buildFrame(uint32_t msgid, const uint8_t* payload, uint8_t payload_len, uint8_t* out,
                     size_t out_cap);

  uint8_t target_system_;
  uint8_t target_component_;
  uint8_t seq_;
};
