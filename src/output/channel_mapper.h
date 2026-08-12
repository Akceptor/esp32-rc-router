#pragma once
#include <stdint.h>
#include "protocols/protocol_types.h"

class ChannelMapper {
 public:
  ChannelMapper();

  // map[out] = in; entries >= RC_CHANNEL_COUNT fall back to identity for that slot.
  void setMap(const uint8_t map[RC_CHANNEL_COUNT]);
  void identity();
  uint8_t mapping(uint8_t out_channel) const;
  void apply(const RCFrame& in, RCFrame& out) const;

 private:
  uint8_t map_[RC_CHANNEL_COUNT];
};
