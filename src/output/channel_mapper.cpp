#include "output/channel_mapper.h"

ChannelMapper::ChannelMapper() {
  identity();
}

void ChannelMapper::identity() {
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; ++i) {
    map_[i] = i;
  }
}

void ChannelMapper::setMap(const uint8_t map[RC_CHANNEL_COUNT]) {
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; ++i) {
    map_[i] = (map[i] < RC_CHANNEL_COUNT) ? map[i] : i;
  }
}

uint8_t ChannelMapper::mapping(uint8_t out_channel) const {
  if (out_channel >= RC_CHANNEL_COUNT) {
    return out_channel;
  }
  return map_[out_channel];
}

void ChannelMapper::apply(const RCFrame& in, RCFrame& out) const {
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; ++i) {
    out.channels[i] = in.channels[map_[i]];
  }
  out.timestamp_ms = in.timestamp_ms;
  out.valid = in.valid;
}
