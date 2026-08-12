#include "protocols/protocol_types.h"

void rcFrameInit(RCFrame& f) {
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    f.channels[i] = RC_PULSE_MID_US;
  }
  f.timestamp_ms = 0;
  f.valid = false;
}

void linkQualityInit(LinkQuality& lq) {
  lq.rssi_percent = 0;
  lq.lq_percent = 0;
  lq.rssi_dbm = 0;
  lq.frame_lost = false;
  lq.failsafe = false;
  lq.last_frame_ms = 0;
  lq.frames_received = 0;
  lq.crc_errors = 0;
  lq.valid = false;
}

uint16_t clampPulseUs(int32_t us) {
  if (us < static_cast<int32_t>(RC_PULSE_MIN_US)) {
    return RC_PULSE_MIN_US;
  }
  if (us > static_cast<int32_t>(RC_PULSE_MAX_US)) {
    return RC_PULSE_MAX_US;
  }
  return static_cast<uint16_t>(us);
}
