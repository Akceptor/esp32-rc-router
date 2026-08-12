// src/protocols/sbus/sbus_generator.cpp
#include "protocols/sbus/sbus_generator.h"
#include "protocols/sbus/sbus_parser.h"
#include <string.h>

static const size_t SBUS_RC_FRAME_SIZE = 25;

SbusGenerator::SbusGenerator() : failsafe_(false), frame_lost_(false) {}

size_t SbusGenerator::maxFrameSize() const { return SBUS_RC_FRAME_SIZE; }

void SbusGenerator::setFailsafe(bool failsafe) { failsafe_ = failsafe; }
void SbusGenerator::setFrameLost(bool frame_lost) { frame_lost_ = frame_lost; }

static uint16_t usToSbusRaw(uint16_t us) {
  int32_t u = (int32_t)us;
  if (u < RC_PULSE_MIN_US) u = RC_PULSE_MIN_US;
  if (u > RC_PULSE_MAX_US) u = RC_PULSE_MAX_US;
  int32_t numerator = (u - 988) * 1639;
  int32_t raw = (numerator + 1024 / 2) / 1024 + 172;
  if (raw < 172) raw = 172;
  if (raw > 1811) raw = 1811;
  return (uint16_t)raw;
}

size_t SbusGenerator::buildRcFrame(const RCFrame& frame, uint8_t* out, size_t out_cap) {
  if (out_cap < SBUS_RC_FRAME_SIZE) return 0;

  out[0] = SBUS_HEADER;
  uint8_t* data = &out[1];
  for (int i = 0; i < 22; i++) data[i] = 0;

  uint32_t bitpos = 0;
  for (int ch = 0; ch < RC_CHANNEL_COUNT; ch++) {
    uint32_t v = usToSbusRaw(frame.channels[ch]) & 0x7FFu;
    for (int b = 0; b < 11; b++) {
      if (v & (1u << b)) {
        uint32_t bit = bitpos + b;
        data[bit / 8] |= (uint8_t)(1u << (bit % 8));
      }
    }
    bitpos += 11;
  }

  uint8_t flags = 0;
  // bit0 = channel 17, bit1 = channel 18 (not modeled by RCFrame's 16 channels, left 0)
  if (frame_lost_) flags |= 0x04;
  if (failsafe_) flags |= 0x08;
  out[23] = flags;
  out[24] = SBUS_FOOTER;

  return SBUS_RC_FRAME_SIZE;
}

size_t SbusGenerator::buildBatteryTelemetry(const BatteryTelemetry& batt, uint8_t* out,
                                             size_t out_cap) {
  (void)batt;
  (void)out;
  (void)out_cap;
  return 0;  // SBUS has no return telemetry path
}
