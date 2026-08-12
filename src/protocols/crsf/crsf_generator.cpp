// src/protocols/crsf/crsf_generator.cpp
#include "protocols/crsf/crsf_generator.h"
#include "protocols/crsf/crsf_parser.h"

static const size_t CRSF_RC_FRAME_SIZE = 26;
static const size_t CRSF_BATTERY_FRAME_SIZE = 12;

CrsfGenerator::CrsfGenerator() {}

size_t CrsfGenerator::maxFrameSize() const { return CRSF_MAX_FRAME_SIZE; }

static uint16_t usToCrsfRaw(uint16_t us) {
  // Inverse of crsfRawToUs: us = ((raw-172)*1024)/1639 + 988
  // => raw = ((us - 988) * 1639) / 1024 + 172, rounded.
  int32_t u = (int32_t)us;
  if (u < RC_PULSE_MIN_US) u = RC_PULSE_MIN_US;
  if (u > RC_PULSE_MAX_US) u = RC_PULSE_MAX_US;
  int32_t numerator = (u - 988) * 1639;
  int32_t raw = (numerator + 1024 / 2) / 1024 + 172;
  if (raw < 172) raw = 172;
  if (raw > 1811) raw = 1811;
  return (uint16_t)raw;
}

size_t CrsfGenerator::buildRcFrame(const RCFrame& frame, uint8_t* out, size_t out_cap) {
  if (out_cap < CRSF_RC_FRAME_SIZE) return 0;

  out[0] = CRSF_SYNC;
  out[1] = 0x18;  // 24 = type(1) + payload(22) + crc(1)
  out[2] = CRSF_TYPE_RC_CHANNELS_PACKED;

  uint8_t* payload = &out[3];
  for (int i = 0; i < 22; i++) payload[i] = 0;

  uint32_t bitpos = 0;
  for (int ch = 0; ch < RC_CHANNEL_COUNT; ch++) {
    uint32_t v = usToCrsfRaw(frame.channels[ch]) & 0x7FFu;
    for (int b = 0; b < 11; b++) {
      if (v & (1u << b)) {
        uint32_t bit = bitpos + b;
        payload[bit / 8] |= (uint8_t)(1u << (bit % 8));
      }
    }
    bitpos += 11;
  }

  uint8_t crc = crsfCrc8(&out[2], 23);
  out[25] = crc;
  return CRSF_RC_FRAME_SIZE;
}

size_t CrsfGenerator::buildBatteryTelemetry(const BatteryTelemetry& batt, uint8_t* out,
                                             size_t out_cap) {
  if (out_cap < CRSF_BATTERY_FRAME_SIZE) return 0;

  out[0] = CRSF_SYNC;
  out[1] = 0x0A;  // 10 = type(1) + payload(8) + crc(1)
  out[2] = CRSF_TYPE_BATTERY_SENSOR;

  out[3] = (uint8_t)((batt.voltage_dv >> 8) & 0xFF);
  out[4] = (uint8_t)(batt.voltage_dv & 0xFF);

  out[5] = (uint8_t)((batt.current_da >> 8) & 0xFF);
  out[6] = (uint8_t)(batt.current_da & 0xFF);

  out[7] = (uint8_t)((batt.used_capacity_mah >> 16) & 0xFF);
  out[8] = (uint8_t)((batt.used_capacity_mah >> 8) & 0xFF);
  out[9] = (uint8_t)(batt.used_capacity_mah & 0xFF);

  out[10] = batt.remaining_percent;

  uint8_t crc = crsfCrc8(&out[2], 9);  // type + 8 payload bytes
  out[11] = crc;
  return CRSF_BATTERY_FRAME_SIZE;
}
