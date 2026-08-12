// src/protocols/crsf/crsf_generator.h
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "protocols/protocol_types.h"

class CrsfGenerator {
 public:
  CrsfGenerator();
  size_t buildRcFrame(const RCFrame& frame, uint8_t* out, size_t out_cap);
  size_t buildBatteryTelemetry(const BatteryTelemetry& batt, uint8_t* out, size_t out_cap);
  size_t maxFrameSize() const;
};
