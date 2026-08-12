// src/protocols/sbus/sbus_generator.h
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "protocols/protocol_types.h"

class SbusGenerator {
 public:
  SbusGenerator();
  size_t buildRcFrame(const RCFrame& frame, uint8_t* out, size_t out_cap);
  // SBUS is a unidirectional, transmitter-to-receiver protocol with no return telemetry
  // channel defined in this router's scope. This always returns 0.
  size_t buildBatteryTelemetry(const BatteryTelemetry& batt, uint8_t* out, size_t out_cap);
  size_t maxFrameSize() const;

  // Additions beyond the uniform generator shape: SBUS flags are sticky local state,
  // not part of RCFrame, so they are set independently and applied on every buildRcFrame call.
  void setFailsafe(bool failsafe);
  void setFrameLost(bool frame_lost);

 private:
  bool failsafe_;
  bool frame_lost_;
};
