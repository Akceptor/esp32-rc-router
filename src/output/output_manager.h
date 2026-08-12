#pragma once
#include <stdint.h>
#include <stddef.h>
#include "hal/uart_port.h"
#include "protocols/protocol_types.h"
#include "config/config_types.h"
#include "output/channel_mapper.h"
#include "protocols/crsf/crsf_generator.h"
#include "protocols/sbus/sbus_generator.h"
#include "protocols/mavlink/mavlink_generator.h"

// OutputManager is the single point where the router's internal, protocol-neutral
// RCFrame is turned into wire bytes for the flight controller. Because ReceiverManager
// always exposes an RCFrame (microseconds, 988..2012) regardless of which input
// protocol produced it, and OutputManager always consumes an RCFrame regardless of
// which output protocol it emits, the input side and the output side are fully
// decoupled: a CRSF receiver can drive an SBUS output, a SBUS receiver can drive a
// MAVLink output, etc. The only place protocol-specific encoding happens is inside
// the three generator members below.
class OutputManager {
 public:
  explicit OutputManager(IUartPort& uart);

  bool begin(const OutputConfig& cfg);
  void setConfig(const OutputConfig& cfg);

  // Maps `frame` through mapper_, then (subject to failsafe rules and rate limiting)
  // encodes it with the active generator and writes it to uart_.
  void update(uint32_t now_ms, const RCFrame& frame, bool link_valid);

  void setFailsafe(FailsafeMode mode, const uint16_t values[RC_CHANNEL_COUNT]);

  // Passthrough telemetry toward the FC. CRSF and MAVLink share the same UART as the
  // RC frame stream (half-duplex-style, App is responsible for not overlapping calls
  // within a single output-task tick); SBUS carries no telemetry channel and always
  // returns 0.
  size_t sendTelemetry(const uint8_t* buf, size_t len);
  size_t sendBattery(const BatteryTelemetry& batt);

  ProtocolType protocol() const { return cfg_.protocol; }
  ChannelMapper& mapper() { return mapper_; }
  uint32_t framesSent() const { return frames_sent_; }
  size_t lastFrameSize() const { return last_frame_size_; }
  uint16_t frameIntervalMs() const;

 private:
  bool applyUartConfig(const OutputConfig& cfg);
  size_t encode(const RCFrame& frame, uint8_t* out, size_t out_cap);
  bool uartConfigChanged(const OutputConfig& a, const OutputConfig& b) const;

  IUartPort& uart_;
  OutputConfig cfg_;
  ChannelMapper mapper_;

  CrsfGenerator crsf_;
  SbusGenerator sbus_;
  MavlinkGenerator mavlink_;

  static const size_t kTxScratchBytes = 280;
  uint8_t tx_scratch_[kTxScratchBytes];   // no heap: single reusable buffer

  RCFrame last_good_frame_;   // last mapped frame sent while link_valid was true
  bool has_last_good_frame_;

  uint32_t last_send_ms_;
  bool has_sent_once_;
  uint32_t frames_sent_;
  size_t last_frame_size_;

  FailsafeMode failsafe_mode_;
  uint16_t failsafe_values_[RC_CHANNEL_COUNT];

  bool stop_pwm_flag_sent_;    // STOP_PWM: true once the single flagged frame has gone out
};
