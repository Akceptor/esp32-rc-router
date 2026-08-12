#include "output/output_manager.h"
#include <string.h>

OutputManager::OutputManager(IUartPort& uart)
    : uart_(uart),
      cfg_(),
      mapper_(),
      crsf_(),
      sbus_(),
      mavlink_(),
      last_good_frame_(),
      has_last_good_frame_(false),
      last_send_ms_(0),
      has_sent_once_(false),
      frames_sent_(0),
      last_frame_size_(0),
      failsafe_mode_(FailsafeMode::HOLD_LAST),
      failsafe_values_(),
      stop_pwm_flag_sent_(false) {
  memset(tx_scratch_, 0, sizeof(tx_scratch_));
  rcFrameInit(last_good_frame_);
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) failsafe_values_[i] = RC_PULSE_MID_US;
}

bool OutputManager::uartConfigChanged(const OutputConfig& a, const OutputConfig& b) const {
  if (a.protocol != b.protocol) return true;
  if (a.baud != b.baud) return true;
  if (a.tx_pin != b.tx_pin || a.rx_pin != b.rx_pin) return true;
  if (a.inverted != b.inverted) return true;
  return false;
}

bool OutputManager::applyUartConfig(const OutputConfig& cfg) {
  uint32_t baud = cfg.baud;
  uint32_t serial_cfg = UART_CONFIG_8N1;
  bool inverted = cfg.inverted;

  if (cfg.protocol == ProtocolType::SBUS) {
    // SBUS is fixed at 100000 baud, 8E2, inverted, regardless of what the caller asked
    // for. Document this: any attempt to configure SBUS at a different baud/parity is
    // silently corrected here.
    baud = 100000;
    serial_cfg = UART_CONFIG_8E2;
    inverted = true;
  }

  uart_.end();
  return uart_.begin(baud, serial_cfg, cfg.rx_pin, cfg.tx_pin, inverted);
}

bool OutputManager::begin(const OutputConfig& cfg) {
  cfg_ = cfg;
  bool ok = applyUartConfig(cfg_);
  mapper_.setMap(cfg_.channel_map);
  has_sent_once_ = false;
  has_last_good_frame_ = false;
  stop_pwm_flag_sent_ = false;
  return ok;
}

void OutputManager::setConfig(const OutputConfig& cfg) {
  bool needs_rebegin = uartConfigChanged(cfg_, cfg);
  cfg_ = cfg;
  mapper_.setMap(cfg_.channel_map);
  if (needs_rebegin) {
    applyUartConfig(cfg_);
    stop_pwm_flag_sent_ = false;
  }
}

void OutputManager::setFailsafe(FailsafeMode mode, const uint16_t values[RC_CHANNEL_COUNT]) {
  failsafe_mode_ = mode;
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) failsafe_values_[i] = values[i];
}

uint16_t OutputManager::frameIntervalMs() const {
  switch (cfg_.protocol) {
    case ProtocolType::CRSF: return 4;
    case ProtocolType::SBUS: return 14;
    case ProtocolType::MAVLINK: return 20;
    case ProtocolType::NONE:
    default: return 0;
  }
}

size_t OutputManager::encode(const RCFrame& frame, uint8_t* out, size_t out_cap) {
  switch (cfg_.protocol) {
    case ProtocolType::CRSF: return crsf_.buildRcFrame(frame, out, out_cap);
    case ProtocolType::SBUS: return sbus_.buildRcFrame(frame, out, out_cap);
    case ProtocolType::MAVLINK: return mavlink_.buildRcFrame(frame, out, out_cap);
    case ProtocolType::NONE:
    default: return 0;
  }
}

void OutputManager::update(uint32_t now_ms, const RCFrame& frame, bool link_valid) {
  uint16_t interval = frameIntervalMs();
  if (interval == 0) return;
  if (has_sent_once_ && (now_ms - last_send_ms_) < interval) return;

  // Link just transitioned bad -> good: resume normal sending and forget any pending
  // STOP_PWM latch so the next loss re-triggers the single flagged frame.
  if (link_valid) {
    stop_pwm_flag_sent_ = false;
  }

  RCFrame to_send;
  bool should_send = true;

  if (link_valid) {
    mapper_.apply(frame, to_send);
    last_good_frame_ = to_send;
    has_last_good_frame_ = true;
  } else {
    switch (failsafe_mode_) {
      case FailsafeMode::HOLD_LAST:
        if (has_last_good_frame_) {
          to_send = last_good_frame_;
        } else {
          rcFrameInit(to_send);
        }
        break;
      case FailsafeMode::FAILSAFE_VALUES:
        rcFrameInit(to_send);
        for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
          to_send.channels[i] = clampPulseUs(failsafe_values_[i]);
        }
        to_send.valid = true;
        break;
      case FailsafeMode::STOP_PWM:
      default:
        if (stop_pwm_flag_sent_) {
          should_send = false;
        } else {
          // One final frame, flagged, then silence. For SBUS the failsafe bit is set
          // in the frame itself via SbusGenerator::setFailsafe(true); for CRSF/MAVLink
          // there is no per-frame failsafe bit, so the single frame carries the last
          // known-good mapped frame (never the configured failsafe_values_ array —
          // STOP_PWM is a silence-based failsafe, not a values-based one), and silence
          // follows immediately after.
          to_send = has_last_good_frame_ ? last_good_frame_ : frame;
          if (cfg_.protocol == ProtocolType::SBUS) {
            sbus_.setFailsafe(true);
          }
          stop_pwm_flag_sent_ = true;
        }
        break;
    }
  }

  if (!should_send) {
    if (cfg_.protocol == ProtocolType::SBUS) {
      sbus_.setFailsafe(false);
    }
    return;
  }

  size_t n = encode(to_send, tx_scratch_, kTxScratchBytes);
  if (cfg_.protocol == ProtocolType::SBUS && link_valid) {
    // Clear the failsafe flag once we resume normal transmission.
    sbus_.setFailsafe(false);
  }
  if (n == 0) return;

  uart_.write(tx_scratch_, n);
  last_send_ms_ = now_ms;
  has_sent_once_ = true;
  frames_sent_++;
  last_frame_size_ = n;
}

size_t OutputManager::sendTelemetry(const uint8_t* buf, size_t len) {
  if (cfg_.protocol == ProtocolType::SBUS) return 0;   // SBUS has no telemetry channel
  return uart_.write(buf, len);
}

size_t OutputManager::sendBattery(const BatteryTelemetry& batt) {
  size_t n = 0;
  switch (cfg_.protocol) {
    case ProtocolType::CRSF: n = crsf_.buildBatteryTelemetry(batt, tx_scratch_, kTxScratchBytes); break;
    case ProtocolType::MAVLINK: n = mavlink_.buildBatteryTelemetry(batt, tx_scratch_, kTxScratchBytes); break;
    case ProtocolType::SBUS:
    default: return 0;   // SBUS carries no telemetry
  }
  if (n == 0) return 0;
  return uart_.write(tx_scratch_, n);
}
