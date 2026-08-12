#include "receiver/receiver_port.h"

ReceiverPort::ReceiverPort(uint8_t index, IUartPort& uart)
    : index_(index), uart_(uart), began_(false), bytes_read_(0) {
  cfg_.enabled = false;
  cfg_.protocol = ProtocolType::NONE;
  cfg_.priority = 0;
  cfg_.baud = 0;
  cfg_.rx_pin = -1;
  cfg_.tx_pin = -1;
  cfg_.inverted = false;
  rcFrameInit(empty_frame_);
  linkQualityInit(empty_lq_);
}

void ReceiverPort::effectiveUartParams(const ReceiverPortConfig& cfg, uint32_t& baud,
                                        uint32_t& serial_config, bool& inverted) {
  switch (cfg.protocol) {
    case ProtocolType::SBUS:
      baud = 100000;
      serial_config = UART_CONFIG_8E2;
      inverted = true;
      break;
    case ProtocolType::CRSF:
      baud = 420000;
      serial_config = UART_CONFIG_8N1;
      inverted = cfg.inverted;
      break;
    case ProtocolType::MAVLINK:
      baud = (cfg.baud != 0) ? cfg.baud : 57600;
      serial_config = UART_CONFIG_8N1;
      inverted = cfg.inverted;
      break;
    default:
      baud = cfg.baud;
      serial_config = UART_CONFIG_8N1;
      inverted = cfg.inverted;
      break;
  }
}

bool ReceiverPort::needsRebegin(const ReceiverPortConfig& next) const {
  if (!began_) return true;
  if (next.protocol != cfg_.protocol) return true;
  if (next.rx_pin != cfg_.rx_pin || next.tx_pin != cfg_.tx_pin) return true;
  uint32_t baud_a, baud_b, sc_a, sc_b;
  bool inv_a, inv_b;
  effectiveUartParams(cfg_, baud_a, sc_a, inv_a);
  effectiveUartParams(next, baud_b, sc_b, inv_b);
  if (baud_a != baud_b || sc_a != sc_b || inv_a != inv_b) return true;
  return false;
}

void ReceiverPort::reapply(const ReceiverPortConfig& cfg) {
  uint32_t baud, serial_config;
  bool inverted;
  effectiveUartParams(cfg, baud, serial_config, inverted);
  uart_.end();
  uart_.begin(baud, serial_config, cfg.rx_pin, cfg.tx_pin, inverted);
  crsf_.reset();
  sbus_.reset();
  mavlink_.reset();
  bytes_read_ = 0;
  began_ = true;
}

bool ReceiverPort::begin(const ReceiverPortConfig& cfg) {
  cfg_ = cfg;
  reapply(cfg_);
  return began_;
}

void ReceiverPort::setConfig(const ReceiverPortConfig& cfg) {
  bool rebegin = needsRebegin(cfg);
  cfg_ = cfg;
  if (rebegin) {
    reapply(cfg_);
  }
}

void ReceiverPort::update(uint32_t now_ms) {
  if (!cfg_.enabled || !began_) {
    return;
  }
  uint8_t chunk[kChunkSize];
  size_t drained = 0;
  while (drained < kMaxDrainPerUpdate) {
    size_t avail = uart_.available();
    if (avail == 0) break;
    size_t want = (avail < kChunkSize) ? avail : kChunkSize;
    if (drained + want > kMaxDrainPerUpdate) {
      want = kMaxDrainPerUpdate - drained;
    }
    if (want == 0) break;
    size_t n = uart_.read(chunk, want);
    if (n == 0) break;
    bytes_read_ += (uint32_t)n;
    drained += n;

    switch (cfg_.protocol) {
      case ProtocolType::CRSF: crsf_.push(chunk, n, now_ms); break;
      case ProtocolType::SBUS: sbus_.push(chunk, n, now_ms); break;
      case ProtocolType::MAVLINK: mavlink_.push(chunk, n, now_ms); break;
      default: break;
    }
  }

  switch (cfg_.protocol) {
    case ProtocolType::CRSF: crsf_.tick(now_ms); break;
    case ProtocolType::SBUS: sbus_.tick(now_ms); break;
    case ProtocolType::MAVLINK: mavlink_.tick(now_ms); break;
    default: break;
  }
}

bool ReceiverPort::isAlive(uint32_t now_ms, uint16_t timeout_ms) const {
  const LinkQuality& lq = getLinkQuality();
  if (!lq.valid) return false;
  return (now_ms - lq.last_frame_ms) < timeout_ms;
}

const RCFrame& ReceiverPort::getFrame() const {
  switch (cfg_.protocol) {
    case ProtocolType::CRSF: return crsf_.frame();
    case ProtocolType::SBUS: return sbus_.frame();
    case ProtocolType::MAVLINK: return mavlink_.frame();
    default: return empty_frame_;
  }
}

const LinkQuality& ReceiverPort::getLinkQuality() const {
  switch (cfg_.protocol) {
    case ProtocolType::CRSF: return crsf_.linkQuality();
    case ProtocolType::SBUS: return sbus_.linkQuality();
    case ProtocolType::MAVLINK: return mavlink_.linkQuality();
    default: return empty_lq_;
  }
}

ProtocolType ReceiverPort::protocol() const {
  return cfg_.protocol;
}

bool ReceiverPort::enabled() const {
  return cfg_.enabled;
}

uint8_t ReceiverPort::index() const {
  return index_;
}

uint8_t ReceiverPort::priority() const {
  return cfg_.priority;
}

size_t ReceiverPort::writeTelemetry(const uint8_t* buf, size_t len) {
  if (cfg_.protocol != ProtocolType::CRSF && cfg_.protocol != ProtocolType::MAVLINK) {
    return 0;
  }
  return uart_.write(buf, len);
}

bool ReceiverPort::popTelemetry(TelemetryPacket& out) {
  switch (cfg_.protocol) {
    case ProtocolType::CRSF: return crsf_.popTelemetry(out);
    case ProtocolType::SBUS: return sbus_.popTelemetry(out);
    case ProtocolType::MAVLINK: return mavlink_.popTelemetry(out);
    default: return false;
  }
}

uint32_t ReceiverPort::bytesRead() const {
  return bytes_read_;
}
