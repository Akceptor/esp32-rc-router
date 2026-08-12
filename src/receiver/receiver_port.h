#pragma once
#include <stddef.h>
#include <stdint.h>
#include "hal/uart_port.h"
#include "config/config_types.h"
#include "protocols/protocol_types.h"
#include "protocols/crsf/crsf_parser.h"
#include "protocols/sbus/sbus_parser.h"
#include "protocols/mavlink/mavlink_parser.h"

class ReceiverPort {
 public:
  ReceiverPort(uint8_t index, IUartPort& uart);

  bool begin(const ReceiverPortConfig& cfg);
  void setConfig(const ReceiverPortConfig& cfg);
  void update(uint32_t now_ms);
  bool isAlive(uint32_t now_ms, uint16_t timeout_ms) const;

  const RCFrame& getFrame() const;
  const LinkQuality& getLinkQuality() const;
  ProtocolType protocol() const;
  bool enabled() const;
  uint8_t index() const;
  uint8_t priority() const;

  size_t writeTelemetry(const uint8_t* buf, size_t len);
  bool popTelemetry(TelemetryPacket& out);
  uint32_t bytesRead() const;

 private:
  static const size_t kChunkSize = 64;
  static const size_t kMaxDrainPerUpdate = 256;

  static void effectiveUartParams(const ReceiverPortConfig& cfg, uint32_t& baud,
                                   uint32_t& serial_config, bool& inverted);
  bool needsRebegin(const ReceiverPortConfig& next) const;
  void reapply(const ReceiverPortConfig& cfg);

  uint8_t index_;
  IUartPort& uart_;
  ReceiverPortConfig cfg_;
  bool began_;

  CrsfParser crsf_;
  SbusParser sbus_;
  MavlinkParser mavlink_;

  uint32_t bytes_read_;
  RCFrame empty_frame_;
  LinkQuality empty_lq_;
};
