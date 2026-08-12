#pragma once
#include <stdint.h>
#include <stddef.h>
#include "receiver/receiver_manager.h"
#include "output/output_manager.h"
#include "telemetry/voltage_monitor.h"
#include "protocols/protocol_types.h"

// TelemetryRouter moves telemetry in both directions across the router:
//
//  1. Receiver -> FC: the active ReceiverPort may have TelemetryPacket entries queued
//     (link stats, GPS, etc. reported by the RC link back to the ground station via the
//     RC protocol's own telemetry channel). Those raw bytes are forwarded verbatim to
//     the FC via OutputManager::sendTelemetry.
//
//  2. FC -> receiver: the FC talks to the router over the *same* UART as OutputManager
//     uses to send RC frames (CRSF/MAVLink are bidirectional on one wire; SBUS has no
//     return channel). App reads whatever bytes are waiting on that UART and hands them
//     to ingestFromFc(), which classifies whole packets using the OUTPUT protocol's
//     framing (because that's the wire format the FC is actually speaking), and forwards
//     them to the currently active receiver via ReceiverPort::writeTelemetry (which
//     re-encodes/relays them in the RECEIVER's protocol).
//
// Voltage override: when VoltageConfig.telemetry_override (mirrored here via
// setVoltageOverride) is enabled, any FC battery packet (CRSF type 0x08, MAVLink msgid
// 147 BATTERY_STATUS or msgid 1 SYS_STATUS) is dropped and replaced by a fresh battery
// packet built from VoltageMonitor::buildBattery, encoded in the active receiver's
// protocol. Everything else passes through byte-for-byte. In addition, every 500 ms,
// if override is enabled, a battery packet is injected toward the active receiver even
// if the FC never sent one in that window (some FCs are silent on the telemetry channel
// unless polled) — this injection is also counted as "overridden".
class TelemetryRouter {
 public:
  TelemetryRouter(ReceiverManager& rx, OutputManager& out, VoltageMonitor& volt);

  void begin(bool voltage_override);
  void setVoltageOverride(bool enabled);

  // Pumps direction 1 (receiver -> FC) and the 500 ms periodic injection.
  void update(uint32_t now_ms);

  // Direction 2 (FC -> receiver). Returns the number of bytes accepted from `buf`
  // (always `len` in this implementation; classification happens internally).
  size_t ingestFromFc(const uint8_t* buf, size_t len, uint32_t now_ms);

  uint32_t packetsForwarded() const { return packets_forwarded_; }
  uint32_t packetsOverridden() const { return packets_overridden_; }
  uint32_t packetsDropped() const { return packets_dropped_; }

 private:
  bool isCrsfBatteryPacket(const uint8_t* buf, size_t len) const;
  bool isMavlinkBatteryPacket(const uint8_t* buf, size_t len) const;
  size_t forwardToActiveReceiver(const uint8_t* buf, size_t len);
  size_t injectBatteryToActiveReceiver();

  ReceiverManager& rx_;
  OutputManager& out_;
  VoltageMonitor& volt_;

  bool voltage_override_;
  uint32_t packets_forwarded_;
  uint32_t packets_overridden_;
  uint32_t packets_dropped_;

  uint32_t last_injection_ms_;
  static const uint32_t kInjectionIntervalMs = 500;

  static const size_t kScratchBytes = 256;
  uint8_t scratch_[kScratchBytes];
};
