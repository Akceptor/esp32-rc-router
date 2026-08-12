#include "telemetry/telemetry_router.h"
#include "protocols/crsf/crsf_generator.h"
#include "protocols/sbus/sbus_generator.h"
#include "protocols/mavlink/mavlink_generator.h"
#include <string.h>

static const uint8_t kCrsfSync = 0xC8;
static const uint8_t kCrsfTypeBattery = 0x08;
static const uint8_t kMavlinkStxV2 = 0xFD;
static const uint16_t kMavBatteryStatus = 147;
static const uint16_t kMavSysStatus = 1;

TelemetryRouter::TelemetryRouter(ReceiverManager& rx, OutputManager& out, VoltageMonitor& volt)
    : rx_(rx),
      out_(out),
      volt_(volt),
      voltage_override_(false),
      packets_forwarded_(0),
      packets_overridden_(0),
      packets_dropped_(0),
      last_injection_ms_(0) {
  memset(scratch_, 0, sizeof(scratch_));
}

void TelemetryRouter::begin(bool voltage_override) {
  voltage_override_ = voltage_override;
  packets_forwarded_ = 0;
  packets_overridden_ = 0;
  packets_dropped_ = 0;
  last_injection_ms_ = 0;
}

void TelemetryRouter::setVoltageOverride(bool enabled) { voltage_override_ = enabled; }

bool TelemetryRouter::isCrsfBatteryPacket(const uint8_t* buf, size_t len) const {
  if (out_.protocol() != ProtocolType::CRSF) return false;
  if (len < 4) return false;
  if (buf[0] != kCrsfSync) return false;
  return buf[2] == kCrsfTypeBattery;
}

bool TelemetryRouter::isMavlinkBatteryPacket(const uint8_t* buf, size_t len) const {
  if (out_.protocol() != ProtocolType::MAVLINK) return false;
  if (len < 10) return false;
  if (buf[0] != kMavlinkStxV2) return false;
  uint16_t msgid = (uint16_t)buf[7] | ((uint16_t)buf[8] << 8);
  return msgid == kMavBatteryStatus || msgid == kMavSysStatus;
}

size_t TelemetryRouter::forwardToActiveReceiver(const uint8_t* buf, size_t len) {
  ReceiverPort* active = rx_.activePort();
  if (active == nullptr) {
    packets_dropped_++;
    return 0;
  }
  size_t written = active->writeTelemetry(buf, len);
  if (written != len) {
    packets_dropped_++;
    return written;
  }
  return written;
}

size_t TelemetryRouter::injectBatteryToActiveReceiver() {
  ReceiverPort* active = rx_.activePort();
  if (active == nullptr) {
    packets_dropped_++;
    return 0;
  }

  BatteryTelemetry batt;
  if (!volt_.buildBattery(batt)) {
    packets_dropped_++;
    return 0;
  }

  size_t n = 0;
  switch (active->protocol()) {
    case ProtocolType::CRSF: {
      CrsfGenerator gen;
      n = gen.buildBatteryTelemetry(batt, scratch_, kScratchBytes);
      break;
    }
    case ProtocolType::MAVLINK: {
      MavlinkGenerator gen;
      n = gen.buildBatteryTelemetry(batt, scratch_, kScratchBytes);
      break;
    }
    case ProtocolType::SBUS:
    default:
      n = 0;   // SBUS carries no telemetry
      break;
  }
  if (n == 0) {
    packets_dropped_++;
    return 0;
  }

  size_t written = active->writeTelemetry(scratch_, n);
  if (written != n) {
    packets_dropped_++;
    return written;
  }
  packets_overridden_++;
  return written;
}

size_t TelemetryRouter::ingestFromFc(const uint8_t* buf, size_t len, uint32_t now_ms) {
  (void)now_ms;
  if (len == 0) return 0;
  if (len > kScratchBytes) len = kScratchBytes;

  bool is_battery = isCrsfBatteryPacket(buf, len) || isMavlinkBatteryPacket(buf, len);

  if (is_battery && voltage_override_) {
    injectBatteryToActiveReceiver();   // drops/counts internally
    return len;
  }

  size_t written = forwardToActiveReceiver(buf, len);
  if (written == len) {
    packets_forwarded_++;
  }
  return len;
}

void TelemetryRouter::update(uint32_t now_ms) {
  // Direction 1: receiver -> FC. Drain any queued TelemetryPacket entries from the
  // currently active receiver and forward the raw bytes to the FC.
  ReceiverPort* active = rx_.activePort();
  if (active != nullptr) {
    TelemetryPacket pkt;
    while (active->popTelemetry(pkt)) {
      size_t written = out_.sendTelemetry(pkt.data, pkt.length);
      if (written == pkt.length) {
        packets_forwarded_++;
      } else {
        packets_dropped_++;
      }
    }
  }

  // Periodic voltage-override injection: fires every 500 ms regardless of whether the
  // FC has sent a battery packet in that window. The very first call establishes the
  // baseline at last_injection_ms_ (defaults to 0, i.e. router start-up time) rather
  // than injecting immediately, so the first injection lands at the 500 ms mark.
  if (voltage_override_) {
    if ((now_ms - last_injection_ms_) >= kInjectionIntervalMs) {
      injectBatteryToActiveReceiver();
      last_injection_ms_ = now_ms;
    }
  }
}
