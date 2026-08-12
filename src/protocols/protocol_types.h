#pragma once
#include <stddef.h>
#include <stdint.h>

static const uint8_t RC_CHANNEL_COUNT = 16;
static const uint16_t RC_PULSE_MIN_US = 988;
static const uint16_t RC_PULSE_MID_US = 1500;
static const uint16_t RC_PULSE_MAX_US = 2012;

enum class ProtocolType : uint8_t { NONE = 0, CRSF = 1, SBUS = 2, MAVLINK = 3 };

struct RCFrame {
  uint16_t channels[RC_CHANNEL_COUNT];  // microseconds, 988..2012
  uint32_t timestamp_ms;
  bool valid;
};

struct LinkQuality {
  uint8_t rssi_percent;    // 0..100
  uint8_t lq_percent;      // 0..100
  int16_t rssi_dbm;        // negative dBm; 0 == unknown
  bool frame_lost;
  bool failsafe;
  uint32_t last_frame_ms;
  uint32_t frames_received;
  uint32_t crc_errors;
  bool valid;
};

enum class TelemetryKind : uint8_t {
  UNKNOWN = 0, BATTERY = 1, ATTITUDE = 2, GPS = 3, HEARTBEAT = 4, PASSTHROUGH = 5
};

static const uint8_t TELEMETRY_MAX_PAYLOAD = 64;

struct TelemetryPacket {
  TelemetryKind kind;
  uint8_t data[TELEMETRY_MAX_PAYLOAD];
  uint8_t length;
  uint32_t timestamp_ms;
};

struct BatteryTelemetry {
  uint16_t voltage_dv;        // decivolts (0.1 V)
  uint16_t current_da;        // deciamps (0.1 A)
  uint32_t used_capacity_mah;
  uint8_t remaining_percent;
};

void rcFrameInit(RCFrame& f);              // all channels RC_PULSE_MID_US, valid=false
void linkQualityInit(LinkQuality& lq);     // all zero, valid=false
uint16_t clampPulseUs(int32_t us);         // clamp to [RC_PULSE_MIN_US, RC_PULSE_MAX_US]
