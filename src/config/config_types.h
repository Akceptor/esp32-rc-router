#pragma once
#include <stdint.h>
#include "protocols/protocol_types.h"

static const uint8_t RECEIVER_PORT_COUNT = 2;
static const uint8_t PWM_PIN_COUNT = 4;
static const uint16_t CONFIG_VERSION = 1;

enum class PwmMode : uint8_t { DISABLED = 0, SERVO = 1, SWITCH = 2 };
enum class FailsafeMode : uint8_t { HOLD_LAST = 0, STOP_PWM = 1, FAILSAFE_VALUES = 2 };

struct ReceiverPortConfig {
  bool enabled;
  ProtocolType protocol;
  uint8_t priority;      // 0 = highest
  uint32_t baud;
  int8_t rx_pin;
  int8_t tx_pin;
  bool inverted;
};

struct SelectionConfig {
  uint8_t rssi_threshold_percent;  // below this a link is "bad"
  uint8_t lq_threshold_percent;
  uint8_t hysteresis_percent;      // challenger must beat active by this margin
  uint16_t switch_delay_ms;        // challenger must stay better this long
  uint16_t min_active_time_ms;     // active must be held this long before switching away
  uint16_t link_timeout_ms;        // no frame for this long => dead
};

struct OutputConfig {
  ProtocolType protocol;
  uint32_t baud;
  int8_t tx_pin;
  int8_t rx_pin;
  bool inverted;
  uint8_t channel_map[RC_CHANNEL_COUNT];  // channel_map[out_ch] = in_ch
};

struct PWMPinConfig {
  PwmMode mode;
  uint8_t pin;
  uint8_t source_channel;      // 0-based index into RCFrame.channels
  uint16_t update_rate_hz;     // servo refresh, typically 50
  bool invert;
  uint16_t switch_threshold_us;
  bool switch_active_high;
  uint16_t failsafe_us;
};

struct VoltageConfig {
  bool enabled;
  uint8_t adc_pin;
  float divider_ratio;        // Vin / Vadc
  float calibration_factor;   // multiplicative trim, default 1.0
  bool telemetry_override;
  uint8_t cell_count;
};

struct NetworkConfig {
  char ssid[33];
  char password[65];
  bool ap_mode;
  bool use_dhcp;
  uint32_t static_ip;
  uint32_t gateway;
  uint32_t netmask;
  char hostname[33];
};

struct SystemConfig {
  uint8_t log_level;              // LogLevel value
  bool serial_console;
  FailsafeMode failsafe_mode;
  uint16_t failsafe_channels[RC_CHANNEL_COUNT];
};

struct RouterConfig {
  uint16_t version;
  ReceiverPortConfig receivers[RECEIVER_PORT_COUNT];
  SelectionConfig selection;
  OutputConfig output;
  PWMPinConfig pwm[PWM_PIN_COUNT];
  VoltageConfig voltage;
  NetworkConfig network;
  SystemConfig system;
  uint32_t crc32;                 // over all preceding bytes
};

void configLoadDefaults(RouterConfig& cfg);
uint32_t configCrc32(const RouterConfig& cfg);   // excludes crc32 field
bool configValidate(RouterConfig& cfg);          // clamps out-of-range, returns false if it changed anything
