#pragma once
#include <stdint.h>
#include "protocols/protocol_types.h"

struct StatusSnapshot {
  int8_t active_receiver;
  ProtocolType active_protocol;
  uint8_t rssi_percent;
  uint8_t lq_percent;
  bool failsafe;
  float battery_voltage;
  uint32_t adc_millivolts;
  uint32_t uptime_s;
  bool wifi_connected;
  bool wifi_ap_mode;
  int8_t wifi_rssi;
  uint32_t ip;
  uint32_t free_heap;
  uint32_t switch_count;
  char firmware_version[16];
};
