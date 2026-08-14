#include "web/config_json.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

uint32_t ipStringToU32(const char* s, bool& ok) {
  unsigned a = 0, b = 0, c = 0, d = 0;
  int n = sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d);
  if (n != 4 || a > 255 || b > 255 || c > 255 || d > 255) {
    ok = false;
    return 0;
  }
  ok = true;
  return (a << 24) | (b << 16) | (c << 8) | d;
}

void ipU32ToString(uint32_t ip, char* out, size_t out_cap) {
  snprintf(out, out_cap, "%u.%u.%u.%u", (unsigned)((ip >> 24) & 0xFF), (unsigned)((ip >> 16) & 0xFF),
           (unsigned)((ip >> 8) & 0xFF), (unsigned)(ip & 0xFF));
}

void receiversToJson(const RouterConfig& cfg, JsonObject out) {
  JsonArray arr = out["receivers"].to<JsonArray>();
  for (uint8_t i = 0; i < RECEIVER_PORT_COUNT; i++) {
    const ReceiverPortConfig& r = cfg.receivers[i];
    JsonObject o = arr.add<JsonObject>();
    o["protocol"] = (int)r.protocol;
    o["enabled"] = r.enabled;
    o["priority"] = r.priority;
    o["baud"] = r.baud;
    o["rx_pin"] = r.rx_pin;
    o["tx_pin"] = r.tx_pin;
    o["inverted"] = r.inverted;
  }
}

bool receiversFromJson(JsonObjectConst in, RouterConfig& cfg) {
  if (!in["receivers"].is<JsonArrayConst>()) return true;   // absent -> untouched
  JsonArrayConst arr = in["receivers"].as<JsonArrayConst>();
  if (arr.size() != RECEIVER_PORT_COUNT) return false;

  ReceiverPortConfig staged[RECEIVER_PORT_COUNT];
  for (uint8_t i = 0; i < RECEIVER_PORT_COUNT; i++) staged[i] = cfg.receivers[i];

  uint8_t i = 0;
  for (JsonObjectConst o : arr) {
    if (o["protocol"].is<int>()) {
      int p = o["protocol"].as<int>();
      if (p < 0 || p > (int)ProtocolType::MAVLINK) return false;
      staged[i].protocol = (ProtocolType)p;
    }
    if (o["enabled"].is<bool>()) staged[i].enabled = o["enabled"].as<bool>();
    if (o["priority"].is<int>()) {
      int pr = o["priority"].as<int>();
      if (pr < 0 || pr >= RECEIVER_PORT_COUNT) return false;
      staged[i].priority = (uint8_t)pr;
    }
    if (o["baud"].is<uint32_t>()) staged[i].baud = o["baud"].as<uint32_t>();
    if (o["rx_pin"].is<int>()) staged[i].rx_pin = (int8_t)o["rx_pin"].as<int>();
    if (o["tx_pin"].is<int>()) staged[i].tx_pin = (int8_t)o["tx_pin"].as<int>();
    if (o["inverted"].is<bool>()) staged[i].inverted = o["inverted"].as<bool>();
    i++;
  }

  for (uint8_t j = 0; j < RECEIVER_PORT_COUNT; j++) cfg.receivers[j] = staged[j];
  return true;
}

void outputToJson(const RouterConfig& cfg, JsonObject out) {
  JsonObject o = out["output"].to<JsonObject>();
  o["protocol"] = (int)cfg.output.protocol;
  o["baud"] = cfg.output.baud;
  o["tx_pin"] = cfg.output.tx_pin;
  o["rx_pin"] = cfg.output.rx_pin;
  o["inverted"] = cfg.output.inverted;
  JsonArray map = o["channel_map"].to<JsonArray>();
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) map.add(cfg.output.channel_map[i]);
}

bool outputFromJson(JsonObjectConst in, RouterConfig& cfg) {
  if (!in["output"].is<JsonObjectConst>()) return true;
  JsonObjectConst o = in["output"].as<JsonObjectConst>();

  OutputConfig staged = cfg.output;
  if (o["protocol"].is<int>()) {
    int p = o["protocol"].as<int>();
    if (p < 0 || p > (int)ProtocolType::MAVLINK) return false;
    staged.protocol = (ProtocolType)p;
  }
  if (o["baud"].is<uint32_t>()) staged.baud = o["baud"].as<uint32_t>();
  if (o["tx_pin"].is<int>()) staged.tx_pin = (int8_t)o["tx_pin"].as<int>();
  if (o["rx_pin"].is<int>()) staged.rx_pin = (int8_t)o["rx_pin"].as<int>();
  if (o["inverted"].is<bool>()) staged.inverted = o["inverted"].as<bool>();
  if (o["channel_map"].is<JsonArrayConst>()) {
    JsonArrayConst map = o["channel_map"].as<JsonArrayConst>();
    if (map.size() != RC_CHANNEL_COUNT) return false;
    uint8_t idx = 0;
    for (JsonVariantConst v : map) {
      int ch = v.as<int>();
      if (ch < 0 || ch >= RC_CHANNEL_COUNT) return false;
      staged.channel_map[idx++] = (uint8_t)ch;
    }
  }

  cfg.output = staged;
  return true;
}

void pwmToJson(const RouterConfig& cfg, JsonObject out) {
  JsonArray arr = out["pwm"].to<JsonArray>();
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    const PWMPinConfig& p = cfg.pwm[i];
    JsonObject o = arr.add<JsonObject>();
    o["mode"] = (int)p.mode;
    o["pin"] = p.pin;
    o["source_channel"] = p.source_channel;
    o["update_rate_hz"] = p.update_rate_hz;
    o["invert"] = p.invert;
    o["switch_threshold_us"] = p.switch_threshold_us;
    o["switch_active_high"] = p.switch_active_high;
    o["failsafe_us"] = p.failsafe_us;
  }
}

bool pwmFromJson(JsonObjectConst in, RouterConfig& cfg) {
  if (!in["pwm"].is<JsonArrayConst>()) return true;
  JsonArrayConst arr = in["pwm"].as<JsonArrayConst>();
  if (arr.size() != PWM_PIN_COUNT) return false;

  PWMPinConfig staged[PWM_PIN_COUNT];
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) staged[i] = cfg.pwm[i];

  uint8_t i = 0;
  for (JsonObjectConst o : arr) {
    if (o["mode"].is<int>()) {
      int m = o["mode"].as<int>();
      if (m < 0 || m > (int)PwmMode::SWITCH) return false;
      staged[i].mode = (PwmMode)m;
    }
    if (o["pin"].is<int>()) staged[i].pin = (uint8_t)o["pin"].as<int>();
    if (o["source_channel"].is<int>()) {
      int sc = o["source_channel"].as<int>();
      if (sc < 0 || sc >= RC_CHANNEL_COUNT) return false;
      staged[i].source_channel = (uint8_t)sc;
    }
    if (o["update_rate_hz"].is<int>()) staged[i].update_rate_hz = (uint16_t)o["update_rate_hz"].as<int>();
    if (o["invert"].is<bool>()) staged[i].invert = o["invert"].as<bool>();
    if (o["switch_threshold_us"].is<int>()) {
      staged[i].switch_threshold_us = (uint16_t)o["switch_threshold_us"].as<int>();
    }
    if (o["switch_active_high"].is<bool>()) staged[i].switch_active_high = o["switch_active_high"].as<bool>();
    if (o["failsafe_us"].is<int>()) staged[i].failsafe_us = (uint16_t)o["failsafe_us"].as<int>();
    i++;
  }

  for (uint8_t j = 0; j < PWM_PIN_COUNT; j++) cfg.pwm[j] = staged[j];
  return true;
}

void voltageToJson(const RouterConfig& cfg, JsonObject out) {
  JsonObject o = out["voltage"].to<JsonObject>();
  o["enabled"] = cfg.voltage.enabled;
  o["adc_pin"] = cfg.voltage.adc_pin;
  o["divider_ratio"] = cfg.voltage.divider_ratio;
  o["calibration_factor"] = cfg.voltage.calibration_factor;
  o["telemetry_override"] = cfg.voltage.telemetry_override;
  o["cell_count"] = cfg.voltage.cell_count;
}

bool voltageFromJson(JsonObjectConst in, RouterConfig& cfg) {
  if (!in["voltage"].is<JsonObjectConst>()) return true;
  JsonObjectConst o = in["voltage"].as<JsonObjectConst>();

  VoltageConfig staged = cfg.voltage;
  if (o["enabled"].is<bool>()) staged.enabled = o["enabled"].as<bool>();
  if (o["adc_pin"].is<int>()) staged.adc_pin = (uint8_t)o["adc_pin"].as<int>();
  if (o["divider_ratio"].is<float>()) {
    float r = o["divider_ratio"].as<float>();
    if (r <= 0.0f) return false;
    staged.divider_ratio = r;
  }
  if (o["calibration_factor"].is<float>()) {
    float f = o["calibration_factor"].as<float>();
    if (f < 0.5f || f > 2.0f) return false;
    staged.calibration_factor = f;
  }
  if (o["telemetry_override"].is<bool>()) staged.telemetry_override = o["telemetry_override"].as<bool>();
  if (o["cell_count"].is<int>()) staged.cell_count = (uint8_t)o["cell_count"].as<int>();

  cfg.voltage = staged;
  return true;
}

void networkToJson(const RouterConfig& cfg, JsonObject out) {
  JsonObject o = out["network"].to<JsonObject>();
  o["ssid"] = cfg.network.ssid;
  o["password"] = cfg.network.password;
  o["ap_mode"] = cfg.network.ap_mode;
  o["use_dhcp"] = cfg.network.use_dhcp;
  char ip_str[16];
  ipU32ToString(cfg.network.static_ip, ip_str, sizeof(ip_str));
  o["static_ip"] = ip_str;
  char gw_str[16];
  ipU32ToString(cfg.network.gateway, gw_str, sizeof(gw_str));
  o["gateway"] = gw_str;
  char mask_str[16];
  ipU32ToString(cfg.network.netmask, mask_str, sizeof(mask_str));
  o["netmask"] = mask_str;
  o["hostname"] = cfg.network.hostname;
}

bool networkFromJson(JsonObjectConst in, RouterConfig& cfg) {
  if (!in["network"].is<JsonObjectConst>()) return true;
  JsonObjectConst o = in["network"].as<JsonObjectConst>();

  NetworkConfig staged = cfg.network;
  if (o["ssid"].is<const char*>()) {
    strncpy(staged.ssid, o["ssid"].as<const char*>(), sizeof(staged.ssid) - 1);
    staged.ssid[sizeof(staged.ssid) - 1] = '\0';
  }
  if (o["password"].is<const char*>()) {
    strncpy(staged.password, o["password"].as<const char*>(), sizeof(staged.password) - 1);
    staged.password[sizeof(staged.password) - 1] = '\0';
  }
  if (o["ap_mode"].is<bool>()) staged.ap_mode = o["ap_mode"].as<bool>();
  if (o["use_dhcp"].is<bool>()) staged.use_dhcp = o["use_dhcp"].as<bool>();
  if (o["static_ip"].is<const char*>()) {
    bool ok = false;
    uint32_t ip = ipStringToU32(o["static_ip"].as<const char*>(), ok);
    if (!ok) return false;
    staged.static_ip = ip;
  }
  if (o["gateway"].is<const char*>()) {
    bool ok = false;
    uint32_t gw = ipStringToU32(o["gateway"].as<const char*>(), ok);
    if (!ok) return false;
    staged.gateway = gw;
  }
  if (o["netmask"].is<const char*>()) {
    bool ok = false;
    uint32_t mask = ipStringToU32(o["netmask"].as<const char*>(), ok);
    if (!ok) return false;
    staged.netmask = mask;
  }
  if (o["hostname"].is<const char*>()) {
    strncpy(staged.hostname, o["hostname"].as<const char*>(), sizeof(staged.hostname) - 1);
    staged.hostname[sizeof(staged.hostname) - 1] = '\0';
  }

  cfg.network = staged;
  return true;
}

void systemToJson(const RouterConfig& cfg, JsonObject out) {
  JsonObject o = out["system"].to<JsonObject>();
  o["log_level"] = cfg.system.log_level;
  o["serial_console"] = cfg.system.serial_console;
  o["failsafe_mode"] = (int)cfg.system.failsafe_mode;
  JsonArray arr = o["failsafe_channels"].to<JsonArray>();
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) arr.add(cfg.system.failsafe_channels[i]);
}

bool systemFromJson(JsonObjectConst in, RouterConfig& cfg) {
  if (!in["system"].is<JsonObjectConst>()) return true;
  JsonObjectConst o = in["system"].as<JsonObjectConst>();

  SystemConfig staged = cfg.system;
  if (o["log_level"].is<int>()) staged.log_level = (uint8_t)o["log_level"].as<int>();
  if (o["serial_console"].is<bool>()) staged.serial_console = o["serial_console"].as<bool>();
  if (o["failsafe_mode"].is<int>()) {
    int m = o["failsafe_mode"].as<int>();
    if (m < 0 || m > (int)FailsafeMode::FAILSAFE_VALUES) return false;
    staged.failsafe_mode = (FailsafeMode)m;
  }
  if (o["failsafe_channels"].is<JsonArrayConst>()) {
    JsonArrayConst arr = o["failsafe_channels"].as<JsonArrayConst>();
    if (arr.size() != RC_CHANNEL_COUNT) return false;
    uint8_t idx = 0;
    for (JsonVariantConst v : arr) {
      staged.failsafe_channels[idx++] = (uint16_t)v.as<int>();
    }
  }

  cfg.system = staged;
  return true;
}

void statusToJson(const StatusSnapshot& s, JsonObject out) {
  out["active_receiver"] = s.active_receiver;
  out["active_protocol"] = (int)s.active_protocol;
  out["rssi_percent"] = s.rssi_percent;
  out["lq_percent"] = s.lq_percent;
  out["failsafe"] = s.failsafe;
  out["battery_voltage"] = s.battery_voltage;
  out["adc_millivolts"] = s.adc_millivolts;
  out["uptime_s"] = s.uptime_s;
  out["wifi_connected"] = s.wifi_connected;
  out["wifi_ap_mode"] = s.wifi_ap_mode;
  out["wifi_rssi"] = s.wifi_rssi;
  char ip_str[16];
  ipU32ToString(s.ip, ip_str, sizeof(ip_str));
  out["ip"] = ip_str;
  out["free_heap"] = s.free_heap;
  out["switch_count"] = s.switch_count;
  out["firmware_version"] = s.firmware_version;
}
