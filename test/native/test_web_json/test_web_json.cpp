#include <unity.h>
#include <ArduinoJson.h>
#include <string.h>
#include "web/config_json.h"
#include "config/config_types.h"

static RouterConfig makeDefaultCfg() {
  RouterConfig cfg;
  configLoadDefaults(cfg);
  return cfg;
}

void test_receivers_to_json_emits_fields(void) {
  RouterConfig cfg = makeDefaultCfg();
  cfg.receivers[0].protocol = ProtocolType::CRSF;
  cfg.receivers[0].enabled = true;
  cfg.receivers[0].priority = 0;
  cfg.receivers[1].protocol = ProtocolType::SBUS;
  cfg.receivers[1].enabled = false;
  cfg.receivers[1].priority = 1;

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  receiversToJson(cfg, root);

  JsonArray arr = root["receivers"].as<JsonArray>();
  TEST_ASSERT_EQUAL_UINT32(RECEIVER_PORT_COUNT, arr.size());
  TEST_ASSERT_EQUAL_INT(1, arr[0]["protocol"].as<int>());   // CRSF == 1
  TEST_ASSERT_TRUE(arr[0]["enabled"].as<bool>());
  TEST_ASSERT_EQUAL_INT(0, arr[0]["priority"].as<int>());
  TEST_ASSERT_EQUAL_INT(2, arr[1]["protocol"].as<int>());   // SBUS == 2
  TEST_ASSERT_FALSE(arr[1]["enabled"].as<bool>());
}

void test_receivers_from_json_applies_valid_body(void) {
  RouterConfig cfg = makeDefaultCfg();

  JsonDocument doc;
  JsonArray arr = doc.createNestedArray("receivers");
  JsonObject r0 = arr.createNestedObject();
  r0["protocol"] = 1;
  r0["enabled"] = true;
  r0["priority"] = 0;
  r0["baud"] = 420000;
  r0["rx_pin"] = 16;
  r0["tx_pin"] = 17;
  r0["inverted"] = false;
  JsonObject r1 = arr.createNestedObject();
  r1["protocol"] = 3;
  r1["enabled"] = true;
  r1["priority"] = 1;
  r1["baud"] = 57600;
  r1["rx_pin"] = 18;
  r1["tx_pin"] = 19;
  r1["inverted"] = false;

  bool ok = receiversFromJson(doc.as<JsonObjectConst>(), cfg);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT((int)ProtocolType::CRSF, (int)cfg.receivers[0].protocol);
  TEST_ASSERT_EQUAL_UINT32(420000, cfg.receivers[0].baud);
  TEST_ASSERT_EQUAL_INT((int)ProtocolType::MAVLINK, (int)cfg.receivers[1].protocol);
}

void test_receivers_from_json_rejects_out_of_range_priority(void) {
  RouterConfig cfg = makeDefaultCfg();
  JsonDocument doc;
  JsonArray arr = doc.createNestedArray("receivers");
  JsonObject r0 = arr.createNestedObject();
  r0["protocol"] = 1;
  r0["enabled"] = true;
  r0["priority"] = 250;   // out of sane range (0..RECEIVER_PORT_COUNT-1)
  r0["baud"] = 420000;
  r0["rx_pin"] = 16;
  r0["tx_pin"] = 17;
  r0["inverted"] = false;
  JsonObject r1 = arr.createNestedObject();
  r1["protocol"] = 2;
  r1["enabled"] = false;
  r1["priority"] = 1;
  r1["baud"] = 100000;
  r1["rx_pin"] = 18;
  r1["tx_pin"] = 19;
  r1["inverted"] = true;

  bool ok = receiversFromJson(doc.as<JsonObjectConst>(), cfg);
  TEST_ASSERT_FALSE(ok);
}

void test_pwm_round_trip_all_pins(void) {
  RouterConfig cfg = makeDefaultCfg();
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    cfg.pwm[i].mode = (i % 2 == 0) ? PwmMode::SERVO : PwmMode::SWITCH;
    cfg.pwm[i].pin = 25 + i;
    cfg.pwm[i].source_channel = i;
    cfg.pwm[i].update_rate_hz = 50;
    cfg.pwm[i].invert = (i == 1);
    cfg.pwm[i].switch_threshold_us = 1500;
    cfg.pwm[i].switch_active_high = true;
    cfg.pwm[i].failsafe_us = 1000;
  }

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  pwmToJson(cfg, root);

  RouterConfig cfg2 = makeDefaultCfg();
  bool ok = pwmFromJson(doc.as<JsonObjectConst>(), cfg2);
  TEST_ASSERT_TRUE(ok);
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    TEST_ASSERT_EQUAL_INT((int)cfg.pwm[i].mode, (int)cfg2.pwm[i].mode);
    TEST_ASSERT_EQUAL_UINT8(cfg.pwm[i].pin, cfg2.pwm[i].pin);
    TEST_ASSERT_EQUAL_UINT8(cfg.pwm[i].source_channel, cfg2.pwm[i].source_channel);
    TEST_ASSERT_EQUAL_UINT16(cfg.pwm[i].switch_threshold_us, cfg2.pwm[i].switch_threshold_us);
    TEST_ASSERT_EQUAL_UINT16(cfg.pwm[i].failsafe_us, cfg2.pwm[i].failsafe_us);
  }
}

void test_voltage_round_trip_preserves_float(void) {
  RouterConfig cfg = makeDefaultCfg();
  cfg.voltage.enabled = true;
  cfg.voltage.adc_pin = 33;
  cfg.voltage.divider_ratio = 11.132f;
  cfg.voltage.calibration_factor = 0.987f;
  cfg.voltage.telemetry_override = true;
  cfg.voltage.cell_count = 4;

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  voltageToJson(cfg, root);

  RouterConfig cfg2 = makeDefaultCfg();
  bool ok = voltageFromJson(doc.as<JsonObjectConst>(), cfg2);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 11.132f, cfg2.voltage.divider_ratio);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.987f, cfg2.voltage.calibration_factor);
  TEST_ASSERT_EQUAL_UINT8(4, cfg2.voltage.cell_count);
}

void test_network_round_trip_static_ip(void) {
  RouterConfig cfg = makeDefaultCfg();
  strncpy(cfg.network.ssid, "MyDrone", sizeof(cfg.network.ssid) - 1);
  strncpy(cfg.network.password, "hunter2hunter2", sizeof(cfg.network.password) - 1);
  cfg.network.ap_mode = false;
  cfg.network.use_dhcp = false;
  cfg.network.static_ip = (192u << 24) | (168u << 16) | (1u << 8) | 50u;
  cfg.network.gateway = (192u << 24) | (168u << 16) | (1u << 8) | 1u;
  cfg.network.netmask = (255u << 24) | (255u << 16) | (255u << 8) | 0u;
  strncpy(cfg.network.hostname, "rcrouter", sizeof(cfg.network.hostname) - 1);

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  networkToJson(cfg, root);
  TEST_ASSERT_EQUAL_STRING("192.168.1.50", root["network"]["static_ip"].as<const char*>());

  RouterConfig cfg2 = makeDefaultCfg();
  bool ok = networkFromJson(doc.as<JsonObjectConst>(), cfg2);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_UINT32(cfg.network.static_ip, cfg2.network.static_ip);
  TEST_ASSERT_EQUAL_STRING("MyDrone", cfg2.network.ssid);
}

void test_system_round_trip_failsafe_channels(void) {
  RouterConfig cfg = makeDefaultCfg();
  cfg.system.failsafe_mode = FailsafeMode::FAILSAFE_VALUES;
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    cfg.system.failsafe_channels[i] = 1000 + i;
  }

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  systemToJson(cfg, root);

  RouterConfig cfg2 = makeDefaultCfg();
  bool ok = systemFromJson(doc.as<JsonObjectConst>(), cfg2);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT((int)FailsafeMode::FAILSAFE_VALUES, (int)cfg2.system.failsafe_mode);
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    TEST_ASSERT_EQUAL_UINT16(1000 + i, cfg2.system.failsafe_channels[i]);
  }
}

void test_status_to_json_field_names(void) {
  StatusSnapshot s;
  s.active_receiver = 0;
  s.active_protocol = ProtocolType::CRSF;
  s.rssi_percent = 90;
  s.lq_percent = 95;
  s.failsafe = false;
  s.battery_voltage = 12.34f;
  s.adc_millivolts = 1234;
  s.uptime_s = 5000;
  s.wifi_connected = true;
  s.wifi_ap_mode = false;
  s.wifi_rssi = -55;
  s.ip = (10u << 24) | (0u << 16) | (0u << 8) | 5u;
  s.free_heap = 123456;
  s.switch_count = 3;
  strncpy(s.firmware_version, "1.0.0", sizeof(s.firmware_version) - 1);

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  statusToJson(s, root);

  TEST_ASSERT_EQUAL_INT(0, root["active_receiver"].as<int>());
  TEST_ASSERT_EQUAL_INT(1, root["active_protocol"].as<int>());
  TEST_ASSERT_EQUAL_INT(90, root["rssi_percent"].as<int>());
  TEST_ASSERT_EQUAL_INT(95, root["lq_percent"].as<int>());
  TEST_ASSERT_FALSE(root["failsafe"].as<bool>());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.34f, root["battery_voltage"].as<float>());
  TEST_ASSERT_EQUAL_UINT32(1234, root["adc_millivolts"].as<uint32_t>());
  TEST_ASSERT_EQUAL_UINT32(5000, root["uptime_s"].as<uint32_t>());
  TEST_ASSERT_TRUE(root["wifi_connected"].as<bool>());
  TEST_ASSERT_FALSE(root["wifi_ap_mode"].as<bool>());
  TEST_ASSERT_EQUAL_INT(-55, root["wifi_rssi"].as<int>());
  TEST_ASSERT_EQUAL_STRING("10.0.0.5", root["ip"].as<const char*>());
  TEST_ASSERT_EQUAL_UINT32(123456, root["free_heap"].as<uint32_t>());
  TEST_ASSERT_EQUAL_UINT32(3, root["switch_count"].as<uint32_t>());
  TEST_ASSERT_EQUAL_STRING("1.0.0", root["firmware_version"].as<const char*>());
}

void test_unknown_missing_fields_leave_config_untouched(void) {
  RouterConfig cfg = makeDefaultCfg();
  cfg.output.protocol = ProtocolType::CRSF;
  cfg.output.baud = 420000;

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["bogus_field"] = "ignored";
  root["another_bogus"] = 42;
  // deliberately omit "protocol", "baud", etc.

  RouterConfig cfg2 = cfg;
  bool ok = outputFromJson(doc.as<JsonObjectConst>(), cfg2);
  TEST_ASSERT_TRUE(ok);   // missing fields are not an error, just left unchanged
  TEST_ASSERT_EQUAL_INT((int)cfg.output.protocol, (int)cfg2.output.protocol);
  TEST_ASSERT_EQUAL_UINT32(cfg.output.baud, cfg2.output.baud);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_receivers_to_json_emits_fields);
  RUN_TEST(test_receivers_from_json_applies_valid_body);
  RUN_TEST(test_receivers_from_json_rejects_out_of_range_priority);
  RUN_TEST(test_pwm_round_trip_all_pins);
  RUN_TEST(test_voltage_round_trip_preserves_float);
  RUN_TEST(test_network_round_trip_static_ip);
  RUN_TEST(test_system_round_trip_failsafe_channels);
  RUN_TEST(test_status_to_json_field_names);
  RUN_TEST(test_unknown_missing_fields_leave_config_untouched);
  return UNITY_END();
}
