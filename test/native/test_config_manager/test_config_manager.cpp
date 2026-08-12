#include <unity.h>
#include <string.h>
#include "config/config_manager.h"
#include "config/config_types.h"

void setUp(void) {}
void tearDown(void) {}

void test_defaults_are_valid(void) {
  RouterConfig cfg;
  configLoadDefaults(cfg);
  RouterConfig before = cfg;
  bool no_change = configValidate(cfg);
  TEST_ASSERT_TRUE(no_change);
  TEST_ASSERT_EQUAL_UINT16(CONFIG_VERSION, cfg.version);
  TEST_ASSERT_EQUAL_MEMORY(&before, &cfg, sizeof(RouterConfig));
}

void test_save_power_cycle_load_round_trips_modified_field(void) {
  MemoryConfigStore store;
  ConfigManager mgr(store);
  TEST_ASSERT_TRUE(mgr.begin());

  mgr.mutableConfig().selection.rssi_threshold_percent = 77;
  TEST_ASSERT_TRUE(mgr.save());

  store.simulatePowerCycle();

  ConfigManager mgr2(store);
  TEST_ASSERT_TRUE(mgr2.load());
  TEST_ASSERT_EQUAL_UINT8(77, mgr2.config().selection.rssi_threshold_percent);
  TEST_ASSERT_TRUE(mgr2.loadedFromStore());
}

void test_corrupted_crc32_falls_back_to_defaults(void) {
  MemoryConfigStore store;
  ConfigManager mgr(store);
  TEST_ASSERT_TRUE(mgr.begin());
  mgr.mutableConfig().selection.rssi_threshold_percent = 12;
  TEST_ASSERT_TRUE(mgr.save());

  RouterConfig corrupted = mgr.config();
  corrupted.crc32 ^= 0xFFFFFFFFu;
  TEST_ASSERT_TRUE(store.writeBlob(CONFIG_BLOB_KEY, &corrupted, sizeof(corrupted)));

  ConfigManager mgr2(store);
  TEST_ASSERT_FALSE(mgr2.load());
  mgr2.loadDefaults();
  RouterConfig defaults;
  configLoadDefaults(defaults);
  TEST_ASSERT_EQUAL_UINT8(defaults.selection.rssi_threshold_percent,
                           mgr2.config().selection.rssi_threshold_percent);
}

void test_version_mismatch_falls_back_to_defaults(void) {
  MemoryConfigStore store;
  ConfigManager mgr(store);
  TEST_ASSERT_TRUE(mgr.begin());

  RouterConfig bad_version = mgr.config();
  bad_version.version = CONFIG_VERSION + 1;
  bad_version.crc32 = configCrc32(bad_version);
  TEST_ASSERT_TRUE(store.writeBlob(CONFIG_BLOB_KEY, &bad_version, sizeof(bad_version)));

  ConfigManager mgr2(store);
  TEST_ASSERT_FALSE(mgr2.load());
}

void test_factory_reset_restores_defaults_and_erases_store(void) {
  MemoryConfigStore store;
  ConfigManager mgr(store);
  TEST_ASSERT_TRUE(mgr.begin());
  mgr.mutableConfig().network.ap_mode = false;
  TEST_ASSERT_TRUE(mgr.save());

  TEST_ASSERT_TRUE(mgr.factoryReset());
  RouterConfig defaults;
  configLoadDefaults(defaults);
  TEST_ASSERT_EQUAL(defaults.network.ap_mode, mgr.config().network.ap_mode);
  TEST_ASSERT_TRUE(store.hasKey(CONFIG_BLOB_KEY));
}

void test_revision_increments_on_save(void) {
  MemoryConfigStore store;
  ConfigManager mgr(store);
  TEST_ASSERT_TRUE(mgr.begin());
  uint32_t rev0 = mgr.revision();
  TEST_ASSERT_TRUE(mgr.save());
  TEST_ASSERT_TRUE(mgr.save());
  TEST_ASSERT_EQUAL_UINT32(rev0 + 2, mgr.revision());
}

void test_configValidate_clamps_out_of_range_value(void) {
  RouterConfig cfg;
  configLoadDefaults(cfg);
  cfg.selection.link_timeout_ms = 5;  // below the 50 ms floor
  bool no_change = configValidate(cfg);
  TEST_ASSERT_FALSE(no_change);
  TEST_ASSERT_EQUAL_UINT16(50, cfg.selection.link_timeout_ms);
}

void test_failed_write_returns_false(void) {
  MemoryConfigStore store;
  ConfigManager mgr(store);
  TEST_ASSERT_TRUE(mgr.begin());
  store.setFailWrites(true);
  TEST_ASSERT_FALSE(mgr.save());
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_defaults_are_valid);
  RUN_TEST(test_save_power_cycle_load_round_trips_modified_field);
  RUN_TEST(test_corrupted_crc32_falls_back_to_defaults);
  RUN_TEST(test_version_mismatch_falls_back_to_defaults);
  RUN_TEST(test_factory_reset_restores_defaults_and_erases_store);
  RUN_TEST(test_revision_increments_on_save);
  RUN_TEST(test_configValidate_clamps_out_of_range_value);
  RUN_TEST(test_failed_write_returns_false);
  return UNITY_END();
}
