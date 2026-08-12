#include "config/config_manager.h"

#include <string.h>

#if defined(ARDUINO)

NvsConfigStore::NvsConfigStore() {}

bool NvsConfigStore::begin() { return prefs_.begin("rcrouter", false); }

bool NvsConfigStore::readBlob(const char* key, void* out, size_t len) {
  size_t n = prefs_.getBytes(key, out, len);
  return n == len;
}

bool NvsConfigStore::writeBlob(const char* key, const void* in, size_t len) {
  size_t n = prefs_.putBytes(key, in, len);
  return n == len;
}

bool NvsConfigStore::eraseAll() { return prefs_.clear(); }

#endif  // defined(ARDUINO)

MemoryConfigStore::MemoryConfigStore()
    : has_data_(false), data_len_(0), fail_writes_(false), write_count_(0) {
  memset(key_, 0, sizeof(key_));
  memset(data_, 0, sizeof(data_));
}

bool MemoryConfigStore::begin() { return true; }

bool MemoryConfigStore::readBlob(const char* key, void* out, size_t len) {
  if (!has_data_) return false;
  if (strncmp(key_, key, sizeof(key_)) != 0) return false;
  if (len != data_len_) return false;
  memcpy(out, data_, len);
  return true;
}

bool MemoryConfigStore::writeBlob(const char* key, const void* in, size_t len) {
  if (fail_writes_) return false;
  if (len > kMaxLen) return false;
  strncpy(key_, key, sizeof(key_) - 1);
  memcpy(data_, in, len);
  data_len_ = len;
  has_data_ = true;
  write_count_++;
  return true;
}

bool MemoryConfigStore::eraseAll() {
  has_data_ = false;
  data_len_ = 0;
  memset(key_, 0, sizeof(key_));
  memset(data_, 0, sizeof(data_));
  return true;
}

void MemoryConfigStore::simulatePowerCycle() {
  // MemoryConfigStore models durable storage: its contents are already
  // retained across this no-op, mirroring how NVS survives a reboot.
}

bool MemoryConfigStore::hasKey(const char* key) const {
  return has_data_ && strncmp(key_, key, sizeof(key_)) == 0;
}

uint32_t MemoryConfigStore::writeCount() const { return write_count_; }

void MemoryConfigStore::setFailWrites(bool fail) { fail_writes_ = fail; }

ConfigManager::ConfigManager(IConfigStore& store)
    : store_(store), revision_(0), loaded_from_store_(false) {
  configLoadDefaults(config_);
}

bool ConfigManager::begin() {
  store_.begin();
  if (!load()) {
    loadDefaults();
    return save();
  }
  return true;
}

bool ConfigManager::load() {
  RouterConfig tmp;
  if (!store_.readBlob(CONFIG_BLOB_KEY, &tmp, sizeof(tmp))) {
    loaded_from_store_ = false;
    return false;
  }
  if (tmp.version != CONFIG_VERSION) {
    loaded_from_store_ = false;
    return false;
  }
  uint32_t stored_crc = tmp.crc32;
  uint32_t computed = configCrc32(tmp);
  if (stored_crc != computed) {
    loaded_from_store_ = false;
    return false;
  }
  configValidate(tmp);
  config_ = tmp;
  loaded_from_store_ = true;
  return true;
}

bool ConfigManager::save() {
  configValidate(config_);
  config_.crc32 = configCrc32(config_);
  if (!store_.writeBlob(CONFIG_BLOB_KEY, &config_, sizeof(config_))) {
    return false;
  }
  revision_++;
  return true;
}

void ConfigManager::loadDefaults() {
  configLoadDefaults(config_);
  loaded_from_store_ = false;
}

bool ConfigManager::factoryReset() {
  store_.eraseAll();
  loadDefaults();
  return save();
}

const RouterConfig& ConfigManager::config() const { return config_; }
RouterConfig& ConfigManager::mutableConfig() { return config_; }
uint32_t ConfigManager::revision() const { return revision_; }
bool ConfigManager::loadedFromStore() const { return loaded_from_store_; }
