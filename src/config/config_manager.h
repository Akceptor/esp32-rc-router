#pragma once
#include <stddef.h>
#include <stdint.h>
#include "config/config_types.h"

#if defined(ARDUINO)
#include <Preferences.h>
#endif

class IConfigStore {
 public:
  virtual ~IConfigStore() {}
  virtual bool begin() = 0;
  virtual bool readBlob(const char* key, void* out, size_t len) = 0;
  virtual bool writeBlob(const char* key, const void* in, size_t len) = 0;
  virtual bool eraseAll() = 0;
};

#if defined(ARDUINO)
class NvsConfigStore : public IConfigStore {
 public:
  NvsConfigStore();
  bool begin() override;
  bool readBlob(const char* key, void* out, size_t len) override;
  bool writeBlob(const char* key, const void* in, size_t len) override;
  bool eraseAll() override;

 private:
  Preferences prefs_;
};
#endif

class MemoryConfigStore : public IConfigStore {
 public:
  MemoryConfigStore();
  bool begin() override;
  bool readBlob(const char* key, void* out, size_t len) override;
  bool writeBlob(const char* key, const void* in, size_t len) override;
  bool eraseAll() override;

  void simulatePowerCycle();
  bool hasKey(const char* key) const;
  uint32_t writeCount() const;
  void setFailWrites(bool fail);

 private:
  static const size_t kMaxLen = 512;
  bool has_data_;
  char key_[16];
  uint8_t data_[kMaxLen];
  size_t data_len_;
  bool fail_writes_;
  uint32_t write_count_;
};

static const char* const CONFIG_BLOB_KEY = "cfg";

class ConfigManager {
 public:
  explicit ConfigManager(IConfigStore& store);
  bool begin();                       // load(); on failure loadDefaults()+save()
  bool load();
  bool save();
  void loadDefaults();
  bool factoryReset();                // eraseAll + defaults + save
  const RouterConfig& config() const;
  RouterConfig& mutableConfig();      // caller must call save()
  uint32_t revision() const;          // bumped on every successful save
  bool loadedFromStore() const;

 private:
  IConfigStore& store_;
  RouterConfig config_;
  uint32_t revision_;
  bool loaded_from_store_;
};
