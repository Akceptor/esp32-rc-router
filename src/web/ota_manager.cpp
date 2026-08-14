#include "web/ota_manager.h"

#include <string.h>

#if defined(ARDUINO)
#include <Arduino.h>
#include <Update.h>
#endif

OtaManager::OtaManager()
    : state_(OtaState::IDLE),
      progress_percent_(0),
      bytes_written_(0),
      total_size_(0),
      success_at_ms_(0),
      hostname_(nullptr) {
  error_[0] = '\0';
}

void OtaManager::setError(const char* msg) {
  strncpy(error_, msg, sizeof(error_) - 1);
  error_[sizeof(error_) - 1] = '\0';
}

bool OtaManager::begin(const char* hostname) {
  hostname_ = hostname;
  state_ = OtaState::IDLE;
  progress_percent_ = 0;
  bytes_written_ = 0;
  total_size_ = 0;
  error_[0] = '\0';
  return true;
}

#if defined(ARDUINO)

bool OtaManager::handleChunk(size_t index, const uint8_t* data, size_t len, bool final_chunk,
                              size_t total_size) {
  if (index == 0) {
    bytes_written_ = 0;
    total_size_ = total_size;
    error_[0] = '\0';
    state_ = OtaState::RUNNING;
    progress_percent_ = 0;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      setError(Update.errorString());
      state_ = OtaState::FAILED;
      return false;
    }
  }

  if (state_ != OtaState::RUNNING) {
    return false;
  }

  if (len > 0) {
    size_t written = Update.write(const_cast<uint8_t*>(data), len);
    if (written != len) {
      setError(Update.errorString());
      state_ = OtaState::FAILED;
      Update.abort();
      return false;
    }
    bytes_written_ += written;
  }

  if (total_size_ > 0) {
    uint32_t pct = static_cast<uint32_t>((bytes_written_ * 100ULL) / total_size_);
    progress_percent_ = pct > 100 ? 100 : static_cast<uint8_t>(pct);
  } else {
    progress_percent_ = 0;
  }

  if (final_chunk) {
    if (Update.end(true)) {
      state_ = OtaState::SUCCESS;
      progress_percent_ = 100;
      success_at_ms_ = millis();
    } else {
      setError(Update.errorString());
      state_ = OtaState::FAILED;
    }
  }
  return true;
}

void OtaManager::update() {
  if (state_ == OtaState::SUCCESS) {
    if (millis() - success_at_ms_ >= kRestartDelayMs) {
      ESP.restart();
    }
  }
}

void OtaManager::abort() {
  if (state_ == OtaState::RUNNING) {
    Update.abort();
    setError("aborted");
    state_ = OtaState::FAILED;
  }
}

#else  // !ARDUINO — native/off-target stub so the header stays safely includable everywhere.

bool OtaManager::handleChunk(size_t, const uint8_t*, size_t, bool, size_t) {
  setError("OTA not available off-target");
  state_ = OtaState::FAILED;
  return false;
}

void OtaManager::update() {}

void OtaManager::abort() {
  state_ = OtaState::IDLE;
}

#endif
