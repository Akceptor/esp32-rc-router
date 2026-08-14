#pragma once
#include <stddef.h>
#include <stdint.h>

enum class OtaState : uint8_t { IDLE = 0, RUNNING = 1, SUCCESS = 2, FAILED = 3 };

// OtaManager wraps the Arduino ESP32 core's `Update` flash-write state machine behind the
// project's plain-C++ interface style. It is Arduino-only (the `Update` global does not exist
// off-target), so the .cpp implementation is entirely inside `#if defined(ARDUINO)`. The header
// itself has no Arduino dependency so it stays includable from anywhere (e.g. web_server.h) even
// though nothing in the native test suite currently exercises it.
//
// Restart discipline: OtaManager NEVER calls ESP.restart() synchronously from handleChunk(). The
// final chunk arrives from inside AsyncWebServer's onUpload callback, which runs on the network
// task; restarting there would tear down the socket before the HTTP response ("{"ok":true}" or
// the final multipart ack) is flushed to the client, so the browser/curl would see a connection
// reset instead of a clean 200. Instead, handleChunk() only flips state to SUCCESS and records a
// timestamp; the actual ESP.restart() is issued from update() — called every 20 ms from App's
// serviceTask — once at least kRestartDelayMs have elapsed, by which time the HTTP stack has had
// several scheduler passes to flush the response.
class OtaManager {
 public:
  OtaManager();

  // hostname is stored for logging only (mDNS setup lives in App/WiFi bring-up, Task 21).
  bool begin(const char* hostname);

  // Called once per HTTP body chunk from the AsyncWebServer onUpload handler.
  // index == 0 marks the first chunk of a new upload (Update.begin() is called here).
  // final_chunk == true marks the last chunk (Update.end(true) is called after writing it).
  // total_size is the Content-Length if known, else 0 — used only for progress% math, never
  // passed to Update.begin() (which always receives UPDATE_SIZE_UNKNOWN per the OTA contract).
  // Returns false if the chunk was rejected (e.g. Update.begin()/Update.write() failed); the
  // caller (web_server.cpp) should abort the AsyncWebServerRequest in that case.
  bool handleChunk(size_t index, const uint8_t* data, size_t len, bool final_chunk,
                   size_t total_size);

  // Poll from App::serviceTask every 20 ms. Performs the deferred ESP.restart() once SUCCESS has
  // been stable for kRestartDelayMs. No-op in all other states.
  void update();

  OtaState state() const { return state_; }
  uint8_t progressPercent() const { return progress_percent_; }
  const char* lastError() const { return error_; }

  // Cancels an in-progress update (e.g. client disconnected mid-upload). Safe to call in any
  // state; no-op if not RUNNING.
  void abort();

  static const uint32_t kRestartDelayMs = 1500;

 private:
  OtaState state_;
  uint8_t progress_percent_;
  size_t bytes_written_;
  size_t total_size_;
  uint32_t success_at_ms_;
  char error_[64];
  const char* hostname_;

  void setError(const char* msg);
};
