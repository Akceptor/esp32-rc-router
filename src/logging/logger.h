#pragma once
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

enum class LogLevel : uint8_t { ERROR = 0, WARN = 1, INFO = 2, DEBUG = 3 };

class Logger {
 public:
  static Logger& instance();
  void begin(LogLevel level, bool serial_console);
  void setLevel(LogLevel level);
  LogLevel level() const;
  bool serialConsole() const;
  void setSerialConsole(bool enabled);
  void log(LogLevel level, const char* tag, const char* fmt, ...);
  void vlog(LogLevel level, const char* tag, const char* fmt, va_list args);
  size_t dump(char* out, size_t cap) const;   // newest-last, NUL terminated, returns bytes written
  void clear();
  uint32_t droppedLines() const;
 private:
  Logger();
  static const size_t kBufferBytes = 4096;
  char buffer_[kBufferBytes];
  size_t head_;
  size_t used_;
  LogLevel level_;
  bool serial_console_;
  uint32_t dropped_;
};

#define LOG_E(tag, ...) Logger::instance().log(LogLevel::ERROR, tag, __VA_ARGS__)
#define LOG_W(tag, ...) Logger::instance().log(LogLevel::WARN, tag, __VA_ARGS__)
#define LOG_I(tag, ...) Logger::instance().log(LogLevel::INFO, tag, __VA_ARGS__)
#define LOG_D(tag, ...) Logger::instance().log(LogLevel::DEBUG, tag, __VA_ARGS__)
