#include "logging/logger.h"

#include <stdio.h>
#include <string.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

namespace {

const char* levelName(LogLevel level) {
  switch (level) {
    case LogLevel::ERROR: return "ERROR";
    case LogLevel::WARN:  return "WARN";
    case LogLevel::INFO:  return "INFO";
    case LogLevel::DEBUG: return "DEBUG";
  }
  return "?";
}

}  // namespace

Logger::Logger()
    : head_(0), used_(0), level_(LogLevel::INFO), serial_console_(false), dropped_(0) {
  memset(buffer_, 0, sizeof(buffer_));
}

Logger& Logger::instance() {
  static Logger logger;
  return logger;
}

void Logger::begin(LogLevel level, bool serial_console) {
  level_ = level;
  serial_console_ = serial_console;
  clear();
}

void Logger::setLevel(LogLevel level) { level_ = level; }
LogLevel Logger::level() const { return level_; }
bool Logger::serialConsole() const { return serial_console_; }
void Logger::setSerialConsole(bool enabled) { serial_console_ = enabled; }

void Logger::log(LogLevel level, const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog(level, tag, fmt, args);
  va_end(args);
}

void Logger::vlog(LogLevel level, const char* tag, const char* fmt, va_list args) {
  if (static_cast<uint8_t>(level) > static_cast<uint8_t>(level_)) {
    return;
  }

  // Build "[LEVEL][tag] msg\n" directly into a single 160-byte line buffer.
  char line[160];
  int prefix_len = snprintf(line, sizeof(line), "[%s][%s] ", levelName(level), tag);
  if (prefix_len < 0) {
    prefix_len = 0;
  }
  if (static_cast<size_t>(prefix_len) > sizeof(line) - 2) {
    prefix_len = static_cast<int>(sizeof(line) - 2);
  }
  int msg_len = vsnprintf(line + prefix_len, sizeof(line) - prefix_len - 1, fmt, args);
  if (msg_len < 0) {
    msg_len = 0;
  }
  size_t body_len = static_cast<size_t>(prefix_len) + static_cast<size_t>(msg_len);
  if (body_len > sizeof(line) - 2) {
    body_len = sizeof(line) - 2;
  }
  line[body_len] = '\n';
  line[body_len + 1] = '\0';
  size_t line_len = body_len + 1;  // includes trailing '\n', excludes the NUL

  // Make room by dropping whole oldest lines (never partial lines).
  while (used_ + line_len > kBufferBytes && used_ > 0) {
    size_t oldest_start = (head_ + kBufferBytes - used_) % kBufferBytes;
    size_t idx = oldest_start;
    size_t oldest_len = 0;
    while (oldest_len < used_) {
      char c = buffer_[idx];
      idx = (idx + 1) % kBufferBytes;
      oldest_len++;
      if (c == '\n') {
        break;
      }
    }
    used_ -= oldest_len;
    dropped_++;
  }

  if (line_len > kBufferBytes) {
    line_len = kBufferBytes;  // pathological case: single line larger than buffer
  }

  for (size_t i = 0; i < line_len; i++) {
    buffer_[head_] = line[i];
    head_ = (head_ + 1) % kBufferBytes;
  }
  used_ += line_len;

#if defined(ARDUINO)
  if (serial_console_) {
    Serial.print(line);
  }
#endif
}

size_t Logger::dump(char* out, size_t cap) const {
  if (cap == 0) {
    return 0;
  }
  size_t n = used_;
  if (n > cap - 1) {
    n = cap - 1;
  }
  size_t skip = used_ - n;
  size_t start = (head_ + kBufferBytes - used_) % kBufferBytes;
  start = (start + skip) % kBufferBytes;
  for (size_t i = 0; i < n; i++) {
    out[i] = buffer_[(start + i) % kBufferBytes];
  }
  out[n] = '\0';
  return n;
}

void Logger::clear() {
  head_ = 0;
  used_ = 0;
  dropped_ = 0;
  memset(buffer_, 0, sizeof(buffer_));
}

uint32_t Logger::droppedLines() const { return dropped_; }
