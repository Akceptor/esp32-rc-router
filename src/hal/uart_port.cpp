#include "hal/uart_port.h"

#include <string.h>

#if defined(ARDUINO)

Esp32UartPort::Esp32UartPort(uint8_t uart_num) : uart_num_(uart_num), serial_(nullptr) {
  switch (uart_num_) {
    case 0: serial_ = &Serial; break;
    case 1: serial_ = &Serial1; break;
    case 2: serial_ = &Serial2; break;
    default: serial_ = &Serial1; break;
  }
}

bool Esp32UartPort::begin(uint32_t baud, uint32_t serial_config, int8_t rx_pin, int8_t tx_pin,
                           bool inverted) {
  if (serial_ == nullptr) {
    return false;
  }
  serial_->begin(baud, serial_config, rx_pin, tx_pin, inverted);
  return true;
}

void Esp32UartPort::end() {
  if (serial_ != nullptr) {
    serial_->end();
  }
}

size_t Esp32UartPort::available() {
  return serial_ != nullptr ? static_cast<size_t>(serial_->available()) : 0;
}

size_t Esp32UartPort::read(uint8_t* buf, size_t len) {
  if (serial_ == nullptr) {
    return 0;
  }
  return serial_->readBytes(buf, len);
}

size_t Esp32UartPort::write(const uint8_t* buf, size_t len) {
  if (serial_ == nullptr) {
    return 0;
  }
  return serial_->write(buf, len);
}

void Esp32UartPort::flush() {
  if (serial_ != nullptr) {
    serial_->flush();
  }
}

#endif  // defined(ARDUINO)

MockUartPort::MockUartPort()
    : rx_head_(0), rx_len_(0), tx_len_(0), begun_(false), last_baud_(0), last_inverted_(false) {
  memset(rx_, 0, sizeof(rx_));
  memset(tx_, 0, sizeof(tx_));
}

bool MockUartPort::begin(uint32_t baud, uint32_t serial_config, int8_t rx_pin, int8_t tx_pin,
                          bool inverted) {
  (void)serial_config;
  (void)rx_pin;
  (void)tx_pin;
  begun_ = true;
  last_baud_ = baud;
  last_inverted_ = inverted;
  return true;
}

void MockUartPort::end() { begun_ = false; }

size_t MockUartPort::available() { return rx_len_; }

size_t MockUartPort::read(uint8_t* buf, size_t len) {
  size_t n = (len < rx_len_) ? len : rx_len_;
  for (size_t i = 0; i < n; i++) {
    buf[i] = rx_[(rx_head_ + i) % sizeof(rx_)];
  }
  rx_head_ = (rx_head_ + n) % sizeof(rx_);
  rx_len_ -= n;
  return n;
}

size_t MockUartPort::write(const uint8_t* buf, size_t len) {
  size_t n = 0;
  for (; n < len && tx_len_ < sizeof(tx_); n++) {
    tx_[tx_len_++] = buf[n];
  }
  return n;
}

void MockUartPort::flush() {}

void MockUartPort::injectRx(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len && rx_len_ < sizeof(rx_); i++) {
    size_t tail = (rx_head_ + rx_len_) % sizeof(rx_);
    rx_[tail] = data[i];
    rx_len_++;
  }
}

size_t MockUartPort::txSize() const { return tx_len_; }
const uint8_t* MockUartPort::txData() const { return tx_; }
void MockUartPort::clearTx() { tx_len_ = 0; }
bool MockUartPort::begun() const { return begun_; }
uint32_t MockUartPort::lastBaud() const { return last_baud_; }
bool MockUartPort::lastInverted() const { return last_inverted_; }
