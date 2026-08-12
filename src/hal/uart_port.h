#pragma once
#include <stddef.h>
#include <stdint.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

class IUartPort {
 public:
  virtual ~IUartPort() {}
  virtual bool begin(uint32_t baud, uint32_t serial_config, int8_t rx_pin, int8_t tx_pin,
                     bool inverted) = 0;
  virtual void end() = 0;
  virtual size_t available() = 0;
  virtual size_t read(uint8_t* buf, size_t len) = 0;
  virtual size_t write(const uint8_t* buf, size_t len) = 0;
  virtual void flush() = 0;
};

static const uint32_t UART_CONFIG_8N1 = 0x800001cu;  // SERIAL_8N1
static const uint32_t UART_CONFIG_8E2 = 0x8000036u;  // SERIAL_8E2 (SBUS)

#if defined(ARDUINO)
class Esp32UartPort : public IUartPort {
 public:
  explicit Esp32UartPort(uint8_t uart_num);
  bool begin(uint32_t baud, uint32_t serial_config, int8_t rx_pin, int8_t tx_pin,
             bool inverted) override;
  void end() override;
  size_t available() override;
  size_t read(uint8_t* buf, size_t len) override;
  size_t write(const uint8_t* buf, size_t len) override;
  void flush() override;

 private:
  uint8_t uart_num_;
  HardwareSerial* serial_;
};
#endif

class MockUartPort : public IUartPort {          // available in native tests AND firmware-less builds
 public:
  MockUartPort();
  bool begin(uint32_t baud, uint32_t serial_config, int8_t rx_pin, int8_t tx_pin,
             bool inverted) override;
  void end() override;
  size_t available() override;
  size_t read(uint8_t* buf, size_t len) override;
  size_t write(const uint8_t* buf, size_t len) override;
  void flush() override;

  void injectRx(const uint8_t* data, size_t len);
  size_t txSize() const;
  const uint8_t* txData() const;
  void clearTx();
  bool begun() const;
  uint32_t lastBaud() const;
  bool lastInverted() const;
 private:
  uint8_t rx_[512]; size_t rx_head_; size_t rx_len_;
  uint8_t tx_[512]; size_t tx_len_;
  bool begun_;
  uint32_t last_baud_;
  bool last_inverted_;
};
