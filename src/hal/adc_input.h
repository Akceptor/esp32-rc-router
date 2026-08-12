#pragma once
#include <stdint.h>

class IAdcInput {
 public:
  virtual ~IAdcInput() {}
  virtual bool begin(uint8_t pin) = 0;
  virtual uint16_t readRaw() = 0;         // 0..4095
  virtual uint32_t readAveragedMv(uint8_t samples) = 0;  // millivolts at the pin
};

#if defined(ARDUINO)
class Esp32AdcInput : public IAdcInput { /* analogReadMilliVolts + rolling average */
 public:
  Esp32AdcInput();
  bool begin(uint8_t pin) override;
  uint16_t readRaw() override;
  uint32_t readAveragedMv(uint8_t samples) override;

 private:
  uint8_t pin_;
};
#endif

class MockAdcInput : public IAdcInput {
 public:
  MockAdcInput();
  bool begin(uint8_t pin) override;
  uint16_t readRaw() override;
  uint32_t readAveragedMv(uint8_t samples) override;

  void setMv(uint32_t mv);
  void setRaw(uint16_t raw);
  uint32_t readCount() const;

 private:
  uint8_t pin_;
  uint16_t raw_;
  uint32_t mv_;
  uint32_t read_count_;
};
