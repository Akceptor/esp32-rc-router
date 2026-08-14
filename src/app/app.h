#pragma once
#include <stdint.h>

#include "config/config_manager.h"
#include "hal/adc_input.h"
#include "hal/gpio_output.h"
#include "hal/status_led.h"
#include "hal/uart_port.h"
#include "output/output_manager.h"
#include "protocols/protocol_types.h"
#include "pwm/pwm_manager.h"
#include "receiver/receiver_manager.h"
#include "receiver/receiver_port.h"
#include "telemetry/telemetry_router.h"
#include "telemetry/voltage_monitor.h"
#include "web/ota_manager.h"
#include "web/web_server.h"

#if defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#endif

static const char* const FIRMWARE_VERSION = "1.0.0";

// App owns every long-lived module instance as a plain member (static storage duration via the
// App singleton itself — see instance() below), so there is zero heap allocation for the object
// graph. It is the only class in the firmware (besides the ESP32 HAL implementations) allowed to
// call millis()/vTaskDelay() directly; every module underneath takes time as an explicit
// parameter so it stays natively testable.
class App {
 public:
  static App& instance();

  bool setup();
  void loop();

 private:
  App();
  App(const App&) = delete;
  App& operator=(const App&) = delete;

  // ---- FreeRTOS task trampolines ----
  static void rxTaskEntry(void* arg);
  static void outputTaskEntry(void* arg);
  static void serviceTaskEntry(void* arg);

  void rxTaskLoop();
  void outputTaskLoop();
  void serviceTaskLoop();

  // ---- boot sub-steps ----
  void beginLogging();
  bool beginFilesystemAndConfig();
  void applyConfigToModules(const RouterConfig& cfg);
  void beginWifi(const NetworkConfig& net_cfg);
  void beginMdns(const char* hostname);
  void createTasks();
  void updateLedPattern();

  // ---- HAL instances (static storage, no heap) ----
  Esp32UartPort rx_uart_a_;
  Esp32UartPort rx_uart_b_;
  Esp32UartPort out_uart_;
  Esp32GpioOutput gpio_out_;
  Esp32AdcInput adc_in_;
  StatusLed status_led_;

  // ---- config ----
  NvsConfigStore nvs_store_;
  ConfigManager config_manager_;

  // ---- receive path ----
  ReceiverPort rx_port_a_;
  ReceiverPort rx_port_b_;
  ReceiverManager receiver_manager_;

  // ---- output path ----
  OutputManager output_manager_;
  PwmManager pwm_manager_;

  // ---- telemetry ----
  VoltageMonitor voltage_monitor_;
  TelemetryRouter telemetry_router_;

  // ---- web / OTA ----
  OtaManager ota_manager_;
  WebServerManager web_server_;
  StatusSnapshot status_snapshot_;

  // ---- shared frame handed off rxTask -> outputTask under a spinlock ----
  struct SharedFrame {
    RCFrame frame;
    bool link_valid;
    uint32_t seq;
  };
  SharedFrame shared_frame_;
#if defined(ARDUINO)
  portMUX_TYPE frame_mux_;
  TaskHandle_t rx_task_handle_;
  TaskHandle_t output_task_handle_;
  TaskHandle_t service_task_handle_;
#endif

  bool wifi_ap_mode_;
  uint32_t boot_ms_;
};
