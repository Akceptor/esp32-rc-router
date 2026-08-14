#include "app/app.h"

#include <string.h>

#include "config/config_types.h"
#include "logging/logger.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#endif

namespace {
// Task tuning constants (see contract: App::*TaskEntry doc comments).
static const uint32_t kRxTaskPeriodMs = 1;
static const uint32_t kServiceTaskPeriodMs = 20;
static const uint32_t kWifiStaTimeoutMs = 15000;
static const char* const kApPasswordDefault = "rcrouter123";  // documented default AP password;
                                                               // changeable via Network page once
                                                               // reachable, or by wiring a new
                                                               // WiFi network and rebooting.
static const uint32_t kTaskWatchdogTimeoutS = 5;
}  // namespace

App& App::instance() {
  static App app;
  return app;
}

App::App()
    : rx_uart_a_(1),
      rx_uart_b_(2),
      out_uart_(0),
      status_led_(gpio_out_, 2, true),
      nvs_store_(),
      config_manager_(nvs_store_),
      rx_port_a_(0, rx_uart_a_),
      rx_port_b_(1, rx_uart_b_),
      receiver_manager_(rx_port_a_, rx_port_b_),
      output_manager_(out_uart_),
      pwm_manager_(gpio_out_),
      voltage_monitor_(adc_in_),
      telemetry_router_(receiver_manager_, output_manager_, voltage_monitor_),
      web_server_(config_manager_, receiver_manager_, output_manager_, pwm_manager_,
                  voltage_monitor_, ota_manager_),
      wifi_ap_mode_(false),
      boot_ms_(0) {
  memset(&shared_frame_, 0, sizeof(shared_frame_));
  rcFrameInit(shared_frame_.frame);
#if defined(ARDUINO)
  frame_mux_ = portMUX_INITIALIZER_UNLOCKED;
  rx_task_handle_ = nullptr;
  output_task_handle_ = nullptr;
  service_task_handle_ = nullptr;
#endif
}

void App::beginLogging() {
  // Early boot: default to INFO + serial console until config is loaded (a few hundred ms later),
  // matching the user-configurable SystemConfig fields once available.
  Logger::instance().begin(LogLevel::INFO, true);
  LOG_I("APP", "RC Signal Router firmware %s booting", FIRMWARE_VERSION);
}

bool App::beginFilesystemAndConfig() {
#if defined(ARDUINO)
  if (!LittleFS.begin(true)) {  // true = format on mount failure
    LOG_E("APP", "LittleFS mount failed even after format attempt");
    return false;
  }
#endif
  if (!config_manager_.begin()) {
    LOG_W("APP", "config load failed, defaults were applied and saved");
  }
  const RouterConfig& cfg = config_manager_.config();
  Logger::instance().setLevel(static_cast<LogLevel>(cfg.system.log_level));
  Logger::instance().setSerialConsole(cfg.system.serial_console);
  LOG_I("APP", "config loaded, revision=%u", config_manager_.revision());
  return true;
}

void App::applyConfigToModules(const RouterConfig& cfg) {
  rx_port_a_.setConfig(cfg.receivers[0]);
  rx_port_b_.setConfig(cfg.receivers[1]);
  receiver_manager_.setConfig(cfg.selection);
  output_manager_.setConfig(cfg.output);
  output_manager_.setFailsafe(cfg.system.failsafe_mode, cfg.system.failsafe_channels);
  for (uint8_t i = 0; i < PWM_PIN_COUNT; i++) {
    pwm_manager_.setConfig(i, cfg.pwm[i]);
  }
  pwm_manager_.setFailsafeMode(cfg.system.failsafe_mode);
  voltage_monitor_.setConfig(cfg.voltage);
  telemetry_router_.setVoltageOverride(cfg.voltage.telemetry_override);
}

void App::beginWifi(const NetworkConfig& net_cfg) {
#if defined(ARDUINO)
  if (!net_cfg.ap_mode && strlen(net_cfg.ssid) > 0) {
    WiFi.mode(WIFI_STA);
    if (!net_cfg.use_dhcp) {
      IPAddress ip(net_cfg.static_ip >> 24, (net_cfg.static_ip >> 16) & 0xff,
                   (net_cfg.static_ip >> 8) & 0xff, net_cfg.static_ip & 0xff);
      IPAddress gw(net_cfg.gateway >> 24, (net_cfg.gateway >> 16) & 0xff,
                   (net_cfg.gateway >> 8) & 0xff, net_cfg.gateway & 0xff);
      IPAddress mask(net_cfg.netmask >> 24, (net_cfg.netmask >> 16) & 0xff,
                     (net_cfg.netmask >> 8) & 0xff, net_cfg.netmask & 0xff);
      WiFi.config(ip, gw, mask);
    }
    WiFi.begin(net_cfg.ssid, net_cfg.password);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < kWifiStaTimeoutMs) {
      delay(100);
    }
  }

  if (net_cfg.ap_mode || WiFi.status() != WL_CONNECTED) {
    wifi_ap_mode_ = true;
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char ap_ssid[33];
    snprintf(ap_ssid, sizeof(ap_ssid), "RC-Router-%02X%02X", mac[4], mac[5]);
    // Default AP password is intentionally documented here (kApPasswordDefault = "rcrouter123")
    // rather than hidden: it is the only way to reach a brand-new or STA-unreachable device, and
    // must be changed via the Network page once connected if the deployment is not physically
    // secured.
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, kApPasswordDefault);
    LOG_W("APP", "WiFi STA failed/disabled, AP mode active: SSID=%s", ap_ssid);
  } else {
    wifi_ap_mode_ = false;
    LOG_I("APP", "WiFi STA connected, ip=%s", WiFi.localIP().toString().c_str());
  }
#else
  (void)net_cfg;
#endif
}

void App::beginMdns(const char* hostname) {
#if defined(ARDUINO)
  if (MDNS.begin(hostname)) {
    MDNS.addService("http", "tcp", 80);
    LOG_I("APP", "mDNS responder started: %s.local", hostname);
  } else {
    LOG_W("APP", "mDNS begin failed");
  }
#else
  (void)hostname;
#endif
}

bool App::setup() {
  boot_ms_ =
#if defined(ARDUINO)
      millis();
#else
      0;
#endif
  beginLogging();
  if (!beginFilesystemAndConfig()) {
    return false;
  }
  const RouterConfig& cfg = config_manager_.config();

  status_led_.begin();
  status_led_.setPattern(LedPattern::DOUBLE_BLINK);  // until link/WiFi state is known

  rx_port_a_.begin(cfg.receivers[0]);
  rx_port_b_.begin(cfg.receivers[1]);
  receiver_manager_.begin(cfg.selection);

  output_manager_.begin(cfg.output);
  output_manager_.setFailsafe(cfg.system.failsafe_mode, cfg.system.failsafe_channels);

  pwm_manager_.begin(cfg.pwm);
  pwm_manager_.setFailsafeMode(cfg.system.failsafe_mode);

  adc_in_.begin(cfg.voltage.adc_pin);
  voltage_monitor_.begin(cfg.voltage);
  telemetry_router_.begin(cfg.voltage.telemetry_override);

  beginWifi(cfg.network);
  beginMdns(cfg.network.hostname);

  ota_manager_.begin(cfg.network.hostname);
  web_server_.setSnapshotSource(&status_snapshot_);
  web_server_.begin(80);

  memset(&status_snapshot_, 0, sizeof(status_snapshot_));
  strncpy(status_snapshot_.firmware_version, FIRMWARE_VERSION,
          sizeof(status_snapshot_.firmware_version) - 1);

  createTasks();

  LOG_I("APP", "setup complete");
  return true;
}

void App::createTasks() {
#if defined(ARDUINO)
  // ESP-IDF 5.x's esp_task_wdt_init() takes a config struct rather than (timeout, panic) args.
  esp_task_wdt_config_t wdt_cfg = {};
  wdt_cfg.timeout_ms = kTaskWatchdogTimeoutS * 1000;
  wdt_cfg.idle_core_mask = 0;
  wdt_cfg.trigger_panic = true;  // panic (reboot) on timeout
  esp_task_wdt_init(&wdt_cfg);

  xTaskCreatePinnedToCore(&App::rxTaskEntry, "rxTask", 4096, this, 5, &rx_task_handle_, 1);
  xTaskCreatePinnedToCore(&App::outputTaskEntry, "outputTask", 4096, this, 5,
                          &output_task_handle_, 1);
  xTaskCreatePinnedToCore(&App::serviceTaskEntry, "serviceTask", 8192, this, 1,
                          &service_task_handle_, 0);

  esp_task_wdt_add(rx_task_handle_);
  esp_task_wdt_add(output_task_handle_);
  esp_task_wdt_add(service_task_handle_);
#endif
}

void App::loop() {
  // All real work happens in the three FreeRTOS tasks created in setup(). The Arduino loop()
  // task is otherwise idle; keep it parked so it never contends for CPU with rx/output/service.
#if defined(ARDUINO)
  vTaskDelay(pdMS_TO_TICKS(1000));
#endif
}

// ------------------------------------------------------------------------------------------
// rxTask: highest-priority, tightest period. Pumps both receiver ports + the selection FSM,
// then publishes the winning frame into the double-buffered shared struct under a spinlock so
// outputTask never blocks rxTask (a spinlock critical section here is at most a few dozen byte
// copies — sub-microsecond — never a syscall or wait).
// ------------------------------------------------------------------------------------------
void App::rxTaskEntry(void* arg) {
  static_cast<App*>(arg)->rxTaskLoop();
}

void App::rxTaskLoop() {
#if defined(ARDUINO)
  for (;;) {
    uint32_t now = millis();
    receiver_manager_.update(now);

    portENTER_CRITICAL(&frame_mux_);
    shared_frame_.frame = receiver_manager_.activeFrame();
    shared_frame_.link_valid = receiver_manager_.hasValidLink();
    shared_frame_.seq++;
    portEXIT_CRITICAL(&frame_mux_);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(kRxTaskPeriodMs));
  }
#endif
}

// ------------------------------------------------------------------------------------------
// outputTask: reads the shared frame under the same spinlock, drives OutputManager (protocol
// generation toward the FC) and PwmManager (direct servo/switch outputs) every protocol frame
// interval. Both managers are non-blocking (fixed-size buffers, UART FIFO writes), so this task
// never waits on I/O beyond the vTaskDelay.
// ------------------------------------------------------------------------------------------
void App::outputTaskEntry(void* arg) {
  static_cast<App*>(arg)->outputTaskLoop();
}

void App::outputTaskLoop() {
#if defined(ARDUINO)
  for (;;) {
    uint32_t now = millis();
    RCFrame frame;
    bool link_valid;

    portENTER_CRITICAL(&frame_mux_);
    frame = shared_frame_.frame;
    link_valid = shared_frame_.link_valid;
    portEXIT_CRITICAL(&frame_mux_);

    output_manager_.update(now, frame, link_valid);
    pwm_manager_.update(now, frame, link_valid);

    esp_task_wdt_reset();
    uint16_t interval_ms = output_manager_.frameIntervalMs();
    if (interval_ms == 0) interval_ms = 4;
    vTaskDelay(pdMS_TO_TICKS(interval_ms));
  }
#endif
}

// ------------------------------------------------------------------------------------------
// serviceTask: lowest priority, 20 ms period, everything that is not latency-critical: voltage
// sampling, telemetry pumping in both directions, LED pattern animation, the web server's
// internal bookkeeping, OTA state machine polling, and applying any config changes the web UI
// queued without requiring a reboot.
// ------------------------------------------------------------------------------------------
void App::serviceTaskEntry(void* arg) {
  static_cast<App*>(arg)->serviceTaskLoop();
}

void App::serviceTaskLoop() {
#if defined(ARDUINO)
  for (;;) {
    uint32_t now = millis();

    voltage_monitor_.update(now);
    telemetry_router_.update(now);
    updateLedPattern();
    status_led_.update(now);
    web_server_.update(now);
    ota_manager_.update();

    // Live re-apply of any config the web UI just saved. Everything EXCEPT WiFi credentials and
    // static IP addressing can be applied without a reboot (those two require re-associating or
    // re-binding the network stack, which WebServerManager instead reports back to the browser
    // as reboot_required=true so the user can choose when to restart).
    if (web_server_.applyPending()) {
      applyConfigToModules(config_manager_.config());
    }

    // refresh the shared StatusSnapshot the web server reads from.
    status_snapshot_.active_receiver = receiver_manager_.activeIndex();
    status_snapshot_.active_protocol = receiver_manager_.activePort()
                                            ? receiver_manager_.activePort()->protocol()
                                            : ProtocolType::NONE;
    status_snapshot_.rssi_percent = receiver_manager_.activeLink().rssi_percent;
    status_snapshot_.lq_percent = receiver_manager_.activeLink().lq_percent;
    status_snapshot_.failsafe = receiver_manager_.activeLink().failsafe;
    status_snapshot_.battery_voltage = voltage_monitor_.voltage();
    status_snapshot_.adc_millivolts = voltage_monitor_.pinMillivolts();
    status_snapshot_.uptime_s = (now - boot_ms_) / 1000;
    status_snapshot_.wifi_connected = (WiFi.status() == WL_CONNECTED);
    status_snapshot_.wifi_ap_mode = wifi_ap_mode_;
    status_snapshot_.wifi_rssi = wifi_ap_mode_ ? 0 : static_cast<int8_t>(WiFi.RSSI());
    IPAddress ip = wifi_ap_mode_ ? WiFi.softAPIP() : WiFi.localIP();
    status_snapshot_.ip = (static_cast<uint32_t>(ip[0]) << 24) |
                           (static_cast<uint32_t>(ip[1]) << 16) |
                           (static_cast<uint32_t>(ip[2]) << 8) | ip[3];
    status_snapshot_.free_heap = ESP.getFreeHeap();
    status_snapshot_.switch_count = receiver_manager_.switchCount();
    strncpy(status_snapshot_.firmware_version, FIRMWARE_VERSION,
            sizeof(status_snapshot_.firmware_version) - 1);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(kServiceTaskPeriodMs));
  }
#endif
}

// LED pattern selection (contract semantics):
//   SOLID        = active receiver + link good
//   SLOW_BLINK   = link degraded / backup active
//   FAST_BLINK   = failsafe / no receiver
//   DOUBLE_BLINK = WiFi AP/config mode
//   TRIPLE_BLINK = OTA in progress
//   OFF          = disabled (not used here; reserved for a future "LED disable" config option)
// Priority order (highest first): OTA > failsafe/no-link > AP mode > degraded/backup > solid-good.
void App::updateLedPattern() {
  if (ota_manager_.state() == OtaState::RUNNING) {
    status_led_.setPattern(LedPattern::TRIPLE_BLINK);
    return;
  }
  if (!receiver_manager_.hasValidLink() ||
      receiver_manager_.state() == SelectionState::NO_LINK ||
      receiver_manager_.activeLink().failsafe) {
    status_led_.setPattern(LedPattern::FAST_BLINK);
    return;
  }
  if (wifi_ap_mode_) {
    status_led_.setPattern(LedPattern::DOUBLE_BLINK);
    return;
  }
  if (receiver_manager_.state() == SelectionState::EVALUATING ||
      receiver_manager_.state() == SelectionState::SWITCHING ||
      receiver_manager_.activeIndex() != 0) {
    // backup port active (index != 0, the highest-priority port) or the FSM is mid-evaluation
    status_led_.setPattern(LedPattern::SLOW_BLINK);
    return;
  }
  status_led_.setPattern(LedPattern::SOLID);
}
