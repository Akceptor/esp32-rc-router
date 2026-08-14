#pragma once
#include <stdint.h>
#include "web/web_server_types.h"
#include "config/config_manager.h"
#include "receiver/receiver_manager.h"
#include "output/output_manager.h"
#include "pwm/pwm_manager.h"
#include "telemetry/voltage_monitor.h"
#include "web/ota_manager.h"

#if defined(ARDUINO)
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#endif

// WebServerManager exposes the REST API described in the contract. All HTTP handling
// happens on ESPAsyncWebServer's own async TCP task, which runs concurrently with the
// rx/output/service FreeRTOS tasks (see src/app/app.h). Because ConfigManager's
// RouterConfig is read by the rx/output tasks every loop iteration, HTTP handlers must
// NEVER mutate ConfigManager directly. Instead:
//
//   1. A POST handler validates the JSON body shape (via config_json's *FromJson
//      helpers) against a *copy* of the current config (`pending_config_`), and if that
//      succeeds sets `pending_ = true` (a `volatile bool`, safe to set/clear across the
//      async task boundary because it is only ever read/written as a single aligned
//      byte and treated as a simple dirty flag, never used for synchronization of
//      larger data — pending_config_ itself is written before pending_ is set to true,
//      and applyPending() copies pending_config_ out before clearing pending_, always
//      called only from serviceTask, never concurrently with a new HTTP write because
//      pending_config_ is only re-armed by a *new* POST, and PlatformIO/FreeRTOS on the
//      ESP32 guarantee byte-aligned bool reads/writes are atomic at this word size).
//   2. `serviceTask` (see src/app/app.h, 20 ms period) calls `applyPending()` every
//      tick. If `pending_` is set, it copies `pending_config_` into
//      `cfg_.mutableConfig()`, runs `configValidate()`, calls `cfg_.save()`, clears
//      `pending_`, and returns true so `App::loop()`/the service task knows to re-apply
//      the new config to ReceiverManager/OutputManager/PwmManager/VoltageMonitor via
//      their respective `setConfig()` calls.
//
// This document only lists the REST handler bodies; the ESPAsyncWebServer wiring
// itself (server_.on(...) registration, multipart OTA handling, static file serving)
// is exercised on real hardware in Task 22. Native tests in this task cover only
// config_json.cpp/.h, which contain 100% of the JSON logic and none of the networking.
class WebServerManager {
 public:
  WebServerManager(ConfigManager& cfg, ReceiverManager& rx, OutputManager& out, PwmManager& pwm,
                    VoltageMonitor& volt, OtaManager& ota);

  bool begin(uint16_t port);
  void update(uint32_t now_ms);
  void setSnapshotSource(StatusSnapshot* snapshot);
  bool applyPending();

 private:
  ConfigManager& cfg_;
  ReceiverManager& rx_;
  OutputManager& out_;
  PwmManager& pwm_;
  VoltageMonitor& volt_;
  OtaManager& ota_;

  StatusSnapshot* snapshot_;

  RouterConfig pending_config_;
  volatile bool pending_;

#if defined(ARDUINO)
  AsyncWebServer server_;

  void registerRoutes();
  void handleGetStatus(AsyncWebServerRequest* request);
  void handleGetConfig(AsyncWebServerRequest* request);
  void handleGetSection(AsyncWebServerRequest* request, const char* section);
  void handlePostReceivers(AsyncWebServerRequest* request, JsonVariant& json);
  void handlePostOutput(AsyncWebServerRequest* request, JsonVariant& json);
  void handlePostPwm(AsyncWebServerRequest* request, JsonVariant& json);
  void handlePostVoltage(AsyncWebServerRequest* request, JsonVariant& json);
  void handlePostNetwork(AsyncWebServerRequest* request, JsonVariant& json);
  void handlePostSystem(AsyncWebServerRequest* request, JsonVariant& json);
  void handlePostVoltageCalibrate(AsyncWebServerRequest* request, JsonVariant& json);
  void handleGetLogs(AsyncWebServerRequest* request);
  void handlePostRestart(AsyncWebServerRequest* request);
  void handlePostFactoryReset(AsyncWebServerRequest* request);
  void handleGetOtaStatus(AsyncWebServerRequest* request);
  void sendOk(AsyncWebServerRequest* request, bool reboot_required);
  void sendError(AsyncWebServerRequest* request, const char* message);
#endif
};
