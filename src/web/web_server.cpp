#include "web/web_server.h"
#include "web/config_json.h"

#if defined(ARDUINO)
#include <LittleFS.h>
#include "logging/logger.h"

static const char* kTag = "WEB";
#endif

WebServerManager::WebServerManager(ConfigManager& cfg, ReceiverManager& rx, OutputManager& out,
                                    PwmManager& pwm, VoltageMonitor& volt, OtaManager& ota)
    : cfg_(cfg),
      rx_(rx),
      out_(out),
      pwm_(pwm),
      volt_(volt),
      ota_(ota),
      snapshot_(nullptr),
      pending_config_(),
      pending_(false)
#if defined(ARDUINO)
      ,
      server_(80)
#endif
{
}

void WebServerManager::setSnapshotSource(StatusSnapshot* snapshot) { snapshot_ = snapshot; }

bool WebServerManager::applyPending() {
  if (!pending_) return false;
  RouterConfig merged = pending_config_;
  bool unchanged_by_clamp = configValidate(merged);
  (void)unchanged_by_clamp;   // configValidate() clamps in place; we always accept the clamped result
  cfg_.mutableConfig() = merged;
  bool saved = cfg_.save();
  pending_ = false;
  return saved;
}

#if defined(ARDUINO)

bool WebServerManager::begin(uint16_t port) {
  server_ = AsyncWebServer(port);
  registerRoutes();
  server_.begin();
  LOG_I(kTag, "web server listening on port %u", (unsigned)port);
  return true;
}

void WebServerManager::update(uint32_t now_ms) {
  (void)now_ms;
  // ESPAsyncWebServer runs its own task; nothing to pump here besides applyPending(),
  // which App's serviceTask calls directly on its own 20 ms cadence.
}

void WebServerManager::sendOk(AsyncWebServerRequest* request, bool reboot_required) {
  JsonDocument doc;
  doc["ok"] = true;
  doc["reboot_required"] = reboot_required;
  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

void WebServerManager::sendError(AsyncWebServerRequest* request, const char* message) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["error"] = message;
  String body;
  serializeJson(doc, body);
  request->send(400, "application/json", body);
}

void WebServerManager::handleGetStatus(AsyncWebServerRequest* request) {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  if (snapshot_ != nullptr) {
    statusToJson(*snapshot_, root);
  }
  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

void WebServerManager::handleGetConfig(AsyncWebServerRequest* request) {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  const RouterConfig& c = cfg_.config();
  receiversToJson(c, root);
  outputToJson(c, root);
  pwmToJson(c, root);
  voltageToJson(c, root);
  networkToJson(c, root);
  systemToJson(c, root);
  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

void WebServerManager::handleGetSection(AsyncWebServerRequest* request, const char* section) {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  const RouterConfig& c = cfg_.config();
  if (strcmp(section, "receivers") == 0) receiversToJson(c, root);
  else if (strcmp(section, "output") == 0) outputToJson(c, root);
  else if (strcmp(section, "pwm") == 0) pwmToJson(c, root);
  else if (strcmp(section, "voltage") == 0) voltageToJson(c, root);
  else if (strcmp(section, "network") == 0) networkToJson(c, root);
  else if (strcmp(section, "system") == 0) systemToJson(c, root);
  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

void WebServerManager::handlePostReceivers(AsyncWebServerRequest* request, JsonVariant& json) {
  RouterConfig staged = pending_ ? pending_config_ : cfg_.config();
  if (!receiversFromJson(json.as<JsonObjectConst>(), staged)) {
    sendError(request, "invalid receivers payload");
    return;
  }
  pending_config_ = staged;
  pending_ = true;
  sendOk(request, true);
}

void WebServerManager::handlePostOutput(AsyncWebServerRequest* request, JsonVariant& json) {
  RouterConfig staged = pending_ ? pending_config_ : cfg_.config();
  if (!outputFromJson(json.as<JsonObjectConst>(), staged)) {
    sendError(request, "invalid output payload");
    return;
  }
  pending_config_ = staged;
  pending_ = true;
  sendOk(request, false);
}

void WebServerManager::handlePostPwm(AsyncWebServerRequest* request, JsonVariant& json) {
  RouterConfig staged = pending_ ? pending_config_ : cfg_.config();
  if (!pwmFromJson(json.as<JsonObjectConst>(), staged)) {
    sendError(request, "invalid pwm payload");
    return;
  }
  pending_config_ = staged;
  pending_ = true;
  sendOk(request, false);
}

void WebServerManager::handlePostVoltage(AsyncWebServerRequest* request, JsonVariant& json) {
  RouterConfig staged = pending_ ? pending_config_ : cfg_.config();
  if (!voltageFromJson(json.as<JsonObjectConst>(), staged)) {
    sendError(request, "invalid voltage payload");
    return;
  }
  pending_config_ = staged;
  pending_ = true;
  sendOk(request, false);
}

void WebServerManager::handlePostNetwork(AsyncWebServerRequest* request, JsonVariant& json) {
  RouterConfig staged = pending_ ? pending_config_ : cfg_.config();
  if (!networkFromJson(json.as<JsonObjectConst>(), staged)) {
    sendError(request, "invalid network payload");
    return;
  }
  pending_config_ = staged;
  pending_ = true;
  sendOk(request, true);
}

void WebServerManager::handlePostSystem(AsyncWebServerRequest* request, JsonVariant& json) {
  RouterConfig staged = pending_ ? pending_config_ : cfg_.config();
  if (!systemFromJson(json.as<JsonObjectConst>(), staged)) {
    sendError(request, "invalid system payload");
    return;
  }
  pending_config_ = staged;
  pending_ = true;
  sendOk(request, false);
}

void WebServerManager::handlePostVoltageCalibrate(AsyncWebServerRequest* request, JsonVariant& json) {
  JsonObjectConst o = json.as<JsonObjectConst>();
  if (!o["actual"].is<float>()) {
    sendError(request, "missing actual voltage");
    return;
  }
  float actual = o["actual"].as<float>();
  float factor = volt_.calibrate(actual);

  RouterConfig staged = pending_ ? pending_config_ : cfg_.config();
  staged.voltage.calibration_factor = factor;
  pending_config_ = staged;
  pending_ = true;

  JsonDocument doc;
  doc["calibration_factor"] = factor;
  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

void WebServerManager::handleGetLogs(AsyncWebServerRequest* request) {
  static const size_t kLogDumpCap = 4096;
  char* buf = new char[kLogDumpCap];
  size_t n = Logger::instance().dump(buf, kLogDumpCap);
  (void)n;

  JsonDocument doc;
  JsonArray lines = doc["lines"].to<JsonArray>();
  char* line_start = buf;
  for (size_t i = 0; i < n; i++) {
    if (buf[i] == '\n') {
      buf[i] = '\0';
      lines.add(line_start);
      line_start = buf + i + 1;
    }
  }
  if (line_start < buf + n) lines.add(line_start);

  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
  delete[] buf;
}

void WebServerManager::handlePostRestart(AsyncWebServerRequest* request) {
  sendOk(request, true);
  ESP.restart();
}

void WebServerManager::handlePostFactoryReset(AsyncWebServerRequest* request) {
  cfg_.factoryReset();
  sendOk(request, true);
}

void WebServerManager::handleGetOtaStatus(AsyncWebServerRequest* request) {
  JsonDocument doc;
  doc["state"] = (int)ota_.state();
  doc["progress_percent"] = ota_.progressPercent();
  doc["last_error"] = ota_.lastError();
  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

void WebServerManager::registerRoutes() {
  server_.on("/api/status", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetStatus(r); });
  server_.on("/api/config", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetConfig(r); });
  server_.on("/api/config/receivers", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetSection(r, "receivers"); });
  server_.on("/api/config/output", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetSection(r, "output"); });
  server_.on("/api/config/pwm", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetSection(r, "pwm"); });
  server_.on("/api/config/voltage", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetSection(r, "voltage"); });
  server_.on("/api/config/network", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetSection(r, "network"); });
  server_.on("/api/config/system", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetSection(r, "system"); });
  server_.on("/api/logs", HTTP_GET, [this](AsyncWebServerRequest* r) { handleGetLogs(r); });
  server_.on("/api/system/restart", HTTP_POST,
             [this](AsyncWebServerRequest* r) { handlePostRestart(r); });
  server_.on("/api/system/factory-reset", HTTP_POST,
             [this](AsyncWebServerRequest* r) { handlePostFactoryReset(r); });
  server_.on("/api/ota/status", HTTP_GET,
             [this](AsyncWebServerRequest* r) { handleGetOtaStatus(r); });

  AsyncCallbackJsonWebHandler* receivers_handler = new AsyncCallbackJsonWebHandler(
      "/api/config/receivers",
      [this](AsyncWebServerRequest* r, JsonVariant& json) { handlePostReceivers(r, json); });
  server_.addHandler(receivers_handler);

  AsyncCallbackJsonWebHandler* output_handler = new AsyncCallbackJsonWebHandler(
      "/api/config/output",
      [this](AsyncWebServerRequest* r, JsonVariant& json) { handlePostOutput(r, json); });
  server_.addHandler(output_handler);

  AsyncCallbackJsonWebHandler* pwm_handler = new AsyncCallbackJsonWebHandler(
      "/api/config/pwm",
      [this](AsyncWebServerRequest* r, JsonVariant& json) { handlePostPwm(r, json); });
  server_.addHandler(pwm_handler);

  AsyncCallbackJsonWebHandler* voltage_handler = new AsyncCallbackJsonWebHandler(
      "/api/config/voltage",
      [this](AsyncWebServerRequest* r, JsonVariant& json) { handlePostVoltage(r, json); });
  server_.addHandler(voltage_handler);

  AsyncCallbackJsonWebHandler* network_handler = new AsyncCallbackJsonWebHandler(
      "/api/config/network",
      [this](AsyncWebServerRequest* r, JsonVariant& json) { handlePostNetwork(r, json); });
  server_.addHandler(network_handler);

  AsyncCallbackJsonWebHandler* system_handler = new AsyncCallbackJsonWebHandler(
      "/api/config/system",
      [this](AsyncWebServerRequest* r, JsonVariant& json) { handlePostSystem(r, json); });
  server_.addHandler(system_handler);

  AsyncCallbackJsonWebHandler* calibrate_handler = new AsyncCallbackJsonWebHandler(
      "/api/voltage/calibrate",
      [this](AsyncWebServerRequest* r, JsonVariant& json) { handlePostVoltageCalibrate(r, json); });
  server_.addHandler(calibrate_handler);

  // OTA upload is handled by a dedicated multipart body handler wired in Task 22
  // alongside OtaManager::handleChunk(); left unregistered here to keep this task's
  // scope to config_json + the routes above.

  server_.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
}

#else  // !defined(ARDUINO)

bool WebServerManager::begin(uint16_t port) {
  (void)port;
  return true;   // no-op outside Arduino/ESP32; native tests exercise config_json only
}

void WebServerManager::update(uint32_t now_ms) { (void)now_ms; }

#endif
