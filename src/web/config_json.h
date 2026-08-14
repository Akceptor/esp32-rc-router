#pragma once
#include <ArduinoJson.h>
#include "config/config_types.h"
#include "web/web_server_types.h"

// Pure-logic JSON <-> RouterConfig translation. ArduinoJson-only (no Arduino core
// dependency), so this pair of files compiles and is unit-tested in the native
// PlatformIO environment. web_server.cpp (Arduino/ESPAsyncWebServer-only) calls these
// functions from inside its HTTP handlers; it adds no serialization logic of its own.
//
// Convention: every *ToJson function only ADDS keys to `out` (never clears siblings),
// and every *FromJson function only OVERWRITES the fields it recognizes in `cfg`,
// leaving everything else (including cfg fields for unrelated sections, and cfg fields
// left unset by a partial/omitted JSON body) untouched. All *FromJson functions return
// false if any recognized field is out of the sane range checked here; they do not
// run the full configValidate() clamp — that happens once in WebServerManager after
// all sections have been merged into pending_config_.

void receiversToJson(const RouterConfig& cfg, JsonObject out);
bool receiversFromJson(JsonObjectConst in, RouterConfig& cfg);

void outputToJson(const RouterConfig& cfg, JsonObject out);
bool outputFromJson(JsonObjectConst in, RouterConfig& cfg);

void pwmToJson(const RouterConfig& cfg, JsonObject out);
bool pwmFromJson(JsonObjectConst in, RouterConfig& cfg);

void voltageToJson(const RouterConfig& cfg, JsonObject out);
bool voltageFromJson(JsonObjectConst in, RouterConfig& cfg);

void networkToJson(const RouterConfig& cfg, JsonObject out);
bool networkFromJson(JsonObjectConst in, RouterConfig& cfg);

void systemToJson(const RouterConfig& cfg, JsonObject out);
bool systemFromJson(JsonObjectConst in, RouterConfig& cfg);

void statusToJson(const StatusSnapshot& s, JsonObject out);

// Shared IPv4 <-> "a.b.c.d" helpers (network_config uses these; exposed for reuse and
// for direct testing).
uint32_t ipStringToU32(const char* s, bool& ok);
void ipU32ToString(uint32_t ip, char* out, size_t out_cap);
