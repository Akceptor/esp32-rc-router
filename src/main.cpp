#include <stdint.h>

#include "logging/logger.h"

// NOTE: This is a placeholder entry point used only for Task 1 scaffolding
// and remains in place through Task 20. It is replaced in Task 21 by a thin
// setup()/loop() shim that delegates to App::instance().setup()/loop()
// (see src/app/app.h), once the App class and all its collaborators exist.

#if defined(ARDUINO)
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Logger::instance().begin(LogLevel::INFO, true);
  LOG_I("MAIN", "RC Signal Router firmware booting (scaffold stage)");
}

void loop() {
  // Intentionally empty until Task 21 wires up App::loop().
}
#endif
