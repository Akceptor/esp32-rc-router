#include "app/app.h"

void setup() {
  App::instance().setup();
}

void loop() {
  App::instance().loop();
}
