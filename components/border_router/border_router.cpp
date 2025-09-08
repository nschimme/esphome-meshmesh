#include "border_router.h"
#include "esphome/core/log.h"

namespace esphome {
namespace border_router {

static const char *const TAG = "border_router";

void BorderRouter::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Border Router...");
}

void BorderRouter::loop() {
  // TODO: Implement the packet forwarding logic here
}

void BorderRouter::dump_config() {
  ESP_LOGCONFIG(TAG, "Border Router:");
}

}  // namespace border_router
}  // namespace esphome
