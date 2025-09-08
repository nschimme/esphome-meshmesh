#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace border_router {

class BorderRouter : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
};

}  // namespace border_router
}  // namespace esphome
