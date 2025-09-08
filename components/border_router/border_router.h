#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ethernet/ethernet_component.h"
#include "esphome/components/meshmesh/meshmesh.h"
#include "AsyncUDP.h"
#include "nat_table.h"

namespace esphome {
namespace border_router {

class BorderRouter : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_meshmesh(meshmesh::MeshmeshComponent *meshmesh) { this->meshmesh_ = meshmesh; }
  void set_ethernet(ethernet::EthernetComponent *ethernet) { this->ethernet_ = ethernet; }

 protected:
  int8_t handle_mesh_packet(uint8_t *buf, uint16_t len, uint32_t from);
  void handle_udp_packet(AsyncUDPPacket &packet);

  meshmesh::MeshmeshComponent *meshmesh_;
  ethernet::EthernetComponent *ethernet_;
  AsyncUDP udp_;
  NATTable nat_table_;
};

}  // namespace border_router
}  // namespace esphome
