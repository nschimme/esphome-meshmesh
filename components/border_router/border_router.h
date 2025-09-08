#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ethernet/ethernet_component.h"
#include "esphome/components/meshmesh/meshmesh.h"
#include "AsyncUDP.h"
#include "nat_table.h"
#include <vector>

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

  // TCP connection handlers
  void on_tcp_data(NATEntry *entry, void *data, size_t len);
  void on_tcp_disconnect(NATEntry *entry);

  // UDP port management
  uint16_t allocate_udp_port();
  void free_udp_port(uint16_t port);

  meshmesh::MeshmeshComponent *meshmesh_;
  ethernet::EthernetComponent *ethernet_;
  AsyncUDP udp_;
  NATTable nat_table_;
  uint32_t last_cleanup_time_{0};
  std::vector<uint16_t> used_udp_ports_;
};

}  // namespace border_router
}  // namespace esphome
