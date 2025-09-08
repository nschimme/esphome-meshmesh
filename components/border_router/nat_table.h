#pragma once

#include "esphome/core/helpers.h"
#include "AsyncTCP.h"
#include "AsyncUDP.h"

namespace esphome {
namespace border_router {

const uint8_t MAX_NAT_ENTRIES = 16;
const uint32_t NAT_ENTRY_TIMEOUT_MS = 60000; // 1 minute

enum NATProtocol {
  NAT_PROTOCOL_TCP,
  NAT_PROTOCOL_UDP,
};

struct NATEntry {
  bool active{false};
  uint32_t mesh_node_id;
  uint16_t session_id;
  NATProtocol protocol;
  union {
    AsyncClient *tcp_client;
    AsyncUDP *udp_socket;
  } socket;
  uint32_t last_activity;

  // Helper to get a unique ID for the entry
  uint64_t get_id() const {
    return (uint64_t)this->mesh_node_id << 16 | this->session_id;
  }
};

class NATTable {
 public:
  NATEntry *find_entry(uint32_t mesh_node_id, uint16_t session_id);
  NATEntry *create_entry(uint32_t mesh_node_id, uint16_t session_id, NATProtocol protocol);
  void remove_entry(NATEntry *entry);
  void cleanup();

 protected:
  NATEntry entries_[MAX_NAT_ENTRIES];
};

}  // namespace border_router
}  // namespace esphome
