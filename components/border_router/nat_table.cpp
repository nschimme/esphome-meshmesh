#include "nat_table.h"
#include "esphome/core/log.h"

namespace esphome {
namespace border_router {

static const char *const TAG = "nat_table";

NATEntry *NATTable::find_entry(uint32_t mesh_node_id, uint16_t session_id) {
  for (int i = 0; i < MAX_NAT_ENTRIES; i++) {
    if (this->entries_[i].active &&
        this->entries_[i].mesh_node_id == mesh_node_id &&
        this->entries_[i].session_id == session_id) {
      return &this->entries_[i];
    }
  }
  return nullptr;
}

NATEntry *NATTable::create_entry(uint32_t mesh_node_id, uint16_t session_id, NATProtocol protocol) {
  if (this->find_entry(mesh_node_id, session_id) != nullptr) {
    ESP_LOGW(TAG, "Entry for %X:%04X already exists", mesh_node_id, session_id);
    return nullptr;
  }

  for (int i = 0; i < MAX_NAT_ENTRIES; i++) {
    if (!this->entries_[i].active) {
      NATEntry *entry = &this->entries_[i];
      entry->active = true;
      entry->mesh_node_id = mesh_node_id;
      entry->session_id = session_id;
      entry->protocol = protocol;
      entry->last_activity = millis();
      ESP_LOGD(TAG, "Created NAT entry for %X:%04X", mesh_node_id, session_id);
      return entry;
    }
  }

  ESP_LOGE(TAG, "NAT table is full!");
  return nullptr;
}

void NATTable::remove_entry(NATEntry *entry) {
  if (entry && entry->active) { // Check for active to prevent double-free
    ESP_LOGD(TAG, "Removing NAT entry for %X:%04X", entry->mesh_node_id, entry->session_id);
    if (entry->protocol == NAT_PROTOCOL_TCP && entry->tcp_client) {
      delete entry->tcp_client;
      entry->tcp_client = nullptr;
    } else if (entry->protocol == NAT_PROTOCOL_UDP && entry->udp_socket) {
      delete entry->udp_socket;
      entry->udp_socket = nullptr;
    }
    entry->active = false;
  }
}

}  // namespace border_router
}  // namespace esphome
