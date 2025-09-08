#include "border_router.h"
#include "nat_table.h"
#include "esphome/core/log.h"
#include "esphome/components/network/ip_address.h"
#include <vector>

namespace esphome {
namespace border_router {

// Mesh-to-Router Protocol Commands
const uint8_t CMD_TCP_CONNECT = 0x01;
const uint8_t CMD_TCP_DATA = 0x02;
const uint8_t CMD_TCP_CLOSE = 0x03;
const uint8_t CMD_UDP_SEND = 0x11;

// Router-to-Mesh Protocol Commands
const uint8_t ROUTER_CMD_TCP_CONNECTED = 0x81;
const uint8_t ROUTER_CMD_TCP_DATA = 0x82;
const uint8_t ROUTER_CMD_TCP_DISCONNECTED = 0x83;

static const char *const TAG = "border_router";

void BorderRouter::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Border Router...");
  if (this->meshmesh_) {
    this->meshmesh_->getNetwork()->addHandleFrameCb(
        std::bind(&BorderRouter::handle_mesh_packet, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
  if (this->ethernet_ && this->ethernet_->is_connected()) {
    if (this->udp_.listen(12345)) {
      ESP_LOGD(TAG, "UDP listener started on port 12345");
      this->udp_.onPacket([this](AsyncUDPPacket &packet) { this->handle_udp_packet(packet); });
    }
  }
}

void BorderRouter::loop() {
  uint32_t now = millis();
  if (now - this->last_cleanup_time_ > 5000) { // Every 5 seconds
    this->last_cleanup_time_ = now;
    this->nat_table_.cleanup();
  }
}

void BorderRouter::dump_config() {
  ESP_LOGCONFIG(TAG, "Border Router:");
  if (this->meshmesh_) {
    ESP_LOGCONFIG(TAG, "  MeshMesh component linked.");
  }
  if (this->ethernet_) {
    ESP_LOGCONFIG(TAG, "  Ethernet component linked.");
  }
}

int8_t BorderRouter::handle_mesh_packet(uint8_t *buf, uint16_t len, uint32_t from) {
  if (len < 3) {
    // Not a valid packet for our protocol
    return 0;
  }

  uint8_t command = buf[0];
  uint16_t session_id = (buf[1] << 8) | buf[2];
  uint8_t *payload = &buf[3];
  uint16_t payload_len = len - 3;

  ESP_LOGD(TAG, "Received mesh packet from %X, command: %02X, session: %04X", from, command, session_id);

  switch (command) {
    case CMD_TCP_CONNECT: {
      if (payload_len < 1) {
        ESP_LOGW(TAG, "TCP_CONNECT packet too short for ip_type");
        return 0;
      }
      uint8_t ip_type = payload[0];
      if (ip_type == 0x04) {
        if (payload_len < 1 + 4 + 2) {
          ESP_LOGW(TAG, "TCP_CONNECT (IPv4) packet too short");
          return 0;
        }
        esphome::network::IPAddress ip_addr(&payload[1], true);
        uint16_t port = (payload[5] << 8) | payload[6];
        ESP_LOGD(TAG, "TCP_CONNECT request for %s:%d", ip_addr.str().c_str(), port);

        NATEntry *entry = this->nat_table_.create_entry(from, session_id, NAT_PROTOCOL_TCP);
        if (entry == nullptr) {
          // Could not create entry (table full, or already exists)
          // TODO: Send an error message back to the mesh node.
          return 0;
        }

        AsyncClient *client = new AsyncClient();
        entry->socket.tcp_client = client;

        client->onData([this, entry](void *data, size_t len) { this->on_tcp_data(entry, data, len); });
        client->onDisconnect([this, entry](void* arg) { this->on_tcp_disconnect(entry); });
        client->onError([this, entry](void* arg, int8_t error) { this->on_tcp_disconnect(entry); });
        client->onTimeout([this, entry](void* arg, uint32_t time) { this->on_tcp_disconnect(entry); });

        client->onConnect([this, entry](void* arg, AsyncClient* client) {
          ESP_LOGD(TAG, "TCP connection established for %X:%04X", entry->mesh_node_id, entry->session_id);

          // Notify the mesh node
          std::vector<uint8_t> buffer(3);
          buffer[0] = ROUTER_CMD_TCP_CONNECTED;
          buffer[1] = (entry->session_id >> 8) & 0xFF;
          buffer[2] = entry->session_id & 0xFF;

          this->meshmesh_->getNetwork()->sendUnicast(entry->mesh_node_id, buffer.data(), buffer.size());
        });

        ESP_LOGD(TAG, "Connecting to %s:%d", ip_addr.str().c_str(), port);
        if (!client->connect(ip_addr, port)) {
          ESP_LOGE(TAG, "Failed to initiate TCP connection.");
          this->nat_table_.remove_entry(entry);
        }
      } else if (ip_type == 0x06) {
        // TODO: Handle IPv6
        ESP_LOGW(TAG, "IPv6 not yet supported");
      } else {
        ESP_LOGW(TAG, "Invalid ip_type in TCP_CONNECT: %02X", ip_type);
      }
      break;
    }
    case CMD_TCP_DATA: {
      NATEntry *entry = this->nat_table_.find_entry(from, session_id);
      if (entry == nullptr) {
        ESP_LOGW(TAG, "No NAT entry found for TCP_DATA from %X:%04X", from, session_id);
        return 0;
      }
      if (entry->protocol != NAT_PROTOCOL_TCP) {
        ESP_LOGW(TAG, "Received TCP_DATA for a non-TCP session from %X:%04X", from, session_id);
        return 0;
      }

      AsyncClient *client = entry->socket.tcp_client;
      if (client && client->canSend()) {
        client->add((const char*)payload, payload_len);
        entry->last_activity = millis();
        ESP_LOGD(TAG, "Forwarded %d bytes from mesh to TCP socket for %X:%04X", payload_len, from, session_id);
      } else {
        ESP_LOGW(TAG, "TCP client not ready to send for %X:%04X", from, session_id);
      }
      break;
    }
    case CMD_TCP_CLOSE:
    case CMD_UDP_SEND:
      ESP_LOGD(TAG, "Command %02X not yet implemented", command);
      break;
    default:
      ESP_LOGW(TAG, "Unknown command: %02X", command);
      break;
  }

  return 0;
}

void BorderRouter::handle_udp_packet(AsyncUDPPacket &packet) {
  ESP_LOGD(TAG, "Received UDP packet from %s, len: %d", packet.remoteIP().toString().c_str(), packet.length());
  if (this->meshmesh_) {
    this->meshmesh_->getNetwork()->politeBroadcast(packet.data(), packet.length());
    ESP_LOGD(TAG, "Forwarded UDP packet to mesh broadcast");
  }
}

void BorderRouter::on_tcp_data(NATEntry *entry, void *data, size_t len) {
  ESP_LOGD(TAG, "TCP data received for %X:%04X, len: %d", entry->mesh_node_id, entry->session_id, len);

  // Construct the router-to-mesh packet: [command] [session_id] [payload]
  size_t packet_len = 1 + 2 + len;
  std::vector<uint8_t> buffer(packet_len);
  buffer[0] = ROUTER_CMD_TCP_DATA;
  buffer[1] = (entry->session_id >> 8) & 0xFF;
  buffer[2] = entry->session_id & 0xFF;
  memcpy(&buffer[3], data, len);

  // Send the packet to the mesh node
  // NOTE: Assuming the existence of a sendUnicast(to, data, len) method.
  ESP_LOGD(TAG, "Forwarding TCP data to mesh node %X", entry->mesh_node_id);
  this->meshmesh_->getNetwork()->sendUnicast(entry->mesh_node_id, buffer.data(), buffer.size());
}

void BorderRouter::on_tcp_disconnect(NATEntry *entry) {
  ESP_LOGD(TAG, "TCP connection closed for %X:%04X", entry->mesh_node_id, entry->session_id);

  // Notify the mesh node
  std::vector<uint8_t> buffer(3);
  buffer[0] = ROUTER_CMD_TCP_DISCONNECTED;
  buffer[1] = (entry->session_id >> 8) & 0xFF;
  buffer[2] = entry->session_id & 0xFF;

  this->meshmesh_->getNetwork()->sendUnicast(entry->mesh_node_id, buffer.data(), buffer.size());

  this->nat_table_.remove_entry(entry);
}

}  // namespace border_router
}  // namespace esphome
