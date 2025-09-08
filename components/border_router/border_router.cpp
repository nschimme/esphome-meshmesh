#include "border_router.h"
#include "nat_table.h"
#include "esphome/core/log.h"
#include "esphome/components/network/ip_address.h"

namespace esphome {
namespace border_router {

// Mesh-to-Router Protocol Commands
const uint8_t CMD_TCP_CONNECT = 0x01;
const uint8_t CMD_TCP_DATA = 0x02;
const uint8_t CMD_TCP_CLOSE = 0x03;
const uint8_t CMD_UDP_SEND = 0x11;

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
  // TODO: Implement the packet forwarding logic here
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
        // TODO: Create NAT entry and connect
      } else if (ip_type == 0x06) {
        // TODO: Handle IPv6
        ESP_LOGW(TAG, "IPv6 not yet supported");
      } else {
        ESP_LOGW(TAG, "Invalid ip_type in TCP_CONNECT: %02X", ip_type);
      }
      break;
    }
    case CMD_TCP_DATA:
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

}  // namespace border_router
}  // namespace esphome
