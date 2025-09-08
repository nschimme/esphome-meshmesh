#include "border_router.h"
#include "esphome/core/log.h"

namespace esphome {
namespace border_router {

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
  ESP_LOGD(TAG, "Received mesh packet from %X, len: %d", from, len);
  if (this->ethernet_ && this->ethernet_->is_connected()) {
    // Forward the packet over UDP
    this->udp_.broadcastTo(buf, len, 12345);
    ESP_LOGD(TAG, "Forwarded mesh packet to UDP broadcast");
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
