#include "border_router.h"
#include "nat_table.h"
#include "border_router_protocol.h"
#include "esphome/core/log.h"
#include "esphome/components/network/ip_address.h"
#include <vector>

namespace esphome {
namespace border_router {

static const char *const TAG = "border_router";

void BorderRouter::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Border Router...");
  if (this->meshmesh_) {
    this->meshmesh_->getNetwork()->addHandleFrameCb(
        std::bind(&BorderRouter::handle_mesh_packet, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
}

void BorderRouter::loop() {
  // Periodically clean up timed-out NAT entries
  uint32_t now = millis();
  if (now - this->last_cleanup_time_ > 5000) { // Every 5 seconds
    this->last_cleanup_time_ = now;

    NATEntry *entries = this->nat_table_.get_entries();
    for (int i = 0; i < MAX_NAT_ENTRIES; i++) {
      NATEntry *entry = &entries[i];
      if (entry->active && (now - entry->last_activity > NAT_ENTRY_TIMEOUT_MS)) {
        ESP_LOGD(TAG, "NAT entry for %X:%04X timed out", entry->mesh_node_id, entry->session_id);
        if (entry->protocol == NAT_PROTOCOL_TCP && entry->tcp_client) {
          entry->tcp_client->close(); // onDisconnect will handle the rest
        } else if (entry->protocol == NAT_PROTOCOL_UDP && entry->udp_socket) {
          uint16_t port = entry->udp_socket->localPort();
          this->nat_table_.remove_entry(entry); // This deletes the socket
          this->free_udp_port(port);
        }
      }
    }
  }
}

void BorderRouter::send_error_response(uint32_t to, uint16_t session_id, uint8_t error_code) {
  ESP_LOGW(TAG, "Sending error %d to %X:%04X", error_code, to, session_id);
  std::vector<uint8_t> buffer(4);
  buffer[0] = ROUTER_CMD_ERROR;
  buffer[1] = (session_id >> 8) & 0xFF;
  buffer[2] = session_id & 0xFF;
  buffer[3] = error_code;
  this->meshmesh_->send_unicast(to, buffer.data(), buffer.size());
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

void BorderRouter::establish_tcp_connection(uint32_t from, uint16_t session_id, const esphome::network::IPAddress &ip_addr, uint16_t port) {
  if (this->nat_table_.find_entry(from, session_id) != nullptr) {
    ESP_LOGW(TAG, "Session %X:%04X already exists.", from, session_id);
    this->send_error_response(from, session_id, ERR_SESSION_EXISTS);
    return;
  }

  NATEntry *entry = this->nat_table_.create_entry(from, session_id, NAT_PROTOCOL_TCP);
  if (entry == nullptr) {
    ESP_LOGE(TAG, "NAT table full. Cannot create entry for %X:%04X.", from, session_id);
    this->send_error_response(from, session_id, ERR_NAT_TABLE_FULL);
    return;
  }

  AsyncClient *client = new AsyncClient();
  entry->tcp_client = client;

  client->onData([this, entry](void *data, size_t len) { this->on_tcp_data(entry, data, len); });
  client->onDisconnect([this, entry](void* arg) { this->on_tcp_disconnect(entry); });
  client->onError([this, entry](void* arg, int8_t error) { this->on_tcp_disconnect(entry); });
  client->onTimeout([this, entry](void* arg, uint32_t time) { this->on_tcp_disconnect(entry); });

  client->onConnect([this, entry](void* arg, AsyncClient* c) {
    ESP_LOGD(TAG, "TCP connection established for %X:%04X", entry->mesh_node_id, entry->session_id);
    std::vector<uint8_t> buffer(3);
    buffer[0] = ROUTER_CMD_TCP_CONNECTED;
    buffer[1] = (entry->session_id >> 8) & 0xFF;
    buffer[2] = entry->session_id & 0xFF;
    this->meshmesh_->send_unicast(entry->mesh_node_id, buffer.data(), buffer.size());
  });

  ESP_LOGD(TAG, "Connecting to %s:%d", ip_addr.str().c_str(), port);
  if (!client->connect(ip_addr, port)) {
    ESP_LOGE(TAG, "Failed to initiate TCP connection.");
    this->nat_table_.remove_entry(entry);
  }
}

void BorderRouter::establish_udp_session(uint32_t from, uint16_t session_id, const esphome::network::IPAddress &ip_addr, uint16_t port, uint8_t *udp_payload, uint16_t udp_payload_len) {
  NATEntry *entry = this->nat_table_.find_entry(from, session_id);
  if (entry == nullptr) {
    // New session
    ESP_LOGD(TAG, "Creating new UDP session for %X:%04X", from, session_id);
    uint16_t local_port = this->allocate_udp_port();
    if (local_port == 0) {
      this->send_error_response(from, session_id, ERR_NAT_TABLE_FULL);
      return;
    }

    entry = this->nat_table_.create_entry(from, session_id, NAT_PROTOCOL_UDP);
    if (entry == nullptr) {
      this->free_udp_port(local_port);
      this->send_error_response(from, session_id, ERR_NAT_TABLE_FULL);
      return;
    }

    AsyncUDP *udp_socket = new AsyncUDP();
    entry->udp_socket = udp_socket;

    udp_socket->onPacket([this, entry](AsyncUDPPacket &packet) {
      std::vector<uint8_t> buffer(packet.length() + 3);
      buffer[0] = ROUTER_CMD_UDP_DATA;
      buffer[1] = (entry->session_id >> 8) & 0xFF;
      buffer[2] = entry->session_id & 0xFF;
      memcpy(&buffer[3], packet.data(), packet.length());
      this->meshmesh_->send_unicast(entry->mesh_node_id, buffer.data(), buffer.size());
      entry->last_activity = millis();
    });

    if (!udp_socket->listen(local_port)) {
      ESP_LOGE(TAG, "Failed to listen on UDP port %d", local_port);
      this->nat_table_.remove_entry(entry);
      this->free_udp_port(local_port);
      return;
    }

    udp_socket->sendTo(udp_payload, udp_payload_len, ip_addr, port);
    ESP_LOGD(TAG, "Sent initial UDP packet for new session %X:%04X", from, session_id);

  } else {
    // Existing session
    if (entry->protocol != NAT_PROTOCOL_UDP) {
      ESP_LOGW(TAG, "Received UDP_SEND for a non-UDP session from %X:%04X", from, session_id);
      return;
    }
    if (entry->udp_socket) {
      entry->udp_socket->sendTo(udp_payload, udp_payload_len, ip_addr, port);
      ESP_LOGD(TAG, "Sent UDP packet for existing session %X:%04X", from, session_id);
    }
  }

  if (entry) {
    entry->last_activity = millis();
  }
}

void BorderRouter::handle_tcp_connect(uint32_t from, uint16_t session_id, uint8_t *payload, uint16_t payload_len) {
  if (payload_len < 1) {
    ESP_LOGW(TAG, "TCP_CONNECT packet too short for ip_type");
    return;
  }
  uint8_t ip_type = payload[0];
  if (ip_type == 0x04) {
    if (payload_len < 1 + 4 + 2) {
      ESP_LOGW(TAG, "TCP_CONNECT (IPv4) packet too short");
      return;
    }
    esphome::network::IPAddress ip_addr(&payload[1], true);
    uint16_t port = (payload[5] << 8) | payload[6];
    this->establish_tcp_connection(from, session_id, ip_addr, port);
  } else if (ip_type == 0x06) {
    if (payload_len < 1 + 16 + 2) {
      ESP_LOGW(TAG, "TCP_CONNECT (IPv6) packet too short");
      return;
    }
    esphome::network::IPAddress ip_addr(&payload[1], false);
    uint16_t port = (payload[17] << 8) | payload[18];
    this->establish_tcp_connection(from, session_id, ip_addr, port);
  } else {
    ESP_LOGW(TAG, "Invalid ip_type in TCP_CONNECT: %02X", ip_type);
  }
}

void BorderRouter::handle_tcp_data(uint32_t from, uint16_t session_id, uint8_t *payload, uint16_t payload_len) {
  NATEntry *entry = this->nat_table_.find_entry(from, session_id);
  if (entry == nullptr || entry->protocol != NAT_PROTOCOL_TCP) {
    ESP_LOGW(TAG, "Invalid session for TCP_DATA from %X:%04X", from, session_id);
    return;
  }

  AsyncClient *client = entry->tcp_client;
  if (client && client->canSend()) {
    client->add((const char*)payload, payload_len);
    entry->last_activity = millis();
  } else {
    ESP_LOGW(TAG, "TCP client not ready to send for %X:%04X", from, session_id);
  }
}

void BorderRouter::handle_tcp_close(uint32_t from, uint16_t session_id, uint8_t *payload, uint16_t payload_len) {
  NATEntry *entry = this->nat_table_.find_entry(from, session_id);
  if (entry == nullptr || entry->protocol != NAT_PROTOCOL_TCP) {
    ESP_LOGW(TAG, "Invalid session for TCP_CLOSE from %X:%04X", from, session_id);
    return;
  }

  if (entry->tcp_client) {
    entry->tcp_client->close();
  }
}

void BorderRouter::handle_udp_send(uint32_t from, uint16_t session_id, uint8_t *payload, uint16_t payload_len) {
  if (payload_len < 1) {
    ESP_LOGW(TAG, "UDP_SEND packet too short for ip_type");
    return;
  }
  uint8_t ip_type = payload[0];
  if (ip_type == 0x04) {
    if (payload_len < 1 + 4 + 2) {
      ESP_LOGW(TAG, "UDP_SEND (IPv4) packet too short for header");
      return;
    }
    esphome::network::IPAddress ip_addr(&payload[1], true);
    uint16_t port = (payload[5] << 8) | payload[6];
    uint8_t *udp_payload = &payload[7];
    uint16_t udp_payload_len = payload_len - 7;
    this->establish_udp_session(from, session_id, ip_addr, port, udp_payload, udp_payload_len);
  } else if (ip_type == 0x06) {
    if (payload_len < 1 + 16 + 2) {
      ESP_LOGW(TAG, "UDP_SEND (IPv6) packet too short for header");
      return;
    }
    esphome::network::IPAddress ip_addr(&payload[1], false);
    uint16_t port = (payload[17] << 8) | payload[18];
    uint8_t *udp_payload = &payload[19];
    uint16_t udp_payload_len = payload_len - 19;
    this->establish_udp_session(from, session_id, ip_addr, port, udp_payload, udp_payload_len);
  } else {
    ESP_LOGW(TAG, "Invalid ip_type in UDP_SEND: %02X", ip_type);
  }
}

int8_t BorderRouter::handle_mesh_packet(uint8_t *buf, uint16_t len, uint32_t from) {
  if (len < 3) {
    return 0;
  }

  uint8_t command = buf[0];
  uint16_t session_id = (buf[1] << 8) | buf[2];
  uint8_t *payload = &buf[3];
  uint16_t payload_len = len - 3;

  ESP_LOGD(TAG, "Received mesh packet from %X, command: %02X, session: %04X", from, command, session_id);

  switch (command) {
    case CMD_TCP_CONNECT:
      this->handle_tcp_connect(from, session_id, payload, payload_len);
      break;
    case CMD_TCP_DATA:
      this->handle_tcp_data(from, session_id, payload, payload_len);
      break;
    case CMD_TCP_CLOSE:
      this->handle_tcp_close(from, session_id, payload, payload_len);
      break;
    case CMD_UDP_SEND:
      this->handle_udp_send(from, session_id, payload, payload_len);
      break;
    default:
      ESP_LOGW(TAG, "Unknown command: %02X", command);
      break;
  }

  return 0;
}

void BorderRouter::on_tcp_data(NATEntry *entry, void *data, size_t len) {
  std::vector<uint8_t> buffer(len + 3);
  buffer[0] = ROUTER_CMD_TCP_DATA;
  buffer[1] = (entry->session_id >> 8) & 0xFF;
  buffer[2] = entry->session_id & 0xFF;
  memcpy(&buffer[3], data, len);
  this->meshmesh_->send_unicast(entry->mesh_node_id, buffer.data(), buffer.size());
}

void BorderRouter::on_tcp_disconnect(NATEntry *entry) {
  std::vector<uint8_t> buffer(3);
  buffer[0] = ROUTER_CMD_TCP_DISCONNECTED;
  buffer[1] = (entry->session_id >> 8) & 0xFF;
  buffer[2] = entry->session_id & 0xFF;
  this->meshmesh_->send_unicast(entry->mesh_node_id, buffer.data(), buffer.size());
  this->nat_table_.remove_entry(entry);
}

const uint16_t UDP_PORT_START = 49152;
const uint16_t UDP_PORT_END = 65535;

uint16_t BorderRouter::allocate_udp_port() {
  for (uint16_t port = UDP_PORT_START; port < UDP_PORT_END; port++) {
    bool in_use = false;
    for (uint16_t used_port : this->used_udp_ports_) {
      if (port == used_port) {
        in_use = true;
        break;
      }
    }
    if (!in_use) {
      this->used_udp_ports_.push_back(port);
      return port;
    }
  }
  return 0;
}

void BorderRouter::free_udp_port(uint16_t port) {
  for (auto it = this->used_udp_ports_.begin(); it != this->used_udp_ports_.end(); ++it) {
    if (*it == port) {
      this->used_udp_ports_.erase(it);
      return;
    }
  }
}

}  // namespace border_router
}  // namespace esphome
