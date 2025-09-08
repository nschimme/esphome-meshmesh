#include "border_router_client_impl.h"
#include "esphome/core/log.h"
#include <map>
#include "esphome/components/meshmesh/meshmesh.h"
#include "esphome/components/border_router/border_router_protocol.h"

namespace esphome {
namespace socket {

static const char *const TAG = "socket.border_router";

static std::map<uint16_t, BorderRouterClientImpl*> g_active_sockets;

// Global handler for packets from the border router
int8_t border_router_client_packet_handler(uint8_t *buf, uint16_t len, uint32_t from) {
  if (len < 3) {
    return FRAME_NOT_HANDLED;
  }

  uint8_t command = buf[0];
  if (command < 0x80) { // Not a router command
    return FRAME_NOT_HANDLED;
  }

  uint16_t session_id = (buf[1] << 8) | buf[2];
  uint8_t *payload = &buf[3];
  uint16_t payload_len = len - 3;

  auto it = g_active_sockets.find(session_id);
  if (it == g_active_sockets.end()) {
    return FRAME_NOT_HANDLED;
  }
  BorderRouterClientImpl *socket = it->second;

  switch (command) {
    case esphome::border_router::ROUTER_CMD_TCP_CONNECTED:
      socket->on_connected();
      break;
    case esphome::border_router::ROUTER_CMD_TCP_DISCONNECTED:
      socket->on_error(ECONNRESET); // Or some other error
      break;
    case esphome::border_router::ROUTER_CMD_TCP_DATA:
      socket->on_data(payload, payload_len);
      break;
    case esphome::border_router::ROUTER_CMD_ERROR:
      socket->on_error(payload[0]);
      break;
    case esphome::border_router::ROUTER_CMD_UDP_DATA:
      // TODO: Handle UDP data
      break;
  }

  return 0; // Packet was handled
}

BorderRouterClientImpl::BorderRouterClientImpl() {
  // Generate a unique session ID and register this socket in the global map
  do {
    this->session_id_ = (uint16_t)rand();
  } while (this->session_id_ == 0 || g_active_sockets.count(this->session_id_)); // 0 is not a valid session id
  g_active_sockets[this->session_id_] = this;
  ESP_LOGD(TAG, "BorderRouterClientImpl created, session_id: %d", this->session_id_);

  // Register the global packet handler if this is the first socket created.
  // This is not thread-safe, but it's unlikely to be an issue in ESPHome.
  static bool handler_registered = false;
  if (!handler_registered) {
    meshmesh::global_meshmesh_component->getNetwork()->addHandleFrameCb(border_router_client_packet_handler);
    handler_registered = true;
    ESP_LOGD(TAG, "Global packet handler registered.");
  }
}

BorderRouterClientImpl::~BorderRouterClientImpl() {
  g_active_sockets.erase(this->session_id_);
  ESP_LOGD(TAG, "BorderRouterClientImpl destroyed, session_id: %d", this->session_id_);
}

int BorderRouterClientImpl::connect(const struct sockaddr *name, socklen_t addrlen) {
  if (name->sa_family != AF_INET) {
    errno = EAFNOSUPPORT;
    return -1;
  }
  const struct sockaddr_in *addr_in = reinterpret_cast<const struct sockaddr_in *>(name);

  std::vector<uint8_t> buffer;
  buffer.push_back(esphome::border_router::CMD_TCP_CONNECT);
  buffer.push_back((this->session_id_ >> 8) & 0xFF);
  buffer.push_back(this->session_id_ & 0xFF);
  buffer.push_back(0x04); // IPv4
  buffer.push_back((addr_in->sin_addr.s_addr >> 0) & 0xFF);
  buffer.push_back((addr_in->sin_addr.s_addr >> 8) & 0xFF);
  buffer.push_back((addr_in->sin_addr.s_addr >> 16) & 0xFF);
  buffer.push_back((addr_in->sin_addr.s_addr >> 24) & 0xFF);
  buffer.push_back((addr_in->sin_port >> 8) & 0xFF);
  buffer.push_back(addr_in->sin_port & 0xFF);

  // For now, assume the border router is the coordinator (address 0)
  // This needs a better mechanism in a real implementation.
  uint32_t border_router_address = 0;
  meshmesh::global_meshmesh_component->send_unicast(border_router_address, buffer.data(), buffer.size());

  this->connect_result_ready_ = false;
  uint32_t start_time = millis();
  while (!this->connect_result_ready_ && (millis() - start_time < 10000)) {
    // Busy wait with yield
    yield();
  }

  if (!this->connect_result_ready_) {
    errno = ETIMEDOUT;
    return -1;
  }

  if (this->connect_error_ != 0) {
    errno = ECONNREFUSED; // Generic error
    return -1;
  }

  return 0; // Success
}

int BorderRouterClientImpl::close() {
  if (!this->is_connected_) {
    return 0; // Already closed
  }

  std::vector<uint8_t> buffer(3);
  buffer[0] = esphome::border_router::CMD_TCP_CLOSE;
  buffer[1] = (this->session_id_ >> 8) & 0xFF;
  buffer[2] = this->session_id_ & 0xFF;

  uint32_t border_router_address = 0; // Assuming coordinator
  meshmesh::global_meshmesh_component->send_unicast(border_router_address, buffer.data(), buffer.size());

  this->is_connected_ = false;
  return 0;
}

ssize_t BorderRouterClientImpl::read(void *buf, size_t len) {
  if (!this->is_connected_) {
    errno = ENOTCONN;
    return -1;
  }

  if (this->rx_buffer_.empty()) {
    errno = EWOULDBLOCK;
    return -1;
  }

  size_t to_read = std::min(len, this->rx_buffer_.size());
  uint8_t *buf8 = reinterpret_cast<uint8_t *>(buf);
  for (size_t i = 0; i < to_read; i++) {
    *buf8++ = this->rx_buffer_.front();
    this->rx_buffer_.pop();
  }

  return to_read;
}

ssize_t BorderRouterClientImpl::write(const void *buf, size_t len) {
  if (!this->is_connected_) {
    errno = ENOTCONN;
    return -1;
  }

  std::vector<uint8_t> buffer(len + 3);
  buffer[0] = esphome::border_router::CMD_TCP_DATA;
  buffer[1] = (this->session_id_ >> 8) & 0xFF;
  buffer[2] = this->session_id_ & 0xFF;
  memcpy(&buffer[3], buf, len);

  uint32_t border_router_address = 0; // Assuming coordinator
  meshmesh::global_meshmesh_component->send_unicast(border_router_address, buffer.data(), buffer.size());

  return len;
}

// Stub implementations for other methods
std::unique_ptr<Socket> BorderRouterClientImpl::accept(struct sockaddr *addr, socklen_t *addrlen) { return nullptr; }
int BorderRouterClientImpl::bind(const struct sockaddr *name, socklen_t addrlen) { return -1; }
int BorderRouterClientImpl::listen(int backlog) { return -1; }
int BorderRouterClientImpl::shutdown(int how) { return 0; }
int BorderRouterClientImpl::getpeername(struct sockaddr *name, socklen_t *addrlen) { return 0; }
std::string BorderRouterClientImpl::getpeername() { return ""; }
int BorderRouterClientImpl::getsockname(struct sockaddr *name, socklen_t *addrlen) { return 0; }
std::string BorderRouterClientImpl::getsockname() { return ""; }
int BorderRouterClientImpl::getsockopt(int level, int optname, void *optval, socklen_t *optlen) { return 0; }
int BorderRouterClientImpl::setsockopt(int level, int optname, const void *optval, socklen_t optlen) { return 0; }
ssize_t BorderRouterClientImpl::readv(const struct iovec *iov, int iovcnt) { return 0; }
ssize_t BorderRouterClientImpl::writev(const struct iovec *iov, int iovcnt) { return 0; }
ssize_t BorderRouterClientImpl::sendto(const void *buf, size_t len, int flags, const struct sockaddr *to, socklen_t tolen) { return 0; }
int BorderRouterClientImpl::setblocking(bool blocking) { return 0; }

void BorderRouterClientImpl::on_connected() {
  this->is_connected_ = true;
  this->connect_error_ = 0;
  this->connect_result_ready_ = true;
}

void BorderRouterClientImpl::on_error(uint8_t error_code) {
  this->is_connected_ = false;
  this->connect_error_ = error_code;
  this->connect_result_ready_ = true;
}

void BorderRouterClientImpl::on_data(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    this->rx_buffer_.push(data[i]);
  }
}

}  // namespace socket
}  // namespace esphome
