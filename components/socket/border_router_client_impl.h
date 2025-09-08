#pragma once

#include "socket.h"
#include <queue>

namespace esphome {
namespace socket {

class BorderRouterClientImpl : public Socket {
 public:
  BorderRouterClientImpl();
  ~BorderRouterClientImpl() override;

  int connect(const struct sockaddr *name, socklen_t addrlen) override;
  int close() override;
  ssize_t read(void *buf, size_t len) override;
  ssize_t write(const void *buf, size_t len) override;

  // Other overridden methods will be stubs for now
  std::unique_ptr<Socket> accept(struct sockaddr *addr, socklen_t *addrlen) override;
  int bind(const struct sockaddr *name, socklen_t addrlen) override;
  int listen(int backlog) override;
  int shutdown(int how) override;
  int getpeername(struct sockaddr *name, socklen_t *addrlen) override;
  std::string getpeername() override;
  int getsockname(struct sockaddr *name, socklen_t *addrlen) override;
  std::string getsockname() override;
  int getsockopt(int level, int optname, void *optval, socklen_t *optlen) override;
  int setsockopt(int level, int optname, const void *optval, socklen_t optlen) override;
  ssize_t readv(const struct iovec *iov, int iovcnt) override;
  ssize_t writev(const struct iovec *iov, int iovcnt) override;
  ssize_t sendto(const void *buf, size_t len, int flags, const struct sockaddr *to, socklen_t tolen) override;
  int setblocking(bool blocking) override;

  // Public methods for the global handler to use
  void on_connected();
  void on_error(uint8_t error_code);
  void on_data(const uint8_t *data, size_t len);

  uint16_t get_session_id() const { return session_id_; }

 private:
  uint16_t session_id_;
  bool is_connected_{false};
  bool connect_result_ready_{false};
  uint8_t connect_error_{0};
  std::queue<uint8_t> rx_buffer_;
};

}  // namespace socket
}  // namespace esphome
