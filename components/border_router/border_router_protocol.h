#pragma once

#include <cstdint>

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
const uint8_t ROUTER_CMD_UDP_DATA = 0x91;
const uint8_t ROUTER_CMD_ERROR = 0xF1;

// Error codes for ROUTER_CMD_ERROR payload
const uint8_t ERR_NAT_TABLE_FULL = 0x01;
const uint8_t ERR_SESSION_EXISTS = 0x02;

}  // namespace border_router
}  // namespace esphome
