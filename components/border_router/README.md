# Border Router Component

This component implements a border router for the `meshmesh` network, allowing it to communicate with standard IPv4 and IPv6 networks.

## Purpose

The `meshmesh` protocol is a custom L2 protocol that does not use IP for addressing. This makes it lightweight and efficient for communication between ESP nodes, but it also isolates the network from standard IP-based networks like WiFi and Ethernet.

This `border_router` component bridges this gap. It acts as a gateway, enabling nodes on the `meshmesh` network to communicate with devices on an Ethernet network, and vice-versa.

## Architecture

The border router is designed to run on a device with both a `meshmesh` radio and a standard network interface, like the `WT32-ETH01` (which has Ethernet).

It works by performing Network Address Translation (NAT):

1.  The border router has an IP address on the Ethernet network.
2.  When a `meshmesh` node sends a packet to a device on the Ethernet, the border router translates the packet from the `meshmesh` protocol to a standard TCP/IP packet. It uses its own IP address as the source for the outgoing packet.
3.  It maintains a state table to keep track of active connections.
4.  When a response is received from the Ethernet, the border router uses its state table to determine which `meshmesh` node the packet is for, translates it back to the `meshmesh` protocol, and forwards it to the correct node.

This architecture allows `meshmesh` nodes to access services on the IP network (like an MQTT broker or a web server) without needing a full IP stack themselves.

## Hardware

This component is designed with the **WT32-ETH01** board in mind, which provides both an ESP32 microcontroller and an Ethernet port. However, it could be adapted for other hardware with similar capabilities.

## Configuration

*This section is a work in progress.*

```yaml
# Example configuration for the border_router component
border_router:
  # Configuration options will be added here
```
