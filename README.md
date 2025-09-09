# [ESPMeshMesh Network](https://github.com/EspMeshMesh/esphome-meshmesh)

ESPMeshMesh is an implementation of a protocol for mesh communication of [ESPHome](https://esphome.io/) nodes that works on ESP8266 and ESP32 based boards and can be integrated with a Home Assistant instance.  The protocol is based on the **802.11B** frame format and is compatible with the radio equipment of chips such as **ESP32** and **ESP8266**.

This repository contains the firmware to load on devices .

1. Is based on the  [ESPHome](https://esphome.io/) firmware
2. It relies on raw **802.11B** packets and does not require a wifi AP to  work 
3. The packets can make jumps on intermediate devices to extend the  range of the network. 
4. There is not any limit on number of nodes. 
5. Beacons are not required to maintain active the connections (less  electromagnetic pollution)
6. Compatible with the [ESPHome](https://esphome.io/) API component 
7. Compatible with [Home Assistant](https://www.home-assistant.io/)  trough the software HUB [meshmeshgo](https://github.com/EspMeshMesh/meshmeshgo). 
8. It require a single [ESPHome](https://esphome.io/) device connected to elaboration unit that run the [HUB](https://github.com/EspMeshMesh/meshmeshgo) software.
9. The topology of the network is dynamic and can be changed easily
10. Implemented from scratch not based from other mesh technologies.

For further explanation and tutorials, go to: [meshmeshgo](https://github.com/EspMeshMesh/meshmeshgo) page.

## Use as external component

The component can be imported in existing configuration as an [external component](https://esphome.io/components/external_components/), in order to make the network work some other bundled components must be overridden. 

```yaml
external_components:
  - source: github://EspMeshMesh/esphome-meshmesh@main
    components: [meshmesh, meshmesh_direct, network, socket, esphome]
```

## Transparent Networking with Border Router

The `border_router` component, when combined with the `border_router` socket implementation, provides transparent networking for mesh nodes. This allows standard ESPHome components (like `http_request`, `mqtt`, etc.) to work over the mesh as if they had a native IP connection.

### How it Works

The `border_router` socket implementation intercepts standard socket API calls made by other ESPHome components. Instead of sending them to the local network stack (which doesn't exist on the mesh nodes), it translates them into the custom "Mesh-to-Router" protocol and sends them to the `border_router` component over the mesh. The `border_router` then performs these network operations on behalf of the mesh node.

### Configuration

To enable this feature, you must configure both the `border_router` component on your gateway device and the `socket` implementation on your mesh nodes.

**Gateway Node (`coordinator-esp32-wroom32.yaml`)**
```yaml
# ... other config ...

# Enable the border router component
border_router:
  id: my_border_router
  meshmesh_id: meshmesh_hub # The ID of your meshmesh component
  ethernet_id: eth # The ID of your ethernet component
```

**Mesh Node (`node-esp32-wroom32s3.yaml`)**
```yaml
# ... other config ...

# Set the socket implementation to use the border router
socket:
  implementation: border_router
  border_router_address: 0x123456 # Address of the node running the border_router

# Now, standard network components will work transparently
http_request:
  - id: my_http_request
    url: http://example.com
    on_response:
      - logger.log: "HTTP Response received!"
```

## Socket component override

The ESPHome socket bundled component must be overridden to make the network compatible with API and OTA components.

```yaml
socket:
  implementation: meshmesh_esp8266
```

* implementation (Required, string): Is the socket implementation compatible with the mesh network architecture. Must be always set to the **meshmesh_esp8266** value.

## Meshmesh component

This component allows ESPHome to communicate with other ESPHome devices using a mesh protocol based on the 802.11B frame. This network is compatible with both ESP8266 and ESP32 devices.

```yaml
# Example configuration 
meshmesh:
  channel: 3
  password: !secret meshmesh_password
```

* **channel** (Required, int): The Wi-Fi channel that ESPMeshmesh will use to send/receive data packets. 
* **password** (Required, string): The AES shared secret used to encrypt all packets on the network.
* **hardware_uart** (Optional, int) default to **0**: UART number used by the base node (coordinator) to communicate with the HUB (meshmeshgo).
* **baud_rate** (Optional, int) default to **460800**: Baud rate of the serial port used to communicate with the HUB.
* **rx_buffer_size** (Optional, int) default to **2048**: Receive buffer size for the serial port used to communicate with the HUB.
* **tx_buffer_size** (Optional, int) default to **4096**: Transmit buffer size for the serial port used to communicate with the HUB.

```yaml
# Example configuration for coordinator node
meshmesh:
  baud_rate: 460800
  rx_buffer_size: 2048
  tx_buffer_size: 4096
  password: !secret meshmesh_password
  channel: 3
```

```yaml
# Example configuration for generic network node
meshmesh:
  baud_rate: 0
  rx_buffer_size: 0
  tx_buffer_size: 0
  password: !secret meshmesh_password
  channel: 3
```

## Packet Transport Platform

The MeshMesh component provide a packet transport platform implementation. 

```yaml
# Example configuration entry for packet transport over espmeshmesh.
packet_transport:
  platform: meshmesh
  update_interval: 5s
  address: 0x000000
```

* **address** (Required, int): The address for the remote node counterpart.

## Meshmesh Direct

```yaml
# Example configuration for meshmesh_direct
meshmesh_direct:
```

Automations:

* **on_receive** (Optional, int) default to **2048**: Receive buffer size for the serial port used to communicate with the HUB.

Actions:

* **meshmesh.send**: This is an action for sending a data packet over the ESPMeshmesh protocol.

Configuration variables:

* **address** (Required, int): Target of the transmission. This can be obtained from the last three bytes of the MAC address.
* **data** (Required, string): Data to transmit to the remote node.

## Meshmesh Direct Switch

Switch component: This is a virtual switch component that can be used to control a remote physical switch present on the remote node.

```yaml
switch:
  - platform: meshmesh_direct
    name: "Switch Name"
    address: 0xAABBCC
    target: 0xDDEE
```

Configuration variables:

* **address** (Required, int): The address for the remote node that contains the switch.
* **target** (Required, int): The hash value of the physical switch on the remote node.
* **id** (Optional, string): Manually specify the ID for code generation. At least one of id and name must be specified.
* **name** (Optional, string): The name of the switch. At least one of id and name must be specified.
