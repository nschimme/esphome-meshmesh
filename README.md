# Meshmesh Network

Meshmesh is an implementation of a protocol for mesh communication of [ESPHome](https://esphome.io/) nodes that works on ESP8266 and ESP32 based boards and can be integrated with a Home Assistant instance.  The protocol is based on the **802.11B** frame format and is compatible with the radio equipment of chips such as **ESP32** and **ESP8266**.

1) Is based on the  [Esphome](https://esphome.io/) firmware
2) It relies on raw **802.11B** packets and does not require a wifi AP to  work 
3) The packets can make jumps on intermediate devices to extend the  range of the network. 
4) There is not any limit on number of nodes. 
5) Beacons are not required to maintain active the connections (less  electromagnetic pollution)
6) Compatible with the [ESPhome](https://esphome.io/) API component 
7) Compatible with [Home Assistant](https://www.home-assistant.io/)  trough the software HUB [meshmeshgo](https://github.com/EspMeshMesh/meshmeshgo). 
8) It require a single [Esphome](https://esphome.io/) device connected to elaboration unit that run the [HUB](https://github.com/EspMeshMesh/meshmeshgo) software.
9) The topology of the network is dynamic and can be changed easily
10) Implemented from scratch not based from other mesh technologies.

For further explanation and tutorials, go to: [meshmeshgo](https://github.com/EspMeshMesh/meshmeshgo) page.

## Use as external component

The component can be imported in existing configuration as an [external component](https://esphome.io/components/external_components/), in order to make the network work some other bundled components must be overridden. 

```yaml
external_components:
  - source: github://EspMeshMesh/esphome-meshmesh@main
    components: [meshmesh, meshmesh_direct, network, socket, esphome]
```

## Meshmesh Component

This component allows ESPHome to communicate with other ESPHome devices using a mesh protocol based on the 802.11B frame. This network is compatible with both ESP8266 and ESP32 devices.

```yaml
# Example configuration 
meshmesh:
  channel: 3
  password: !secret meshmesh_password
```

* **channel** (Required, int): The Wi-Fi channel that Meshmesh will use to send/receive data packets. 
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

## Meshmesh Direct

```yaml
# Example configuration for meshmesh_direct
meshmesh_direct:
```

Automations:

* **on_receive** (Optional, int) default to **2048**: Receive buffer size for the serial port used to communicate with the HUB.

Actions:

* **meshmesh.send**: This is an action for sending a data packet over the meshmesh protocol.

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
