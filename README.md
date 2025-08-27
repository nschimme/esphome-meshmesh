# Meshmesh network

MeshMesh is an implementation of a protocol for mesh communication of ESPHome nodes framework that works on ESP8266 and ESP32 based boards and can be integrated with an HomeAssistant instance. 
The protocol is based on 802.11B frame format and is compatible with the radio equipment of chips such as ESP32 and ESP8266.

For for further explanation and tutorials go to: [meshmeshgo](https://github.com/EspMeshMesh/meshmeshgo)

## meshmesh compoenent

This component allows ESPHome to communicate with other ESPHome devices using a mesh protocol based on the 802.11B frame. This network is compatible either with esp8266 and esp32 devices.

```yaml
# Example of configuration 
meshmesh:
  channel: 3
  password: !secret meshmesh_password
```

* **channel** (Required, int): The Wi-Fi channel that the ESPMeshMesh will use to send/receive data packets. 
* **password** (Required, string): The AES shared secret used to cypher all packets on the network.
* **hardware_uart** (Optional, int) default to **0**: UART number to used by the base node (coordinator) to communicate with the HUB (meshmeshgo):
* **baud_rate** (Optional, int) default to **460800**: Baud rate of the serial port used to communicate with the HUB.
* **rx_buffer_size** (Optional, int) default to **2048**:  Receive buffer size for the  serial port used to communicate with the HUB.
* **tx_buffer_size** (Optional, int) default to **4096**: Transmit buffer size  for the  serial port used to communicate with the HUB

```yaml
# Example of configuration for coordinator node
meshmesh:
  baud_rate: 460800
  rx_buffer_size: 2048
  tx_buffer_size: 4096
  password: !secret meshmesh_password
  channel: 3
```

```yaml
# Example of configuration for generic network node
meshmesh:
  baud_rate: 0
  rx_buffer_size: 0
  tx_buffer_size: 0
  password: !secret meshmesh_password
  channel: 3
```

## meshmmesh_direct

```yaml
# Example of configuration for the meshmesh_direct
meshmesh_direct:
```

Automatons:

* **on_receive** (Optional, int) default to **2048**:  Receive buffer size for the  serial port used to communicate with the HUB.

Actions:

* **meshmesh.send**: This is an Action for sending a data packet over the meshmesh protocol.

Configuration variables:

* **address** (Required, int):  Target of the transmission. This can be obtained from the last three bytes of the MAC address.
* **data** (Required, string): Data to  transmit to the remote node.


