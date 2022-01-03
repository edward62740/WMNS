# Mesh Firmware
Currently UNDER DEVELOPMENT.

This is a wireless mesh network system which integrates smart home/industry sensing, tracking and security into a single network.

The wireless mesh network is a proprietary, decentralized flood type network. The network consists of three distinct node types: routers (R), gateways (GT) and low-power end devices (LP).
Routers retransmit received data to form the backbone of the network.
Gateways are similar to routers and forward mesh network data to a server/cloud.
End devices are low-power nodes that sleep for most of the time, periodically transmitting data.

![alt text](https://github.com/edward62740/Mesh-Firmware/blob/master/docu/meshsensorspartial.jpeg?raw=true)

This project involves the design of several hardware devices (https://github.com/edward62740/Mesh-Hardware.git), as well as firmware. It is important to note that each hardware device may be compatible with one or more firmware types.

![alt text](https://github.com/edward62740/Mesh-Firmware/blob/master/docu/meshnetwork.jpg?raw=true)

# Compatibility Matrix
y-axis is the HARDWARE DEVICE, x-axis is the FIRMWARE TYPE. if compatible, node type is shown.
It is recommended to use the highlighted combination where possible.
|            | Environment | Motion | Airquality | Ranger | Proximity | Gateway | Router | Controller |
|------------|-------------|--------|------------|--------|-----------|---------|--------|------------|
| GPSN       |     LP      |   LP   |     LP     |        |           |         |        |            |
| LRSN       |             |        |            |   LP   |    LP     |         |        |            |
| Gateway    |             |        |            |        |           |    GT   |        |            |
| Router     |             |        |            |        |           |         |    R   |            |
| Controller |             |        |            |        |           |         |        |     AP     |

# Completion % Sept 2020
|            | Environment | Motion | Airquality | Ranger | Proximity | Gateway | Router | Controller |
|------------|-------------|--------|------------|--------|-----------|---------|--------|------------|
|            |     100     |   100  |    DEPR    |   60   |    30     |   100   |   95   |     50     |


Released under the GPL-3.0 License
