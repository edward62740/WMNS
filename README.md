# Wireless Mesh Network System


![alt text](https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/mesh.png "Mesh Devices")

This is a wireless sensor network system as a section of a larger project for smart home sensing and security, to provide real-time data collection of various data types (see below).
The data is collected in InfluxDB and can be viewed graphically in Grafana.
![alt text](https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/graph.jpg "Grafana")
\


This wireless mesh network is a proprietary, decentralized flood type network. The network consists of three distinct node types: routers (R), gateways (GT) and low-power end devices (LP).
Routers retransmit received data to form the backbone of the network.
Gateways are similar to routers and forward mesh network data to a server/cloud.
End devices are low-power nodes that sleep for most of the time, periodically transmitting data.

\
![alt text](https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/network.png "Mesh Devices")
\
Currently, the following data is collected by the system through multiple sensor types:
* Temperature
* Relative humidity
* Barometric pressure
* Illuminance
* UV levels
* Sudden/discontinous motion
* Person counter
* CO2 concentration
* VOC concentration
* Battery levels
* Signal strength

The focus of this project is to create easily deployable, low-maintenance (thereby low-power) sensors for collection of non-sensitive data. The various sensors have a battery life typically between 1-4 years.


Released under the GPL-2.0 License
