# Wireless Mesh Network System (WMNS)


![alt text](https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/mesh.png "Mesh Devices")

## Overview

This is a wireless sensor network system as a part of an ongoing project for smart home sensing and security. The intent is to create easily deployable, low-maintenance (thereby low-power) sensors for real-time collection of non-sensitive data. The various sensors have a battery life typically between 1-4 years off a CR2032 (excl. CO2SN - 4 months, Li-ion).
The data is collected in InfluxDB and can be viewed graphically in Grafana.
![alt text](https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/graph.jpg "Grafana")


## Network Structure
This wireless mesh network is a proprietary, decentralized flood type network. The network consists of three distinct node types: routers (R), gateways (GT) and low-power end devices (LP).\
Routers retransmit received data to form the backbone of the network.\
Gateways are similar to routers and forward mesh network data to a server/cloud.\
End devices are low-power nodes that sleep for most of the time, periodically transmitting data.


![alt text](https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/network.png "Mesh Devices")


## Measured data
Currently, the following data is collected by the system through multiple sensor types (sensors in brackets, not necessarily utilized as such):
* Temperature (BME280, BME680, Si7021, SCD41, BMA400, mcu_internal)
* Relative humidity (BME280, BME680, Si7021, SCD41)
* Barometric pressure (BME280, BME680)
* Illuminance (OPT3001, VEML6030, Si1133./5)
* UV levels (Si1133/5)
* Sudden/discontinous motion (BMA400)
* Person counter (VL53L1X)
* CO2 concentration (SCD41)
* VOC concentration (BME680)
* Battery levels (ADC)
* Signal strength (radio)


## Devices

GPSN             |  LRSN      |  CO2SN  | ALSN
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/gpsn.png" alt="GPSN" width="200"/><br />General-purpose configurable sensor for: temperature, humidity, air pressure, light, VOCs, motion|<img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/lrsn.png" alt="LRSN" width="200"/><br />Person counter sensor |  <img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/co2sn.png" alt="CO2SN" width="200"/><br />Carbon dioxide concentration sensor | <img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/alsn.png" alt="ALSN" width="200"/><br />Specialized light sensor for visible, UV, IR

GATEWAY             |  ROUTER32PA      |  ROUTER40  | LTSN | TERMINAL
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/gateway.png" alt="GATEWAY" width="150"/><br />Internet gateway node  |<img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/router32pa.png" alt="ROUTER32PA" width="150"/><br />Power amplified router node |  <img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/router40.png" alt="ROUTER40" width="150"/><br />USB-powered router node |<img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/ltsn.png" alt="LTSN" width="150"/><br />Specialized temp sensor for sub-zero|<img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/terminal.png" alt="TERMINAL" width="150"/><br />redacted for security reasons

## Performance

The following performance factors were considered during the design of the WMNS:
* Low maintenance - essentially long battery life and reliability
</br> _Low power circuit and software design from bottom up. Certain sensors (i.e those expected to face harsher conditions) are conformally coated to increase reliability._
</br> _Network has no central "coordinator" type node which is required to sustain the network. Packets can take multiple paths to the gateway. Gateways can be configured as redundancies._
* Ease of use - able to easily add more sensors (and more types), and modify configurations.
</br> _Easy plug-and-play to add more sensors. Only the gateway needs modifications to add new sensor types (for database side processing)._
</br> _Sensors can easily be configured for different sample rates._
* Range - good wireless range vs power consumption compromise
</br> _Tuned internal antennas allow for lower tx power (and less power consumption)._ 
</br><img src="https://github.com/edward62740/Wireless-Mesh-Network-System/blob/master/Documentation/gpsntune.PNG" alt="GPSN Return Loss" width="400"/>
* Low-cost - cheap sensors
</br> _Runs off comparatively low-cost and widely used nRF52 series SoCs. Same design template was used across sensors to drop development time and costs._
</br> _No protocol overhead or IP costs involved._

This system has been deployed in stages since Q1 21 with zero system failures (e.g gateway restarts, sensor malfunctions and drops out of network, hardware failure..).
Total deployed devices: 28

Preliminary. CAA 060222

Released under the GPL-2.0 License
