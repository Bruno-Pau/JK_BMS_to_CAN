# JK_BMS_to_CAN
RS232 TTL to CAN Pylon Emulation Interface on ESP32
# Introduction
Last year I built a photovoltaic system with batteries.<br>
I bought a SOFAR HYD 6000-ES inverter, LiFePo4 batteries and BMS (Battery Management System).
But the SOFAR only accepted BMS Pylon CAN connections. <br>
So I had to develop an interface between my JK BMS and the SOFAR. <br><br>
![Image](schema_install_photovoltaique_2.png) <br> <br>
The interface is based on a ESP32 whith a MPC2515 CAN interface <br>
![Image](carte_ESP32.png) <br> <br>
Communication between ESP32 and JK BMS is based on RS232 TTL with JK proprietary protocol. <br>
Communication between ESP32 and SOFAR is based on CAN whith Pylon emulation. <br> <br>
# List of equipment
[ESP32](https://fr.aliexpress.com/item/1005006629784548.html)<br>
[MPC2515](https://fr.aliexpress.com/item/1005005223498304.html)<br>
[ADuM1201](https://fr.aliexpress.com/item/32815864904.html)<br>
[Micro JST 1.25mm Connector](https://fr.aliexpress.com/item/4001171710583.html)<br>
[JK BMS](https://fr.aliexpress.com/item/1005004590744267.html)<br>

# ESP32 card schematic diagram
![Image](Interface_SOFAR_JK_2.png) <br><br>
This card can communicate with 4 serial lines, but only one (BMS1) is used in this program.<br>

# JK BMS protocol
Look at 
[BMS-RS485 Communication protocol.pdf](https://github.com/Bruno-Pau/JK_BMS_to_CAN/blob/main/BMS-RS485Communication%20protocol.pdf)<br>

# CAN Pylon emulation protocol
Look at [CAN Pylon emulation protocol.pdf](https://github.com/Bruno-Pau/JK_BMS_to_CAN/blob/main/CAN-Bus-protocol-PYLON-low-voltage-V1.2-20180408.pdf)<br>

# ESP32 code
code was developed under Arduino 2.0 environnement
[Sofar5.ino](https://github.com/Bruno-Pau/JK_BMS_to_CAN/blob/main/JkBms_Sofar5.ino) <br>

# Other links
[JK BMS RS485 protocol](https://github.com/jblance/mpp-solar/issues/112) <br>

