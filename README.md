# Jeep-AuxBus
Accessory Controller for Jeep Wrangler using CANBUS

To use this Arduino sketch you need to download the SparkFun CANBUS library from my github.  The library source files have been modified to allow filtering.  Currently there are 3 message IDs that are allowed through the filter.  

ID 0x211  Vehicle Speed Data
ID 0x208  Lighting Control
ID 0x308  Dimmer Control

You can have a total of 6 message IDs come through the filters.  I will upload a seperate document explaining how to add and modify filters.

