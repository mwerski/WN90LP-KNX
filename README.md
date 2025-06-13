# WN90LP-KNX
ESP32 based Modbus->KNX Gateway for Ecowitt WN90

Used hardware:
* Ecowitt WN90LP
* Lilygo T-RSS3
* SDS011 Air Quality Sensor
* Dallas 1820 1wire Temperature Sensor

Assembly:
DS1820 and SDS011 will be connected to the GPIO pinout header of the T-RSS3 (see back of the board for the pinout).
DS1820 gets connected to 3V3, GPIO11 and GND, a 4k7 pullup resistor will have to be connected to 3V3 and DATA.
SDS011 gets connected to 5V, RXD, TXD and GND - RXD on the T-RSS3 gets connected to TXD on the SDS011 and vice versa.