#pragma once
// Hardwaredefinition LilyGO T-RSS3 (ESP32-S3, RS485, KNX IP ueber WLAN)
#include "HardwareConfig.h"

#ifdef DEVICE_WN90_TRSS3
    #define DEVICE_ID "WN90TRSS3"
    #define DEVICE_NAME "WN90LP Gateway (T-RSS3)"

    #ifndef KNX_IP_WIFI
        #define KNX_IP_WIFI
    #endif

    // Prog-Taster (aktiv LOW) und Prog-LED als einzelne WS2812 an GPIO 1
    #define PROG_BUTTON_PIN 5
    #define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
    #define OPENKNX_SERIALLED_ENABLE
    #define OPENKNX_SERIALLED_PIN 1
    #define OPENKNX_SERIALLED_NUM 1
    #define PROG_LED_PIN 0
    #define PROG_LED_COLOR 20, 0, 0

    // RS485 zur WN90LP (Modbus RTU, 9600 8N1)
    #define WX_RS485_SERIAL Serial1
    #define WX_RS485_RX_PIN 2
    #define WX_RS485_TX_PIN 3
    #define WX_RS485_DE_PIN 4

    // Feinstaubsensor SDS011 am Steckverbinder RXD/TXD
    #define WX_SDS_SERIAL Serial2
    #define WX_SDS_RX_PIN 35
    #define WX_SDS_TX_PIN 38

    // 1-Wire (DS18B20) mit 4k7 Pull-up nach 3V3
    #define WX_ONEWIRE_PIN 11
#endif
