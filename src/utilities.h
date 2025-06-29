/*
	utilities, hardware definitions and helpers
*/
#ifndef _utilities_h_
#define _utilities_h_

#pragma once

#include <Arduino.h>

#define Serial232               Serial0
#define Serial485               Serial1
#define RS485_BAUD              9600
#define RS232_BAUD              9600
#define RS485_TX_ENABLE         HIGH
#define RS485_RX_ENABLE         LOW
#define NUMPIXELS               1
#define WIREPIN									11
#define KNXPIN									5

#if CONFIG_IDF_TARGET_ESP32C3

#define RS485_RX_PIN            3
#define RS485_TX_PIN            10
#define RS485_CON_PIN           5
#define RS232_RX_PIN            1
#define RS232_TX_PIN            0
#define LED_PIN                 4
#define KEY_PIN                 2
#define BOOT_PIN                9

#elif CONFIG_IDF_TARGET_ESP32S3


#define RS485_RX_PIN            2
#define RS485_TX_PIN            3
#define RS485_CON_PIN           4

#define RS232_RX_PIN            35
#define RS232_TX_PIN            38
#define LED_PIN                 1
#define KEY_PIN                 5
#define BOOT_PIN                0
#endif

double round(double n) { // rounds to 2 digits
	double m = powf( 10.0f, 2 ); // truncate to x.yz
	n = roundf( n * m ) / m;
	return n;
}

char* toCharArray(String str) {	// string to character array
	return &str[0];
}

String getChipID() {	// returns ESP chip ID
  String id;
  uint64_t chipid;
  char ssid[13];
  chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  snprintf(ssid, 13, "%04X%08X", chip, (uint32_t)chipid);
  for ( int i = 0; i < 12; i++){
    id += String(ssid[i]);
  }
  return String(id);
}




#endif