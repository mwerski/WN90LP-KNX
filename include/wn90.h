#ifndef wn90_h
#define wn90_h

#include <Arduino.h>
#include "knxprod.h"
#include <knx.h>
#include <TimeLib.h>
#include <ModbusMaster.h>

struct dp {
	double		value;
	double		lastvalue;
	double		
}
class wn90 {

public:
wn90(ModbusMaster none);
void begin();

#pragma region KNX Callbacks

void send_Temperature();
void send_Humidity();
void send_WindSpeed();
void send_GustSpeed();
void send_WindDirection();
void send_Pressure();
void send_RainFall();
void send_RainCounter();
void send_UVindex();
void send_Brightness();
void send_PM25();
void send_PM10();
void send_Temperature1();
void send_PM25_normalized();
void send_PM10_normalized();
void send_dewPoint();
void send_frostPoint();
void send_PressureTrend1();
void send_PressureTrend3();

// receiving date/time from the bus, setting localtime
void callback_dateTime (GroupObject& go);
void callback_time(GroupObject& go);
void callback_date(GroupObject& go);

// measurements gathered by wn90
void callback_Temperature (GroupObject& go);
void callback_Humidity (GroupObject& go);
void callback_WindSpeed (GroupObject& go);
void callback_GustSpeed (GroupObject& go);
void callback_WindDirection (GroupObject& go);
void callback_Pressure (GroupObject& go);
void callback_RainFall (GroupObject& go);
void callback_RainCounter (GroupObject& go);
void callback_UVindex (GroupObject& go);
void callback_Brightness (GroupObject& go);

// measurements gathered by other sensors (1wire, sds011, ...)
void callback_Temperature1 (GroupObject& go);
void callback_pm25 (GroupObject& go);
void callback_pm10 (GroupObject& go);

// these callbacks refer to calculated measurements
void callback_dewPoint (GroupObject& go);
void callback_frostPoint (GroupObject& go);
void callback_pm25_normalized (GroupObject& go);
void callback_pm10_normalized (GroupObject& go);
void callback_PressureTrend1 (GroupObject& go);
void callback_PressureTrend3 (GroupObject& go);
/*
void callback_WindSpeedBFT (GroupObject& go);
void callback_GustSpeedBFT (GroupObject& go);
*/
#pragma endregion

private:
};

#endif