#include "wn90.h"

wn90::wn90(ModbusMaster node) {
	
}

void wn90::begin() {

}

struct tm myTime;
bool timeKnown = false; // do we know the correct time?
bool dateKnown = false; // do we know the correct date?

#pragma region KNX send to bus
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
#pragma endregion

#pragma region KNX callbacks
// receiving date, time from the bus, setting localtime
void callback_dateTime (GroupObject& go) {
	// Untested
	if (go.value()) {
		dateKnown = true;
		timeKnown = true;
		myTime = KoAPP_Date.value();
		unsigned short tmp_sec = myTime.tm_sec;
		unsigned short tmp_min = myTime.tm_min;
		unsigned short tmp_hour = myTime.tm_hour;
		unsigned short tmp_mday = myTime.tm_mday;
		unsigned short tmp_mon = myTime.tm_mon;
		unsigned short tmp_year = myTime.tm_year;
		char buf[52];
		sprintf(buf, "DateTime received from bus: %d.%d.%d, %02d:%02d:%02d", tmp_mday, tmp_mon, tmp_year, tmp_hour, tmp_min, tmp_sec );
		Serial.println(buf);
		Serial.println("Setting/Adjusting system time");
		setTime(tmp_hour, tmp_min, tmp_sec, tmp_mday, tmp_mon, tmp_year);
	}
}
void callback_time(GroupObject& go) {
	if (go.value()) {
		timeKnown = true;
		myTime = KoAPP_Time.value();
		unsigned short tmp_sec = myTime.tm_sec;
		unsigned short tmp_min = myTime.tm_min;
		unsigned short tmp_hour = myTime.tm_hour;
		char buf[52];
		sprintf(buf, "Time received from bus: %02d:%02d:%02d", tmp_hour, tmp_min, tmp_sec );
		Serial.println(buf);
		time_t t = now();
		setTime(tmp_hour, tmp_min, tmp_sec, day(t), month(t), year(t));
		if (dateKnown == true) {
			sprintf(buf, "Setting/Adjusting system time: %d.%d.%d, %02d:%02d:%02d", day(t), month(t), year(t), tmp_hour, tmp_min, tmp_sec );
			Serial.println(buf);
		}
	}
}
void callback_date(GroupObject& go) {
	if (go.value()) {
		dateKnown = true;
		myTime = KoAPP_Date.value();
		unsigned short tmp_mday = myTime.tm_mday;
		unsigned short tmp_mon = myTime.tm_mon;
		unsigned short tmp_year = myTime.tm_year;
		char buf[52];
		sprintf(buf, "Date received from bus: %d.%d.%d", tmp_mday, tmp_mon, tmp_year );
		Serial.println(buf);
		time_t t = now();
		setTime(hour(t), minute(t), second(t), tmp_mday, tmp_mon, tmp_year);
		if (timeKnown == true) {
			sprintf(buf, "Setting/Adjusting system time: %d.%d.%d, %02d:%02d:%02d", tmp_mday, tmp_mon, tmp_year, hour(t), minute(t), second(t) );
			Serial.println(buf);
		}
	}
}

// measurements gathered by wn90
void callback_Temperature (GroupObject& go) {
 if (!go.value()) send_Temperature();
}
void callback_Humidity (GroupObject& go) {
 if (!go.value()) send_Humidity();
}
void callback_WindSpeed (GroupObject& go) {
 if (!go.value()) send_WindSpeed();
}
void callback_GustSpeed (GroupObject& go) {
 if (!go.value()) send_GustSpeed();
}
void callback_WindDirection (GroupObject& go) {
 if (!go.value()) send_WindDirection();
}
void callback_Pressure (GroupObject& go) {
 if (!go.value()) send_Pressure();
}
void callback_RainFall (GroupObject& go) {
 if (!go.value()) send_RainFall();
}
void callback_RainCounter (GroupObject& go) {
 if (!go.value()) send_RainCounter();
}
void callback_UVindex (GroupObject& go) {
 if (!go.value()) send_UVindex();
}
void callback_Brightness (GroupObject& go) {
 if (!go.value()) send_Brightness();
}

// measurements gathered by other sensors (1wire, sds011, ...)
void callback_pm25 (GroupObject& go) {
 if (!go.value()) send_PM25();
}
void callback_pm10 (GroupObject& go) {
 if (!go.value()) send_PM10();
}
void callback_Temperature1 (GroupObject& go) {
 if (!go.value()) send_Temperature1();
}

// these callbacks refer to calculated measurements
void callback_pm25_normalized (GroupObject& go) {
 if (!go.value()) send_PM25_normalized();
}
void callback_pm10_normalized (GroupObject& go) {
 if (!go.value()) send_PM10_normalized();
}
void callback_dewPoint (GroupObject& go) {
 if (!go.value()) send_dewPoint();
}
void callback_frostPoint (GroupObject& go) {
 if (!go.value()) send_frostPoint();
}
void callback_PressureTrend1 (GroupObject& go) {
 if (!go.value()) send_PressureTrend1();
}
void callback_PressureTrend3 (GroupObject& go) {
 if (!go.value()) send_PressureTrend3();
}
/*
void callback_WindSpeedBFT (GroupObject& go) {
	if (!go.value()) send_WindSpeedBFT();
}
void callback_GustSpeedBFT (GroupObject& go) {
		if (!go.value()) send_GustSpeedBFT();
}
*/
#pragma endregion
