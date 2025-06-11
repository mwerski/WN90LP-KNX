#include "wn90.h"
struct tm myTime;
bool timeKnown = false; // do we know the correct time?
bool dateKnown = false; // do we know the correct date?

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
 if (!go.value()) sendTemperature();
}
void callback_Humidity (GroupObject& go) {
 if (!go.value()) sendHumidity();
}
void callback_WindSpeed (GroupObject& go) {
 if (!go.value()) sendWindSpeed();
}
void callback_GustSpeed (GroupObject& go) {
 if (!go.value()) sendGustSpeed();
}
void callback_WindDirection (GroupObject& go) {
 if (!go.value()) sendWindDirection();
}
void callback_Pressure (GroupObject& go) {
 if (!go.value()) sendPressure();
}
void callback_RainFall (GroupObject& go) {
 if (!go.value()) sendRainFall();
}
void callback_RainCounter (GroupObject& go) {
 if (!go.value()) sendRainCounter();
}
void callback_UVindex (GroupObject& go) {
 if (!go.value()) sendUVindex();
}
void callback_Brightness (GroupObject& go) {
 if (!go.value()) sendBrightness();
}

// measurements gathered by other sensors (1wire, sds011, ...)
void callback_pm25 (GroupObject& go) {
 if (!go.value()) sendPM25();
}
void callback_pm10 (GroupObject& go) {
 if (!go.value()) sendPM10();
}
void callback_Temperature1 (GroupObject& go) {
 if (!go.value()) sendTemperature1();
}

// these callbacks refer to calculated measurements
void callback_pm25_normalized (GroupObject& go) {
 if (!go.value()) sendPM25();
}
void callback_pm10_normalized (GroupObject& go) {
 if (!go.value()) sendPM10();
}
void callback_dewPoint (GroupObject& go) {
 if (!go.value()) sendDewpoint();
}
void callback_frostPoint (GroupObject& go) {
 if (!go.value()) sendFrostpoint();
}
void callback_PressureTrend1 (GroupObject& go) {
 if (!go.value()) sendPressureTrend1();
}
void callback_PressureTrend3 (GroupObject& go) {
 if (!go.value()) sendPressureTrend3();
}

