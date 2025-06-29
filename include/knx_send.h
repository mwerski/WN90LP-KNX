#ifndef _knx_send_h_
#define _knx_send_h_

#pragma region KNX send functions
/*
	Senders - these functions are sending data to the bus
*/
void sendHeartbeat() {
	Serial.println(" -> Sending heartbeat to bus");
	KoAPP_Heartbeat.value(1);
}

void sendTemperature() {
	if (temperature.read) {
		Serial.printf(" -> Sending temperature in °C (%0.2f) to bus\n", temperature.value);
		(ParamAPP_Temperatur_DPT == 0) ? KoAPP_Temperatur_DPT9.value(temperature.value) : KoAPP_Temperatur_DPT14.value(temperature.value);
		temperature.last = temperature.value;
		temperature.lastread = true;
	} else {
		Serial.println(" -- Temperature not yet available, won't send to bus - delay task");
		task_sendTemperature.cancel();
	}
}

void sendTemperature1() {
	if (temperature1.read) {
		Serial.printf(" -> Sending 1wire temperature in °C (%0.2f) to bus\n", temperature1.value);
		(ParamAPP_Temperatur_DPT == 0) ? KoAPP_Temperatur_1wire_DPT9.value(temperature1.value) : KoAPP_Temperatur_1wire_DPT14.value(temperature1.value);
		temperature1.last = temperature1.value;
		temperature1.lastread = true;
	} else {
		Serial.println(" -- 1wire temperature not yet available, won't send to bus - delay task");
		task_sendTemperature1.cancel();
	}
}

void senddewpoint() {
	if (dewpoint.read) {
		Serial.printf(" -> Sending dewpoint in °C (%0.2f) to bus\n", dewpoint.value);
		(ParamAPP_Temperatur_DPT == 0) ? KoAPP_Taupunkt_DPT9.value(dewpoint.value) : KoAPP_Taupunkt_DPT14.value(dewpoint.value);
		dewpoint.last = dewpoint.value;
		dewpoint.lastread = true;
	} else {
		Serial.println(" -- dewpoint not yet available, won't send to bus - delay task");
		task_senddewpoint.cancel();
	}
}

void sendfrostpoint() {
	if (frostpoint.read) {
		Serial.printf(" -> Sending frostpoint in °C (%0.2f) to bus\n", frostpoint.value);
		(ParamAPP_Temperatur_DPT == 0) ? KoAPP_Frostpunkt_DPT9.value(frostpoint.value) : KoAPP_Frostpunkt_DPT14.value(frostpoint.value);
		frostpoint.last = dewpoint.value;
		frostpoint.lastread = true;
	} else {
		Serial.println(" -- frostpoint not yet available, won't send to bus - delay task");
		task_sendfrostpoint.cancel();
	}
}

void sendHumidity() {
	if (humidity.read) {
		Serial.printf(" -> Sending humidity in %% (%0.0f) to bus\n", humidity.value);
		switch (ParamAPP_Feuchte_DPT) {
			case 0: KoAPP_Feuchte_DPT6.value(humidity.value); break;
			case 1: KoAPP_Feuchte_DPT9.value(humidity.value); break;
			case 2: KoAPP_Feuchte_DPT14.value(humidity.value); break;
		}
		humidity.last = humidity.value;
		humidity.lastread = true;
	} else {
		Serial.println(" -- Humidity not yet available, won't send to bus - delay task");
		task_sendHumidity.cancel();
	}
}

void sendAbsHumidity() {
	if (humidity.read && temperature.read) {
		float absH = absoluteHumidity( temperature.value, humidity.value );
		Serial.printf(" -> Sending absolute humidity in g/m3 (%0.0f) to bus\n", humidity.value);
		switch (ParamAPP_Feuchte_DPT) {
			case 0: KoAPP_Feuchte_DPT6.value(absH); break;
			case 1: KoAPP_Feuchte_DPT9.value(absH); break;
			case 2: KoAPP_Feuchte_DPT14.value(absH); break;
		}
	} else {
		Serial.println(" -- Absolute humidity not yet available, won't send to bus");
	}
}

void sendWindSpeed() {
	if (windSpeed.read) {
		Serial.printf(" -> Sending wind speed in m/s (%0.2f) to bus\n", windSpeed.value);
		(ParamAPP_WindSpeed_DPT == 0) ? KoAPP_WindSpeed_DPT9.value(windSpeed.value) : KoAPP_WindSpeed_DPT14.value(windSpeed.value);
		windSpeed.last = windSpeed.value;
		windSpeed.lastread = true;
	} else {
		Serial.println(" -- Wind speed not yet available, won't send to bus - delay task");
		task_sendWindSpeed.cancel();
	}
}

void sendGustSpeed() {
	if (gustSpeed.read) {
		Serial.printf(" -> Sending gust speed in m/s (%0.2f) to bus\n", gustSpeed.value);
		(ParamAPP_WindSpeed_DPT == 0) ? KoAPP_GustSpeed_DPT9.value(gustSpeed.value) : KoAPP_GustSpeed_DPT14.value(gustSpeed.value);
		gustSpeed.last = gustSpeed.value;
		gustSpeed.lastread = true;
	} else {
		Serial.println(" -- Gust speed not yet available, won't send to bus - delay task");
		task_sendGustSpeed.cancel();
	}
}

void sendGustSpeedBft() {
	if (gustSpeed.read) {
		u_int8_t s = bft(gustSpeed.value);
		Serial.printf(" -> Sending gust speed in beaufort (%0.0f) to bus\n", s);
		KoAPP_GustSpeed_BFT_DPT5.value(s);
	} else {
		Serial.println(" -- Gust speed in beaufort not yet available, won't send to bus");
	}
}

void sendWindSpeedBft() {
	if (windSpeed.read) {
		u_int8_t s = bft(windSpeed.value);
		Serial.printf(" -> Sending wind speed in beaufort (%0.0f) to bus\n", s);
		KoAPP_WindSpeed_BFT_DPT5.value(s);
	} else {
		Serial.println(" -- Wind speed in feaufort not yet available, won't send to bus");
	}
}

void sendWindDirection() {
	if (windDirection.read) {
		Serial.printf(" -> Sending wind direction in ° (%0.0f) to bus\n", windDirection.value);
		(ParamAPP_WindDir_DPT == 0) ? KoAPP_WindDir_DPT9.value(windDirection.value) : KoAPP_WindDir_DPT14.value(windDirection.value);
		windDirection.last = windDirection.value;
		windDirection.lastread = true;
	} else {
		Serial.println(" -- Wind direction not yet available, won't send to bus - delay task");
		task_sendWindDirection.cancel();
	}
}

void sendPressure() {
	if (pressure.read) {
		Serial.printf(" -> Sending air pressure in hPa (%0.2f) to bus\n", pressure.value);
		(ParamAPP_Pressure_DPT == 0) ? KoAPP_Pressure_DPT9.value(pressure.value) : KoAPP_Pressure_DPT14.value(pressure.value);
		pressure.last = pressure.value;
		pressure.lastread = true;
	} else {
		Serial.println(" -- Air pressure not yet available, won't send to bus - delay task");
		task_sendPressure.cancel();
	}
}

void sendPressureTrend1() {
	if (pressureTrend1.read) {
		Serial.printf(" -> Sending hourly air pressure trend (%0.2f) to bus\n", pressureTrend1.value);
		(ParamAPP_Pressure_DPT == 0) ? KoAPP_PressureTrend1h_DPT9.value(pressureTrend1.value) : KoAPP_PressureTrend1h_DPT14.value(pressureTrend1.value);
		pressureTrend1.last = pressureTrend1.value;
		pressureTrend1.lastread = true;
	} else {
		Serial.println(" -- Hourly air pressure trend not yet available, won't send to bus - delay task");
		task_sendPressureTrend1.cancel();
	}
}

void sendPressureTrend3() {
	if (pressureTrend3.read) {
		Serial.printf(" -> Sending 3 hourly air pressure trend (%0.2f) to bus\n", pressureTrend3.value);
		(ParamAPP_Pressure_DPT == 0) ? KoAPP_PressureTrend3h_DPT9.value(pressureTrend3.value) : KoAPP_PressureTrend3h_DPT14.value(pressureTrend3.value);
		pressureTrend3.last = pressureTrend3.value;
		pressureTrend3.lastread = true;
	} else {
		Serial.println(" -- 3 hourly air pressure trend not yet available, won't send to bus - delay task");
		task_sendPressureTrend3.cancel();
	}
}

void sendRainFall() {
	if (rainFall.read) {
		Serial.printf(" -> Sending rainfall in mm (%0.2f) to bus\n", rainFall.value);
		(ParamAPP_Regen_DPT == 0) ? KoAPP_RainFall_DPT9.value(rainFall.value) : KoAPP_RainFall_DPT14.value(rainFall.value);
		rainFall.last = rainFall.value;
		rainFall.lastread = true;
	} else {
		Serial.println(" -- Rainfall not yet available, won't send to bus - delay task");
		task_sendRainFall.cancel();
	}
}

void sendRainCounter() {
	if (rainCounter.read) {
		Serial.printf(" -> Sending raincounter in mm (%0.2f) to bus\n", rainCounter.value);
		(ParamAPP_Regen_DPT == 0) ? KoAPP_RainFall_DPT9.value(rainCounter.value) : KoAPP_RainFall_DPT14.value(rainCounter.value);
		rainCounter.last = rainCounter.value;
		rainCounter.lastread = true;
	} else {
		Serial.println(" -- Rain counter not yet available, won't send to bus - delay task");
		task_sendRainCounter.cancel();
	}
}

void sendUVindex() {
	if (uvIndex.read) {
		Serial.printf(" -> Sending ultraviolet index (%0.2f) to bus\n", uvIndex.value);
		(ParamAPP_UVindex_DPT == 0) ? KoAPP_UVindex_DPT9.value(uvIndex.value) : KoAPP_UVindex_DPT14.value(uvIndex.value);
		uvIndex.last = uvIndex.value;
		uvIndex.lastread = true;
	} else {
		Serial.println(" -- Ultraviolet index not yet available, won't send to bus - delay task");
		task_sendUVindex.cancel();
	}
}

void sendBrightness() {
	if (light.read) {
		Serial.printf(" -> Sending brightness in Lux (%0.0f) to bus\n", light.value);
		(ParamAPP_Helligkeit_DPT == 0) ? KoAPP_Helligkeit_DPT9.value(light.value) : KoAPP_Helligkeit_DPT14.value(light.value);
		light.last = light.value;
		light.lastread = true;
	} else {
		Serial.println(" -- Brightness value not yet available, won't send to bus - delay task");
		task_sendBrightness.cancel();
	}
}

void sendPM25() {
	if (pm25.read) {
		Serial.printf(" -> Sending PM2.5 concentration µg/m3 (%0.2f) to bus\n", pm25.value);
		(ParamAPP_Feinstaub_DPT == 0) ? KoAPP_PM25_DPT9.value(pm25.value) : KoAPP_PM25_DPT14.value(pm25.value);
		pm25.last = pm25.value;
		pm25.lastread = true;
	} else {
		Serial.println(" -- PM2.5 concentration not yet available, won't send to bus - delay task");
		task_sendPM25.cancel();
	}
}

void sendPM10() {
	if (pm10.read) {
		Serial.printf(" -> Sending PM10 concentration µg/m3 (%0.2f) to bus\n", pm10.value);
		(ParamAPP_Feinstaub_DPT == 0) ? KoAPP_PM10_DPT9.value(pm10.value) : KoAPP_PM10_DPT14.value(pm10.value);
		pm10.last = pm10.value;
		pm10.lastread = true;
	} else {
		Serial.println(" -- PM10 concentration not yet available, won't send to bus - delay task");
		task_sendPM10.cancel();
	}
}

void sendPM25_normalized() {
	if (pm25_normalized.read) {
		Serial.printf(" -> Sending normalized PM2.5 concentration µg/m3 (%0.2f) to bus\n", pm25_normalized.value);
		(ParamAPP_Feinstaub_DPT == 0) ? KoAPP_PM25_DPT9.value(pm25_normalized.value) : KoAPP_PM25_DPT14.value(pm25_normalized.value);
		pm25_normalized.last = pm25_normalized.value;
		pm25_normalized.lastread = true;
	} else {
		Serial.println(" -- Normalized PM2.5 concentration not yet available, won't send to bus - delay task");
		task_sendPM25_normalized.cancel();
	}
}

void sendPM10_normalized() {
	if (pm10_normalized.read) {
		Serial.printf(" -> Sending normalized PM10 concentration µg/m3 (%0.2f) to bus\n", pm10_normalized.value);
		(ParamAPP_Feinstaub_DPT == 0) ? KoAPP_PM10_DPT9.value(pm10_normalized.value) : KoAPP_PM10_DPT14.value(pm10_normalized.value);
		pm10_normalized.last = pm10_normalized.value;
		pm10_normalized.lastread = true;
	} else {
		Serial.println(" -- Normalized PM10 concentration not yet available, won't send to bus - delay task");
		task_sendPM10_normalized.cancel();
	}
}
#pragma endregion

#pragma region KNX callbacks
/*
	Callbacks - these functions are receiving data from the bus, might it be data or requests
	if go.value is not defined, we are dealing with a read request, if defined it is a write received from the bus
*/
void callback_time(GroupObject& go) {	// receive time from bus
	if (go.value()) {
		timeKnown = true;
		myTime = KoAPP_Time.value();
		unsigned short tmp_sec = myTime.tm_sec;
		unsigned short tmp_min = myTime.tm_min;
		unsigned short tmp_hour = myTime.tm_hour;
		char buf[52];
		sprintf(buf, "Time received from bus: %02d:%02d:%02d", tmp_hour, tmp_min, tmp_sec );
		Serial.println(buf);
		debugV("Time received from bus: %02d:%02d:%02d", tmp_hour, tmp_min, tmp_sec );
		time_t t = now();
		setTime(tmp_hour, tmp_min, tmp_sec, day(t), month(t), year(t));
		if (dateKnown == true) {
			sprintf(buf, "Setting/Adjusting system time: %d.%d.%d, %02d:%02d:%02d", day(t), month(t), year(t), tmp_hour, tmp_min, tmp_sec );
			Serial.println(buf);
			debugV("Setting/Adjusting system time: %d.%d.%d, %02d:%02d:%02d", day(t), month(t), year(t), tmp_hour, tmp_min, tmp_sec );
		}
	}
}

void callback_date(GroupObject& go) { // receive date from bus
	if (go.value()) {
		dateKnown = true;
		myTime = KoAPP_Date.value();
		unsigned short tmp_mday = myTime.tm_mday;
		unsigned short tmp_mon = myTime.tm_mon;
		unsigned short tmp_year = myTime.tm_year;
		char buf[52];
		sprintf(buf, "Date received from bus: %d.%d.%d", tmp_mday, tmp_mon, tmp_year );
		Serial.println(buf);
		debugV("Date received from bus: %d.%d.%d", tmp_mday, tmp_mon, tmp_year );
		time_t t = now();
		setTime(hour(t), minute(t), second(t), tmp_mday, tmp_mon, tmp_year);
		if (timeKnown == true) {
			sprintf(buf, "Setting/Adjusting system time: %d.%d.%d, %02d:%02d:%02d", tmp_mday, tmp_mon, tmp_year, hour(t), minute(t), second(t) );
			Serial.println(buf);
			debugV("Setting/Adjusting system time: %d.%d.%d, %02d:%02d:%02d", tmp_mday, tmp_mon, tmp_year, hour(t), minute(t), second(t) );
		}
	}
}

void callback_dateTime(GroupObject& go) {	// receive date & time in one DPT from bus
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
		debugV("DateTime received from bus: %d.%d.%d, %02d:%02d:%02d", tmp_mday, tmp_mon, tmp_year, tmp_hour, tmp_min, tmp_sec );
		Serial.println("Setting/Adjusting system time");
		debugV("Setting/Adjusting system time");
		setTime(tmp_hour, tmp_min, tmp_sec, tmp_mday, tmp_mon, tmp_year);
	}
}

void callback_Temperature(GroupObject& go) { // read request for temperature
 if (!go.value()) sendTemperature();
}

void callback_Temperature1(GroupObject& go) { // read request for 1wire temperature
 if (!go.value()) sendTemperature1();
}

void callback_dewpoint(GroupObject& go) { // read request for dewpoint
 if (!go.value()) senddewpoint();
}

void callback_frostpoint(GroupObject& go) { // read request for frostpoint
 if (!go.value()) sendfrostpoint();
}

void callback_Humidity(GroupObject& go) { // read request for relative humidity
 if (!go.value()) sendHumidity();
}

void callback_AbsHumidity(GroupObject& go) { // read request for absolute humidity
 if (!go.value()) sendAbsHumidity();
}

void callback_WindSpeed(GroupObject& go) { // read request for windspeed
 if (!go.value()) sendWindSpeed();
}

void callback_GustSpeed(GroupObject& go) { // read request for gustspeed
 if (!go.value()) sendGustSpeed();
}

void callback_WindSpeedBft(GroupObject& go) { // read request for windspeed in beaufort
 if (!go.value()) sendWindSpeedBft();
}

void callback_GustSpeedBft(GroupObject& go) { // read request for gustspeed in beaufort
 if (!go.value()) sendGustSpeedBft();
}

void callback_WindDirection(GroupObject& go) { // read request for wind direction
 if (!go.value()) sendWindDirection();
}

void callback_Pressure(GroupObject& go) { // read request for absolute air pressure
 if (!go.value()) sendPressure();
}

void callback_PressureTrend1(GroupObject& go) { // read request for hourly air pressure trend
 if (!go.value()) sendPressureTrend1();
}

void callback_PressureTrend3(GroupObject& go) { // read request for 3 hourly air pressure trend
 if (!go.value()) sendPressureTrend3();
}

void callback_RainFall(GroupObject& go) { // read request for rainfall
 if (!go.value()) sendRainFall();
}

void callback_RainCounter(GroupObject& go) { // read request for raincounter
 if (!go.value()) sendRainCounter();
}

void callback_UVindex(GroupObject& go) { // read request for UVindex
 if (!go.value()) sendUVindex();
}

void callback_Brightness(GroupObject& go) { // read request for brightness
 if (!go.value()) sendBrightness();
}

void callback_pm25(GroupObject& go) { // read request for paritcles 2.5
 if (!go.value()) sendPM25();
}

void callback_pm10(GroupObject& go) { // read request for particles 10
 if (!go.value()) sendPM10();
}

void callback_pm25_normalized(GroupObject& go) { // read request for normalized particle value 2.5
 if (!go.value()) sendPM25_normalized();
}

void callback_pm10_normalized(GroupObject& go) { // read request for normalized particle value 10
 if (!go.value()) sendPM10_normalized();
}

#pragma endregion

#endif