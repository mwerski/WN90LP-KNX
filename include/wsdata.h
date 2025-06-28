#ifndef _wsdata_h_
#define _wsdata_h_

#pragma region Weather data structures and variables
struct wsdata {
	double value;
	bool read = false;
	double last; // value that was last sent to the bus
	bool lastread = false;
	double abs_change = 0;
	double rel_change = 0;
};
struct wsdata_integer {
	int32_t value;
	bool read = false;
	int32_t last; // value that was last sent to the bus
	bool lastread = false;
};
wsdata uvIndex; // UV index
wsdata light; // brightness in Lux
wsdata temperature; // temperature in degree celsius
wsdata temperature1; // temperature in degree celsius, measured via 1wire
wsdata humidity; // relative humidity in percent
wsdata abshumidity; // absolute humidity in grams per qubic meter
wsdata windSpeed; // windspeed in m/s
wsdata gustSpeed; // gust speed in m/s
wsdata windSpeedBFT; // windspeed in beaufort
wsdata gustSpeedBFT; // gust speed in beaufort
wsdata windDirection; // wind direction in degrees
wsdata pressure; // absolute pressure in hPa / mBar
wsdata pressureTrend1; // gets measured and updated every 15 minutes, shows pressure differences from now to -1 hour, available after ~1 hour uptime
wsdata pressureTrend3; // gets measured every full hour, shows pressure differences from now to -3 hours, available after ~3 hours uptime
wsdata rainFall; // rainfall in mm / liter - 0.1mm resolution
wsdata rainCounter; // rainfall in mm / liter - 0.01mm resolution
wsdata dewpoint; // dewpoint in degree celsius
wsdata frostpoint; // frostpoint in degree celsius
wsdata humIdex; // Humidex in !C
wsdata heatindex; // Heatindex in °C
wsdata windchill; // Windchill in °C
wsdata discomfort; // Discomfort index in °C
wsdata pm25; // PM2.5 value
wsdata pm10; // PM10 value
wsdata pm25_normalized; // humidity compensated PM2.5 value
wsdata pm10_normalized; // humidity compensated PM10 value
/*
Es existieren 2 Ringbuffer mit jeweils 12 Elementen zur Speicherung der rainfall sowie raincounter Werte im 5 Minuten Takt (Minutenarray oder MA)
Desweiteren existieren 2 Ringbuffer mit jeweils 72 Elementen in denen rainfall und raincounter jeweils stuendlich gespeichert werden (Stundenarray oder SA)
Zusaetzlich existieren 2 T-Werte, die jeweils um Mitternacht auf 0 gesetzt werden. Die aktuelle Regenmenge abzueglich dieses Wertes ergibt die Tages-Regenmenge.
*/
ringBuffer<u_int16_t> rainfallSA(73), raincounterSA(73), rainfallMA(13), raincounterMA(13);
u_int16_t rainfallT, raincounterT;
ringBuffer<u_int16_t> pressureRing(13); // gets updated every 15 minutes, holds pressure data for 3 hours - needed for calculating pressure trends

double normalizePM25(double pm25, double humidity) {
	double p = pm25/(1.0+0.48756*pow((humidity/100.0), 8.60068));
	double m = powf( 10.0f, 2 ); // truncate to x.yz
	p = roundf( p * m ) / m;
	return p;
}
double normalizePM10(double pm10, double humidity) {
	double p = pm10/(1.0+0.81559*pow((humidity/100.0), 5.83411));
	double m = powf( 10.0f, 2 ); // truncate to x.yz
	p = roundf( p * m ) / m;
	return p;
}
bool abs_change(wsdata m) {
	if ( m.abs_change != 0 && m.lastread != 0 ) {
		if ( abs( m.last - m.value) >= m.abs_change ) return true;
	}
	return false;
}
bool rel_change(wsdata m) {
	if ( m.rel_change != 0 && m.lastread != 0 && m.last != 0 ) {
		if ( abs( (m.value - m.last) / m.last * 100) >= m.rel_change ) return true;
	}
	return false;
}
double get_abschange(wsdata m) {
	if (m.lastread) return abs(m.value - m.last);
	return NAN;
}
double get_relchange(wsdata m) {
	if (m.lastread && m.last != 0) return abs((m.value - m.last) / m.last * 100);
	return NAN;
}
double frostPoint(float t, float f) {
	// calculates frostpoint from given temperature and relative hjmidity
	float a, b;
  if (t >= 0) {
    a = 7.5;
    b = 237.3;
  } else {
    a = 9.5;
    b = 266.5;
  }
  double sdd = 6.1078 * pow(10, (a*t)/(b+t));  // Sättigungsdampfdruck in hPa
  double dd = sdd * (f/100);  // Dampfdruck in mBar
  double v = log10(dd/6.1078);  // v-Parameter
  double p = (b*v) / (a-v);  // Frostpunkttemperatur (°C)
//	return dp;
	double m = powf( 10.0f, 2 ); // truncate to x.yz
	p = roundf( p * m ) / m;
	return p;
}
/* double dewpoint(float t, float f) {
	// calculates dewpoint from given temperature and relative hjmidity
	float a, b;
  if (t >= 0) {
    a = 7.5;
    b = 237.3;
  } else {
    a = 7.6;
    b = 240.7;
  }
  double sdd = 6.1078 * pow(10, (a*t)/(b+t));  // Sättigungsdampfdruck in hPa
  double dd = sdd * (f/100);  // Dampfdruck in mBar
  double v = log10(dd/6.1078);  // v-Parameter
  double p = (b*v) / (a-v);  // Taupunkttemperatur (°C)
//	return p;
	double m = powf( 10.0f, 2 ); // truncate to x.yz
	p = roundf( p * m ) / m;
	return p;
}
	*/


	/*
https://github.com/RobTillaart/Temperature?tab=readme-ov-file


Windchill (in °C) = 13.12 + 0.6215 * Temperatur(°C) - 11.37 * Windgeschwindigkeit(km/h)^0.16 + 0.3965 * Temperatur(°C) * Windgeschwindigkeit(km/h)^0.16
float Windchill  = 35.74 + 0.6215*T - 35.75*pow(V,0.16) + 0.4275*T*pow(V,0.16);


// Funktion zur Berechnung des Hitzeindex (THSW)
float heatIndex(float temperature, float humidity) {
  float hic = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (humidity * 0.094));
  if (hic > 79) {
    hic = -42.379 + 2.04901523 * temperature + 10.14333127 * humidity - 0.22475541 * temperature * humidity - 0.00683783 * pow(temperature, 2) - 0.05481717 * pow(humidity, 2) + 0.00122874 * pow(temperature, 2) * humidity + 0.00085282 * temperature * pow(humidity, 2) - 0.00000199 * pow(temperature, 2) * pow(humidity, 2);
  }
  return hic;
}

*/
u_int8_t bft(float s) {
	if ( s >= 32.7 ) { return 12; } else
	if ( s >= 28.5 ) { return 11; } else
	if ( s >= 24.5 ) { return 10; } else
	if ( s >= 20.8 ) { return 9; } else
	if ( s >= 17.2 ) { return 8; } else
	if ( s >= 13.9 ) { return 7; } else
	if ( s >= 10.8 ) { return 6; } else
	if ( s >= 8.0 ) { return 5; } else
	if ( s >= 5.5 ) { return 4; } else
	if ( s >= 3.4 ) { return 3; } else
	if ( s >= 1.6 ) { return 2; } else
	if ( s >= 0.3 ) { return 1; }
	return 0;
}
#pragma endregion

#endif