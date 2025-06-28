#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Adafruit_NeoPixel.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include <Json.h>
#include <knx.h>
#include <RemoteDebug.h>
#include <TimeLib.h>
#include <uptime_formatter.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "NovaSDS011.h"
#include <SoftwareSerial.h>

struct tm myTime;
bool timeKnown = false;
bool dateKnown = false;
bool sensorfailure_wn90 = false;
bool sensorfailure_1wire = false;
bool sensorfailure_sds = false;
bool ow_awailable = false;
bool sds_available = false;

#include "utilities.h"
#include <knxprod.h>
#include <wsdata.h>>
#include <tasks.h>
#include <knx_send.h>


#define WDT_TIMEOUT 10 // Task Watchdog Timeout
#define DEBUG_DISABLE_DEBUGGER true	// Debug Optionen in SerialDebug deaktivieren
#define DEBUG_INITIAL_LEVEL DEBUG_LEVEL_VERBOSE	// Default Debug Level
#define SDS_PIN_RX 35
#define SDS_PIN_TX 38

RemoteDebug Debug;

OneWire ow(WIREPIN);
DallasTemperature ds(&ow);
DeviceAddress sensor;




u_int8_t lasthour = NAN;	// last hour (if this changes, a full hour has passed)
u_int8_t lastminute = NAN; // last minute

WebServer server(80);

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Modbus
ModbusMaster node;
bool RS485mode = true;
void RS485_Mode(int Mode);
void RS485_TX();
void RS485_RX();

struct netconfig {
	IPAddress ip;
	IPAddress gateway;
	IPAddress netmask;
	IPAddress dns;
	bool dhcp = true;
	char*	hostname = "wn90";
	bool mqtt = false;
	char* mqttHost;
	u_int16_t mqttPort = 1883;
	char* mqttUser;
	char* mqttPass;
	char* mqttTopic;
	u_int16_t mqttFreq;
	bool remoteDebug = true;
	bool serialDebug = true;
};
netconfig net;


#pragma region MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Json mqttMsg;
void mqtt_reconnect() {
	Serial.print("Attempting MQTT connection...");
	// Attempt to connect
	String client_id = net.hostname; //  + String(WiFi.macAddress());
	if (mqttClient.connect(client_id.c_str(), net.mqttUser, net.mqttPass)) {
		Serial.println("connected");
	} else {
		Serial.print("failed, rc=");
		Serial.print(mqttClient.state());
		Serial.println(" try again next time");
	}
}
#pragma endregion



void progLedOff() {
	pixels.clear();
	pixels.show();
}

void progLedOn() {
	pixels.setPixelColor(0, pixels.Color(20, 0, 0));
	pixels.show();
}

#pragma region Particle sensor
NovaSDS011 sds011;

void read_SDS() {
	Serial.println("TASK: Reading SDS011");
	double pm25r, pm10r;
  if (sds011.queryData(pm25r, pm10r) == QueryError::no_error) {
		Serial.println(F("Reading available, begin handling SDS011 query data"));
		Serial.print(F("PM10: "));
		Serial.println(float(pm10r));
		Serial.print(F("PM2.5: "));
		Serial.println(float(pm25r));
		pm25.value = pm25r;
		pm25.read = true;
		if ( task_sendPM25.canceled() ) task_sendPM25.enableDelayed(TASK_DELAY);
		if ( abs_change(pm25)) {
			Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): \n",get_abschange(pm25), pm25.abs_change);
			sendPM25();
		} else if ( rel_change(pm25)) {
			Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): \n",get_relchange(pm25), pm25.rel_change);
			sendPM25();
		}
		pm10.value = pm10r;
		pm10.read = true;
		if ( task_sendPM10.canceled() ) task_sendPM10.enableDelayed(TASK_DELAY);
		if ( abs_change(pm10)) {
			Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): \n",get_abschange(pm10), pm10.abs_change);
			sendPM10();
		} else if ( rel_change(pm10)) {
			Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): \n",get_relchange(pm10), pm10.rel_change);
			sendPM10();
		}
		if (humidity.read) {
			pm25_normalized.value = normalizePM25(pm25.value, humidity.value);
			pm25_normalized.read = true;
			if ( task_sendPM25_normalized.canceled() ) task_sendPM25_normalized.enableDelayed(TASK_DELAY);
			if ( abs_change(pm25_normalized)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): \n",get_abschange(pm25_normalized), pm25_normalized.abs_change);
				sendPM25_normalized();
			} else if ( rel_change(pm25_normalized)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): \n",get_relchange(pm25_normalized), pm25_normalized.rel_change);
				sendPM25_normalized();
			}
			pm10_normalized.value = normalizePM10(pm10.value, humidity.value);
			pm10_normalized.read = true;
			if ( task_sendPM10_normalized.canceled() ) task_sendPM10_normalized.enableDelayed(TASK_DELAY);
			if ( abs_change(pm10_normalized)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): \n",get_abschange(pm10_normalized), pm10_normalized.abs_change);
				sendPM10_normalized();
			} else if ( rel_change(pm10_normalized)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): \n",get_relchange(pm10_normalized), pm10_normalized.rel_change);
				sendPM10_normalized();
			}
		}
		Serial.println(F("End Handling SDS011 query data"));
	} else {
		Serial.println("ERROR: could not fetch SDS011 readings");
	}
}

#pragma endregion

unsigned long start_ms=0;
#pragma region setup
void setup() {
	Serial.begin(115200);
	delay (2000);
	pinMode(KNXPIN, INPUT);

	knx.buttonPin(KNXPIN);	// PROG button on GPIO5
	knx.readMemory();	// read adress table, association table, groupobject table and parameters from eeprom
	
	#pragma region WiFiManager
  WiFiManager wifiManager;
	Serial.println(digitalRead(KNXPIN));

	if ( !digitalRead(KNXPIN) ) {	// reset WiFi Settings if PROG-Button is pressed during boot
		Serial.println("Reset WiFi settings...");
		wifiManager.resetSettings();
	}

	if ( knx.configured() && ParamAPP_UseDHCP == false ) {	// use DHCP if KNX is not configured
		// we are using a static IP config stored as KNX paramaeters
		net.dhcp = false;
		// convert IPs from little to big endian
		net.ip = IPAddress((ParamAPP_IP & 0xFF) << 24) | ((ParamAPP_IP & 0xFF00) << 8) | ((ParamAPP_IP & 0xFF0000) >> 8) | ((ParamAPP_IP & 0xFF000000) >> 24);
		net.netmask = IPAddress((ParamAPP_Netzmaske & 0xFF) << 24) | ((ParamAPP_Netzmaske & 0xFF00) << 8) | ((ParamAPP_Netzmaske & 0xFF0000) >> 8) | ((ParamAPP_Netzmaske & 0xFF000000) >> 24);
		net.gateway = IPAddress((ParamAPP_Gateway & 0xFF) << 24) | ((ParamAPP_Gateway & 0xFF00) << 8) | ((ParamAPP_Gateway & 0xFF0000) >> 8) | ((ParamAPP_Gateway & 0xFF000000) >> 24);
		net.dns = IPAddress((ParamAPP_DNS & 0xFF) << 24) | ((ParamAPP_DNS & 0xFF00) << 8) | ((ParamAPP_DNS & 0xFF0000) >> 8) | ((ParamAPP_DNS & 0xFF000000) >> 24);
		Serial.println("Static network configuration:");
		Serial.print("IP....... "); Serial.println(net.ip);
		Serial.print("Netmask.. "); Serial.println(net.netmask);
		Serial.print("Gateway.. "); Serial.println(net.gateway);
		Serial.print("DNS...... "); Serial.println(net.dns);
	} else {
		net.dhcp = true;
	}

	if ( net.dhcp == false ) {
		wifiManager.setSTAStaticIPConfig( net.ip, net.gateway, net.netmask, net.dns );
		Serial.print("static setup: ");
	}

	wifiManager.setHostname( net.hostname );
  wifiManager.setConfigPortalTimeout(180);
	if (!wifiManager.autoConnect("AutoConnectAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  } 

	Serial.print("IP Address: ");
	Serial.println(WiFi.localIP());
	#pragma endregion

	#pragma region RemoteDebug
	if ( ( knx.configured() && ParamAPP_RemoteDebug ) || !knx.configured() ) {
		net.remoteDebug = true;
		Debug.begin(net.hostname); // Initialize the WiFi server
		Debug.setResetCmdEnabled(true); // Enable the reset command
		Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
		Debug.showColors(true); // Colors
//		if ( ( knx.configured() && ParamApp_SerialDebug ) || !knx.configured() ) {
//			Debug.setSerialEnabled(true);
//		}
	}
	#pragma endregion

	pinMode(RS485_CON_PIN, OUTPUT);
	pinMode(KEY_PIN, INPUT_PULLUP);
	Serial485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN, true);
	// communicate with Modbus slave ID 144 over Serial (port 0)
  node.preTransmission(RS485_TX);
  node.postTransmission(RS485_RX);
  node.begin(0x90, Serial485);
	RS485_Mode(RS485_TX_ENABLE);
	delay(20); 



	Serial.println("Starting up...");
	Serial.print("KNX configured: ");
	Serial.println(knx.configured());
	knx.setProgLedOffCallback(progLedOff);
	knx.setProgLedOnCallback(progLedOn);

	Serial.println("Initializing scheduler");
	runner.init();


	if (knx.configured()) {


		net.hostname = (char *) ParamAPP_Hostname;
		net.mqtt = ParamAPP_useMQTT;
		if ( net.mqtt == true ) {
			Serial.println("MQTT configured");
			// convert data paraeters to char arrays
			net.mqttHost = (char *) ParamAPP_MQTT_Host;
			net.mqttUser = (char *) ParamAPP_MQTT_User;
			net.mqttPass = (char *) ParamAPP_MQTT_Password;
			net.mqttTopic = (char *) ParamAPP_MQTT_Topic;
			net.mqttPort = ParamAPP_MQTT_Port;
			net.mqttFreq = ParamAPP_MQTT_Frequency;
			Serial.print("Broker..... "); Serial.print(net.mqttHost); Serial.print(":"); Serial.println(net.mqttPort);
			Serial.print("User....... "); Serial.println(net.mqttUser);
			Serial.print("Topic...... "); Serial.println(net.mqttTopic);
			Serial.print("Frequency.. "); Serial.println(net.mqttFreq);
		} else {
			Serial.println("MQTT disabled");
		}

		if (ParamAPP_Heartbeat > 0) {
			Serial.print("Send heartbeat every "); Serial.print(ParamAPP_Heartbeat); Serial.println("s");
			// ParamAPP_Heartbeat=10:
			KoAPP_Heartbeat.dataPointType(Dpt(1, 1));
			runner.addTask(task_heartbeat);
			task_heartbeat.setInterval(ParamAPP_Heartbeat*1000);
			task_heartbeat.enable();
			Serial.println("Task enabled");
		} else {
			Serial.println("Send no heartbeat");
		}

		if (ParamAPP_Feinstaubsensor_vorhanden) {
			Serial.println("Particle sensor configured, locating device");
			sds011.begin(SDS_PIN_RX, SDS_PIN_TX);
			SDS011Version sds_version = sds011.getVersionDate();
			if (sds_version.valid) {
				Serial.println("SDS011 Firmware Vesion: " + String(sds_version.year) + "-" + String(sds_version.month) + "-" + String(sds_version.day));
				sds_available = true;
				Serial.println("Setting duty cacle to 2 minutes");
				sds011.setDutyCycle(2);
				Serial.println("Registering task");
				runner.addTask(task_readSDS);
				task_readSDS.enable();
			} else {
				Serial.println("ERROR: Sensor not found, disabling SDS011 routines!");
			}
		}
		

		if (ParamAPP_1wire_vorhanden) {
			Serial.println("1wire configured, locating sensor");
			ds.begin();
			u_int8_t ow_count = ds.getDeviceCount();
			if (ow_count == 0) {
				Serial.println("ERROR: no sensor found, disabling 1wire!");
			} else {
				if (ow_count > 1) {
					Serial.println("WARNING: more than one sensor found, will only query the first one!");
				} else {
					Serial.println("1wire sensor found");
				}
				if (ds.isParasitePowerMode()) {
					Serial.println("WARNING: parasite power is ON, it is recommended not to use parasite power mode!");
				} else {
					Serial.println("Parasite power mode is off");
				}
				if (!ds.getAddress(sensor, 0)) {
					Serial.println("ERROR: unable to find address for sensor 0");
				} else {
					ds.setResolution(sensor, 12);
					Serial.print("Sensor address is ");
					for (uint8_t i = 0; i < 8; i++) {
						if (sensor[i] < 16) Serial.print("0");
						Serial.print(sensor[i], HEX);
					}
				}
				Serial.print(", sensor resolution is ");
				Serial.println(ds.getResolution(sensor), DEC);
				ow_awailable = true;
				Serial.println("Enabling 1wire task");
				runner.addTask(task_read1wTemperature);
				runner.addTask(task_request1wTemperature);
				task_request1wTemperature.enable();
			}
		} else {
			Serial.println("1wire disabled");
		}

		Serial.println("Defining DPTs and callbacks");
		switch ( ParamAPP_Temperatur_DPT ) {
			case 0: 
				KoAPP_Temperatur_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_Temperatur_DPT9.callback(callback_Temperature);
				if (ParamAPP_1wire_vorhanden) {
					KoAPP_Temperatur_1wire_DPT9.dataPointType(Dpt(9, 1));
					KoAPP_Temperatur_1wire_DPT9.callback(callback_Temperature1);
				}
				KoAPP_Taupunkt_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_Taupunkt_DPT9.callback(callback_dewpoint);
				KoAPP_Frostpunkt_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_Frostpunkt_DPT9.callback(callback_frostpoint);
				break;
			case 1:
				KoAPP_Temperatur_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_Temperatur_DPT14.callback(callback_Temperature);
				if (ParamAPP_1wire_vorhanden) {
					KoAPP_Temperatur_1wire_DPT14.dataPointType(Dpt(14, 1));
					KoAPP_Temperatur_1wire_DPT14.callback(callback_Temperature1);
				}
				KoAPP_Taupunkt_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_Taupunkt_DPT14.callback(callback_dewpoint);
				KoAPP_Frostpunkt_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_Frostpunkt_DPT14.callback(callback_frostpoint);
				break;
		}

		switch ( ParamAPP_Feuchte_DPT ) {
			case 0: 
				KoAPP_Feuchte_DPT6.dataPointType(Dpt(6, 1));
				KoAPP_Feuchte_DPT6.callback(callback_Humidity);
				break;
			case 1: 
				KoAPP_Feuchte_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_Feuchte_DPT9.callback(callback_Humidity);
				break;
			case 2:
				KoAPP_Feuchte_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_Feuchte_DPT14.callback(callback_Humidity);
				break;
		}

		switch ( ParamAPP_WindSpeed_DPT ) {
			case 0: 
				KoAPP_WindSpeed_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_WindSpeed_DPT9.callback(callback_WindSpeed);
				KoAPP_GustSpeed_DPT9.dataPointType(Dpt(14, 1));
				KoAPP_GustSpeed_DPT9.callback(callback_GustSpeed);
				break;
			case 1:
				KoAPP_WindSpeed_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_WindSpeed_DPT14.callback(callback_WindSpeed);
				KoAPP_GustSpeed_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_GustSpeed_DPT14.callback(callback_GustSpeed);
				break;
		}

		switch ( ParamAPP_WindDir_DPT ) {
			case 0: 
				KoAPP_WindDir_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_WindDir_DPT9.callback(callback_WindDirection);
				break;
			case 1:
				KoAPP_WindDir_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_WindDir_DPT14.callback(callback_WindDirection);
				break;
		}

		switch ( ParamAPP_Pressure_DPT ) {
			case 0: 
				KoAPP_Pressure_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_Pressure_DPT9.callback(callback_Pressure);
				KoAPP_PressureTrend1h_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_PressureTrend1h_DPT9.callback(callback_PressureTrend1);
				KoAPP_PressureTrend3h_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_PressureTrend3h_DPT9.callback(callback_PressureTrend3);
				break;
			case 1:
				KoAPP_Pressure_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_Pressure_DPT14.callback(callback_Pressure);
				KoAPP_PressureTrend1h_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_PressureTrend1h_DPT14.callback(callback_PressureTrend1);
				KoAPP_PressureTrend3h_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_PressureTrend3h_DPT14.callback(callback_PressureTrend3);
				break;
		}

		switch ( ParamAPP_Regen_DPT ) {
			case 0: 
				KoAPP_RainFall_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_RainFall_DPT9.callback(callback_RainFall);
				KoAPP_RainCounter_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_RainCounter_DPT9.callback(callback_RainCounter);
				break;
			case 1:
				KoAPP_RainFall_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_RainFall_DPT14.callback(callback_RainFall);
				KoAPP_RainCounter_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_RainCounter_DPT14.callback(callback_RainCounter);
				break;
		}

		switch ( ParamAPP_Feinstaub_DPT ) {
			case 0: 
				KoAPP_PM25_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_PM25_DPT9.callback(callback_pm25);
				KoAPP_PM10_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_PM10_DPT9.callback(callback_pm10);
				KoAPP_PM25_Normalized_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_PM25_Normalized_DPT9.callback(callback_pm25_normalized);
				KoAPP_PM10_Normalized_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_PM10_Normalized_DPT9.callback(callback_pm10_normalized);
				break;
			case 1:
				KoAPP_PM25_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_PM25_DPT14.callback(callback_pm25);
				KoAPP_PM10_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_PM10_DPT14.callback(callback_pm10);
				KoAPP_PM25_Normalized_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_PM25_Normalized_DPT14.callback(callback_pm25_normalized);
				KoAPP_PM10_Normalized_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_PM10_Normalized_DPT14.callback(callback_pm10_normalized);
				break;
		}

		switch ( ParamAPP_Helligkeit_DPT ) {
			case 0: 
				KoAPP_Helligkeit_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_Helligkeit_DPT9.callback(callback_Brightness);
				break;
			case 1:
				KoAPP_Helligkeit_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_Helligkeit_DPT14.callback(callback_Brightness);
				break;
		}

		switch ( ParamAPP_UVindex_DPT ) {
			case 0: 
				KoAPP_UVindex_DPT9.dataPointType(Dpt(9, 1));
				KoAPP_UVindex_DPT9.callback(callback_UVindex);
				break;
			case 1:
				KoAPP_UVindex_DPT14.dataPointType(Dpt(14, 1));
				KoAPP_UVindex_DPT14.callback(callback_UVindex);
				break;
		}

		Serial.println("Define thresholds for value changes");
		light.abs_change = ParamAPP_Helligkeit_Senden_Wertaenderung_absolut;
		light.rel_change = ParamAPP_Helligkeit_Senden_Wertaenderung_relativ;
		uvIndex.abs_change = ParamAPP_UVindex_Senden_Wertaenderung_absolut;
		uvIndex.rel_change = ParamAPP_UVindex_Senden_Wertaenderung_relativ;
		temperature.abs_change = ParamAPP_Temperatur_Senden_Wertaenderung_absolut;
		temperature.rel_change = ParamAPP_Temperatur_Senden_Wertaenderung_relativ;
		temperature1.abs_change = ParamAPP_Temperatur_Senden_Wertaenderung_absolut;
		temperature1.rel_change = ParamAPP_Temperatur_Senden_Wertaenderung_relativ;
		dewpoint.abs_change = ParamAPP_Temperatur_Senden_Wertaenderung_absolut;
		dewpoint.rel_change = ParamAPP_Temperatur_Senden_Wertaenderung_relativ;
		humidity.abs_change = ParamAPP_Feuchte_Senden_Wertaenderung_absolut;
		humidity.rel_change = ParamAPP_Feuchte_Senden_Wertaenderung_relativ;
		windSpeed.abs_change = ParamAPP_WindSpeed_Senden_Wertaenderung_absolut;
		windSpeed.rel_change = ParamAPP_WindSpeed_Senden_Wertaenderung_relativ;
		gustSpeed.abs_change = ParamAPP_WindSpeed_Senden_Wertaenderung_absolut;
		gustSpeed.rel_change = ParamAPP_WindSpeed_Senden_Wertaenderung_relativ;
		windDirection.abs_change = ParamAPP_WindDir_Senden_Wertaenderung_absolut;
		windDirection.rel_change = ParamAPP_WindDir_Senden_Wertaenderung_relativ;
		pressure.abs_change = ParamAPP_Pressure_Senden_Wertaenderung_absolut;
		pressure.rel_change = ParamAPP_Pressure_Senden_Wertaenderung_relativ;
		rainFall.abs_change = ParamAPP_Regen_Senden_Wertaenderung_absolut;
		rainFall.rel_change = ParamAPP_Regen_Senden_Wertaenderung_relativ;
		rainCounter.abs_change = ParamAPP_Regen_Senden_Wertaenderung_absolut;
		rainCounter.rel_change = ParamAPP_Regen_Senden_Wertaenderung_relativ;
		pm25.abs_change = ParamAPP_Feinstaub_Senden_Wertaenderung_absolut;
		pm25.rel_change = ParamAPP_Feinstaub_Senden_Wertaenderung_relativ;
		pm10.abs_change = ParamAPP_Feinstaub_Senden_Wertaenderung_absolut;
		pm10.rel_change = ParamAPP_Feinstaub_Senden_Wertaenderung_relativ;
		pm25_normalized.abs_change = ParamAPP_Feinstaub_Senden_Wertaenderung_absolut;
		pm25_normalized.rel_change = ParamAPP_Feinstaub_Senden_Wertaenderung_relativ;
		pm10_normalized.abs_change = ParamAPP_Feinstaub_Senden_Wertaenderung_absolut;
		pm10_normalized.rel_change = ParamAPP_Feinstaub_Senden_Wertaenderung_relativ;

	}


	// Arduino OTA on üport 3232
	// ArduinoOTA.setPort(3232);
	ArduinoOTA.setHostname(net.hostname);
	// ArduinoOTA.setPassword("admin");
	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
	ArduinoOTA
		.onStart([]() {
			String type;
			esp_task_wdt_deinit();
			if (ArduinoOTA.getCommand() == U_FLASH) {
				type = "sketch";
			} else {  // U_SPIFFS
				type = "filesystem";
			}

			// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
			Serial.println("Start updating " + type);
		})
		.onEnd([]() {
			Serial.println("\nEnd");
		})
		.onProgress([](unsigned int progress, unsigned int total) {
			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		})
		.onError([](ota_error_t error) {
			Serial.printf("Error[%u]: ", error);
			if (error == OTA_AUTH_ERROR) {
				Serial.println("Auth Failed");
			} else if (error == OTA_BEGIN_ERROR) {
				Serial.println("Begin Failed");
			} else if (error == OTA_CONNECT_ERROR) {
				Serial.println("Connect Failed");
			} else if (error == OTA_RECEIVE_ERROR) {
				Serial.println("Receive Failed");
			} else if (error == OTA_END_ERROR) {
				Serial.println("End Failed");
			}
		});
	ArduinoOTA.begin();


/*		// Initialize mDNS
		if (!MDNS.begin("wl90")) {   // Set the hostname to "wl90.local"
			Serial.println("Error setting up MDNS responder!");
			while(1) {
				delay(1000);
			}
		}
		Serial.println("mDNS responder started");
*/



	server.begin();

	ElegantOTA.onStart([]() {
		esp_task_wdt_deinit();
	});
	ElegantOTA.begin(&server);

	// MQTT
	mqttClient.setServer(net.mqttHost, net.mqttPort);
	

	

	// initialize ringbuffer
	//	for (u_int8_t i=0; i<RINGBUFFERSIZE; i++) { pressureRing[i] = NAN; }

	if ( net.mqtt == true ) {
		runner.addTask(task_MQTTpublish);
		task_MQTTpublish.setInterval(net.mqttFreq*1000);
		task_MQTTpublish.enable();
	}

	if (knx.configured()) {
		setup_tasks();		


		//	runner.addTask(task_updatePressureRingbuffer);

		// define callbacks
	/*	if (ParamAPP_Temperatur_DPT == 0) {
			// DPT9
			KoAPP_Temperatur_DPT9.callback(callback_sendTemperature);
		} else {
			// DPT14
			KoAPP_Temperatur_DPT14.callback(sendTemperature));
		} */
	}

	knx.start();
	#pragma region KNX Request Time

	if (knx.configured()) {
		if (ParamAPP_DateTime_DPTs == 1) {
			Serial.println("Receive time and date from different KOs, registering callbacks");
			KoAPP_Time.dataPointType(DPT_TimeOfDay);
			KoAPP_Time.callback(callback_time);
			KoAPP_Date.dataPointType(DPT_Date);
			KoAPP_Date.callback(callback_date);
			if (ParamAPP_Uhrzeit_beim_Start_lesen == 1) {
				Serial.println("Reading time and date from Bus");
				KoAPP_Time.requestObjectRead();
				KoAPP_Date.requestObjectRead();
			}
		} else {
			Serial.println("Receive time and date from a single KO, registering callback");
			KoAPP_DateTime.dataPointType(DPT_DateTime);
			KoAPP_DateTime.callback(callback_dateTime);
			if (ParamAPP_Uhrzeit_beim_Start_lesen == 1) {
				Serial.println("Reading time and date from Bus");
				KoAPP_DateTime.requestObjectRead();
			}
		}
		#pragma endregion
	}

	if ( ParamAPP_Watchdog ) {
		Serial.printf("Enable hardware watchdog (%ds)\n", WDT_TIMEOUT);
		esp_task_wdt_init(WDT_TIMEOUT,true);
		esp_task_wdt_add(NULL); 
	} else {
		Serial.println("Deactivate hardware watchdog");
	}

}
#pragma endregion

void printLocaltime(bool newline=false) {
	time_t t = now();
	char buf[20];
	sprintf(buf, "%d.%d.%d, %02d:%02d:%02d", day(t), month(t), year(t), hour(t), minute(t), second(t));
	Serial.print(buf);
	if (newline == true) Serial.println();
}

void read_wn90() {
//void read_wn90(NonBlockingModbusMaster &node) {
	Serial.println("------------------------------------------------------------");
	Serial.println("Process weather station data");
	debugV("Process weather station data");
	uint8_t c=10;
	uint8_t result = node.readHoldingRegisters( 0x165, c );

//	int err = node.getError(); // 0 for OK
//	debugV("ERR: %u", err);
	if (result == node.ku8MBSuccess) {
	//if (!err) {
//		debugV("no error");
		sensorfailure_sds = false;
		Serial.println("WDT timer reset");
		Serial.print ("Uptime......... "); Serial.println( uptime_formatter::getUptime() );
		Serial.print ("Localtime...... "); printLocaltime(true);
		Serial.print ("AppVersion..... "); Serial.println( MAIN_ApplicationVersion );

		if ( node.getResponseBuffer(0) != 0xFFFF ) { // BRIGHTNESS
			light.value = node.getResponseBuffer(0) * 10;
			light.read = true;
			Serial.printf("Brightness..... %0.1f Lux (%0.1f: ∆%0.1f, ∆%0.1f%%)", light.value, light.last, get_abschange(light), get_relchange(light));
			if ( task_sendBrightness.canceled() ) task_sendBrightness.enableDelayed(TASK_DELAY);
			if ( abs_change(light)) {
				Serial.printf(" - value change (%0.1f) exceeded absolute threshold (%0.1f): ",get_abschange(light), light.abs_change);
				sendBrightness();
			} else if ( rel_change(light)) {
				Serial.printf(" - value change (%0.1f) exceeded relative threshold (%0.1f): ",get_relchange(light), light.rel_change);
				sendBrightness();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(1) != 0xFFFF ) { // UVINDEX
			uvIndex.value = node.getResponseBuffer(1) / 10.0;
			uvIndex.read = true;
			Serial.printf("UV Index....... %0.1f (%0.1f: ∆%0.1f, ∆%0.1f%%)", uvIndex.value, uvIndex.last, get_abschange(uvIndex), get_relchange(uvIndex));
			if ( task_sendUVindex.canceled() ) task_sendUVindex.enableDelayed(TASK_DELAY);
			if ( abs_change(uvIndex)) {
				Serial.printf(" - value change (%0.1f) exceeded absolute threshold (%0.1f): ",get_abschange(uvIndex), uvIndex.abs_change);
				sendUVindex();
			} else if ( rel_change(uvIndex)) {
				Serial.printf(" - value change (%0.1f) exceeded relative threshold (%0.1f): ",get_relchange(uvIndex), uvIndex.rel_change);
				sendUVindex();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(2) != 0xFFFF ) { // TEMPERATURE
			temperature.value = node.getResponseBuffer(2) / 10.0 - 40;
			temperature.read = true;
			Serial.printf("Temperature.... %0.1f °C (%0.1f: ∆%0.1f, ∆%0.1f%%)", temperature.value, temperature.last, get_abschange(temperature), get_relchange(temperature));
			if ( task_sendTemperature.canceled() ) task_sendTemperature.enableDelayed(TASK_DELAY);
			if ( abs_change(temperature)) {
				Serial.printf(" - value change (%0.1f) exceeded absolute threshold (%0.1f): ",get_abschange(temperature), temperature.abs_change);
				sendTemperature();
			} else if ( rel_change(temperature)) {
				Serial.printf(" - value change (%0.1f) exceeded relative threshold (%0.1f): ",get_relchange(temperature), temperature.rel_change);
				sendTemperature();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(3) != 0xFFFF ) { // HUMIDITY
			humidity.value = node.getResponseBuffer(3); 
			humidity.read = true; 
			Serial.printf("Humidity....... %0.0f %% (%0.0f: ∆%0.0f, ∆%0.0f%%)", humidity.value, humidity.last, get_abschange(humidity), get_relchange(humidity));
			if ( task_sendHumidity.canceled() ) task_sendHumidity.enableDelayed(TASK_DELAY);
			if ( abs_change(humidity)) {
				Serial.printf(" - value change (%0.0f) exceeded absolute threshold (%0.0f): ",get_abschange(humidity), humidity.abs_change);
				sendHumidity();
				sendAbsHumidity();
			} else if ( rel_change(humidity)) {
				Serial.printf(" - value change (%0.0f) exceeded relative threshold (%0.0f): ",get_relchange(humidity), humidity.rel_change);
				sendHumidity();
				sendAbsHumidity();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(4) != 0xFFFF ) { // WINDSPEED
			windSpeed.value = node.getResponseBuffer(4) / 10.0;
			windSpeed.read = true;
			windSpeedBFT.value = bft(windSpeed.value);
			windSpeedBFT.read = true;
			Serial.printf("Wind Speed..... %0.2f m/s (%0.2f: ∆%0.2f, ∆%0.2f%%)", windSpeed.value, windSpeed.last, get_abschange(windSpeed), get_relchange(windSpeed));
			if ( task_sendWindSpeed.canceled() ) task_sendWindSpeed.enableDelayed(TASK_DELAY);
			if ( abs_change(windSpeed)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): ",get_abschange(windSpeed), windSpeed.abs_change);
				sendWindSpeed();
			} else if ( rel_change(windSpeed)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): ",get_relchange(windSpeed), windSpeed.rel_change);
				sendWindSpeed();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(5) != 0xFFFF ) { // GUSTSPEED
			gustSpeed.value = node.getResponseBuffer(4) / 10.0;
			gustSpeed.read = true;
			gustSpeedBFT.value = bft(gustSpeed.value);
			gustSpeedBFT.read = true;
			Serial.printf("Gust Speed..... %0.2f m/s (%0.2f: ∆%0.2f, ∆%0.2f%%)", gustSpeed.value, gustSpeed.last, get_abschange(gustSpeed), get_relchange(gustSpeed));
			if ( task_sendGustSpeed.canceled() ) task_sendGustSpeed.enableDelayed(TASK_DELAY);
			if ( abs_change(gustSpeed)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): ",get_abschange(gustSpeed), gustSpeed.abs_change);
				sendGustSpeed();
			} else if ( rel_change(gustSpeed)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): ",get_relchange(gustSpeed), gustSpeed.rel_change);
				sendGustSpeed();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(6) != 0xFFFF ) { // WINDDIRECTION
			windDirection.value = node.getResponseBuffer(6);
			windDirection.read = true;
			Serial.printf("Wind Direction. %0.0f ° (%0.0f: ∆%0.0f, ∆%0.0f%%)", windDirection.value, windDirection.last, get_abschange(windDirection), get_relchange(windDirection));
			if ( task_sendWindDirection.canceled() ) task_sendWindDirection.enableDelayed(TASK_DELAY);
			if ( abs_change(windDirection)) {
				Serial.printf(" - value change (%0.0f) exceeded absolute threshold (%0.0f): ",get_abschange(windDirection), windDirection.abs_change);
				sendWindDirection();
			} else if ( rel_change(windDirection)) {
				Serial.printf(" - value change (%0.0f) exceeded relative threshold (%0.0f): ",get_relchange(windDirection), windDirection.rel_change);
				sendWindDirection();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(7) != 0xFFFF ) { // RAINFALL
			rainFall.value = node.getResponseBuffer(7) / 10.0;
			rainFall.read = true;
			Serial.printf("Rainfall....... %0.1f mm (%0.1f: ∆%0.1f, ∆%0.1f%%)", rainFall.value, rainFall.last, get_abschange(rainFall), get_relchange(rainFall));
			if ( task_sendRainFall.canceled() ) task_sendRainFall.enableDelayed(TASK_DELAY);
			if ( abs_change(rainFall)) {
				Serial.printf(" - value change (%0.1f) exceeded absolute threshold (%0.1f): ",get_abschange(rainFall), rainFall.abs_change);
				sendRainFall();
			} else if ( rel_change(rainFall)) {
				Serial.printf(" - value change (%0.1f) exceeded relative threshold (%0.1f): ",get_relchange(rainFall), rainFall.rel_change);
				sendRainFall();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(9) != 0xFFFF ) { // RAINCOUNTER
			rainCounter.value = node.getResponseBuffer(9) / 100.0;
			rainCounter.read = true;
			Serial.printf("Raincounter.... %0.2f mm (%0.2f: ∆%0.2f, ∆%0.2f%%)", rainCounter.value, rainCounter.last, get_abschange(rainCounter), get_relchange(rainCounter));
			if ( task_sendRainCounter.canceled() ) task_sendRainCounter.enableDelayed(TASK_DELAY);
			if ( abs_change(rainCounter)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): ",get_abschange(rainCounter), rainCounter.abs_change);
				sendRainCounter();
			} else if ( rel_change(rainCounter)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): ",get_relchange(rainCounter), rainCounter.rel_change);
				sendRainCounter();
			} else {
				Serial.println();
			}
		}
		if ( node.getResponseBuffer(8) != 0xFFFF ) { // PRESSURE
			pressure.value = node.getResponseBuffer(8) / 10.0;
			pressure.read = true;
			Serial.printf("Pressure....... %0.1f hPa (%0.1f: ∆%0.1f, ∆%0.1f%%)", pressure.value, pressure.last, get_abschange(pressure), get_relchange(pressure));
			if ( task_sendPressure.canceled() ) task_sendPressure.enableDelayed(TASK_DELAY);
			if ( abs_change(pressure)) {
				Serial.printf(" - value change (%0.1f) exceeded absolute threshold (%0.1f): ",get_abschange(pressure), pressure.abs_change);
				sendPressure();
			} else if ( rel_change(pressure)) {
				Serial.printf(" - value change (%0.1f) exceeded relative threshold (%0.1f): ",get_relchange(pressure), pressure.rel_change);
				sendPressure();
			} else {
				Serial.println();
			}
		}
		if ( temperature.read && humidity.read ) { // dewpoint, frostpoint, absolute humidity, discomfort, heatindex, humidex
//			dewpoint.value = dewpoint (temperature.value, humidity.value);
			dewpoint.value = dewPoint (temperature.value, humidity.value);
			dewpoint.read = true;
			Serial.printf("dewpoint....... %0.2f °C (%0.2f: ∆%0.2f, ∆%0.2f%%)", dewpoint.value, dewpoint.last, get_abschange(dewpoint), get_relchange(dewpoint));
			if ( task_senddewpoint.canceled() ) task_senddewpoint.enableDelayed(TASK_DELAY);
			if ( abs_change(dewpoint)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): ",get_abschange(dewpoint), dewpoint.abs_change);
				senddewpoint();
			} else if ( rel_change(dewpoint)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): ",get_relchange(dewpoint), dewpoint.rel_change);
				senddewpoint();
			} else {
				Serial.println();
			}
			frostpoint.value = frostPoint (temperature.value, humidity.value);
			frostpoint.read = true;
			Serial.printf("frostpoint..... %0.2f °C (%0.2f: ∆%0.2f, ∆%0.2f%%)", frostpoint.value, frostpoint.last, get_abschange(frostpoint), get_relchange(frostpoint));
			if ( task_sendfrostpoint.canceled() ) task_sendfrostpoint.enableDelayed(TASK_DELAY);
			if ( abs_change(frostpoint)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): ",get_abschange(frostpoint), frostpoint.abs_change);
				sendfrostpoint();
			} else if ( rel_change(frostpoint)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): ",get_relchange(frostpoint), frostpoint.rel_change);
				sendfrostpoint();
			} else {
				Serial.println();
			}
			abshumidity.value = absoluteHumidity( temperature.value, humidity.value );
			abshumidity.read = true;
			Serial.printf("abs humidity... %0.2f °C (%0.2f: ∆%0.2f, ∆%0.2f%%)", abshumidity.value, abshumidity.last, get_abschange(abshumidity), get_relchange(abshumidity));
			if ( abs_change(abshumidity)) {
				Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): ",get_abschange(abshumidity), abshumidity.abs_change);
				sendAbsHumidity();
			} else if ( rel_change(abshumidity)) {
				Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): ",get_relchange(abshumidity), abshumidity.rel_change);
				sendAbsHumidity();
			} else {
				Serial.println();
			}
			
			humIdex.value = humidex( temperature.value, dewpoint.value);
			humIdex.read = true;

			heatindex.value = heatIndexC( temperature.value, humidity.value);
			heatindex.read = true;

			discomfort.value = discomfortIndex( temperature.value, humidity.value);
			discomfort.read = true;

		}
		if ( pressureTrend1.read ) {
			if ( pressureTrend1.read ) Serial.printf("Trend (1h)..... %0.0f (%0.0f)\n", pressureTrend1.value, pressureTrend1.last);
			if ( task_sendPressureTrend1.canceled() ) task_sendPressureTrend1.enableDelayed(TASK_DELAY);
		}
		if ( pressureTrend3.read ) {
			if ( pressureTrend3.read ) Serial.printf("Trend (3h)..... %0.0f (%0.0f)\n", pressureTrend3.value, pressureTrend3.last);
			if ( task_sendPressureTrend3.canceled() ) task_sendPressureTrend3.enableDelayed(TASK_DELAY);
		}

	} else {
		Serial.print("Modbus Error: ");
//		Serial.println( err );
		sensorfailure_wn90 = true;
	}
//	return result;
}

void RS485_Mode(int Mode) {
	digitalWrite(RS485_CON_PIN, Mode);
}

void RS485_TX() {
//	debugV("Modbus TX mode");
	RS485_Mode(RS485_TX_ENABLE);
}

void RS485_RX() {
//	debugV("Modbus RX mode");
	RS485_Mode(RS485_RX_ENABLE);
}

void MQTTpublish() {
	mqttMsg.add("chip_id", getChipID());
	mqttMsg.add("wifi_rssi", WiFi.RSSI());

	if ( temperature.read ) mqttMsg.add("temperature", temperature.value );
	if ( temperature1.read ) mqttMsg.add("temperature1", temperature1.value );
	if ( humidity.read ) mqttMsg.add("humidity", humidity.value );
	if ( abshumidity.read ) mqttMsg.add("humidity_abs", round(abshumidity.value) );
	if ( dewpoint.read ) mqttMsg.add("dewpoint", round(dewpoint.value) );
	if ( frostpoint.read ) mqttMsg.add("frostpoint", frostpoint.value );
	if ( humIdex.read ) mqttMsg.add("humidex", round(humIdex.value) );
	if ( heatindex.read ) mqttMsg.add("heatindex", round(heatindex.value) );
	if ( windchill.read ) mqttMsg.add("windchill", round(windchill.value) );
	if ( discomfort.read ) mqttMsg.add("discomfort", round(discomfort.value) );
	if ( pressure.read ) mqttMsg.add("pressure", pressure.value );
	if ( pressureTrend1.read ) mqttMsg.add("pressuretrend1", pressureTrend1.value );
	if ( pressureTrend3.read ) mqttMsg.add("pressuretrend3", pressureTrend3.value );
	if ( light.read ) mqttMsg.add("light", light.value );
	if ( uvIndex.read ) mqttMsg.add("uvindex", uvIndex.value );
	if ( windSpeed.read ) mqttMsg.add("windspeed", windSpeed.value );
	if ( gustSpeed.read ) mqttMsg.add("gustspeed", gustSpeed.value );
	if ( windSpeedBFT.read ) mqttMsg.add("windspeed_bft", windSpeedBFT.value );
	if ( gustSpeedBFT.read ) mqttMsg.add("gustspeed_bft", gustSpeedBFT.value );
	if ( windDirection.read ) mqttMsg.add("winddirection", windDirection.value );
	if ( rainFall.read ) mqttMsg.add("rainfall", rainFall.value );
	if ( rainCounter.read ) mqttMsg.add("raincounter", rainCounter.value );
	if ( pm25.read ) mqttMsg.add("pm25", pm25.value );
	if ( pm10.read ) mqttMsg.add("pm10", pm10.value );
	if ( pm25_normalized.read ) mqttMsg.add("pm25_normalized", pm25_normalized.value );
	if ( pm10_normalized.read ) mqttMsg.add("pm10_normalized", pm10_normalized.value );

	if (!mqttClient.connected()) mqtt_reconnect();

	if (mqttClient.connected()) {
		mqttClient.setBufferSize(1024);
/*		String msg = mqttMsg.toString();
		uint16_t msgLength = msg.length() + 1;
		if (msgLength > 10) {
			// only publish if messgae is not empty
			Serial.print("Publish results to MQTT broker "); Serial.println(net.mqttTopic);
			char msgChar[msgLength];
			msg.toCharArray(msgChar, msgLength);
			mqttClient.publish(net.mqttTopic, msgChar );
		}
		*/
		if (mqttMsg.size() > 0) mqttClient.publish(net.mqttTopic,toCharArray(mqttMsg.toString()) );
	// mqttClient.publish(net.mqttTopic,toCharArray(msg) );	}
//	Serial.println();
	}
}


void request1wTemperature() {
	Serial.println("Requesting 1wire temperature");
	ds.setWaitForConversion(false);
	ds.requestTemperaturesByAddress(sensor);
	task_read1wTemperature.restartDelayed();
}

void read1wTemperature() {
	double temp = ds.getTempC(sensor);
//	task_read1wTemperature.disable();
	if (temp == DEVICE_DISCONNECTED_C) {
    Serial.println("ERROR: Could not read 1wire temperature");
    return;
  }
	Serial.printf("1wire temperature received: %0.2f°C\n",temp);
	temperature1.value = temp;
	temperature1.read = true;
	if ( task_sendTemperature1.canceled() ) task_sendTemperature1.enableDelayed(TASK_DELAY);
	if ( abs_change(temperature1)) {
		Serial.printf(" - value change (%0.2f) exceeded absolute threshold (%0.2f): \n",get_abschange(temperature1), temperature1.abs_change);
		sendTemperature1();
	} else if ( rel_change(temperature1)) {
		Serial.printf(" - value change (%0.2f) exceeded relative threshold (%0.2f): \n",get_relchange(temperature1), temperature1.rel_change);
		sendTemperature1();
	}
}


unsigned long lastChange = 0;
unsigned long delayTime  = 2000;

void loop() {

	if (!sensorfailure_1wire && !sensorfailure_sds && !sensorfailure_wn90) esp_task_wdt_reset(); // reset WDT timer if all sensors are available
esp_task_wdt_reset();
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print("WiFi lost, restarting...");
		ESP.restart();
		delay(5000);
	}

	time_t t = now();
	server.handleClient();
	ArduinoOTA.handle();
	ElegantOTA.loop();
	runner.execute();
	knx.loop();
	Debug.handle();
	if(!knx.configured()) return;

	if (ParamAPP_useMQTT) mqttClient.loop();


	if (millis()-lastChange >= delayTime) {
		lastChange = millis();
//		nbModbusMaster.readHoldingRegisters(slaveId, 0x165, 10, read_wn90);
//		read_wn90();
		Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
//		debugV("Start Modbus cycle");
	}

/*  if (samplingDelay.justFinished()) {
    samplingDelay.restart();
		debugV("Sampling just finished");
    // this will return false and be ignored if nbModbusMaster is still processing the last cmd
    // i.e. if nbModbusMaster.isProcessing() returns true
		nbModbusMaster.readHoldingRegisters(0x90, 0x165, 12, read_wn90); // add processing fn
  }

  if (nbModbusMaster.justFinished()) {
		debugV("Modbus finished");
  } */
//      Serial.println();

	if ( timeKnown && dateKnown && (minute(t) != lastminute) ) {   // every minute
		lastminute = minute(t);
		lasthour = hour(t);
		for (int i = 1; i<=pressureRing.size(); i++) {
			debugV("Pressure ringbuffer at %d is %u", i, pressureRing.get(i));
		} 
		debugV("Ringbuffer contains %u elements, index is at %u", pressureRing.elements(), pressureRing.index());

		if ( lastminute % 5 == 0) {		// every 5 minutes
			#pragma region update rain amount ringbuffers
			raincounterMA.add(rainCounter.value);
			rainfallMA.add(rainFall.value);
			#ifndef DEBUG_DISABLED
			debugV("Update raincounter minute ringbuffer with value %u: ", rainCounter.value);
			debugV("Raincounter minute ringbuffer contains %u elements, index is at %u", raincounterMA.elements(), raincounterMA.index());
			debugV("Update rainfall minute ringbuffer with value %u: ", rainFall.value);
			debugV("Rainfall minute ringbuffer contains %u elements, index is at %u", rainfallMA.elements(), rainfallMA.index());
			#endif
			#pragma endregion
			#pragma region set hourly pressure trend if enough data is available
			if ( pressureRing.getDelta(-4) > 0 ) {
				pressureTrend1.value = pressureRing.getDelta(0) - pressureRing.getDelta(-4);
				pressureTrend1.read = true; 
				debugV("Estimated hourly pressure trend %.01f", pressureTrend1.value );
			} else {
				pressureTrend1.read = false;
			}
			#pragma endregion
		}
		if ( lastminute % 15 == 0) {	// every 15 minutes
			#pragma region update air pressure ringbuffer
			Serial.print("Update air pressure ringbuffer: ");
			debugV("Update air pressure ringbuffer with value %u: ", pressure.value*10);
			pressureRing.add(pressure.value*10);
			for (int i = 1; i<=pressureRing.size(); i++) {
			debugV("Pressure ringbuffer at %d is %u", i, pressureRing.get(i));
			} 
			debugV("Ringbuffer contains %u elements, index is at %u", pressureRing.elements(), pressureRing.index());
//			updatePressureRingbuffer();
			#pragma endregion
			#pragma region set hourly pressure trend if enough data is available
			if ( pressureRing.getDelta(-4) > 0 ) {
				pressureTrend1.value = pressureRing.getDelta(0) - pressureRing.getDelta(-4);
				pressureTrend1.read = true; 
				debugV("Estimated hourly pressure trend %.01f", pressureTrend1.value );
			} else {
				pressureTrend1.read = false;
			}
			#pragma endregion
		}
		// every full hour
		if ( lastminute == 0 ) {	// every full hour
			#pragma region update rain amount ringbuffers
			raincounterSA.add(rainCounter.value);
			debugV("Update raincounter hour ringbuffer with value %u: ", rainCounter.value);
			debugV("Raincounter hour ringbuffer contains %u elements, index is at %u", raincounterSA.elements(), raincounterSA.index());
			rainfallMA.add(rainFall.value);
			debugV("Update rainfall hour ringbuffer with value %u: ", rainFall.value);
			debugV("Rainfall hour ringbuffer contains %u elements, index is at %u", rainfallSA.elements(), rainfallSA.index());
			#pragma endregion
			#pragma region set 3 hourly pressure trend if enough data is available
			if (pressureRing.getDelta(-12) > 0 ) {
				pressureTrend3.value = pressureRing.getDelta(0) - pressureRing.getDelta(-12);
				pressureTrend3.read = true;
				debugV("Estimated 3 hourly pressure trend %.01f", pressureTrend3.value );
			} else {
				pressureTrend3.read = false;
			}
			#pragma endregion
		}
		if ( lastminute == 0 && lasthour == 0 ) {	// at midnight
			// reset values for total rain amount for today
			raincounterT = 0;
			rainfallT = 0;


		}
	}

	/*
Es existieren 2 Ringbuffer mit jeweils 13 Elementen zur Speicherung der rainfall sowie raincounter Werte im 5 Minuten Takt (Minutenarray oder MA)
Desweiteren existieren 2 Ringbuffer mit jeweils 73 Elementen in denen rainfall und raincounter jeweils stuendlich gespeichert werden (Stundenarray oder SA)
Zusaetzlich existieren 2 T-Werte, die jeweils um Mitternacht auf 0 gesetzt werden. Die aktuelle Regenmenge abzueglich dieses Wertes ergibt die Tages-Regenmenge.
*/




}
