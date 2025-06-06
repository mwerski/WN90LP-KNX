#include <Arduino.h>
#include "utilities.h"
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
#include "knxprod.h"
#include <knx.h>
#define DEBUG_DISABLE_DEBUGGER true	// Debug Optionen in SerialDebug deaktivieren
#define DEBUG_INITIAL_LEVEL DEBUG_LEVEL_VERBOSE	// Default Debug Level
#include <RemoteDebug.h>
#include <TimeLib.h>


struct tm myTime;
bool timeKnown = false;
bool dateKnown = false;

void timeCallback(GroupObject& go) {
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
			sprintf(buf, "Setting/Adjusting system time: %2d.%2d.%4d, %02d:%02d:%02d", day(t), month(t), year(t), tmp_hour, tmp_min, tmp_sec );
			Serial.println(buf);
		}
	}
}
void dateCallback(GroupObject& go) {
	if (go.value()) {
		dateKnown = true;
		myTime = KoAPP_Date.value();
		unsigned short tmp_mday = myTime.tm_mday;
		unsigned short tmp_mon = myTime.tm_mon;
		unsigned short tmp_year = myTime.tm_year;
		char buf[52];
		sprintf(buf, "Date received from bus: %2d.%2d.%4d", tmp_mday, tmp_mon, tmp_year );
		Serial.println(buf);
		time_t t = now();
		setTime(hour(t), minute(t), second(t), tmp_mday, tmp_mon, tmp_year);
		if (timeKnown == true) {
			sprintf(buf, "Setting/Adjusting system time: %2d.%2d.%4d, %02d:%02d:%02d", tmp_mday, tmp_mon, tmp_year, hour(t), minute(t), second(t) );
			Serial.println(buf);
		}
	}
}
void dateTimeCallback(GroupObject& go) {
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
		sprintf(buf, "DateTime received from bus: %2d.%2d.%4d, %02d:%02d:%02d", tmp_mday, tmp_mon, tmp_year, tmp_hour, tmp_min, tmp_sec );
		Serial.println(buf);
		Serial.println("Setting/Adjusting system time");
		setTime(tmp_hour, tmp_min, tmp_sec, tmp_mday, tmp_mon, tmp_year);
	}
}



const char* hostname = "wn90";
const char* mqtt_server = "broker.localnet";
const char *mqtt_username = "";
const char *mqtt_password = "";
u_int8_t	lastHour = NAN;	// last hour (if this changes, a full hour has passed)

WebServer server(80);

// MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Json mqttMsg;
void mqtt_reconnect() {
	Serial.print("Attempting MQTT connection...");
	// Attempt to connect
	if (mqttClient.connect(hostname, mqtt_username, mqtt_password)) {
		Serial.println("connected");
	} else {
		Serial.print("failed, rc=");
		Serial.print(mqttClient.state());
		Serial.println(" try again next time");
	}
}

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Modbus
ModbusMaster node;
bool RS485mode = true;
void RS485_Mode(int Mode);
void RS485_TX();
void RS485_RX();

u_int16_t pressureRing[12] = { NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN };	// Ringbuffer for calculating pressure tendencies
u_int32_t light = NAN;
double uvIndex = NAN;
double temperature = NAN;
u_int8_t humidity = NAN;
double absHumidity = NAN;
double windSpeed = NAN;
u_int16_t windDirection = NAN;
double gustSpeed = NAN;
double rainfall = NAN;
double absPressure = NAN;
double rainCounter = NAN;
float dewPoint = NAN;
float dewpoint(float t, float f) {
	float a, b;
  if (t >= 0) {
    a = 7.5;
    b = 237.3;
  } else {
    a = 7.6;
    b = 240.7;
  }
  float sdd = 6.1078 * pow(10, (a*t)/(b+t));  // Sättigungsdampfdruck in hPa
  float dd = sdd * (f/100);  // Dampfdruck in mBar
  float v = log10(dd/6.1078);  // v-Parameter
  float dp = (b*v) / (a-v);  // Taupunkttemperatur (°C)
//	return dp;
	float m = powf( 10.0f, 2 ); // truncate to x.yz
	dp = roundf( dp * m ) / m;
	return dp;
}

void progLedOff() {
	pixels.clear();
	pixels.show();
}

void progLedOn() {
	pixels.setPixelColor(0, pixels.Color(20, 0, 0));
	pixels.show();
}

void setup() {
	Serial.begin(115200);

	pinMode(RS485_CON_PIN, OUTPUT);
	pinMode(KEY_PIN, INPUT_PULLUP);
	Serial485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN, true);
	// communicate with Modbus slave ID 144 over Serial (port 0)
  node.preTransmission(RS485_TX);
  node.postTransmission(RS485_RX);
  node.begin(0x90, Serial485);
	RS485_Mode(RS485_TX_ENABLE);
	delay(20);


  //WiFiManager
  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.setConfigPortalTimeout(180);
	if (!wifiManager.autoConnect("AutoConnectAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  } 

	Serial.print("IP Address: ");
	Serial.println(WiFi.localIP());

	// Arduino OTA on üport 3232
	// ArduinoOTA.setPort(3232);
	ArduinoOTA.setHostname(hostname);
	// ArduinoOTA.setPassword("admin");
	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
	ArduinoOTA
		.onStart([]() {
			String type;
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

	ElegantOTA.begin(&server);

	// MQTT
	mqttClient.setServer(mqtt_server, 1883);
	

	// KNX stuff
	knx.buttonPin(5);
	//knx.ledPin(15);
	//knx.ledPinActiveOn(HIGH);

	// read adress table, association table, groupobject table and parameters from eeprom
	knx.readMemory();

	delay(5000);
	Serial.println("Starting up...");
	Serial.print("KNX configured: ");
	Serial.println(knx.configured());
	knx.setProgLedOffCallback(progLedOff);
	knx.setProgLedOnCallback(progLedOn);

	if (knx.configured()) {
		if (ParamAPP_Heartbeat > 0) {
			Serial.print("Sende Neartbeat alle "); Serial.print(ParamAPP_Heartbeat); Serial.println("s");
			// ParamAPP_Heartbeat=10:
			KoAPP_Heartbeat.dataPointType(Dpt(1, 1));
			//runner.addTask(task_heartbeat);
			//task_heartbeat.setInterval(ParamAPP_Heartbeat*1000);
			//task_heartbeat.enable();
			Serial.println("Task(s) enabled");
		} else {
			Serial.println("Sende keinen Heartbeat");
		}

		if (ParamAPP_Uhrzeit_beim_Start_lesen == 1) {
			Serial.print("Lese Uhrzeit vom Bus");

		}

		if (ParamAPP_DateTime_DPTs == 0) {
			Serial.println("Receive time and date from different KOs, define callbacks");
			KoAPP_Time.dataPointType(DPT_TimeOfDay);
			KoAPP_Time.callback(timeCallback);
			KoAPP_Date.dataPointType(DPT_Date);
			KoAPP_Date.callback(dateCallback);
		} else {
			Serial.println("Receive time and date from a single KOs, define callback");
			KoAPP_DateTime.dataPointType(DPT_DateTime);
			KoAPP_DateTime.callback(dateTimeCallback);
		}
	}
	knx.start();

}

unsigned long lastChange = 0;
unsigned long delayTime  = 30000;

void loop() {
	server.handleClient();
	ArduinoOTA.handle();
	ElegantOTA.loop();


	if (millis()-lastChange >= delayTime) {
		lastChange = millis();
		
		uint8_t j;
		uint8_t c=10;
		uint8_t result = node.readHoldingRegisters( 0x165, c );
		Serial.print("result = ");
		Serial.println( result );
		
		if (result == node.ku8MBSuccess) {
			if ( node.getResponseBuffer(0) != 0xFFFF ) { light = node.getResponseBuffer(0) * 10; }
			if ( node.getResponseBuffer(1) != 0xFFFF ) { uvIndex = node.getResponseBuffer(1) / 10.0; }
			if ( node.getResponseBuffer(2) != 0xFFFF ) { temperature = node.getResponseBuffer(2) / 10.0 - 40; }
			if ( node.getResponseBuffer(3) != 0xFFFF ) { humidity = node.getResponseBuffer(3); }
			if ( node.getResponseBuffer(4) != 0xFFFF ) { windSpeed = node.getResponseBuffer(4) / 10.0; }
			if ( node.getResponseBuffer(5) != 0xFFFF ) { gustSpeed = node.getResponseBuffer(5) / 10.0; }
			if ( node.getResponseBuffer(6) != 0xFFFF ) { windDirection = node.getResponseBuffer(6); }
			if ( node.getResponseBuffer(7) != 0xFFFF ) { rainfall = node.getResponseBuffer(7) / 10.0; }
			if ( node.getResponseBuffer(8) != 0xFFFF ) { absPressure = node.getResponseBuffer(8) / 10.0; }
			if ( node.getResponseBuffer(9) != 0xFFFF ) { rainCounter = node.getResponseBuffer(9) / 100.0; }
			if ( (node.getResponseBuffer(2) != 0xFFFF) && (node.getResponseBuffer(3) != 0xFFFF) ) { dewPoint = dewpoint (temperature, humidity); }

//			printLocalTime();
			Serial.print("Light:          "); Serial.print( light ); Serial.println(" Lux");
			Serial.print("UVI:            "); Serial.print( uvIndex , 1); Serial.println("");
			Serial.print("Temperature:    "); Serial.print(temperature, 1); Serial.println(" °C");
			Serial.print("Humidity:       "); Serial.print(humidity); Serial.println(" %");
			Serial.print("Wind Speed:     "); Serial.print(windSpeed, 1); Serial.println(" m/s");
			Serial.print("Gust Speed:     "); Serial.print(gustSpeed , 1); Serial.println(" m/s");
			Serial.print("Wind Direction: "); Serial.print(windDirection); Serial.println(" °");
			Serial.print("Rainfall:       "); Serial.print(rainfall , 1); Serial.println(" mm");
			Serial.print("ABS Pressure:   "); Serial.print(absPressure , 1); Serial.println(" mbar");
			Serial.print("Rain Counter:   "); Serial.print(rainCounter, 2); Serial.println(" mm");
			Serial.print("Dewpoint:       "); Serial.print(dewPoint, 2); Serial.println(" °C");
			

			if (light != NAN) { mqttMsg.add("light", light ); }
			if (uvIndex != NAN) { mqttMsg.add("uvIndex", uvIndex ); }
			if (temperature != NAN) { mqttMsg.add("temperature", temperature ); }
			if (humidity != NAN) { mqttMsg.add("humidity", humidity ); }
			if (windSpeed != NAN) { mqttMsg.add("windSpeed", windSpeed ); }
			if (gustSpeed != NAN) { mqttMsg.add("gustSpeed", gustSpeed ); }
			if (windDirection != NAN) { mqttMsg.add("windDirection", windDirection ); }
			if (rainfall != NAN) { mqttMsg.add("rainFall", rainfall ); }
			if (absPressure != NAN) { mqttMsg.add("pressure", absPressure ); }
			if (rainCounter != NAN) { mqttMsg.add("rainCounter", rainCounter ); }
			if (dewPoint != NAN) { mqttMsg.add("dewPoint", dewPoint ); }

			if (!mqttClient.connected()) {
				mqtt_reconnect();
			}

			if (mqttClient.connected()) {
				String msg = mqttMsg.toString();
				uint16_t msgLength = msg.length() + 1;
				char msgChar[msgLength];
				msg.toCharArray(msgChar, msgLength);
				mqttClient.publish("environmental/wn90", msgChar );
			}

		}
	}

	knx.loop();
	if(!knx.configured()) return;


}

void RS485_Mode(int Mode) {
	digitalWrite(RS485_CON_PIN, Mode);
}

void RS485_TX() {
	RS485_Mode(RS485_TX_ENABLE);
}

void RS485_RX() {
	RS485_Mode(RS485_RX_ENABLE);
}

