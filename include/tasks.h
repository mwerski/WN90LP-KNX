#ifndef _tasks_h
#define _tasks_h

#pragma region Scheduler definitions and prototypes
#include <TaskScheduler.h>
void MQTTpublish();
void sendTemperature();
void sendTemperature1();
void senddewpoint();
void sendfrostpoint();
void sendHumidity();
void sendAbsHumidity();
void sendWindSpeed();
void sendGustSpeed();
void sendWindSpeedBft();
void sendGustSpeedBft();
void sendWindDirection();
void sendPressure();
void sendPressureTrend1();
void sendPressureTrend3();
void sendRainFall();
void sendRainCounter();
void sendUVindex();
void sendBrightness();
void sendPM25();
void sendPM10();
void sendPM25_normalized();
void sendPM10_normalized();
void request1wTemperature();
void read1wTemperature();
void sendHeartbeat();
void read_SDS();

//Task task_updatePressureRingbuffer(RINGBUFFER_UPDATE, TASK_FOREVER, &updatePressureRingbuffer);	// update ringbuffer for trends of pressure readings
#define TASK_DELAY 170	// delay in ms when enablinig a task
Task task_MQTTpublish(10000, TASK_FOREVER, &MQTTpublish);
Task task_sendTemperature(60000, TASK_FOREVER, &sendTemperature);
Task task_sendTemperature1(60000, TASK_FOREVER, &sendTemperature1);
Task task_senddewpoint(60000, TASK_FOREVER, &senddewpoint);
Task task_sendfrostpoint(60000, TASK_FOREVER, &sendfrostpoint);
Task task_sendHumidity(60000, TASK_FOREVER, &sendHumidity);
Task task_sendWindSpeed(60000, TASK_FOREVER, &sendWindSpeed);
Task task_sendGustSpeed(60000, TASK_FOREVER, &sendGustSpeed);
Task task_sendWindSpeedBft(60000, TASK_FOREVER, &sendWindSpeedBft);
Task task_sendGustSpeedBft(60000, TASK_FOREVER, &sendGustSpeedBft);
Task task_sendWindDirection(60000, TASK_FOREVER, &sendWindDirection);
Task task_sendPressure(60000, TASK_FOREVER, &sendPressure);
Task task_sendPressureTrend1(60000, TASK_FOREVER, &sendPressureTrend1);
Task task_sendPressureTrend3(60000, TASK_FOREVER, &sendPressureTrend3);
Task task_sendRainFall(60000, TASK_FOREVER, &sendRainFall);
Task task_sendRainCounter(60000, TASK_FOREVER, &sendRainCounter);
Task task_sendUVindex(60000, TASK_FOREVER, &sendUVindex);
Task task_sendBrightness(60000, TASK_FOREVER, &sendBrightness);
Task task_sendPM25_normalized(60000, TASK_FOREVER, &sendPM25_normalized);
Task task_sendPM10_normalized(60000, TASK_FOREVER, &sendPM10_normalized);
Task task_sendPM25(60000, TASK_FOREVER, &sendPM25);
Task task_sendPM10(60000, TASK_FOREVER, &sendPM10);
Task task_request1wTemperature(2000, TASK_FOREVER, &request1wTemperature);
Task task_read1wTemperature(800, TASK_ONCE, &read1wTemperature);
Task task_heartbeat(10000, TASK_FOREVER, &sendHeartbeat);
Task task_readSDS(30000, TASK_FOREVER, &read_SDS);
Scheduler runner;
#pragma endregion

void setup_tasks() {
	Serial.println("Registering and enabling tasks");
	if (ParamAPP_Temperatur_Senden_zyklisch > 0) {
		Serial.print("Send temperature values every "); Serial.print(ParamAPP_Temperatur_Senden_zyklisch); Serial.println("s, enabling tasks");
		runner.addTask(task_sendTemperature);
		task_sendTemperature.setInterval(ParamAPP_Temperatur_Senden_zyklisch*1000);
		task_sendTemperature.enableDelayed(TASK_DELAY);
		runner.addTask(task_senddewpoint);
		task_senddewpoint.setInterval(ParamAPP_Temperatur_Senden_zyklisch*1000);
		task_senddewpoint.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendfrostpoint);
		task_sendfrostpoint.setInterval(ParamAPP_Temperatur_Senden_zyklisch*1000);
		task_sendfrostpoint.enableDelayed(TASK_DELAY);
		if (ParamAPP_1wire_vorhanden) {
			runner.addTask(task_sendTemperature1);
			task_sendTemperature1.setInterval(ParamAPP_Temperatur_Senden_zyklisch*1000);
			task_sendTemperature1.enableDelayed(TASK_DELAY);
		}
	}
	if (ParamAPP_Feuchte_Senden_zyklisch > 0) {
		Serial.print("Send humidity every "); Serial.print(ParamAPP_Feuchte_Senden_zyklisch); Serial.println("s, enabling task");
		runner.addTask(task_sendHumidity);
		task_sendHumidity.setInterval(ParamAPP_Feuchte_Senden_zyklisch*1000);
		task_sendHumidity.enableDelayed(TASK_DELAY);
	}
	if (ParamAPP_WindSpeed_Senden_zyklisch > 0) {
		Serial.print("Send wind speed values every "); Serial.print(ParamAPP_WindSpeed_Senden_zyklisch); Serial.println("s, enabling tasks");
		runner.addTask(task_sendWindSpeed);
		task_sendWindSpeed.setInterval(ParamAPP_WindSpeed_Senden_zyklisch*1000);
		task_sendWindSpeed.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendGustSpeed);
		task_sendGustSpeed.setInterval(ParamAPP_WindSpeed_Senden_zyklisch*1000);
		task_sendGustSpeed.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendWindSpeedBft);
		task_sendWindSpeedBft.setInterval(ParamAPP_WindSpeed_Senden_zyklisch*1000);
		task_sendWindSpeedBft.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendGustSpeedBft);
		task_sendGustSpeedBft.setInterval(ParamAPP_WindSpeed_Senden_zyklisch*1000);
		task_sendGustSpeedBft.enableDelayed(TASK_DELAY);
	}
	if (ParamAPP_WindDir_Senden_zyklisch > 0) {
		Serial.print("Send wind direction every "); Serial.print(ParamAPP_WindDir_Senden_zyklisch); Serial.println("s, enabling task");
		runner.addTask(task_sendWindDirection);
		task_sendWindDirection.setInterval(ParamAPP_WindDir_Senden_zyklisch*1000);
		task_sendWindDirection.enableDelayed(TASK_DELAY);
	}
	if (ParamAPP_Pressure_Senden_zyklisch > 0) {
		Serial.print("Send pressure values every "); Serial.print(ParamAPP_Pressure_Senden_zyklisch); Serial.println("s, enabling tasks");
		runner.addTask(task_sendPressure);
		task_sendPressure.setInterval(ParamAPP_Pressure_Senden_zyklisch*1000);
		task_sendPressure.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendPressureTrend1);
		task_sendPressureTrend1.setInterval(ParamAPP_Pressure_Senden_zyklisch*1000);
		task_sendPressureTrend1.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendPressureTrend3);
		task_sendPressureTrend3.setInterval(ParamAPP_Pressure_Senden_zyklisch*1000);
		task_sendPressureTrend3.enableDelayed(TASK_DELAY);
	}
	if (ParamAPP_Regen_Senden_zyklisch > 0) {
		Serial.print("Send rain values every "); Serial.print(ParamAPP_Regen_Senden_zyklisch); Serial.println("s, enabling tasks");
		runner.addTask(task_sendRainFall);
		task_sendRainFall.setInterval(ParamAPP_Regen_Senden_zyklisch*1000);
		task_sendRainFall.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendRainCounter);
		task_sendRainCounter.setInterval(ParamAPP_Regen_Senden_zyklisch*1000);
		task_sendRainCounter.enableDelayed(TASK_DELAY);
	}
	if (ParamAPP_Helligkeit_Senden_zyklisch > 0) {
		Serial.print("Send brightness every "); Serial.print(ParamAPP_Helligkeit_Senden_zyklisch); Serial.println("s, enabling task");
		runner.addTask(task_sendBrightness);
		task_sendBrightness.setInterval(ParamAPP_Helligkeit_Senden_zyklisch*1000);
		task_sendBrightness.enableDelayed(TASK_DELAY);
	}
	if (ParamAPP_UVindex_Senden_zyklisch > 0) {
		Serial.print("Send ultraviolet index every "); Serial.print(ParamAPP_UVindex_Senden_zyklisch); Serial.println("s, enabling task");
		runner.addTask(task_sendUVindex);
		task_sendUVindex.setInterval(ParamAPP_UVindex_Senden_zyklisch*1000);
		task_sendUVindex.enableDelayed(TASK_DELAY);
	}
	if (ParamAPP_Feinstaubsensor_vorhanden && ParamAPP_Feinstaub_Senden_zyklisch > 0) {
		Serial.print("Send particle concentrations every "); Serial.print(ParamAPP_Feinstaub_Senden_zyklisch); Serial.println("s, enabling tasks");
		runner.addTask(task_sendPM25);
		task_sendPM25.setInterval(ParamAPP_Feinstaub_Senden_zyklisch*1000);
		task_sendPM25.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendPM10);
		task_sendPM10.setInterval(ParamAPP_Feinstaub_Senden_zyklisch*1000);
		task_sendPM10.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendPM25_normalized);
		task_sendPM25_normalized.setInterval(ParamAPP_Feinstaub_Senden_zyklisch*1000);
		task_sendPM25_normalized.enableDelayed(TASK_DELAY);
		runner.addTask(task_sendPM10_normalized);
		task_sendPM10_normalized.setInterval(ParamAPP_Feinstaub_Senden_zyklisch*1000);
		task_sendPM10_normalized.enableDelayed(TASK_DELAY);
	}
}



#endif