#pragma once

#define paramDelay(time) (uint32_t)( \
            (time & 0xC000) == 0xC000 ? (time & 0x3FFF) * 100 : \
            (time & 0xC000) == 0x0000 ? (time & 0x3FFF) * 1000 : \
            (time & 0xC000) == 0x4000 ? (time & 0x3FFF) * 60000 : \
            (time & 0xC000) == 0x8000 ? ((time & 0x3FFF) > 1000 ? 3600000 : \
                                            (time & 0x3FFF) * 3600000 ) : 0 )
//--------------------Allgemein---------------------------
#define MAIN_OpenKnxId 0xAF
#define MAIN_ApplicationNumber 0x00
#define MAIN_ApplicationVersion 0x02
#define MAIN_OrderNumber "p3-00012.1"
#define MAIN_ParameterSize 239
#define MAIN_MaxKoNumber 21


#define APP_startUp_Delay		0x0000
#define APP_startUp_Delay_Shift	3
#define APP_startUp_Delay_Mask	0x001F
// Offset: 0, Size: 5 Bit, Text: Verzögerung beim Start
#define ParamAPP_startUp_Delay ((uint32_t)((knx.paramByte(APP_startUp_Delay) >> APP_startUp_Delay_Shift) & APP_startUp_Delay_Mask))
#define APP_Heartbeat		0x0001
#define APP_Heartbeat_Shift	4
#define APP_Heartbeat_Mask	0x0FFF
// Offset: 1, Size: 12 Bit, Text: "In Betrieb" senden alle
#define ParamAPP_Heartbeat ((uint32_t)((knx.paramWord(APP_Heartbeat) >> APP_Heartbeat_Shift) & APP_Heartbeat_Mask))
#define APP_DateTime_DPTs		0x0000
// Offset: 0, BitOffset: 5, Size: 1 Bit, Text: Zeit empfangen über
#define ParamAPP_DateTime_DPTs knx.paramBit(APP_DateTime_DPTs, 5)
#define APP_Watchdog		0x0000
// Offset: 0, BitOffset: 6, Size: 1 Bit, Text: Watchdog aktivieren
#define ParamAPP_Watchdog knx.paramBit(APP_Watchdog, 6)
#define APP_1wire_vorhanden		0x0000
// Offset: 0, BitOffset: 7, Size: 1 Bit, Text: 1wire Sensor
#define ParamAPP_1wire_vorhanden knx.paramBit(APP_1wire_vorhanden, 7)
#define APP_Feinstaubsensor_vorhanden		0x0003
// Offset: 3, Size: 1 Bit, Text: Feinstaubsensor
#define ParamAPP_Feinstaubsensor_vorhanden knx.paramBit(APP_Feinstaubsensor_vorhanden, 0)
#define APP_useMQTT		0x0003
// Offset: 3, BitOffset: 1, Size: 1 Bit, Text: MQTT nutzen
#define ParamAPP_useMQTT knx.paramBit(APP_useMQTT, 1)
#define APP_MQTT_Host		0x0004
// Offset: 4, Size: 256 Bit (32 Byte), Text: Broker Hostname/IP
#define ParamAPP_MQTT_Host knx.paramData(APP_MQTT_Host)
#define APP_MQTT_Port		0x0024
// Offset: 36, Size: 16 Bit (2 Byte), Text: Broker Port
#define ParamAPP_MQTT_Port ((uint32_t)((knx.paramWord(APP_MQTT_Port))))
#define APP_MQTT_User		0x0026
// Offset: 38, Size: 256 Bit (32 Byte), Text: Username
#define ParamAPP_MQTT_User knx.paramData(APP_MQTT_User)
#define APP_MQTT_Password		0x0046
// Offset: 70, Size: 256 Bit (32 Byte), Text: Password
#define ParamAPP_MQTT_Password knx.paramData(APP_MQTT_Password)
#define APP_MQTT_Topic		0x0066
// Offset: 102, Size: 256 Bit (32 Byte), Text: Topic
#define ParamAPP_MQTT_Topic knx.paramData(APP_MQTT_Topic)
#define APP_MQTT_Frequency		0x0086
#define APP_MQTT_Frequency_Shift	4
#define APP_MQTT_Frequency_Mask	0x0FFF
// Offset: 134, Size: 12 Bit, Text: Sende Messwerte alle
#define ParamAPP_MQTT_Frequency ((uint32_t)((knx.paramWord(APP_MQTT_Frequency) >> APP_MQTT_Frequency_Shift) & APP_MQTT_Frequency_Mask))
#define APP_Uhrzeit_beim_Start_lesen		0x0003
// Offset: 3, BitOffset: 2, Size: 1 Bit, Text: Uhrzeit beim Start lesen
#define ParamAPP_Uhrzeit_beim_Start_lesen knx.paramBit(APP_Uhrzeit_beim_Start_lesen, 2)
#define APP_RemoteDebug		0x0003
// Offset: 3, BitOffset: 3, Size: 1 Bit, Text: RemoteDebug aktivieren
#define ParamAPP_RemoteDebug knx.paramBit(APP_RemoteDebug, 3)
#define APP_Temperatur_DPT		0x0003
// Offset: 3, BitOffset: 4, Size: 1 Bit, Text: Sende Temperatur als
#define ParamAPP_Temperatur_DPT knx.paramBit(APP_Temperatur_DPT, 4)
#define APP_Temperatur_Senden_zyklisch		0x0088
#define APP_Temperatur_Senden_zyklisch_Shift	4
#define APP_Temperatur_Senden_zyklisch_Mask	0x0FFF
// Offset: 136, Size: 12 Bit, Text: Sende Temperatur alle
#define ParamAPP_Temperatur_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_Temperatur_Senden_zyklisch) >> APP_Temperatur_Senden_zyklisch_Shift) & APP_Temperatur_Senden_zyklisch_Mask))
#define APP_Temperatur_Senden_Wertaenderung_absolut		0x008A
// Offset: 138, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Temperatur_Senden_Wertaenderung_absolut knx.paramFloat(APP_Temperatur_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_Temperatur_Senden_Wertaenderung_relativ		0x008C
// Offset: 140, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Temperatur_Senden_Wertaenderung_relativ knx.paramFloat(APP_Temperatur_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_Feuchte_DPT		0x0003
#define APP_Feuchte_DPT_Shift	1
#define APP_Feuchte_DPT_Mask	0x0003
// Offset: 3, BitOffset: 5, Size: 2 Bit, Text: Sende Feuchte als
#define ParamAPP_Feuchte_DPT ((uint32_t)((knx.paramByte(APP_Feuchte_DPT) >> APP_Feuchte_DPT_Shift) & APP_Feuchte_DPT_Mask))
#define APP_Feuchte_Senden_zyklisch		0x008E
#define APP_Feuchte_Senden_zyklisch_Shift	4
#define APP_Feuchte_Senden_zyklisch_Mask	0x0FFF
// Offset: 142, Size: 12 Bit, Text: Sende relative Feuchte alle
#define ParamAPP_Feuchte_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_Feuchte_Senden_zyklisch) >> APP_Feuchte_Senden_zyklisch_Shift) & APP_Feuchte_Senden_zyklisch_Mask))
#define APP_Feuchte_Senden_Wertaenderung_absolut		0x0090
// Offset: 144, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Feuchte_Senden_Wertaenderung_absolut knx.paramFloat(APP_Feuchte_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_Feuchte_Senden_Wertaenderung_relativ		0x0092
// Offset: 146, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Feuchte_Senden_Wertaenderung_relativ knx.paramFloat(APP_Feuchte_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_WindSpeed_DPT		0x0003
// Offset: 3, BitOffset: 7, Size: 1 Bit, Text: Sende Windgeschwindigkeit als
#define ParamAPP_WindSpeed_DPT knx.paramBit(APP_WindSpeed_DPT, 7)
#define APP_WindDir_DPT		0x0094
// Offset: 148, Size: 1 Bit, Text: Sende Windgeschwindigkeit als
#define ParamAPP_WindDir_DPT knx.paramBit(APP_WindDir_DPT, 0)
#define APP_WindSpeed_Senden_zyklisch		0x0095
#define APP_WindSpeed_Senden_zyklisch_Shift	4
#define APP_WindSpeed_Senden_zyklisch_Mask	0x0FFF
// Offset: 149, Size: 12 Bit, Text: Sende Windgeschwindigket alle
#define ParamAPP_WindSpeed_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_WindSpeed_Senden_zyklisch) >> APP_WindSpeed_Senden_zyklisch_Shift) & APP_WindSpeed_Senden_zyklisch_Mask))
#define APP_WindSpeed_Senden_Wertaenderung_absolut		0x0097
// Offset: 151, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_WindSpeed_Senden_Wertaenderung_absolut knx.paramFloat(APP_WindSpeed_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_WindSpeed_Senden_Wertaenderung_relativ		0x0099
// Offset: 153, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_WindSpeed_Senden_Wertaenderung_relativ knx.paramFloat(APP_WindSpeed_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_WindDir_Senden_zyklisch		0x009B
#define APP_WindDir_Senden_zyklisch_Shift	4
#define APP_WindDir_Senden_zyklisch_Mask	0x0FFF
// Offset: 155, Size: 12 Bit, Text: Sende Windrichtung alle
#define ParamAPP_WindDir_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_WindDir_Senden_zyklisch) >> APP_WindDir_Senden_zyklisch_Shift) & APP_WindDir_Senden_zyklisch_Mask))
#define APP_WindDir_Senden_Wertaenderung_absolut		0x009D
// Offset: 157, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_WindDir_Senden_Wertaenderung_absolut knx.paramFloat(APP_WindDir_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_WindDir_Senden_Wertaenderung_relativ		0x009F
// Offset: 159, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_WindDir_Senden_Wertaenderung_relativ knx.paramFloat(APP_WindDir_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_Pressure_DPT		0x0094
// Offset: 148, BitOffset: 1, Size: 1 Bit, Text: Sende Luftdruck als
#define ParamAPP_Pressure_DPT knx.paramBit(APP_Pressure_DPT, 1)
#define APP_Pressure_Senden_zyklisch		0x00A1
#define APP_Pressure_Senden_zyklisch_Shift	4
#define APP_Pressure_Senden_zyklisch_Mask	0x0FFF
// Offset: 161, Size: 12 Bit, Text: Sende Luftdruck alle
#define ParamAPP_Pressure_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_Pressure_Senden_zyklisch) >> APP_Pressure_Senden_zyklisch_Shift) & APP_Pressure_Senden_zyklisch_Mask))
#define APP_Pressure_Senden_Wertaenderung_absolut		0x00A3
// Offset: 163, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Pressure_Senden_Wertaenderung_absolut knx.paramFloat(APP_Pressure_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_Pressure_Senden_Wertaenderung_relativ		0x00A5
// Offset: 165, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Pressure_Senden_Wertaenderung_relativ knx.paramFloat(APP_Pressure_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_Hostname		0x00A7
// Offset: 167, Size: 256 Bit (32 Byte), Text: Hostname
#define ParamAPP_Hostname knx.paramData(APP_Hostname)
#define APP_Helligkeit_DPT		0x0094
// Offset: 148, BitOffset: 2, Size: 1 Bit, Text: Sende Helligkeit als
#define ParamAPP_Helligkeit_DPT knx.paramBit(APP_Helligkeit_DPT, 2)
#define APP_Helligkeit_Senden_zyklisch		0x00C7
#define APP_Helligkeit_Senden_zyklisch_Shift	4
#define APP_Helligkeit_Senden_zyklisch_Mask	0x0FFF
// Offset: 199, Size: 12 Bit, Text: Sende Helligkeit alle
#define ParamAPP_Helligkeit_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_Helligkeit_Senden_zyklisch) >> APP_Helligkeit_Senden_zyklisch_Shift) & APP_Helligkeit_Senden_zyklisch_Mask))
#define APP_Helligkeit_Senden_Wertaenderung_absolut		0x00C9
// Offset: 201, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Helligkeit_Senden_Wertaenderung_absolut knx.paramFloat(APP_Helligkeit_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_Helligkeit_Senden_Wertaenderung_relativ		0x00CB
// Offset: 203, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Helligkeit_Senden_Wertaenderung_relativ knx.paramFloat(APP_Helligkeit_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_UVindex_DPT		0x0094
// Offset: 148, BitOffset: 3, Size: 1 Bit, Text: Sende UV-Index als
#define ParamAPP_UVindex_DPT knx.paramBit(APP_UVindex_DPT, 3)
#define APP_UVindex_Senden_zyklisch		0x00CD
#define APP_UVindex_Senden_zyklisch_Shift	4
#define APP_UVindex_Senden_zyklisch_Mask	0x0FFF
// Offset: 205, Size: 12 Bit, Text: Sende UV-Index alle
#define ParamAPP_UVindex_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_UVindex_Senden_zyklisch) >> APP_UVindex_Senden_zyklisch_Shift) & APP_UVindex_Senden_zyklisch_Mask))
#define APP_UVindex_Senden_Wertaenderung_absolut		0x00CF
// Offset: 207, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_UVindex_Senden_Wertaenderung_absolut knx.paramFloat(APP_UVindex_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_UVindex_Senden_Wertaenderung_relativ		0x00D1
// Offset: 209, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_UVindex_Senden_Wertaenderung_relativ knx.paramFloat(APP_UVindex_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_Regen_DPT		0x0094
// Offset: 148, BitOffset: 4, Size: 1 Bit, Text: Sende Regenmenge als
#define ParamAPP_Regen_DPT knx.paramBit(APP_Regen_DPT, 4)
#define APP_Regen_Senden_zyklisch		0x00D3
#define APP_Regen_Senden_zyklisch_Shift	4
#define APP_Regen_Senden_zyklisch_Mask	0x0FFF
// Offset: 211, Size: 12 Bit, Text: Sende Regenmenge alle
#define ParamAPP_Regen_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_Regen_Senden_zyklisch) >> APP_Regen_Senden_zyklisch_Shift) & APP_Regen_Senden_zyklisch_Mask))
#define APP_Regen_Senden_Wertaenderung_absolut		0x00D5
// Offset: 213, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Regen_Senden_Wertaenderung_absolut knx.paramFloat(APP_Regen_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_Regen_Senden_Wertaenderung_relativ		0x00D7
// Offset: 215, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Regen_Senden_Wertaenderung_relativ knx.paramFloat(APP_Regen_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
#define APP_IP		0x00D9
// Offset: 217, Size: 32 Bit (4 Byte), Text: IP Adresse
#define ParamAPP_IP knx.paramInt(APP_IP)
#define APP_Netzmaske		0x00DD
// Offset: 221, Size: 32 Bit (4 Byte), Text: Netzmaske
#define ParamAPP_Netzmaske knx.paramInt(APP_Netzmaske)
#define APP_Gateway		0x00E1
// Offset: 225, Size: 32 Bit (4 Byte), Text: Gateway
#define ParamAPP_Gateway knx.paramInt(APP_Gateway)
#define APP_DNS		0x00E5
// Offset: 229, Size: 32 Bit (4 Byte), Text: Nameserver
#define ParamAPP_DNS knx.paramInt(APP_DNS)
#define APP_UseDHCP		0x0094
// Offset: 148, BitOffset: 5, Size: 1 Bit, Text: Nutze DHCP
#define ParamAPP_UseDHCP knx.paramBit(APP_UseDHCP, 5)
#define APP_Feinstaub_DPT		0x0094
// Offset: 148, BitOffset: 6, Size: 1 Bit, Text: Sende Feinstaubwerte als
#define ParamAPP_Feinstaub_DPT knx.paramBit(APP_Feinstaub_DPT, 6)
#define APP_Feinstaub_Senden_zyklisch		0x00E9
#define APP_Feinstaub_Senden_zyklisch_Shift	4
#define APP_Feinstaub_Senden_zyklisch_Mask	0x0FFF
// Offset: 233, Size: 12 Bit, Text: Sende Feinstaubwerte allle
#define ParamAPP_Feinstaub_Senden_zyklisch ((uint32_t)((knx.paramWord(APP_Feinstaub_Senden_zyklisch) >> APP_Feinstaub_Senden_zyklisch_Shift) & APP_Feinstaub_Senden_zyklisch_Mask))
#define APP_Feinstaub_Senden_Wertaenderung_absolut		0x00EB
// Offset: 235, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Feinstaub_Senden_Wertaenderung_absolut knx.paramFloat(APP_Feinstaub_Senden_Wertaenderung_absolut, Float_Enc_DPT9)
#define APP_Feinstaub_Senden_Wertaenderung_relativ		0x00ED
// Offset: 237, Size: 16 Bit (2 Byte), Text: Sende bei Änderung um
#define ParamAPP_Feinstaub_Senden_Wertaenderung_relativ knx.paramFloat(APP_Feinstaub_Senden_Wertaenderung_relativ, Float_Enc_DPT9)
//!< Number: 1, Text: In Betrieb, Function: Statusmeldung
#define APP_KoHeartbeat 1
#define KoAPP_Heartbeat knx.getGroupObject(APP_KoHeartbeat)
//!< Number: 2, Text: Fehlermeldung, Function: Statusmeldung
#define APP_KoFehlermeldung 2
#define KoAPP_Fehlermeldung knx.getGroupObject(APP_KoFehlermeldung)
//!< Number: 3, Text: Zeit, Function: Eingang
#define APP_KoTime 3
#define KoAPP_Time knx.getGroupObject(APP_KoTime)
//!< Number: 4, Text: Datum, Function: Eingang
#define APP_KoDate 4
#define KoAPP_Date knx.getGroupObject(APP_KoDate)
//!< Number: 5, Text: Datum/Zeit, Function: Eingang
#define APP_KoDateTime 5
#define KoAPP_DateTime knx.getGroupObject(APP_KoDateTime)
//!< Number: 6, Text: Temperatur (°C), Function: Messwert
#define APP_KoTemperatur_DPT9 6
#define KoAPP_Temperatur_DPT9 knx.getGroupObject(APP_KoTemperatur_DPT9)
//!< Number: 6, Text: Temperatur (°C), Function: Messwert
#define APP_KoTemperatur_DPT14 6
#define KoAPP_Temperatur_DPT14 knx.getGroupObject(APP_KoTemperatur_DPT14)
//!< Number: 7, Text: Temperatur 1wire (°C), Function: Messwert
#define APP_KoTemperatur_1wire_DPT9 7
#define KoAPP_Temperatur_1wire_DPT9 knx.getGroupObject(APP_KoTemperatur_1wire_DPT9)
//!< Number: 7, Text: Temperatur 1wire (°C), Function: Messwert
#define APP_KoTemperatur_1wire_DPT14 7
#define KoAPP_Temperatur_1wire_DPT14 knx.getGroupObject(APP_KoTemperatur_1wire_DPT14)
//!< Number: 8, Text: Taupunkt (°C), Function: Messwert
#define APP_KoTaupunkt_DPT9 8
#define KoAPP_Taupunkt_DPT9 knx.getGroupObject(APP_KoTaupunkt_DPT9)
//!< Number: 8, Text: Taupunkt (°C), Function: Messwert
#define APP_KoTaupunkt_DPT14 8
#define KoAPP_Taupunkt_DPT14 knx.getGroupObject(APP_KoTaupunkt_DPT14)
//!< Number: 9, Text: relative Feuchte (%), Function: Messwert
#define APP_KoFeuchte_DPT6 9
#define KoAPP_Feuchte_DPT6 knx.getGroupObject(APP_KoFeuchte_DPT6)
//!< Number: 9, Text: relative Feuchte (%), Function: Messwert
#define APP_KoFeuchte_DPT9 9
#define KoAPP_Feuchte_DPT9 knx.getGroupObject(APP_KoFeuchte_DPT9)
//!< Number: 9, Text: relative Feuchte (%), Function: Messwert
#define APP_KoFeuchte_DPT14 9
#define KoAPP_Feuchte_DPT14 knx.getGroupObject(APP_KoFeuchte_DPT14)
//!< Number: 10, Text: Luftdruck (mBar), Function: Messwert
#define APP_KoPressure_DPT9 10
#define KoAPP_Pressure_DPT9 knx.getGroupObject(APP_KoPressure_DPT9)
//!< Number: 10, Text: Luftdruck (mBar), Function: Messwert
#define APP_KoPressure_DPT14 10
#define KoAPP_Pressure_DPT14 knx.getGroupObject(APP_KoPressure_DPT14)
//!< Number: 11, Text: Luftdruck Tendenz (stündlch), Function: Messwert
#define APP_KoPressureTrend1h_DPT9 11
#define KoAPP_PressureTrend1h_DPT9 knx.getGroupObject(APP_KoPressureTrend1h_DPT9)
//!< Number: 11, Text: Luftdruck Tendenz (stündlch), Function: Messwert
#define APP_KoPressureTrend1h_DPT14 11
#define KoAPP_PressureTrend1h_DPT14 knx.getGroupObject(APP_KoPressureTrend1h_DPT14)
//!< Number: 12, Text: Luftdruck Tendenz (3 stündlch), Function: Messwert
#define APP_KoPressureTrend3h_DPT9 12
#define KoAPP_PressureTrend3h_DPT9 knx.getGroupObject(APP_KoPressureTrend3h_DPT9)
//!< Number: 12, Text: Luftdruck Tendenz (3 stündlch), Function: Messwert
#define APP_KoPressureTrend3h_DPT14 12
#define KoAPP_PressureTrend3h_DPT14 knx.getGroupObject(APP_KoPressureTrend3h_DPT14)
//!< Number: 13, Text: Windgeschwindigkeit (m/s), Function: Messwert
#define APP_KoWindSpeed_DPT9 13
#define KoAPP_WindSpeed_DPT9 knx.getGroupObject(APP_KoWindSpeed_DPT9)
//!< Number: 13, Text: Windgeschwindigkeit (m/s), Function: Messwert
#define APP_KoWindSpeed_DPT14 13
#define KoAPP_WindSpeed_DPT14 knx.getGroupObject(APP_KoWindSpeed_DPT14)
//!< Number: 14, Text: Windgeschwindigkeit in Böen (m/s), Function: Messwert
#define APP_KoGustSpeed_DPT9 14
#define KoAPP_GustSpeed_DPT9 knx.getGroupObject(APP_KoGustSpeed_DPT9)
//!< Number: 14, Text: Windgeschwindigkeit in Böen (m/s), Function: Messwert
#define APP_KoGustSpeed_DPT14 14
#define KoAPP_GustSpeed_DPT14 knx.getGroupObject(APP_KoGustSpeed_DPT14)
//!< Number: 15, Text: Windrichtung (Grad), Function: Messwert
#define APP_KoWindDir_DPT9 15
#define KoAPP_WindDir_DPT9 knx.getGroupObject(APP_KoWindDir_DPT9)
//!< Number: 15, Text: Windrichtung (Grad), Function: Messwert
#define APP_KoWindDir_DPT14 15
#define KoAPP_WindDir_DPT14 knx.getGroupObject(APP_KoWindDir_DPT14)
//!< Number: 16, Text: Helligkeit (Lux), Function: Messwert
#define APP_KoHelligkeit_DPT9 16
#define KoAPP_Helligkeit_DPT9 knx.getGroupObject(APP_KoHelligkeit_DPT9)
//!< Number: 16, Text: Helligkeit (Lux), Function: Messwert
#define APP_KoHelligkeit_DPT14 16
#define KoAPP_Helligkeit_DPT14 knx.getGroupObject(APP_KoHelligkeit_DPT14)
//!< Number: 17, Text: UV-Index, Function: Messwert
#define APP_KoUVindex_DPT9 17
#define KoAPP_UVindex_DPT9 knx.getGroupObject(APP_KoUVindex_DPT9)
//!< Number: 17, Text: UV-Index, Function: Messwert
#define APP_KoUVindex_DPT14 17
#define KoAPP_UVindex_DPT14 knx.getGroupObject(APP_KoUVindex_DPT14)
//!< Number: 18, Text: Regenmenge (0.1mm), Function: Messwert
#define APP_KoRainFall_DPT9 18
#define KoAPP_RainFall_DPT9 knx.getGroupObject(APP_KoRainFall_DPT9)
//!< Number: 18, Text: Regenmenge (0.1mm), Function: Messwert
#define APP_KoRainFall_DPT14 18
#define KoAPP_RainFall_DPT14 knx.getGroupObject(APP_KoRainFall_DPT14)
//!< Number: 19, Text: Regenmenge (0.01mm), Function: Messwert
#define APP_KoRainCounter_DPT9 19
#define KoAPP_RainCounter_DPT9 knx.getGroupObject(APP_KoRainCounter_DPT9)
//!< Number: 19, Text: Regenmenge (0.01mm), Function: Messwert
#define APP_KoRainCounter_DPT14 19
#define KoAPP_RainCounter_DPT14 knx.getGroupObject(APP_KoRainCounter_DPT14)
//!< Number: 20, Text: PM2.5 Konzentration, Function: Messwert
#define APP_KoPM25_DPT9 20
#define KoAPP_PM25_DPT9 knx.getGroupObject(APP_KoPM25_DPT9)
//!< Number: 20, Text: PM2.5 Konzentration, Function: Messwert
#define APP_KoPM25_DPT14 20
#define KoAPP_PM25_DPT14 knx.getGroupObject(APP_KoPM25_DPT14)
//!< Number: 21, Text: PM10 Konzentration, Function: Messwert
#define APP_KoPM10_DPT9 21
#define KoAPP_PM10_DPT9 knx.getGroupObject(APP_KoPM10_DPT9)
//!< Number: 21, Text: PM10 Konzentration, Function: Messwert
#define APP_KoPM10_DPT14 21
#define KoAPP_PM10_DPT14 knx.getGroupObject(APP_KoPM10_DPT14)

