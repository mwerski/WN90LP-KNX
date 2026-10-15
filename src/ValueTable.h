#pragma once
// ERZEUGT durch scripts/generate_ets.py - nicht von Hand aendern
#include "knxprod.h"
#include <stdint.h>

enum class WxId : uint8_t
{
    Temp,
    Temp1W,
    Humidity,
    AbsHumidity,
    DewPoint,
    FrostPoint,
    Humidex,
    HeatIndex,
    WindChill,
    Discomfort,
    WindSpeed,
    GustSpeed,
    WindBft,
    GustBft,
    WindDir,
    Pressure,
    PressureSea,
    Trend1h,
    Trend3h,
    Light,
    UvIndex,
    RainTotal,
    RainRate,
    Rain5m,
    Rain1h,
    Rain6h,
    Rain24h,
    Rain48h,
    Rain72h,
    RainToday,
    Raining,
    Pm25,
    Pm10,
    Pm25Norm,
    Pm10Norm,
    Tendency,
    Forecast,
    ForecastText,
    WeatherSymbol,
    WindAvg10,
    GustMax10,
    WindAlarm1,
    WindAlarm2,
    WindAlarm3,
    SunAzimuth,
    SunElevation,
    DayNight,
    TempMinToday,
    TempMaxToday,
    TempMinYesterday,
    TempMaxYesterday,
    GustMaxToday,
    GustMaxYesterday,
    Frost,
    IceWarning,
    SummerWinter,
    MeanTemp,
    DailyMeanYesterday,
    SummerDay,
    HotDay,
    FrostDay,
    IceDay,
    GlobalRadiation,
    CloudCover,
    RadiationToday,
    RadiationYesterday,
    PvPower,
    PvEnergyToday,
    PvEnergyYesterday,
    Facade1Sun,
    Facade1Irradiance,
    Facade1Profile,
    Facade2Sun,
    Facade2Irradiance,
    Facade2Profile,
    Facade3Sun,
    Facade3Irradiance,
    Facade3Profile,
    Facade4Sun,
    Facade4Irradiance,
    Facade4Profile,
    Facade5Sun,
    Facade5Irradiance,
    Facade5Profile,
    Facade6Sun,
    Facade6Irradiance,
    Facade6Profile,
    Count
};

enum class WxSensor : uint8_t
{
    None,
    Sds,
    OneWire
};

struct WxDptOption
{
    uint8_t main;   // Hauptgruppe fuer den knx-Stack
    uint16_t sub;   // Untergruppe fuer den knx-Stack (Kodierung, nicht ETS-DPT)
    float factor;   // Umrechnung Messwert -> Buswert
};

struct WxValueDef
{
    const char *name;     // Anzeige (Konsole, ASCII)
    const char *label;    // Anzeige (Webseite, UTF-8)
    const char *group;    // ETS-Seite (Webseite, UTF-8)
    const char *unitUtf8; // Einheit (Webseite, UTF-8)
    const char *mqttKey;  // Feldname im MQTT-JSON
    const char *influxKey; // Feldname in InfluxDB (nullptr = nicht schreiben)
    const char *unit;
    uint16_t paramOffset; // erstes Byte des Parameterblocks
    uint16_t koNumber;
    WxSensor sensor;
    uint16_t condParam;   // Parameter, der aktiv sein muss (Bit 0), 0xFFFF = keiner
    bool thresholds;      // false: bei jeder Aenderung senden
    uint8_t decimals;
    const char *haClass;  // Home Assistant device_class (nullptr = keine)
    const char *haUnit;   // Home Assistant unit_of_measurement (nullptr = keine)
    uint8_t dptCount;
    WxDptOption dpts[3];
};

#define WX_VALUE_COUNT 87
#define WX_KO_FAULT_WN90 (WX_KoFaultWn90)
#define WX_KO_FAULT_SDS (WX_KoFaultSds)
#define WX_KO_FAULT_1W (WX_KoFault1W)

static const WxValueDef WX_VALUES[WX_VALUE_COUNT] = {
    {"Temperatur", "Temperatur", "Temperatur", "\u00b0C", "temperature", "temp", "C", WX_TempActive, WX_KoTemp, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur 1-Wire", "Temperatur 1-Wire", "Temperatur", "\u00b0C", "temperature1", "temp", "C", WX_Temp1WActive, WX_KoTemp1W, WxSensor::OneWire, 0xFFFF, true, 2, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Relative Luftfeuchte", "Relative Luftfeuchte", "Luftfeuchte", "%", "humidity", "humidity", "%", WX_HumidityActive, WX_KoHumidity, WxSensor::None, 0xFFFF, true, 0, "humidity", "%", 2, {{9, 1, 1.0f}, {5, 1, 1.0f}}},
    {"Absolute Luftfeuchte", "Absolute Luftfeuchte", "Luftfeuchte", "g/m\u00b3", "humidity_abs", "humidity_abs", "g/m3", WX_AbsHumidityActive, WX_KoAbsHumidity, WxSensor::None, 0xFFFF, true, 2, "absolute_humidity", "g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Taupunkt", "Taupunkt", "Luftfeuchte", "\u00b0C", "dewpoint", "dewpoint", "C", WX_DewPointActive, WX_KoDewPoint, WxSensor::None, 0xFFFF, true, 2, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Frostpunkt", "Frostpunkt", "Luftfeuchte", "\u00b0C", "frostpoint", "frostpoint", "C", WX_FrostPointActive, WX_KoFrostPoint, WxSensor::None, 0xFFFF, true, 2, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Humidex", "Humidex", "Behaglichkeit", "\u00b0C", "humidex", "humidex", "C", WX_HumidexActive, WX_KoHumidex, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Hitzeindex", "Hitzeindex", "Behaglichkeit", "\u00b0C", "heatindex", "heatindex", "C", WX_HeatIndexActive, WX_KoHeatIndex, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Windchill", "Windchill", "Behaglichkeit", "\u00b0C", "windchill", "windchill", "C", WX_WindChillActive, WX_KoWindChill, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Discomfort-Index", "Discomfort-Index", "Behaglichkeit", "\u00b0C", "discomfort", "discomfort", "C", WX_DiscomfortActive, WX_KoDiscomfort, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Windgeschwindigkeit", "Windgeschwindigkeit", "Wind", "m/s", "windspeed", "windspeed", "m/s", WX_WindSpeedActive, WX_KoWindSpeed, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Boeengeschwindigkeit", "B\u00f6engeschwindigkeit", "Wind", "m/s", "gustspeed", "gustspeed", "m/s", WX_GustSpeedActive, WX_KoGustSpeed, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Windstaerke (Beaufort)", "Windst\u00e4rke (Beaufort)", "Wind", "Bft", "windspeed_bft", "windspeed_bft", "Bft", WX_WindBftActive, WX_KoWindBft, WxSensor::None, 0xFFFF, true, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Boeenstaerke (Beaufort)", "B\u00f6enst\u00e4rke (Beaufort)", "Wind", "Bft", "gustspeed_bft", "gustspeed_bft", "Bft", WX_GustBftActive, WX_KoGustBft, WxSensor::None, 0xFFFF, true, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Windrichtung", "Windrichtung", "Wind", "\u00b0", "winddirection", "winddirection", "Grad", WX_WindDirActive, WX_KoWindDir, WxSensor::None, 0xFFFF, true, 0, nullptr, "\u00b0", 2, {{5, 3, 1.0f}, {14, 0, 1.0f}}},
    {"Luftdruck (absolut)", "Luftdruck (absolut)", "Luftdruck", "hPa", "pressure", "pressure", "hPa", WX_PressureActive, WX_KoPressure, WxSensor::None, 0xFFFF, true, 1, "atmospheric_pressure", "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdruck (auf Meereshoehe)", "Luftdruck (auf Meeresh\u00f6he)", "Luftdruck", "hPa", "pressure_sea", "pressure_sea", "hPa", WX_PressureSeaActive, WX_KoPressureSea, WxSensor::None, 0xFFFF, true, 1, "atmospheric_pressure", "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdrucktendenz 1 Stunde", "Luftdrucktendenz 1 Stunde", "Luftdruck", "hPa", "pressuretrend1", "pressureTrend1", "hPa", WX_Trend1hActive, WX_KoTrend1h, WxSensor::None, 0xFFFF, true, 1, nullptr, "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdrucktendenz 3 Stunden", "Luftdrucktendenz 3 Stunden", "Luftdruck", "hPa", "pressuretrend3", "pressureTrend3", "hPa", WX_Trend3hActive, WX_KoTrend3h, WxSensor::None, 0xFFFF, true, 1, nullptr, "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Helligkeit", "Helligkeit", "Helligkeit und UV", "lx", "light", "brightness", "lx", WX_LightActive, WX_KoLight, WxSensor::None, 0xFFFF, true, 0, "illuminance", "lx", 1, {{9, 1, 1.0f}}},
    {"UV-Index", "UV-Index", "Helligkeit und UV", "", "uvindex", "uvindex", "", WX_UvIndexActive, WX_KoUvIndex, WxSensor::None, 0xFFFF, true, 1, nullptr, nullptr, 2, {{9, 1, 1.0f}, {5, 10, 1.0f}}},
    {"Regenmenge gesamt", "Regenmenge gesamt", "Regen", "mm", "raincounter", "raincounter", "mm", WX_RainTotalActive, WX_KoRainTotal, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Regenintensitaet", "Regenintensit\u00e4t", "Regen", "mm/h", "rain_rate", "rain_rate", "mm/h", WX_RainRateActive, WX_KoRainRate, WxSensor::None, 0xFFFF, true, 1, "precipitation_intensity", "mm/h", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 5 Minuten", "Regenmenge letzte 5 Minuten", "Regen", "mm", "rain_5min", "raincounter_5min", "mm", WX_Rain5mActive, WX_KoRain5m, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte Stunde", "Regenmenge letzte Stunde", "Regen", "mm", "rain_1h", "raincounter_1h", "mm", WX_Rain1hActive, WX_KoRain1h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 6 Stunden", "Regenmenge letzte 6 Stunden", "Regen", "mm", "rain_6h", "raincounter_6h", "mm", WX_Rain6hActive, WX_KoRain6h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 24 Stunden", "Regenmenge letzte 24 Stunden", "Regen", "mm", "rain_24h", "raincounter_24h", "mm", WX_Rain24hActive, WX_KoRain24h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 48 Stunden", "Regenmenge letzte 48 Stunden", "Regen", "mm", "rain_48h", "raincounter_48h", "mm", WX_Rain48hActive, WX_KoRain48h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 72 Stunden", "Regenmenge letzte 72 Stunden", "Regen", "mm", "rain_72h", "raincounter_72h", "mm", WX_Rain72hActive, WX_KoRain72h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge heute", "Regenmenge heute", "Regen", "mm", "rain_today", "raincounter_today", "mm", WX_RainTodayActive, WX_KoRainToday, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regen", "Regen", "Regen", "", "raining", "raining", "", WX_RainingActive, WX_KoRaining, WxSensor::None, 0xFFFF, false, 0, "moisture", nullptr, 1, {{1, 2, 1.0f}}},
    {"Feinstaub PM2,5", "Feinstaub PM2,5", "Feinstaub", "\u00b5g/m\u00b3", "pm25", "pm25", "ug/m3", WX_Pm25Active, WX_KoPm25, WxSensor::Sds, 0xFFFF, true, 1, "pm25", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM10", "Feinstaub PM10", "Feinstaub", "\u00b5g/m\u00b3", "pm10", "pm10", "ug/m3", WX_Pm10Active, WX_KoPm10, WxSensor::Sds, 0xFFFF, true, 1, "pm10", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM2,5 (feuchtekorrigiert)", "Feinstaub PM2,5 (feuchtekorrigiert)", "Feinstaub", "\u00b5g/m\u00b3", "pm25_normalized", "pm25_normalized", "ug/m3", WX_Pm25NormActive, WX_KoPm25Norm, WxSensor::Sds, 0xFFFF, true, 1, "pm25", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM10 (feuchtekorrigiert)", "Feinstaub PM10 (feuchtekorrigiert)", "Feinstaub", "\u00b5g/m\u00b3", "pm10_normalized", "pm10_normalized", "ug/m3", WX_Pm10NormActive, WX_KoPm10Norm, WxSensor::Sds, 0xFFFF, true, 1, "pm10", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Wettertendenz", "Wettertendenz", "Prognose und Display", "", "weather_tendency", "weather_tendency", "", WX_TendencyActive, WX_KoTendency, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{6, 10, 1.0f}}},
    {"Wetterprognose (Zambretti-Nummer)", "Wetterprognose (Zambretti-Nummer)", "Prognose und Display", "", "forecast", "forecast", "", WX_ForecastActive, WX_KoForecast, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Wetterprognose (Text)", "Wetterprognose (Text)", "Prognose und Display", "", "forecast_text", nullptr, "", WX_ForecastTextActive, WX_KoForecastText, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{16, 1, 1.0f}}},
    {"Wettersymbol (Display)", "Wettersymbol (Display)", "Prognose und Display", "", "weather_symbol", "weather_symbol", "", WX_WeatherSymbolActive, WX_KoWeatherSymbol, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Windgeschwindigkeit (10-min-Mittel)", "Windgeschwindigkeit (10-min-Mittel)", "Wind", "m/s", "windspeed_avg10", "windspeed_avg10", "m/s", WX_WindAvg10Active, WX_KoWindAvg10, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Boee (Maximum 10 min)", "B\u00f6e (Maximum 10 min)", "Wind", "m/s", "gust_max10", "gust_max10", "m/s", WX_GustMax10Active, WX_KoGustMax10, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Windalarm Stufe 1", "Windalarm Stufe 1", "Windalarm", "", "wind_alarm1", "wind_alarm1", "", WX_WindAlarm1Active, WX_KoWindAlarm1, WxSensor::None, 0xFFFF, false, 0, "safety", nullptr, 1, {{1, 5, 1.0f}}},
    {"Windalarm Stufe 2", "Windalarm Stufe 2", "Windalarm", "", "wind_alarm2", "wind_alarm2", "", WX_WindAlarm2Active, WX_KoWindAlarm2, WxSensor::None, 0xFFFF, false, 0, "safety", nullptr, 1, {{1, 5, 1.0f}}},
    {"Windalarm Stufe 3", "Windalarm Stufe 3", "Windalarm", "", "wind_alarm3", "wind_alarm3", "", WX_WindAlarm3Active, WX_KoWindAlarm3, WxSensor::None, 0xFFFF, false, 0, "safety", nullptr, 1, {{1, 5, 1.0f}}},
    {"Sonnenstand Azimut", "Sonnenstand Azimut", "Sonne und D\u00e4mmerung", "\u00b0", "sun_azimuth", "sun_azimuth", "Grad", WX_SunAzimuthActive, WX_KoSunAzimuth, WxSensor::None, 0xFFFF, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonnenstand Elevation", "Sonnenstand Elevation", "Sonne und D\u00e4mmerung", "\u00b0", "sun_elevation", "sun_elevation", "Grad", WX_SunElevationActive, WX_KoSunElevation, WxSensor::None, 0xFFFF, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Tag/Nacht (Daemmerung)", "Tag/Nacht (D\u00e4mmerung)", "Sonne und D\u00e4mmerung", "", "night", "night", "", WX_DayNightActive, WX_KoDayNight, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Temperatur Minimum heute", "Temperatur Minimum heute", "Minimum und Maximum", "\u00b0C", "temp_min_today", "temp_min_today", "C", WX_TempMinTodayActive, WX_KoTempMinToday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur Maximum heute", "Temperatur Maximum heute", "Minimum und Maximum", "\u00b0C", "temp_max_today", "temp_max_today", "C", WX_TempMaxTodayActive, WX_KoTempMaxToday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur Minimum gestern", "Temperatur Minimum gestern", "Minimum und Maximum", "\u00b0C", "temp_min_yesterday", "temp_min_yesterday", "C", WX_TempMinYesterdayActive, WX_KoTempMinYesterday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur Maximum gestern", "Temperatur Maximum gestern", "Minimum und Maximum", "\u00b0C", "temp_max_yesterday", "temp_max_yesterday", "C", WX_TempMaxYesterdayActive, WX_KoTempMaxYesterday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Staerkste Boee heute", "St\u00e4rkste B\u00f6e heute", "Minimum und Maximum", "m/s", "gust_max_today", "gust_max_today", "m/s", WX_GustMaxTodayActive, WX_KoGustMaxToday, WxSensor::None, 0xFFFF, false, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Staerkste Boee gestern", "St\u00e4rkste B\u00f6e gestern", "Minimum und Maximum", "m/s", "gust_max_yesterday", "gust_max_yesterday", "m/s", WX_GustMaxYesterdayActive, WX_KoGustMaxYesterday, WxSensor::None, 0xFFFF, false, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Frost", "Frost", "Warnungen", "", "frost", "frost", "", WX_FrostActive, WX_KoFrost, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 5, 1.0f}}},
    {"Glaettegefahr", "Gl\u00e4ttegefahr", "Warnungen", "", "ice_warning", "ice_warning", "", WX_IceWarningActive, WX_KoIceWarning, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 5, 1.0f}}},
    {"Sommer/Winter", "Sommer/Winter", "Sommer/Winter und Kenntage", "", "summer", "summer", "", WX_SummerWinterActive, WX_KoSummerWinter, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 2, {{1, 2, 1.0f}, {1, 2, -1.0f}}},
    {"Mittlere Aussentemperatur (mehrere Tage)", "Mittlere Au\u00dfentemperatur (mehrere Tage)", "Sommer/Winter und Kenntage", "\u00b0C", "temp_mean_days", "temp_mean_days", "C", WX_MeanTempActive, WX_KoMeanTemp, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Tagesmitteltemperatur gestern", "Tagesmitteltemperatur gestern", "Sommer/Winter und Kenntage", "\u00b0C", "temp_mean_yesterday", "temp_mean_yesterday", "C", WX_DailyMeanYesterdayActive, WX_KoDailyMeanYesterday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Sommertag (Maximum ab 25 °C)", "Sommertag (Maximum ab 25 \u00b0C)", "Sommer/Winter und Kenntage", "", "summer_day", "summer_day", "", WX_SummerDayActive, WX_KoSummerDay, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Heisser Tag (Maximum ab 30 °C)", "Hei\u00dfer Tag (Maximum ab 30 \u00b0C)", "Sommer/Winter und Kenntage", "", "hot_day", "hot_day", "", WX_HotDayActive, WX_KoHotDay, WxSensor::None, 0xFFFF, false, 0, "heat", nullptr, 1, {{1, 2, 1.0f}}},
    {"Frosttag (Minimum unter 0 °C)", "Frosttag (Minimum unter 0 \u00b0C)", "Sommer/Winter und Kenntage", "", "frost_day", "frost_day", "", WX_FrostDayActive, WX_KoFrostDay, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 2, 1.0f}}},
    {"Eistag (Maximum unter 0 °C)", "Eistag (Maximum unter 0 \u00b0C)", "Sommer/Winter und Kenntage", "", "ice_day", "ice_day", "", WX_IceDayActive, WX_KoIceDay, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 2, 1.0f}}},
    {"Globalstrahlung (geschaetzt)", "Globalstrahlung (gesch\u00e4tzt)", "Strahlung und PV", "W/m\u00b2", "global_radiation", "global_radiation", "W/m²", WX_GlobalRadiationActive, WX_KoGlobalRadiation, WxSensor::None, 0xFFFF, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Bewoelkungsgrad (geschaetzt)", "Bew\u00f6lkungsgrad (gesch\u00e4tzt)", "Strahlung und PV", "%", "cloud_cover", "cloud_cover", "%", WX_CloudCoverActive, WX_KoCloudCover, WxSensor::None, 0xFFFF, true, 0, nullptr, "%", 1, {{5, 1, 1.0f}}},
    {"Einstrahlung heute", "Einstrahlung heute", "Strahlung und PV", "kWh/m\u00b2", "radiation_today", "radiation_today", "kWh/m²", WX_RadiationTodayActive, WX_KoRadiationToday, WxSensor::None, 0xFFFF, true, 2, nullptr, "kWh/m\u00b2", 1, {{14, 0, 1.0f}}},
    {"Einstrahlung gestern", "Einstrahlung gestern", "Strahlung und PV", "kWh/m\u00b2", "radiation_yesterday", "radiation_yesterday", "kWh/m²", WX_RadiationYesterdayActive, WX_KoRadiationYesterday, WxSensor::None, 0xFFFF, false, 2, nullptr, "kWh/m\u00b2", 1, {{14, 0, 1.0f}}},
    {"PV-Leistung (geschaetzt)", "PV-Leistung (gesch\u00e4tzt)", "Strahlung und PV", "W", "pv_power", "pv_power", "W", WX_PvPowerActive, WX_KoPvPower, WxSensor::None, 0xFFFF, true, 0, "power", "W", 1, {{14, 0, 1.0f}}},
    {"PV-Ertrag heute (geschaetzt)", "PV-Ertrag heute (gesch\u00e4tzt)", "Strahlung und PV", "kWh", "pv_energy_today", "pv_energy_today", "kWh", WX_PvEnergyTodayActive, WX_KoPvEnergyToday, WxSensor::None, 0xFFFF, true, 2, "energy", "kWh", 2, {{13, 10, 1000.0f}, {14, 0, 1.0f}}},
    {"PV-Ertrag gestern (geschaetzt)", "PV-Ertrag gestern (gesch\u00e4tzt)", "Strahlung und PV", "kWh", "pv_energy_yesterday", "pv_energy_yesterday", "kWh", WX_PvEnergyYesterdayActive, WX_KoPvEnergyYesterday, WxSensor::None, 0xFFFF, false, 2, "energy", "kWh", 2, {{13, 10, 1000.0f}, {14, 0, 1.0f}}},
    {"Sonne auf Fassade 1", "Sonne auf Fassade 1", "Fassaden", "", "facade1_sun", "facade1_sun", "", WX_Facade1SunActive, WX_KoFacade1Sun, WxSensor::None, WX_Facade1Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 1 (geschaetzt)", "Einstrahlung Fassade 1 (gesch\u00e4tzt)", "Fassaden", "W/m\u00b2", "facade1_irradiance", "facade1_irradiance", "W/m²", WX_Facade1IrradianceActive, WX_KoFacade1Irradiance, WxSensor::None, WX_Facade1Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 1", "Profilwinkel Fassade 1", "Fassaden", "\u00b0", "facade1_profile", "facade1_profile", "Grad", WX_Facade1ProfileActive, WX_KoFacade1Profile, WxSensor::None, WX_Facade1Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 2", "Sonne auf Fassade 2", "Fassaden", "", "facade2_sun", "facade2_sun", "", WX_Facade2SunActive, WX_KoFacade2Sun, WxSensor::None, WX_Facade2Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 2 (geschaetzt)", "Einstrahlung Fassade 2 (gesch\u00e4tzt)", "Fassaden", "W/m\u00b2", "facade2_irradiance", "facade2_irradiance", "W/m²", WX_Facade2IrradianceActive, WX_KoFacade2Irradiance, WxSensor::None, WX_Facade2Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 2", "Profilwinkel Fassade 2", "Fassaden", "\u00b0", "facade2_profile", "facade2_profile", "Grad", WX_Facade2ProfileActive, WX_KoFacade2Profile, WxSensor::None, WX_Facade2Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 3", "Sonne auf Fassade 3", "Fassaden", "", "facade3_sun", "facade3_sun", "", WX_Facade3SunActive, WX_KoFacade3Sun, WxSensor::None, WX_Facade3Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 3 (geschaetzt)", "Einstrahlung Fassade 3 (gesch\u00e4tzt)", "Fassaden", "W/m\u00b2", "facade3_irradiance", "facade3_irradiance", "W/m²", WX_Facade3IrradianceActive, WX_KoFacade3Irradiance, WxSensor::None, WX_Facade3Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 3", "Profilwinkel Fassade 3", "Fassaden", "\u00b0", "facade3_profile", "facade3_profile", "Grad", WX_Facade3ProfileActive, WX_KoFacade3Profile, WxSensor::None, WX_Facade3Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 4", "Sonne auf Fassade 4", "Fassaden", "", "facade4_sun", "facade4_sun", "", WX_Facade4SunActive, WX_KoFacade4Sun, WxSensor::None, WX_Facade4Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 4 (geschaetzt)", "Einstrahlung Fassade 4 (gesch\u00e4tzt)", "Fassaden", "W/m\u00b2", "facade4_irradiance", "facade4_irradiance", "W/m²", WX_Facade4IrradianceActive, WX_KoFacade4Irradiance, WxSensor::None, WX_Facade4Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 4", "Profilwinkel Fassade 4", "Fassaden", "\u00b0", "facade4_profile", "facade4_profile", "Grad", WX_Facade4ProfileActive, WX_KoFacade4Profile, WxSensor::None, WX_Facade4Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 5", "Sonne auf Fassade 5", "Fassaden", "", "facade5_sun", "facade5_sun", "", WX_Facade5SunActive, WX_KoFacade5Sun, WxSensor::None, WX_Facade5Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 5 (geschaetzt)", "Einstrahlung Fassade 5 (gesch\u00e4tzt)", "Fassaden", "W/m\u00b2", "facade5_irradiance", "facade5_irradiance", "W/m²", WX_Facade5IrradianceActive, WX_KoFacade5Irradiance, WxSensor::None, WX_Facade5Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 5", "Profilwinkel Fassade 5", "Fassaden", "\u00b0", "facade5_profile", "facade5_profile", "Grad", WX_Facade5ProfileActive, WX_KoFacade5Profile, WxSensor::None, WX_Facade5Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 6", "Sonne auf Fassade 6", "Fassaden", "", "facade6_sun", "facade6_sun", "", WX_Facade6SunActive, WX_KoFacade6Sun, WxSensor::None, WX_Facade6Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 6 (geschaetzt)", "Einstrahlung Fassade 6 (gesch\u00e4tzt)", "Fassaden", "W/m\u00b2", "facade6_irradiance", "facade6_irradiance", "W/m²", WX_Facade6IrradianceActive, WX_KoFacade6Irradiance, WxSensor::None, WX_Facade6Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 6", "Profilwinkel Fassade 6", "Fassaden", "\u00b0", "facade6_profile", "facade6_profile", "Grad", WX_Facade6ProfileActive, WX_KoFacade6Profile, WxSensor::None, WX_Facade6Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
};

