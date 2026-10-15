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
    const char *name;     // Anzeige (Konsole)
    const char *mqttKey;  // Feldname im MQTT-JSON
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
    {"Temperatur", "temperature", "C", WX_TempActive, WX_KoTemp, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur 1-Wire", "temperature1", "C", WX_Temp1WActive, WX_KoTemp1W, WxSensor::OneWire, 0xFFFF, true, 2, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Relative Luftfeuchte", "humidity", "%", WX_HumidityActive, WX_KoHumidity, WxSensor::None, 0xFFFF, true, 0, "humidity", "%", 2, {{9, 1, 1.0f}, {5, 1, 1.0f}}},
    {"Absolute Luftfeuchte", "humidity_abs", "g/m3", WX_AbsHumidityActive, WX_KoAbsHumidity, WxSensor::None, 0xFFFF, true, 2, "absolute_humidity", "g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Taupunkt", "dewpoint", "C", WX_DewPointActive, WX_KoDewPoint, WxSensor::None, 0xFFFF, true, 2, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Frostpunkt", "frostpoint", "C", WX_FrostPointActive, WX_KoFrostPoint, WxSensor::None, 0xFFFF, true, 2, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Humidex", "humidex", "C", WX_HumidexActive, WX_KoHumidex, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Hitzeindex", "heatindex", "C", WX_HeatIndexActive, WX_KoHeatIndex, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Windchill", "windchill", "C", WX_WindChillActive, WX_KoWindChill, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Discomfort-Index", "discomfort", "C", WX_DiscomfortActive, WX_KoDiscomfort, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Windgeschwindigkeit", "windspeed", "m/s", WX_WindSpeedActive, WX_KoWindSpeed, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Boeengeschwindigkeit", "gustspeed", "m/s", WX_GustSpeedActive, WX_KoGustSpeed, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Windstaerke (Beaufort)", "windspeed_bft", "Bft", WX_WindBftActive, WX_KoWindBft, WxSensor::None, 0xFFFF, true, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Boeenstaerke (Beaufort)", "gustspeed_bft", "Bft", WX_GustBftActive, WX_KoGustBft, WxSensor::None, 0xFFFF, true, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Windrichtung", "winddirection", "Grad", WX_WindDirActive, WX_KoWindDir, WxSensor::None, 0xFFFF, true, 0, nullptr, "\u00b0", 2, {{5, 3, 1.0f}, {14, 0, 1.0f}}},
    {"Luftdruck (absolut)", "pressure", "hPa", WX_PressureActive, WX_KoPressure, WxSensor::None, 0xFFFF, true, 1, "atmospheric_pressure", "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdruck (auf Meereshoehe)", "pressure_sea", "hPa", WX_PressureSeaActive, WX_KoPressureSea, WxSensor::None, 0xFFFF, true, 1, "atmospheric_pressure", "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdrucktendenz 1 Stunde", "pressuretrend1", "hPa", WX_Trend1hActive, WX_KoTrend1h, WxSensor::None, 0xFFFF, true, 1, nullptr, "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdrucktendenz 3 Stunden", "pressuretrend3", "hPa", WX_Trend3hActive, WX_KoTrend3h, WxSensor::None, 0xFFFF, true, 1, nullptr, "hPa", 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Helligkeit", "light", "lx", WX_LightActive, WX_KoLight, WxSensor::None, 0xFFFF, true, 0, "illuminance", "lx", 1, {{9, 1, 1.0f}}},
    {"UV-Index", "uvindex", "", WX_UvIndexActive, WX_KoUvIndex, WxSensor::None, 0xFFFF, true, 1, nullptr, nullptr, 2, {{9, 1, 1.0f}, {5, 10, 1.0f}}},
    {"Regenmenge gesamt", "raincounter", "mm", WX_RainTotalActive, WX_KoRainTotal, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Regenintensitaet", "rain_rate", "mm/h", WX_RainRateActive, WX_KoRainRate, WxSensor::None, 0xFFFF, true, 1, "precipitation_intensity", "mm/h", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 5 Minuten", "rain_5min", "mm", WX_Rain5mActive, WX_KoRain5m, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte Stunde", "rain_1h", "mm", WX_Rain1hActive, WX_KoRain1h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 6 Stunden", "rain_6h", "mm", WX_Rain6hActive, WX_KoRain6h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 24 Stunden", "rain_24h", "mm", WX_Rain24hActive, WX_KoRain24h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 48 Stunden", "rain_48h", "mm", WX_Rain48hActive, WX_KoRain48h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 72 Stunden", "rain_72h", "mm", WX_Rain72hActive, WX_KoRain72h, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regenmenge heute", "rain_today", "mm", WX_RainTodayActive, WX_KoRainToday, WxSensor::None, 0xFFFF, true, 2, "precipitation", "mm", 1, {{9, 1, 1.0f}}},
    {"Regen", "raining", "", WX_RainingActive, WX_KoRaining, WxSensor::None, 0xFFFF, false, 0, "moisture", nullptr, 1, {{1, 2, 1.0f}}},
    {"Feinstaub PM2,5", "pm25", "ug/m3", WX_Pm25Active, WX_KoPm25, WxSensor::Sds, 0xFFFF, true, 1, "pm25", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM10", "pm10", "ug/m3", WX_Pm10Active, WX_KoPm10, WxSensor::Sds, 0xFFFF, true, 1, "pm10", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM2,5 (feuchtekorrigiert)", "pm25_normalized", "ug/m3", WX_Pm25NormActive, WX_KoPm25Norm, WxSensor::Sds, 0xFFFF, true, 1, "pm25", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM10 (feuchtekorrigiert)", "pm10_normalized", "ug/m3", WX_Pm10NormActive, WX_KoPm10Norm, WxSensor::Sds, 0xFFFF, true, 1, "pm10", "\u00b5g/m\u00b3", 1, {{9, 1, 1.0f}}},
    {"Wettertendenz", "weather_tendency", "", WX_TendencyActive, WX_KoTendency, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{6, 10, 1.0f}}},
    {"Wetterprognose (Zambretti-Nummer)", "forecast", "", WX_ForecastActive, WX_KoForecast, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Wetterprognose (Text)", "forecast_text", "", WX_ForecastTextActive, WX_KoForecastText, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{16, 1, 1.0f}}},
    {"Wettersymbol (Display)", "weather_symbol", "", WX_WeatherSymbolActive, WX_KoWeatherSymbol, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{5, 10, 1.0f}}},
    {"Windgeschwindigkeit (10-min-Mittel)", "windspeed_avg10", "m/s", WX_WindAvg10Active, WX_KoWindAvg10, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Boee (Maximum 10 min)", "gust_max10", "m/s", WX_GustMax10Active, WX_KoGustMax10, WxSensor::None, 0xFFFF, true, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Windalarm Stufe 1", "wind_alarm1", "", WX_WindAlarm1Active, WX_KoWindAlarm1, WxSensor::None, 0xFFFF, false, 0, "safety", nullptr, 1, {{1, 5, 1.0f}}},
    {"Windalarm Stufe 2", "wind_alarm2", "", WX_WindAlarm2Active, WX_KoWindAlarm2, WxSensor::None, 0xFFFF, false, 0, "safety", nullptr, 1, {{1, 5, 1.0f}}},
    {"Windalarm Stufe 3", "wind_alarm3", "", WX_WindAlarm3Active, WX_KoWindAlarm3, WxSensor::None, 0xFFFF, false, 0, "safety", nullptr, 1, {{1, 5, 1.0f}}},
    {"Sonnenstand Azimut", "sun_azimuth", "Grad", WX_SunAzimuthActive, WX_KoSunAzimuth, WxSensor::None, 0xFFFF, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonnenstand Elevation", "sun_elevation", "Grad", WX_SunElevationActive, WX_KoSunElevation, WxSensor::None, 0xFFFF, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Tag/Nacht (Daemmerung)", "night", "", WX_DayNightActive, WX_KoDayNight, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Temperatur Minimum heute", "temp_min_today", "C", WX_TempMinTodayActive, WX_KoTempMinToday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur Maximum heute", "temp_max_today", "C", WX_TempMaxTodayActive, WX_KoTempMaxToday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur Minimum gestern", "temp_min_yesterday", "C", WX_TempMinYesterdayActive, WX_KoTempMinYesterday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur Maximum gestern", "temp_max_yesterday", "C", WX_TempMaxYesterdayActive, WX_KoTempMaxYesterday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Staerkste Boee heute", "gust_max_today", "m/s", WX_GustMaxTodayActive, WX_KoGustMaxToday, WxSensor::None, 0xFFFF, false, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Staerkste Boee gestern", "gust_max_yesterday", "m/s", WX_GustMaxYesterdayActive, WX_KoGustMaxYesterday, WxSensor::None, 0xFFFF, false, 1, "wind_speed", "m/s", 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Frost", "frost", "", WX_FrostActive, WX_KoFrost, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 5, 1.0f}}},
    {"Glaettegefahr", "ice_warning", "", WX_IceWarningActive, WX_KoIceWarning, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 5, 1.0f}}},
    {"Sommer/Winter", "summer", "", WX_SummerWinterActive, WX_KoSummerWinter, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 2, {{1, 2, 1.0f}, {1, 2, -1.0f}}},
    {"Mittlere Aussentemperatur (mehrere Tage)", "temp_mean_days", "C", WX_MeanTempActive, WX_KoMeanTemp, WxSensor::None, 0xFFFF, true, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Tagesmitteltemperatur gestern", "temp_mean_yesterday", "C", WX_DailyMeanYesterdayActive, WX_KoDailyMeanYesterday, WxSensor::None, 0xFFFF, false, 1, "temperature", "\u00b0C", 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Sommertag (Maximum ab 25 °C)", "summer_day", "", WX_SummerDayActive, WX_KoSummerDay, WxSensor::None, 0xFFFF, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Heisser Tag (Maximum ab 30 °C)", "hot_day", "", WX_HotDayActive, WX_KoHotDay, WxSensor::None, 0xFFFF, false, 0, "heat", nullptr, 1, {{1, 2, 1.0f}}},
    {"Frosttag (Minimum unter 0 °C)", "frost_day", "", WX_FrostDayActive, WX_KoFrostDay, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 2, 1.0f}}},
    {"Eistag (Maximum unter 0 °C)", "ice_day", "", WX_IceDayActive, WX_KoIceDay, WxSensor::None, 0xFFFF, false, 0, "cold", nullptr, 1, {{1, 2, 1.0f}}},
    {"Globalstrahlung (geschaetzt)", "global_radiation", "W/m²", WX_GlobalRadiationActive, WX_KoGlobalRadiation, WxSensor::None, 0xFFFF, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Bewoelkungsgrad (geschaetzt)", "cloud_cover", "%", WX_CloudCoverActive, WX_KoCloudCover, WxSensor::None, 0xFFFF, true, 0, nullptr, "%", 1, {{5, 1, 1.0f}}},
    {"Einstrahlung heute", "radiation_today", "kWh/m²", WX_RadiationTodayActive, WX_KoRadiationToday, WxSensor::None, 0xFFFF, true, 2, nullptr, "kWh/m\u00b2", 1, {{14, 0, 1.0f}}},
    {"Einstrahlung gestern", "radiation_yesterday", "kWh/m²", WX_RadiationYesterdayActive, WX_KoRadiationYesterday, WxSensor::None, 0xFFFF, false, 2, nullptr, "kWh/m\u00b2", 1, {{14, 0, 1.0f}}},
    {"PV-Leistung (geschaetzt)", "pv_power", "W", WX_PvPowerActive, WX_KoPvPower, WxSensor::None, 0xFFFF, true, 0, "power", "W", 1, {{14, 0, 1.0f}}},
    {"PV-Ertrag heute (geschaetzt)", "pv_energy_today", "kWh", WX_PvEnergyTodayActive, WX_KoPvEnergyToday, WxSensor::None, 0xFFFF, true, 2, "energy", "kWh", 2, {{13, 10, 1000.0f}, {14, 0, 1.0f}}},
    {"PV-Ertrag gestern (geschaetzt)", "pv_energy_yesterday", "kWh", WX_PvEnergyYesterdayActive, WX_KoPvEnergyYesterday, WxSensor::None, 0xFFFF, false, 2, "energy", "kWh", 2, {{13, 10, 1000.0f}, {14, 0, 1.0f}}},
    {"Sonne auf Fassade 1", "facade1_sun", "", WX_Facade1SunActive, WX_KoFacade1Sun, WxSensor::None, WX_Facade1Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 1 (geschaetzt)", "facade1_irradiance", "W/m²", WX_Facade1IrradianceActive, WX_KoFacade1Irradiance, WxSensor::None, WX_Facade1Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 1", "facade1_profile", "Grad", WX_Facade1ProfileActive, WX_KoFacade1Profile, WxSensor::None, WX_Facade1Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 2", "facade2_sun", "", WX_Facade2SunActive, WX_KoFacade2Sun, WxSensor::None, WX_Facade2Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 2 (geschaetzt)", "facade2_irradiance", "W/m²", WX_Facade2IrradianceActive, WX_KoFacade2Irradiance, WxSensor::None, WX_Facade2Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 2", "facade2_profile", "Grad", WX_Facade2ProfileActive, WX_KoFacade2Profile, WxSensor::None, WX_Facade2Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 3", "facade3_sun", "", WX_Facade3SunActive, WX_KoFacade3Sun, WxSensor::None, WX_Facade3Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 3 (geschaetzt)", "facade3_irradiance", "W/m²", WX_Facade3IrradianceActive, WX_KoFacade3Irradiance, WxSensor::None, WX_Facade3Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 3", "facade3_profile", "Grad", WX_Facade3ProfileActive, WX_KoFacade3Profile, WxSensor::None, WX_Facade3Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 4", "facade4_sun", "", WX_Facade4SunActive, WX_KoFacade4Sun, WxSensor::None, WX_Facade4Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 4 (geschaetzt)", "facade4_irradiance", "W/m²", WX_Facade4IrradianceActive, WX_KoFacade4Irradiance, WxSensor::None, WX_Facade4Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 4", "facade4_profile", "Grad", WX_Facade4ProfileActive, WX_KoFacade4Profile, WxSensor::None, WX_Facade4Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 5", "facade5_sun", "", WX_Facade5SunActive, WX_KoFacade5Sun, WxSensor::None, WX_Facade5Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 5 (geschaetzt)", "facade5_irradiance", "W/m²", WX_Facade5IrradianceActive, WX_KoFacade5Irradiance, WxSensor::None, WX_Facade5Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 5", "facade5_profile", "Grad", WX_Facade5ProfileActive, WX_KoFacade5Profile, WxSensor::None, WX_Facade5Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
    {"Sonne auf Fassade 6", "facade6_sun", "", WX_Facade6SunActive, WX_KoFacade6Sun, WxSensor::None, WX_Facade6Active, false, 0, nullptr, nullptr, 1, {{1, 2, 1.0f}}},
    {"Einstrahlung Fassade 6 (geschaetzt)", "facade6_irradiance", "W/m²", WX_Facade6IrradianceActive, WX_KoFacade6Irradiance, WxSensor::None, WX_Facade6Active, true, 0, "irradiance", "W/m\u00b2", 1, {{9, 1, 1.0f}}},
    {"Profilwinkel Fassade 6", "facade6_profile", "Grad", WX_Facade6ProfileActive, WX_KoFacade6Profile, WxSensor::None, WX_Facade6Active, true, 1, nullptr, "\u00b0", 1, {{14, 0, 1.0f}}},
};

