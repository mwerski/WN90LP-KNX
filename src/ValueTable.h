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
    bool thresholds;      // false: bei jeder Aenderung senden
    uint8_t decimals;
    uint8_t dptCount;
    WxDptOption dpts[3];
};

#define WX_VALUE_COUNT 39
#define WX_KO_FAULT_WN90 (WX_KoFaultWn90)
#define WX_KO_FAULT_SDS (WX_KoFaultSds)
#define WX_KO_FAULT_1W (WX_KoFault1W)

static const WxValueDef WX_VALUES[WX_VALUE_COUNT] = {
    {"Temperatur", "temperature", "C", WX_TempActive, WX_KoTemp, WxSensor::None, true, 1, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Temperatur 1-Wire", "temperature1", "C", WX_Temp1WActive, WX_KoTemp1W, WxSensor::OneWire, true, 2, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Relative Luftfeuchte", "humidity", "%", WX_HumidityActive, WX_KoHumidity, WxSensor::None, true, 0, 2, {{9, 1, 1.0f}, {5, 1, 1.0f}}},
    {"Absolute Luftfeuchte", "humidity_abs", "g/m3", WX_AbsHumidityActive, WX_KoAbsHumidity, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Taupunkt", "dewpoint", "C", WX_DewPointActive, WX_KoDewPoint, WxSensor::None, true, 2, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Frostpunkt", "frostpoint", "C", WX_FrostPointActive, WX_KoFrostPoint, WxSensor::None, true, 2, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Humidex", "humidex", "C", WX_HumidexActive, WX_KoHumidex, WxSensor::None, true, 1, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Hitzeindex", "heatindex", "C", WX_HeatIndexActive, WX_KoHeatIndex, WxSensor::None, true, 1, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Windchill", "windchill", "C", WX_WindChillActive, WX_KoWindChill, WxSensor::None, true, 1, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Discomfort-Index", "discomfort", "C", WX_DiscomfortActive, WX_KoDiscomfort, WxSensor::None, true, 1, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Windgeschwindigkeit", "windspeed", "m/s", WX_WindSpeedActive, WX_KoWindSpeed, WxSensor::None, true, 1, 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Boeengeschwindigkeit", "gustspeed", "m/s", WX_GustSpeedActive, WX_KoGustSpeed, WxSensor::None, true, 1, 3, {{9, 1, 1.0f}, {9, 1, 3.6f}, {14, 0, 1.0f}}},
    {"Windstaerke (Beaufort)", "windspeed_bft", "Bft", WX_WindBftActive, WX_KoWindBft, WxSensor::None, true, 0, 1, {{5, 10, 1.0f}}},
    {"Boeenstaerke (Beaufort)", "gustspeed_bft", "Bft", WX_GustBftActive, WX_KoGustBft, WxSensor::None, true, 0, 1, {{5, 10, 1.0f}}},
    {"Windrichtung", "winddirection", "Grad", WX_WindDirActive, WX_KoWindDir, WxSensor::None, true, 0, 2, {{5, 3, 1.0f}, {14, 0, 1.0f}}},
    {"Luftdruck (absolut)", "pressure", "hPa", WX_PressureActive, WX_KoPressure, WxSensor::None, true, 1, 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdruck (auf Meereshoehe)", "pressure_sea", "hPa", WX_PressureSeaActive, WX_KoPressureSea, WxSensor::None, true, 1, 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdrucktendenz 1 Stunde", "pressuretrend1", "hPa", WX_Trend1hActive, WX_KoTrend1h, WxSensor::None, true, 1, 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Luftdrucktendenz 3 Stunden", "pressuretrend3", "hPa", WX_Trend3hActive, WX_KoTrend3h, WxSensor::None, true, 1, 2, {{9, 1, 100.0f}, {14, 0, 100.0f}}},
    {"Helligkeit", "light", "lx", WX_LightActive, WX_KoLight, WxSensor::None, true, 0, 1, {{9, 1, 1.0f}}},
    {"UV-Index", "uvindex", "", WX_UvIndexActive, WX_KoUvIndex, WxSensor::None, true, 1, 2, {{9, 1, 1.0f}, {5, 10, 1.0f}}},
    {"Regenmenge gesamt", "raincounter", "mm", WX_RainTotalActive, WX_KoRainTotal, WxSensor::None, true, 2, 2, {{9, 1, 1.0f}, {14, 0, 1.0f}}},
    {"Regenintensitaet", "rain_rate", "mm/h", WX_RainRateActive, WX_KoRainRate, WxSensor::None, true, 1, 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 5 Minuten", "rain_5min", "mm", WX_Rain5mActive, WX_KoRain5m, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte Stunde", "rain_1h", "mm", WX_Rain1hActive, WX_KoRain1h, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 6 Stunden", "rain_6h", "mm", WX_Rain6hActive, WX_KoRain6h, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 24 Stunden", "rain_24h", "mm", WX_Rain24hActive, WX_KoRain24h, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 48 Stunden", "rain_48h", "mm", WX_Rain48hActive, WX_KoRain48h, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Regenmenge letzte 72 Stunden", "rain_72h", "mm", WX_Rain72hActive, WX_KoRain72h, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Regenmenge heute", "rain_today", "mm", WX_RainTodayActive, WX_KoRainToday, WxSensor::None, true, 2, 1, {{9, 1, 1.0f}}},
    {"Regen", "raining", "", WX_RainingActive, WX_KoRaining, WxSensor::None, false, 0, 1, {{1, 2, 1.0f}}},
    {"Feinstaub PM2,5", "pm25", "ug/m3", WX_Pm25Active, WX_KoPm25, WxSensor::Sds, true, 1, 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM10", "pm10", "ug/m3", WX_Pm10Active, WX_KoPm10, WxSensor::Sds, true, 1, 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM2,5 (feuchtekorrigiert)", "pm25_normalized", "ug/m3", WX_Pm25NormActive, WX_KoPm25Norm, WxSensor::Sds, true, 1, 1, {{9, 1, 1.0f}}},
    {"Feinstaub PM10 (feuchtekorrigiert)", "pm10_normalized", "ug/m3", WX_Pm10NormActive, WX_KoPm10Norm, WxSensor::Sds, true, 1, 1, {{9, 1, 1.0f}}},
    {"Wettertendenz", "weather_tendency", "", WX_TendencyActive, WX_KoTendency, WxSensor::None, false, 0, 1, {{6, 10, 1.0f}}},
    {"Wetterprognose (Zambretti-Nummer)", "forecast", "", WX_ForecastActive, WX_KoForecast, WxSensor::None, false, 0, 1, {{5, 10, 1.0f}}},
    {"Wetterprognose (Text)", "forecast_text", "", WX_ForecastTextActive, WX_KoForecastText, WxSensor::None, false, 0, 1, {{16, 1, 1.0f}}},
    {"Wettersymbol (Display)", "weather_symbol", "", WX_WeatherSymbolActive, WX_KoWeatherSymbol, WxSensor::None, false, 0, 1, {{5, 10, 1.0f}}},
};

