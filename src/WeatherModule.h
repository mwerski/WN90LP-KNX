#pragma once
#include "MeasuredValue.h"
#include "OneWireSensor.h"
#include "OpenKNX.h"
#include "PressureTrend.h"
#include "RainStatistics.h"
#include "Season.h"
#include "Sds011Sensor.h"
#include "ValueTable.h"
#include "WebStatus.h"
#include "Astro.h"
#include "DegreeDays.h"
#include "HaDiscovery.h"
#include "Irrigation.h"
#include "InfluxWriter.h"
#include "WindStats.h"
#include "Wn90Sensor.h"

#define WX_FIRMWARE_VERSION "0.12.0"

class WeatherModule : public OpenKNX::Module
{
  public:
    const std::string name() override;
    const std::string version() override;
    void setup(bool configured) override;
    void loop(bool configured) override;
    void processAfterStartupDelay() override;
    void processBeforeRestart() override;
    void processInputKo(GroupObject &ko) override;

    uint16_t flashSize() override;
    void readFlash(const uint8_t *data, const uint16_t size) override;
    void writeFlash() override;

    void showHelp() override;
    bool processCommand(const std::string cmd, bool diagnoseKo) override;

  private:
    struct WindAlarm
    {
        uint8_t source = 0; // 0 Boee, 1 Wind, 2 Wind 10-min-Mittel
        float threshold = 10;
        uint32_t onDelayMs = 3000;
        uint32_t offDelayMs = 600000;
        bool state = false;
        bool timing = false;
        uint32_t since = 0;
    };

    struct MinMax
    {
        uint32_t dayKey = 0;
        float tempMin = NAN, tempMax = NAN, gustMax = NAN;
        float tempMinYesterday = NAN, tempMaxYesterday = NAN, gustMaxYesterday = NAN;
        float radToday = 0, radYesterday = NAN; // kWh/m2
        float pvToday = 0, pvYesterday = NAN;   // kWh
        float sunshineToday = 0, sunshineYesterday = NAN; // h
    };

    struct Facade
    {
        bool active = false;
        uint16_t azimuth = 180;
        uint8_t range = 80;
        uint8_t minElevation = 5;
        uint8_t maxElevation = 90;
        uint16_t threshold = 200;
        uint32_t onDelayMs = 120000;
        uint32_t offDelayMs = 900000;
        bool sun = false;
        bool timing = false;
        uint32_t since = 0;
        float irradiance = NAN;
    };

    MeasuredValue &val(WxId id) { return _values[(uint8_t)id]; }
    void set(WxId id, float value) { val(id).set(value); }

    void processWn90();
    float calcTemperature(float stationTemp, const char **source = nullptr);
    void invalidateWn90();
    void processSds();
    void updateRain();
    void updateForecast();
    void updateWindAlarms();
    void updateSunAndTwilight();
    void updateWarnings();
    void updateMinMax(float temp, float gust);
    void checkMinMaxDay();
    void updateSeason();
    void updateSolar();
    void updateAstro();
    void updateVentilation();
    void updateIrrigation();
    bool sunny(uint8_t zambretti);
    void updateFault(uint16_t ko, bool &state, bool faulty, const char *name);
    void sendValues();
    void publishMqtt();
    std::string buildStatusJson();
    std::string buildInfluxBody();

    void showValues();
    void showSensors();
    void showRaw();
    void showRain();
    void showPressure();
    void showForecast();
    void showMinMax();
    void showSeason();
    void showSolar();
    void showAstro();
    void showClimate();
    void formatValue(char *buf, size_t len, const MeasuredValue &v);

    Wn90Sensor _wn90;
    Sds011Sensor _sds;
    OneWireSensor _oneWire;
    RainStatistics _rain;
    PressureTrend _pressure;
    MeasuredValue _values[WX_VALUE_COUNT];

    bool _configured = false;
    bool _sendAllowed = false;
    bool _sdsEnabled = false;
    bool _oneWireEnabled = false;
    bool _calcWithOneWire = false; // 1-Wire-Temperatur fuer abgeleitete Werte verwenden
    float _tempOffset = 0;
    int16_t _altitude = 0;

    // Wettersymbol
    float _heavyRain = 10;
    uint8_t _stormBft = 7;
    uint8_t _uvHigh = 6;
    float _sunnyRatio = 0.6f;
    float _lightRatio = NAN; // geglaettet: Helligkeit / Helligkeit bei klarem Himmel
    bool _sunny = false;
    uint32_t _forecastTimer = 0;

    WindStats _windStats;
    WindAlarm _alarms[3];
    uint32_t _alarmTimer = 0;

    float _twilightNight = 30, _twilightDay = 60;
    uint32_t _twilightDelayMs = 120000;
    int8_t _night = -1; // -1 unbekannt
    bool _twilightTiming = false;
    uint32_t _twilightSince = 0;

    float _iceThreshold = 1.0f;
    bool _frost = false;
    bool _ice = false;
    uint32_t _lastMoisture = 0;
    bool _moistureSeen = false;

    MinMax _minMax;

    float _radFactor = 1.0f;
    float _albedo = 0.2f;
    float _pvPeak = 0;
    uint8_t _pvTilt = 30;
    uint16_t _pvAzimuth = 180;
    float _pvPr = 0.8f;
    Facade _facades[6];
    uint32_t _solarLast = 0;
    float _ghi = NAN;

    // Sonnenzeiten und Mond
    bool _sunDnElevation = false;
    int16_t _sunDnOffsetRise = 0, _sunDnOffsetSet = 0; // min
    float _sunDnElevRise = 0, _sunDnElevSet = 0;
    float _sunshineThreshold = 120;
    uint32_t _astroDayKey = 0;
    Astro::SunDay _sunDay = {};
    float _moonRise = NAN, _moonSet = NAN;
    float _nextFullMoon = NAN, _nextNewMoon = NAN; // Tage seit 2000
    bool _dayNightSun = false;

    // Lueftungsempfehlung je Raum (Eingaenge vom Bus)
    struct Room
    {
        bool active = false;
        float minDiff = 1, minRh = 50, minIndoorT = 10, minOutdoorT = -5;
        float temp = NAN, hum = NAN;
        uint32_t tempTime = 0, humTime = 0;
        bool vent = false;
    };
    Room _rooms[3];
    Irrigation _irrigation;
    DegreeDays _degreeDays;
    uint32_t _irrigationLast = 0;
    Season _season;
    HaDiscovery _ha;
    WebStatus _web;
    InfluxWriter _influx;
    uint32_t _influxErrorsLogged = 0;
    std::string _influxMeasurement, _influxFloor, _influxRoom, _influxSensor;

    bool _faultWn90 = false;
    bool _faultSds = false;
    bool _fault1W = false;

    uint8_t _nextSend = 0;
    uint32_t _lastSendTime = 0;
    uint32_t _rainTimer = 0;
    uint32_t _mqttInterval = 0;
    uint32_t _mqttTimer = 0;
};

extern WeatherModule openknxWeatherModule;
