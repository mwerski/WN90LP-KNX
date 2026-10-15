#pragma once
#include "MeasuredValue.h"
#include "OneWireSensor.h"
#include "OpenKNX.h"
#include "PressureTrend.h"
#include "RainStatistics.h"
#include "Sds011Sensor.h"
#include "ValueTable.h"
#include "HaDiscovery.h"
#include "WindStats.h"
#include "Wn90Sensor.h"

#define WX_FIRMWARE_VERSION "0.5.0"

class WeatherModule : public OpenKNX::Module
{
  public:
    const std::string name() override;
    const std::string version() override;
    void setup(bool configured) override;
    void loop(bool configured) override;
    void processAfterStartupDelay() override;

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
    bool sunny(uint8_t zambretti);
    void updateFault(uint16_t ko, bool &state, bool faulty, const char *name);
    void sendValues();
    void publishMqtt();

    void showValues();
    void showSensors();
    void showRaw();
    void showRain();
    void showPressure();
    void showForecast();
    void showMinMax();
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
    HaDiscovery _ha;

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
