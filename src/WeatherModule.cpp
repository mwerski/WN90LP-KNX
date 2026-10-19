#include "WeatherModule.h"
#include "NetworkModule.h"
#include "Diag.h"
#include "OtaGuard.h"
#include "WeatherCalc.h"
#include "Forecast.h"
#include "Solar.h"
#include "JsonUtil.h"
#include "RgbLed.h"
#include <esp32-hal-periman.h>
#include <Update.h>
#include <ArduinoJson.h>

WeatherModule openknxWeatherModule;

static uint32_t dayKeyOf(const OpenKNX::DateTime &d)
{
    return d.year * 10000UL + d.month * 100UL + d.day;
}

static constexpr uint8_t FLASH_VERSION = 7; // 7: + Luftdruckverlauf; 1: Regen, 2: + Min/Max, 3: + Sommer/Winter, 4: + Strahlung/PV, 5: + Sonnenschein, 6: + Bewaesserung/Gradtage
static constexpr uint16_t MINMAX_FLASH_SIZE = 4 + 6 * 4;
static constexpr uint16_t SOLAR_FLASH_SIZE = 4 * 4;
static constexpr uint16_t SUNSHINE_FLASH_SIZE = 2 * 4;
// Lichtausbeute des Tageslichts zur Umrechnung Helligkeit -> Strahlung
static constexpr float LUX_PER_WM2 = 110.0f;

const std::string WeatherModule::name()
{
    return "Weather";
}

const std::string WeatherModule::version()
{
    return WX_FIRMWARE_VERSION;
}

void WeatherModule::setup(bool configured)
{
    _configured = configured;
    uint8_t rainStopDelay = 15;
    uint8_t rainWindow = 10;
    float rainMinAmount = 0.1f;
    uint8_t sdsDutyCycle = 5;
    if (configured)
    {
        _sdsEnabled = ParamWX_SdsPresent;
        _oneWireEnabled = ParamWX_OneWirePresent;
        _calcWithOneWire = _oneWireEnabled && ParamWX_CalcTempSource == 1;
        sdsDutyCycle = ParamWX_SdsDutyCycle;
        _altitude = ParamWX_Altitude;
        _tempOffset = (float)ParamWX_TempOffset;
        rainStopDelay = ParamWX_RainStopDelay;
        rainWindow = ParamWX_RainWindow;
        rainMinAmount = (float)ParamWX_RainMinAmount;
        _mqttInterval = (uint32_t)ParamWX_MqttInterval * 1000;
        _heavyRain = (float)ParamWX_SymHeavyRain;
        _stormBft = ParamWX_SymStormGust;
        _uvHigh = ParamWX_SymUvHigh;
        _sunnyRatio = ParamWX_SymSunnyRatio / 100.0f;
        _twilightNight = (float)ParamWX_TwilightNight;
        _twilightDay = max((float)ParamWX_TwilightDay, _twilightNight);
        _twilightDelayMs = (uint32_t)ParamWX_TwilightDelay * 60000UL;
        _iceThreshold = (float)ParamWX_IceThreshold;
        _ha.setup(ParamWX_HaDiscovery);
        _radFactor = ParamWX_RadFactor / 100.0f;
        _albedo = ParamWX_Albedo / 100.0f;
        _pvPeak = (float)ParamWX_PvPeak;
        _pvTilt = ParamWX_PvTilt;
        _pvAzimuth = ParamWX_PvAzimuth;
        _pvPr = ParamWX_PvPr / 100.0f;
        const uint16_t facadeBase[6] = {WX_Facade1Active, WX_Facade2Active, WX_Facade3Active, WX_Facade4Active, WX_Facade5Active, WX_Facade6Active};
        for (uint8_t i = 0; i < 6; i++)
        {
            // Block je Fassade (12 Byte): +0 aktiv, +1 Ausrichtung, +3 Bereich, +4/+5 Hoehe min/max, +6 Schwelle, +8/+9 Verzoegerung
            Facade &f = _facades[i];
            const uint16_t b = facadeBase[i];
            f.active = knx.paramByte(b) & 0x01;
            f.azimuth = knx.paramWord(b + 1);
            f.range = knx.paramByte(b + 3);
            f.minElevation = knx.paramByte(b + 4);
            f.maxElevation = knx.paramByte(b + 5);
            f.threshold = knx.paramWord(b + 6);
            f.onDelayMs = knx.paramByte(b + 8) * 60000UL;
            f.offDelayMs = knx.paramByte(b + 9) * 60000UL;
        }
        if (ParamWX_InfluxActive)
        {
            _influxMeasurement = ParamWX_InfluxMeasurementStr;
            _influxFloor = ParamWX_InfluxTagFloorStr;
            _influxRoom = ParamWX_InfluxTagRoomStr;
            _influxSensor = ParamWX_InfluxTagSensorStr;
            if (_influxMeasurement.empty())
                _influxMeasurement = "weather";
            _influx.setup(ParamWX_InfluxServerStr, ParamWX_InfluxPort, ParamWX_InfluxDatabaseStr, ParamWX_InfluxRpStr, ParamWX_InfluxUserStr,
                          ParamWX_InfluxPasswordStr, (uint32_t)ParamWX_InfluxInterval * 1000);
        }
        _sunDnElevation = ParamWX_SunDnMode;
        _sunDnOffsetRise = (int16_t)ParamWX_SunDnOffsetRise;
        _sunDnOffsetSet = (int16_t)ParamWX_SunDnOffsetSet;
        _sunDnElevRise = (float)ParamWX_SunDnElevRise;
        _sunDnElevSet = (float)ParamWX_SunDnElevSet;
        _sunshineThreshold = ParamWX_SunshineThreshold;
        const uint16_t roomBase[3] = {WX_Room1Active, WX_Room2Active, WX_Room3Active};
        for (uint8_t i = 0; i < 3; i++)
        {
            // Block je Raum (8 Byte): +0 aktiv, +1 Mindestunterschied (float), +5 min. Feuchte, +6/+7 min. Temperatur innen/aussen
            Room &r = _rooms[i];
            const uint16_t b = roomBase[i];
            r.active = knx.paramByte(b) & 0x01;
            r.minDiff = (float)knx.paramFloat(b + 1, Float_Enc_IEEE754Single);
            r.minRh = knx.paramByte(b + 5);
            r.minIndoorT = (int8_t)knx.paramByte(b + 6);
            r.minOutdoorT = (int8_t)knx.paramByte(b + 7);
        }
        _irrigation.setup(ParamWX_IrrKc / 100.0f, (float)ParamWX_IrrThreshold, (float)ParamWX_IrrStorage, (float)ParamWX_IrrAmount);
        _degreeDays.setup((float)ParamWX_DdIndoor, (float)ParamWX_DdHeatLimit, (float)ParamWX_DdCoolBase);
        // FAO-56 Gl. 47: Wind in Messhoehe z auf 2 m umrechnen
        const float z = max<uint8_t>(ParamWX_StationHeight, 1);
        _windHeightFactor = z <= 2.0f ? 1.0f : 4.87f / logf(67.8f * z - 5.42f);
        _warnings.setup(ParamWX_WarnHold);
        _sendSpacing = constrain((uint32_t)ParamWX_SendSpacing, 20UL, 1000UL);
        _ledMode = min<uint8_t>(ParamWX_LedMode, 2);
        _ledBrightness = constrain(ParamWX_LedBrightness, 1, 100);
        _ledHeartbeat = ParamWX_LedHeartbeat;
        if (ParamWX_DwdActive)
            dwdWarnings.setup(ParamWX_DwdCell, ParamWX_DwdInterval);
        _season.setup(ParamWX_SwMethod, ParamWX_SwEvening ? 22 : 21, ParamWX_SwDays, (float)ParamWX_SwThreshold,
                      (float)ParamWX_SwHysteresis);
        const uint8_t sources[3] = {(uint8_t)ParamWX_Alarm1Source, (uint8_t)ParamWX_Alarm2Source, (uint8_t)ParamWX_Alarm3Source};
        const float thresholds[3] = {(float)ParamWX_Alarm1Threshold, (float)ParamWX_Alarm2Threshold, (float)ParamWX_Alarm3Threshold};
        const uint8_t onDelays[3] = {ParamWX_Alarm1OnDelay, ParamWX_Alarm2OnDelay, ParamWX_Alarm3OnDelay};
        const uint8_t offDelays[3] = {ParamWX_Alarm1OffDelay, ParamWX_Alarm2OffDelay, ParamWX_Alarm3OffDelay};
        for (uint8_t i = 0; i < 3; i++)
        {
            _alarms[i].source = sources[i];
            _alarms[i].threshold = thresholds[i];
            _alarms[i].onDelayMs = onDelays[i] * 1000UL;
            _alarms[i].offDelayMs = offDelays[i] * 60000UL;
        }
    }

    for (uint8_t i = 0; i < WX_VALUE_COUNT; i++)
    {
        const WxValueDef *def = &WX_VALUES[i];
        const bool sensorOk = def->sensor == WxSensor::None ||
                              (def->sensor == WxSensor::Sds && _sdsEnabled) ||
                              (def->sensor == WxSensor::OneWire && _oneWireEnabled);
        // Werte hinter einem abgeschalteten Bedingungsparameter (z. B. Fassade) sind in der ETS ausgeblendet
        const bool condOk = def->condParam == 0xFFFF || (configured && (knx.paramByte(def->condParam) & 0x01));
        _values[i].init(def, configured && sensorOk && condOk);
    }

    _web.setup();
    _wn90.setup();
    _rain.setup(rainWindow, rainMinAmount, rainStopDelay);
    if (_sdsEnabled)
        _sds.setup(sdsDutyCycle);
    if (_oneWireEnabled)
        _oneWire.setup();

    logInfoP("Setup: SDS011 %s, 1-Wire %s, Hoehe %d m, Temperaturkorrektur %.1f K", _sdsEnabled ? "ja" : "nein",
             _oneWireEnabled ? "ja" : "nein", _altitude, _tempOffset);
}

void WeatherModule::processAfterStartupDelay()
{
    if (!_configured)
        return;
    _sendAllowed = true;
    // Stoerungszustand einmal senden
    if (Diag::koExists(WX_KO_FAULT_WN90))
        knx.getGroupObject(WX_KO_FAULT_WN90).value(_faultWn90, DPT_Alarm);
    if (_sdsEnabled && Diag::koExists(WX_KO_FAULT_SDS))
        knx.getGroupObject(WX_KO_FAULT_SDS).value(_faultSds, DPT_Alarm);
    if (_oneWireEnabled && Diag::koExists(WX_KO_FAULT_1W))
        knx.getGroupObject(WX_KO_FAULT_1W).value(_fault1W, DPT_Alarm);
    // Raumwerte fuer die Lueftungsempfehlung abfragen
    const uint16_t roomKos[3][2] = {{WX_KoRoom1TempIn, WX_KoRoom1HumIn}, {WX_KoRoom2TempIn, WX_KoRoom2HumIn}, {WX_KoRoom3TempIn, WX_KoRoom3HumIn}};
    for (uint8_t i = 0; i < 3; i++)
        if (_rooms[i].active)
            for (uint16_t ko : roomKos[i])
                if (Diag::koExists(ko))
                    knx.getGroupObject(ko).requestObjectRead();
}

void WeatherModule::processInputKo(GroupObject &ko)
{
    if (!_configured)
        return;
    Diag::stage(Diag::InputKo);
    const uint16_t asap = ko.asap();
    const uint16_t roomKos[3][2] = {{WX_KoRoom1TempIn, WX_KoRoom1HumIn}, {WX_KoRoom2TempIn, WX_KoRoom2HumIn}, {WX_KoRoom3TempIn, WX_KoRoom3HumIn}};
    for (uint8_t i = 0; i < 3; i++)
    {
        Room &r = _rooms[i];
        if (!r.active)
            continue;
        if (asap == roomKos[i][0])
        {
            r.temp = (float)(double)ko.value(DPT_Value_Temp);
            r.tempTime = millis();
        }
        else if (asap == roomKos[i][1])
        {
            r.hum = (float)(double)ko.value(DPT_Value_Humidity);
            r.humTime = millis();
        }
    }
    if (asap == WX_KoIrrigationDone && (bool)ko.value(DPT_Trigger))
        _irrigation.credit();
}

void WeatherModule::processBeforeRestart()
{
    // Gewollter Neustart (ETS-Programmierung, Konsole, OTA): neue Firmware laeuft, kein Rollback ausloesen
    OtaGuard::confirm("gewollter Neustart");
    // Statistiken und Luftdruckverlauf sichern; OpenKNX speichert sonst nicht, wenn die letzte Sicherung < 3 min her ist
    openknx.flash.save(true);
}

void WeatherModule::loop(bool configured)
{
    Diag::stage(Diag::Wn90);
    if (_wn90.loop())
        processWn90();
    updateFault(WX_KO_FAULT_WN90, _faultWn90, _wn90.faulty(), "Wetterstation");
    if (_faultWn90)
        invalidateWn90();

    Diag::stage(Diag::Sds);
    if (_sdsEnabled)
    {
        if (_sds.loop())
            processSds();
        updateFault(WX_KO_FAULT_SDS, _faultSds, _sds.faulty(), "Feinstaubsensor");
        if (_faultSds)
        {
            for (WxId id : {WxId::Pm25, WxId::Pm10, WxId::Pm25Norm, WxId::Pm10Norm})
                val(id).invalidate();
        }
    }

    Diag::stage(Diag::OneWire);
    if (_oneWireEnabled && _oneWire.loop())
    {
        set(WxId::Temp1W, _oneWire.temperature());
        updateFault(WX_KO_FAULT_1W, _fault1W, _oneWire.faulty(), "1-Wire-Sensor");
    }

    Diag::stage(Diag::Rain);
    _rain.loop();
    _pressure.loop();
    if (delayCheck(_rainTimer, 1000))
    {
        _rainTimer = millis();
        updateRain();
        set(WxId::Trend1h, _pressure.trend(1));
        set(WxId::Trend3h, _pressure.trend(3));
    }
    if (delayCheck(_forecastTimer, 10000))
    {
        _forecastTimer = millis();
        Diag::stage(Diag::Forecast);
        updateForecast();
        Diag::stage(Diag::Sun);
        updateSunAndTwilight();
        Diag::stage(Diag::Warnings);
        updateWarnings();
        Diag::stage(Diag::Season);
        updateSeason();
        Diag::stage(Diag::Solar);
        updateSolar();
        Diag::stage(Diag::Astro);
        updateAstro();
        Diag::stage(Diag::Ventilation);
        updateVentilation();
        Diag::stage(Diag::Irrigation);
        updateIrrigation();
        Diag::stage(Diag::Warnings2);
        updateWarnings2();
        Diag::stage(Diag::Dwd);
        updateDwd();
        Diag::stage(Diag::MinMax);
        checkMinMaxDay();
    }
    if (delayCheck(_alarmTimer, 500))
    {
        _alarmTimer = millis();
        updateWindAlarms();
    }
    updateStatusLed();
    if (_rain.saveRequested())
        openknx.flash.save(true);

    Diag::stage(Diag::Send);
    if (configured && _sendAllowed)
        sendValues();
    // Die Textausgaben belegen kurzzeitig viel Speicher; bei Speichermangel diese Ausgabe auslassen statt abzustuerzen
    try
    {
        if (configured)
        {
            Diag::stage(Diag::Mqtt);
            publishMqtt();
            Diag::stage(Diag::HaDiscovery);
            _ha.loop(_values, WX_VALUE_COUNT);
        }
        Diag::stage(Diag::WebStatus);
        if (_web.due())
            _web.update(buildStatusJson());
        Diag::stage(Diag::Influx);
        if (_influx.due())
            _influx.submit(buildInfluxBody());
    }
    catch (const std::bad_alloc &)
    {
        Diag::noMemory();
    }
    Diag::stage(Diag::Dwd);
    if (dwdWarnings.enabled())
        dwdWarnings.loop(openknx.time.isValid() ? (int64_t)openknx.time.getUtcTime().toTime_t() : 0);
    Diag::stage(Diag::Network);
    if (_influx.errorCount() != _influxErrorsLogged)
    {
        _influxErrorsLogged = _influx.errorCount();
        // Nur bis die Wartezeit ihr Maximum erreicht hat melden, danach jeden 12. Fehler (ca. stuendlich)
        const uint8_t streak = _influx.failStreak();
        if (streak <= 6 || streak % 12 == 0)
            logErrorP("InfluxDB: Schreiben fehlgeschlagen (HTTP %d) %s, naechster Versuch in %lu s", _influx.lastCode(),
                      _influx.lastError().c_str(), _influx.waitSeconds());
    }
}

void WeatherModule::processWn90()
{
    using R = Wn90Sensor::Reg;
    auto reg = [this](R r, float factor, float offset = 0) -> float {
        return _wn90.valid(r) ? _wn90.raw(r) * factor + offset : NAN;
    };

    const float temp = reg(R::Temperature, 0.1f, -40.0f) + _tempOffset;
    const float hum = reg(R::Humidity, 1.0f);
    const float wind = reg(R::Wind, 0.1f);
    const float gust = reg(R::Gust, 0.1f);
    const float pressure = reg(R::Pressure, 0.1f);

    set(WxId::Temp, temp);
    set(WxId::Humidity, hum);
    set(WxId::WindSpeed, wind);
    set(WxId::GustSpeed, gust);
    set(WxId::WindBft, isnan(wind) ? NAN : WeatherCalc::beaufort(wind));
    set(WxId::GustBft, isnan(gust) ? NAN : WeatherCalc::beaufort(gust));
    set(WxId::WindDir, reg(R::WindDir, 1.0f));
    set(WxId::Light, reg(R::Light, 10.0f));
    set(WxId::UvIndex, reg(R::Uv, 0.1f));
    set(WxId::Pressure, pressure);

    // Abgeleitete Werte wahlweise mit der 1-Wire-Temperatur
    const float t = calcTemperature(temp);
    if (!isnan(pressure))
    {
        _pressure.feed(pressure);
        set(WxId::PressureSea, (isnan(t) || _altitude == 0) ? pressure : baroToSeaLevelC(pressure, t, _altitude));
    }
    else
        val(WxId::PressureSea).invalidate();

    if (!isnan(t) && !isnan(hum))
    {
        const float dew = dewPoint(t, hum);
        set(WxId::AbsHumidity, absoluteHumidity(t, hum));
        set(WxId::DewPoint, dew);
        set(WxId::FrostPoint, WeatherCalc::frostPoint(t, hum));
        set(WxId::Humidex, humidex(t, dew));
        set(WxId::HeatIndex, WeatherCalc::heatIndex(t, hum));
        set(WxId::Discomfort, discomfortIndex(t, hum));
    }
    else
    {
        for (WxId id : {WxId::AbsHumidity, WxId::DewPoint, WxId::FrostPoint, WxId::Humidex, WxId::HeatIndex, WxId::Discomfort})
            val(id).invalidate();
    }
    set(WxId::WindChill, (isnan(t) || isnan(wind)) ? NAN : WeatherCalc::windChill(t, wind));

    if (_wn90.valid(R::RainCounter))
        _rain.feed(_wn90.raw(R::RainCounter));

    _windStats.feed(wind, gust);
    set(WxId::WindAvg10, _windStats.average());
    set(WxId::GustMax10, _windStats.maxGust());
    updateMinMax(t, gust);
    _season.feed(t);
}

float WeatherModule::calcTemperature(float stationTemp, const char **source)
{
    const MeasuredValue &oneWire = val(WxId::Temp1W);
    if (_calcWithOneWire && oneWire.valid())
    {
        if (source)
            *source = "1-Wire";
        return oneWire.value();
    }
    if (source)
        *source = _calcWithOneWire ? "Wetterstation (1-Wire ausgefallen)" : "Wetterstation";
    return stationTemp;
}

void WeatherModule::invalidateWn90()
{
    // Nur direkt gemessene und daraus abgeleitete Werte; Regenstatistik, Minimum/Maximum,
    // Alarme und Prognose behalten ihren Zustand bzw. werden selbst neu bewertet
    static const WxId ids[] = {WxId::Temp, WxId::Humidity, WxId::AbsHumidity, WxId::DewPoint, WxId::FrostPoint,
                               WxId::Humidex, WxId::HeatIndex, WxId::WindChill, WxId::Discomfort, WxId::WindSpeed,
                               WxId::GustSpeed, WxId::WindBft, WxId::GustBft, WxId::WindDir, WxId::Pressure,
                               WxId::PressureSea, WxId::Light, WxId::UvIndex, WxId::WindAvg10, WxId::GustMax10};
    for (WxId id : ids)
        val(id).invalidate();
}

void WeatherModule::updateWindAlarms()
{
    const float sources[3] = {val(WxId::GustSpeed).valid() ? val(WxId::GustSpeed).value() : NAN,
                              val(WxId::WindSpeed).valid() ? val(WxId::WindSpeed).value() : NAN,
                              val(WxId::WindAvg10).valid() ? val(WxId::WindAvg10).value() : NAN};
    for (uint8_t i = 0; i < 3; i++)
    {
        WindAlarm &a = _alarms[i];
        const WxId id = (WxId)((uint8_t)WxId::WindAlarm1 + i);
        // Nicht aktivierte Alarme gar nicht auswerten (ihre Parameter sind in der ETS ausgeblendet)
        if (!val(id).active())
        {
            a.state = false;
            a.timing = false;
            continue;
        }
        const float wind = sources[min<uint8_t>(a.source, 2)];
        if (_faultWn90)
        {
            // Ohne Messwerte sicherheitshalber Alarm (Markisen/Jalousien schuetzen)
            a.state = true;
            a.timing = false;
        }
        else if (!isnan(wind))
        {
            const bool over = wind >= a.threshold;
            if (over != a.state)
            {
                if (!a.timing)
                {
                    a.timing = true;
                    a.since = millis();
                }
                if (millis() - a.since >= (over ? a.onDelayMs : a.offDelayMs))
                {
                    a.state = over;
                    a.timing = false;
                    logInfoP("Windalarm %u %s (%.1f m/s)", i + 1, over ? "ausgeloest" : "zurueckgenommen", wind);
                }
            }
            else
                a.timing = false;
        }
        set(id, a.state ? 1.0f : 0.0f);
    }
}

void WeatherModule::updateStatusLed()
{
    RgbLed *led = RgbLed::instance();
    if (led == nullptr)
        return;
    using P = RgbLed::Pattern;
    const uint8_t br = _ledBrightness;
    if (_ledMode == 0)
        led->status(P::Off);
    // Vorrang: OTA, WLAN, Sensorstoerung, dann (nur "alle Zustaende") Windalarm und Lebenszeichen
    else if (Update.isRunning())
        led->status(P::Pulse, 160, 0, 255, br);
    else if (!openknxNetwork.established())
        led->status(P::Blink, 0, 0, 255, br);
    else if (_faultWn90 || _faultSds || _fault1W)
        led->status(P::Blink, 255, 170, 0, br);
    else if (_ledMode == 2 && (_alarms[0].state || _alarms[1].state || _alarms[2].state))
        led->status(P::On, 255, 70, 0, br);
    else if (_ledMode == 2 && _ledHeartbeat)
        led->status(P::Flash, 0, 255, 0, br);
    else
        led->status(P::Off);
    led->tick();
}

void WeatherModule::updateSunAndTwilight()
{
#ifdef OPENKNX_SUN_POSITION
    if (openknx.sun.isSunCalculatioValid())
    {
        set(WxId::SunAzimuth, openknx.sun.azimuth());
        set(WxId::SunElevation, openknx.sun.elevation());
    }
    else
    {
        val(WxId::SunAzimuth).invalidate();
        val(WxId::SunElevation).invalidate();
    }
#endif

    const MeasuredValue &light = val(WxId::Light);
    if (!light.valid())
        return;
    if (_night < 0)
        _night = light.value() < _twilightNight ? 1 : 0; // Startzustand ohne Verzoegerung
    const bool change = _night ? light.value() > _twilightDay : light.value() < _twilightNight;
    if (!change)
        _twilightTiming = false;
    else if (!_twilightTiming)
    {
        _twilightTiming = true;
        _twilightSince = millis();
    }
    else if (millis() - _twilightSince >= _twilightDelayMs)
    {
        _night = !_night;
        _twilightTiming = false;
    }
    set(WxId::DayNight, _night);
}

void WeatherModule::updateWarnings()
{
    const MeasuredValue &temp = val(WxId::Temp);
    const float t = calcTemperature(temp.valid() ? temp.value() : NAN);
    if (isnan(t))
    {
        val(WxId::Frost).invalidate();
        val(WxId::IceWarning).invalidate();
        return;
    }

    // Frost mit 0,5 K Hysterese
    if (t < 0.0f)
        _frost = true;
    else if (t >= 0.5f)
        _frost = false;
    set(WxId::Frost, _frost);

    // Feuchtigkeit: Regen in der letzten Stunde oder Taupunkt nahe an der Temperatur
    const MeasuredValue &dew = val(WxId::DewPoint);
    const float rain1h = _rain.amountMinutes(60);
    const bool moisture = _rain.raining() || (!isnan(rain1h) && rain1h > 0.0f) || (dew.valid() && t - dew.value() <= 1.0f);
    if (moisture)
    {
        _moistureSeen = true;
        _lastMoisture = millis();
    }
    const bool moistureRecent = _moistureSeen && millis() - _lastMoisture < 30 * 60000UL;
    if (t <= _iceThreshold && moisture)
        _ice = true;
    else if (t > _iceThreshold + 1.0f || !moistureRecent)
        _ice = false;
    set(WxId::IceWarning, _ice);
}

void WeatherModule::checkMinMaxDay()
{
    if (!openknx.time.isValid())
        return;
    OpenKNX::DateTime now = openknx.time.getLocalTime();
    const uint32_t today = dayKeyOf(now);
    if (_minMax.dayKey == today)
        return;
    OpenKNX::DateTime yesterday = now;
    yesterday.addDays(-1);
    const bool wasYesterday = _minMax.dayKey == dayKeyOf(yesterday);
    if (_minMax.dayKey != 0)
    {
        // Tageswechsel: heutige Werte werden zu "gestern" (nur wenn sie wirklich von gestern sind)
        _minMax.tempMinYesterday = wasYesterday ? _minMax.tempMin : NAN;
        _minMax.tempMaxYesterday = wasYesterday ? _minMax.tempMax : NAN;
        _minMax.gustMaxYesterday = wasYesterday ? _minMax.gustMax : NAN;
        _minMax.radYesterday = wasYesterday ? _minMax.radToday : NAN;
        _minMax.pvYesterday = wasYesterday ? _minMax.pvToday : NAN;
        _minMax.sunshineYesterday = wasYesterday ? _minMax.sunshineToday : NAN;
        _irrigation.dayRollover(wasYesterday);
        _minMax.sunshineToday = 0;
        _minMax.tempMin = _minMax.tempMax = _minMax.gustMax = NAN;
        _minMax.radToday = 0;
        _minMax.pvToday = 0;
    }
    // Ohne gespeicherten Tag (erste gueltige Uhrzeit): bisher gesammelte Werte gelten fuer heute
    _minMax.dayKey = today;
    openknx.flash.save(true);
}

void WeatherModule::updateSeason()
{
    if (openknx.time.isValid())
    {
        const OpenKNX::DateTime now = openknx.time.getLocalTime();
        const MeasuredValue &temp = val(WxId::Temp);
        const float t = calcTemperature(temp.valid() ? temp.value() : NAN);
        _degreeDays.start(dayKeyOf(now));
        if (_season.loop(dayKeyOf(now), now.hour, t))
            _degreeDays.addDay(_season.finishedDayKey(), _season.finishedMean());
    }
    set(WxId::HddMonth, _degreeDays.hddMonth());
    set(WxId::HddLastMonth, _degreeDays.hddLastMonth());
    set(WxId::HddYear, _degreeDays.hddYear());
    set(WxId::HddLastYear, _degreeDays.hddLastYear());
    set(WxId::CddMonth, _degreeDays.cddMonth());
    set(WxId::CddLastMonth, _degreeDays.cddLastMonth());
    set(WxId::CddYear, _degreeDays.cddYear());
    set(WxId::CddLastYear, _degreeDays.cddLastYear());
    set(WxId::SummerWinter, _season.summer() < 0 ? NAN : (float)_season.summer());
    set(WxId::MeanTemp, _season.average());
    set(WxId::DailyMeanYesterday, _season.yesterday());

    // Kenntage nach bisherigem Minimum/Maximum des laufenden Tages
    const bool valid = _minMax.dayKey != 0 && !isnan(_minMax.tempMax);
    set(WxId::SummerDay, valid ? (float)(_minMax.tempMax >= 25.0f) : NAN);
    set(WxId::HotDay, valid ? (float)(_minMax.tempMax >= 30.0f) : NAN);
    set(WxId::FrostDay, valid ? (float)(_minMax.tempMin < 0.0f) : NAN);
    set(WxId::IceDay, valid ? (float)(_minMax.tempMax < 0.0f) : NAN);
}

void WeatherModule::updateSolar()
{
    const uint32_t now = millis();
    const float hours = _solarLast ? (now - _solarLast) / 3600000.0f : 0;
    _solarLast = now;

    const MeasuredValue &light = val(WxId::Light);
    _ghi = light.valid() ? light.value() / LUX_PER_WM2 * _radFactor : NAN;
    set(WxId::GlobalRadiation, _ghi);

    bool sunValid = false;
    float elevation = NAN, azimuth = NAN;
#ifdef OPENKNX_SUN_POSITION
    if (openknx.sun.isSunCalculatioValid())
    {
        sunValid = true;
        elevation = openknx.sun.elevation();
        azimuth = openknx.sun.azimuth();
    }
#endif
    uint16_t doy = 172;
    if (openknx.time.isValid())
    {
        const OpenKNX::DateTime d = openknx.time.getLocalTime();
        doy = Solar::dayOfYear(d.year, d.month, d.day);
    }

    // Bewoelkung tagsueber aus dem Verhaeltnis zum klaren Himmel
    if (sunValid && elevation > 5.0f && !isnan(_ghi))
        set(WxId::CloudCover, constrain((1.0f - _ghi / Solar::clearSky(elevation)) * 100.0f, 0.0f, 100.0f));
    else
        val(WxId::CloudCover).invalidate();

    if (isnan(_ghi))
    {
        val(WxId::PvPower).invalidate();
        for (uint8_t i = 0; i < 6; i++)
        {
            val((WxId)((uint8_t)WxId::Facade1Irradiance + i * 3)).invalidate();
            val((WxId)((uint8_t)WxId::Facade1Profile + i * 3)).invalidate();
        }
        return;
    }

    const Solar::Split sp = sunValid ? Solar::split(_ghi, elevation, doy) : Solar::Split{0, _ghi};

    // PV-Leistung: kWp * Einstrahlung auf die Modulflaeche [kW/m2] * Performance Ratio -> W
    float pv = NAN;
    if (_pvPeak > 0)
    {
        const float poa = sunValid ? Solar::plane(_ghi, sp, elevation, azimuth, _pvTilt, _pvAzimuth, _albedo) : _ghi;
        pv = _pvPeak * poa * _pvPr;
        set(WxId::PvPower, pv);
    }

    // Tagessummen
    if (hours > 0 && hours < 0.1f)
    {
        if (_ghi >= _sunshineThreshold)
            _minMax.sunshineToday += hours;
        _minMax.radToday += _ghi * hours / 1000.0f;
        if (!isnan(pv))
            _minMax.pvToday += pv * hours / 1000.0f;
    }
    const bool dayValid = _minMax.dayKey != 0;
    set(WxId::RadiationToday, dayValid ? _minMax.radToday : NAN);
    set(WxId::RadiationYesterday, _minMax.radYesterday);
    set(WxId::SunshineToday, dayValid ? _minMax.sunshineToday : NAN);
    set(WxId::SunshineYesterday, _minMax.sunshineYesterday);
    set(WxId::PvEnergyToday, (dayValid && _pvPeak > 0) ? _minMax.pvToday : NAN);
    set(WxId::PvEnergyYesterday, _pvPeak > 0 ? _minMax.pvYesterday : NAN);

    // Fassaden: WxId-Reihenfolge je Fassade Sonne, Einstrahlung, Profilwinkel
    for (uint8_t i = 0; i < 6; i++)
    {
        Facade &f = _facades[i];
        if (!f.active)
            continue;
        const WxId idSun = (WxId)((uint8_t)WxId::Facade1Sun + i * 3);
        if (!sunValid)
        {
            val((WxId)((uint8_t)idSun + 1)).invalidate();
            val((WxId)((uint8_t)idSun + 2)).invalidate();
            continue;
        }
        f.irradiance = Solar::plane(_ghi, sp, elevation, azimuth, 90.0f, f.azimuth, _albedo);
        const float offset = fabsf(Solar::azimuthOffset(azimuth, f.azimuth));
        const bool inRange = offset <= f.range && elevation >= f.minElevation && elevation <= f.maxElevation;
        const bool sunNow = inRange && f.irradiance >= f.threshold;
        if (sunNow == f.sun)
            f.timing = false;
        else if (!f.timing)
        {
            f.timing = true;
            f.since = now;
        }
        else if (now - f.since >= (sunNow ? f.onDelayMs : f.offDelayMs))
        {
            f.sun = sunNow;
            f.timing = false;
        }
        set(idSun, f.sun);
        set((WxId)((uint8_t)idSun + 1), f.irradiance);
        set((WxId)((uint8_t)idSun + 2), inRange ? Solar::profileAngle(elevation, azimuth, f.azimuth) : NAN);
    }
}

void WeatherModule::updateAstro()
{
    static const WxId sunTimes[] = {WxId::Sunrise, WxId::Sunset, WxId::SolarNoon, WxId::CivilDawn, WxId::CivilDusk, WxId::NauticalDawn,
                                    WxId::NauticalDusk, WxId::AstroDawn, WxId::AstroDusk, WxId::SunriseAzimuth, WxId::SunsetAzimuth,
                                    WxId::TransitElevation, WxId::DayLength, WxId::DayProgress, WxId::SunFalling, WxId::DayNightSun,
                                    WxId::DayNightCivil, WxId::DayNightNautical, WxId::DayNightAstro, WxId::ClearSkyRadiation,
                                    WxId::ClearSkyMax, WxId::MoonPhase, WxId::MoonPhaseText, WxId::MoonIllumination, WxId::MoonAge,
                                    WxId::MoonAzimuth, WxId::MoonElevation, WxId::Moonrise, WxId::Moonset, WxId::NextFullMoon,
                                    WxId::NextNewMoon};
    if (!_configured || !openknx.time.isValid())
    {
        for (WxId id : sunTimes)
            val(id).invalidate();
        return;
    }
    const float lat = (float)ParamBASE_Latitude;
    const float lon = (float)ParamBASE_Longitude;
    const OpenKNX::DateTime local = openknx.time.getLocalTime();
    const time_t utc = local.toTime_t();
    // Ortszeit als "Unixzeit der Wanduhr" -> Verschiebung zu UTC und Mitternacht Ortszeit
    const long localDays = MeasuredValue::daysFromCivil(local.year, local.month, local.day);
    const int32_t secOfDay = local.hour * 3600 + local.minute * 60 + local.second;
    const int32_t tzOffset = (int32_t)((int64_t)localDays * 86400 + secOfDay - (int64_t)utc);
    const time_t midnight = utc - secOfDay;
    MeasuredValue::setDayContext(midnight, tzOffset);

    // Tageswerte einmal je Tag (und nach dem ersten Erhalt der Uhrzeit)
    const uint32_t key = dayKeyOf(local);
    if (key != _astroDayKey)
    {
        _astroDayKey = key;
        _sunDay = Astro::sunDay(midnight, tzOffset, lat, lon);
        Astro::moonRiseSet(midnight, lat, lon, _moonRise, _moonSet);
        // Datum (Ortszeit) des naechsten Voll-/Neumonds als Tage seit 1.1.2000
        auto localDate = [&](time_t t) -> float { return (float)((t + tzOffset) / 86400 - 10957); };
        _nextFullMoon = localDate(Astro::nextMoonPhase(utc, 180.0f));
        _nextNewMoon = localDate(Astro::nextMoonPhase(utc, 0.0f));
        logInfoP("Sonnenzeiten berechnet: Aufgang %s, Untergang %s", MeasuredValue::format(_sunDay.rise, &WX_VALUES[(uint8_t)WxId::Sunrise], false).c_str(),
                 MeasuredValue::format(_sunDay.set, &WX_VALUES[(uint8_t)WxId::Sunset], false).c_str());
    }
    // Neu-/Vollmond am heutigen Tag erreicht: naechsten Termin bestimmen
    const float today = (float)(localDays - 10957);
    if ((!isnan(_nextFullMoon) && today > _nextFullMoon) || (!isnan(_nextNewMoon) && today > _nextNewMoon))
        _astroDayKey = 0;

    set(WxId::Sunrise, _sunDay.rise);
    set(WxId::Sunset, _sunDay.set);
    set(WxId::SolarNoon, _sunDay.transit);
    set(WxId::CivilDawn, _sunDay.civilDawn);
    set(WxId::CivilDusk, _sunDay.civilDusk);
    set(WxId::NauticalDawn, _sunDay.nauticalDawn);
    set(WxId::NauticalDusk, _sunDay.nauticalDusk);
    set(WxId::AstroDawn, _sunDay.astroDawn);
    set(WxId::AstroDusk, _sunDay.astroDusk);
    set(WxId::SunriseAzimuth, _sunDay.riseAzimuth);
    set(WxId::SunsetAzimuth, _sunDay.setAzimuth);
    set(WxId::TransitElevation, _sunDay.transitElevation);
    set(WxId::DayLength, _sunDay.dayLength);
    set(WxId::ClearSkyMax, Solar::clearSky(_sunDay.transitElevation));

    // Aktuelle Sonnenhoehe aus der eigenen Berechnung (gleiche Basis wie die Zeiten)
    const Astro::Position sun = Astro::sunPosition(utc, lat, lon);
    const float now = secOfDay;
    set(WxId::SunFalling, !isnan(_sunDay.transit) && now > _sunDay.transit);
    set(WxId::DayNightCivil, sun.elevation < -6.0f);
    set(WxId::DayNightNautical, sun.elevation < -12.0f);
    set(WxId::DayNightAstro, sun.elevation < -18.0f);
    set(WxId::ClearSkyRadiation, Solar::clearSky(sun.elevation));
    if (!isnan(_sunDay.rise) && !isnan(_sunDay.set))
        set(WxId::DayProgress, constrain((now - _sunDay.rise) / (_sunDay.set - _sunDay.rise) * 100.0f, 0.0f, 100.0f));
    else
        set(WxId::DayProgress, sun.elevation > 0 ? 100.0f : 0.0f);

    // Tag/Nacht wie im EDOMI-LBS: Auf-/Untergang mit Versatz oder Elevationsschwellen (morgens/abends)
    bool night;
    if (_sunDnElevation)
    {
        const bool falling = !isnan(_sunDay.transit) && now > _sunDay.transit;
        night = falling ? sun.elevation < _sunDnElevSet : !(sun.elevation > _sunDnElevRise);
    }
    else if (!isnan(_sunDay.rise) && !isnan(_sunDay.set))
        night = !(now > _sunDay.rise + _sunDnOffsetRise * 60 && now < _sunDay.set + _sunDnOffsetSet * 60);
    else
        night = sun.elevation < -0.833f; // Polartag/-nacht
    set(WxId::DayNightSun, night);

    // Mond
    const Astro::MoonState moon = Astro::moon(utc, lat, lon);
    set(WxId::MoonPhase, moon.phase);
    set(WxId::MoonPhaseText, moon.phase);
    set(WxId::MoonIllumination, moon.illumination * 100.0f);
    set(WxId::MoonAge, moon.age);
    set(WxId::MoonAzimuth, moon.position.azimuth);
    set(WxId::MoonElevation, moon.position.elevation);
    set(WxId::Moonrise, _moonRise);
    set(WxId::Moonset, _moonSet);
    set(WxId::NextFullMoon, _nextFullMoon);
    set(WxId::NextNewMoon, _nextNewMoon);
}

void WeatherModule::updateVentilation()
{
    static constexpr uint32_t MAX_AGE = 2 * 3600000UL; // Raumwerte gelten 2 h
    const MeasuredValue &outAbs = val(WxId::AbsHumidity);
    const MeasuredValue &outTemp = val(WxId::Temp);
    const float tOut = calcTemperature(outTemp.valid() ? outTemp.value() : NAN);
    for (uint8_t i = 0; i < 3; i++)
    {
        Room &r = _rooms[i];
        const WxId base = (WxId)((uint8_t)WxId::Room1Vent + i * 4); // je Raum: Vent, AbsHum, DewPoint, Diff
        if (!r.active)
            continue;
        const bool fresh = !isnan(r.temp) && !isnan(r.hum) && millis() - r.tempTime < MAX_AGE && millis() - r.humTime < MAX_AGE;
        if (!fresh)
        {
            for (uint8_t k = 0; k < 4; k++)
                val((WxId)((uint8_t)base + k)).invalidate();
            r.vent = false;
            continue;
        }
        const float absIn = absoluteHumidity(r.temp, r.hum);
        set((WxId)((uint8_t)base + 1), absIn);
        set((WxId)((uint8_t)base + 2), dewPoint(r.temp, r.hum));
        if (!outAbs.valid() || isnan(tOut))
        {
            val((WxId)((uint8_t)base + 3)).invalidate();
            val(base).invalidate();
            continue;
        }
        const float diff = absIn - outAbs.value();
        set((WxId)((uint8_t)base + 3), diff);
        // Hysterese 0,5 g/m3; Raumfeuchte und Temperaturen als Freigabe
        const bool allowed = r.hum >= r.minRh && r.temp >= r.minIndoorT && tOut >= r.minOutdoorT;
        if (!allowed || diff < max(r.minDiff - 0.5f, 0.0f))
            r.vent = false;
        else if (diff >= r.minDiff)
            r.vent = true;
        set(base, r.vent);
    }
}

void WeatherModule::updateIrrigation()
{
    const uint32_t now = millis();
    const float hours = _irrigationLast ? (now - _irrigationLast) / 3600000.0f : 0;
    _irrigationLast = now;

    const MeasuredValue &temp = val(WxId::Temp);
    const MeasuredValue &hum = val(WxId::Humidity);
    const MeasuredValue &wind = val(WxId::WindAvg10);
    const MeasuredValue &pressure = val(WxId::Pressure);
    float rso = 0;
    if (_configured && openknx.time.isValid())
        rso = Solar::clearSky(Astro::sunPosition(openknx.time.getUtcTime().toTime_t(), (float)ParamBASE_Latitude, (float)ParamBASE_Longitude).elevation);
    _irrigation.update(hours, calcTemperature(temp.valid() ? temp.value() : NAN), hum.valid() ? hum.value() : NAN,
                       wind.valid() ? wind.value() * _windHeightFactor : NAN, _ghi, rso, pressure.valid() ? pressure.value() : NAN,
                       _rain.hasData() ? _rain.total() : NAN);

    const bool dayValid = _minMax.dayKey != 0;
    set(WxId::Et0Today, dayValid ? _irrigation.et0Today() : NAN);
    set(WxId::Et0Yesterday, _irrigation.et0Yesterday());
    set(WxId::WaterBalance, _irrigation.balance());
    set(WxId::Irrigate, _irrigation.irrigate());
}

void WeatherModule::updateWarnings2()
{
    auto v = [this](WxId id) { return val(id).valid() ? val(id).value() : NAN; };
    const MeasuredValue &temp = val(WxId::Temp);
    Warnings::Input in;
    in.gustKmh = v(WxId::GustMax10) * 3.6f;
    in.rain1h = _rain.hasData() ? _rain.amountMinutes(60) : NAN;
    in.rain6h = _rain.hasData() ? _rain.amountHours(6) : NAN;
    in.rain24h = _rain.hasData() ? _rain.amountHours(24) : NAN;
    in.rain48h = _rain.hasData() ? _rain.amountHours(48) : NAN;
    in.rain72h = _rain.hasData() ? _rain.amountHours(72) : NAN;
    in.temp = calcTemperature(temp.valid() ? temp.value() : NAN);
    in.heatIndex = v(WxId::HeatIndex);
    in.uv = v(WxId::UvIndex);
    in.raining = _rain.raining();
    in.iceWarning = _ice;
    _warnings.update(in);

    set(WxId::WarnLevel, _warnings.overall());
    set(WxId::WarnText, _warnings.code());
    set(WxId::WarnSevere, _warnings.overall() >= 3);
    set(WxId::WarnWind, _warnings.level(Warnings::Wind));
    set(WxId::WarnHeavyRain, _warnings.level(Warnings::HeavyRain));
    set(WxId::WarnLongRain, _warnings.level(Warnings::LongRain));
    set(WxId::WarnFrost, _warnings.level(Warnings::Frost));
    set(WxId::WarnHeat, _warnings.level(Warnings::Heat));
    set(WxId::WarnUv, _warnings.level(Warnings::Uv));
}

void WeatherModule::updateDwd()
{
    if (!dwdWarnings.enabled())
        return;
    const DwdWarnings::Result r = dwdWarnings.result();
    set(WxId::DwdFault, dwdWarnings.faulty());
    if (!r.valid)
        return;
    set(WxId::DwdLevel, r.levelNow);
    set(WxId::DwdLevelUpcoming, r.levelAll);
    set(WxId::DwdSevere, r.levelNow >= 3);
    set(WxId::DwdCount, r.countNow);
    // Text-KO: neuer Wert (und damit Senden) nur, wenn sich das Ereignis aendert
    if (r.event != _dwdEvent || !val(WxId::DwdEvent).valid())
    {
        _dwdEvent = r.event;
        _dwdEventSeq += 1;
        set(WxId::DwdEvent, _dwdEventSeq);
        logInfoP("DWD: %s", r.headline.empty() ? "keine Warnung" : r.headline.c_str());
    }
}

void WeatherModule::updateMinMax(float temp, float gust)
{
    if (!isnan(temp))
    {
        if (isnan(_minMax.tempMin) || temp < _minMax.tempMin)
            _minMax.tempMin = temp;
        if (isnan(_minMax.tempMax) || temp > _minMax.tempMax)
            _minMax.tempMax = temp;
    }
    if (!isnan(gust) && (isnan(_minMax.gustMax) || gust > _minMax.gustMax))
        _minMax.gustMax = gust;

    // Erst mit gueltiger Uhrzeit ist "heute" definiert
    const bool valid = _minMax.dayKey != 0;
    set(WxId::TempMinToday, valid ? _minMax.tempMin : NAN);
    set(WxId::TempMaxToday, valid ? _minMax.tempMax : NAN);
    set(WxId::GustMaxToday, valid ? _minMax.gustMax : NAN);
    set(WxId::TempMinYesterday, _minMax.tempMinYesterday);
    set(WxId::TempMaxYesterday, _minMax.tempMaxYesterday);
    set(WxId::GustMaxYesterday, _minMax.gustMaxYesterday);
}

void WeatherModule::processSds()
{
    const float pm25 = _sds.pm25();
    const float pm10 = _sds.pm10();
    set(WxId::Pm25, pm25);
    set(WxId::Pm10, pm10);
    const MeasuredValue &hum = val(WxId::Humidity);
    if (hum.valid())
    {
        set(WxId::Pm25Norm, WeatherCalc::normalizePm25(pm25, hum.value()));
        set(WxId::Pm10Norm, WeatherCalc::normalizePm10(pm10, hum.value()));
    }
}

void WeatherModule::updateRain()
{
    if (!_rain.hasData())
        return;
    set(WxId::RainTotal, (float)_rain.total());
    set(WxId::RainRate, _rain.rate());
    set(WxId::Rain5m, _rain.amountMinutes(5));
    set(WxId::Rain1h, _rain.amountMinutes(60));
    set(WxId::Rain6h, _rain.amountHours(6));
    set(WxId::Rain24h, _rain.amountHours(24));
    set(WxId::Rain48h, _rain.amountHours(48));
    set(WxId::Rain72h, _rain.amountHours(72));
    set(WxId::RainToday, _rain.today());
    set(WxId::Raining, _rain.raining() ? 1.0f : 0.0f);
}

void WeatherModule::updateForecast()
{
    // Tendenz ueber 3 h; in den ersten 3 h nach dem Start aus 1 h hochgerechnet
    float trend3h = _pressure.trend(3);
    if (isnan(trend3h))
        trend3h = _pressure.trend(1) * 3.0f;
    set(WxId::Tendency, isnan(trend3h) ? NAN : Forecast::tendency(trend3h));

    const MeasuredValue &sea = val(WxId::PressureSea);
    const MeasuredValue &wind = val(WxId::WindSpeed);
    const MeasuredValue &dir = val(WxId::WindDir);
    // Bei Windstille ist die Richtung bedeutungslos
    const float windDir = (wind.valid() && wind.value() >= 1.0f && dir.valid()) ? dir.value() : NAN;
    const uint8_t month = openknx.time.isValid() ? openknx.time.getLocalTime().month : 0;
    const uint8_t z = Forecast::zambretti(sea.valid() ? sea.value() : NAN, trend3h, windDir, month);
    set(WxId::Forecast, z ? z : NAN);
    set(WxId::ForecastText, z ? z : NAN);

    // Wettersymbol fuer Displays: 0 sonnig, 1 bewoelkt, 2 Schauer, 3 starker Regen, 4 Gewitter, 5 hoher UV
    if (!val(WxId::Light).valid() && !_rain.hasData())
    {
        val(WxId::WeatherSymbol).invalidate();
        return;
    }
    const MeasuredValue &rate = val(WxId::RainRate);
    const MeasuredValue &gust = val(WxId::GustSpeed);
    const MeasuredValue &uv = val(WxId::UvIndex);
    const bool heavy = rate.valid() && rate.value() >= _heavyRain;
    uint8_t symbol;
    if (_rain.raining())
    {
        if (heavy && gust.valid() && WeatherCalc::beaufort(gust.value()) >= _stormBft)
            symbol = 4;
        else
            symbol = heavy ? 3 : 2;
    }
    else if (uv.valid() && uv.value() >= _uvHigh)
        symbol = 5;
    else
        symbol = sunny(z) ? 0 : 1;
    set(WxId::WeatherSymbol, symbol);
}

bool WeatherModule::sunny(uint8_t zambretti)
{
    const MeasuredValue &light = val(WxId::Light);
#ifdef OPENKNX_SUN_POSITION
    if (openknx.sun.isSunCalculatioValid())
    {
        const float elevation = openknx.sun.elevation();
        if (elevation > 3.0f && light.valid())
        {
            // Klarer Himmel nach Haurwitz (W/m2) mit ca. 110 lx pro W/m2
            const float s = sinf(elevation * DEG_TO_RAD);
            const float clearSky = 110.0f * 1098.0f * s * expf(-0.057f / s);
            const float ratio = light.value() / clearSky;
            // Glaettung ueber ca. 5 min (Aufruf alle 10 s), damit einzelne Wolken nicht flackern
            _lightRatio = isnan(_lightRatio) ? ratio : _lightRatio + (ratio - _lightRatio) * (10.0f / 300.0f);
            // Hysterese 10 Prozentpunkte
            if (_lightRatio >= _sunnyRatio)
                _sunny = true;
            else if (_lightRatio < _sunnyRatio - 0.1f)
                _sunny = false;
            return _sunny;
        }
        // Nacht oder Daemmerung: Prognose A-F (schoenes Wetter) gilt als klar
        _lightRatio = NAN;
        _sunny = zambretti >= 1 && zambretti <= 6;
        return _sunny;
    }
#endif
    // Ohne Uhrzeit/Standort: feste Helligkeitsschwelle
    _sunny = light.valid() && light.value() >= 20000.0f;
    return _sunny;
}

void WeatherModule::updateFault(uint16_t ko, bool &state, bool faulty, const char *name)
{
    if (state == faulty)
        return;
    state = faulty;
    if (faulty)
        logErrorP("Stoerung: %s antwortet nicht", name);
    else
        logInfoP("%s wieder verfuegbar", name);
    if (!_configured || !Diag::koExists(ko))
        return;
    if (_sendAllowed)
        knx.getGroupObject(ko).value(faulty, DPT_Alarm);
    else
        knx.getGroupObject(ko).valueNoSend(faulty, DPT_Alarm);
}

void WeatherModule::sendValues()
{
    if (!delayCheck(_lastSendTime, _sendSpacing))
        return;
    // Reihum hoechstens ein Telegramm pro Durchlauf
    for (uint8_t n = 0; n < WX_VALUE_COUNT; n++)
    {
        MeasuredValue &v = _values[_nextSend];
        _nextSend = (_nextSend + 1) % WX_VALUE_COUNT;
        if (v.loop())
        {
            _lastSendTime = millis();
            return;
        }
    }
}

void WeatherModule::publishMqtt()
{
#ifdef OPENKNX_MQTT
    if (_mqttInterval == 0 || !delayCheck(_mqttTimer, _mqttInterval))
        return;
    _mqttTimer = millis();
    if (!openknxNetwork.mqtt.connected())
        return;

    JsonDocument doc;
    doc["wifi_rssi"] = WiFi.RSSI();
    doc["uptime"] = millis() / 1000;
    for (uint8_t i = 0; i < WX_VALUE_COUNT; i++)
    {
        const MeasuredValue &v = _values[i];
        if (!v.valid())
            continue;
        const WxFmt fmt = WX_VALUES[i].fmt;
        if (fmt == WxFmt::Forecast || fmt == WxFmt::MoonText || fmt == WxFmt::WarnText || fmt == WxFmt::DwdText || fmt == WxFmt::Time ||
            fmt == WxFmt::Date)
        {
            doc[WX_VALUES[i].mqttKey] = MeasuredValue::format(v.value(), &WX_VALUES[i], false);
            // Texte zusaetzlich ungekuerzt als <feld>_long (das Feld selbst bleibt der 14-Zeichen-Text wie am KO)
            if (fmt == WxFmt::Forecast || fmt == WxFmt::MoonText || fmt == WxFmt::WarnText)
                doc[std::string(WX_VALUES[i].mqttKey) + "_long"] = MeasuredValue::format(v.value(), &WX_VALUES[i], false, true);
            int64_t unix;
            if (MeasuredValue::toUnix(v.value(), &WX_VALUES[i], unix))
                doc[std::string(WX_VALUES[i].mqttKey) + "_unix"] = unix;
            continue;
        }
        const float f = powf(10.0f, WX_VALUES[i].decimals);
        doc[WX_VALUES[i].mqttKey] = roundf(v.value() * f) / f;
    }
    if (_sdsEnabled)
        doc["fault_sds"] = _faultSds;
    if (dwdWarnings.enabled())
    {
        const DwdWarnings::Result r = dwdWarnings.result();
        if (r.valid)
        {
            doc["dwd_headline"] = r.headline;
            doc["dwd_onset"] = r.onset;
            doc["dwd_expires"] = r.expires;
        }
    }
    doc["fault_wn90"] = _faultWn90;

    std::string payload;
    if (serializeJsonSafe(doc, payload))
        openknxNetwork.mqtt.publishP("state", payload);
#endif
}

// Line Protocol: Kommas, Leerzeichen und Gleichheitszeichen in Namen und Tags maskieren
static std::string influxEscape(const std::string &s, bool measurement = false)
{
    std::string out;
    for (char c : s)
    {
        if (c == ',' || c == ' ' || (!measurement && c == '='))
            out += '\\';
        out += c;
    }
    return out;
}

std::string WeatherModule::buildInfluxBody()
{
    // Schema wie im bisherigen Node-RED-Flow: ein Punkt je Sensor, unterschieden ueber den Tag sensor
    std::string tags;
    if (!_influxFloor.empty())
        tags += ",floor=" + influxEscape(_influxFloor);
    if (!_influxRoom.empty())
        tags += ",room=" + influxEscape(_influxRoom);
    const std::string base = _influxSensor.empty() ? "wn90" : _influxSensor;
    const struct
    {
        WxSensor sensor;
        std::string tag;
    } points[] = {{WxSensor::None, base}, {WxSensor::OneWire, base + "_1wire"}, {WxSensor::Sds, base + "_sds011"}};

    std::string body;
    char num[24];
    for (const auto &pt : points)
    {
        std::string fields;
        if (pt.sensor == WxSensor::None)
            fields = "rssi=" + std::to_string(WiFi.RSSI());
        for (uint8_t i = 0; i < WX_VALUE_COUNT; i++)
        {
            const WxValueDef &def = WX_VALUES[i];
            if (def.sensor != pt.sensor || !_values[i].valid())
                continue;
            int64_t unix;
            if (MeasuredValue::toUnix(_values[i].value(), &def, unix))
            {
                // Zeitpunkte als Unixzeit (Gleitkommafeld, wie alle Felder)
                if (!fields.empty())
                    fields += ',';
                fields += std::string(def.mqttKey) + "_unix=" + std::to_string(unix);
                continue;
            }
            if (!def.influxKey)
                continue;
            // Immer Gleitkomma (ohne i-Suffix), damit keine Feldtyp-Konflikte entstehen
            snprintf(num, sizeof(num), "%.*f", def.decimals, _values[i].value());
            if (!fields.empty())
                fields += ',';
            fields += def.influxKey;
            fields += '=';
            fields += num;
        }
        if (fields.empty())
            continue;
        body += influxEscape(_influxMeasurement, true) + tags + ",sensor=" + influxEscape(pt.tag) + " " + fields + "\n";
    }
    return body;
}

std::string WeatherModule::buildStatusJson()
{
    JsonDocument doc;
    doc["fw"] = WX_FIRMWARE_VERSION;
    doc["up"] = millis() / 1000;
    doc["rssi"] = WiFi.RSSI();
    doc["time"] = openknx.time.isValid();
#ifdef OPENKNX_MQTT
    doc["mqtt"] = openknxNetwork.mqtt.connected();
#endif
    doc["wn90"] = _wn90.faulty() ? "Stoerung" : (_wn90.ok() ? "ok" : "keine Daten");
    if (_sdsEnabled)
        doc["sds"] = _sds.faulty() ? "Stoerung" : "ok";
    if (_oneWireEnabled)
        doc["ow"] = _oneWire.faulty() ? "Stoerung" : "ok";
    if (dwdWarnings.enabled())
    {
        const DwdWarnings::Result r = dwdWarnings.result();
        doc["dwd"] = dwdWarnings.faulty() ? "Fehler" : (r.valid ? "ok" : "wartet");
        if (r.valid && !r.headline.empty())
            doc["dwd_headline"] = r.headline;
    }
    if (_influx.enabled())
        doc["influx"] = _influx.lastCode() == 204 ? "ok" : (_influx.okCount() + _influx.errorCount() == 0 ? "wartet" : "Fehler");

    JsonArray values = doc["values"].to<JsonArray>();
    for (uint8_t i = 0; i < WX_VALUE_COUNT; i++)
    {
        const WxValueDef &def = WX_VALUES[i];
        const MeasuredValue &v = _values[i];
        // Werte fehlender Sensoren und abgeschalteter Fassaden gar nicht anzeigen
        if (def.sensor == WxSensor::Sds && !_sdsEnabled)
            continue;
        if (def.sensor == WxSensor::OneWire && !_oneWireEnabled)
            continue;
        if (def.condParam != 0xFFFF && !v.active())
            continue;
        JsonObject o = values.add<JsonObject>();
        o["n"] = def.label;
        o["g"] = def.group;
        o["k"] = def.mqttKey;
        o["u"] = def.unitUtf8;
        o["d"] = def.decimals;
        o["a"] = v.active();
        if (def.dpts[0].main == 1)
            o["b"] = true;
        const float f = powf(10.0f, def.decimals);
        if (v.valid())
            o["v"] = roundf(v.value() * f) / f;
        else
            o["v"] = nullptr;
        if (v.sent())
            o["s"] = roundf(v.lastSent() * f) / f;
        else
            o["s"] = nullptr;
        o["f"] = (uint8_t)def.fmt;
        if (def.fmt != WxFmt::Number && def.fmt != WxFmt::Duration)
        {
            if (v.valid())
                o["t"] = MeasuredValue::format(v.value(), &def, false, true);
            if (v.sent())
                o["ts"] = MeasuredValue::format(v.lastSent(), &def, false, true);
        }
    }

    std::string json;
    if (!serializeJsonSafe(doc, json))
        json.clear();
    return json;
}

// ---------------------------------------------------------------------------
// Remanenz: Regenstatistik
// ---------------------------------------------------------------------------
uint16_t WeatherModule::flashSize()
{
    return 1 + RainStatistics::FLASH_SIZE + MINMAX_FLASH_SIZE + Season::FLASH_SIZE + SOLAR_FLASH_SIZE + SUNSHINE_FLASH_SIZE +
           Irrigation::FLASH_SIZE + DegreeDays::FLASH_SIZE + PressureTrend::FLASH_SIZE;
}

void WeatherModule::writeFlash()
{
    openknx.flash.writeByte(FLASH_VERSION);
    _rain.writeFlash();
    openknx.flash.writeInt(_minMax.dayKey);
    openknx.flash.writeFloat(_minMax.tempMin);
    openknx.flash.writeFloat(_minMax.tempMax);
    openknx.flash.writeFloat(_minMax.gustMax);
    openknx.flash.writeFloat(_minMax.tempMinYesterday);
    openknx.flash.writeFloat(_minMax.tempMaxYesterday);
    openknx.flash.writeFloat(_minMax.gustMaxYesterday);
    _season.writeFlash();
    openknx.flash.writeFloat(_minMax.radToday);
    openknx.flash.writeFloat(_minMax.radYesterday);
    openknx.flash.writeFloat(_minMax.pvToday);
    openknx.flash.writeFloat(_minMax.pvYesterday);
    openknx.flash.writeFloat(_minMax.sunshineToday);
    openknx.flash.writeFloat(_minMax.sunshineYesterday);
    _irrigation.writeFlash();
    _degreeDays.writeFlash();
    _pressure.writeFlash();
}

void WeatherModule::readFlash(const uint8_t *data, const uint16_t size)
{
    if (size == 0)
        return;
    const uint8_t version = openknx.flash.readByte();
    if (version < 1 || version > FLASH_VERSION)
        return;
    _rain.readFlash(data, RainStatistics::FLASH_SIZE);
    if (version < 2 || size < 1 + RainStatistics::FLASH_SIZE + MINMAX_FLASH_SIZE)
        return;
    _minMax.dayKey = openknx.flash.readInt();
    _minMax.tempMin = openknx.flash.readFloat();
    _minMax.tempMax = openknx.flash.readFloat();
    _minMax.gustMax = openknx.flash.readFloat();
    _minMax.tempMinYesterday = openknx.flash.readFloat();
    _minMax.tempMaxYesterday = openknx.flash.readFloat();
    _minMax.gustMaxYesterday = openknx.flash.readFloat();
    if (version < 3 || size < 1 + RainStatistics::FLASH_SIZE + MINMAX_FLASH_SIZE + Season::FLASH_SIZE)
        return;
    _season.readFlash();
    if (version < 4 || size < flashSize() - SUNSHINE_FLASH_SIZE)
        return;
    _minMax.radToday = openknx.flash.readFloat();
    _minMax.radYesterday = openknx.flash.readFloat();
    _minMax.pvToday = openknx.flash.readFloat();
    _minMax.pvYesterday = openknx.flash.readFloat();
    if (isnan(_minMax.radToday))
        _minMax.radToday = 0;
    if (isnan(_minMax.pvToday))
        _minMax.pvToday = 0;
    if (version < 5 || size < flashSize() - Irrigation::FLASH_SIZE - DegreeDays::FLASH_SIZE)
        return;
    _minMax.sunshineToday = openknx.flash.readFloat();
    _minMax.sunshineYesterday = openknx.flash.readFloat();
    if (isnan(_minMax.sunshineToday))
        _minMax.sunshineToday = 0;
    if (version < 6 || size < flashSize() - PressureTrend::FLASH_SIZE)
        return;
    _irrigation.readFlash();
    _degreeDays.readFlash();
    if (version < 7 || size < flashSize())
        return;
    _pressure.readFlash();
}

// ---------------------------------------------------------------------------
// Konsole
// ---------------------------------------------------------------------------
void WeatherModule::showHelp()
{
    openknx.console.printHelpLine("wx values", "Alle Messwerte mit Sendezustand");
    openknx.console.printHelpLine("wx sensors", "Zustand von WN90LP, SDS011 und 1-Wire");
    openknx.console.printHelpLine("wx raw", "Rohwerte der Modbus-Register");
    openknx.console.printHelpLine("wx rain", "Regenstatistik");
    openknx.console.printHelpLine("wx pressure", "Luftdruckverlauf und Tendenz");
    openknx.console.printHelpLine("wx forecast", "Prognose und Wettersymbol");
    openknx.console.printHelpLine("wx minmax", "Minimum/Maximum, Windalarme, Daemmerung, Warnungen");
    openknx.console.printHelpLine("wx season", "Sommer/Winter, Tagesmittel und Kenntage");
    openknx.console.printHelpLine("wx solar", "Strahlung, PV-Schaetzung und Fassaden");
    openknx.console.printHelpLine("wx astro", "Sonnenzeiten, Daemmerung und Mond");
    openknx.console.printHelpLine("wx climate", "Lueftung, Bewaesserung, Gradtage");
    openknx.console.printHelpLine("wx perf", "Laufzeiten seit dem letzten Aufruf (Diagnose)");
    openknx.console.printHelpLine("wx 1w scan", "1-Wire-Bus nach Sensoren durchsuchen");
    openknx.console.printHelpLine("wx led <pin> <r> <g> <b>", "WS2812 an GPIO direkt ansteuern (Diagnose)");
}

bool WeatherModule::processCommand(const std::string cmd, bool diagnoseKo)
{
    if (cmd.compare(0, 2, "wx") != 0)
        return false;

    if (diagnoseKo)
    {
        // Kurzabfragen ueber das Diagnose-KO (max. 14 Zeichen Antwort)
        auto diag = [this](WxId id, const char *fmt) {
            const MeasuredValue &v = val(id);
            if (v.valid())
                openknx.console.writeDiagnoseKo(fmt, v.value());
            else
                openknx.console.writeDiagnoseKo("ungueltig");
        };
        if (cmd == "wx t")
            diag(WxId::Temp, "T %.1f C");
        else if (cmd == "wx h")
            diag(WxId::Humidity, "H %.0f %%");
        else if (cmd == "wx w")
            diag(WxId::WindSpeed, "W %.1f m/s");
        else if (cmd == "wx p")
            diag(WxId::Pressure, "P %.1f hPa");
        else if (cmd == "wx r")
            diag(WxId::RainToday, "R %.2f mm");
        else
            openknx.console.writeDiagnoseKo("wx t|h|w|p|r");
        return true;
    }

    if (cmd == "wx" || cmd == "wx values")
        showValues();
    else if (cmd == "wx sensors")
        showSensors();
    else if (cmd == "wx raw")
        showRaw();
    else if (cmd == "wx rain")
        showRain();
    else if (cmd == "wx pressure")
        showPressure();
    else if (cmd == "wx forecast")
        showForecast();
    else if (cmd == "wx minmax")
        showMinMax();
    else if (cmd == "wx season")
        showSeason();
    else if (cmd == "wx solar")
        showSolar();
    else if (cmd == "wx astro")
        showAstro();
    else if (cmd == "wx climate")
        showClimate();
    else if (cmd == "wx perf")
        Diag::perf();
    else if (cmd == "wx 1w scan")
    {
        if (_oneWireEnabled)
            _oneWire.scan();
        else
            logInfoP("1-Wire ist in der ETS nicht aktiviert");
    }
    else if (cmd.compare(0, 7, "wx led ") == 0)
    {
        // Diagnose: wx led <pin> <r> <g> <b> - Pin-Belegung anzeigen und WS2812 direkt ansteuern
        int pin = -1, r = 0, g = 0, b = 0;
        if (sscanf(cmd.c_str() + 7, "%d %d %d %d", &pin, &r, &g, &b) < 1 || pin < 0 || pin >= SOC_GPIO_PIN_COUNT)
            logInfoP("Aufruf: wx led <pin> <r> <g> <b>");
        else
        {
            logInfoP("GPIO %d belegt durch: %s", pin, perimanGetTypeName(perimanGetPinBusType(pin)));
            RgbLed::releasePin(pin);
            rgbLedWrite(pin, r, g, b);
            logInfoP("GPIO %d gesendet: R %d G %d B %d, jetzt belegt durch: %s", pin, r, g, b,
                     perimanGetTypeName(perimanGetPinBusType(pin)));
        }
    }
    else
        return false;
    return true;
}

void WeatherModule::formatValue(char *buf, size_t len, const MeasuredValue &v)
{
    snprintf(buf, len, "%s", v.valid() ? MeasuredValue::format(v.value(), v.def(), true).c_str() : "-");
}

void WeatherModule::showValues()
{
    logInfoP("Messwerte (A = aktiv in der ETS, zuletzt gesendet in Klammern):");
    logIndentUp();
    char value[24], sent[24];
    for (uint8_t i = 0; i < WX_VALUE_COUNT; i++)
    {
        const MeasuredValue &v = _values[i];
        formatValue(value, sizeof(value), v);
        if (v.sent())
            snprintf(sent, sizeof(sent), "(%s)", MeasuredValue::format(v.lastSent(), v.def(), false).c_str());
        else
            snprintf(sent, sizeof(sent), "%s", v.active() ? "(-)" : "");
        logInfoP("%-44.44s %c %-16s %s", WX_VALUES[i].name, v.active() ? 'A' : ' ', value, sent);
    }
    logIndentDown();
}

void WeatherModule::showSensors()
{
    logInfoP("WN90LP: %s, %lu Abfragen, %lu Fehler (letzter Fehlercode 0x%02X)", _wn90.faulty() ? "STOERUNG" : (_wn90.ok() ? "ok" : "keine Daten"),
             _wn90.reads(), _wn90.errors(), _wn90.lastError());
    if (_sdsEnabled)
        logInfoP("SDS011: %s, Messintervall %u min, %lu Messungen, %lu Pruefsummenfehler, letzte vor %lu s",
                 _sds.faulty() ? "STOERUNG" : "ok", _sds.dutyCycle(), _sds.frames(), _sds.checksumErrors(), _sds.lastFrameAge() / 1000);
    else
        logInfoP("SDS011: nicht aktiviert");
    if (_oneWireEnabled)
        logInfoP("1-Wire: %s, Sensor %s, %lu Fehler in Folge", _oneWire.faulty() ? "STOERUNG" : "ok", _oneWire.found() ? "gefunden" : "nicht gefunden",
                 _oneWire.failures());
    else
        logInfoP("1-Wire: nicht aktiviert");
    const char *source = "";
    const MeasuredValue &temp = val(WxId::Temp);
    const float t = calcTemperature(temp.valid() ? temp.value() : NAN, &source);
    logInfoP("Temperatur fuer Berechnungen: %.2f C aus %s", t, source);
    Diag::report();
    logInfoP("Uhrzeit: %s, Firmware %s%s", openknx.time.isValid() ? "gueltig" : "unbekannt", WX_FIRMWARE_VERSION,
             OtaGuard::pending() ? " (noch nicht bestaetigt, Rollback aktiv)" : "");
    if (dwdWarnings.enabled())
    {
        const DwdWarnings::Result r = dwdWarnings.result();
        logInfoP("DWD: %lu ok, %lu Fehler, letzter Code %d, Stufe %u (inkl. angekuendigt %u), %s", r.ok, r.errors, r.lastCode, r.levelNow,
                 r.levelAll, r.headline.empty() ? "keine Warnung" : r.headline.c_str());
    }
    else
        logInfoP("DWD-Warnungen: nicht aktiviert");
    if (_influx.enabled())
        logInfoP("InfluxDB: %lu ok, %lu Fehler, letzter Code %d, letzter Erfolg vor %lu s (%s)", _influx.okCount(), _influx.errorCount(),
                 _influx.lastCode(), _influx.lastOkAge(), _influx.url().c_str());
    else
        logInfoP("InfluxDB: nicht aktiviert");
}

void WeatherModule::showRaw()
{
    static const char *names[] = {"Licht", "UV", "Temperatur", "Feuchte", "Wind", "Boe", "Richtung", "Regen", "Luftdruck", "Regenzaehler"};
    logInfoP("Modbus-Register ab 0x0165 (%s):", _wn90.ok() ? "aktuell" : "keine gueltigen Daten");
    logIndentUp();
    for (uint8_t i = 0; i < Wn90Sensor::REG_COUNT; i++)
        logInfoP("0x%04X %-12s 0x%04X = %u", 0x0165 + i, names[i], _wn90.raw((Wn90Sensor::Reg)i), _wn90.raw((Wn90Sensor::Reg)i));
    logIndentDown();
}

void WeatherModule::showRain()
{
    if (!_rain.hasData())
    {
        logInfoP("Regenstatistik: noch keine Daten");
        return;
    }
    logInfoP("Regen gesamt %.2f mm, heute %.2f mm, Intensitaet %.1f mm/h, regnet: %s", _rain.total(), _rain.today(), _rain.rate(),
             _rain.raining() ? "ja" : "nein");
    logInfoP("5 min %.2f | 1 h %.2f | 6 h %.2f | 24 h %.2f | 48 h %.2f | 72 h %.2f mm", _rain.amountMinutes(5), _rain.amountMinutes(60),
             _rain.amountHours(6), _rain.amountHours(24), _rain.amountHours(48), _rain.amountHours(72));
    logInfoP("Werte im Puffer: %u Minuten, %u Stunden", _rain.minuteSamples(), _rain.hourSamples());
}

void WeatherModule::showPressure()
{
    logInfoP("Luftdruck: %u Werte (alle 15 min), Tendenz 1 h %.1f hPa, 3 h %.1f hPa", _pressure.samples(), _pressure.trend(1), _pressure.trend(3));
    logIndentUp();
    for (uint8_t i = 0; i < _pressure.samples(); i++)
        logInfoP("vor %3u min: %.1f hPa", i * 15, _pressure.sample(i));
    logIndentDown();
}

void WeatherModule::showForecast()
{
    static const char *symbols[] = {"sonnig", "bewoelkt", "Schauer", "starker Regen", "Gewitter", "hoher UV-Index"};
    const MeasuredValue &z = val(WxId::Forecast);
    const MeasuredValue &sym = val(WxId::WeatherSymbol);
    logInfoP("Luftdruck Meereshoehe %.1f hPa, Tendenz 3 h %.1f hPa (1 h %.1f hPa)", val(WxId::PressureSea).value(), _pressure.trend(3),
             _pressure.trend(1));
    if (z.valid())
        logInfoP("Zambretti: %c (%u) %s", 'A' + (int)z.value() - 1, (uint8_t)z.value(), Forecast::text((uint8_t)z.value(), true));
    else
        logInfoP("Zambretti: noch keine Prognose (Luftdruck/Tendenz fehlen)");
    if (sym.valid())
        logInfoP("Wettersymbol: %u (%s), Regen %s, Intensitaet %.1f mm/h, Helligkeitsanteil %.0f %%", (uint8_t)sym.value(),
                 symbols[min<uint8_t>((uint8_t)sym.value(), 5)], _rain.raining() ? "ja" : "nein", val(WxId::RainRate).value(),
                 _lightRatio * 100.0f);
    else
        logInfoP("Wettersymbol: keine Daten");
}

void WeatherModule::showMinMax()
{
    logInfoP("Temperatur heute %.1f .. %.1f C, gestern %.1f .. %.1f C (Tag %lu)", _minMax.tempMin, _minMax.tempMax, _minMax.tempMinYesterday,
             _minMax.tempMaxYesterday, _minMax.dayKey);
    logInfoP("Boee heute max. %.1f m/s, gestern %.1f m/s, 10-min-Mittel %.1f m/s, Boee 10 min %.1f m/s", _minMax.gustMax,
             _minMax.gustMaxYesterday, _windStats.average(), _windStats.maxGust());
    static const char *sources[] = {"Boee", "Wind", "Wind 10 min"};
    for (uint8_t i = 0; i < 3; i++)
        logInfoP("Windalarm %u: %s, %s ab %.1f m/s%s", i + 1, _alarms[i].state ? "AUSGELOEST" : "aus", sources[min<uint8_t>(_alarms[i].source, 2)],
                 _alarms[i].threshold, _alarms[i].timing ? " (Verzoegerung laeuft)" : "");
    logInfoP("Daemmerung: %s, Frost: %s, Glaettegefahr: %s", _night < 0 ? "unbekannt" : (_night ? "Nacht" : "Tag"), _frost ? "ja" : "nein",
             _ice ? "ja" : "nein");
}

void WeatherModule::showSeason()
{
    const int8_t summer = _season.summer();
    logInfoP("Sommer/Winter: %s, Mittel %u Tage %.1f C, Tagesmittel gestern %.1f C", summer < 0 ? "noch unbekannt" : (summer ? "Sommer" : "Winter"),
             _season.days(), _season.average(), _season.yesterday());
    logInfoP("Heute: 7 Uhr %.1f C, 14 Uhr %.1f C, Abend %.1f C, Tagesmittel bisher %.1f C", _season.reading(0), _season.reading(1),
             _season.reading(2), _season.todayMean());
    logInfoP("Kenntage heute: Sommertag %s, heisser Tag %s, Frosttag %s, Eistag %s", _minMax.tempMax >= 25.0f ? "ja" : "nein",
             _minMax.tempMax >= 30.0f ? "ja" : "nein", _minMax.tempMin < 0.0f ? "ja" : "nein", _minMax.tempMax < 0.0f ? "ja" : "nein");
}

void WeatherModule::showSolar()
{
    logInfoP("Globalstrahlung %.0f W/m2, Bewoelkung %.0f %%, heute %.2f kWh/m2, gestern %.2f kWh/m2", _ghi, val(WxId::CloudCover).value(),
             _minMax.radToday, _minMax.radYesterday);
#ifdef OPENKNX_SUN_POSITION
    if (openknx.sun.isSunCalculatioValid())
        logInfoP("Sonne: Azimut %.1f, Hoehe %.1f Grad", openknx.sun.azimuth(), openknx.sun.elevation());
    else
        logInfoP("Sonne: unbekannt (Uhrzeit/Standort fehlen)");
#endif
    if (_pvPeak > 0)
        logInfoP("PV %.1f kWp, %u Grad / %u Grad: %.0f W, heute %.2f kWh, gestern %.2f kWh", _pvPeak, _pvTilt, _pvAzimuth, val(WxId::PvPower).value(),
                 _minMax.pvToday, _minMax.pvYesterday);
    for (uint8_t i = 0; i < 6; i++)
    {
        const Facade &f = _facades[i];
        if (f.active)
            logInfoP("Fassade %u (%u Grad): %s, %.0f W/m2%s", i + 1, f.azimuth, f.sun ? "Sonne" : "keine Sonne", f.irradiance,
                     f.timing ? " (Verzoegerung laeuft)" : "");
    }
}

void WeatherModule::showAstro()
{
    if (!_configured || !openknx.time.isValid())
    {
        logInfoP("Sonnenzeiten: Uhrzeit oder Standort fehlen");
        return;
    }
    auto t = [this](WxId id) { return MeasuredValue::format(val(id).valid() ? val(id).value() : NAN, &WX_VALUES[(uint8_t)id], false); };
    logInfoP("Sonne: Aufgang %s (%.0f Grad), Hoechststand %s (%.1f Grad), Untergang %s (%.0f Grad), Tag %s", t(WxId::Sunrise).c_str(),
             _sunDay.riseAzimuth, t(WxId::SolarNoon).c_str(), _sunDay.transitElevation, t(WxId::Sunset).c_str(), _sunDay.setAzimuth,
             t(WxId::DayLength).c_str());
    logInfoP("Daemmerung buergerlich %s-%s, nautisch %s-%s, astronomisch %s-%s", t(WxId::CivilDawn).c_str(), t(WxId::CivilDusk).c_str(),
             t(WxId::NauticalDawn).c_str(), t(WxId::NauticalDusk).c_str(), t(WxId::AstroDawn).c_str(), t(WxId::AstroDusk).c_str());
    logInfoP("Mond: %s, %.0f %% beleuchtet, Alter %.1f Tage, Aufgang %s, Untergang %s, Vollmond %s, Neumond %s", t(WxId::MoonPhaseText).c_str(),
             val(WxId::MoonIllumination).value(), val(WxId::MoonAge).value(), t(WxId::Moonrise).c_str(), t(WxId::Moonset).c_str(),
             t(WxId::NextFullMoon).c_str(), t(WxId::NextNewMoon).c_str());
}

void WeatherModule::showClimate()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        const Room &r = _rooms[i];
        if (!r.active)
            continue;
        const WxId base = (WxId)((uint8_t)WxId::Room1Vent + i * 4);
        logInfoP("Raum %u: %.1f C / %.0f %% (vor %lu s / %lu s), abs. %.2f g/m3, Unterschied %.2f g/m3 -> %s", i + 1, r.temp, r.hum,
                 (millis() - r.tempTime) / 1000, (millis() - r.humTime) / 1000, val((WxId)((uint8_t)base + 1)).value(),
                 val((WxId)((uint8_t)base + 3)).value(), r.vent ? "LUEFTEN" : "nicht lueften");
    }
    logInfoP("Bewaesserung: ET0 heute %.2f mm (aktuell %.2f mm/h), gestern %.2f mm, Bilanz %.1f mm -> %s", _irrigation.et0Today(),
             _irrigation.rate(), _irrigation.et0Yesterday(), _irrigation.balance(), _irrigation.irrigate() ? "BEWAESSERN" : "kein Bedarf");
    logInfoP("Heizgradtage: Monat %.1f, Vormonat %.1f, Jahr %.1f, Vorjahr %.1f", _degreeDays.hddMonth(), _degreeDays.hddLastMonth(),
             _degreeDays.hddYear(), _degreeDays.hddLastYear());
    logInfoP("Warnung: Stufe %u (%s); Wind %u, Starkregen %u, Dauerregen %u, Frost/Glaette %u, Hitze %u, UV %u", _warnings.overall(),
             Warnings::text(_warnings.code(), true), _warnings.level(Warnings::Wind), _warnings.level(Warnings::HeavyRain),
             _warnings.level(Warnings::LongRain), _warnings.level(Warnings::Frost), _warnings.level(Warnings::Heat), _warnings.level(Warnings::Uv));
    logInfoP("Kuehlgradtage: Monat %.1f, Vormonat %.1f, Jahr %.1f, Vorjahr %.1f", _degreeDays.cddMonth(), _degreeDays.cddLastMonth(),
             _degreeDays.cddYear(), _degreeDays.cddLastYear());
}
