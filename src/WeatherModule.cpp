#include "WeatherModule.h"
#include "NetworkModule.h"
#include "WeatherCalc.h"
#include "Forecast.h"
#include <ArduinoJson.h>

WeatherModule openknxWeatherModule;

// Mindestabstand zwischen zwei Telegrammen, damit nach dem Start nicht alle Werte gleichzeitig kommen
static constexpr uint32_t SEND_SPACING = 25;
static constexpr uint8_t FLASH_VERSION = 2; // 1: nur Regen, 2: + Minimum/Maximum
static constexpr uint16_t MINMAX_FLASH_SIZE = 4 + 6 * 4;

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
        _values[i].init(def, configured && sensorOk);
    }

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
    knx.getGroupObject(WX_KO_FAULT_WN90).value(_faultWn90, DPT_Alarm);
    if (_sdsEnabled)
        knx.getGroupObject(WX_KO_FAULT_SDS).value(_faultSds, DPT_Alarm);
    if (_oneWireEnabled)
        knx.getGroupObject(WX_KO_FAULT_1W).value(_fault1W, DPT_Alarm);
}

void WeatherModule::loop(bool configured)
{
    if (_wn90.loop())
        processWn90();
    updateFault(WX_KO_FAULT_WN90, _faultWn90, _wn90.faulty(), "Wetterstation");
    if (_faultWn90)
        invalidateWn90();

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

    if (_oneWireEnabled && _oneWire.loop())
    {
        set(WxId::Temp1W, _oneWire.temperature());
        updateFault(WX_KO_FAULT_1W, _fault1W, _oneWire.faulty(), "1-Wire-Sensor");
    }

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
        updateForecast();
        updateSunAndTwilight();
        updateWarnings();
        checkMinMaxDay();
    }
    if (delayCheck(_alarmTimer, 500))
    {
        _alarmTimer = millis();
        updateWindAlarms();
    }
    if (_rain.saveRequested())
        openknx.flash.save(true);

    if (configured && _sendAllowed)
        sendValues();
    if (configured)
    {
        publishMqtt();
        _ha.loop(_values, WX_VALUE_COUNT);
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
        set((WxId)((uint8_t)WxId::WindAlarm1 + i), a.state ? 1.0f : 0.0f);
    }
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

static uint32_t dayKeyOf(const OpenKNX::DateTime &d)
{
    return d.year * 10000UL + d.month * 100UL + d.day;
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
        _minMax.tempMin = _minMax.tempMax = _minMax.gustMax = NAN;
    }
    // Ohne gespeicherten Tag (erste gueltige Uhrzeit): bisher gesammelte Werte gelten fuer heute
    _minMax.dayKey = today;
    openknx.flash.save(true);
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
    if (!_configured)
        return;
    if (_sendAllowed)
        knx.getGroupObject(ko).value(faulty, DPT_Alarm);
    else
        knx.getGroupObject(ko).valueNoSend(faulty, DPT_Alarm);
}

void WeatherModule::sendValues()
{
    if (!delayCheck(_lastSendTime, SEND_SPACING))
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
        if ((WxId)i == WxId::ForecastText)
        {
            doc[WX_VALUES[i].mqttKey] = Forecast::text((uint8_t)v.value(), true);
            continue;
        }
        const float f = powf(10.0f, WX_VALUES[i].decimals);
        doc[WX_VALUES[i].mqttKey] = roundf(v.value() * f) / f;
    }
    if (_sdsEnabled)
        doc["fault_sds"] = _faultSds;
    doc["fault_wn90"] = _faultWn90;

    std::string payload;
    serializeJson(doc, payload);
    openknxNetwork.mqtt.publishP("state", payload);
#endif
}

// ---------------------------------------------------------------------------
// Remanenz: Regenstatistik
// ---------------------------------------------------------------------------
uint16_t WeatherModule::flashSize()
{
    return 1 + RainStatistics::FLASH_SIZE + MINMAX_FLASH_SIZE;
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
    openknx.console.printHelpLine("wx 1w scan", "1-Wire-Bus nach Sensoren durchsuchen");
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
    else if (cmd == "wx 1w scan")
    {
        if (_oneWireEnabled)
            _oneWire.scan();
        else
            logInfoP("1-Wire ist in der ETS nicht aktiviert");
    }
    else
        return false;
    return true;
}

void WeatherModule::formatValue(char *buf, size_t len, const MeasuredValue &v)
{
    if (!v.valid())
        snprintf(buf, len, "-");
    else
        snprintf(buf, len, "%.*f %s", v.def()->decimals, v.value(), v.def()->unit);
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
            snprintf(sent, sizeof(sent), "(%.*f)", v.def()->decimals, v.lastSent());
        else
            snprintf(sent, sizeof(sent), "%s", v.active() ? "(-)" : "");
        logInfoP("%-36s %c %-14s %s", WX_VALUES[i].name, v.active() ? 'A' : ' ', value, sent);
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
    logInfoP("Uhrzeit: %s", openknx.time.isValid() ? "gueltig" : "unbekannt");
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
