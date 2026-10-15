#include "WeatherModule.h"
#include "NetworkModule.h"
#include "WeatherCalc.h"
#include <ArduinoJson.h>

WeatherModule openknxWeatherModule;

// Mindestabstand zwischen zwei Telegrammen, damit nach dem Start nicht alle Werte gleichzeitig kommen
static constexpr uint32_t SEND_SPACING = 25;
static constexpr uint8_t FLASH_VERSION = 1;

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
        sdsDutyCycle = ParamWX_SdsDutyCycle;
        _altitude = ParamWX_Altitude;
        _tempOffset = (float)ParamWX_TempOffset;
        rainStopDelay = ParamWX_RainStopDelay;
        rainWindow = ParamWX_RainWindow;
        rainMinAmount = (float)ParamWX_RainMinAmount;
        _mqttInterval = (uint32_t)ParamWX_MqttInterval * 1000;
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
    if (_rain.saveRequested())
        openknx.flash.save(true);

    if (configured && _sendAllowed)
        sendValues();
    if (configured)
        publishMqtt();
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

    if (!isnan(pressure))
    {
        _pressure.feed(pressure);
        set(WxId::PressureSea, (isnan(temp) || _altitude == 0) ? pressure : baroToSeaLevelC(pressure, temp, _altitude));
    }
    else
        val(WxId::PressureSea).invalidate();

    if (!isnan(temp) && !isnan(hum))
    {
        const float dew = dewPoint(temp, hum);
        set(WxId::AbsHumidity, absoluteHumidity(temp, hum));
        set(WxId::DewPoint, dew);
        set(WxId::FrostPoint, WeatherCalc::frostPoint(temp, hum));
        set(WxId::Humidex, humidex(temp, dew));
        set(WxId::HeatIndex, WeatherCalc::heatIndex(temp, hum));
        set(WxId::Discomfort, discomfortIndex(temp, hum));
    }
    else
    {
        for (WxId id : {WxId::AbsHumidity, WxId::DewPoint, WxId::FrostPoint, WxId::Humidex, WxId::HeatIndex, WxId::Discomfort})
            val(id).invalidate();
    }
    set(WxId::WindChill, (isnan(temp) || isnan(wind)) ? NAN : WeatherCalc::windChill(temp, wind));

    if (_wn90.valid(R::RainCounter))
        _rain.feed(_wn90.raw(R::RainCounter));
}

void WeatherModule::invalidateWn90()
{
    for (uint8_t i = 0; i < WX_VALUE_COUNT; i++)
    {
        const WxId id = (WxId)i;
        // Werte anderer Sensoren und die Regenstatistik (Zaehler bleibt gueltig) behalten
        if (WX_VALUES[i].sensor != WxSensor::None)
            continue;
        if (id >= WxId::RainTotal && id <= WxId::Raining)
            continue;
        _values[i].invalidate();
    }
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
    return 1 + RainStatistics::FLASH_SIZE;
}

void WeatherModule::writeFlash()
{
    openknx.flash.writeByte(FLASH_VERSION);
    _rain.writeFlash();
}

void WeatherModule::readFlash(const uint8_t *data, const uint16_t size)
{
    if (size == 0 || openknx.flash.readByte() != FLASH_VERSION)
        return;
    _rain.readFlash(data, size - 1);
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
