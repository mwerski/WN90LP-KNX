#include "OneWireSensor.h"
#include "OpenKNX.h"

OneWireSensor::OneWireSensor() : _oneWire(WX_ONEWIRE_PIN), _sensors(&_oneWire) {}

void OneWireSensor::setup()
{
    _sensors.begin();
    _sensors.setWaitForConversion(false);
    findSensor();
}

bool OneWireSensor::findSensor()
{
    _found = _sensors.getAddress(_address, 0);
    if (_found)
        _sensors.setResolution(_address, 12);
    return _found;
}

bool OneWireSensor::loop()
{
    if (!_converting)
    {
        if (!delayCheck(_timer, INTERVAL))
            return false;
        _timer = millis();
        if (!_found && !findSensor())
        {
            _failures++;
            _temperature = NAN;
            return true;
        }
        _sensors.requestTemperaturesByAddress(_address);
        _converting = true;
        return false;
    }

    if (!delayCheck(_timer, CONVERSION_TIME))
        return false;
    _converting = false;

    const float t = _sensors.getTempC(_address);
    // -127 = nicht erreichbar, 85 = Einschaltwert ohne Wandlung
    if (t == DEVICE_DISCONNECTED_C || t == 85.0f || t < -55.0f || t > 125.0f)
    {
        _failures++;
        if (faulty())
        {
            _temperature = NAN;
            _found = false;
        }
        return true;
    }
    _failures = 0;
    _temperature = t;
    return true;
}

void OneWireSensor::scan()
{
    _sensors.begin();
    const uint8_t count = _sensors.getDeviceCount();
    logInfo("Weather", "1-Wire: %u Sensor(en), parasitaere Versorgung: %s", count, _sensors.isParasitePowerMode() ? "ja" : "nein");
    logIndentUp();
    for (uint8_t i = 0; i < count; i++)
    {
        DeviceAddress a;
        if (_sensors.getAddress(a, i))
            logInfo("Weather", "%u: %02X%02X%02X%02X%02X%02X%02X%02X%s", i, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7],
                    i == 0 ? " (verwendet)" : "");
    }
    logIndentDown();
    findSensor();
}
