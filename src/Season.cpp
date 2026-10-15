#include "Season.h"
#include "OpenKNX.h"

void Season::setup(uint8_t method, uint8_t eveningHour, uint8_t days, float threshold, float hysteresis)
{
    _method = method;
    _hours[2] = eveningHour;
    _days = constrain(days, 1, MAX_DAYS);
    _threshold = threshold;
    _hysteresis = max(hysteresis, 0.0f);
}

void Season::feed(float temp)
{
    if (isnan(temp))
        return;
    _sum += temp;
    if (_samples < 65535)
        _samples++;
    else
    {
        // Ueberlauf vermeiden: Mittel beibehalten, Gewicht halbieren
        _sum /= 2;
        _samples /= 2;
    }
}

bool Season::loop(uint32_t dayKey, uint8_t hour, float temp)
{
    bool newDay = false;
    if (_dayKey != dayKey)
    {
        if (_dayKey != 0)
        {
            finishDay();
            newDay = true;
        }
        _dayKey = dayKey;
        _sum = 0;
        _samples = 0;
        for (float &r : _readings)
            r = NAN;
    }

    // Messwerte zu den Terminen 7, 14 und 21/22 Uhr (erster Wert in der jeweiligen Stunde)
    if (!isnan(temp))
        for (uint8_t i = 0; i < 3; i++)
            if (hour == _hours[i] && isnan(_readings[i]))
                _readings[i] = temp;
    return newDay;
}

float Season::todayMean() const
{
    if (_method == 0 && !isnan(_readings[0]) && !isnan(_readings[1]) && !isnan(_readings[2]))
        return (_readings[0] + _readings[1] + 2 * _readings[2]) / 4.0f;
    // DIN-Termine unvollstaendig (z. B. Neustart): Mittel aller Messwerte
    return _samples ? _sum / _samples : NAN;
}

void Season::finishDay()
{
    const float mean = todayMean();
    if (isnan(mean))
        return;
    _lastMean = mean;
    _means[_head] = mean;
    _head = (_head + 1) % MAX_DAYS;
    if (_count < MAX_DAYS)
        _count++;

    const float avg = average();
    if (avg > _threshold + _hysteresis)
        _summer = 1;
    else if (avg <= _threshold - _hysteresis || _summer < 0)
        _summer = 0;
    logInfo("Weather", "Tagesmittel %.1f C, Mittel %u Tage %.1f C -> %s", mean, min(_count, _days), avg, _summer ? "Sommer" : "Winter");
}

float Season::average() const
{
    const uint8_t n = min(_count, _days);
    if (n == 0)
        return NAN;
    float sum = 0;
    for (uint8_t i = 1; i <= n; i++)
        sum += _means[(_head + MAX_DAYS - i) % MAX_DAYS];
    return sum / n;
}

float Season::yesterday() const
{
    return _lastMean;
}

void Season::writeFlash()
{
    openknx.flash.writeInt(_dayKey);
    openknx.flash.writeFloat(_sum);
    openknx.flash.writeWord(_samples);
    for (float r : _readings)
        openknx.flash.writeFloat(r);
    for (float m : _means)
        openknx.flash.writeFloat(m);
    openknx.flash.writeByte(_count);
    openknx.flash.writeByte(_head);
    openknx.flash.writeByte((uint8_t)_summer);
    openknx.flash.writeFloat(_lastMean);
}

void Season::readFlash()
{
    _dayKey = openknx.flash.readInt();
    _sum = openknx.flash.readFloat();
    _samples = openknx.flash.readWord();
    for (float &r : _readings)
        r = openknx.flash.readFloat();
    for (float &m : _means)
        m = openknx.flash.readFloat();
    _count = min<uint8_t>(openknx.flash.readByte(), MAX_DAYS);
    _head = openknx.flash.readByte() % MAX_DAYS;
    _summer = (int8_t)openknx.flash.readByte();
    if (_summer < -1 || _summer > 1)
        _summer = -1;
    _lastMean = openknx.flash.readFloat();
    if (isnan(_sum))
    {
        _sum = 0;
        _samples = 0;
    }
}
