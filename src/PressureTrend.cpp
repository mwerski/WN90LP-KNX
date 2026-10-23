#include "PressureTrend.h"
#include "OpenKNX.h"

void PressureTrend::feed(float pressure)
{
    const bool first = isnan(_current);
    _current = pressure;
    if (_restorePending)
        applyRestore();
    if ((first && _count == 0) || (!_restorePending && _count == 0))
    {
        _ring[0] = pressure;
        _head = 0;
        _count = 1;
        _timer = millis();
    }
}

void PressureTrend::loop()
{
    if (_restorePending)
    {
        applyRestore();
        // Ohne Uhrzeit laesst sich die Luecke nicht pruefen: nach 2 min neu beginnen
        if (_restorePending && millis() > 120000)
        {
            _restorePending = false;
            _count = 0;
        }
    }
    if (isnan(_current) || !delayCheck(_timer, STEP))
        return;
    _timer += STEP;
    _head = (_head + 1) % SIZE;
    _ring[_head] = _current;
    if (_count < SIZE)
        _count++;
    _lastEpoch = openknx.time.isValid() ? (uint32_t)openknx.time.getUtcTime().toTime_t() : 0;
}

void PressureTrend::writeFlash()
{
    openknx.flash.writeByte(_count);
    openknx.flash.writeByte(_head);
    for (float v : _ring)
        openknx.flash.writeFloat(v);
    openknx.flash.writeInt(_lastEpoch);
}

void PressureTrend::readFlash()
{
    _count = min<uint8_t>(openknx.flash.readByte(), SIZE);
    _head = openknx.flash.readByte() % SIZE;
    for (float &v : _ring)
        v = openknx.flash.readFloat();
    _lastEpoch = openknx.flash.readInt();
    // Erst mit gueltiger Uhrzeit pruefen, ob der Verlauf noch passt
    _restorePending = _count > 0 && _lastEpoch != 0 && _lastEpoch != 0xFFFFFFFF;
    if (!_restorePending)
        _count = 0;
}

void PressureTrend::applyRestore()
{
    // Fuer das Auffuellen werden Uhrzeit und ein aktueller Messwert gebraucht
    if (!openknx.time.isValid() || isnan(_current))
        return;
    _restorePending = false;
    const uint32_t now = (uint32_t)openknx.time.getUtcTime().toTime_t();
    if (now < _lastEpoch || now - _lastEpoch > MAX_GAP_S)
    {
        _count = 0; // Luecke zu gross: neu beginnen
        return;
    }
    // Verpasste 15-min-Werte zwischen letztem gespeichertem und aktuellem Wert linear auffuellen
    const uint32_t missed = (now - _lastEpoch) / (STEP / 1000);
    const float last = _ring[_head];
    for (uint32_t k = 1; k <= missed; k++)
    {
        _head = (_head + 1) % SIZE;
        _ring[_head] = last + (_current - last) * k / (missed + 1);
        if (_count < SIZE)
            _count++;
    }
    _lastEpoch += missed * (STEP / 1000);
    // Naechsten Wert im 15-min-Raster ab dem letzten (aufgefuellten) Wert einreihen
    const uint32_t ageMs = (now - _lastEpoch) * 1000UL;
    _timer = millis() - min(ageMs, STEP);
    logInfo("Weather", "Luftdruckverlauf uebernommen: %u Werte, %lu aufgefuellt", _count, missed);
}

float PressureTrend::sample(uint8_t stepsBack) const
{
    if (stepsBack >= _count)
        return NAN;
    return _ring[(_head + SIZE - stepsBack) % SIZE];
}

float PressureTrend::trend(uint8_t hours) const
{
    if (_restorePending || isnan(_current))
        return NAN;
    const float old = sample(hours * 4);
    return isnan(old) ? NAN : _current - old;
}
