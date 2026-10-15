#include "PressureTrend.h"
#include "OpenKNX.h"

void PressureTrend::feed(float pressure)
{
    const bool first = isnan(_current);
    _current = pressure;
    if (first)
    {
        _ring[0] = pressure;
        _head = 0;
        _count = 1;
        _timer = millis();
    }
}

void PressureTrend::loop()
{
    if (isnan(_current) || !delayCheck(_timer, STEP))
        return;
    _timer += STEP;
    _head = (_head + 1) % SIZE;
    _ring[_head] = _current;
    if (_count < SIZE)
        _count++;
}

float PressureTrend::sample(uint8_t stepsBack) const
{
    if (stepsBack >= _count)
        return NAN;
    return _ring[(_head + SIZE - stepsBack) % SIZE];
}

float PressureTrend::trend(uint8_t hours) const
{
    const float old = sample(hours * 4);
    return isnan(old) ? NAN : _current - old;
}
