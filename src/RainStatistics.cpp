#include "RainStatistics.h"
#include "OpenKNX.h"

// Sprung von mehr als 50 mm zwischen zwei Abfragen (2 s): Zaehler der Wetterstation wurde zurueckgesetzt
static constexpr uint16_t MAX_PLAUSIBLE_DELTA = 5000;

void RainStatistics::setup(uint8_t windowMin, float minAmount, uint8_t stopDelayMin)
{
    _windowMin = constrain(windowMin, 1, 60);
    _minAmount = max(minAmount, 0.01f);
    _stopDelayMs = (uint32_t)max<uint8_t>(stopDelayMin, 1) * 60000UL;
    _minuteTimer = millis();
    _hourTimer = millis();
}

void RainStatistics::feed(uint16_t rawCounter)
{
    if (!_initialized)
    {
        _lastRaw = rawCounter;
        _initialized = true;
        // Startwerte, damit die Zeitfenster sofort (anteilig) Werte liefern
        _minutes[0] = _total;
        _minuteHead = 0;
        _minuteCount = 1;
        if (_hourCount == 0)
        {
            _hours[0] = _total;
            _hourHead = 0;
            _hourCount = 1;
        }
        return;
    }

    const uint16_t delta = (uint16_t)(rawCounter - _lastRaw); // Ueberlauf-sicher
    _lastRaw = rawCounter;
    if (delta == 0)
        return;
    if (delta > MAX_PLAUSIBLE_DELTA)
    {
        logInfo("Weather", "Regenzaehler der Wetterstation zurueckgesetzt (%u)", rawCounter);
        return;
    }
    _total += delta / 100.0;
    _lastRain = millis();
    _rainSeen = true;
}

void RainStatistics::loop()
{
    if (!_initialized)
        return;

    if (delayCheck(_minuteTimer, 60000UL))
    {
        _minuteTimer += 60000UL;
        _minuteHead = (_minuteHead + 1) % MINUTES;
        _minutes[_minuteHead] = _total;
        if (_minuteCount < MINUTES)
            _minuteCount++;
    }
    updateRaining();

    if (delayCheck(_hourTimer, 3600000UL))
    {
        _hourTimer += 3600000UL;
        pushHour();
        _saveRequested = true;
    }

    if (openknx.time.isValid())
    {
        // Ausfallzeit seit dem letzten gespeicherten Stundenwert nachtragen (es regnete nicht "gemessen")
        if (_restoreShiftPending)
        {
            _restoreShiftPending = false;
            const time_t now = openknx.time.getUtcTime().toTime_t();
            if (_lastHourEpoch > 0 && now > (time_t)_lastHourEpoch)
            {
                const uint32_t hours = (now - _lastHourEpoch) / 3600;
                if (hours >= HOURS)
                {
                    _hourCount = 0;
                    pushHour();
                }
                else
                    for (uint32_t i = 0; i < hours; i++)
                        pushHour();
                logInfo("Weather", "Regenstatistik: %u h Ausfallzeit nachgetragen", hours);
            }
        }
        checkDay();
    }
}

void RainStatistics::pushHour()
{
    _hourHead = (_hourHead + 1) % HOURS;
    _hours[_hourHead] = _total;
    if (_hourCount < HOURS)
        _hourCount++;
    _lastHourEpoch = openknx.time.isValid() ? (uint32_t)openknx.time.getUtcTime().toTime_t() : 0;
}

void RainStatistics::checkDay()
{
    const OpenKNX::DateTime now = openknx.time.getLocalTime();
    const uint32_t key = now.year * 10000UL + now.month * 100UL + now.day;
    if (key == _dayKey)
        return;
    // Neuer Tag (oder erste gueltige Uhrzeit): Tagesmenge beginnt beim aktuellen Stand
    _dayKey = key;
    _midnightTotal = _total;
    _saveRequested = true;
}

float RainStatistics::back(const float *ring, uint8_t size, uint8_t head, uint8_t count, uint8_t steps) const
{
    if (count == 0)
        return NAN;
    // Liegen noch nicht genug Werte vor, gilt der aelteste (anteiliges Fenster)
    if (steps >= count)
        steps = count - 1;
    return ring[(head + size - steps) % size];
}

float RainStatistics::amountMinutes(uint8_t minutes) const
{
    const float start = back(_minutes, MINUTES, _minuteHead, _minuteCount, minutes);
    return isnan(start) ? NAN : (float)(_total - start);
}

float RainStatistics::amountHours(uint8_t hours) const
{
    const float start = back(_hours, HOURS, _hourHead, _hourCount, hours);
    return isnan(start) ? NAN : (float)(_total - start);
}

float RainStatistics::today() const
{
    if (_dayKey == 0)
        return NAN;
    return (float)(_total - _midnightTotal);
}

void RainStatistics::updateRaining()
{
    if (!_raining)
    {
        const float amount = amountMinutes(_windowMin);
        _raining = !isnan(amount) && amount >= _minAmount - 0.0001f;
    }
    else if (!_rainSeen || millis() - _lastRain >= _stopDelayMs)
        _raining = false;
}

float RainStatistics::rate() const
{
    const float amount = amountMinutes(10);
    if (isnan(amount))
        return NAN;
    // Solange noch keine 10 Minuten vorliegen, auf die vorhandene Zeit hochrechnen
    const uint8_t minutes = min<uint8_t>(10, max<uint8_t>(_minuteCount - 1, 1));
    return amount * 60.0f / minutes;
}

bool RainStatistics::saveRequested()
{
    const bool r = _saveRequested;
    _saveRequested = false;
    return r;
}

void RainStatistics::writeFlash()
{
    openknx.flash.writeByte(FLASH_VERSION);
    openknx.flash.writeDouble(_total);
    openknx.flash.writeDouble(_midnightTotal);
    openknx.flash.writeInt(_dayKey);
    openknx.flash.writeInt(_lastHourEpoch);
    openknx.flash.writeByte(_hourHead);
    openknx.flash.writeByte(_hourCount);
    for (uint8_t i = 0; i < HOURS; i++)
        openknx.flash.writeFloat(_hours[i]);
}

void RainStatistics::readFlash(const uint8_t *data, uint16_t size)
{
    if (size < FLASH_SIZE || openknx.flash.readByte() != FLASH_VERSION)
        return;
    _total = openknx.flash.readDouble();
    _midnightTotal = openknx.flash.readDouble();
    _dayKey = openknx.flash.readInt();
    _lastHourEpoch = openknx.flash.readInt();
    _hourHead = openknx.flash.readByte() % HOURS;
    _hourCount = min<uint8_t>(openknx.flash.readByte(), HOURS);
    for (uint8_t i = 0; i < HOURS; i++)
        _hours[i] = openknx.flash.readFloat();
    if (isnan(_total) || _total < 0)
    {
        _total = 0;
        _midnightTotal = 0;
        _hourCount = 0;
    }
    _restoreShiftPending = true;
    logInfo("Weather", "Regenstatistik geladen: gesamt %.2f mm, %u Stundenwerte", _total, _hourCount);
}
