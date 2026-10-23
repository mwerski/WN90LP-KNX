#include "ParticleStats.h"
#include "MeasuredValue.h"
#include "OpenKNX.h"

static constexpr uint32_t TEN_MIN_MS = 600000UL;
static constexpr uint32_t HOUR_MS = 3600000UL;

void ParticleStats::setLimits(float pm25, float pm10)
{
    _limit[Pm25] = pm25;
    _limit[Pm10] = pm10;
}

void ParticleStats::feed(float pm25, float pm10)
{
    const float v[Count] = {pm25, pm10};
    for (uint8_t i = 0; i < Count; i++)
    {
        if (isnan(v[i]) || v[i] < 0)
            continue;
        Channel &c = _ch[i];
        c.tenSum[_tenHead] += v[i];
        c.tenCount[_tenHead]++;
        c.hourSum += v[i];
        c.hourCount++;
        if (_dayKey)
        {
            c.daySum += v[i];
            c.dayCount++;
        }
    }
}

void ParticleStats::loop()
{
    if (delayCheck(_tenTimer, TEN_MIN_MS))
    {
        _tenTimer += TEN_MIN_MS;
        _tenHead = (_tenHead + 1) % TEN_MINUTES;
        for (Channel &c : _ch)
        {
            c.tenSum[_tenHead] = 0;
            c.tenCount[_tenHead] = 0;
        }
    }
    if (delayCheck(_hourTimer, HOUR_MS))
    {
        _hourTimer += HOUR_MS;
        pushHour();
        _saveRequested = true;
    }
    if (!openknx.time.isValid())
        return;
    // Ausfallzeit seit dem letzten gespeicherten Stundenwert als Stunden ohne Messung nachtragen
    if (_restoreShiftPending)
    {
        _restoreShiftPending = false;
        const uint32_t now = (uint32_t)openknx.time.getUtcTime().toTime_t();
        if (_lastHourEpoch > 0 && now > _lastHourEpoch)
        {
            const uint32_t missed = min<uint32_t>((now - _lastHourEpoch) / 3600, HOURS);
            for (uint32_t i = 0; i < missed; i++)
                pushHour();
        }
    }
    checkDay();
}

void ParticleStats::pushHour()
{
    _hourHead = (_hourHead + 1) % HOURS;
    for (Channel &c : _ch)
    {
        c.hours[_hourHead] = c.hourCount ? (float)(c.hourSum / c.hourCount) : NAN;
        c.hourSum = 0;
        c.hourCount = 0;
    }
    if (_hourFill < HOURS)
        _hourFill++;
    _lastHourEpoch = openknx.time.isValid() ? (uint32_t)openknx.time.getUtcTime().toTime_t() : 0;
}

void ParticleStats::checkDay()
{
    const OpenKNX::DateTime now = openknx.time.getLocalTime();
    const uint32_t key = now.year * 10000UL + now.month * 100UL + now.day;
    if (key == _dayKey)
        return;

    // Abgeschlossener Tag zaehlt nur, wenn er direkt der Vortag war (sonst fehlt Messzeit). Sein Tagesmittel
    // geht noch in das Jahr ein, zu dem er gehoert; am 1. Januar werden die Jahreswerte danach zurueckgesetzt.
    const long today = MeasuredValue::daysFromCivil(now.year, now.month, now.day);
    const long previous = _dayKey ? MeasuredValue::daysFromCivil(_dayKey / 10000, (_dayKey / 100) % 100, _dayKey % 100) : 0;
    const bool complete = _dayKey && previous == today - 1;
    for (uint8_t i = 0; i < Count; i++)
    {
        Channel &c = _ch[i];
        c.yesterday = (complete && c.dayCount) ? (float)(c.daySum / c.dayCount) : NAN;
        if (isnan(c.yesterday))
            continue;
        c.yearSum += c.yesterday;
        c.yearDays++;
        if (c.yesterday > _limit[i])
            c.exceedDays++;
    }
    if (_yearKey != now.year)
    {
        _yearKey = now.year;
        for (Channel &c : _ch)
        {
            c.yearSum = 0;
            c.yearDays = 0;
            c.exceedDays = 0;
        }
    }
    for (Channel &c : _ch)
    {
        c.daySum = 0;
        c.dayCount = 0;
    }
    _dayKey = key;
    _saveRequested = true;
}

float ParticleStats::mean1h(Pm pm) const
{
    double sum = 0;
    uint32_t count = 0;
    for (uint8_t i = 0; i < TEN_MINUTES; i++)
    {
        sum += _ch[pm].tenSum[i];
        count += _ch[pm].tenCount[i];
    }
    return count ? (float)(sum / count) : NAN;
}

float ParticleStats::mean24h(Pm pm) const
{
    double sum = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < _hourFill; i++)
    {
        const float h = _ch[pm].hours[(_hourHead + HOURS - i) % HOURS];
        if (!isnan(h))
        {
            sum += h;
            n++;
        }
    }
    return n >= MIN_HOURS_24H ? (float)(sum / n) : NAN;
}

float ParticleStats::yearMean(Pm pm) const
{
    return _ch[pm].yearDays ? (float)(_ch[pm].yearSum / _ch[pm].yearDays) : NAN;
}

float ParticleStats::index(Pm pm) const
{
    // Luftqualitaetsindex des Umweltbundesamts (Obergrenzen in ug/m3, 24-h-Mittel)
    static const float LIMITS[Count][4] = {{10, 20, 25, 50}, {20, 35, 50, 100}};
    const float m = mean24h(pm);
    if (isnan(m))
        return NAN;
    uint8_t idx = 1;
    while (idx < 5 && m > LIMITS[pm][idx - 1])
        idx++;
    return idx;
}

const char *ParticleStats::indexText(uint8_t index, bool utf8)
{
    switch (index)
    {
        case 1:
            return "sehr gut";
        case 2:
            return "gut";
        case 3:
            return utf8 ? "mäßig" : "m\xE4\xDF" "ig";
        case 4:
            return "schlecht";
        case 5:
            return "sehr schlecht";
        default:
            return "";
    }
}

bool ParticleStats::saveRequested()
{
    const bool r = _saveRequested;
    _saveRequested = false;
    return r;
}

void ParticleStats::writeFlash()
{
    openknx.flash.writeByte(_hourHead);
    openknx.flash.writeByte(_hourFill);
    openknx.flash.writeInt(_lastHourEpoch);
    openknx.flash.writeInt(_dayKey);
    openknx.flash.writeWord(_yearKey);
    for (const Channel &c : _ch)
    {
        for (float h : c.hours)
            openknx.flash.writeFloat(h);
        openknx.flash.writeDouble(c.daySum);
        openknx.flash.writeWord(c.dayCount);
        openknx.flash.writeFloat(c.yesterday);
        openknx.flash.writeDouble(c.yearSum);
        openknx.flash.writeWord(c.yearDays);
        openknx.flash.writeWord(c.exceedDays);
    }
}

void ParticleStats::readFlash()
{
    _hourHead = openknx.flash.readByte() % HOURS;
    _hourFill = min<uint8_t>(openknx.flash.readByte(), HOURS);
    _lastHourEpoch = openknx.flash.readInt();
    _dayKey = openknx.flash.readInt();
    _yearKey = openknx.flash.readWord();
    for (Channel &c : _ch)
    {
        for (float &h : c.hours)
            h = openknx.flash.readFloat();
        c.daySum = openknx.flash.readDouble();
        c.dayCount = openknx.flash.readWord();
        c.yesterday = openknx.flash.readFloat();
        c.yearSum = openknx.flash.readDouble();
        c.yearDays = openknx.flash.readWord();
        c.exceedDays = openknx.flash.readWord();
        // Ungueltige Daten (z. B. leerer Flash): neu beginnen
        if (isnan(c.daySum) || isnan(c.yearSum) || c.yearDays > 366)
        {
            c = Channel();
            c.yesterday = NAN;
            _hourFill = 0;
        }
    }
    if (_lastHourEpoch == 0xFFFFFFFF)
        _lastHourEpoch = 0;
    if (_dayKey == 0xFFFFFFFF || _yearKey == 0xFFFF)
    {
        _dayKey = 0;
        _yearKey = 0;
    }
    _restoreShiftPending = _hourFill > 0;
}
