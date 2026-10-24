#include "MeasuredValue.h"
#include "Diag.h"
#include "Astro.h"
#include "Comfort.h"
#include "DwdWarnings.h"
#include "Forecast.h"
#include "ParticleStats.h"
#include "Warnings.h"

int64_t MeasuredValue::s_localMidnightUtc = 0;
int32_t MeasuredValue::s_tzOffset = 0;
bool MeasuredValue::s_dayContext = false;

void MeasuredValue::setDayContext(int64_t localMidnightUtc, int32_t tzOffset)
{
    s_localMidnightUtc = localMidnightUtc;
    s_tzOffset = tzOffset;
    s_dayContext = true;
}

bool MeasuredValue::toUnix(float value, const WxValueDef *def, int64_t &unix)
{
    if (isnan(value))
        return false;
    if (def->fmt == WxFmt::Time && s_dayContext)
    {
        unix = s_localMidnightUtc + lroundf(value);
        return true;
    }
    if (def->fmt == WxFmt::Date)
    {
        // 0:00 Uhr Ortszeit des Datums
        unix = ((int64_t)lroundf(value) + 10957) * 86400 - s_tzOffset;
        return true;
    }
    return false;
}

void MeasuredValue::init(const WxValueDef *def, bool configured)
{
    _def = def;
    if (!configured)
    {
        _active = false;
        return;
    }

    // Layout siehe scripts/generate_ets.py: +0 Aktiv/DPT, +1 Zyklus, +3 abs., +7 rel.
    const uint16_t base = def->paramOffset;
    const uint8_t flags = knx.paramByte(base);
    _active = flags & 0x01;
    _dptIndex = def->dptCount > 1 ? (flags >> 1) & 0x03 : 0;
    if (_dptIndex >= def->dptCount)
        _dptIndex = 0;
    _cycleMs = (uint32_t)knx.paramWord(base + 1) * 1000;
    // Ausgeblendete KOs fehlen in der Gruppenobjekt-Tabelle; ein Zugriff wuerde abstuerzen
    if (_active && !Diag::koExists(def->koNumber))
    {
        logError("Weather", "KO %u (%s) fehlt in der ETS-Tabelle, Wert deaktiviert", def->koNumber, def->name);
        _active = false;
    }
    if (def->thresholds)
    {
        _absChange = (float)knx.paramFloat(base + 3, Float_Enc_IEEE754Single);
        _relChange = (float)knx.paramFloat(base + 7, Float_Enc_IEEE754Single);
    }
}

void MeasuredValue::set(float value)
{
    if (isnan(value))
    {
        invalidate();
        return;
    }
    _value = value;
    _valid = true;
    if (!_active)
        return;

    // Leseanfragen sollen immer den aktuellen Wert liefern
    knx.getGroupObject(_def->koNumber).valueNoSend(busValue(), dpt());

    if (!_sent || changeExceedsThreshold())
        _sendPending = true;
}

void MeasuredValue::invalidate()
{
    _valid = false;
    _sendPending = false;
}

bool MeasuredValue::loop()
{
    if (!_active || !_valid)
        return false;

    if (_sendPending || (_cycleMs > 0 && delayCheck(_lastSendTime, _cycleMs)))
    {
        send();
        return true;
    }
    return false;
}

bool MeasuredValue::changeExceedsThreshold() const
{
    const float diff = fabsf(_value - _lastSent);
    if (!_def->thresholds)
        return diff > 0.0001f;
    if (_absChange > 0 && diff >= _absChange)
        return true;
    if (_relChange > 0)
    {
        // Von 0 aus ist jede Aenderung relativ unendlich gross (z. B. Helligkeit am Morgen)
        if (fabsf(_lastSent) < 0.0001f)
            return diff > 0.0001f;
        if (diff / fabsf(_lastSent) * 100.0f >= _relChange)
            return true;
    }
    return false;
}

void MeasuredValue::send()
{
    knx.getGroupObject(_def->koNumber).value(busValue(), dpt());
    _lastSent = _value;
    _sent = true;
    _sendPending = false;
    _lastSendTime = millis();
}

KNXValue MeasuredValue::busValue() const
{
    const WxDptOption &opt = _def->dpts[_dptIndex];
    const float bus = _value * opt.factor;
    switch (opt.main)
    {
        case 1:
            // Negativer Faktor = invertiert (z. B. DPT 1.100: 1 = Heizen = Winter)
            return KNXValue(opt.factor < 0 ? !(_value >= 0.5f) : bus >= 0.5f);
        case 6:
            return KNXValue((int8_t)constrain(lroundf(bus), -128L, 127L));
        case 16:
            // Text: der Messwert ist der Index (Zambretti-Nummer bzw. Mondphase)
            if (_def->fmt == WxFmt::MoonText)
                return KNXValue(Astro::moonPhaseText((uint8_t)lroundf(bus), false));
            if (_def->fmt == WxFmt::DwdText)
                return KNXValue(DwdWarnings::eventText(false));
            if (_def->fmt == WxFmt::WarnText)
                return KNXValue(Warnings::text((uint8_t)lroundf(bus), false));
            if (_def->fmt == WxFmt::AqiText)
                return KNXValue(ParticleStats::indexText((uint8_t)lroundf(bus), false));
            if (_def->fmt == WxFmt::ComfortText)
            {
                static std::string comfort; // KNXValue haelt nur den Zeiger
                comfort = Comfort::text((uint8_t)lroundf(bus));
                return KNXValue(comfort.c_str());
            }
            return KNXValue(Forecast::text((uint8_t)lroundf(bus), false));
        case 10:
        {
            // Sekunden seit Mitternacht -> Uhrzeit (ohne Wochentag)
            const long s = constrain(lroundf(_value), 0L, 86399L);
            struct tm t = {};
            t.tm_hour = s / 3600;
            t.tm_min = (s / 60) % 60;
            t.tm_sec = s % 60;
            return KNXValue(t);
        }
        case 12:
        {
            // Unix-Zeit (Sekunden seit 1970) als DPT 12.001
            int64_t unix = 0;
            toUnix(_value, _def, unix);
            return KNXValue((uint32_t)max<int64_t>(unix, 0));
        }
        case 19:
        {
            // Datum und Uhrzeit (Ortszeit); knx-Stack erwartet Jahr vierstellig, Monat 1..12
            int64_t unix = 0;
            toUnix(_value, _def, unix);
            const long days = (long)((unix + s_tzOffset) / 86400);
            const long secs = (long)((unix + s_tzOffset) % 86400);
            struct tm t = {};
            int y, m, d;
            civilFromDays(days, y, m, d);
            t.tm_year = y;
            t.tm_mon = m;
            t.tm_mday = d;
            t.tm_hour = secs / 3600;
            t.tm_min = (secs / 60) % 60;
            t.tm_sec = secs % 60;
            t.tm_isdst = -1;
            return KNXValue(t);
        }
        case 11:
        {
            // Tage seit 1.1.2000 -> Datum (knx-Stack erwartet Jahr vierstellig, Monat 1..12)
            struct tm t = {};
            int y, m, d;
            civilFromDays((long)lroundf(_value) + 10957, y, m, d);
            t.tm_year = y;
            t.tm_mon = m;
            t.tm_mday = d;
            return KNXValue(t);
        }
        case 5:
            if (opt.sub == 10)
                return KNXValue((uint8_t)constrain(lroundf(bus), 0L, 255L));
            return KNXValue((double)bus); // 5.001 / 5.003 skaliert der Stack selbst
        default:
            return KNXValue((double)bus);
    }
}

Dpt MeasuredValue::dpt() const
{
    const WxDptOption &opt = _def->dpts[_dptIndex];
    // DPT 10.001 schreibt im knx-Stack die Uhrzeit nur mit Index 1 (Index 0 = Wochentag)
    if (opt.main == 10)
        return Dpt(10, 1, 1);
    return Dpt(opt.main, opt.sub);
}

void MeasuredValue::civilFromDays(long z, int &y, int &m, int &d)
{
    // Tage seit 1.1.1970 -> Kalenderdatum (Algorithmus nach H. Hinnant)
    z += 719468;
    const long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y = (int)(yoe + era * 400) + (m <= 2);
}

long MeasuredValue::daysFromCivil(int y, int m, int d)
{
    y -= m <= 2;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (long)doe - 719468;
}

std::string MeasuredValue::format(float value, const WxValueDef *def, bool withUnit, bool full)
{
    char buf[32];
    if (isnan(value))
        return "";
    switch (def->fmt)
    {
        case WxFmt::Time:
        {
            const long s = lroundf(value);
            snprintf(buf, sizeof(buf), "%02ld:%02ld:%02ld", s / 3600, (s / 60) % 60, s % 60);
            return buf;
        }
        case WxFmt::Date:
        {
            int y, m, d;
            civilFromDays((long)lroundf(value) + 10957, y, m, d);
            snprintf(buf, sizeof(buf), "%04d-%02d-%02d", y, m, d);
            return buf;
        }
        case WxFmt::Duration:
        {
            const long s = lroundf(value);
            snprintf(buf, sizeof(buf), "%ld:%02ld h", s / 3600, (s / 60) % 60);
            return buf;
        }
        case WxFmt::Forecast:
            return full ? Forecast::longText((uint8_t)lroundf(value)) : Forecast::text((uint8_t)lroundf(value), true);
        case WxFmt::WarnText:
            return full ? Warnings::longText((uint8_t)lroundf(value)) : Warnings::text((uint8_t)lroundf(value), true);
        case WxFmt::DwdText:
            // Ereignistext des DWD ist ungekuerzt; nur das KO kuerzt auf 14 Zeichen
            return DwdWarnings::eventText(true);
        case WxFmt::AqiText:
            return ParticleStats::indexText((uint8_t)lroundf(value), true);
        case WxFmt::ComfortText:
            return full ? Comfort::longText((uint8_t)lroundf(value)) : Comfort::text((uint8_t)lroundf(value));
        case WxFmt::MoonText:
        case WxFmt::MoonPhase:
            return full ? Astro::moonPhaseLongText((uint8_t)lroundf(value)) : Astro::moonPhaseText((uint8_t)lroundf(value), true);
        default:
            snprintf(buf, sizeof(buf), withUnit && def->unit[0] ? "%.*f %s" : "%.*f", def->decimals, value, def->unit);
            return buf;
    }
}
