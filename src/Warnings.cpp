#include "Warnings.h"

// Haltezeit: eine Stufe sinkt erst, wenn sie fuer _holdMs nicht mehr erreicht wurde
void Warnings::apply(Category c, uint8_t raw, uint8_t variant)
{
    Held &h = _levels[c];
    if (raw >= h.level && raw > 0)
    {
        h.level = raw;
        h.variant = variant;
        h.lastHit = millis();
    }
    else if (raw < h.level && millis() - h.lastHit >= _holdMs)
    {
        h.level = raw;
        h.variant = variant;
        if (raw > 0)
            h.lastHit = millis();
    }
}

void Warnings::update(const Input &in)
{
    // Wind (Boeen): 50 / 65 / 105 / 140 km/h
    uint8_t wind = 0;
    if (!isnan(in.gustKmh))
        wind = in.gustKmh >= 140 ? 4 : in.gustKmh >= 105 ? 3 : in.gustKmh >= 65 ? 2 : in.gustKmh >= 50 ? 1 : 0;
    apply(Wind, wind);

    // Starkregen: 1 h 15/25/40 mm oder 6 h 20/35/60 mm
    auto over = [](float v, float limit) { return !isnan(v) && v >= limit; };
    uint8_t heavy = 0;
    if (over(in.rain1h, 40) || over(in.rain6h, 60))
        heavy = 4;
    else if (over(in.rain1h, 25) || over(in.rain6h, 35))
        heavy = 3;
    else if (over(in.rain1h, 15) || over(in.rain6h, 20))
        heavy = 2;
    apply(HeavyRain, heavy);

    // Dauerregen: 24 h 30/50/80, 48 h 40/60/90, 72 h 60/90/120 mm
    uint8_t lasting = 0;
    if (over(in.rain24h, 80) || over(in.rain48h, 90) || over(in.rain72h, 120))
        lasting = 4;
    else if (over(in.rain24h, 50) || over(in.rain48h, 60) || over(in.rain72h, 90))
        lasting = 3;
    else if (over(in.rain24h, 30) || over(in.rain48h, 40) || over(in.rain72h, 60))
        lasting = 2;
    apply(LongRain, lasting);

    // Frost/Glaette: Frost < 0, strenger Frost < -10, Glatteis = Regen bei hoechstens 0 C (Stufe 3)
    uint8_t frost = 0, variant = 0;
    if (!isnan(in.temp))
    {
        if (in.raining && in.temp <= 0.0f)
        {
            frost = 3;
            variant = 2;
        }
        else if (in.temp < -10.0f)
            frost = 2;
        else if (in.iceWarning)
        {
            frost = 1;
            variant = 1;
        }
        else if (in.temp < 0.0f)
            frost = 1;
    }
    apply(Frost, frost, variant);

    // Hitze (gefuehlte Temperatur): 32 / 38 C
    uint8_t heat = 0;
    if (!isnan(in.heatIndex))
        heat = in.heatIndex >= 38 ? 3 : in.heatIndex >= 32 ? 2 : 0;
    apply(Heat, heat);

    // UV-Index ab 8
    apply(Uv, (!isnan(in.uv) && in.uv >= 8) ? 1 : 0);
}

uint8_t Warnings::overall() const
{
    uint8_t m = 0;
    for (const Held &h : _levels)
        m = max(m, h.level);
    return m;
}

uint8_t Warnings::code() const
{
    // Hoechste Stufe; bei Gleichstand Reihenfolge Wind, Starkregen, Dauerregen, Frost, Hitze, UV
    uint8_t best = 0, cat = 0;
    for (uint8_t c = 0; c < Count; c++)
        if (_levels[c].level > best)
        {
            best = _levels[c].level;
            cat = c;
        }
    if (best == 0)
        return 0;
    // Frost-Varianten als eigene Codes: 31 Glaette, 33 Glatteis
    if (cat == Frost && _levels[Frost].variant == 1 && best == 1)
        return 39;
    return cat * 10 + best + 10;
}

const char *Warnings::longText(uint8_t code)
{
    switch (code)
    {
        case 0:
            return "Keine Warnung";
        case 11:
            return "Windböen";
        case 12:
            return "Sturmböen";
        case 13:
            return "Orkanartige Böen";
        case 14:
            return "Orkanböen";
        case 22:
            return "Starkregen";
        case 23:
            return "Heftiger Starkregen";
        case 24:
            return "Extrem heftiger Starkregen";
        case 32:
            return "Dauerregen";
        case 33:
            return "Ergiebiger Dauerregen";
        case 34:
            return "Extrem ergiebiger Dauerregen";
        case 41:
            return "Frost";
        case 39:
            return "Glätte";
        case 42:
            return "Strenger Frost";
        case 43:
            return "Glatteis";
        case 52:
            return "Hitze";
        case 53:
            return "Extreme Hitze";
        case 61:
            return "Hohe UV-Intensität";
        default:
            return "";
    }
}

const char *Warnings::text(uint8_t code, bool utf8)
{
    // max. 14 Zeichen (DPT 16); Umlaute je nach Kodierung
    switch (code)
    {
        case 0:
            return "Keine Warnung";
        case 11:
            return utf8 ? "Windb\u00f6en" : "Windb\xF6" "en";
        case 12:
            return utf8 ? "Sturmb\u00f6en" : "Sturmb\xF6" "en";
        case 13:
            return utf8 ? "Orkanart. B\u00f6en" : "Orkanart. B\xF6" "en";
        case 14:
            return utf8 ? "Orkanb\u00f6en" : "Orkanb\xF6" "en";
        case 22:
            return "Starkregen";
        case 23:
            return "Heft. Starkr.";
        case 24:
            return "Extr. Starkr.";
        case 32:
            return "Dauerregen";
        case 33:
            return "Ergieb. Dauerr";
        case 34:
            return "Extr. Dauerr.";
        case 41:
            return "Frost";
        case 39:
            return utf8 ? "Gl\u00e4tte" : "Gl\xE4" "tte";
        case 42:
            return "Strenger Frost";
        case 43:
            return "Glatteis";
        case 52:
            return "Hitze";
        case 53:
            return "Extreme Hitze";
        case 61:
            return "Hohe UV-Int.";
        default:
            return "";
    }
}
