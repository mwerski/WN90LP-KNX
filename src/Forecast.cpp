#include "Forecast.h"

namespace Forecast
{
    // Schwellen der Barometer-Regel in hPa ueber 3 h
    static constexpr float TREND_SLOW = 1.6f;
    static constexpr float TREND_FAST = 3.5f;

    int8_t tendency(float trend3h)
    {
        if (isnan(trend3h))
            return 0;
        if (trend3h <= -TREND_FAST)
            return -2;
        if (trend3h <= -TREND_SLOW)
            return -1;
        if (trend3h >= TREND_FAST)
            return 2;
        if (trend3h >= TREND_SLOW)
            return 1;
        return 0;
    }

    // Kurztexte A..Z, UTF-8 (\u-Escapes, damit der Quelltext ohne Umlaute bleibt), max. 14 Zeichen
    static const char *const TEXTS[26] = {
        "Stabil sch\u00f6n",  // A Settled fine
        "Sch\u00f6n",         // B Fine weather
        "Wird sch\u00f6n",    // C Becoming fine
        "Sch\u00f6n, labiler", // D Fine, becoming less settled
        "Sch\u00f6n,Schauer?", // E Fine, possible showers
        "Heiter, besser",      // F Fairly fine, improving
        "Fr\u00fch Schauer?",  // G Fairly fine, possible showers early
        "Heiter,sp.Sch.",      // H Fairly fine, showery later
        "Schauer,besser",      // I Showery early, improving
        "Wechsel,besser",      // J Changeable, mending
        "Heiter,Schauer",      // K Fairly fine, showers likely
        "Unbest.,aufkl.",      // L Rather unsettled clearing later
        "Unbest.,besser",      // M Unsettled, probably improving
        "Schauer,heiter",      // N Showery, bright intervals
        "Schauer,labil",       // O Showery, becoming less settled
        "Wechsel,Regen",       // P Changeable, some rain
        "Unbest.,L\u00fccken", // Q Unsettled, short fine intervals
        "Sp\u00e4ter Regen",   // R Unsettled, rain later
        "Unbest.,Regen",       // S Unsettled, some rain
        "Sehr unstet",         // T Mostly very unsettled
        "Regen,zunehm.",       // U Occasional rain, worsening
        "Regen,unstet",        // V Rain at times, very unsettled
        "Oft Regen",           // W Rain at frequent intervals
        "Regen,s.unstet",      // X Rain, very unsettled
        "Sturm,besser",        // Y Stormy, may improve
        "Sturm,v.Regen",       // Z Stormy, much rain
    };

    // Zuordnung Druckstufe (0 = 950 hPa .. 21 = 1050 hPa) -> Prognose-Index, nach Zambretti/beteljuice
    static const uint8_t OPT_RISING[22] = {25, 25, 25, 24, 24, 19, 16, 12, 11, 9, 8, 6, 5, 2, 1, 1, 0, 0, 0, 0, 0, 0};
    static const uint8_t OPT_STEADY[22] = {25, 25, 25, 25, 25, 25, 23, 23, 22, 18, 15, 13, 10, 4, 1, 1, 0, 0, 0, 0, 0, 0};
    static const uint8_t OPT_FALLING[22] = {25, 25, 25, 25, 25, 25, 25, 25, 23, 23, 21, 20, 17, 14, 7, 3, 1, 1, 1, 0, 0, 0};

    // Windkorrektur in hPa (Prozent des Bereichs 950..1050) fuer 16 Richtungen ab Nord, Nordhalbkugel
    static const float WIND[16] = {6, 5, 5, 2, -0.5f, -2, -5, -8.5f, -12, -10, -6, -4.5f, -3, -0.5f, 1.5f, 3};

    uint8_t zambretti(float pressureSea, float trend3h, float windDir, uint8_t month)
    {
        if (isnan(pressureSea) || isnan(trend3h))
            return 0;

        const int8_t trend = tendency(trend3h);
        float p = pressureSea;
        if (!isnan(windDir))
            p += WIND[(uint8_t)lroundf(fmodf(windDir + 360.0f, 360.0f) / 22.5f) % 16];
        // Sommerhalbjahr (April bis September): steigender/fallender Druck wirkt staerker
        if (month >= 4 && month <= 9)
        {
            if (trend > 0)
                p += 7.0f;
            else if (trend < 0)
                p -= 7.0f;
        }

        int option = (int)floorf((p - 950.0f) / (100.0f / 22.0f));
        option = constrain(option, 0, 21);
        const uint8_t index = trend > 0 ? OPT_RISING[option] : (trend < 0 ? OPT_FALLING[option] : OPT_STEADY[option]);
        return index + 1;
    }

    // Volltexte A..Z, UTF-8
    static const char *const LONG_TEXTS[26] = {
        "Beständig schönes Wetter",            // A Settled fine
        "Schönes Wetter",                           // B Fine weather
        "Wetter wird schön",                        // C Becoming fine
        "Schön, wird unbeständiger",           // D Fine, becoming less settled
        "Schön, eventuell Schauer",                 // E Fine, possible showers
        "Recht schön, Besserung",                   // F Fairly fine, improving
        "Recht schön, anfangs eventuell Schauer",   // G Fairly fine, possible showers early
        "Recht schön, später Schauer",         // H Fairly fine, showery later
        "Anfangs Schauer, dann Besserung",               // I Showery early, improving
        "Wechselhaft, Besserung",                        // J Changeable, mending
        "Recht schön, Schauer wahrscheinlich",      // K Fairly fine, showers likely
        "Ziemlich unbeständig, später aufklarend", // L Rather unsettled clearing later
        "Unbeständig, wahrscheinlich Besserung",    // M Unsettled, probably improving
        "Schauer mit Aufheiterungen",                    // N Showery, bright intervals
        "Schauer, wird unbeständiger",              // O Showery, becoming less settled
        "Wechselhaft, etwas Regen",                      // P Changeable, some rain
        "Unbeständig, kurze schöne Abschnitte", // Q Unsettled, short fine intervals
        "Unbeständig, später Regen",           // R Unsettled, rain later
        "Unbeständig, etwas Regen",                 // S Unsettled, some rain
        "Überwiegend sehr unbeständig",        // T Mostly very unsettled
        "Gelegentlich Regen, Verschlechterung",          // U Occasional rain, worsening
        "Zeitweise Regen, sehr unbeständig",        // V Rain at times, very unsettled
        "Häufig Regen",                             // W Rain at frequent intervals
        "Regen, sehr unbeständig",                  // X Rain, very unsettled
        "Stürmisch, eventuell Besserung",           // Y Stormy, may improve
        "Stürmisch, viel Regen",                    // Z Stormy, much rain
    };

    const char *longText(uint8_t zambretti)
    {
        return (zambretti >= 1 && zambretti <= 26) ? LONG_TEXTS[zambretti - 1] : "";
    }

    const char *text(uint8_t zambretti, bool utf8)
    {
        if (zambretti < 1 || zambretti > 26)
            return "";
        const char *src = TEXTS[zambretti - 1];
        if (utf8)
            return src;

        // UTF-8 -> ISO 8859-1 (nur 2-Byte-Folgen C2/C3 noetig), auf 14 Zeichen begrenzt
        static char iso[15];
        uint8_t n = 0;
        for (const uint8_t *s = (const uint8_t *)src; *s && n < 14; s++)
        {
            if ((*s == 0xC2 || *s == 0xC3) && s[1])
            {
                iso[n++] = (char)(((*s & 0x03) << 6) | (s[1] & 0x3F));
                s++;
            }
            else
                iso[n++] = (char)*s;
        }
        iso[n] = 0;
        return iso;
    }
} // namespace Forecast
