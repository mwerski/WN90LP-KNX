#pragma once
// Abgeleitete Wetterwerte; Formeln ueberwiegend aus RobTillaart/Temperature
#include <math.h>
#include <stdint.h>
#include <temperature.h>

namespace WeatherCalc
{
    // Frostpunkt (Magnus-Formel ueber Eis), Temperatur in C, relative Feuchte in %
    inline float frostPoint(float t, float rh)
    {
        const float a = t >= 0 ? 7.5f : 9.5f;
        const float b = t >= 0 ? 237.3f : 265.5f;
        const float sdd = 6.1078f * powf(10.0f, (a * t) / (b + t)); // Saettigungsdampfdruck in hPa
        const float dd = sdd * (rh / 100.0f);                       // Dampfdruck in hPa
        const float v = log10f(dd / 6.1078f);
        return (b * v) / (a - v);
    }

    // Windchill nach JAG/TI; ausserhalb des Gueltigkeitsbereichs (> 10 C oder < 5 km/h) die Lufttemperatur
    inline float windChill(float t, float windMs)
    {
        const float kmh = windMs * 3.6f;
        if (t > 10.0f || kmh < 5.0f)
            return t;
        // Eigene Formel statt WindChill_C_kmph(): Dort potenziert erst convert=true die Geschwindigkeit
        // (v^0,16); mit false ging v linear ein und lieferte z. B. bei 10 C und 10 km/h -55 C.
        const float v016 = powf(kmh, 0.16f);
        return min(t, 13.12f + 0.6215f * t + (0.3965f * t - 11.37f) * v016);
    }

    // Hitzeindex ist nur fuer hohe Temperaturen definiert, darunter die Lufttemperatur
    inline float heatIndex(float t, float rh)
    {
        if (t < 27.0f)
            return t;
        return heatIndexC(t, rh);
    }

    // Gefuehlte Temperatur wie in Wetter-Apps: Windchill bei Kaelte, Hitzeindex bei Waerme, sonst Lufttemperatur.
    // Fehlt Wind bzw. Feuchte, gilt der jeweilige Index nicht.
    inline float feelsLike(float t, float rh, float windMs)
    {
        if (t <= 10.0f)
            return isnan(windMs) ? t : windChill(t, windMs);
        return isnan(rh) ? t : heatIndex(t, rh);
    }

    // Feuchtekorrektur fuer SDS011-Messwerte (Faktoren nach Zbyszek Kilianski / luftdaten.info)
    inline float normalizePm25(float pm, float rh)
    {
        return pm / (1.0f + 0.48756f * powf(rh / 100.0f, 8.60068f));
    }

    inline float normalizePm10(float pm, float rh)
    {
        return pm / (1.0f + 0.81559f * powf(rh / 100.0f, 5.83411f));
    }

    // Beaufort-Skala aus m/s
    inline uint8_t beaufort(float s)
    {
        static const float limits[] = {0.3f, 1.6f, 3.4f, 5.5f, 8.0f, 10.8f, 13.9f, 17.2f, 20.8f, 24.5f, 28.5f, 32.7f};
        uint8_t bft = 0;
        while (bft < 12 && s >= limits[bft])
            bft++;
        return bft;
    }
} // namespace WeatherCalc
