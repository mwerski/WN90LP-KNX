#pragma once
#include <Arduino.h>
#include <time.h>

// Sonnen- und Mondberechnung (NOAA-Algorithmus fuer die Sonne, vereinfachte Meeus-Reihen fuer den Mond).
// Genauigkeit: Sonnenzeiten ca. 1 min, Mondposition ca. 0,5 Grad, Mondauf-/untergang einige Minuten.
namespace Astro
{
    struct Position
    {
        float azimuth;   // Grad, 0 = Nord, 90 = Ost
        float elevation; // Grad ueber dem Horizont (geozentrisch, ohne Refraktion)
    };

    // Sonnenzeiten eines Tages in Sekunden seit Mitternacht Ortszeit (NAN = findet nicht statt)
    struct SunDay
    {
        float rise, set, transit;
        float civilDawn, civilDusk, nauticalDawn, nauticalDusk, astroDawn, astroDusk;
        float riseAzimuth, setAzimuth, transitElevation;
        float dayLength; // s (NAN bei Polartag/-nacht)
    };

    struct MoonState
    {
        float illumination; // 0..1
        float age;          // Tage seit Neumond
        uint8_t phase;      // 0 Neumond .. 4 Vollmond .. 7 abnehmende Sichel
        Position position;
    };

    // utc: Unixzeit; lat/lon in Grad (Ost positiv)
    Position sunPosition(time_t utc, float lat, float lon);
    // localMidnightUtc: Unixzeit von 0:00 Ortszeit des Tages; tzOffset: Ortszeit - UTC in s
    SunDay sunDay(time_t localMidnightUtc, int32_t tzOffset, float lat, float lon);

    MoonState moon(time_t utc, float lat, float lon);
    // Mondauf-/untergang des Tages in Sekunden seit Mitternacht Ortszeit (NAN = keiner)
    void moonRiseSet(time_t localMidnightUtc, float lat, float lon, float &rise, float &set);
    // Naechster Zeitpunkt (Unixzeit), zu dem die Elongation Mond-Sonne targetDeg erreicht (0 = Neumond, 180 = Vollmond)
    time_t nextMoonPhase(time_t fromUtc, float targetDeg);

    const char *moonPhaseText(uint8_t phase, bool utf8);
} // namespace Astro
