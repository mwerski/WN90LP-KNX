#pragma once
#include <Arduino.h>

// Wetterprognose nach Zambretti (Barometer-Prognose fuer ca. 12 h, Nordhalbkugel)
namespace Forecast
{
    // Luftdrucktendenz aus der Aenderung ueber 3 h: -2 stark fallend .. +2 stark steigend
    int8_t tendency(float trend3h);

    // Zambretti-Prognose 1..26 (A..Z); 0 = keine Prognose moeglich
    // pressureSea in hPa, windDir in Grad (NAN = ohne Windkorrektur), month 1..12 (0 = unbekannt)
    uint8_t zambretti(float pressureSea, float trend3h, float windDir, uint8_t month);

    // Kurztext (max. 14 Zeichen); utf8 = true fuer MQTT/Konsole, sonst ISO 8859-1 fuer DPT 16
    const char *text(uint8_t zambretti, bool utf8);
} // namespace Forecast
