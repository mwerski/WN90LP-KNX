#pragma once
#include <Arduino.h>

// Wetterwarnungen nach den Warnkriterien des DWD, angewandt auf die eigenen Messwerte (keine Vorhersage).
// Stufen: 0 keine, 1 Wetterwarnung, 2 markantes Wetter, 3 Unwetter, 4 extremes Unwetter.
class Warnings
{
  public:
    enum Category : uint8_t
    {
        Wind,
        HeavyRain,
        LongRain,
        Frost,
        Heat,
        Uv,
        Count
    };

    struct Input
    {
        float gustKmh;           // staerkste Boee der letzten 10 min
        float rain1h, rain6h;    // mm
        float rain24h, rain48h, rain72h;
        float temp;              // C (Temperatur fuer Berechnungen)
        float heatIndex;         // gefuehlte Temperatur bei Waerme
        float uv;
        bool raining;
        bool iceWarning;         // Glaettegefahr der Station
    };

    void setup(uint8_t holdMinutes) { _holdMs = (uint32_t)holdMinutes * 60000UL; }
    void update(const Input &in);

    uint8_t level(Category c) const { return _levels[c].level; }
    uint8_t overall() const;
    // Kategorie * 10 + Stufe der hoechsten Warnung (0 = keine), fuer den Text
    uint8_t code() const;
    static const char *text(uint8_t code, bool utf8);

  private:
    struct Held
    {
        uint8_t level = 0;
        uint8_t variant = 0; // Frost: 1 = Glaette, 2 = Glatteis
        uint32_t lastHit = 0;
    };
    void apply(Category c, uint8_t raw, uint8_t variant = 0);

    Held _levels[Count];
    uint32_t _holdMs = 30 * 60000UL;
};
