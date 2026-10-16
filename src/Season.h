#pragma once
#include <Arduino.h>

// Sommer/Winter-Umschaltung ueber das gleitende Mittel der Tagesmitteltemperaturen.
// Tagesmittel wahlweise nach DIN/Mannheimer Stunden (T7 + T14 + 2*T21|22) / 4
// oder als Mittel aller Messwerte des Tages. Bewertet wird einmal taeglich beim Tageswechsel.
class Season
{
  public:
    static constexpr uint16_t FLASH_SIZE = 4 + 4 + 2 + 3 * 4 + 7 * 4 + 1 + 1 + 1 + 4;

    // method: 0 = DIN (Mannheimer Stunden), 1 = Mittel aller Messwerte
    void setup(uint8_t method, uint8_t eveningHour, uint8_t days, float threshold, float hysteresis);
    void feed(float temp);
    // Aufruf regelmaessig mit gueltiger Uhrzeit; true bei Tageswechsel
    bool loop(uint32_t dayKey, uint8_t hour, float temp);

    // -1 unbekannt, 0 Winter, 1 Sommer
    int8_t summer() const { return _summer; }
    float average() const;      // gleitendes Mittel der letzten Tage
    float yesterday() const;    // Tagesmittel des Vortags
    uint8_t days() const { return _count; }
    float reading(uint8_t i) const { return _readings[i]; }
    float todayMean() const;
    // Zuletzt abgeschlossener Tag (JJJJMMTT) und sein Tagesmittel (NAN = keine Werte)
    uint32_t finishedDayKey() const { return _finishedKey; }
    float finishedMean() const { return _finishedMean; }

    void writeFlash();
    void readFlash();

  private:
    static constexpr uint8_t MAX_DAYS = 7;
    void finishDay();

    uint8_t _method = 0;
    uint8_t _hours[3] = {7, 14, 22};
    uint8_t _days = 3;
    float _threshold = 16;
    float _hysteresis = 0;

    uint32_t _dayKey = 0;
    float _sum = 0;
    uint16_t _samples = 0;
    float _readings[3] = {NAN, NAN, NAN};
    float _means[MAX_DAYS] = {};
    uint8_t _count = 0;
    uint8_t _head = 0;
    int8_t _summer = -1;
    float _lastMean = NAN;
    uint32_t _finishedKey = 0;
    float _finishedMean = NAN;
};
