#pragma once
#include <Arduino.h>

// Feinstaub-Auswertung (PM2,5 und PM10): gleitende Mittel ueber 1 h und 24 h, Tagesmittel des Vortags,
// Jahresmittel aus Tagesmitteln, Ueberschreitungstage im laufenden Jahr und Luftqualitaetsindex des
// Umweltbundesamts. Zeitraster wie bei der Regenstatistik: 10-min- und Stunden-Ring ueber millis(),
// Tages- und Jahreswechsel nach Ortszeit.
class ParticleStats
{
  public:
    enum Pm : uint8_t
    {
        Pm25,
        Pm10,
        Count
    };

    static constexpr uint8_t HOURS = 24;
    static constexpr uint16_t CHANNEL_FLASH_SIZE = HOURS * 4 + 8 + 2 + 4 + 8 + 2 + 2;
    static constexpr uint16_t FLASH_SIZE = 1 + 1 + 4 + 4 + 2 + Count * CHANNEL_FLASH_SIZE;

    // Grenzwert fuer Ueberschreitungstage (Tagesmittel in ug/m3)
    void setLimits(float pm25, float pm10);
    void feed(float pm25, float pm10);
    void loop();

    float mean1h(Pm pm) const;
    float mean24h(Pm pm) const;   // NAN, solange weniger als 18 Stundenmittel vorliegen
    float yesterday(Pm pm) const { return _ch[pm].yesterday; }
    float yearMean(Pm pm) const;  // aus den vollstaendigen Tagen seit 1. Januar
    float exceedDays(Pm pm) const { return _yearKey ? _ch[pm].exceedDays : NAN; }

    // Index des Umweltbundesamts 1 (sehr gut) .. 5 (sehr schlecht) aus dem 24-h-Mittel, NAN ohne Daten
    float index(Pm pm) const;
    static const char *indexText(uint8_t index, bool utf8);

    bool saveRequested();
    void writeFlash();
    void readFlash();

  private:
    static constexpr uint8_t TEN_MINUTES = 6;
    static constexpr uint8_t MIN_HOURS_24H = 18; // 75 % Datenabdeckung

    struct Channel
    {
        // 1 h: sechs 10-min-Abschnitte (Summe/Anzahl), aktueller Abschnitt an _tenHead
        float tenSum[TEN_MINUTES] = {};
        uint16_t tenCount[TEN_MINUTES] = {};
        // 24 h: Stundenmittel (NAN = keine Messung in dieser Stunde)
        float hours[HOURS] = {};
        double hourSum = 0;
        uint16_t hourCount = 0;
        // Tag (seit Mitternacht) und Jahr
        double daySum = 0;
        uint16_t dayCount = 0;
        float yesterday = NAN;
        double yearSum = 0; // Summe der Tagesmittel
        uint16_t yearDays = 0;
        uint16_t exceedDays = 0;
    };

    void pushHour();
    void checkDay();

    Channel _ch[Count];
    float _limit[Count] = {15.0f, 45.0f};
    uint8_t _tenHead = 0;
    uint32_t _tenTimer = 0;
    uint8_t _hourHead = 0;
    uint8_t _hourFill = 0; // Anzahl belegter Stundenplaetze
    uint32_t _hourTimer = 0;
    uint32_t _lastHourEpoch = 0;
    uint32_t _dayKey = 0;  // JJJJMMTT
    uint16_t _yearKey = 0; // JJJJ
    bool _restoreShiftPending = false;
    bool _saveRequested = false;
};
