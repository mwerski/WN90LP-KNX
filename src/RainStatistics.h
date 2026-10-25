#pragma once
#include <Arduino.h>

// Regenmengen aus dem 16-Bit-Regenzaehler der WN90LP (0,01 mm, laeuft ueber).
// Gesamtzaehler und Zeitfenster werden aus Differenzen gebildet, damit Ueberlaeufe
// und Neustarts der Wetterstation keine Spruenge erzeugen.
class RainStatistics
{
  public:
    static constexpr uint16_t FLASH_SIZE = 1 + 8 + 8 + 4 + 4 + 1 + 1 + 73 * 4;
    // Kalendersummen (Vortag, Woche, Monat, Jahr), im Modul-Flash hinter den uebrigen Daten
    static constexpr uint16_t PERIOD_FLASH_SIZE = 4 + 3 * (8 + 4);
    // Minutenstaende (5 min / 1 h), im Modul-Flash hinter den Kalendersummen
    static constexpr uint16_t MINUTE_FLASH_SIZE = 4 + 1 + 1 + 61 * 4;

    // Regen beginnt bei mindestens minAmount mm innerhalb von windowMin Minuten
    // und endet, wenn stopDelayMin Minuten kein Niederschlag gemessen wurde
    void setup(uint8_t windowMin, float minAmount, uint8_t stopDelayMin);
    void feed(uint16_t rawCounter);
    void loop();

    double total() const { return _total; }
    float amountMinutes(uint8_t minutes) const;
    float amountHours(uint8_t hours) const;
    float today() const; // NAN ohne gueltige Uhrzeit
    // Kalendersummen; NAN ohne gueltige Uhrzeit bzw. (gestern) ohne vollstaendig gemessenen Vortag
    float yesterday() const { return _yesterday; }
    float week() const { return period(_week); }
    float month() const { return period(_month); }
    float year() const { return period(_year); }
    bool raining() const { return _raining; }
    float rate() const; // mm/h aus den letzten 10 Minuten
    bool hasData() const { return _initialized; }
    uint8_t minuteSamples() const { return _minuteCount; }
    uint8_t hourSamples() const { return _hourCount; }

    // true: Daten haben sich so geaendert, dass sie gespeichert werden sollten
    bool saveRequested();
    void writeFlash();
    void readFlash(const uint8_t *data, uint16_t size);
    void writePeriodFlash();
    void readPeriodFlash();
    void writeMinuteFlash();
    void readMinuteFlash();

  private:
    static constexpr uint8_t MINUTES = 61; // 60 Minuten + aktueller Stand
    static constexpr uint8_t HOURS = 73;   // 72 Stunden + aktueller Stand
    static constexpr uint8_t FLASH_VERSION = 1;
    // Zusaetzlich speichern, wenn seit dem letzten Speichern so viel Regen fiel (Absturz verliert sonst bis zu 1 h),
    // hoechstens alle SAVE_MIN_MS
    static constexpr double SAVE_RAIN_MM = 1.0;
    static constexpr uint32_t SAVE_MIN_MS = 10 * 60000UL;
    // Ohne gueltige Uhrzeit nach dem Start werden die geladenen Minutenstaende verworfen
    static constexpr uint32_t MINUTE_RESTORE_TIMEOUT_MS = 3 * 60000UL;

    struct Period
    {
        double start = 0; // Gesamtstand zu Beginn des Zeitraums
        uint32_t key = 0; // Kennung des Zeitraums (0 = unbekannt)
    };

    void checkDay();
    float period(const Period &p) const { return p.key ? (float)(_total - p.start) : NAN; }
    void pushHour();
    void restoreMinutes();
    void resetMinutes();
    float back(const float *ring, uint8_t size, uint8_t head, uint8_t count, uint8_t steps) const;

    void updateRaining();

    uint8_t _windowMin = 10;
    float _minAmount = 0.1f;
    uint32_t _stopDelayMs = 15 * 60000UL;
    bool _raining = false;
    bool _initialized = false;
    uint16_t _lastRaw = 0;
    double _total = 0;
    uint32_t _lastRain = 0;
    bool _rainSeen = false;

    float _minutes[MINUTES] = {};
    uint8_t _minuteHead = 0;
    uint8_t _minuteCount = 0;
    uint32_t _minuteTimer = 0;
    uint32_t _minuteEpoch = 0;         // Unixzeit des Speicherns der Minutenstaende (0 = unbekannt)
    bool _minuteRestorePending = false; // geladene Minutenstaende warten auf die Uhrzeit (Ring steht so lange)

    float _hours[HOURS] = {};
    uint8_t _hourHead = 0;
    uint8_t _hourCount = 0;
    uint32_t _hourTimer = 0;
    uint32_t _lastHourEpoch = 0; // Unixzeit des letzten Stundenwerts (0 = unbekannt)

    double _midnightTotal = 0;
    uint32_t _dayKey = 0; // JJJJMMTT des Tages, auf den sich _midnightTotal bezieht
    float _yesterday = NAN;
    Period _week;  // Kennung: Tagesnummer (seit 1970) des Montags
    Period _month; // JJJJMM
    Period _year;  // JJJJ
    bool _restoreShiftPending = false;
    bool _saveRequested = false;
    double _savedTotal = 0;  // Gesamtstand beim letzten Speichern
    uint32_t _savedAt = 0;   // millis() des letzten Speicherns
};
