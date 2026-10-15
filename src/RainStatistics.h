#pragma once
#include <Arduino.h>

// Regenmengen aus dem 16-Bit-Regenzaehler der WN90LP (0,01 mm, laeuft ueber).
// Gesamtzaehler und Zeitfenster werden aus Differenzen gebildet, damit Ueberlaeufe
// und Neustarts der Wetterstation keine Spruenge erzeugen.
class RainStatistics
{
  public:
    static constexpr uint16_t FLASH_SIZE = 1 + 8 + 8 + 4 + 4 + 1 + 1 + 73 * 4;

    // Regen beginnt bei mindestens minAmount mm innerhalb von windowMin Minuten
    // und endet, wenn stopDelayMin Minuten kein Niederschlag gemessen wurde
    void setup(uint8_t windowMin, float minAmount, uint8_t stopDelayMin);
    void feed(uint16_t rawCounter);
    void loop();

    double total() const { return _total; }
    float amountMinutes(uint8_t minutes) const;
    float amountHours(uint8_t hours) const;
    float today() const; // NAN ohne gueltige Uhrzeit
    bool raining() const { return _raining; }
    float rate() const; // mm/h aus den letzten 10 Minuten
    bool hasData() const { return _initialized; }
    uint8_t minuteSamples() const { return _minuteCount; }
    uint8_t hourSamples() const { return _hourCount; }

    // true: Daten haben sich so geaendert, dass sie gespeichert werden sollten
    bool saveRequested();
    void writeFlash();
    void readFlash(const uint8_t *data, uint16_t size);

  private:
    static constexpr uint8_t MINUTES = 61; // 60 Minuten + aktueller Stand
    static constexpr uint8_t HOURS = 73;   // 72 Stunden + aktueller Stand
    static constexpr uint8_t FLASH_VERSION = 1;

    void checkDay();
    void pushHour();
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

    float _hours[HOURS] = {};
    uint8_t _hourHead = 0;
    uint8_t _hourCount = 0;
    uint32_t _hourTimer = 0;
    uint32_t _lastHourEpoch = 0; // Unixzeit des letzten Stundenwerts (0 = unbekannt)

    double _midnightTotal = 0;
    uint32_t _dayKey = 0; // JJJJMMTT des Tages, auf den sich _midnightTotal bezieht
    bool _restoreShiftPending = false;
    bool _saveRequested = false;
};
