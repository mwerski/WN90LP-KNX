#pragma once
#include <Arduino.h>

// Luftdrucktendenz: alle 15 min ein Wert, Differenz zu vor 1 h bzw. 3 h (in hPa)
class PressureTrend
{
  public:
    static constexpr uint16_t FLASH_SIZE = 1 + 1 + 13 * 4 + 4;

    void feed(float pressure);
    void loop();
    float trend(uint8_t hours) const; // NAN, solange zu wenig Werte vorliegen
    uint8_t samples() const { return _count; }
    float sample(uint8_t stepsBack) const;

    // Verlauf ueber Neustarts (OTA, ETS) retten; wird uebernommen, wenn die Luecke hoechstens 30 min betraegt
    void writeFlash();
    void readFlash();

  private:
    static constexpr uint8_t SIZE = 13; // 3 h in 15-min-Schritten + aktueller Wert
    static constexpr uint32_t STEP = 15 * 60000UL;

    float _ring[SIZE] = {};
    uint8_t _head = 0;
    uint8_t _count = 0;
    float _current = NAN;
    uint32_t _timer = 0;
    uint32_t _lastEpoch = 0;     // Unixzeit des letzten Werts im Ring (0 = unbekannt)
    bool _restorePending = false;
    void applyRestore();
};
