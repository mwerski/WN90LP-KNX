#pragma once
#include <Arduino.h>

// Luftdrucktendenz: alle 15 min ein Wert, Differenz zu vor 1 h bzw. 3 h (in hPa)
class PressureTrend
{
  public:
    void feed(float pressure);
    void loop();
    float trend(uint8_t hours) const; // NAN, solange zu wenig Werte vorliegen
    uint8_t samples() const { return _count; }
    float sample(uint8_t stepsBack) const;

  private:
    static constexpr uint8_t SIZE = 13; // 3 h in 15-min-Schritten + aktueller Wert
    static constexpr uint32_t STEP = 15 * 60000UL;

    float _ring[SIZE] = {};
    uint8_t _head = 0;
    uint8_t _count = 0;
    float _current = NAN;
    uint32_t _timer = 0;
};
