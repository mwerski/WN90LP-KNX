#pragma once
#include <Arduino.h>

// Bewaesserungsbedarf: Referenzverdunstung ET0 nach FAO-56 Penman-Monteith (Stundenform,
// laufend integriert) und Wasserbilanz Regen - Kc * ET0 + Bewaesserung.
class Irrigation
{
  public:
    static constexpr uint16_t FLASH_SIZE = 4 * 4;

    void setup(float kc, float threshold, float storage, float amount);
    // dtHours: Zeit seit dem letzten Aufruf; Werte NAN, wenn nicht verfuegbar
    // t C, rh %, wind m/s, rs W/m2 (Globalstrahlung), rso W/m2 (klarer Himmel, 0 nachts), p hPa, rainTotal mm
    void update(float dtHours, float t, float rh, float wind, float rs, float rso, float p, double rainTotal);
    void credit(); // "Bewaesserung erfolgt"
    void dayRollover(bool consecutive);

    float et0Today() const { return _et0Today; }
    float et0Yesterday() const { return _et0Yesterday; }
    float balance() const { return _balance; }
    bool irrigate() const { return _irrigate; }
    float rate() const { return _lastRate; } // mm/h

    void writeFlash();
    void readFlash();

  private:
    static float et0Hourly(float t, float rh, float wind, float rs, float rso, float p);
    void clampAndEvaluate();

    float _kc = 0.8f;
    float _threshold = 5;
    float _storage = 20;
    float _amount = 10;

    float _et0Today = 0;
    float _et0Yesterday = NAN;
    float _balance = 0;
    float _lastRate = 0;
    double _lastRain = NAN;
    bool _irrigate = false;
};
