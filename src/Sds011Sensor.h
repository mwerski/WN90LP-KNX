#pragma once
#include <Arduino.h>

// Feinstaubsensor SDS011 im aktiven Modus: der Sensor sendet selbststaendig
// Messrahmen (AA C0 ... AB), die hier nicht-blockierend ausgewertet werden.
class Sds011Sensor
{
  public:
    // dutyCycleMin: 0 = Dauerbetrieb (1 Messung/s), 1..30 = Messintervall in Minuten
    void setup(uint8_t dutyCycleMin);
    // true, wenn ein neuer Messwert empfangen wurde
    bool loop();

    float pm25() const { return _pm25; }
    float pm10() const { return _pm10; }
    bool faulty() const { return _faulty; }
    uint32_t frames() const { return _frames; }
    uint32_t checksumErrors() const { return _checksumErrors; }
    uint32_t lastFrameAge() const { return _frames ? millis() - _lastFrame : 0; }
    uint8_t dutyCycle() const { return _dutyCycle; }

  private:
    void sendCommand(uint8_t cmd, uint8_t data2, uint8_t data3);

    uint8_t _dutyCycle = 0;
    uint8_t _buf[10] = {};
    uint8_t _pos = 0;
    float _pm25 = NAN;
    float _pm10 = NAN;
    bool _faulty = false;
    uint32_t _start = 0;
    uint32_t _lastFrame = 0;
    uint32_t _frames = 0;
    uint32_t _checksumErrors = 0;
};
