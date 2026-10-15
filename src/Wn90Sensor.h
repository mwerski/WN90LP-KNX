#pragma once
#include <Arduino.h>

// Ecowitt WN90LP ueber Modbus RTU (Slave 0x90, 9600 8N1), nicht-blockierend.
// Register 0x0165-0x016E: Licht, UV, Temperatur, Feuchte, Wind, Boe, Richtung,
// Regen (0,1 mm), Luftdruck, Regenzaehler (0,01 mm). 0xFFFF = ungueltig.
class Wn90Sensor
{
  public:
    static constexpr uint8_t REG_COUNT = 10;
    static constexpr uint16_t INVALID = 0xFFFF;

    enum Reg : uint8_t
    {
        Light,
        Uv,
        Temperature,
        Humidity,
        Wind,
        Gust,
        WindDir,
        Rainfall,
        Pressure,
        RainCounter
    };

    void setup();
    // true, wenn ein neuer Datensatz gelesen wurde
    bool loop();

    uint16_t raw(Reg reg) const { return _raw[reg]; }
    bool valid(Reg reg) const { return _ok && _raw[reg] != INVALID; }
    bool ok() const { return _ok; }
    bool faulty() const { return _consecutiveErrors >= FAULT_AFTER; }
    uint32_t reads() const { return _reads; }
    uint32_t errors() const { return _errors; }
    uint8_t lastError() const { return _lastError; }

  private:
    static constexpr uint32_t POLL_INTERVAL = 2000;
    static constexpr uint8_t FAULT_AFTER = 5; // 10 s ohne Antwort

    uint16_t _raw[REG_COUNT] = {};
    bool _ok = false;
    bool _busy = false;
    uint32_t _lastPoll = 0;
    uint32_t _reads = 0;
    uint32_t _errors = 0;
    uint8_t _consecutiveErrors = 0;
    uint8_t _lastError = 0;
};
