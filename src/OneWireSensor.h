#pragma once
#include <DallasTemperature.h>
#include <OneWire.h>

// Erster DS18B20 am 1-Wire-Bus, nicht-blockierend: anfordern, 800 ms spaeter lesen
class OneWireSensor
{
  public:
    OneWireSensor();
    void setup();
    // true, wenn ein neuer Messwert gelesen wurde (auch bei Fehler: dann NAN)
    bool loop();
    void scan();

    float temperature() const { return _temperature; }
    bool faulty() const { return _failures >= FAULT_AFTER; }
    bool found() const { return _found; }
    uint32_t failures() const { return _failures; }

  private:
    static constexpr uint32_t INTERVAL = 10000;
    static constexpr uint32_t CONVERSION_TIME = 800;
    static constexpr uint8_t FAULT_AFTER = 3;

    bool findSensor();

    OneWire _oneWire;
    DallasTemperature _sensors;
    DeviceAddress _address = {};
    bool _found = false;
    bool _converting = false;
    uint32_t _timer = 0;
    float _temperature = NAN;
    uint32_t _failures = 0;
};
