#pragma once
#include "OpenKNX.h"
#include "OpenKNX/Led/RGB.h"
#include <mutex>

// Einzelne WS2812 ueber rgbLedWrite() des Arduino-Kerns. Ersetzt den SerialLED-Treiber von
// OGM-Common, der die LED des T-RSS3 nach dem Start nicht mehr aktualisierte (blieb blau).
// Aufrufe kommen aus der LED-Task von OpenKNX und aus der Hauptschleife, daher mit Sperre.
class RgbLed : public OpenKNX::Led::RGB
{
  public:
    RgbLed(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) : _pin(pin), _rgb{r, g, b} {}

    void init() override
    {
        _initialized = true;
        show(true);
    }

    void color(uint8_t r, uint8_t g, uint8_t b) override
    {
        _rgb[0] = r;
        _rgb[1] = g;
        _rgb[2] = b;
        show(false);
    }

  private:
    void writeLed(uint8_t brightness) override
    {
        _currentLedBrightness = brightness;
        show(false);
    }

    void show(bool force)
    {
        if (!_initialized)
            return;
        uint8_t v[3];
        for (uint8_t i = 0; i < 3; i++)
            v[i] = (uint32_t)_rgb[i] * _currentLedBrightness * _maxBrightness / (255 * 255);
        std::lock_guard<std::mutex> guard(_lock);
        if (!force && v[0] == _shown[0] && v[1] == _shown[1] && v[2] == _shown[2])
            return;
        memcpy(_shown, v, 3);
        rgbLedWrite(_pin, v[0], v[1], v[2]);
    }

    uint8_t _pin;
    uint8_t _rgb[3];
    uint8_t _shown[3] = {0, 0, 0};
    std::mutex _lock;
};
