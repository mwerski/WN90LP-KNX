#pragma once
#include "OpenKNX.h"
#include "OpenKNX/Led/RGB.h"
#include <driver/rtc_io.h>
#include <esp32-hal-periman.h>
#include <mutex>

// Einzelne WS2812 ueber rgbLedWrite() des Arduino-Kerns. Ersetzt den SerialLED-Treiber von
// OGM-Common, der die LED des T-RSS3 nach dem Start nicht mehr aktualisierte (blieb blau).
// Dient als Prog-LED von OpenKNX und zusaetzlich als Statusanzeige (status()/tick() aus dem
// Wettermodul). Programmiermodus und Fehlercodes von OpenKNX haben Vorrang vor dem Status.
// Aufrufe kommen aus der LED-Task von OpenKNX und aus der Hauptschleife, daher mit Sperre.
class RgbLed : public OpenKNX::Led::RGB
{
  public:
    enum class Pattern : uint8_t
    {
        Off,
        On,
        Blink, // 0,5 s an, 0,5 s aus
        Pulse, // 2 s auf- und abdimmen
        Flash  // 100 ms alle 10 s
    };

    RgbLed(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) : _pin(pin), _rgb{r, g, b} { s_instance = this; }

    static RgbLed *instance() { return s_instance; }

    void init() override
    {
        _initialized = true;
        show(true);
    }

    // Pin zurueckholen, wenn ein anderer Programmteil ihn als Analogeingang genutzt hat. Das Logikmodul
    // ruft in setup() randomSeed(analogRead(A0)) auf, und A0 ist beim ESP32-S3 GPIO 1: Der ADC legt den
    // Pin in die RTC-Domaene, die RMT-Initialisierung holt ihn nicht zurueck, die LED bekommt kein Signal.
    static void releasePin(uint8_t pin)
    {
        perimanClearPinBus(pin);
        if (rtc_gpio_is_valid_gpio((gpio_num_t)pin))
            rtc_gpio_deinit((gpio_num_t)pin);
    }

    void reattach()
    {
        {
            std::lock_guard<std::mutex> guard(_lock);
            releasePin(_pin);
        }
        show(true);
    }

    void color(uint8_t r, uint8_t g, uint8_t b) override
    {
        _rgb[0] = r;
        _rgb[1] = g;
        _rgb[2] = b;
        show(false);
    }

    // Statusanzeige setzen (Farbe bei voller Helligkeit, brightness 1-100 %)
    void status(Pattern pattern, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t brightness = 100)
    {
        _status = pattern;
        _statusRgb[0] = (uint16_t)r * brightness / 100;
        _statusRgb[1] = (uint16_t)g * brightness / 100;
        _statusRgb[2] = (uint16_t)b * brightness / 100;
    }

    // Aus der Hauptschleife fuer die zeitabhaengigen Muster
    void tick() { show(false); }

  private:
    void writeLed(uint8_t brightness) override
    {
        _currentLedBrightness = brightness;
        show(false);
    }

    // Helligkeitsfaktor 0-255 des Statusmusters zum Zeitpunkt now
    static uint8_t level(Pattern p, uint32_t now)
    {
        switch (p)
        {
            case Pattern::On:
                return 255;
            case Pattern::Blink:
                return now % 1000 < 500 ? 255 : 0;
            case Pattern::Pulse:
            {
                const uint32_t t = now % 2000;
                return (t < 1000 ? t : 2000 - t) * 255 / 1000;
            }
            case Pattern::Flash:
                return now % 10000 < 100 ? 255 : 0;
            default:
                return 0;
        }
    }

    void show(bool force)
    {
        if (!_initialized)
            return;
        uint8_t v[3];
        if (_status != Pattern::Off && !knx.progMode() && !_errorMode)
        {
            const uint8_t l = level(_status, millis());
            for (uint8_t i = 0; i < 3; i++)
                v[i] = (uint16_t)_statusRgb[i] * l / 255;
        }
        else
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
    volatile Pattern _status = Pattern::Off;
    volatile uint8_t _statusRgb[3] = {0, 0, 0};
    std::mutex _lock;
    static inline RgbLed *s_instance = nullptr;
};
