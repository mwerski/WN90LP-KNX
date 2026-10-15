#pragma once
#include <mutex>
#include <string>

// Statusseite "Wetter" im Webserver von OFM-Network (/weather, Daten unter /weather.json).
// Die HTTP-Handler laufen auf dem ESP32 in der httpd-Task: sie lesen nur eine Kopie,
// die die Hauptschleife alle 2 s neu erstellt, solange die Seite abgefragt wird.
class WebStatus
{
  public:
    void setup();
    // true, wenn ein neuer Schnappschuss erstellt werden soll
    bool due();
    void update(std::string &&json);

  private:
    std::string snapshot();

    std::mutex _lock;
    std::string _json = "{}";
    volatile uint32_t _demandUntil = 0;
    uint32_t _lastBuild = 0;
};
