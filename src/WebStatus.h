#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Statusseite "Wetter" im Webserver von OFM-Network (/weather, Daten unter /weather.json).
// Die HTTP-Handler laufen auf dem ESP32 in der httpd-Task: sie lesen nur einen Schnappschuss,
// den die Hauptschleife alle 2 s neu erstellt, solange die Seite abgefragt wird.
// Der Schnappschuss besteht aus Stuecken von etwa 1,5 KB und wird stueckweise (chunked) gesendet,
// damit kein grosser zusammenhaengender Speicherblock noetig ist (das JSON hat ueber 19 KB).
class WebStatus
{
  public:
    using Snapshot = std::vector<std::string>;

    void setup();
    // true, wenn ein neuer Schnappschuss erstellt werden soll
    bool due();
    // nullptr (Speichermangel): alten Schnappschuss behalten
    void update(std::shared_ptr<const Snapshot> snapshot);

  private:
    std::mutex _lock;
    std::shared_ptr<const Snapshot> _snapshot;
    volatile uint32_t _demandUntil = 0;
    uint32_t _lastBuild = 0;
};
