#pragma once
#include <Arduino.h>
#include <mutex>
#include <string>
#include <vector>

// Amtliche Warnungen des DWD fuer eine Warnzelle (WarnCellID), per HTTPS vom GeoServer maps.dwd.de
// (offene Daten, WFS dwd:Warnungen_Gemeinden bzw. dwd:Warnungen_Landkreise). Benoetigt Internetzugang.
// Der Abruf laeuft in einer eigenen Task; die Hauptschleife liest nur das Ergebnis.
class DwdWarnings
{
  public:
    struct Result
    {
        bool valid = false;        // mindestens ein erfolgreicher Abruf
        uint8_t levelNow = 0;      // hoechste derzeit gueltige Warnung (0-4)
        uint8_t levelAll = 0;      // inkl. angekuendigter Warnungen
        uint8_t countNow = 0;
        std::string event;         // Ereignis der hoechsten gueltigen (sonst angekuendigten) Warnung, UTF-8
        std::string headline;
        std::string onset, expires; // ISO 8601 (UTC)
        uint32_t updated = 0;      // millis() des letzten Erfolgs
        int lastCode = 0;
        uint32_t ok = 0, errors = 0;
    };

    // Einzelne Warnung mit Volltexten (fuer MQTT und Statusseite)
    struct Warning
    {
        std::string event, headline, description, instruction; // UTF-8, Texte auf MAX_TEXT gekuerzt
        std::string onset, expires;                            // ISO 8601 (UTC)
        uint8_t level = 0;
        bool now = false; // gilt derzeit (sonst angekuendigt)
        bool operator==(const Warning &o) const
        {
            return level == o.level && now == o.now && event == o.event && headline == o.headline && description == o.description &&
                   instruction == o.instruction && onset == o.onset && expires == o.expires;
        }
    };
    static constexpr uint8_t MAX_WARNINGS = 8;
    static constexpr size_t MAX_TEXT = 600;

    void setup(uint32_t warnCell, uint8_t intervalMin);
    bool enabled() const { return _enabled; }
    // Aufruf aus der Hauptschleife; startet bei Bedarf einen Abruf (now: Unixzeit, 0 = unbekannt)
    void loop(int64_t nowUtc);
    Result result();
    bool faulty();
    // Alle Warnungen: gueltige vor angekuendigten, dann hoechste Stufe, dann frueherer Beginn
    std::vector<Warning> warnings();
    // Erhoeht sich bei jeder Aenderung der Warnungsliste
    uint32_t warningsSeq();
    // Text fuer DPT 16 (ISO 8859-1, max. 14 Zeichen) bzw. UTF-8; Puffer gilt bis zum naechsten Aufruf
    static const char *eventText(bool utf8);
    // Kurzbeschreibung eines Ergebniscodes (HTTP-Status, Fehler von HTTPClient oder eigene Codes)
    static std::string codeText(int code);

  private:
    static void taskEntry(void *arg);
    void run();
    void fetch(int64_t nowUtc);

    bool _enabled = false;
    std::string _url;
    uint32_t _intervalMs = 300000;
    uint32_t _timer = 0;
    bool _first = true;

    std::mutex _lock;
    Result _result;
    std::vector<Warning> _warnings;
    uint32_t _warningsSeq = 0;
    volatile bool _busy = false;
    volatile int64_t _requestNow = 0;
    TaskHandle_t _task = nullptr;

    static std::string s_event;
};

extern DwdWarnings dwdWarnings;
