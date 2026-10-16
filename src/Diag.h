#pragma once
#include <Arduino.h>

// Diagnose ohne USB: zuletzt ausgefuehrter Programmteil und Neustartgrund ueberstehen den Neustart
// (RTC-Speicher ohne Initialisierung) und werden nach dem Start in der (Web-)Konsole gemeldet.
namespace Diag
{
    enum Stage : uint8_t
    {
        None,
        Wn90,
        Sds,
        OneWire,
        Rain,
        Forecast,
        Sun,
        Warnings,
        Season,
        Solar,
        Astro,
        Ventilation,
        Irrigation,
        Warnings2,
        Dwd,
        MinMax,
        Send,
        Mqtt,
        HaDiscovery,
        WebStatus,
        Influx,
        InputKo,
        Logic,
        Network,
        Count
    };

    void stage(Stage s);
    void setup();          // Vorstart auswerten und protokollieren
    void report();         // fuer "wx sensors"
    void perf();           // fuer "wx perf": Laufzeiten je Programmteil, Schleifenrate, Speicher
    void loopTick();       // einmal je Hauptschleife
    // true, wenn das KO in der vom ETS geladenen Gruppenobjekt-Tabelle existiert
    bool koExists(uint16_t ko);
} // namespace Diag
