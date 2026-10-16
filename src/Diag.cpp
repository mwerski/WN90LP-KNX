#include "Diag.h"
#include "OpenKNX.h"
#include <esp_system.h>

namespace Diag
{
    static constexpr uint32_t MAGIC = 0x57583930; // "WX90"
    RTC_NOINIT_ATTR static uint32_t s_magic;
    RTC_NOINIT_ATTR static uint8_t s_stage;
    static uint8_t s_lastStage = None;
    static esp_reset_reason_t s_reason = ESP_RST_UNKNOWN;

    static const char *stageName(uint8_t s)
    {
        static const char *names[Count] = {"-", "WN90LP", "SDS011", "1-Wire", "Regen", "Prognose", "Sonne/Daemmerung", "Warnungen",
                                           "Sommer/Winter", "Strahlung/Fassaden", "Sonnenzeiten/Mond", "Lueftung", "Bewaesserung",
                                           "DWD-Kriterien", "DWD-Abruf", "Min/Max", "Senden", "MQTT", "HA-Discovery", "Statusseite",
                                           "InfluxDB", "Eingangs-KO", "Logikmodul", "OpenKNX (KNX/Netzwerk/Logik)"};
        return s < Count ? names[s] : "?";
    }

    static const char *reasonName(esp_reset_reason_t r)
    {
        switch (r)
        {
            case ESP_RST_POWERON:
                return "Einschalten";
            case ESP_RST_SW:
                return "Neustart durch Software";
            case ESP_RST_PANIC:
                return "ABSTURZ (Exception/Panic)";
            case ESP_RST_INT_WDT:
                return "ABSTURZ (Interrupt-Watchdog)";
            case ESP_RST_TASK_WDT:
                return "ABSTURZ (Task-Watchdog, Hauptschleife blockiert)";
            case ESP_RST_WDT:
                return "ABSTURZ (Watchdog)";
            case ESP_RST_BROWNOUT:
                return "Unterspannung";
            case ESP_RST_DEEPSLEEP:
                return "Deep Sleep";
            default:
                return "unbekannt";
        }
    }

    // Laufzeitmessung: Zeit von einem Stufenwechsel zum naechsten wird der vorherigen Stufe zugerechnet
    struct Timing
    {
        uint32_t maxUs = 0;
        uint64_t sumUs = 0;
        uint32_t count = 0;
    };
    static Timing s_timing[Count];
    static uint32_t s_stageStart = 0;
    static uint8_t s_current = None;
    static uint32_t s_loops = 0;
    static uint32_t s_perfStart = 0;
    static uint32_t s_loopStart = 0;
    static uint32_t s_maxLoopUs = 0;
    static uint32_t s_noMemory = 0;

    void noMemory()
    {
        s_noMemory++;
    }

    void stage(Stage s)
    {
        const uint32_t now = micros();
        if (s_stageStart)
        {
            Timing &t = s_timing[s_current];
            const uint32_t d = now - s_stageStart;
            if (d > t.maxUs)
                t.maxUs = d;
            t.sumUs += d;
            t.count++;
        }
        s_stageStart = now;
        s_current = s;
        s_stage = s;
    }

    void loopTick()
    {
        const uint32_t now = micros();
        if (s_loopStart && now - s_loopStart > s_maxLoopUs)
            s_maxLoopUs = now - s_loopStart;
        s_loopStart = now;
        s_loops++;
        if (!s_perfStart)
            s_perfStart = millis();
    }

    void perf()
    {
        const float secs = max<uint32_t>(millis() - s_perfStart, 1) / 1000.0f;
        logInfo("Weather", "Hauptschleife: %.0f Durchlaeufe/s, laengster Durchlauf %.1f ms, freier Speicher %u Byte (min. %u, groesster Block %u)",
                s_loops / secs, s_maxLoopUs / 1000.0f, ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
        logIndentUp();
        if (s_noMemory)
            logInfo("Weather", "Wegen Speichermangel ausgelassene Ausgaben seit Start: %lu", s_noMemory);
        for (uint8_t i = 0; i < Count; i++)
        {
            const Timing &t = s_timing[i];
            if (!t.count)
                continue;
            logInfo("Weather", "%-30s max %7.1f ms, Mittel %6.2f ms, Anteil %4.1f %%", stageName(i), t.maxUs / 1000.0f,
                    t.sumUs / 1000.0f / t.count, t.sumUs / 10000.0f / secs);
        }
        logIndentDown();
        // Messung neu beginnen
        for (Timing &t : s_timing)
            t = Timing();
        s_loops = 0;
        s_maxLoopUs = 0;
        s_perfStart = millis();
    }

    void setup()
    {
        s_reason = esp_reset_reason();
        s_lastStage = s_magic == MAGIC ? s_stage : None;
        s_magic = MAGIC;
        s_stage = None;
        report();
    }

    void report()
    {
        const bool crash = s_reason == ESP_RST_PANIC || s_reason == ESP_RST_INT_WDT || s_reason == ESP_RST_TASK_WDT || s_reason == ESP_RST_WDT;
        if (crash)
            logError("Weather", "Letzter Neustart: %s, zuletzt aktiv: %s", reasonName(s_reason), stageName(s_lastStage));
        else
            logInfo("Weather", "Letzter Neustart: %s", reasonName(s_reason));
    }

    bool koExists(uint16_t ko)
    {
        return ko >= 1 && ko <= knx.bau().groupObjectTable().entryCount();
    }
} // namespace Diag
