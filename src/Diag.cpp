#include "Diag.h"
#include "OpenKNX.h"
#include <esp_system.h>

namespace Diag
{
    static constexpr uint32_t MAGIC = 0x57583930; // "WX90"
    RTC_NOINIT_ATTR static uint32_t s_magic;
    RTC_NOINIT_ATTR static uint8_t s_stage;

    // Absturzdaten aus dem Panic-Handler (ueberstehen den Neustart, ohne USB auswertbar)
    static constexpr uint8_t CRASH_DEPTH = 16;
    struct CrashInfo
    {
        uint32_t magic;
        char reason[48];
        char task[16];
        uint8_t depth;
        uint32_t pc[CRASH_DEPTH];
    };
    RTC_NOINIT_ATTR static CrashInfo s_crash;
    static CrashInfo s_lastCrash; // Kopie fuer die Anzeige
    static bool s_haveCrash = false;
    static void recordCrash(arduino_panic_info_t *info, void *);
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
        if (s_crash.magic == MAGIC && s_crash.depth <= CRASH_DEPTH)
        {
            s_lastCrash = s_crash;
            s_lastCrash.reason[sizeof(s_lastCrash.reason) - 1] = 0;
            s_lastCrash.task[sizeof(s_lastCrash.task) - 1] = 0;
            s_haveCrash = true;
        }
        s_crash.magic = 0;
        set_arduino_panic_handler(recordCrash, nullptr);
        report();
    }

    void report()
    {
        const bool crash = s_reason == ESP_RST_PANIC || s_reason == ESP_RST_INT_WDT || s_reason == ESP_RST_TASK_WDT || s_reason == ESP_RST_WDT;
        if (crash)
            logError("Weather", "Letzter Neustart: %s, zuletzt aktiv: %s", reasonName(s_reason), stageName(s_lastStage));
        else
            logInfo("Weather", "Letzter Neustart: %s", reasonName(s_reason));
        if (!s_haveCrash)
            return;
        // Backtrace wie im seriellen Absturzprotokoll; mit addr2line und der firmware.elf dieses Builds auswerten
        char bt[CRASH_DEPTH * 11 + 1];
        size_t n = 0;
        for (uint8_t i = 0; i < s_lastCrash.depth && n + 11 < sizeof(bt); i++)
            n += snprintf(bt + n, sizeof(bt) - n, " 0x%08lx", (unsigned long)s_lastCrash.pc[i]);
        bt[n] = 0;
        logError("Weather", "Absturz in Task \"%s\": %s", s_lastCrash.task, s_lastCrash.reason);
        logError("Weather", "Backtrace:%s", bt);
    }

    static void IRAM_ATTR copyText(char *dst, size_t size, const char *src)
    {
        size_t i = 0;
        if (src)
            for (; i + 1 < size && src[i]; i++)
                dst[i] = src[i];
        dst[i] = 0;
    }

    // Aufruf aus dem Panic-Handler des Arduino-Kerns (set_arduino_panic_handler): nur RTC-Speicher beschreiben
    static void IRAM_ATTR recordCrash(arduino_panic_info_t *info, void *)
    {
        s_crash.magic = MAGIC;
        copyText(s_crash.reason, sizeof(s_crash.reason), info->reason);
        copyText(s_crash.task, sizeof(s_crash.task), pcTaskGetName(NULL));
        s_crash.depth = 0;
        for (uint8_t i = 0; i < info->backtrace_len && s_crash.depth < CRASH_DEPTH; i++)
            s_crash.pc[s_crash.depth++] = info->backtrace[i];
    }

    bool koExists(uint16_t ko)
    {
        return ko >= 1 && ko <= knx.bau().groupObjectTable().entryCount();
    }
} // namespace Diag
