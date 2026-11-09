#include "Diag.h"
#include "OpenKNX.h"
#include <atomic>
#include <esp_heap_caps.h>
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

    // Speicher und Stack-Reserven, laufend aus der Hauptschleife (ueberstehen den Absturz)
    static const char *const TASKS[] = {"loopTask", "mqtt", "DwdWarnings", "InfluxWriter", "httpd"};
    static constexpr uint8_t TASK_COUNT = sizeof(TASKS) / sizeof(TASKS[0]);
    struct HeapInfo
    {
        uint32_t magic;
        uint32_t upSec;
        uint32_t freeHeap, minFree, maxBlock;
        uint32_t stackMin[TASK_COUNT]; // kleinste gemessene Stack-Reserve in Byte (0xFFFFFFFF = Task fehlt)
    };
    RTC_NOINIT_ATTR static HeapInfo s_heap;
    static HeapInfo s_lastHeap; // Stand vor dem letzten Neustart
    static bool s_haveHeap = false;
    static uint32_t s_heapTimer = 0;
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
    static uint32_t s_noMemoryBy[Count] = {};
    static uint32_t s_noMemoryLast = 0; // Laufzeit in s

    // Speichertiefstwert: Zeitpunkt, laufende Stufe der Hauptschleife und aktive Hintergrundaufgaben
    static std::atomic<uint8_t> s_activity{0};
    static uint32_t s_lowFree = UINT32_MAX;
    static uint32_t s_lowSec = 0;
    static uint8_t s_lowStage = None;
    static uint8_t s_lowActivity = 0;

    // Fehlgeschlagene Speicheranforderungen (Callback aus beliebiger Task, nur Variablen schreiben)
    static volatile uint32_t s_allocFails = 0;
    static volatile uint32_t s_allocFailSize = 0;
    static volatile uint32_t s_allocFailSec = 0;
    static char s_allocFailTask[16] = "";

    static void allocFailed(size_t size, uint32_t caps, const char *function)
    {
        s_allocFails = s_allocFails + 1;
        s_allocFailSize = size;
        s_allocFailSec = millis() / 1000;
        const char *name = pcTaskGetName(NULL);
        size_t i = 0;
        if (name)
            for (; i + 1 < sizeof(s_allocFailTask) && name[i]; i++)
                s_allocFailTask[i] = name[i];
        s_allocFailTask[i] = 0;
    }

    void noMemory()
    {
        s_noMemory++;
        s_noMemoryBy[s_current]++;
        s_noMemoryLast = millis() / 1000;
    }

    void activity(Activity a, bool on)
    {
        if (on)
            s_activity.fetch_or(a);
        else
            s_activity.fetch_and(~a);
    }

    static void activityText(uint8_t act, char *buf, size_t size)
    {
        snprintf(buf, size, "%s%s%s", act ? "" : "keine", act & ActDwd ? "DWD-Abruf " : "", act & ActInflux ? "InfluxDB " : "");
    }

    static void lowReport()
    {
        if (s_lowFree != UINT32_MAX)
        {
            char act[40];
            activityText(s_lowActivity, act, sizeof(act));
            logInfo("Weather", "Speichertiefstwert %lu Byte nach %lu s (vor %lu s), Hauptschleife in \"%s\", Hintergrund: %s",
                    (unsigned long)s_lowFree, (unsigned long)s_lowSec, (unsigned long)(millis() / 1000 - s_lowSec), stageName(s_lowStage), act);
        }
        if (s_allocFails)
            logError("Weather", "Fehlgeschlagene Speicheranforderungen: %lu, zuletzt %lu Byte in Task \"%s\" vor %lu s",
                     (unsigned long)s_allocFails, (unsigned long)s_allocFailSize, s_allocFailTask,
                     (unsigned long)(millis() / 1000 - s_allocFailSec));
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
        // Neuer Tiefstwert des freien Speichers: Umstaende festhalten (Abfrage ist billig, kein Heap-Durchlauf)
        const uint32_t minFree = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        if (minFree < s_lowFree)
        {
            s_lowFree = minFree;
            s_lowSec = millis() / 1000;
            s_lowStage = s_current;
            s_lowActivity = s_activity.load();
        }
        if (!s_perfStart)
            s_perfStart = millis();
        // Speicherstand jede Sekunde sichern (getMaxAllocHeap durchsucht den Heap)
        if (delayCheck(s_heapTimer, 1000))
        {
            s_heapTimer = millis();
            s_heap.upSec = millis() / 1000;
            s_heap.freeHeap = ESP.getFreeHeap();
            s_heap.minFree = ESP.getMinFreeHeap();
            s_heap.maxBlock = ESP.getMaxAllocHeap();
            for (uint8_t i = 0; i < TASK_COUNT; i++)
            {
                TaskHandle_t h = xTaskGetHandle(TASKS[i]);
                if (h)
                    s_heap.stackMin[i] = min<uint32_t>(s_heap.stackMin[i], uxTaskGetStackHighWaterMark(h));
            }
        }
    }

    void perf()
    {
        const float secs = max<uint32_t>(millis() - s_perfStart, 1) / 1000.0f;
        logInfo("Weather", "Hauptschleife: %.0f Durchlaeufe/s, laengster Durchlauf %.1f ms, freier Speicher %u Byte (min. %u, groesster Block %u)",
                s_loops / secs, s_maxLoopUs / 1000.0f, ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
        logIndentUp();
        if (s_noMemory)
        {
            logInfo("Weather", "Wegen Speichermangel ausgelassene Ausgaben seit Start: %lu, zuletzt vor %lu s", s_noMemory,
                    (unsigned long)(millis() / 1000 - s_noMemoryLast));
            logIndentUp();
            for (uint8_t i = 0; i < Count; i++)
                if (s_noMemoryBy[i])
                    logInfo("Weather", "%-30s %lu", stageName(i), (unsigned long)s_noMemoryBy[i]);
            logIndentDown();
        }
        lowReport();
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
        if (s_heap.magic == MAGIC)
        {
            s_lastHeap = s_heap;
            s_haveHeap = true;
        }
        s_heap = HeapInfo();
        s_heap.magic = MAGIC;
        for (uint32_t &v : s_heap.stackMin)
            v = UINT32_MAX;
        set_arduino_panic_handler(recordCrash, nullptr);
        heap_caps_register_failed_alloc_callback(allocFailed);
        report();
    }

    static void stacks(const char *title, const uint32_t *values)
    {
        char buf[160];
        size_t n = snprintf(buf, sizeof(buf), "%s:", title);
        for (uint8_t i = 0; i < TASK_COUNT && n < sizeof(buf); i++)
            if (values[i] != UINT32_MAX)
                n += snprintf(buf + n, sizeof(buf) - n, " %s %lu", TASKS[i], (unsigned long)values[i]);
        logInfo("Weather", "%s Byte", buf);
    }

    void report()
    {
        const bool crash = s_reason == ESP_RST_PANIC || s_reason == ESP_RST_INT_WDT || s_reason == ESP_RST_TASK_WDT || s_reason == ESP_RST_WDT;
        if (crash)
            logError("Weather", "Letzter Neustart: %s, zuletzt aktiv: %s", reasonName(s_reason), stageName(s_lastStage));
        else
            logInfo("Weather", "Letzter Neustart: %s", reasonName(s_reason));
        if (crash && s_haveHeap)
        {
            logError("Weather", "Vor dem Absturz (Laufzeit %lu s): frei %lu Byte (min. %lu), groesster Block %lu Byte",
                     (unsigned long)s_lastHeap.upSec, (unsigned long)s_lastHeap.freeHeap, (unsigned long)s_lastHeap.minFree,
                     (unsigned long)s_lastHeap.maxBlock);
            stacks("Stack-Reserven vor dem Absturz", s_lastHeap.stackMin);
        }
        stacks("Stack-Reserven seit Start", s_heap.stackMin);
        lowReport();
        if (!s_haveCrash)
        {
            // Der Kern laeuft den Stack ab, bevor er den Handler ruft; scheitert das, fehlt der Eintrag
            if (s_reason == ESP_RST_PANIC)
                logError("Weather", "Kein Backtrace: Panic-Handler nicht erreicht (Stack vermutlich beschaedigt)");
            return;
        }
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
