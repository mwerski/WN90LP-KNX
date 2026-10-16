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

    void stage(Stage s)
    {
        s_stage = s;
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
