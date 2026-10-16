#include "OtaGuard.h"
#include "OpenKNX.h"
#include <esp_ota_ops.h>

// Bestaetigung uebernimmt OtaGuard statt des Arduino-Kerns (der bestaetigt sonst sofort beim Start)
extern "C" bool verifyRollbackLater()
{
    return true;
}

namespace OtaGuard
{
    static constexpr uint32_t CONFIRM_AFTER = 60000;   // mit Netz mindestens 1 min stabil
    static constexpr uint32_t ROLLBACK_AFTER = 600000; // ohne Netz nach 10 min zurueck
    static bool s_pending = false;

    void setup()
    {
        esp_ota_img_states_t state;
        if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            s_pending = true;
            logInfo("Main", "Neue Firmware noch nicht bestaetigt: Rollback aktiv, bis das Netz 1 min stabil laeuft");
        }
    }

    void confirm(const char *reason)
    {
        if (!s_pending)
            return;
        esp_ota_mark_app_valid_cancel_rollback();
        s_pending = false;
        logInfo("Main", "Neue Firmware bestaetigt (%s)", reason);
    }

    bool pending()
    {
        return s_pending;
    }

    void loop(bool networkEstablished)
    {
        if (!s_pending)
            return;
        if (networkEstablished && millis() > CONFIRM_AFTER)
            confirm("Netz stabil");
        else if (millis() > ROLLBACK_AFTER)
        {
            logError("Main", "Neue Firmware ohne Netz: Rueckkehr zur vorherigen Firmware");
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }
} // namespace OtaGuard
