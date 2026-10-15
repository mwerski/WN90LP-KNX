#include "NetworkModule.h"
#include "OpenKNX.h"
#include "WeatherModule.h"
#include <WiFi.h>
#include <esp_ota_ops.h>

// Sicherheitsnetz fuer OTA-Updates (Bootloader mit App-Rollback): Eine neu eingespielte Firmware
// wird erst bestaetigt, wenn sie stabil im Netz laeuft. Neustart, Absturz oder Watchdog vorher,
// oder kein Netz nach 10 min, fuehren zurueck zur vorherigen Firmware.
extern "C" bool verifyRollbackLater()
{
    return true; // Bestaetigung uebernimmt checkOtaRollback()
}

static bool otaPendingVerify = false;
static constexpr uint32_t OTA_CONFIRM_AFTER = 120000;  // stabil laufen mindestens 2 min
static constexpr uint32_t OTA_ROLLBACK_AFTER = 600000; // ohne Netz nach 10 min zurueck

static void checkOtaRollback()
{
    if (!otaPendingVerify)
        return;
    if (openknxNetwork.established() && millis() > OTA_CONFIRM_AFTER)
    {
        esp_ota_mark_app_valid_cancel_rollback();
        otaPendingVerify = false;
        logInfo("Main", "Neue Firmware bestaetigt (Rollback abgeschaltet)");
    }
    else if (millis() > OTA_ROLLBACK_AFTER)
    {
        logError("Main", "Neue Firmware ohne Netz: Rueckkehr zur vorherigen Firmware");
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
}

void setup()
{
    // Mit mehreren APs (Mesh, Roaming) alle Kanaele scannen und den AP mit dem besten Signal
    // nehmen statt des ersten gefundenen. Gilt fuer begin() in OFM-Network und fuer Reconnects.
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

    openknx.init(0); // Firmware-Revision
    openknx.addModule(1, openknxWeatherModule);
    openknx.addModule(7, openknxNetwork);
    openknx.setup();

    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY)
    {
        otaPendingVerify = true;
        logInfo("Main", "Neue Firmware noch nicht bestaetigt: Rollback aktiv, bis das Netz 2 min stabil laeuft");
    }
}

void loop()
{
    openknx.loop();
    checkOtaRollback();

    // Im Modem-Sleep empfaengt der ESP32 Multicast (KNX IP Routing) nur zu DTIM-Zeitpunkten,
    // die ETS laeuft dann in Zeitueberschreitungen. Gilt unabhaengig vom
    // Konfigurationszustand, damit auch die erste ETS-Programmierung gelingt.
    static bool wlanSparmodusAus = false;
    if (!wlanSparmodusAus && openknxNetwork.established())
    {
        WiFi.setSleep(false);
        wlanSparmodusAus = true;
        logInfo("Main", "WLAN-Energiesparmodus deaktiviert");
    }
}
