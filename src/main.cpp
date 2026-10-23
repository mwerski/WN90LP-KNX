#include "Logic.h"
#include "NetworkModule.h"
#include "OpenKNX.h"
#include "WeatherModule.h"
#include <WiFi.h>
#include "Diag.h"
#include "OtaGuard.h"
#include "RgbLed.h"

void setup()
{
    // Mit mehreren APs (Mesh, Roaming) alle Kanaele scannen und den AP mit dem besten Signal
    // nehmen statt des ersten gefundenen. Gilt fuer begin() in OFM-Network und fuer Reconnects.
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

    // Prog-LED vor openknx.init() anmelden, dort werden die LEDs initialisiert
    RgbLed *progLed = new RgbLed(WX_LED_PIN, WX_LED_COLOR);
    openknx.leds.addLed(progLed, OpenKNX::Led::LED_TYPE_PROG);
    openknx.init(WX_FIRMWARE_REVISION);
    openknx.addModule(1, openknxWeatherModule);
    openknx.addModule(3, openknxLogic);
    openknx.addModule(7, openknxNetwork);
    openknx.setup();
    // Das Logikmodul hat in setup() GPIO 1 (A0) analog gelesen, die LED braucht den Pin zurueck
    progLed->reattach();

    OtaGuard::setup();
    Diag::setup();
}

void loop()
{
    Diag::loopTick();
    Diag::stage(Diag::Network);
    openknx.loop();
    OtaGuard::loop(openknxNetwork.established());

    // Im Modem-Sleep empfaengt der ESP32 Multicast (KNX IP Routing) nur zu DTIM-Zeitpunkten,
    // die ETS laeuft dann in Zeitueberschreitungen. Gilt unabhaengig vom
    // Konfigurationszustand, damit auch die erste ETS-Programmierung gelingt.
    static bool wlanSparmodusAus = false;
    const bool online = openknxNetwork.established();
    if (!wlanSparmodusAus && online)
    {
        WiFi.setSleep(false);
        wlanSparmodusAus = true;
        logInfo("Main", "WLAN-Energiesparmodus deaktiviert");
    }

    // WLAN-Waechter: Der automatische Reconnect des Arduino-Kerns gab nach einer Trennung mitunter auf
    // (Station 30 min offline, "net reset" half sofort). Erst nach der ersten Verbindung scharf, damit die
    // Einrichtung per Konsole nicht gestoert wird. 5 min offline: Adapter neu initialisieren (alle 5 min
    // wiederholen), 30 min offline: Neustart (Daten werden vorher gespeichert).
    static constexpr uint32_t RESET_AFTER_MS = 5 * 60000UL;
    static constexpr uint32_t RESTART_AFTER_MS = 30 * 60000UL;
    static bool armed = false;
    static uint32_t offlineSince = 0;
    static uint32_t lastReset = 0;
    if (online)
    {
        armed = true;
        offlineSince = 0;
        lastReset = 0;
    }
    else if (armed)
    {
        // Nach einer Neuinitialisierung muss der Energiesparmodus erneut abgeschaltet werden
        wlanSparmodusAus = false;
        if (offlineSince == 0)
            offlineSince = max<uint32_t>(millis(), 1);
        else if (delayCheck(offlineSince, RESTART_AFTER_MS))
        {
            logError("Main", "WLAN seit 30 min getrennt - Neustart");
            openknx.restart();
        }
        else if (delayCheck(offlineSince, RESET_AFTER_MS) && (lastReset == 0 || delayCheck(lastReset, RESET_AFTER_MS)))
        {
            logError("Main", "WLAN seit %lu min getrennt - Netzwerkadapter wird neu initialisiert", (millis() - offlineSince) / 60000);
            lastReset = max<uint32_t>(millis(), 1);
            openknxNetwork.resetNetwork();
        }
    }
}
