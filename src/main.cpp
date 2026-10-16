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
    openknx.leds.addLed(new RgbLed(WX_LED_PIN, WX_LED_COLOR), OpenKNX::Led::LED_TYPE_PROG);
    openknx.init(WX_FIRMWARE_REVISION);
    openknx.addModule(1, openknxWeatherModule);
    openknx.addModule(3, openknxLogic);
    openknx.addModule(7, openknxNetwork);
    openknx.setup();

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
    if (!wlanSparmodusAus && openknxNetwork.established())
    {
        WiFi.setSleep(false);
        wlanSparmodusAus = true;
        logInfo("Main", "WLAN-Energiesparmodus deaktiviert");
    }
}
