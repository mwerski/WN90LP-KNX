# WN90LP-KNX – Projektkontext für Claude

## Projekt

OpenKNX-Firmware für die Wetterstation Ecowitt WN90LP (Modbus RTU über RS485) am LilyGO T-RSS3.
Portierung der früheren Eigenbau-Firmware (WiFiManager, ElegantOTA, RemoteDebug, PubSubClient,
Kaenx-Creator-Applikation) auf OpenKNX nach dem Muster von `~/Documents/Projects/OAM-Zapfanlage`.
Stand vor der Umstellung: Tag `milestone-2026-10-15`.

- **Plattform:** ESP32-S3, 8 MB Flash, OpenKNX-Partitionstabelle `esp32_8mb.csv` (zwei OTA-Slots à 2,5 MB)
- **KNX-Medium:** ausschließlich KNX IP über WLAN (Maske 0x57B0, `KNX_IP_WIFI`)
- **Sprache:** Kommentare und Log-Texte deutsch, ohne Umlaute im Quelltext; ETS-Texte deutsch

## Verzeichnisstruktur

```
platformio.ini / platformio.custom.ini   Umgebungen develop/release/ota, Build-Flags, lib_deps
dependencies.txt, restore/restore.py     Gepinnte OpenKNX-Bibliotheken nach lib/ (nicht versioniert)
include/hardware.h                        Pins des T-RSS3 (DEVICE_WN90_TRSS3)
include/knxprod.h                         GENERIERT (OpenKNXproducer), nicht versioniert
scripts/generate_ets.py                   QUELLE der Messwerttabelle: erzeugt WN90LP.share.xml + ValueTable.h
src/WN90LP.xml / .conf.xml                ETS-Hauptdatei, Versionen
src/WN90LP.share.xml, src/ValueTable.h    GENERIERT durch generate_ets.py (versioniert, nicht von Hand ändern)
src/WeatherModule.*                       OpenKNX-Modul: Sensoren, Messwerte, Störungen, MQTT, Konsole, Flash
src/MeasuredValue.*                       Sendelogik je Messwert (zyklisch, abs./rel. Änderung, DPT-Auswahl)
src/Wn90Sensor.*                          Modbus nicht-blockierend (NonBlockingModbusMaster, DE-Pin per Callback)
src/Sds011Sensor.*                        eigener SDS011-Treiber (aktiver Modus, Rahmen-Parser)
src/OneWireSensor.*                       DS18B20 nicht-blockierend
src/RainStatistics.*, PressureTrend.*     Regenmengen/Zeitfenster, Luftdrucktendenz
src/WeatherCalc.h                         Formeln (Frostpunkt, Windchill, Beaufort, PM-Korrektur)
```

## Arbeitsabläufe

1. Messwert/ETS ändern: `scripts/generate_ets.py` anpassen → `python3 scripts/generate_ets.py`
   → `OpenKNXproducer create --Debug -h include/knxprod.h -o <ABS>/src/WN90LP.knxprod src/WN90LP`
   → bauen. `%WX_ApplicationVersion%` erhöhen und alte Version in `%WX_ReplacesVersions%` aufnehmen.
2. OpenKNXproducer liegt auf dem Mac unter `~/.local/bin/OpenKNXproducer`; die `.knxprod` entsteht nur unter Windows mit ETS.
3. PlatformIO: `~/.platformio/penv/bin/pio` verwenden (das systemweite `pio` 6.1.18 ist zu alt).

## Wichtige Details

- **Parameterlayout je Messwert** (11 Byte): +0 Bit0 aktiv, Bit1-2 DPT-Index, +1 uint16 Zyklus s,
  +3 float abs., +7 float rel. %. `MeasuredValue::init` liest relativ zu `WX_<Key>Active`.
- **DPT-Kodierung:** ETS-DPT (z. B. 9.030) und Kodierung für den knx-Stack (`WxDptOption.main/sub`) sind getrennt,
  weil der Stack nicht alle Untertypen kennt. `factor` rechnet um (hPa→Pa ×100, m/s→km/h ×3,6).
- **KO-Zugriff nur für aktive Messwerte:** Die ETS kürzt die GO-Tabelle hinter dem höchsten sichtbaren KO.
  Zugriff auf ausgeblendete KOs führt zum Absturz (siehe Fix vor dem Meilenstein).
- **Senden:** höchstens ein Telegramm alle 25 ms, reihum; erst nach der Startverzögerung.
- **Regen:** Register 0x016E (0,01 mm, 16 Bit, läuft über); Differenzen > 50 mm/2 s gelten als Zählerreset.
  Remanent (Flash-Version 1): Gesamt, Mitternachtsstand, Tag, 73 Stundenwerte mit Zeitstempel.
  Bei Layoutänderung `FLASH_VERSION` in `WeatherModule.cpp`/`RainStatistics.h` erhöhen.
- **WLAN:** `main.cpp` setzt Scan aller Kanäle/stärkster AP vor `openknx.init` und `WiFi.setSleep(false)`
  nach Verbindungsaufbau (sonst Multicast-Probleme bei der ETS-Programmierung).
- **MQTT-Thema:** `openknx/<Präfix>/state` (Präfix aus den NET-Parametern).

## Regeln

- Kein `delay()`, alles nicht-blockierend mit `delayCheck()`.
- Formatierung gemäß `.clang-format` (Allman, 4 Leerzeichen).
- Keine Zugangsdaten im Quelltext oder in Konfigurationsdateien.
- Vor Änderungen an OpenKNX-Schnittstellen die Header in `lib/` prüfen (`lib/OGM-Common/AGENTS.md`, `lib/OFM-Network/AGENTS.md`).
