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
- **InfluxDB** (`InfluxWriter.*`): HTTP-POST in eigener FreeRTOS-Task (6 KB Stack), Daten per Mutex übergeben, kein Logging aus der Task (Fehlertext wird gespeichert, die Hauptschleife loggt). Schema und Feldnamen (`influx=` in generate_ets.py) entsprechen dem früheren Node-RED-Flow; immer Float-Felder.
- **Allgemeine Parameter** reichen inzwischen bis Byte 367 (InfluxDB ab 160).
- **Astronomie** (`Astro.*`): Sonne nach NOAA (Zeiten ±1 min), Mond nach gekürzten Meeus-Reihen (Phasen ±10 min, gegen 2026 geprüft). Uhrzeiten als Sekunden seit Mitternacht (WxFmt::Time, DPT 10.001 mit Index 1!), Datum als Tage seit 2000 (WxFmt::Date, DPT 11.001: Jahr vierstellig, Monat 1–12). DPT 7.006 wird über die 7.001-Kodierung gesendet.
- **DWD-Warnungen** (`DwdWarnings.*`, einzige Internet-Funktion): HTTPS-GET in eigener Task (12 KB Stack) an `maps.dwd.de` WFS, Ebene `Warnungen_Gemeinden` (Filter `WARNCELLID`) bzw. `Warnungen_Landkreise` (Filter `GC_WARNCELLID`, Zellen 1xxxxxxxx), JSON per Datenstrom-Filter. Zertifikatsprüfung mit `src/DwdRootCa.h` (USERTrust RSA, ISRG X1, DigiCert G2; bei CA-Wechsel des DWD ergänzen). Flash: Develop-Build ~92 % – bei weiteren Funktionen Release-Build zum Testen nutzen.
- **Statusseite** (`WebStatus.*`): HTTP-Handler laufen auf dem ESP32 in der httpd-Task und lesen nur einen per Mutex geschützten JSON-Schnappschuss, den `WeatherModule::buildStatusJson()` in der Hauptschleife alle 2 s erstellt (nur während Abfragen). Seite als ASCII-Rohstring, Sonderzeichen als HTML-/JS-Escapes, Daten nur per `textContent`.
- **Neue Werte immer hinten in VALUES anfügen**, damit KO-Nummern bestehender Installationen gleich bleiben.
- **KO-Bereiche:** Wetterstation KO 21–194 (inkl. Eingängen) (KoSingleOffset 20), Logikmodul KO 300–389 (`LOG`, KoOffset 300). Wetter-KOs dürfen 299 nicht überschreiten. `doc/KO-Liste.md` wird vom Generator erzeugt.
- **Logikmodul** (OFM-LogicModule 4.4.1, Modul-Id 3): Eingänge „bestehendes KO“ erhalten nur gesendete Werte (`value()`), nicht `valueNoSend()`.
- **KO-Nummern** stehen dauerhaft in `scripts/ko_numbers.json` (relativ zu KoSingleOffset 20), **Parameter-Ids** dauerhaft in `scripts/param_ids.json` (die ETS ordnet beim Applikations-Update darüber zu). Nie von Hand ändern oder löschen; neue Parameter bekommen automatisch die nächste freie Id. Datei immer mit committen.
- **Versionen synchron halten:** `%WX_ApplicationVersion%` und `%WX_VerifyVersion%` (WN90LP.conf.xml), `version` in library.json (Major.Minor) und `WX_FIRMWARE_VERSION` (WeatherModule.h). `%WX_VerifyVersion%` ist die Modulversion im Konfigurationstransfer; OpenKNXproducer prüft sie gegen library.json.
- **ETS 5.7.4:** Beim Update 0.10 → 0.14 setzte die ETS trotz identischer Parameter-/KO-Ids alle Werte zurück (Ursache offen). Vor Updates Projekt sichern bzw. Konfigurationstransfer-Export anlegen (Import mit Prüfstufe „lockerer“).
- **ETS-Update ohne Neuparametrierung:** Applikationsversion erhöhen (`%WX_ApplicationVersion%`), alte Version in `%WX_ReplacesVersions%`, Firmware mit dem neuen `knxprod.h` bauen und flashen, in der ETS „Applikation aktualisieren“ und vollständig programmieren.
- **Allgemeine Parameter** (GENERAL, 160 Byte): 0–36 allgemein, 37–63 Windalarme (3×9), 64–73 Sommer/Winter, 74–75 Strahlung, 80–87 PV, 88–159 Fassaden (6×12).
- **Bedingte Werte:** `cond` in generate_ets.py (z. B. Fassade n aktiv, Bit 0 am Blockanfang) blendet KOs aus; die Firmware prüft `condParam` vor jedem KO-Zugriff.
- **Strahlung:** Helligkeit/110 lx je W/m² × Korrekturfaktor; Erbs-Aufteilung, isotrope Diffusstrahlung, Albedo (`Solar.h`).
- **DPT-Faktor < 0** bei DPT 1 bedeutet invertiert (Sommer/Winter als 1.100 Heizen/Kühlen).
- **Flash-Version 7** im Modul: [Version][Regen][Min/Max 28][Season][Strahlung/PV 16][Sonnenschein 8][Bewässerung 16][Gradtage 38][Luftdruck 58]; ältere Versionen werden noch gelesen. Vor jedem gewollten Neustart (ETS, OTA, Konsole) erzwingt `processBeforeRestart()` das Speichern.
- **Windalarm** ist fail-safe: bei Störung der WN90LP ausgelöst.
- **HA Discovery** (`HaDiscovery.*`): eine Nachricht je aktivem Wert alle 100 ms (MQTT-Warteschlange max. 32),
  erneut nach jedem Verbindungsaufbau; `OPENKNX_MQTT_TXBUF_MAX=4096` für das große JSON.

## Regeln

- Kein `delay()`, alles nicht-blockierend mit `delayCheck()`.
- Formatierung gemäß `.clang-format` (Allman, 4 Leerzeichen).
- Keine Zugangsdaten im Quelltext oder in Konfigurationsdateien.
- Vor Änderungen an OpenKNX-Schnittstellen die Header in `lib/` prüfen (`lib/OGM-Common/AGENTS.md`, `lib/OFM-Network/AGENTS.md`).
