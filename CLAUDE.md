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
restore/patches/<lib>/                    Lokale Patches, von restore.py angewendet (knx: leere UDP-Pakete)
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
  Minutenstände (5 min/1 h) mit Unixzeit ab Flash-Version 10: nach dem Laden steht der Ring, bis die Uhrzeit gültig ist, dann wird die Ausfallzeit nachgetragen (> 60 min oder 3 min ohne Uhrzeit: verworfen).
  Zusätzliches Speichern ab 1 mm Regen seit dem letzten Speichern, höchstens alle 10 min (sonst nur stündlich).
  Bei Layoutänderung `FLASH_VERSION` in `WeatherModule.cpp`/`RainStatistics.h` erhöhen.
- **WLAN:** `main.cpp` setzt Scan aller Kanäle/stärkster AP vor `openknx.init` und `WiFi.setSleep(false)`
  nach Verbindungsaufbau (sonst Multicast-Probleme bei der ETS-Programmierung).
  WLAN-Wächter in `main.cpp`: nach 5 min ohne Netz `openknxNetwork.resetNetwork()` (alle 5 min), nach 30 min Neustart;
  scharf erst nach der ersten Verbindung (der Auto-Reconnect des Arduino-Kerns gab nach einer Trennung auf).
- **MQTT-Themen:** `openknx/<Präfix>/state` (Messwerte, im Intervall), `state/calc`, `state/astro`, `state/stats` (nur bei Änderung per FNV-Prüfsumme, spätestens alle 15 min), alle retained, je Hauptschleifendurchlauf höchstens ein Thema. Zuordnung der ETS-Gruppen in `MQTT_TOPICS`/`GROUP_TOPIC` (generate_ets.py, neue Gruppen dort eintragen) (nach Inhalt, ganze ETS-Gruppen: astro = alles zu Sonne/Mond), Feld `mqttTopic` in `WX_VALUES`; HA-Discovery nutzt dasselbe Thema.
- **InfluxDB** (`InfluxWriter.*`): HTTP-POST in eigener FreeRTOS-Task (6 KB Stack), Daten per Mutex übergeben, kein Logging aus der Task (Fehlertext wird gespeichert, die Hauptschleife loggt). Schema und Feldnamen (`influx=` in generate_ets.py) entsprechen dem früheren Node-RED-Flow; immer Float-Felder.
- **Allgemeine Parameter** (GENERAL_SIZE 472): InfluxDB ab 160, Räume 1–3 ab 384, Räume 4–6 ab 448 (je 8 Byte); frei nur Byte 447.
- **Astronomie** (`Astro.*`): Sonne nach NOAA (Zeiten ±1 min), Mond nach gekürzten Meeus-Reihen (Phasen ±10 min, gegen 2026 geprüft). Uhrzeiten als Sekunden seit Mitternacht (WxFmt::Time, DPT 10.001 mit Index 1!), Datum als Tage seit 2000 (WxFmt::Date, DPT 11.001: Jahr vierstellig, Monat 1–12). DPT 7.006 wird über die 7.001-Kodierung gesendet.
- **DWD-Warnungen** (`DwdWarnings.*`, einzige Internet-Funktion): HTTPS-GET in eigener Task (12 KB Stack) an `maps.dwd.de` WFS, Ebene `Warnungen_Gemeinden` (Filter `WARNCELLID`) bzw. `Warnungen_Landkreise` (Filter `GC_WARNCELLID`, Zellen 1xxxxxxxx), JSON per Datenstrom-Filter. Unbedingt `http.useHTTP10(true)`: der Server antwortet sonst chunked, `getStream()` liefert die Längenangaben mit, und ArduinoJson liest z. B. „263“ als gültiges Dokument (bis 1.2.3: „ok“, aber nie eine Warnung). Antworten ohne `features`-Array gelten als Fehler. Vor dem Abruf eigene DNS-Prüfung (HTTPClient meldet auch DNS-Fehler als „connection refused“, -1); bei -1 stehen Dauer, IP, Ursache aus `NetworkClientSecure::lastError()` (-1 = TCP, sonst mbedTLS-Code des Handshakes) sowie freier Speicher und größter Block vor dem Aufbau im Fehlertext (`Result::error`); TLS braucht je ca. 17 KB Ein-/Ausgabepuffer am Stück im internen RAM; bei zerstückeltem Speicher scheiterte der Handshake („Memory allocation failed“, -0x7F00, bei 84 KB frei, 30 KB größtem Block), daher belegt `reserve()` beim Start zwei Blöcke à 17 KB und gibt sie nur während des Abrufs frei; fehlende Blöcke versucht `loop()` jede Minute neu (1.5.7 mit einem 36-KB-Block konnte ihn nach einem Abruf nicht wieder belegen). Ist kein Block ≥ 16,5 KB frei, entfällt der Abruf (Code -201). `wx sensors` zeigt den Zustand. Nach einem Fehler bis zu 3 Wiederholungen im Minutenabstand. Alle Warnungen (max. 8, Texte max. 600 Byte) stehen in `warnings()`/`warningsSeq()`; `publishDwd()` sendet sie retained an `openknx/<Präfix>/dwd` (höchstens `OPENKNX_MQTT_TXBUF_MAX` − 300 Byte), die Statusseite hängt sie als eigene Stücke (`dwd_list`) an. Zertifikatsprüfung mit `src/DwdRootCa.h` (USERTrust RSA, ISRG X1, DigiCert G2; bei CA-Wechsel des DWD ergänzen). Flash: Develop-Build ~92 % – bei weiteren Funktionen Release-Build zum Testen nutzen.
- **Statusseite** (`WebStatus.*`): HTTP-Handler laufen auf dem ESP32 in der httpd-Task und lesen nur einen Schnappschuss (`shared_ptr` auf Stücke von ca. 1,5 KB), den `WeatherModule::buildStatusSnapshot()` in der Hauptschleife alle 2 s erstellt (nur während Abfragen); gesendet wird chunked über `sendStream()`. Kein großer zusammenhängender Puffer: das JSON hat über 19 KB, mit einem Block scheiterte die Erstellung bei fragmentiertem Speicher und die Seite blieb auf einem alten Stand stehen. Seite als ASCII-Rohstring, Sonderzeichen als HTML-/JS-Escapes, Daten nur per `textContent`.
- **Dokumentation:** Jeder Messwert braucht einen Hinweistext (`info=` oder `INFO`-Tabelle in generate_ets.py, sonst bricht der Generator ab). `doc/Applikationsbeschreibung.md`: Text von Hand, Tabellen zwischen `<!-- GEN:<Seite> -->` und `<!-- /GEN -->` erzeugt der Generator; neue allgemeine Parameter in `PAGE_PARAMS` eintragen (wird geprüft).
- **Neue Werte immer hinten in VALUES anfügen**, damit KO-Nummern bestehender Installationen gleich bleiben.
- **KO-Bereiche:** Wetterstation KO 21–271 (inkl. Eingängen) (KoSingleOffset 20), Logikmodul KO 300–389 (`LOG`, KoOffset 300). Wetter-KOs dürfen 299 nicht überschreiten. `doc/KO-Liste.md` wird vom Generator erzeugt.
- **Prog-LED** (`src/RgbLed.h`): eigene WS2812-Klasse über `rgbLedWrite()` an GPIO 1. Das Logikmodul ruft in `setup()` `analogRead(A0)` auf (A0 = GPIO 1 beim S3); `main.cpp` holt den Pin danach mit `reattach()` zurück. Diagnose: `wx led <pin> <r> <g> <b>`. Statusanzeige: `WeatherModule::updateStatusLed()` setzt per `RgbLed::status()` ein Muster; Prog-Modus und OpenKNX-Fehlercodes haben Vorrang. ETS-Parameter LedMode/LedBrightness/LedHeartbeat (Byte 443-445).
- **Logikmodul** (OFM-LogicModule 4.4.1, Modul-Id 3): Eingänge „bestehendes KO“ erhalten nur gesendete Werte (`value()`), nicht `valueNoSend()`.
- **KO-Nummern** stehen dauerhaft in `scripts/ko_numbers.json` (relativ zu KoSingleOffset 20), **Parameter-Ids** dauerhaft in `scripts/param_ids.json` (die ETS ordnet beim Applikations-Update darüber zu). Nie von Hand ändern oder löschen; neue Parameter bekommen automatisch die nächste freie Id. Datei immer mit committen.
- **Versionen synchron halten:** `%WX_ApplicationVersion%` und `%WX_VerifyVersion%` (WN90LP.conf.xml), `version` in library.json und `WX_FIRMWARE_VERSION` (WeatherModule.h, Major.Minor fest, Revision aus `WX_FIRMWARE_REVISION`, die auch an `openknx.init()` geht; bei jeder Firmware-Änderung erhöhen, bei neuer Applikationsversion auf 0). Applikationsversion: 4 Bit Major, 4 Bit Minor (nach 0.15 kommt 1.0). Die OpenKNX-Anzeige hängt den Git-Hash an (Stand des letzten Commits beim Bauen). `%WX_VerifyVersion%` ist die Modulversion im Konfigurationstransfer; OpenKNXproducer prüft sie gegen library.json.
- **ETS-Update richtig durchführen** (`lib/OGM-Common/doc/Update-ETS-Application.md`): neue Version irgendwo ins Projekt einfügen, dann in den Eigenschaften des bestehenden Geräts den **Update-Button** nutzen. Die Versions-Auswahlliste setzt alle Parameter und GA-Zuordnungen zurück (so geschehen beim Update 0.10 → 0.14 mit ETS 5.7.4). Vorher Projekt sichern bzw. Konfigurationstransfer-Export anlegen.
- **ETS-Update ohne Neuparametrierung:** Applikationsversion erhöhen (`%WX_ApplicationVersion%`), alte Version in `%WX_ReplacesVersions%`, Firmware mit dem neuen `knxprod.h` bauen und flashen, in der ETS „Applikation aktualisieren“ und vollständig programmieren.
- **Allgemeine Parameter** (GENERAL, 160 Byte): 0–36 allgemein, 37–63 Windalarme (3×9), 64–73 Sommer/Winter, 74–75 Strahlung, 80–87 PV, 88–159 Fassaden (6×12).
- **Bedingte Werte:** `cond` in generate_ets.py (z. B. Fassade n aktiv, Bit 0 am Blockanfang) blendet KOs aus; die Firmware prüft `condParam` vor jedem KO-Zugriff.
- **Räume:** 6 (`ROOMS` in generate_ets.py und WeatherModule.h). WxId-Blöcke je Raum sind zusammenhängend (Lüftung 4, Behaglichkeit 7), KO-Nummern der Räume 4–6 liegen hinten (ko_numbers.json). WxId ist uint8_t: höchstens 255 Werte (aktuell 235).
- **Behaglichkeit nach Frank** (`Comfort.h`, Felder aus dem EDOMI-Baustein 0.6): je Raum 7 Werte (Stufe, Text = Stufe·16 + Grund-Bits, Abweichung, zu feucht/trocken/kalt/warm), berechnet in `updateVentilation()` aus den Raumeingängen.
- **Strahlung:** Helligkeit/110 lx je W/m² × Korrekturfaktor; Erbs-Aufteilung, isotrope Diffusstrahlung, Albedo (`Solar.h`).
- **DPT-Faktor < 0** bei DPT 1 bedeutet invertiert (Sommer/Winter als 1.100 Heizen/Kühlen).
- **Flash-Version 10** im Modul: [Version][Regen][Min/Max 28][Season][Strahlung/PV 16][Sonnenschein 8][Bewässerung 16][Gradtage 38][Luftdruck 58][Regen gestern/Woche/Monat/Jahr 40][Feinstaub 256][Regen-Minutenstände 250]; ältere Versionen werden noch gelesen. Grenzen je Version als `FLASH_END_Vn` (kumuliert) in WeatherModule.cpp – neue Blöcke nur hinten anfügen und eine neue Grenze ergänzen. Vor jedem gewollten Neustart (ETS, OTA, Konsole) erzwingt `processBeforeRestart()` das Speichern.
- **Absturzdiagnose ohne USB** (`Diag.cpp`): `set_arduino_panic_handler()` sichert Ursache, Task und bis zu 16 Backtrace-Adressen im RTC-Speicher; nach dem Neustart im Log und bei `wx sensors`. Zusätzlich sichert `loopTick()` jede Sekunde freien Speicher, größten Block und die kleinste Stack-Reserve der Tasks (loopTask, DwdWarnings, InfluxWriter, httpd) im RTC-Speicher; nach einem Absturz zeigt `wx sensors` diesen Stand. Fehlt der Backtrace bei einer Panic, ist der Handler nicht erreicht worden (der Kern läuft vorher den Stack ab, bei beschädigtem Stack scheitert das). Auswerten mit `xtensa-esp32s3-elf-addr2line -pfiaC -e <firmware.elf> <Adressen>` – dafür die `firmware.elf` jedes ausgelieferten Builds unter `release/<version>.elf` aufheben (nicht versioniert).
- **Luftdruckverlauf** (`PressureTrend`): Nach einem Neustart werden Lücken bis 3 h linear aufgefüllt, längere Ausfälle beginnen neu.
- **Feinstaub-Auswertung** (`ParticleStats.*`): feuchtekorrigierte Werte (ohne Luftfeuchte Rohwerte); 1 h aus sechs 10-min-Abschnitten, 24 h aus 24 Stundenmitteln (gültig ab 18), Tagesmittel nach Ortszeit, Jahresmittel aus Tagesmitteln, Überschreitungstage nach ETS-Parameter PmLimit (Byte 446). UBA-Index aus dem 24-h-Mittel, Ampel 0/1/2. WxFmt::AqiText für den Indextext.
- **Windalarm** ist fail-safe: bei Störung der WN90LP ausgelöst.
- **Zambretti** nutzt die mittlere Windrichtung über 10 min (`WindStats::averageDir()`, Einheitsvektoren, NAN bei Windstille < 1 m/s oder Vektorlänge < 0,3).
- **Windchill** rechnet wie die Wetterdienste mit dem 10-min-Windmittel (`WindAvg10`), nicht mit dem Momentanwind (sonst Zittern an der 5-km/h-Grenze). **Gefühlte Temperatur** (`FeelsLike`, KO 211): bis 10 °C Windchill, ab 27 °C Hitzeindex, sonst Lufttemperatur.
- **MQTT-Warteschlange** auf 8 begrenzt (`OPENKNX_MQTT_MAX_QUEUE`): bei stockender Verbindung behält der Client jede unversendete Nachricht; 32 Status-JSON à ca. 5 KB brauchten den Speicher auf (1.6.1/1.6.2: Abstürze bei 10 KB frei). Das Status-JSON entfällt unter 30 KB freiem Speicher (`MQTT_MIN_FREE_HEAP`).
- **HA Discovery** (`HaDiscovery.*`): eine Nachricht je aktivem Wert alle 100 ms (bei voller Warteschlange erneut versucht),
  erneut nach jedem Verbindungsaufbau; `OPENKNX_MQTT_TXBUF_MAX=8192` für das große JSON (Status mit allen Werten lag bei 1.5.1 schon bei 3,8 KB; zu große Nachrichten verwirft der Client mit „message too large“).
- **Leere UDP-Pakete an `224.0.23.12:3671`** (z. B. Tasmota mit aktivem KNX) machten das Gerät dauerhaft
  taub: Senden geht, Empfang und ETS-Programmierung nicht, auch nicht nach Neustart. Ursache:
  `NetworkUDP::parsePacket()` (Arduino-Core 3.x) liest nur bei `FIONREAD > 0`; leere Datagramme bleiben
  liegen und füllen die lwIP-Warteschlange (6 Plätze). Gelöst durch
  `restore/patches/knx/esp32-leere-udp-pakete.patch` (eigener Socket mit `recvfrom()`), den `restore.py`
  automatisch anwendet. Bei Update von `lib/knx` prüfen, ob der Patch noch passt.

## Regeln

- Kein `delay()`, alles nicht-blockierend mit `delayCheck()`.
- `delayCheck(last, dauer)` ist ein Makro (`millis() - last >= dauer`) ohne Klammern: als Dauer nie einen Ausdruck mit `?:`, `|`, `&` o. ä. übergeben, sondern vorher in eine Variable rechnen (1.5.3/1.5.4: DWD-Abruf in Dauerschleife, Speichermangel, Abstürze).
- Formatierung gemäß `.clang-format` (Allman, 4 Leerzeichen).
- Keine Zugangsdaten im Quelltext oder in Konfigurationsdateien.
- Vor Änderungen an OpenKNX-Schnittstellen die Header in `lib/` prüfen (`lib/OGM-Common/AGENTS.md`, `lib/OFM-Network/AGENTS.md`).
