# WN90LP-KNX

OpenKNX-Firmware für die Wetterstation **Ecowitt WN90LP** am **LilyGO T-RSS3** (ESP32-S3, RS485).
Das Gerät arbeitet als **KNX-IP-Gerät über WLAN** und stellt die Messwerte als Kommunikationsobjekte
und optional per MQTT bereit.

## Hardware

- Ecowitt WN90LP (Modbus RTU, Adresse 0x90, 9600 Baud) an der RS485-Schnittstelle des T-RSS3
- optional Feinstaubsensor SDS011 an RXD/TXD der Stiftleiste (RXD des T-RSS3 an TXD des SDS011 und umgekehrt, 5 V)
- optional Temperatursensor DS18B20 an GPIO 11 (3V3, GND, 4,7 kΩ Pull-up von DATA nach 3V3)
- Prog-Taster GPIO 5, Prog-LED ist die WS2812 an GPIO 1

## Funktionen

- Messwerte der WN90LP: Temperatur, Luftfeuchte, Wind, Böe, Windrichtung, Luftdruck, Helligkeit, UV-Index, Regen
- Berechnet: absolute Feuchte, Taupunkt, Frostpunkt, Humidex, Hitzeindex, Windchill, Discomfort-Index,
  Beaufort, Luftdruck auf Meereshöhe, Luftdrucktendenz 1 h / 3 h
- Regen: Gesamtmenge, Intensität (mm/h), Mengen der letzten 5 min / 1 h / 6 h / 24 h / 48 h / 72 h, heute,
  „Regen“ (ja/nein, mit Mindestmenge im Zeitfenster gegen Fehlauslösung).
  Gesamtmenge, Tageswert und Stundenwerte bleiben über Neustart und Update erhalten.
- Feinstaub PM2,5 / PM10, auch feuchtekorrigiert
- Optional verwenden die Berechnungen (Taupunkt, Behaglichkeit, Luftdruck auf Meereshöhe …) die 1-Wire-Temperatur
  statt der Temperatur der Wetterstation; bei Ausfall des 1-Wire-Sensors automatisch wieder die Wetterstation.
- Prognose: Wettertendenz (-2..+2 aus der Luftdruckänderung über 3 h), Zambretti-Prognose (1–26) und
  Prognosetext (DPT 16), Wettersymbol für Displays (z. B. Tantron 4"): 0 sonnig, 1 bewölkt, 2 Schauer,
  3 starker Regen, 4 Gewitter (Schätzung aus Regen und Böen), 5 hoher UV-Index.
  Sonnig/bewölkt nutzt den Sonnenstand; dafür in der ETS den Gerätestandort eintragen und eine Uhrzeit bereitstellen.
- Wind: 10-Minuten-Mittel und stärkste Böe der letzten 10 Minuten; drei Windalarm-Stufen (DPT 1.005) mit
  Schwelle, Auslöse- und Rücknahmeverzögerung für Markisen/Jalousien (bei Störung der Wetterstation: Alarm)
- Sonnenstand (Azimut/Elevation) und Tag/Nacht aus der Helligkeit mit Hysterese und Verzögerung
- Minimum/Maximum: Temperatur und stärkste Böe heute und gestern (remanent)
- Warnungen: Frost und Glättegefahr
- Sommer/Winter-Umschaltung: gleitendes Mittel der Tagesmitteltemperaturen (DIN/Mannheimer Stunden
  (T7 + T14 + 2·T21|22) / 4 oder Mittel aller Messwerte) über 1–7 Tage gegen eine Schwelle (Standard 16 °C),
  wahlweise als DPT 1.100 Heizen/Kühlen; dazu mittlere Außentemperatur, Tagesmittel gestern und die Kenntage
  Sommertag, heißer Tag, Frosttag, Eistag
- Strahlung (geschätzt aus der Helligkeit): Globalstrahlung, Bewölkungsgrad, Einstrahlung heute/gestern
- PV-Ertrag (geschätzt): Leistung sowie Ertrag heute/gestern aus kWp, Neigung, Ausrichtung und Performance Ratio
- Bis zu 6 Fassaden: „Sonne auf Fassade“ mit Schwelle und Verzögerungen, Einstrahlung auf die Fassade und
  Profilwinkel für die Lamellennachführung (Direkt-/Diffusaufteilung nach Erbs, Sonnenstand aus OpenKNX)
- Sonnenzeiten (nach dem EDOMI-LBS „Sonnenstand“): Auf-/Untergang, Höchststand, bürgerliche/nautische/astronomische
  Dämmerung (DPT 10.001), Azimut bei Auf-/Untergang, Elevation im Höchststand, Tageslänge, Tagesfortschritt,
  Sonne steigt/fällt, Tag/Nacht nach Sonnenstand (Versatz oder Elevationsschwellen) und je Dämmerungsstufe,
  Strahlung bei klarem Himmel, Sonnenscheindauer heute/gestern (WMO, > 120 W/m²)
- Mond: Phase (Nummer und Text), Beleuchtung, Alter, Azimut/Elevation, Auf-/Untergang, nächster Voll- und Neumond (DPT 11.001)
- Zeitpunkte wahlweise als DPT 10.001 bzw. 11.001, DPT 19.001 (Datum und Uhrzeit) oder Unix-Zeit (DPT 12.001);
  in MQTT und InfluxDB zusätzlich immer als `<feld>_unix`
- Home Assistant MQTT Discovery (alle aktiven Messwerte als Sensoren)
- Je Messwert in der ETS: aktiv, Datenpunkttyp, zyklisch senden, senden bei absoluter/relativer Änderung
- Störungsobjekte je Sensor, Heartbeat, Uhrzeit/Datum (KNX oder NTP) aus OpenKNX
- WLAN, IP-Einstellungen, mDNS, NTP, OTA, Webserver mit Web-Konsole und MQTT aus OFM-Network

## Projekt einrichten

```
python3 restore/restore.py          # OpenKNX-Bibliotheken nach lib/ klonen
python3 scripts/generate_ets.py     # src/WN90LP.share.xml und src/ValueTable.h erzeugen
OpenKNXproducer create --Debug -h include/knxprod.h -o <ABSOLUTER_PFAD>/src/WN90LP.knxprod src/WN90LP
```

- OpenKNXproducer ≥ 4.3.12. Unter macOS/Linux entstehen nur `include/knxprod.h` und die Debug-XML;
  die `.knxprod` für den ETS-Import erzeugt OpenKNXproducer nur unter **Windows mit installierter ETS**.
- `knxprod.h` und `.knxprod` müssen aus demselben XML-Stand stammen.
- Messwerte, Texte und Standardwerte der ETS-Applikation stehen in `scripts/generate_ets.py`;
  `src/WN90LP.share.xml` und `src/ValueTable.h` nicht von Hand ändern.

## Firmware bauen und flashen

| Umgebung | Zweck |
|---|---|
| `develop_WN90` | Entwicklung über USB, ausführliche Logs |
| `release_WN90` | Betrieb über USB, mit Watchdog |
| `ota_WN90` | Release per OTA (ArduinoOTA, Port 3232), IP in `platformio.custom.ini` |
| `ota_develop_WN90` | Entwicklung per OTA |

Geräte, die bereits die OpenKNX-Partitionstabelle haben (alte Firmware ab Commit 19d6929), lassen sich
direkt per OTA umstellen; die alte Firmware nimmt ArduinoOTA auf Port 3232 an. Die neue Firmware startet
danach unkonfiguriert und verbindet sich mit den im ESP32 gespeicherten WLAN-Zugangsdaten.

Ältere Geräte mit anderer Partitionstabelle einmal über USB löschen und flashen:

```
pio run -e release_WN90 -t erase
pio run -e release_WN90 -t upload
```

## Inbetriebnahme

1. **WLAN** über die serielle Konsole einrichten: `wifi <SSID> <Passwort>` (Status mit `net`),
   alternativ mit dem WLAN-Assistenten in der ETS.
2. Applikation `WN90LP Wetterstation` (OpenKnxId 0xFE, Applikation 0x02) in die ETS importieren,
   Gerät einfügen, Prog-Taster drücken, physikalische Adresse und Applikation programmieren.
3. In der ETS auf der Netzwerkseite Webserver und bei Bedarf MQTT aktivieren.

Nach einer Änderung der Applikationsversion immer vollständig programmieren (nicht „partiell“).

## Logikmodul

Das OpenKNX-Logikmodul (OFM-LogicModule) stellt 30 Logikkanäle bereit (KOs 300–389): UND/ODER, Schwellwerte,
Verzögerungen, Treppenlicht, Zeitschaltuhren mit Sonnenauf-/untergang und Feiertagen, Formeln. Ein Logikeingang kann
direkt ein KO der Wetterstation nutzen („Bestehendes KO“, Nummern in [doc/KO-Liste.md](doc/KO-Liste.md)).
Voraussetzungen: Der Wert ist in der ETS aktiv, der DPT im Logikkanal passt, und der Wert wird gesendet
(zyklisch oder bei Änderung) – nur gesendete Werte erreichen die Logik. Dokumentation:
`lib/OFM-LogicModule/doc/Applikationsbeschreibung-Logik.md`. Konsolenbefehl: `logic`.

## InfluxDB

Optional schreibt das Gerät alle gültigen Messwerte selbst per HTTP in eine InfluxDB 1.x (`/write`, ohne TLS);
Einstellungen in der ETS unter Wetterstation → InfluxDB. Das Schema entspricht dem früheren Node-RED-Flow:
Measurement `weather`, Tags `floor`, `room` und `sensor` (`wn90`, `wn90_1wire`, `wn90_sds011`), Feldnamen wie
`temp`, `brightness`, `pressureTrend1`, `raincounter_1h` …, alle Zahlen als Gleitkommawerte, Zeitstempel vom Server.
Zustand und Fehler zeigen `wx sensors` und die Statusseite.

## Statusseite

Bei aktivem Webserver (ETS, Netzwerkseite) zeigt `http://<IP>/weather` (Menüpunkt „Wetter“) alle Messwerte
gruppiert nach ETS-Seiten mit aktuellem und zuletzt gesendetem Wert sowie den Zustand der Sensoren, der Uhrzeit
und von MQTT. Die Seite aktualisiert sich alle 5 Sekunden; die Rohdaten liefert `/weather.json`.

## Konsole

Die Befehle funktionieren seriell und in der Web-Konsole (`http://<IP>/console`):

| Befehl | Ausgabe |
|---|---|
| `wx values` | alle Messwerte, aktiv in der ETS, zuletzt gesendeter Wert |
| `wx sensors` | Zustand WN90LP (Modbus), SDS011, 1-Wire, Uhrzeit |
| `wx raw` | Rohwerte der Modbus-Register 0x0165–0x016E |
| `wx rain` | Regenstatistik |
| `wx pressure` | Luftdruckverlauf und Tendenz |
| `wx forecast` | Zambretti-Prognose und Wettersymbol mit Begründung |
| `wx minmax` | Minimum/Maximum, Windalarme, Dämmerung, Frost/Glätte |
| `wx season` | Sommer/Winter, Messtermine und Tagesmittel, Kenntage |
| `wx solar` | Globalstrahlung, Sonnenstand, PV-Schätzung, Fassaden |
| `wx astro` | Sonnenzeiten, Dämmerung und Mond |
| `wx 1w scan` | 1-Wire-Bus durchsuchen |

Über das Diagnose-KO: `wx t`, `wx h`, `wx w`, `wx p`, `wx r`.

## MQTT

Bei aktivem MQTT (ETS, Netzwerkseite) sendet das Gerät im eingestellten Intervall ein JSON-Objekt an
`openknx/<Präfix>/state` mit den gültigen Messwerten (Feldnamen wie `temperature`, `humidity`,
`windspeed`, `pressure`, `raincounter`, `rain_today`, `pm25` …) sowie `wifi_rssi` und `uptime`.

Mit „Home Assistant MQTT Discovery“ (ETS, Wetterstation → Allgemein) meldet das Gerät nach jedem Verbinden
alle aktiven Messwerte unter `homeassistant/sensor|binary_sensor/<Knoten>/<Feld>/config` an (retained).
