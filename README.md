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
- Regen: Gesamtmenge, Mengen der letzten 5 min / 1 h / 6 h / 24 h / 48 h / 72 h, heute, „Regen“ (ja/nein).
  Gesamtmenge, Tageswert und Stundenwerte bleiben über Neustart und Update erhalten.
- Feinstaub PM2,5 / PM10, auch feuchtekorrigiert
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

Beim Umstieg von der alten Firmware (andere Partitionstabelle) einmal über USB löschen und flashen:

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

## Konsole

Die Befehle funktionieren seriell und in der Web-Konsole (`http://<IP>/console`):

| Befehl | Ausgabe |
|---|---|
| `wx values` | alle Messwerte, aktiv in der ETS, zuletzt gesendeter Wert |
| `wx sensors` | Zustand WN90LP (Modbus), SDS011, 1-Wire, Uhrzeit |
| `wx raw` | Rohwerte der Modbus-Register 0x0165–0x016E |
| `wx rain` | Regenstatistik |
| `wx pressure` | Luftdruckverlauf und Tendenz |
| `wx 1w scan` | 1-Wire-Bus durchsuchen |

Über das Diagnose-KO: `wx t`, `wx h`, `wx w`, `wx p`, `wx r`.

## MQTT

Bei aktivem MQTT (ETS, Netzwerkseite) sendet das Gerät im eingestellten Intervall ein JSON-Objekt an
`openknx/<Präfix>/state` mit den gültigen Messwerten (Feldnamen wie `temperature`, `humidity`,
`windspeed`, `pressure`, `raincounter`, `rain_today`, `pm25` …) sowie `wifi_rssi` und `uptime`.
