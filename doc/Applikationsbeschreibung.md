# Applikationsbeschreibung WN90LP-KNX

OpenKNX-Applikation für die Wetterstation Ecowitt WN90LP am LilyGO T-RSS3 (KNX IP über WLAN).

Die Tabellen mit Parametern und Kommunikationsobjekten erzeugt `scripts/generate_ets.py` aus denselben Daten wie die
ETS-Applikation. Sie stimmen daher immer mit der ETS überein. Der Text dazwischen wird von Hand gepflegt. Die Spalte
„Standard“ nennt die Voreinstellung eines neu eingefügten Geräts.

## Inhalt

- [Grundlagen](#grundlagen)
- [Allgemein](#allgemein)
- [Datenexport (InfluxDB)](#datenexport-influxdb)
- [Temperatur](#temperatur) · [Luftfeuchte](#luftfeuchte) · [Behaglichkeit](#behaglichkeit)
- [Wind](#wind) · [Windalarm](#windalarm) · [Luftdruck](#luftdruck) · [Helligkeit und UV](#helligkeit-und-uv)
- [Regen](#regen) · [Feinstaub](#feinstaub) · [Prognose und Display](#prognose-und-display)
- [Sonne und Dämmerung](#sonne-und-dämmerung) · [Minimum und Maximum](#minimum-und-maximum) · [Warnungen](#warnungen)
- [Sommer/Winter und Kenntage](#sommerwinter-und-kenntage) · [Strahlung und PV](#strahlung-und-pv) · [Fassaden](#fassaden)
- [Sonnenzeiten](#sonnenzeiten) · [Mond](#mond) · [Lüftungsempfehlung und Raumklima](#lüftungsempfehlung-und-raumklima)
- [Bewässerung](#bewässerung) · [Gradtage](#gradtage) · [Wetterwarnungen (DWD-Kriterien)](#wetterwarnungen-dwd-kriterien)
- [DWD-Warnungen (Internet)](#dwd-warnungen-internet)

## Grundlagen

### Messwerte senden

Jeder Messwert hat in der ETS dieselben Einstellungen:

- **Aktiv:** Nur aktive Werte haben ein sichtbares Kommunikationsobjekt (KO) und werden gesendet. Inaktive Werte
  berechnet die Station trotzdem, sie erscheinen auf der Statusseite, per MQTT und in der InfluxDB.
- **Datenpunkttyp:** Bei manchen Werten wählbar, etwa 2 Byte (DPT 9) oder 4 Byte (DPT 14). DPT 14 ist genauer, DPT 9 löst
  große Werte nur grob auf.
- **Zyklisch senden:** in Sekunden, 0 = nie.
- **Senden bei Änderung:** um einen absoluten Betrag oder um einen Prozentsatz des zuletzt gesendeten Werts, 0 = nie.
  Beide Bedingungen gelten nebeneinander. Werte ohne diese Einstellungen (Schaltwerte, Texte, Zähler) werden bei jeder
  Änderung gesendet.

Ungültige Werte, etwa bei einer Sensorstörung oder fehlender Uhrzeit, sendet die Station nicht. Eine Leseanfrage auf ein
KO beantwortet sie immer mit dem aktuellen Wert.

Nach dem Start sendet die Station erst nach der OpenKNX-Startverzögerung. Danach sendet sie höchstens ein Telegramm je
„Mindestabstand zwischen zwei Telegrammen“ (Seite Allgemein), reihum über alle Werte.

### Voraussetzungen

- **Uhrzeit:** Tageswerte (heute/gestern), Sonnen- und Mondzeiten, Sommer/Winter und Gradtage brauchen eine gültige
  Uhrzeit. Sie kommt über die Uhrzeit-KOs von OpenKNX oder per NTP (Seite Netzwerk).
- **Gerätestandort:** Sonnenstand, Strahlungsaufteilung, Fassaden und Mond brauchen Breiten- und Längengrad. Beide stehen
  auf der OpenKNX-Seite Allgemein (Standort).
- **Höhe des Standorts:** für den Luftdruck auf Meereshöhe und die Zambretti-Prognose (Seite Allgemein dieser
  Applikation).

### Temperatur für Berechnungen

Ist ein 1-Wire-Sensor vorhanden, lässt sich wählen, ob abgeleitete Werte die Temperatur der WN90LP oder des 1-Wire-Sensors
verwenden. Betroffen sind Taupunkt, Frostpunkt, absolute Feuchte, Behaglichkeitswerte, Luftdruck auf Meereshöhe,
Minimum/Maximum und Sommer/Winter. Fällt der 1-Wire-Sensor aus, rechnet die Station mit der WN90LP weiter.

### Remanenz

Regenzähler und Regenstatistik, Minimum/Maximum, Tagessummen, Sommer/Winter, Bewässerung, Gradtage, Luftdruckverlauf und
Feinstaubstatistik speichert die Station im Flash. Sie bleiben über Neustarts und Updates erhalten. Vor jedem gewollten
Neustart (ETS, OTA, Konsole) speichert sie zusätzlich.

## Allgemein

Sensoren, Standort, Regenerkennung, Senderate, Status-LED und MQTT.

- **Feinstaubsensor SDS011:** Der Sensor misst im eingestellten Intervall jeweils etwa 30 s und schläft dazwischen. Das
  schont den Laser (Lebensdauer etwa 8000 Betriebsstunden).
- **Korrektur der Lufttemperatur:** Ausgleich, wenn die WN90LP durch Sonne oder Montage dauerhaft abweicht.
- **Regen:** Regen beginnt, wenn innerhalb des Zeitfensters die Mindestmenge fällt. Einzelne Zählerschritte, etwa durch Tau
  oder Erschütterungen, lösen so noch keinen Regen aus. Regen endet, wenn die eingestellte Zeit kein Niederschlag mehr
  gemessen wurde.
- **Status-LED:** Rot im Programmiermodus. Je nach Einstellung zusätzlich, nach Vorrang: OTA-Update (violett pulsierend),
  WLAN getrennt (blau blinkend), Sensorstörung (gelb blinkend), bei „alle Zustände“ auch ausgelöster Windalarm (orange)
  und das Lebenszeichen (kurz grün alle 10 s).
- **MQTT:** Broker und Themenpräfix stehen auf der Seite Netzwerk. Die Station sendet ein JSON-Objekt an
  `openknx/<Präfix>/state`. Gekürzte Texte (Prognose, Mond, Warnung, Behaglichkeit) kommen zusätzlich ungekürzt als `<feld>_long`. Mit Home Assistant Discovery legt Home Assistant
  alle aktiven Werte automatisch als Sensoren an.

Störungs-KOs: 1 = Sensor antwortet nicht. Der Windalarm wird bei einer Störung der WN90LP ausgelöst (fail-safe).

<!-- GEN:General -->
| Parameter | Standard | Bereich |
|---|---|---|
| Feinstaubsensor SDS011 vorhanden | nein | ja / nein |
| Messintervall Feinstaubsensor (0 = Dauerbetrieb) | 5 min | 0 … 30 min |
| Temperatursensor 1-Wire (DS18B20) vorhanden | nein | ja / nein |
| Temperatur für Berechnungen | Wetterstation | Wetterstation / 1-Wire-Sensor |
| Höhe des Standorts über Meeresspiegel | 0 m | -500 … 9000 m |
| Korrektur der Lufttemperatur | 0 K | -10 … 10 K |
| Regen beginnt bei Niederschlag innerhalb von | 10 min | 1 … 60 min |
| von mindestens | 0,1 mm | 0,01 … 10 mm |
| Regen endet, wenn so lange kein Niederschlag gemessen wurde | 15 min | 1 … 240 min |
| Mindestabstand zwischen zwei Telegrammen | 100 ms | 20 … 1000 ms |
| Statusanzeige | alle Zustände | nur Programmiermodus / Programmiermodus und Störungen / alle Zustände |
| Helligkeit der Statusanzeige | 10 % | 1 … 100 % |
| Lebenszeichen (kurz grün alle 10 s) | nein | ja / nein |
| Messwerte per MQTT senden alle (0 = nie) | 60 s | 0 … 43200 s |
| Home Assistant MQTT Discovery | nein | ja / nein |

| KO | Störung | DPT |
|---:|---|---|
| 21 | Störung Wetterstation | 1.005 |
| 22 | Störung Feinstaubsensor | 1.005 |
| 23 | Störung 1-Wire-Sensor | 1.005 |
<!-- /GEN -->

## Datenexport (InfluxDB)

Schreibt alle gültigen Messwerte per HTTP direkt in eine InfluxDB 1.x (Schnittstelle `/write`, ohne TLS). Je Intervall
entstehen drei Punkte: Wetterstation, 1-Wire und SDS011. Sie unterscheiden sich im Tag `sensor`. Alle Zahlen sind
Gleitkommafelder, den Zeitstempel setzt der Server. Bei Fehlern verlängert die Station die Wartezeit schrittweise.
Das Passwort steht im ETS-Projekt und im Gerät im Klartext.

<!-- GEN:Influx -->
| Parameter | Standard | Bereich |
|---|---|---|
| Messwerte in InfluxDB 1.x schreiben | nein | ja / nein |
| Server (Name oder IP-Adresse, ohne http://) | (leer) |  |
| Port | 8086 | 1 … 65535 |
| Datenbank | (leer) |  |
| Retention Policy (leer = Standard) | (leer) |  |
| Benutzer (leer = ohne Anmeldung) | (leer) |  |
| Passwort | (leer) |  |
| Schreiben alle | 60 s | 10 … 3600 s |
| Measurement | weather |  |
| Tag floor | Aussen |  |
| Tag room | Garten |  |
| Tag sensor (1-Wire: _1wire, SDS011: _sds011 angehängt) | wn90 |  |
<!-- /GEN -->

## Temperatur

Lufttemperatur der WN90LP und optional eines DS18B20 am 1-Wire-Anschluss. Der 1-Wire-Sensor eignet sich zum Beispiel für
einen strahlungsgeschützten Messort im Schatten.

<!-- GEN:Temperature -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 24 | Temperatur | 9.001, 14.068 | aktiv, alle 300 s, ±0,5 °C | Lufttemperatur der WN90LP (Auflösung 0,1 °C) einschließlich der Korrektur auf der Seite Allgemein. |
| 25 | Temperatur 1-Wire | 9.001, 14.068 | aktiv, alle 300 s, ±0,5 °C | Temperatur des DS18B20 am 1-Wire-Anschluss (12 Bit, 0,0625 °C). Unplausible Werte (nicht erreichbar, 85 °C nach dem Einschalten) werden verworfen. |
<!-- /GEN -->

## Luftfeuchte

Die relative Feuchte sagt, wie nahe die Luft an der Sättigung ist. Die absolute Feuchte (g/m³) gibt den tatsächlichen
Wassergehalt an und ist unabhängig von der Temperatur vergleichbar. Der Taupunkt ist die Temperatur, auf die sich Luft
abkühlen muss, damit Tau entsteht. Liegt die Temperatur nahe am Taupunkt, ist Nebel oder Tau wahrscheinlich.

<!-- GEN:Humidity -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 26 | Relative Luftfeuchte | 9.007, 5.001 | aktiv, alle 300 s, ±2 % | Relative Luftfeuchte der WN90LP (Auflösung 1 %). |
| 27 | Absolute Luftfeuchte | 9.029 | inaktiv, alle 300 s, ±0,5 g/m³ | Wassergehalt der Luft aus Temperatur und relativer Feuchte; Grundlage der Lüftungsempfehlung. |
| 28 | Taupunkt | 9.001, 14.068 | inaktiv, alle 300 s, ±0,5 °C | Temperatur, bei der die Luft mit Wasserdampf gesättigt ist und Tau entsteht (Magnus-Formel). |
| 29 | Frostpunkt | 9.001, 14.068 | inaktiv, alle 300 s, ±0,5 °C | Wie der Taupunkt, aber über Eis gerechnet: unter 0 °C bildet sich bei dieser Temperatur Reif. |
<!-- /GEN -->

## Behaglichkeit

Indizes für die gefühlte Temperatur im Freien:

| Index | Gültig | Grundlage |
|---|---|---|
| Windchill | bis 10 °C und ab 5 km/h Wind | Temperatur und 10-Minuten-Mittel des Windes (Environment Canada) |
| Hitzeindex | ab 27 °C | Temperatur und relative Feuchte (Rothfusz, US-Wetterdienst) |
| Humidex | immer | Temperatur und Taupunkt (kanadischer Index) |
| Discomfort-Index | immer | Temperatur und relative Feuchte (Thom) |
| Gefühlte Temperatur | immer | bis 10 °C Windchill, ab 27 °C Hitzeindex, dazwischen Lufttemperatur |

Außerhalb des Gültigkeitsbereichs liefern Windchill und Hitzeindex die Lufttemperatur. Die gefühlte Temperatur entspricht
der Angabe „gefühlt“ vieler Wetter-Apps.

<!-- GEN:Comfort -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 30 | Humidex | 9.001, 14.068 | inaktiv, alle 600 s, ±1 °C | Gefühlte Temperatur aus Temperatur und Taupunkt (kanadischer Index). |
| 31 | Hitzeindex | 9.001, 14.068 | inaktiv, alle 600 s, ±1 °C | Gefühlte Temperatur bei Wärme aus Temperatur und Luftfeuchte. |
| 32 | Windchill | 9.001, 14.068 | inaktiv, alle 600 s, ±1 °C | Gefühlte Temperatur bei Kälte und Wind (10-Minuten-Mittel). Gilt nur bis 10 °C und ab 5 km/h, sonst wird die Lufttemperatur gesendet. |
| 33 | Discomfort-Index | 9.001, 14.068 | inaktiv, alle 600 s, ±1 °C | Unbehaglichkeitsindex nach Thom aus Temperatur und Luftfeuchte. |
| 211 | Gefühlte Temperatur | 9.001, 14.068 | inaktiv, alle 600 s, ±1 °C | Wie in Wetter-Apps: bis 10 °C der Windchill, ab 27 °C der Hitzeindex, dazwischen die Lufttemperatur. |
<!-- /GEN -->

## Wind

Die WN90LP misst Wind per Ultraschall und liefert alle 2 s Windgeschwindigkeit, Böe und Richtung. Momentanwerte schwanken
stark. Für Anzeigen und Logik eignen sich das 10-Minuten-Mittel (meteorologischer Standard) und die stärkste Böe der
letzten 10 Minuten besser.

Beaufort-Grenzen (m/s): 1 ab 0,3 · 2 ab 1,6 · 3 ab 3,4 · 4 ab 5,5 · 5 ab 8,0 · 6 ab 10,8 · 7 ab 13,9 · 8 ab 17,2 ·
9 ab 20,8 · 10 ab 24,5 · 11 ab 28,5 · 12 ab 32,7.

<!-- GEN:Wind -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 34 | Windgeschwindigkeit | 9.005, 9.028, 14.065 | aktiv, alle 60 s, ±1 m/s | Momentanwert der WN90LP (Ultraschall), alle 2 s neu. Für Schaltaufgaben besser das 10-Minuten-Mittel oder die Windalarme verwenden. |
| 35 | Böengeschwindigkeit | 9.005, 9.028, 14.065 | aktiv, alle 60 s, ±1 m/s | Böengeschwindigkeit der WN90LP, alle 2 s neu. |
| 36 | Windstärke (Beaufort) | 5.010 | inaktiv, alle 300 s, ±1 Bft | Windgeschwindigkeit (Momentanwert) in der Beaufort-Skala 0–12. |
| 37 | Böenstärke (Beaufort) | 5.010 | inaktiv, alle 300 s, ±1 Bft | Böengeschwindigkeit in der Beaufort-Skala 0–12. |
| 38 | Windrichtung | 5.003, 14.007 | aktiv, alle 60 s, ±22,5 ° | Herkunftsrichtung des Windes: 0° Nord, 90° Ost, 180° Süd, 270° West. Bei schwachem Wind wenig aussagekräftig; Sprünge zwischen 359° und 0° sind normal. |
| 63 | Windgeschwindigkeit (10-min-Mittel) | 9.005, 9.028, 14.065 | aktiv, alle 300 s, ±1 m/s | Gleitendes Mittel der letzten 10 Minuten (meteorologischer Standard). |
| 64 | Böe (Maximum 10 min) | 9.005, 9.028, 14.065 | aktiv, alle 300 s, ±1 m/s | Stärkste Böe der letzten 10 Minuten. |
<!-- /GEN -->

## Windalarm

Drei unabhängige Alarmstufen, zum Beispiel für Markisen, Jalousien und Rollläden. Der Alarm löst aus, wenn die gewählte
Quelle die Schwelle für die Auslöseverzögerung überschreitet. Er wird erst zurückgenommen, wenn sie die Rücknahmezeit lang
darunter bleibt. Bei einer Störung der WN90LP lösen alle aktiven Stufen aus.

Empfehlung: Böe als Quelle mit kurzer Auslöseverzögerung (wenige Sekunden) und langer Rücknahme (10 Minuten oder mehr).
Anhaltswerte: Markisen 8–10 m/s, Raffstores 12–15 m/s, Rollläden 15–20 m/s (Herstellerangaben beachten).

<!-- GEN:WindAlarm -->
Parameter unter „Windalarm Stufe 1“:

| Parameter | Standard | Bereich |
|---|---|---|
| Auslösen durch | Böe (Momentanwert) | Böe (Momentanwert) / Windgeschwindigkeit (Momentanwert) / Windgeschwindigkeit (10-min-Mittel) |
| Schwelle | 10 m/s | 1 … 50 m/s |
| Auslösen nach | 3 s | 0 … 60 s |
| Zurücknehmen nach (unter der Schwelle) | 10 min | 1 … 120 min |

Parameter unter „Windalarm Stufe 2“:

| Parameter | Standard | Bereich |
|---|---|---|
| Auslösen durch | Böe (Momentanwert) | Böe (Momentanwert) / Windgeschwindigkeit (Momentanwert) / Windgeschwindigkeit (10-min-Mittel) |
| Schwelle | 13 m/s | 1 … 50 m/s |
| Auslösen nach | 3 s | 0 … 60 s |
| Zurücknehmen nach (unter der Schwelle) | 10 min | 1 … 120 min |

Parameter unter „Windalarm Stufe 3“:

| Parameter | Standard | Bereich |
|---|---|---|
| Auslösen durch | Böe (Momentanwert) | Böe (Momentanwert) / Windgeschwindigkeit (Momentanwert) / Windgeschwindigkeit (10-min-Mittel) |
| Schwelle | 17 m/s | 1 … 50 m/s |
| Auslösen nach | 3 s | 0 … 60 s |
| Zurücknehmen nach (unter der Schwelle) | 10 min | 1 … 120 min |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 65 | Windalarm Stufe 1 | 1.005 | aktiv, alle 300 s, bei Änderung | 1 = Alarm. Quelle, Schwelle und Verzögerungen stehen darunter. |
| 66 | Windalarm Stufe 2 | 1.005 | inaktiv, alle 300 s, bei Änderung | 1 = Alarm. Quelle, Schwelle und Verzögerungen stehen darunter. |
| 67 | Windalarm Stufe 3 | 1.005 | inaktiv, alle 300 s, bei Änderung | 1 = Alarm. Quelle, Schwelle und Verzögerungen stehen darunter. |
<!-- /GEN -->

## Luftdruck

Der absolute Luftdruck gilt für die Höhe der Station. Für Wetterberichte und Prognosen rechnet die Station ihn mit Höhe
und Temperatur auf Meereshöhe um. Die Tendenz ist die Differenz zum Wert vor 1 bzw. 3 Stunden. Nach einem Neustart füllt
die Station Lücken bis 3 Stunden im gespeicherten Verlauf linear auf, längere Ausfälle beginnen neu.

<!-- GEN:Pressure -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 39 | Luftdruck (absolut) | 9.006, 14.058 | aktiv, alle 600 s, ±1 hPa | Luftdruck am Standort (Stationsdruck). Für den Vergleich mit Wetterberichten den Luftdruck auf Meereshöhe verwenden. |
| 40 | Luftdruck (auf Meereshöhe) | 9.006, 14.058 | inaktiv, alle 600 s, ±1 hPa | Benötigt die Höhe des Standorts auf der Seite Allgemein. |
| 41 | Luftdrucktendenz 1 Stunde | 9.006, 14.058 | inaktiv, alle 900 s, ±0,5 hPa | Differenz zum Luftdruck vor einer Stunde; verfügbar frühestens eine Stunde nach dem Start. |
| 42 | Luftdrucktendenz 3 Stunden | 9.006, 14.058 | inaktiv, alle 900 s, ±0,5 hPa | Differenz zum Luftdruck vor drei Stunden; verfügbar frühestens drei Stunden nach dem Start. |
<!-- /GEN -->

## Helligkeit und UV

Die Helligkeit ist Grundlage für die geschätzte Globalstrahlung, Tag/Nacht, das Wettersymbol und die Fassaden. Der
UV-Index richtet sich nach der WHO-Skala.

<!-- GEN:Light -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 43 | Helligkeit | 9.004 | aktiv, alle 300 s, ±20 % | Beleuchtungsstärke (volle Sonne etwa 100.000 lx). Grundlage für Globalstrahlung, Tag/Nacht und Wettersymbol. |
| 44 | UV-Index | 9, 5.010 | aktiv, alle 300 s, ±1 | UV-Index nach WHO: 0–2 niedrig, 3–5 mäßig, 6–7 hoch, 8–10 sehr hoch, ab 11 extrem. |
<!-- /GEN -->

## Regen

Grundlage ist der Regenzähler der WN90LP (0,01 mm). Zeitfenster laufen gleitend, Kalenderwerte (heute, gestern, Woche,
Monat, Jahr) richten sich nach der Ortszeit. Ein Zählerreset der WN90LP wird erkannt und nicht als Regen gewertet.
Nach der ersten Inbetriebnahme enthalten lange Fenster nur die Zeit seit dem Start, 72 Stunden zum Beispiel erst nach drei
Tagen.

<!-- GEN:Rain -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 45 | Regenmenge gesamt | 9.026, 14 | aktiv, alle 3600 s, ±1 mm | Fortlaufender Zähler seit Inbetriebnahme, bleibt bei Neustart und Update erhalten. DPT 14 empfohlen: DPT 9 wird bei großen Werten grob (ab 1000 mm in Schritten von 0,64 mm). |
| 46 | Regenintensität | 9 | aktiv, alle 600 s, ±1 mm/h | Hochgerechnet aus der Regenmenge der letzten 10 Minuten. |
| 47 | Regenmenge letzte 5 Minuten | 9.026 | inaktiv, alle 300 s, ±0,2 mm | Niederschlag der letzten 5 Minuten (gleitend). |
| 48 | Regenmenge letzte Stunde | 9.026 | aktiv, alle 600 s, ±0,5 mm | Niederschlag der letzten 60 Minuten (gleitend). |
| 49 | Regenmenge letzte 6 Stunden | 9.026 | inaktiv, alle 1800 s, ±1 mm | Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr. |
| 50 | Regenmenge letzte 24 Stunden | 9.026 | aktiv, alle 3600 s, ±1 mm | Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr. |
| 51 | Regenmenge letzte 48 Stunden | 9.026 | inaktiv, alle 3600 s, ±1 mm | Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr. |
| 52 | Regenmenge letzte 72 Stunden | 9.026 | inaktiv, alle 3600 s, ±1 mm | Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr. |
| 53 | Regenmenge heute | 9.026 | aktiv, alle 3600 s, ±0,5 mm | Wird um Mitternacht zurückgesetzt; benötigt eine gültige Uhrzeit (KO Uhrzeit/Datum oder NTP). |
| 54 | Regen | 1.002 | aktiv, alle 600 s, bei Änderung | 1 = es regnet: Mindestmenge im Zeitfenster erreicht (Seite Allgemein). Wird bei jeder Änderung gesendet. |
| 190 | Regenmenge gestern | 9.026 | aktiv, alle 3600 s, ±0,5 mm | Tagessumme des Vortags (0-24 Uhr Ortszeit), wird um Mitternacht übernommen und bleibt über Neustarts erhalten. Ungültig, solange noch kein vollständiger Tag gemessen wurde oder die Station über Mitternacht aus war. |
| 191 | Regenmenge diese Woche | 9.026, 14 | inaktiv, alle 3600 s, ±1 mm | Seit Montag 0 Uhr. Im ersten Zeitraum nach der Inbetriebnahme nur seit dem Start. |
| 192 | Regenmenge dieser Monat | 9.026, 14 | inaktiv, alle 3600 s, ±1 mm | Seit dem Monatsersten 0 Uhr. Im ersten Zeitraum nach der Inbetriebnahme nur seit dem Start. |
| 193 | Regenmenge dieses Jahr | 9.026, 14 | inaktiv, alle 3600 s, ±5 mm | Seit dem 1. Januar 0 Uhr. Im ersten Zeitraum nach der Inbetriebnahme nur seit dem Start. DPT 9 löst große Werte grob auf (bei 1000 mm etwa 0,6 mm), DPT 14 ist genau. |
<!-- /GEN -->

## Feinstaub

Nur sichtbar, wenn auf der Seite Allgemein ein SDS011 eingetragen ist. Bei hoher Luftfeuchte quellen Partikel auf, und der
optische Sensor zeigt zu viel an. Die feuchtekorrigierten Werte gleichen das aus und sind Grundlage der Auswertung.

- **Mittelwerte:** 1 Stunde aus sechs 10-Minuten-Abschnitten, 24 Stunden aus 24 Stundenmitteln (gültig ab 18 Stunden),
  Tagesmittel nach Ortszeit, Jahresmittel aus den Tagesmitteln.
- **Luftqualitätsindex** des Umweltbundesamts aus dem 24-Stunden-Mittel (1 sehr gut bis 5 sehr schlecht). Die Ampel fasst
  ihn zusammen: grün = sehr gut/gut, gelb = mäßig, rot = schlecht/sehr schlecht.
- **Überschreitungstage** zählen die Tage seit 1. Januar mit Tagesmittel über der gewählten Schwelle.

Der SDS011 ist ein preiswerter Sensor. Seine Werte eignen sich für Tendenz und Ampel, nicht als Nachweis von Grenzwerten.

<!-- GEN:Particles -->
| Parameter | Standard | Bereich |
|---|---|---|
| Überschreitungstage zählen nach | WHO 2021 (Tagesmittel PM2,5 über 15, PM10 über 45 µg/m³) | WHO 2021 (Tagesmittel PM2,5 über 15, PM10 über 45 µg/m³) / EU ab 2030 (Tagesmittel PM2,5 über 25, PM10 über 45 µg/m³) / EU bis 2029 (Tagesmittel PM10 über 50 µg/m³, PM2,5 über 25 µg/m³) |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 55 | Feinstaub PM2,5 | 9.030 | aktiv, alle 300 s, ±2 µg/m³ | Rohwert des SDS011 für Partikel bis 2,5 µm. Bei hoher Luftfeuchte zeigt der Sensor zu viel an; dann den feuchtekorrigierten Wert verwenden. |
| 56 | Feinstaub PM10 | 9.030 | aktiv, alle 300 s, ±2 µg/m³ | Rohwert des SDS011 für Partikel bis 10 µm. Bei hoher Luftfeuchte zeigt der Sensor zu viel an; dann den feuchtekorrigierten Wert verwenden. |
| 57 | Feinstaub PM2,5 (feuchtekorrigiert) | 9.030 | inaktiv, alle 300 s, ±2 µg/m³ | Rohwert korrigiert um den Einfluss der Luftfeuchte (Faktoren nach luftdaten.info); Grundlage der Mittelwerte und des Luftqualitätsindex. |
| 58 | Feinstaub PM10 (feuchtekorrigiert) | 9.030 | inaktiv, alle 300 s, ±2 µg/m³ | Rohwert korrigiert um den Einfluss der Luftfeuchte (Faktoren nach luftdaten.info); Grundlage der Mittelwerte und des Luftqualitätsindex. |
| 194 | PM2,5 Mittel 1 Stunde | 9.030 | inaktiv, alle 600 s, ±2 µg/m³ | Gleitendes Mittel der letzten Stunde. |
| 195 | PM10 Mittel 1 Stunde | 9.030 | inaktiv, alle 600 s, ±2 µg/m³ | Gleitendes Mittel der letzten Stunde. |
| 196 | PM2,5 Mittel 24 Stunden | 9.030 | aktiv, alle 1800 s, ±1 µg/m³ | Gleitendes Mittel der letzten 24 Stundenmittel; gültig, sobald 18 Stunden mit Messwerten vorliegen. Grundlage für WHO-Richtwert und Luftqualitätsindex. |
| 197 | PM10 Mittel 24 Stunden | 9.030 | aktiv, alle 1800 s, ±1 µg/m³ | Gleitendes Mittel der letzten 24 Stundenmittel; gültig, sobald 18 Stunden mit Messwerten vorliegen. |
| 198 | PM2,5 Tagesmittel gestern | 9.030 | inaktiv, alle 3600 s, ±1 µg/m³ | Mittel des Vortags (0-24 Uhr Ortszeit); ungültig, wenn die Station über Mitternacht aus war. |
| 199 | PM10 Tagesmittel gestern | 9.030 | inaktiv, alle 3600 s, ±1 µg/m³ | Mittel des Vortags (0-24 Uhr Ortszeit); ungültig, wenn die Station über Mitternacht aus war. |
| 200 | PM2,5 Jahresmittel (laufendes Jahr) | 9.030 | inaktiv, alle 3600 s, ±0,5 µg/m³ | Mittel der Tagesmittel seit 1. Januar. WHO-Leitwert 5 µg/m³, EU-Grenzwert 25 µg/m³ (ab 2030: 10). |
| 201 | PM10 Jahresmittel (laufendes Jahr) | 9.030 | inaktiv, alle 3600 s, ±0,5 µg/m³ | Mittel der Tagesmittel seit 1. Januar. WHO-Leitwert 15 µg/m³, EU-Grenzwert 40 µg/m³ (ab 2030: 20). |
| 202 | PM2,5 über WHO-Richtwert (24 h) | 1.005 | aktiv, alle 3600 s, bei Änderung | 1, solange das 24-Stunden-Mittel über 15 µg/m³ liegt (WHO 2021). |
| 203 | PM10 über WHO-Richtwert (24 h) | 1.005 | aktiv, alle 3600 s, bei Änderung | 1, solange das 24-Stunden-Mittel über 45 µg/m³ liegt (WHO 2021). |
| 204 | PM2,5 Überschreitungstage (laufendes Jahr) | 7.001 | inaktiv, alle 3600 s, bei Änderung | Tage seit 1. Januar mit Tagesmittel über der eingestellten Schwelle (unten auf dieser Seite). |
| 205 | PM10 Überschreitungstage (laufendes Jahr) | 7.001 | inaktiv, alle 3600 s, bei Änderung | Tage seit 1. Januar mit Tagesmittel über der eingestellten Schwelle (unten auf dieser Seite). |
| 206 | Luftqualitätsindex PM2,5 (1-5) | 5.010 | inaktiv, alle 3600 s, bei Änderung | Umweltbundesamt, 24-Stunden-Mittel: 1 sehr gut (bis 10), 2 gut (bis 20), 3 mäßig (bis 25), 4 schlecht (bis 50), 5 sehr schlecht. |
| 207 | Luftqualitätsindex PM10 (1-5) | 5.010 | inaktiv, alle 3600 s, bei Änderung | Umweltbundesamt, 24-Stunden-Mittel: 1 sehr gut (bis 20), 2 gut (bis 35), 3 mäßig (bis 50), 4 schlecht (bis 100), 5 sehr schlecht. |
| 208 | Luftqualitätsindex Feinstaub (1-5) | 5.010 | aktiv, alle 3600 s, bei Änderung | Schlechterer Wert aus PM2,5 und PM10. |
| 209 | Luftqualität Feinstaub (Text) | 16.001 | aktiv, alle 3600 s, bei Änderung | „sehr gut“, „gut“, „mäßig“, „schlecht“ oder „sehr schlecht“. |
| 210 | Feinstaub-Ampel (0 grün, 1 gelb, 2 rot) | 5.010 | aktiv, alle 3600 s, bei Änderung | 0 grün (Index sehr gut oder gut), 1 gelb (mäßig), 2 rot (schlecht oder sehr schlecht). |
<!-- /GEN -->

## Prognose und Display

- **Wettertendenz** aus der Luftdruckänderung über 3 Stunden (−2 bis +2).
- **Zambretti-Prognose** für etwa 12 Stunden aus Luftdruck auf Meereshöhe, Tendenz, mittlerer Windrichtung der letzten 10 Minuten (bei Windstille oder umlaufendem Wind ohne) und Jahreszeit
  (1 = beständig schön bis 26 = Sturm mit viel Regen). Die Prognose beschreibt die Entwicklung, nicht das aktuelle Wetter:
  Bei hohem, steigendem Druck kann sie auch während eines Schauers „schön“ melden.
- **Wettersymbol** für Displays, nach Vorrang: Gewitter (Schätzung aus starkem Regen und Böen), starker Regen, Schauer,
  hoher UV-Index, sonnig/bewölkt. Sonnig/bewölkt vergleicht tagsüber die Helligkeit mit dem Wert bei klarem Himmel,
  nachts entscheidet die Prognose.

<!-- GEN:Forecast -->
| Parameter | Standard | Bereich |
|---|---|---|
| Starker Regen ab | 10 mm/h | 0,5 … 100 mm/h |
| Gewitter: starker Regen und Böen ab | 7 Bft | 1 … 12 Bft |
| Hoher UV-Index ab | 6 | 1 … 15 |
| Sonnig ab Anteil der Helligkeit bei klarem Himmel | 60 % | 10 … 100 % |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 59 | Wettertendenz | 6.010 | aktiv, alle 1800 s, bei Änderung | Aus der Luftdruckänderung über 3 Stunden: -2 stark fallend (Sturm möglich), -1 fallend (Verschlechterung), 0 gleichbleibend, +1 steigend (Besserung), +2 stark steigend. In den ersten 3 Stunden nach dem Start aus 1 Stunde hochgerechnet. |
| 60 | Wetterprognose (Zambretti-Nummer) | 5.010 | inaktiv, alle 1800 s, bei Änderung | Prognose für etwa 12 Stunden nach Zambretti aus Luftdruck auf Meereshöhe, Tendenz, Windrichtung und Jahreszeit: 1 (A, beständig schön) bis 26 (Z, Sturm mit viel Regen). Die Höhe des Standorts auf der Seite Allgemein muss stimmen. |
| 61 | Wetterprognose (Text) | 16.001 | aktiv, alle 1800 s, bei Änderung | Kurztext der Zambretti-Prognose, z. B. „Wechsel,Regen“. |
| 62 | Wettersymbol (Display) | 5.010 | aktiv, alle 600 s, bei Änderung | 0 sonnig, 1 bewölkt, 2 Schauer, 3 starker Regen, 4 Gewitter, 5 hoher UV-Index (z. B. für Tantron-Displays). Gewitter ist eine Schätzung aus starkem Regen und Böen, die WN90LP misst keine Blitze. |
<!-- /GEN -->

## Sonne und Dämmerung

Sonnenstand aus OpenKNX (Gerätestandort und Uhrzeit nötig). Tag/Nacht nach Helligkeit mit zwei Schwellen (Hysterese) und
Verzögerung, unabhängig vom Sonnenstand. So schaltet etwa die Außenbeleuchtung bei dunklem Gewitterhimmel früher.

<!-- GEN:Sun -->
Parameter unter „Tag/Nacht (Dämmerung)“:

| Parameter | Standard | Bereich |
|---|---|---|
| Nacht unterhalb von | 30 lx | 1 … 10000 lx |
| Tag oberhalb von | 60 lx | 1 … 10000 lx |
| Verzögerung | 2 min | 0 … 60 min |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 68 | Sonnenstand Azimut | 14.007 | inaktiv, alle 300 s, ±2 ° | 0° = Nord, 90° = Ost. Benötigt Gerätestandort und gültige Uhrzeit. |
| 69 | Sonnenstand Elevation | 14.007 | inaktiv, alle 300 s, ±1 ° | Höhe über dem Horizont, negativ nachts. Benötigt Gerätestandort und gültige Uhrzeit. |
| 70 | Tag/Nacht (Dämmerung) | 1.024 | aktiv, alle 1800 s, bei Änderung | Aus der Helligkeit: Nacht unterhalb der unteren, Tag oberhalb der oberen Schwelle, jeweils nach der Verzögerung. |
<!-- /GEN -->

## Minimum und Maximum

Werte seit Mitternacht bzw. des Vortags. Ist die Station über Mitternacht aus, sind die Vortagswerte ungültig.

<!-- GEN:MinMax -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 71 | Temperatur Minimum heute | 9.001, 14.068 | aktiv, alle 3600 s, bei Änderung | Tiefste Temperatur seit 0 Uhr Ortszeit. |
| 72 | Temperatur Maximum heute | 9.001, 14.068 | aktiv, alle 3600 s, bei Änderung | Höchste Temperatur seit 0 Uhr Ortszeit. |
| 73 | Temperatur Minimum gestern | 9.001, 14.068 | inaktiv, alle 3600 s, bei Änderung | Tiefste Temperatur des Vortags; ungültig, wenn die Station über Mitternacht aus war. |
| 74 | Temperatur Maximum gestern | 9.001, 14.068 | inaktiv, alle 3600 s, bei Änderung | Höchste Temperatur des Vortags; ungültig, wenn die Station über Mitternacht aus war. |
| 75 | Stärkste Böe heute | 9.005, 9.028, 14.065 | aktiv, alle 3600 s, bei Änderung | Stärkste Böe seit 0 Uhr Ortszeit. |
| 76 | Stärkste Böe gestern | 9.005, 9.028, 14.065 | inaktiv, alle 3600 s, bei Änderung | Stärkste Böe des Vortags. |
<!-- /GEN -->

## Warnungen

- **Frost:** unter 0 °C, zurück ab +0,5 °C.
- **Glättegefahr:** Temperatur höchstens an der Schwelle und Feuchtigkeit vorhanden, also Regen in der letzten Stunde
  oder Taupunkt höchstens 1 K unter der Temperatur. Zurück, wenn die Temperatur 1 K über der Schwelle liegt oder
  30 Minuten keine Feuchtigkeit mehr vorhanden war.

<!-- GEN:Warnings -->
Parameter unter „Glättegefahr“:

| Parameter | Standard | Bereich |
|---|---|---|
| Glättegefahr bei Temperatur bis | 1 °C | -5 … 5 °C |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 77 | Frost | 1.005 | aktiv, alle 1800 s, bei Änderung | 1 bei Temperatur unter 0 °C, zurück ab +0,5 °C (Temperatur für Berechnungen). |
| 78 | Glättegefahr | 1.005 | aktiv, alle 1800 s, bei Änderung | 1, wenn die Temperatur unter der Schwelle liegt und Feuchtigkeit vorhanden ist: Regen in der letzten Stunde oder Taupunkt höchstens 1 K unter der Temperatur. Zurück, wenn die Temperatur 1 K über der Schwelle liegt oder 30 Minuten keine Feuchtigkeit mehr vorhanden war. |
<!-- /GEN -->

## Sommer/Winter und Kenntage

Sommer, wenn das gleitende Mittel der Tagesmitteltemperaturen über die eingestellte Zahl von Tagen über der Schwelle
liegt. Die Station bewertet einmal täglich um Mitternacht. Das Tagesmittel entsteht nach DIN aus den Mannheimer Stunden
(T7 + T14 + 2·T21) / 4 bzw. mit 22 Uhr als Abendtermin, oder als Mittel aller Messwerte. Mit DPT 1.100 lässt sich das
Ergebnis direkt als Heizen/Kühlen-Umschaltung verwenden (1 = Heizen = Winter).

Kenntage nach DWD: Sommertag (Maximum ab 25 °C), heißer Tag (ab 30 °C), Frosttag (Minimum unter 0 °C), Eistag (Maximum
unter 0 °C).

<!-- GEN:Season -->
Parameter unter „Sommer/Winter“:

| Parameter | Standard | Bereich |
|---|---|---|
| Tagesmitteltemperatur | DIN (7, 14 und Abendtermin, Abendwert doppelt) | DIN (7, 14 und Abendtermin, Abendwert doppelt) / Mittel aller Messwerte des Tages |
| Abendtermin (nur DIN) | 22 Uhr | 21 Uhr (Mannheimer Stunden) / 22 Uhr |
| Mittel über | 3 Tage | 1 … 7 Tage |
| Sommer bei Mitteltemperatur über | 16 °C | 5 … 25 °C |
| Hysterese (±) | 0 K | 0 … 5 K |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 79 | Sommer/Winter | 1.002, 1.100 | aktiv, alle 3600 s, bei Änderung | Sommer, wenn das Mittel der Tagesmitteltemperaturen der letzten Tage über der Schwelle liegt, sonst Winter. Bewertet wird einmal täglich um Mitternacht; der erste Wert liegt nach dem ersten vollständigen Tag vor. |
| 80 | Mittlere Außentemperatur (mehrere Tage) | 9.001, 14.068 | aktiv, alle 3600 s, ±0,5 °C | Gleitendes Mittel der Tagesmittel über die eingestellte Anzahl Tage (Grundlage der Sommer/Winter-Umschaltung). |
| 81 | Tagesmitteltemperatur gestern | 9.001, 14.068 | inaktiv, alle 3600 s, bei Änderung | Tagesmittel des Vortags nach dem eingestellten Verfahren (DIN/Mannheimer Stunden oder Mittel aller Messwerte). |
| 82 | Sommertag (Maximum ab 25 °C) | 1.002 | inaktiv, alle 3600 s, bei Änderung | 1, sobald das Tagesmaximum 25 °C erreicht. |
| 83 | Heißer Tag (Maximum ab 30 °C) | 1.002 | inaktiv, alle 3600 s, bei Änderung | 1, sobald das Tagesmaximum 30 °C erreicht. |
| 84 | Frosttag (Minimum unter 0 °C) | 1.002 | inaktiv, alle 3600 s, bei Änderung | 1, sobald das Tagesminimum unter 0 °C fällt. |
| 85 | Eistag (Maximum unter 0 °C) | 1.002 | inaktiv, alle 3600 s, bei Änderung | Kenntage gelten für den laufenden Tag nach dem bisherigen Minimum/Maximum; der Eistag steht erst am Tagesende endgültig fest. |
<!-- /GEN -->

## Strahlung und PV

Die WN90LP misst die Helligkeit, keine Strahlung. Die Station schätzt die Globalstrahlung mit etwa 110 lx je W/m². Die
typische Abweichung liegt bei 15–20 %. Mit dem Korrekturfaktor lässt sie sich an eine Referenz angleichen, etwa an eine
DWD-Station in der Nähe oder an den Ertrag der eigenen PV-Anlage.

Für geneigte Flächen (PV, Fassaden) teilt die Station die Globalstrahlung nach Erbs in direkten und diffusen Anteil auf.
Sie rechnet den Diffusanteil isotrop und berücksichtigt die Bodenreflexion (Albedo: Wiese etwa 20 %, frischer Schnee
80–90 %).

PV-Leistung = Anlagenleistung (kWp) × Einstrahlung auf die Modulfläche (kW/m²) × Performance Ratio. Verschattung und
Modultemperatur bleiben unberücksichtigt.

<!-- GEN:Solar -->
Parameter unter „Globalstrahlung (geschätzt)“:

| Parameter | Standard | Bereich |
|---|---|---|
| Korrekturfaktor Globalstrahlung | 100 % | 50 … 200 % |
| Bodenreflexion (Albedo) | 20 % | 0 … 90 % |

Parameter unter „PV-Leistung (geschätzt)“:

| Parameter | Standard | Bereich |
|---|---|---|
| Anlagenleistung | 5 kWp | 0,1 … 1000 kWp |
| Neigung der Module | 30 ° | 0 … 90 ° |
| Ausrichtung der Module (0° = Nord, 180° = Süd) | 180 ° | 0 … 359 ° |
| Systemwirkungsgrad (Performance Ratio) | 80 % | 10 … 100 % |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 86 | Globalstrahlung (geschätzt) | 9.022 | aktiv, alle 300 s, ±50 W/m² | Aus der Helligkeit geschätzt (ca. 110 lx je W/m²), typische Abweichung 15–20 %. Mit dem Korrekturfaktor an eine Referenz angleichen. Die Albedo wirkt nur auf Fassaden und geneigte PV-Flächen. |
| 87 | Bewölkungsgrad (geschätzt) | 5.001 | inaktiv, alle 600 s, ±10 % | Abweichung der Strahlung vom Wert bei klarem Himmel, nur tagsüber (Sonnenhöhe über 5°). Benötigt Uhrzeit und Gerätestandort. |
| 88 | Einstrahlung heute | 14 | inaktiv, alle 3600 s, ±0,1 kWh/m² | Summe der geschätzten Globalstrahlung seit 0 Uhr (klarer Sommertag etwa 7–8 kWh/m², trüber Wintertag unter 0,5 kWh/m²). |
| 89 | Einstrahlung gestern | 14 | inaktiv, alle 3600 s, bei Änderung | Tagessumme der Einstrahlung des Vortags. |
| 90 | PV-Leistung (geschätzt) | 14.056 | inaktiv, alle 300 s, ±100 W | Aus der Globalstrahlung auf die Modulfläche umgerechnet; typische Abweichung 20–30 %. Keine Verschattung, kein Temperatureinfluss. |
| 91 | PV-Ertrag heute (geschätzt) | 13.010, 14 | inaktiv, alle 3600 s, ±0,1 kWh | Benötigt den aktiven Wert PV-Leistung (Parameter der Anlage). |
| 92 | PV-Ertrag gestern (geschätzt) | 13.010, 14 | inaktiv, alle 3600 s, bei Änderung | Geschätzter PV-Ertrag des Vortags. |
<!-- /GEN -->

## Fassaden

Bis zu sechs Fassaden mit eigener Ausrichtung. Je Fassade berechnet die Station:

- **Einstrahlung auf die Fassade:** direkte Sonne, Diffusstrahlung und Bodenreflexion auf die senkrechte Fläche.
- **Sonne auf Fassade:** 1, wenn die Sonne im eingestellten Bereich steht (seitliche Abweichung, Mindest- und Höchsthöhe)
  und die Einstrahlung die Schwelle für die Meldeverzögerung überschreitet. 0, wenn eine Bedingung für die
  Rücknahmeverzögerung nicht mehr erfüllt ist. Geeignet für die Beschattungsautomatik.
- **Profilwinkel:** wie steil die Sonne aus Sicht der Fassade einfällt.

### Ausrichtung

Die Ausrichtung ist die Blickrichtung aus dem Fenster nach außen: 0° Nord, 90° Ost, 180° Süd, 270° West. Eine Südfassade
hat also 180°.

### Profilwinkel

Die Sonnenhöhe misst den Winkel über dem Horizont, unabhängig von der Fassade. Für Lamellen und Überstände zählt aber, wie
steil das Licht in der Ebene senkrecht zur Wand einfällt. Steht die Sonne seitlich, wirkt sie steiler:

tan(Profilwinkel) = tan(Sonnenhöhe) / cos(seitliche Abweichung zwischen Sonne und Fassadenrichtung)

Beispiel: Sonnenhöhe 30°, Sonne 60° seitlich zu einer Südfassade, Profilwinkel etwa 49°.

Anwendungen:

- **Lamellennachführung bei Raffstores:** Die Lamellen werden so gestellt, dass sie die direkte Sonne gerade abhalten und
  möglichst viel Tageslicht und Durchsicht lassen. Der passende Lamellenwinkel folgt aus Profilwinkel und Lamellengeometrie.
  Viele Jalousieaktoren und Beschattungsmodule nehmen den Profilwinkel direkt als Eingang.
- **Überstände und Balkone:** Die Schattentiefe unter einem Überstand ist Überstandstiefe × tan(Profilwinkel). Ein Vordach
  von 1 m wirft bei 49° etwa 1,15 m Schatten an die Wand.

Der Profilwinkel ist gültig, solange die Sonne geometrisch im Bereich der Fassade steht (seitliche Abweichung, Mindest- und
Höchsthöhe), unabhängig von Wolken. Außerhalb davon, also auch nachts, sendet die Station ihn nicht und schreibt ihn nicht
in die InfluxDB. Auf dem Bus bleibt der zuletzt gesendete Wert stehen. Grafana zeichnet die Lücke je nach Abfrage
(`fill(previous)`) oder Einstellung „Connect null values“ als waagerechte Linie.

### Grenzen

Die WN90LP misst nur waagerecht an ihrem Standort. Verschattung durch Nachbargebäude, Bäume oder Berge erkennt sie nicht.
Über die Mindesthöhe der Sonne lässt sich eine gleichmäßige Horizontverschattung nachbilden.

<!-- GEN:Facades -->
Einstellungen je Fassade (Fassade 1–6, hier Fassade 1):

| Parameter | Standard | Bereich |
|---|---|---|
| Fassade 1 auswerten | nein | ja / nein |
| Ausrichtung (Blickrichtung nach außen, 0° = Nord, 90° = Ost) | 90 ° | 0 … 359 ° |
| Sonne erfasst bis zu einer seitlichen Abweichung von ± | 80 ° | 10 … 90 ° |
| Mindesthöhe der Sonne | 5 ° | 0 … 90 ° |
| Höchste Sonnenhöhe (z. B. Dachüberstand) | 90 ° | 0 … 90 ° |
| Sonne ab Einstrahlung auf die Fassade von | 200 W/m² | 0 … 1000 W/m² |
| Sonne meldet nach | 2 min | 0 … 60 min |
| Sonne weg nach | 15 min | 1 … 120 min |

Objekte je Fassade (KO-Nummern für Fassade 1; alle Nummern in [KO-Liste.md](KO-Liste.md)):

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 93 | Sonne auf Fassade 1 | 1.002 | aktiv, alle 600 s, bei Änderung | 1, wenn die Sonne im eingestellten Winkelbereich steht und die Einstrahlung die Schwelle für die Dauer der Meldeverzögerung überschreitet; 0 nach der Rücknahmeverzögerung. |
| 94 | Einstrahlung Fassade 1 (geschätzt) | 9.022 | aktiv, alle 300 s, ±50 W/m² | Geschätzte Strahlung auf die senkrechte Fassade: direkter, diffuser und vom Boden reflektierter Anteil. |
| 95 | Profilwinkel Fassade 1 | 14.007 | inaktiv, alle 300 s, ±2 ° | Sonnenhöhe in der Ebene senkrecht zur Fassade (Lamellennachführung); nur gültig, solange die Sonne auf die Fassade scheint. |
<!-- /GEN -->

## Sonnenzeiten

Berechnet nach dem NOAA-Verfahren (Genauigkeit etwa 1 Minute) für den Gerätestandort, einmal täglich nach Mitternacht.
Uhrzeiten wahlweise als DPT 10.001, DPT 19.001 (Datum und Uhrzeit) oder Unix-Zeit (DPT 12.001).

| Ereignis | Sonnenhöhe (Mitte) |
|---|---|
| Auf-/Untergang | −0,833° (oberer Rand am Horizont, mit Refraktion) |
| Bürgerliche Dämmerung | −6° |
| Nautische Dämmerung | −12° |
| Astronomische Dämmerung | −18° |

Tag/Nacht nach Sonnenstand (nach dem EDOMI-Baustein „Sonnenstand“) kennt zwei Verfahren:

- **Auf-/Untergang mit Versatz:** Tag ab Sonnenaufgang + Versatz bis Sonnenuntergang + Versatz (negativ = früher).
- **Elevationsschwellen:** morgens Tag, sobald die Sonne über die Morgenschwelle steigt, abends Nacht, sobald sie unter die
  Abendschwelle sinkt.

Die Sonnenscheindauer zählt die Zeit mit Globalstrahlung über der Schwelle (WMO-Definition: 120 W/m²).

<!-- GEN:SunTimes -->
Parameter unter „Tag/Nacht (Sonnenstand)“:

| Parameter | Standard | Bereich |
|---|---|---|
| Verfahren | Sonnenauf-/untergang mit Versatz | Sonnenauf-/untergang mit Versatz / Elevationsschwellen |
| Versatz zum Sonnenaufgang | 0 min | -240 … 240 min |
| Versatz zum Sonnenuntergang | 0 min | -240 … 240 min |
| Morgens Tag ab Elevation | 0 ° | -18 … 30 ° |
| Abends Nacht unter Elevation | 0 ° | -18 … 30 ° |

Parameter unter „Sonnenscheindauer heute“:

| Parameter | Standard | Bereich |
|---|---|---|
| Sonnenschein ab Globalstrahlung | 120 W/m² | 0 … 1000 W/m² |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 111 | Sonnenaufgang | 10.001, 19.001, Unix-Zeit | aktiv, bei Änderung | Oberer Sonnenrand am Horizont (Sonnenmitte −0,833° mit Refraktion), Ortszeit. Wird täglich nach Mitternacht neu berechnet; Genauigkeit etwa 1 Minute. |
| 112 | Sonnenuntergang | 10.001, 19.001, Unix-Zeit | aktiv, bei Änderung | Oberer Sonnenrand am Horizont (Sonnenmitte −0,833° mit Refraktion), Ortszeit. |
| 113 | Sonnenhöchststand (Transit) | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Zeitpunkt des höchsten Sonnenstands (wahrer Mittag), Ortszeit. |
| 114 | Beginn bürgerliche Dämmerung | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Sonne steigt über −6°: hell genug für Tätigkeiten im Freien ohne Licht. |
| 115 | Ende bürgerliche Dämmerung | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Sonne sinkt unter −6°. |
| 116 | Beginn nautische Dämmerung | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Sonne steigt über −12°: der Horizont wird sichtbar. |
| 117 | Ende nautische Dämmerung | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Sonne sinkt unter −12°. |
| 118 | Beginn astronomische Dämmerung | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Sonne steigt über −18°: Ende der völligen Dunkelheit. |
| 119 | Ende astronomische Dämmerung | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Sonne sinkt unter −18°: völlige Dunkelheit. |
| 120 | Azimut bei Sonnenaufgang | 14.007 | inaktiv, bei Änderung | Himmelsrichtung des Sonnenaufgangs (0° Nord, 90° Ost). |
| 121 | Azimut bei Sonnenuntergang | 14.007 | inaktiv, bei Änderung | Himmelsrichtung des Sonnenuntergangs (0° Nord, 270° West). |
| 122 | Elevation beim Höchststand | 14.007 | inaktiv, bei Änderung | Höchste Sonnenhöhe des Tages. |
| 123 | Tageslänge | 13.100, 7.006 | inaktiv, bei Änderung | Zeit zwischen Sonnenaufgang und Sonnenuntergang. |
| 124 | Tagesfortschritt (Aufgang bis Untergang) | 5.001 | inaktiv, ±5 % | 0 % bei Sonnenaufgang, 100 % bei Sonnenuntergang (z. B. für die Positionierung eines Sonnensymbols in der Visualisierung). |
| 125 | Sonne steigt (0) / fällt (1) | 1.002 | inaktiv, bei Änderung | 0 bis zum Höchststand, 1 danach. |
| 126 | Tag/Nacht (Sonnenstand) | 1.024 | aktiv, alle 1800 s, bei Änderung | Nach Sonnenauf-/untergang mit Versatz oder nach Elevationsschwellen (morgens Tag ab, abends Nacht unter). Die Versätze gelten nur im ersten, die Schwellen nur im zweiten Verfahren. |
| 127 | Tag/Nacht (bürgerliche Dämmerung, -6°) | 1.024 | inaktiv, alle 1800 s, bei Änderung | 1 = Nacht, solange die Sonne tiefer als −6° steht. |
| 128 | Tag/Nacht (nautische Dämmerung, -12°) | 1.024 | inaktiv, alle 1800 s, bei Änderung | 1 = Nacht, solange die Sonne tiefer als −12° steht. |
| 129 | Tag/Nacht (astronomische Dämmerung, -18°) | 1.024 | inaktiv, alle 1800 s, bei Änderung | 1 = Nacht, solange die Sonne tiefer als −18° steht. |
| 130 | Globalstrahlung bei klarem Himmel | 9.022 | inaktiv, alle 300 s, ±50 W/m² | Theoretischer Wert für den aktuellen Sonnenstand (Haurwitz). |
| 131 | Globalstrahlung bei klarem Himmel, Tagesmaximum | 9.022 | inaktiv, bei Änderung | Strahlung bei klarem Himmel zum Höchststand (Haurwitz); Vergleichswert für die gemessene Globalstrahlung. |
| 132 | Sonnenscheindauer heute | 9, 14 | inaktiv, alle 3600 s, ±0,1 h | Zeit mit Globalstrahlung über der Schwelle (WMO: 120 W/m²), bleibt über Neustarts erhalten. |
| 133 | Sonnenscheindauer gestern | 9, 14 | inaktiv, bei Änderung | Sonnenscheindauer des Vortags. |
<!-- /GEN -->

## Mond

Mondposition und -phase nach gekürzten Meeus-Reihen (Phasenzeitpunkte etwa ±10 Minuten, Position etwa 0,5°).
Mondauf- und -untergang fehlen an Tagen, an denen sie nicht stattfinden. Das kommt etwa einmal im Monat vor.

<!-- GEN:Moon -->
| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 134 | Mondphase (0-7) | 5.010 | aktiv, bei Änderung | 0 Neumond, 1 zunehmende Sichel, 2 erstes Viertel, 3 zunehmender Mond, 4 Vollmond, 5 abnehmender Mond, 6 letztes Viertel, 7 abnehmende Sichel. |
| 135 | Mondphase (Text) | 16.001 | inaktiv, bei Änderung | Name der Mondphase, z. B. „Vollmond“; Statusseite und MQTT mit vollem Text. |
| 136 | Mond beleuchtet | 5.001 | aktiv, alle 3600 s, ±2 % | Beleuchteter Anteil der Mondscheibe. |
| 137 | Mondalter (Tage seit Neumond) | 9 | inaktiv, alle 3600 s, ±0,5 d | Tage seit dem letzten Neumond (ein Mondzyklus dauert etwa 29,5 Tage). |
| 138 | Mond Azimut | 14.007 | inaktiv, alle 600 s, ±5 ° | Himmelsrichtung des Mondes (0° Nord, 90° Ost). |
| 139 | Mond Elevation | 14.007 | inaktiv, alle 600 s, ±2 ° | Höhe des Mondes über dem Horizont, negativ unter dem Horizont. |
| 140 | Mondaufgang | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Leer, wenn der Mond an diesem Tag nicht aufgeht. |
| 141 | Monduntergang | 10.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Leer, wenn der Mond an diesem Tag nicht untergeht. |
| 142 | Nächster Vollmond | 11.001, 19.001, Unix-Zeit | aktiv, bei Änderung | Datum des nächsten Vollmonds (Ortszeit). |
| 143 | Nächster Neumond | 11.001, 19.001, Unix-Zeit | inaktiv, bei Änderung | Datum des nächsten Neumonds (Ortszeit). |
<!-- /GEN -->

## Lüftungsempfehlung und Raumklima

Für bis zu sechs Räume. Raumtemperatur und Raumfeuchte kommen über Eingangs-KOs vom Bus, etwa von Präsenzmeldern oder
Raumtemperaturreglern. Liegen seit 2 Stunden keine Werte vor, gibt die Station für den Raum nichts aus.

### Taupunktlüftung

Lüften senkt die Raumfeuchte nur, wenn die Außenluft weniger Wasser enthält als die Raumluft. Maßgeblich ist die absolute
Feuchte, nicht die relative: Kalte Außenluft mit 90 % relativer Feuchte enthält oft weniger Wasser als warme Raumluft mit
60 %. Ein typischer Fall ist ein Keller im Sommer: Warme, feuchte Außenluft würde dort kondensieren.

„Lüften sinnvoll“ wird 1, wenn der Unterschied der absoluten Feuchte (innen − außen) die Schwelle erreicht und alle
Freigaben erfüllt sind (Mindestfeuchte im Raum, Mindesttemperatur innen und außen). Es wird 0, wenn der Unterschied unter
die Schwelle minus 0,5 g/m³ fällt.

### Behaglichkeitsfeld nach Frank

Bewertung aus Raumtemperatur und relativer Raumfeuchte nach dem Behaglichkeitsfeld von W. Frank (1975, *Raumklima und
thermische Behaglichkeit*). Die Feldgrenzen stammen aus dem EDOMI-Baustein „Behaglichkeitsfeld nach Frank“:

| Feld | Eckpunkte (°C / % rF) |
|---|---|
| behaglich | 17,6/73,3 · 22,0/67,0 · 24,2/33,3 · 18,9/39,4 |
| noch behaglich | 19,3/20 · 25,3/17 · 26,7/31,8 · 24,5/60 · 20,5/81,8 · 17,0/87,3 · 16,0/76,4 · 17,1/38,8 |

- **Stufe:** 0 behaglich, 1 noch behaglich, 2 unbehaglich.
- **Gründe:** zu feucht (über 65 %), zu trocken (unter 30 %), zu kalt (außerhalb des behaglichen Feldes und unter
  18,9 °C), zu warm (außerhalb und über 22 °C).
- **Text:** „behaglich“, sonst der Grund, zum Beispiel „zu trocken“ oder „kalt, feucht“. Auf Statusseite und per MQTT
  ausführlich, etwa „noch behaglich (zu warm, zu trocken)“.
- **Abweichung:** Abstand zum Optimum 21 °C / 55 %. 1 entspricht 1 K oder 5 % relativer Feuchte.

<!-- GEN:Ventilation -->
Einstellungen je Raum (Raum 1–6, hier Raum 1):

| Parameter | Standard | Bereich |
|---|---|---|
| Raum 1 auswerten | nein | ja / nein |
| Lüften ab einem Feuchteunterschied von | 1 g/m³ | 0,1 … 10 g/m³ |
| nur wenn die Raumfeuchte mindestens beträgt | 50 % | 0 … 100 % |
| nur wenn die Raumtemperatur mindestens beträgt | 10 °C | -30 … 40 °C |
| nur wenn die Außentemperatur mindestens beträgt | -5 °C | -30 … 40 °C |

Objekte je Raum (KO-Nummern für Raum 1; alle Nummern in [KO-Liste.md](KO-Liste.md)):

| KO | Eingang | DPT |
|---:|---|---|
| 168 | Raumtemperatur Raum 1 | 9.001 |
| 169 | Raumfeuchte Raum 1 | 9.007 |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 144 | Lüften sinnvoll Raum 1 | 1.002 | aktiv, alle 1800 s, bei Änderung | 1 = Lüften senkt die Raumfeuchte: Feuchteunterschied mindestens die Schwelle und alle Freigaben erfüllt; 0 unter der Schwelle minus 0,5 g/m³. |
| 145 | Absolute Feuchte Raum 1 | 9.029 | inaktiv, alle 1800 s, ±0,5 g/m³ | Wassergehalt der Raumluft aus Raumtemperatur und Raumfeuchte. |
| 146 | Taupunkt Raum 1 | 9.001, 14.068 | inaktiv, alle 1800 s, ±0,5 °C | Taupunkt der Raumluft: Flächen, die kälter sind (Fensterlaibung, Außenwandecke), werden feucht. |
| 147 | Feuchteunterschied innen − außen Raum 1 | 9.029 | inaktiv, alle 1800 s, ±0,5 g/m³ | Positiv: draußen ist die Luft trockener, Lüften senkt die Feuchte im Raum. |
| 212 | Behaglichkeit Raum 1 (0 behaglich, 1 noch, 2 unbehaglich) | 5.010 | aktiv, alle 1800 s, bei Änderung | Behaglichkeitsfeld nach Frank (1975) aus Raumtemperatur und relativer Raumfeuchte. |
| 213 | Behaglichkeit Raum 1 (Text) | 16.001 | aktiv, alle 1800 s, bei Änderung | „behaglich“, sonst der Grund, z. B. „zu trocken“ oder „kalt, feucht“. |
| 214 | Abweichung vom Behaglichkeitsoptimum Raum 1 | 9 | inaktiv, alle 1800 s, ±0,5 | Abstand zu 21 °C / 55 %: 1 entspricht 1 K oder 5 % relativer Feuchte. |
| 215 | Raum 1 zu feucht (über 65 %) | 1.002 | inaktiv, alle 1800 s, bei Änderung | Relative Raumfeuchte über 65 % (Schimmelgefahr). |
| 216 | Raum 1 zu trocken (unter 30 %) | 1.002 | inaktiv, alle 1800 s, bei Änderung | Relative Raumfeuchte unter 30 %. |
| 217 | Raum 1 zu kalt | 1.002 | inaktiv, alle 1800 s, bei Änderung | Außerhalb des behaglichen Feldes und unter 18,9 °C. |
| 218 | Raum 1 zu warm | 1.002 | inaktiv, alle 1800 s, bei Änderung | Außerhalb des behaglichen Feldes und über 22 °C. |
<!-- /GEN -->

## Bewässerung

Tägliche Wasserbilanz für Garten oder Rasen:

- **Verdunstung (ET0):** Referenzverdunstung nach FAO-56 Penman-Monteith aus Temperatur, Feuchte, Wind (auf 2 m Höhe
  umgerechnet), Strahlung und Luftdruck.
- **Wasserbilanz:** Regen − ET0 × Pflanzenfaktor + Bewässerung. Positive Werte sind durch den Bodenspeicher begrenzt,
  der Überschuss versickert.
- **Bewässern:** 1, sobald das Defizit die Schwelle erreicht. 0, wenn es unter die halbe Schwelle sinkt.
- **Bewässerung erfolgt** (Eingang): schreibt die eingestellte Wassermenge gut, etwa vom Bewässerungsventil.

Pflanzenfaktoren (Anhalt): Rasen 80 %, Gemüse 100–110 %, Sträucher 50–70 %.

<!-- GEN:Irrigation -->
| Parameter | Standard | Bereich |
|---|---|---|
| Pflanzenfaktor (Kc, Rasen ca. 80 %) | 80 % | 10 … 100 % |
| Bewässern ab einem Defizit von | 5 mm | 0,5 … 100 mm |
| Bodenspeicher (Überschuss darüber versickert) | 20 mm | 0,5 … 100 mm |
| Gutschrift je Meldung „Bewässerung erfolgt“ | 10 mm | 0,5 … 100 mm |
| Montagehöhe der Wetterstation über Grund | 2 m | 1 … 50 m |

| KO | Eingang | DPT |
|---:|---|---|
| 174 | Bewässerung erfolgt | 1.017 |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 156 | Verdunstung heute (ET0) | 9.026 | aktiv, alle 3600 s, ±0,5 mm | Referenzverdunstung nach FAO-56 Penman-Monteith, laufend aus Temperatur, Feuchte, Wind, Strahlung und Luftdruck. |
| 157 | Verdunstung gestern (ET0) | 9.026 | inaktiv, alle 3600 s, bei Änderung | Referenzverdunstung des Vortags. |
| 158 | Wasserbilanz | 9 | aktiv, alle 3600 s, ±1 mm | Regen minus Verdunstung (mit Pflanzenfaktor) plus Bewässerung; negativ = Defizit. Nach oben durch den Bodenspeicher begrenzt. |
| 159 | Bewässern | 1.002 | aktiv, alle 3600 s, bei Änderung | 1, sobald das Defizit die Schwelle erreicht; 0, wenn es wieder unter der halben Schwelle liegt. |
<!-- /GEN -->

## Gradtage

Gradtagzahlen dienen dem witterungsbereinigten Vergleich des Heiz- oder Kühlenergieverbrauchs, etwa zwischen zwei
Wintern.

- **Heizgradtage (VDI 3807, G20/15):** Für jeden Tag mit Tagesmittel unter der Heizgrenze (15 °C) zählt die Differenz zur
  Innentemperatur (20 °C). Ein Tag mit 5 °C ergibt 15 Kd.
- **Kühlgradtage:** Für jeden Tag mit Tagesmittel über der Basistemperatur (18 °C) zählt die Differenz.

Die Einheit ist Kelvin·Tage (Kd), keine Anzahl Tage. In Deutschland kommen im Jahr typischerweise 3500–4000 Kd zusammen.
Das Tagesmittel entsteht wie bei Sommer/Winter. Die Station schreibt die Werte um Mitternacht fort.

<!-- GEN:DegreeDays -->
| Parameter | Standard | Bereich |
|---|---|---|
| Heizgradtage: Innentemperatur | 20 °C | 5 … 25 °C |
| Heizgradtage: Heizgrenze (Tagesmittel unter) | 15 °C | 5 … 25 °C |
| Kühlgradtage: Basistemperatur (Tagesmittel über) | 18 °C | 5 … 25 °C |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 160 | Heizgradtage laufender Monat | 9, 14 | aktiv, bei Änderung | Summe aus (Innentemperatur − Tagesmittel) über alle Tage mit Tagesmittel unter der Heizgrenze, in Kelvin·Tagen (Kd), keine Anzahl Tage: 60 Kd entsprechen z. B. 6 Tagen mit 10 °C Tagesmittel. |
| 161 | Heizgradtage Vormonat | 9, 14 | inaktiv, bei Änderung | Summe aus (Innentemperatur − Tagesmittel) über alle Tage mit Tagesmittel unter der Heizgrenze, in Kelvin·Tagen (Kd), keine Anzahl Tage: 60 Kd entsprechen z. B. 6 Tagen mit 10 °C Tagesmittel. |
| 162 | Heizgradtage laufendes Jahr | 9, 14 | aktiv, bei Änderung | Summe aus (Innentemperatur − Tagesmittel) über alle Tage mit Tagesmittel unter der Heizgrenze, in Kelvin·Tagen (Kd), keine Anzahl Tage: 60 Kd entsprechen z. B. 6 Tagen mit 10 °C Tagesmittel. |
| 163 | Heizgradtage Vorjahr | 9, 14 | inaktiv, bei Änderung | Summe aus (Innentemperatur − Tagesmittel) über alle Tage mit Tagesmittel unter der Heizgrenze, in Kelvin·Tagen (Kd), keine Anzahl Tage: 60 Kd entsprechen z. B. 6 Tagen mit 10 °C Tagesmittel. |
| 164 | Kühlgradtage laufender Monat | 9, 14 | aktiv, bei Änderung | Summe aus (Tagesmittel − Basistemperatur) über alle Tage mit Tagesmittel über der Basistemperatur, in Kelvin·Tagen (Kd). |
| 165 | Kühlgradtage Vormonat | 9, 14 | inaktiv, bei Änderung | Summe aus (Tagesmittel − Basistemperatur) über alle Tage mit Tagesmittel über der Basistemperatur, in Kelvin·Tagen (Kd). |
| 166 | Kühlgradtage laufendes Jahr | 9, 14 | aktiv, bei Änderung | Summe aus (Tagesmittel − Basistemperatur) über alle Tage mit Tagesmittel über der Basistemperatur, in Kelvin·Tagen (Kd). |
| 167 | Kühlgradtage Vorjahr | 9, 14 | inaktiv, bei Änderung | Summe aus (Tagesmittel − Basistemperatur) über alle Tage mit Tagesmittel über der Basistemperatur, in Kelvin·Tagen (Kd). |
<!-- /GEN -->

## Wetterwarnungen (DWD-Kriterien)

Die Station wendet die Warnkriterien des Deutschen Wetterdienstes auf ihre eigenen Messwerte an. Sie meldet, wenn die
Bedingungen tatsächlich eintreten. Das ist keine amtliche Warnung und keine Vorhersage. Gewitter und Schnee erkennt sie
nicht.

Stufen: 0 keine, 1 Wetterwarnung, 2 markantes Wetter, 3 Unwetter, 4 extremes Unwetter. Eine Stufe sinkt erst, wenn sie für
die Haltezeit nicht mehr erreicht wurde.

<!-- GEN:Warnings2 -->
| Parameter | Standard | Bereich |
|---|---|---|
| Warnung bleibt nach Ende der Bedingung bestehen für | 30 min | 1 … 120 min |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 175 | Warnstufe gesamt (0-4) | 5.010 | aktiv, alle 3600 s, bei Änderung | 0 keine Warnung, 1 Wetterwarnung, 2 markantes Wetter, 3 Unwetter, 4 extremes Unwetter (höchste Stufe aller Kategorien). |
| 176 | Warnung (Text) | 16.001 | aktiv, alle 3600 s, bei Änderung | Bezeichnung der höchsten Warnung, z. B. „Sturmböen“ oder „Starkregen“. |
| 177 | Unwetter (Stufe 3 oder 4) | 1.005 | aktiv, alle 3600 s, bei Änderung | 1 bei Stufe 3 oder 4 in mindestens einer Kategorie. |
| 178 | Warnstufe Wind | 5.010 | inaktiv, alle 3600 s, bei Änderung | Stärkste Böe der letzten 10 Minuten: 1 ab 50 km/h, 2 ab 65, 3 ab 105, 4 ab 140 km/h. |
| 179 | Warnstufe Starkregen | 5.010 | inaktiv, alle 3600 s, bei Änderung | 2 ab 15 mm in 1 h oder 20 mm in 6 h, 3 ab 25 bzw. 35 mm, 4 ab 40 bzw. 60 mm. |
| 180 | Warnstufe Dauerregen | 5.010 | inaktiv, alle 3600 s, bei Änderung | 2 ab 30 mm in 24 h, 40 mm in 48 h oder 60 mm in 72 h; 3 ab 50/60/90 mm; 4 ab 80/90/120 mm. |
| 181 | Warnstufe Frost/Glätte | 5.010 | inaktiv, alle 3600 s, bei Änderung | 1 Frost (unter 0 °C) oder Glätte, 2 strenger Frost (unter −10 °C), 3 Glatteis (Regen bei höchstens 0 °C). |
| 182 | Warnstufe Hitze | 5.010 | inaktiv, alle 3600 s, bei Änderung | Nach dem Hitzeindex: 2 ab 32 °C, 3 ab 38 °C. |
| 183 | Warnstufe UV | 5.010 | inaktiv, alle 3600 s, bei Änderung | 1 ab UV-Index 8. |
<!-- /GEN -->

## DWD-Warnungen (Internet)

Die einzige Funktion mit Internetzugang. Die Station ruft die amtlichen Warnungen für eine Warnzelle per HTTPS von
`maps.dwd.de` ab (offene Daten des DWD).

- **WarnCellID:** 9-stellige Nummer der Gemeinde (beginnt mit 8) oder des Landkreises (beginnt mit 1). Großstädte sind
  teils in Stadtteile mit 7…-Nummern aufgeteilt. Die Liste steht beim DWD unter „cap_warncellids_csv“.
- **Aktuell und angekündigt:** „aktuell“ umfasst nur derzeit gültige Warnungen, „inkl. angekündigter“ auch solche, die
  später beginnen.
- **Störung:** 1, wenn seit drei Abrufintervallen keine Daten kamen (Internet, DNS oder Server).
  Die übrigen DWD-Werte behalten dann den letzten bekannten Stand. Die Statusseite kennzeichnet sie als veraltet und nennt
  in der Kopfzeile den Fehler und das Alter der Daten.
- **Volltexte:** Die Statusseite zeigt alle Warnungen mit Zeitraum, Beschreibung und Verhaltenshinweisen. Per MQTT
  kommen sie unter `openknx/<Präfix>/dwd` (retained, bei Änderung). Das Format beschreibt die README.

<!-- GEN:DwdOfficial -->
| Parameter | Standard | Bereich |
|---|---|---|
| Amtliche DWD-Warnungen abrufen (benötigt Internet) | nein | ja / nein |
| Warnzellen-ID (WarnCellID, 9-stellig) | 0 | 0 … 999999999 |
| Abruf alle | 5 min | 2 … 60 min |

| KO | Wert | DPT | Standard | Beschreibung |
|---:|---|---|---|---|
| 184 | DWD-Warnstufe aktuell (0-4) | 5.010 | aktiv, alle 3600 s, bei Änderung | Höchste derzeit gültige amtliche Warnung: 1 Wetterwarnung, 2 markant, 3 Unwetter, 4 extremes Unwetter. |
| 185 | DWD-Warnstufe inkl. angekündigter Warnungen | 5.010 | inaktiv, alle 3600 s, bei Änderung | Höchste Stufe aller vorliegenden Warnungen, auch solcher, die erst später beginnen. |
| 186 | DWD-Warnung (Ereignis) | 16.001 | aktiv, alle 3600 s, bei Änderung | Ereignis der höchsten gültigen Warnung, z. B. „STURMBÖEN“ (max. 14 Zeichen). |
| 187 | DWD-Unwetterwarnung (Stufe 3 oder 4) | 1.005 | aktiv, alle 3600 s, bei Änderung | 1, solange eine amtliche Warnung der Stufe 3 oder 4 gilt. |
| 188 | Anzahl gültiger DWD-Warnungen | 5.010 | inaktiv, alle 3600 s, bei Änderung | Anzahl der derzeit gültigen amtlichen Warnungen für die Warnzelle. |
| 189 | Störung DWD-Abruf | 1.005 | aktiv, alle 3600 s, bei Änderung | 1, wenn seit drei Abrufintervallen keine Daten vom DWD kamen (Internet, DNS oder Server). |
<!-- /GEN -->
