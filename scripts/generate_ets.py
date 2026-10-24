#!/usr/bin/env python3
"""Erzeugt aus einer gemeinsamen Messwerttabelle
  - src/WN90LP.share.xml  (ETS-Modul WX fuer OpenKNXproducer)
  - src/ValueTable.h       (Messwerttabelle fuer die Firmware)

Aufruf im Projektverzeichnis: python3 scripts/generate_ets.py
Danach OpenKNXproducer ausfuehren (siehe README.md).

Speicherlayout je Messwert (11 Byte, byte-ausgerichtet):
  +0 Bit 7     Aktiv
  +0 Bit 5..6  DPT-Auswahl (nur bei mehreren DPTs)
  +1 uint16    Zyklisch senden in s (0 = nie)
  +3 float     Senden bei absoluter Aenderung (0 = nie)
  +7 float     Senden bei relativer Aenderung in % (0 = nie)
Die Bitnummern folgen der ETS-Zaehlweise (Bit 0 = MSB), die Firmware liest
die Werte ueber die von OpenKNXproducer erzeugten Makros.
"""
import pathlib
from xml.sax.saxutils import escape

ROOT = pathlib.Path(__file__).resolve().parent.parent
SHARE = ROOT / "src" / "WN90LP.share.xml"
HEADER = ROOT / "src" / "ValueTable.h"
KOLIST = ROOT / "doc" / "KO-Liste.md"
KO_OFFSET = 20  # muss zu KoSingleOffset in src/WN90LP.xml passen

VALUE_SIZE = 11  # Byte je Messwert


# DPT-Optionen: (ETS-Text, ETS-DPT, Objektgroesse, Kodierung Haupt/Untergruppe fuer den knx-Stack, Faktor)
def T9(sub, text):
    return (text, f"DPST-9-{sub}", "2 Bytes", 9, 1, 1.0)


def T14(sub, text, factor=1.0):
    dpt = f"DPST-14-{sub}" if sub is not None else "DPT-14"
    return (text, dpt, "4 Bytes", 14, 0, factor)


TEMP = [T9(1, "DPT 9.001 (2 Byte, °C)"), T14(68, "DPT 14.068 (4 Byte, °C)")]
SPEED = [
    T9(5, "DPT 9.005 (2 Byte, m/s)"),
    ("DPT 9.028 (2 Byte, km/h)", "DPST-9-28", "2 Bytes", 9, 1, 3.6),
    T14(65, "DPT 14.065 (4 Byte, m/s)"),
]
PRESSURE = [
    ("DPT 9.006 (2 Byte, Pa)", "DPST-9-6", "2 Bytes", 9, 1, 100.0),
    T14(58, "DPT 14.058 (4 Byte, Pa)", 100.0),
]
RAIN = [T9(26, "DPT 9.026 (2 Byte, mm)"), T14(None, "DPT 14 (4 Byte, mm)")]
RAIN9 = [T9(26, "DPT 9.026 (2 Byte, mm)")]
PM = [T9(30, "DPT 9.030 (2 Byte, µg/m³)")]
BFT = [("DPT 5.010 (1 Byte)", "DPST-5-10", "1 Byte", 5, 10, 1.0)]
BOOL = [("DPT 1.002 (1 Bit)", "DPST-1-2", "1 Bit", 1, 2, 1.0)]
TENDENCY = [("DPT 6.010 (1 Byte, -2..+2)", "DPST-6-10", "1 Byte", 6, 10, 1.0)]
TEXT = [("DPT 16.001 (14 Byte, Text)", "DPST-16-1", "14 Bytes", 16, 1, 1.0)]
ALARM = [("DPT 1.005 (1 Bit, Alarm)", "DPST-1-5", "1 Bit", 1, 5, 1.0)]
DAYNIGHT = [("DPT 1.024 (1 Bit, 0 = Tag, 1 = Nacht)", "DPST-1-24", "1 Bit", 1, 2, 1.0)]
ANGLE = [T14(7, "DPT 14.007 (4 Byte, °)")]
IRRADIANCE = [T9(22, "DPT 9.022 (2 Byte, W/m²)")]
# Zeitpunkte: KNX-Uhrzeit, KNX-Datum+Uhrzeit oder Unix-Zeit (Sekunden seit 1970 als DPT 12.001)
DATETIME = ("DPT 19.001 (8 Byte, Datum und Uhrzeit)", "DPST-19-1", "8 Bytes", 19, 1, 1.0)
UNIX = ("Unix-Zeit (DPT 12.001, 4 Byte, s seit 1970)", "DPST-12-1", "4 Bytes", 12, 1, 1.0)
TIME = [("DPT 10.001 (3 Byte, Uhrzeit)", "DPST-10-1", "3 Bytes", 10, 1, 1.0), DATETIME, UNIX]
DATE = [("DPT 11.001 (3 Byte, Datum)", "DPST-11-1", "3 Bytes", 11, 1, 1.0), DATETIME, UNIX]
# Dauer in Sekunden; 7.006 wird ueber die 16-Bit-Kodierung von 7.001 gesendet (gleiche Bytes)
DURATION = [("DPT 13.100 (4 Byte, s)", "DPST-13-100", "4 Bytes", 13, 100, 1.0),
            ("DPT 7.006 (2 Byte, min)", "DPST-7-6", "2 Bytes", 7, 1, 1.0 / 60)]
HOURS = [("DPT 9 (2 Byte, h)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, h)")]
NUMBER = [("DPT 9 (2 Byte)", "DPT-9", "2 Bytes", 9, 1, 1.0)]
PERCENT = [("DPT 5.001 (1 Byte, %)", "DPST-5-1", "1 Byte", 5, 1, 1.0)]
COUNT = [("DPT 7.001 (2 Byte, Anzahl)", "DPST-7-1", "2 Bytes", 7, 1, 1.0)]
ENERGY = [("DPT 13.010 (4 Byte, Wh)", "DPST-13-10", "4 Bytes", 13, 10, 1000.0), T14(None, "DPT 14 (4 Byte, kWh)")]
# Faktor -1 = invertiert (1.100: 1 = Heizen = Winter)
SEASON = [("DPT 1.002 (1 Bit, 1 = Sommer)", "DPST-1-2", "1 Bit", 1, 2, 1.0),
          ("DPT 1.100 (1 Bit, Heizen/Kühlen: 1 = Heizen = Winter)", "DPST-1-100", "1 Bit", 1, 2, -1.0)]

# Gruppen = ETS-Seiten
GROUPS = [
    ("Temperature", "Temperatur", "thermometer", None),
    ("Humidity", "Luftfeuchte", "water-percent", None),
    ("Comfort", "Behaglichkeit", "calculator-variant-outline", None),
    ("Wind", "Wind", "windsock", None),
    ("WindAlarm", "Windalarm", "alert-outline", None),
    ("Pressure", "Luftdruck", "gauge", None),
    ("Light", "Helligkeit und UV", "white-balance-sunny", None),
    ("Rain", "Regen", "weather-pouring", None),
    ("Particles", "Feinstaub", "blur", "sds"),
    ("Forecast", "Prognose und Display", "weather-partly-snowy-rainy", None),
    ("Sun", "Sonne und Dämmerung", "white-balance-sunny", None),
    ("MinMax", "Minimum und Maximum", "thermometer", None),
    ("Warnings", "Warnungen", "alert-outline", None),
    ("Season", "Sommer/Winter und Kenntage", "thermometer", None),
    ("Solar", "Strahlung und PV", "white-balance-sunny", None),
    ("Facades", "Fassaden", "white-balance-sunny", None),
    ("SunTimes", "Sonnenzeiten", "white-balance-sunny", None),
    ("Moon", "Mond", "weather-partly-snowy-rainy", None),
    ("Ventilation", "Lüftungsempfehlung", "water-percent", None),
    ("Irrigation", "Bewässerung", "weather-pouring", None),
    ("DegreeDays", "Gradtage", "thermometer", None),
    ("Warnings2", "Wetterwarnungen (DWD-Kriterien)", "alert-outline", None),
    ("DwdOfficial", "DWD-Warnungen (Internet)", "alert-outline", None),
]
ROOMS = 6  # Raeume 1-3 ab ETS 0.12, 4-6 ab ETS 1.5
# Seiten mit Abschnitten, die einzeln aktiviert werden: Seite -> (Anzahl, Praefix, Titel, Parameter je Abschnitt)
SECTIONS = {
    "Facades": (6, "Facade", "Fassade", ["Azimuth", "Range", "MinElev", "MaxElev", "Threshold", "OnDelay", "OffDelay"]),
    "Ventilation": (ROOMS, "Room", "Raum", ["MinDiff", "MinRh", "MinIndoorT", "MinOutdoorT"]),
}
FACADES = 6
FACADE_AZIMUTH = [90, 180, 270, 0, 135, 225]


def V(key, group, text, unit, dpts, cycle, abs_, rel, active=False, sensor=None,
      thresholds=True, decimals=1, mqtt=None, absmax=100000, info=None, dpt=0, extra=None, cond=None, influx=None, fmt="number"):
    return dict(key=key, group=group, text=text, unit=unit, dpts=dpts, cycle=cycle,
                abs=abs_, rel=rel, active=active, sensor=sensor, thresholds=thresholds,
                decimals=decimals, mqtt=mqtt or key.lower(), absmax=absmax, info=info, dpt=dpt,
                extra=extra or [], cond=cond, influx=influx, fmt=fmt)


# Reihenfolge = KO-Reihenfolge und Reihenfolge im Parameterspeicher
VALUES = [
    V("Temp", "Temperature", "Temperatur", "°C", TEMP, 300, 0.5, 0, True, mqtt="temperature", influx="temp"),
    V("Temp1W", "Temperature", "Temperatur 1-Wire", "°C", TEMP, 300, 0.5, 0, True, sensor="onewire", decimals=2,
      mqtt="temperature1", influx="temp"),
    V("Humidity", "Humidity", "Relative Luftfeuchte", "%",
      [T9(7, "DPT 9.007 (2 Byte, %)"), ("DPT 5.001 (1 Byte, %)", "DPST-5-1", "1 Byte", 5, 1, 1.0)],
      300, 2, 0, True, decimals=0, mqtt="humidity"),
    V("AbsHumidity", "Humidity", "Absolute Luftfeuchte", "g/m³", [T9(29, "DPT 9.029 (2 Byte, g/m³)")], 300, 0.5, 0,
      decimals=2, mqtt="humidity_abs"),
    V("DewPoint", "Humidity", "Taupunkt", "°C", TEMP, 300, 0.5, 0, decimals=2, mqtt="dewpoint"),
    V("FrostPoint", "Humidity", "Frostpunkt", "°C", TEMP, 300, 0.5, 0, decimals=2, mqtt="frostpoint"),
    V("Humidex", "Comfort", "Humidex", "°C", TEMP, 600, 1, 0, mqtt="humidex",
      info="Gefühlte Temperatur aus Temperatur und Taupunkt (kanadischer Index)."),
    V("HeatIndex", "Comfort", "Hitzeindex", "°C", TEMP, 600, 1, 0, mqtt="heatindex",
      info="Gefühlte Temperatur bei Wärme aus Temperatur und Luftfeuchte."),
    V("WindChill", "Comfort", "Windchill", "°C", TEMP, 600, 1, 0, mqtt="windchill",
      info="Gefühlte Temperatur bei Kälte und Wind (10-Minuten-Mittel). Gilt nur bis 10 °C und ab 5 km/h, sonst wird die Lufttemperatur gesendet."),
    V("Discomfort", "Comfort", "Discomfort-Index", "°C", TEMP, 600, 1, 0, mqtt="discomfort",
      info="Unbehaglichkeitsindex nach Thom aus Temperatur und Luftfeuchte."),
    V("WindSpeed", "Wind", "Windgeschwindigkeit", "m/s", SPEED, 60, 1, 0, True, mqtt="windspeed"),
    V("GustSpeed", "Wind", "Böengeschwindigkeit", "m/s", SPEED, 60, 1, 0, True, mqtt="gustspeed"),
    V("WindBft", "Wind", "Windstärke (Beaufort)", "Bft", BFT, 300, 1, 0, decimals=0, mqtt="windspeed_bft", absmax=12),
    V("GustBft", "Wind", "Böenstärke (Beaufort)", "Bft", BFT, 300, 1, 0, decimals=0, mqtt="gustspeed_bft", absmax=12),
    V("WindDir", "Wind", "Windrichtung", "°",
      [("DPT 5.003 (1 Byte, °)", "DPST-5-3", "1 Byte", 5, 3, 1.0), T14(7, "DPT 14.007 (4 Byte, °)")],
      60, 22.5, 0, True, decimals=0, mqtt="winddirection", absmax=359),
    V("Pressure", "Pressure", "Luftdruck (absolut)", "hPa", PRESSURE, 600, 1, 0, True, mqtt="pressure"),
    V("PressureSea", "Pressure", "Luftdruck (auf Meereshöhe)", "hPa", PRESSURE, 600, 1, 0, mqtt="pressure_sea",
      info="Benötigt die Höhe des Standorts auf der Seite Allgemein."),
    V("Trend1h", "Pressure", "Luftdrucktendenz 1 Stunde", "hPa", PRESSURE, 900, 0.5, 0, mqtt="pressuretrend1", influx="pressureTrend1",
      info="Differenz zum Luftdruck vor einer Stunde; verfügbar frühestens eine Stunde nach dem Start."),
    V("Trend3h", "Pressure", "Luftdrucktendenz 3 Stunden", "hPa", PRESSURE, 900, 0.5, 0, mqtt="pressuretrend3", influx="pressureTrend3",
      info="Differenz zum Luftdruck vor drei Stunden; verfügbar frühestens drei Stunden nach dem Start."),
    V("Light", "Light", "Helligkeit", "lx", [T9(4, "DPT 9.004 (2 Byte, lx)")], 300, 0, 20, True, decimals=0,
      mqtt="light", absmax=300000, influx="brightness"),
    V("UvIndex", "Light", "UV-Index", "",
      [("DPT 9 (2 Byte)", "DPT-9", "2 Bytes", 9, 1, 1.0), ("DPT 5.010 (1 Byte, ganzzahlig)", "DPST-5-10", "1 Byte", 5, 10, 1.0)],
      300, 1, 0, True, mqtt="uvindex", absmax=15),
    V("RainTotal", "Rain", "Regenmenge gesamt", "mm", RAIN, 3600, 1, 0, True, decimals=2, mqtt="raincounter", dpt=1,
      info="Fortlaufender Zähler seit Inbetriebnahme, bleibt bei Neustart und Update erhalten. "
           "DPT 14 empfohlen: DPT 9 wird bei großen Werten grob (ab 1000 mm in Schritten von 0,64 mm)."),
    V("RainRate", "Rain", "Regenintensität", "mm/h", [("DPT 9 (2 Byte, mm/h)", "DPT-9", "2 Bytes", 9, 1, 1.0)],
      600, 1, 0, True, decimals=1, mqtt="rain_rate",
      info="Hochgerechnet aus der Regenmenge der letzten 10 Minuten."),
    V("Rain5m", "Rain", "Regenmenge letzte 5 Minuten", "mm", RAIN9, 300, 0.2, 0, decimals=2, mqtt="rain_5min", influx="raincounter_5min"),
    V("Rain1h", "Rain", "Regenmenge letzte Stunde", "mm", RAIN9, 600, 0.5, 0, True, decimals=2, mqtt="rain_1h", influx="raincounter_1h"),
    V("Rain6h", "Rain", "Regenmenge letzte 6 Stunden", "mm", RAIN9, 1800, 1, 0, decimals=2, mqtt="rain_6h", influx="raincounter_6h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("Rain24h", "Rain", "Regenmenge letzte 24 Stunden", "mm", RAIN9, 3600, 1, 0, True, decimals=2, mqtt="rain_24h", influx="raincounter_24h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("Rain48h", "Rain", "Regenmenge letzte 48 Stunden", "mm", RAIN9, 3600, 1, 0, decimals=2, mqtt="rain_48h", influx="raincounter_48h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("Rain72h", "Rain", "Regenmenge letzte 72 Stunden", "mm", RAIN9, 3600, 1, 0, decimals=2, mqtt="rain_72h", influx="raincounter_72h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("RainToday", "Rain", "Regenmenge heute", "mm", RAIN9, 3600, 0.5, 0, True, decimals=2, mqtt="rain_today", influx="raincounter_today",
      info="Wird um Mitternacht zurückgesetzt; benötigt eine gültige Uhrzeit (KO Uhrzeit/Datum oder NTP)."),
    V("Raining", "Rain", "Regen", "", BOOL, 600, 0, 0, True, thresholds=False, decimals=0, mqtt="raining",
      info="1 = es regnet: Mindestmenge im Zeitfenster erreicht (Seite Allgemein). Wird bei jeder Änderung gesendet."),
    V("Pm25", "Particles", "Feinstaub PM2,5", "µg/m³", PM, 300, 2, 0, True, sensor="sds", mqtt="pm25"),
    V("Pm10", "Particles", "Feinstaub PM10", "µg/m³", PM, 300, 2, 0, True, sensor="sds", mqtt="pm10"),
    V("Pm25Norm", "Particles", "Feinstaub PM2,5 (feuchtekorrigiert)", "µg/m³", PM, 300, 2, 0, sensor="sds",
      mqtt="pm25_normalized"),
    V("Pm10Norm", "Particles", "Feinstaub PM10 (feuchtekorrigiert)", "µg/m³", PM, 300, 2, 0, sensor="sds",
      mqtt="pm10_normalized"),
    # Ab ETS 0.3, hinten angefuegt, damit die KO-Nummern der bisherigen Werte gleich bleiben
    V("Tendency", "Forecast", "Wettertendenz", "", TENDENCY, 1800, 0, 0, True, thresholds=False, decimals=0,
      mqtt="weather_tendency",
      info="Aus der Luftdruckänderung über 3 Stunden: -2 stark fallend (Sturm möglich), -1 fallend (Verschlechterung), "
           "0 gleichbleibend, +1 steigend (Besserung), +2 stark steigend. In den ersten 3 Stunden nach dem Start aus 1 Stunde hochgerechnet."),
    V("Forecast", "Forecast", "Wetterprognose (Zambretti-Nummer)", "", BFT, 1800, 0, 0, thresholds=False, decimals=0,
      mqtt="forecast",
      info="Prognose für etwa 12 Stunden nach Zambretti aus Luftdruck auf Meereshöhe, Tendenz, Windrichtung und Jahreszeit: "
           "1 (A, beständig schön) bis 26 (Z, Sturm mit viel Regen). Die Höhe des Standorts auf der Seite Allgemein muss stimmen."),
    V("ForecastText", "Forecast", "Wetterprognose (Text)", "", TEXT, 1800, 0, 0, True, thresholds=False, decimals=0,
      mqtt="forecast_text", influx="", fmt="forecast", info="Kurztext der Zambretti-Prognose, z. B. „Wechsel,Regen“."),
    V("WeatherSymbol", "Forecast", "Wettersymbol (Display)", "", BFT, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="weather_symbol",
      info="0 sonnig, 1 bewölkt, 2 Schauer, 3 starker Regen, 4 Gewitter, 5 hoher UV-Index (z. B. für Tantron-Displays). "
           "Gewitter ist eine Schätzung aus starkem Regen und Böen, die WN90LP misst keine Blitze."),
    # Ab ETS 0.5
    V("WindAvg10", "Wind", "Windgeschwindigkeit (10-min-Mittel)", "m/s", SPEED, 300, 1, 0, True, mqtt="windspeed_avg10",
      info="Gleitendes Mittel der letzten 10 Minuten (meteorologischer Standard)."),
    V("GustMax10", "Wind", "Böe (Maximum 10 min)", "m/s", SPEED, 300, 1, 0, True, mqtt="gust_max10",
      info="Stärkste Böe der letzten 10 Minuten."),
    V("WindAlarm1", "WindAlarm", "Windalarm Stufe 1", "", ALARM, 300, 0, 0, True, thresholds=False, decimals=0,
      mqtt="wind_alarm1", extra=["Alarm1Source", "Alarm1Threshold", "Alarm1OnDelay", "Alarm1OffDelay"]),
    V("WindAlarm2", "WindAlarm", "Windalarm Stufe 2", "", ALARM, 300, 0, 0, False, thresholds=False, decimals=0,
      mqtt="wind_alarm2", extra=["Alarm2Source", "Alarm2Threshold", "Alarm2OnDelay", "Alarm2OffDelay"]),
    V("WindAlarm3", "WindAlarm", "Windalarm Stufe 3", "", ALARM, 300, 0, 0, False, thresholds=False, decimals=0,
      mqtt="wind_alarm3", extra=["Alarm3Source", "Alarm3Threshold", "Alarm3OnDelay", "Alarm3OffDelay"]),
    V("SunAzimuth", "Sun", "Sonnenstand Azimut", "°", ANGLE, 300, 2, 0, mqtt="sun_azimuth",
      info="0° = Nord, 90° = Ost. Benötigt Gerätestandort und gültige Uhrzeit."),
    V("SunElevation", "Sun", "Sonnenstand Elevation", "°", ANGLE, 300, 1, 0, mqtt="sun_elevation",
      info="Höhe über dem Horizont, negativ nachts. Benötigt Gerätestandort und gültige Uhrzeit."),
    V("DayNight", "Sun", "Tag/Nacht (Dämmerung)", "", DAYNIGHT, 1800, 0, 0, True, thresholds=False, decimals=0,
      mqtt="night", extra=["TwilightNight", "TwilightDay", "TwilightDelay"],
      info="Aus der Helligkeit: Nacht unterhalb der unteren, Tag oberhalb der oberen Schwelle, jeweils nach der Verzögerung."),
    V("TempMinToday", "MinMax", "Temperatur Minimum heute", "°C", TEMP, 3600, 0, 0, True, thresholds=False, mqtt="temp_min_today"),
    V("TempMaxToday", "MinMax", "Temperatur Maximum heute", "°C", TEMP, 3600, 0, 0, True, thresholds=False, mqtt="temp_max_today"),
    V("TempMinYesterday", "MinMax", "Temperatur Minimum gestern", "°C", TEMP, 3600, 0, 0, thresholds=False,
      mqtt="temp_min_yesterday"),
    V("TempMaxYesterday", "MinMax", "Temperatur Maximum gestern", "°C", TEMP, 3600, 0, 0, thresholds=False,
      mqtt="temp_max_yesterday"),
    V("GustMaxToday", "MinMax", "Stärkste Böe heute", "m/s", SPEED, 3600, 0, 0, True, thresholds=False, mqtt="gust_max_today"),
    V("GustMaxYesterday", "MinMax", "Stärkste Böe gestern", "m/s", SPEED, 3600, 0, 0, thresholds=False,
      mqtt="gust_max_yesterday"),
    V("Frost", "Warnings", "Frost", "", ALARM, 1800, 0, 0, True, thresholds=False, decimals=0, mqtt="frost",
      info="1 bei Temperatur unter 0 °C, zurück ab +0,5 °C (Temperatur für Berechnungen)."),
    V("IceWarning", "Warnings", "Glättegefahr", "", ALARM, 1800, 0, 0, True, thresholds=False, decimals=0,
      mqtt="ice_warning", extra=["IceThreshold"],
      info="1, wenn die Temperatur unter der Schwelle liegt und Feuchtigkeit vorhanden ist: Regen in der letzten Stunde "
           "oder Taupunkt höchstens 1 K unter der Temperatur. Zurück, wenn die Temperatur 1 K über der Schwelle liegt "
           "oder 30 Minuten keine Feuchtigkeit mehr vorhanden war."),
    # Ab ETS 0.6
    V("SummerWinter", "Season", "Sommer/Winter", "", SEASON, 3600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="summer", extra=["SwMethod", "SwEvening", "SwDays", "SwThreshold", "SwHysteresis"],
      info="Sommer, wenn das Mittel der Tagesmitteltemperaturen der letzten Tage über der Schwelle liegt, sonst Winter. "
           "Bewertet wird einmal täglich um Mitternacht; der erste Wert liegt nach dem ersten vollständigen Tag vor."),
    V("MeanTemp", "Season", "Mittlere Außentemperatur (mehrere Tage)", "°C", TEMP, 3600, 0.5, 0, True, mqtt="temp_mean_days",
      info="Gleitendes Mittel der Tagesmittel über die eingestellte Anzahl Tage (Grundlage der Sommer/Winter-Umschaltung)."),
    V("DailyMeanYesterday", "Season", "Tagesmitteltemperatur gestern", "°C", TEMP, 3600, 0, 0, thresholds=False,
      mqtt="temp_mean_yesterday"),
    V("SummerDay", "Season", "Sommertag (Maximum ab 25 °C)", "", BOOL, 3600, 0, 0, thresholds=False, decimals=0,
      mqtt="summer_day"),
    V("HotDay", "Season", "Heißer Tag (Maximum ab 30 °C)", "", BOOL, 3600, 0, 0, thresholds=False, decimals=0, mqtt="hot_day"),
    V("FrostDay", "Season", "Frosttag (Minimum unter 0 °C)", "", BOOL, 3600, 0, 0, thresholds=False, decimals=0,
      mqtt="frost_day"),
    V("IceDay", "Season", "Eistag (Maximum unter 0 °C)", "", BOOL, 3600, 0, 0, thresholds=False, decimals=0, mqtt="ice_day",
      info="Kenntage gelten für den laufenden Tag nach dem bisherigen Minimum/Maximum; der Eistag steht erst am Tagesende "
           "endgültig fest."),
    # Ab ETS 0.7
    V("GlobalRadiation", "Solar", "Globalstrahlung (geschätzt)", "W/m²", IRRADIANCE, 300, 50, 0, True, decimals=0,
      mqtt="global_radiation", extra=["RadFactor", "Albedo"],
      info="Aus der Helligkeit geschätzt (ca. 110 lx je W/m²), typische Abweichung 15–20 %. Mit dem Korrekturfaktor an "
           "eine Referenz angleichen. Die Albedo wirkt nur auf Fassaden und geneigte PV-Flächen."),
    V("CloudCover", "Solar", "Bewölkungsgrad (geschätzt)", "%", [("DPT 5.001 (1 Byte, %)", "DPST-5-1", "1 Byte", 5, 1, 1.0)],
      600, 10, 0, decimals=0, mqtt="cloud_cover",
      info="Abweichung der Strahlung vom Wert bei klarem Himmel, nur tagsüber (Sonnenhöhe über 5°). Benötigt Uhrzeit und Gerätestandort."),
    V("RadiationToday", "Solar", "Einstrahlung heute", "kWh/m²", [T14(None, "DPT 14 (4 Byte, kWh/m²)")], 3600, 0.1, 0, decimals=2,
      mqtt="radiation_today"),
    V("RadiationYesterday", "Solar", "Einstrahlung gestern", "kWh/m²", [T14(None, "DPT 14 (4 Byte, kWh/m²)")], 3600, 0, 0,
      thresholds=False, decimals=2, mqtt="radiation_yesterday"),
    V("PvPower", "Solar", "PV-Leistung (geschätzt)", "W", [("DPT 14.056 (4 Byte, W)", "DPST-14-56", "4 Bytes", 14, 0, 1.0)],
      300, 100, 0, decimals=0, mqtt="pv_power", extra=["PvPeak", "PvTilt", "PvAzimuth", "PvPr"],
      info="Aus der Globalstrahlung auf die Modulfläche umgerechnet; typische Abweichung 20–30 %. Keine Verschattung, "
           "kein Temperatureinfluss."),
    V("PvEnergyToday", "Solar", "PV-Ertrag heute (geschätzt)", "kWh", ENERGY, 3600, 0.1, 0, decimals=2, mqtt="pv_energy_today",
      info="Benötigt den aktiven Wert PV-Leistung (Parameter der Anlage)."),
    V("PvEnergyYesterday", "Solar", "PV-Ertrag gestern (geschätzt)", "kWh", ENERGY, 3600, 0, 0, thresholds=False, decimals=2,
      mqtt="pv_energy_yesterday"),
    V("Facade1Sun", "Facades", "Sonne auf Fassade 1", "", BOOL, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="facade1_sun", cond="Facade1Active"),
    V("Facade1Irradiance", "Facades", "Einstrahlung Fassade 1 (geschätzt)", "W/m²", IRRADIANCE, 300, 50, 0, True, decimals=0,
      mqtt="facade1_irradiance", cond="Facade1Active"),
    V("Facade1Profile", "Facades", "Profilwinkel Fassade 1", "°", ANGLE, 300, 2, 0, mqtt="facade1_profile",
      cond="Facade1Active",
      info="Sonnenhöhe in der Ebene senkrecht zur Fassade (Lamellennachführung); nur gültig, solange die Sonne auf die Fassade scheint."),
    V("Facade2Sun", "Facades", "Sonne auf Fassade 2", "", BOOL, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="facade2_sun", cond="Facade2Active"),
    V("Facade2Irradiance", "Facades", "Einstrahlung Fassade 2 (geschätzt)", "W/m²", IRRADIANCE, 300, 50, 0, True, decimals=0,
      mqtt="facade2_irradiance", cond="Facade2Active"),
    V("Facade2Profile", "Facades", "Profilwinkel Fassade 2", "°", ANGLE, 300, 2, 0, mqtt="facade2_profile",
      cond="Facade2Active",
      info="Sonnenhöhe in der Ebene senkrecht zur Fassade (Lamellennachführung); nur gültig, solange die Sonne auf die Fassade scheint."),
    V("Facade3Sun", "Facades", "Sonne auf Fassade 3", "", BOOL, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="facade3_sun", cond="Facade3Active"),
    V("Facade3Irradiance", "Facades", "Einstrahlung Fassade 3 (geschätzt)", "W/m²", IRRADIANCE, 300, 50, 0, True, decimals=0,
      mqtt="facade3_irradiance", cond="Facade3Active"),
    V("Facade3Profile", "Facades", "Profilwinkel Fassade 3", "°", ANGLE, 300, 2, 0, mqtt="facade3_profile",
      cond="Facade3Active",
      info="Sonnenhöhe in der Ebene senkrecht zur Fassade (Lamellennachführung); nur gültig, solange die Sonne auf die Fassade scheint."),
    V("Facade4Sun", "Facades", "Sonne auf Fassade 4", "", BOOL, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="facade4_sun", cond="Facade4Active"),
    V("Facade4Irradiance", "Facades", "Einstrahlung Fassade 4 (geschätzt)", "W/m²", IRRADIANCE, 300, 50, 0, True, decimals=0,
      mqtt="facade4_irradiance", cond="Facade4Active"),
    V("Facade4Profile", "Facades", "Profilwinkel Fassade 4", "°", ANGLE, 300, 2, 0, mqtt="facade4_profile",
      cond="Facade4Active",
      info="Sonnenhöhe in der Ebene senkrecht zur Fassade (Lamellennachführung); nur gültig, solange die Sonne auf die Fassade scheint."),
    V("Facade5Sun", "Facades", "Sonne auf Fassade 5", "", BOOL, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="facade5_sun", cond="Facade5Active"),
    V("Facade5Irradiance", "Facades", "Einstrahlung Fassade 5 (geschätzt)", "W/m²", IRRADIANCE, 300, 50, 0, True, decimals=0,
      mqtt="facade5_irradiance", cond="Facade5Active"),
    V("Facade5Profile", "Facades", "Profilwinkel Fassade 5", "°", ANGLE, 300, 2, 0, mqtt="facade5_profile",
      cond="Facade5Active",
      info="Sonnenhöhe in der Ebene senkrecht zur Fassade (Lamellennachführung); nur gültig, solange die Sonne auf die Fassade scheint."),
    V("Facade6Sun", "Facades", "Sonne auf Fassade 6", "", BOOL, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="facade6_sun", cond="Facade6Active"),
    V("Facade6Irradiance", "Facades", "Einstrahlung Fassade 6 (geschätzt)", "W/m²", IRRADIANCE, 300, 50, 0, True, decimals=0,
      mqtt="facade6_irradiance", cond="Facade6Active"),
    V("Facade6Profile", "Facades", "Profilwinkel Fassade 6", "°", ANGLE, 300, 2, 0, mqtt="facade6_profile",
      cond="Facade6Active",
      info="Sonnenhöhe in der Ebene senkrecht zur Fassade (Lamellennachführung); nur gültig, solange die Sonne auf die Fassade scheint."),
    # Ab ETS 0.9 (Sonnenzeiten nach dem EDOMI-LBS "Sonnenstand", Mond)
    V("Sunrise", "SunTimes", "Sonnenaufgang", "", TIME, 0, 0, 0, True, thresholds=False, decimals=0, mqtt="sunrise", influx="", fmt="time"),
    V("Sunset", "SunTimes", "Sonnenuntergang", "", TIME, 0, 0, 0, True, thresholds=False, decimals=0, mqtt="sunset", influx="", fmt="time"),
    V("SolarNoon", "SunTimes", "Sonnenhöchststand (Transit)", "", TIME, 0, 0, 0, False, thresholds=False, decimals=0, mqtt="solar_noon", influx="", fmt="time"),
    V("CivilDawn", "SunTimes", "Beginn bürgerliche Dämmerung", "", TIME, 0, 0, 0, False, thresholds=False, decimals=0, mqtt="civil_dawn", influx="", fmt="time"),
    V("CivilDusk", "SunTimes", "Ende bürgerliche Dämmerung", "", TIME, 0, 0, 0, False, thresholds=False, decimals=0, mqtt="civil_dusk", influx="", fmt="time"),
    V("NauticalDawn", "SunTimes", "Beginn nautische Dämmerung", "", TIME, 0, 0, 0, False, thresholds=False, decimals=0, mqtt="nautical_dawn", influx="", fmt="time"),
    V("NauticalDusk", "SunTimes", "Ende nautische Dämmerung", "", TIME, 0, 0, 0, False, thresholds=False, decimals=0, mqtt="nautical_dusk", influx="", fmt="time"),
    V("AstroDawn", "SunTimes", "Beginn astronomische Dämmerung", "", TIME, 0, 0, 0, False, thresholds=False, decimals=0, mqtt="astro_dawn", influx="", fmt="time"),
    V("AstroDusk", "SunTimes", "Ende astronomische Dämmerung", "", TIME, 0, 0, 0, False, thresholds=False, decimals=0, mqtt="astro_dusk", influx="", fmt="time"),
    V("SunriseAzimuth", "SunTimes", "Azimut bei Sonnenaufgang", "°", ANGLE, 0, 0, 0, thresholds=False, mqtt="sunrise_azimuth"),
    V("SunsetAzimuth", "SunTimes", "Azimut bei Sonnenuntergang", "°", ANGLE, 0, 0, 0, thresholds=False, mqtt="sunset_azimuth"),
    V("TransitElevation", "SunTimes", "Elevation beim Höchststand", "°", ANGLE, 0, 0, 0, thresholds=False, mqtt="transit_elevation"),
    V("DayLength", "SunTimes", "Tageslänge", "s", DURATION, 0, 0, 0, thresholds=False, decimals=0, mqtt="day_length", fmt="duration"),
    V("DayProgress", "SunTimes", "Tagesfortschritt (Aufgang bis Untergang)", "%", PERCENT, 0, 5, 0, decimals=0, mqtt="day_progress",
      info="0 % bei Sonnenaufgang, 100 % bei Sonnenuntergang (z. B. für die Positionierung eines Sonnensymbols in der Visualisierung)."),
    V("SunFalling", "SunTimes", "Sonne steigt (0) / fällt (1)", "", BOOL, 0, 0, 0, thresholds=False, decimals=0, mqtt="sun_falling"),
    V("DayNightSun", "SunTimes", "Tag/Nacht (Sonnenstand)", "", DAYNIGHT, 1800, 0, 0, True, thresholds=False, decimals=0,
      mqtt="night_sun", extra=["SunDnMode", "SunDnOffsetRise", "SunDnOffsetSet", "SunDnElevRise", "SunDnElevSet"],
      info="Nach Sonnenauf-/untergang mit Versatz oder nach Elevationsschwellen (morgens Tag ab, abends Nacht unter). "
           "Die Versätze gelten nur im ersten, die Schwellen nur im zweiten Verfahren."),
    V("DayNightCivil", "SunTimes", "Tag/Nacht (bürgerliche Dämmerung, -6°)", "", DAYNIGHT, 1800, 0, 0, thresholds=False, decimals=0,
      mqtt="night_civil"),
    V("DayNightNautical", "SunTimes", "Tag/Nacht (nautische Dämmerung, -12°)", "", DAYNIGHT, 1800, 0, 0, thresholds=False, decimals=0,
      mqtt="night_nautical"),
    V("DayNightAstro", "SunTimes", "Tag/Nacht (astronomische Dämmerung, -18°)", "", DAYNIGHT, 1800, 0, 0, thresholds=False, decimals=0,
      mqtt="night_astro"),
    V("ClearSkyRadiation", "SunTimes", "Globalstrahlung bei klarem Himmel", "W/m²", IRRADIANCE, 300, 50, 0, decimals=0,
      mqtt="clear_sky_radiation", info="Theoretischer Wert für den aktuellen Sonnenstand (Haurwitz)."),
    V("ClearSkyMax", "SunTimes", "Globalstrahlung bei klarem Himmel, Tagesmaximum", "W/m²", IRRADIANCE, 0, 0, 0, thresholds=False, decimals=0,
      mqtt="clear_sky_max"),
    V("SunshineToday", "SunTimes", "Sonnenscheindauer heute", "h", HOURS, 3600, 0.1, 0, decimals=2, mqtt="sunshine_today",
      extra=["SunshineThreshold"], info="Zeit mit Globalstrahlung über der Schwelle (WMO: 120 W/m²), bleibt über Neustarts erhalten."),
    V("SunshineYesterday", "SunTimes", "Sonnenscheindauer gestern", "h", HOURS, 0, 0, 0, thresholds=False, decimals=2,
      mqtt="sunshine_yesterday"),
    V("MoonPhase", "Moon", "Mondphase (0-7)", "", BFT, 0, 0, 0, True, thresholds=False, decimals=0, mqtt="moon_phase", fmt="moonphase",
      info="0 Neumond, 1 zunehmende Sichel, 2 erstes Viertel, 3 zunehmender Mond, 4 Vollmond, 5 abnehmender Mond, "
           "6 letztes Viertel, 7 abnehmende Sichel."),
    V("MoonPhaseText", "Moon", "Mondphase (Text)", "", TEXT, 0, 0, 0, thresholds=False, decimals=0, mqtt="moon_phase_text",
      influx="", fmt="moontext"),
    V("MoonIllumination", "Moon", "Mond beleuchtet", "%", PERCENT, 3600, 2, 0, True, decimals=0, mqtt="moon_illumination"),
    V("MoonAge", "Moon", "Mondalter (Tage seit Neumond)", "d", [("DPT 9 (2 Byte, Tage)", "DPT-9", "2 Bytes", 9, 1, 1.0)], 3600, 0.5, 0,
      decimals=1, mqtt="moon_age"),
    V("MoonAzimuth", "Moon", "Mond Azimut", "°", ANGLE, 600, 5, 0, mqtt="moon_azimuth"),
    V("MoonElevation", "Moon", "Mond Elevation", "°", ANGLE, 600, 2, 0, mqtt="moon_elevation"),
    V("Moonrise", "Moon", "Mondaufgang", "", TIME, 0, 0, 0, thresholds=False, decimals=0, mqtt="moonrise", influx="", fmt="time",
      info="Leer, wenn der Mond an diesem Tag nicht aufgeht."),
    V("Moonset", "Moon", "Monduntergang", "", TIME, 0, 0, 0, thresholds=False, decimals=0, mqtt="moonset", influx="", fmt="time"),
    V("NextFullMoon", "Moon", "Nächster Vollmond", "", DATE, 0, 0, 0, True, thresholds=False, decimals=0, mqtt="next_full_moon",
      influx="", fmt="date"),
    V("NextNewMoon", "Moon", "Nächster Neumond", "", DATE, 0, 0, 0, thresholds=False, decimals=0, mqtt="next_new_moon", influx="",
      fmt="date"),
    # Ab ETS 0.12: Lueftungsempfehlung (Raeume in der Schleife unten), Bewaesserung, Gradtage
    *[v for _n in range(1, ROOMS + 1) for v in (
        V(f"Room{_n}Vent", "Ventilation", f"Lüften sinnvoll Raum {_n}", "", BOOL, 1800, 0, 0, True, thresholds=False, decimals=0,
          mqtt=f"room{_n}_ventilate", cond=f"Room{_n}Active"),
        V(f"Room{_n}AbsHum", "Ventilation", f"Absolute Feuchte Raum {_n}", "g/m³", [T9(29, "DPT 9.029 (2 Byte, g/m³)")], 1800, 0.5,
          0, decimals=2, mqtt=f"room{_n}_humidity_abs", cond=f"Room{_n}Active"),
        V(f"Room{_n}DewPoint", "Ventilation", f"Taupunkt Raum {_n}", "°C", TEMP, 1800, 0.5, 0, decimals=2,
          mqtt=f"room{_n}_dewpoint", cond=f"Room{_n}Active"),
        V(f"Room{_n}Diff", "Ventilation", f"Feuchteunterschied innen − außen Raum {_n}", "g/m³", [T9(29, "DPT 9.029 (2 Byte, g/m³)")],
          1800, 0.5, 0, decimals=2, mqtt=f"room{_n}_humidity_diff", cond=f"Room{_n}Active",
          info="Positiv: draußen ist die Luft trockener, Lüften senkt die Feuchte im Raum."),
    )],
    V("Et0Today", "Irrigation", "Verdunstung heute (ET0)", "mm", RAIN9, 3600, 0.5, 0, True, decimals=2, mqtt="et0_today",
      info="Referenzverdunstung nach FAO-56 Penman-Monteith, laufend aus Temperatur, Feuchte, Wind, Strahlung und Luftdruck."),
    V("Et0Yesterday", "Irrigation", "Verdunstung gestern (ET0)", "mm", RAIN9, 3600, 0, 0, thresholds=False, decimals=2,
      mqtt="et0_yesterday"),
    V("WaterBalance", "Irrigation", "Wasserbilanz", "mm", [("DPT 9 (2 Byte, mm)", "DPT-9", "2 Bytes", 9, 1, 1.0)], 3600, 1, 0, True,
      decimals=1, mqtt="water_balance",
      info="Regen minus Verdunstung (mit Pflanzenfaktor) plus Bewässerung; negativ = Defizit. Nach oben durch den Bodenspeicher begrenzt."),
    V("Irrigate", "Irrigation", "Bewässern", "", BOOL, 3600, 0, 0, True, thresholds=False, decimals=0, mqtt="irrigate",
      info="1, sobald das Defizit die Schwelle erreicht; 0, wenn es wieder unter der halben Schwelle liegt."),
    V("HddMonth", "DegreeDays", "Heizgradtage laufender Monat", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, True, thresholds=False, decimals=1, mqtt="hdd_month"),
    V("HddLastMonth", "DegreeDays", "Heizgradtage Vormonat", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, False, thresholds=False, decimals=1, mqtt="hdd_last_month"),
    V("HddYear", "DegreeDays", "Heizgradtage laufendes Jahr", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, True, thresholds=False, decimals=1, mqtt="hdd_year"),
    V("HddLastYear", "DegreeDays", "Heizgradtage Vorjahr", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, False, thresholds=False, decimals=1, mqtt="hdd_last_year"),
    V("CddMonth", "DegreeDays", "Kühlgradtage laufender Monat", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, True, thresholds=False, decimals=1, mqtt="cdd_month"),
    V("CddLastMonth", "DegreeDays", "Kühlgradtage Vormonat", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, False, thresholds=False, decimals=1, mqtt="cdd_last_month"),
    V("CddYear", "DegreeDays", "Kühlgradtage laufendes Jahr", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, True, thresholds=False, decimals=1, mqtt="cdd_year"),
    V("CddLastYear", "DegreeDays", "Kühlgradtage Vorjahr", "Kd", [("DPT 9 (2 Byte, Kd)", "DPT-9", "2 Bytes", 9, 1, 1.0), T14(None, "DPT 14 (4 Byte, Kd)")],
      0, 0, 0, False, thresholds=False, decimals=1, mqtt="cdd_last_year"),
    # Ab ETS 0.13: Wetterwarnungen nach DWD-Kriterien aus eigenen Messwerten
    V("WarnLevel", "Warnings2", "Warnstufe gesamt (0-4)", "", BFT, 3600, 0, 0, True, thresholds=False, decimals=0, mqtt="warn_level",
      info="0 keine Warnung, 1 Wetterwarnung, 2 markantes Wetter, 3 Unwetter, 4 extremes Unwetter (höchste Stufe aller Kategorien)."),
    V("WarnText", "Warnings2", "Warnung (Text)", "", TEXT, 3600, 0, 0, True, thresholds=False, decimals=0, mqtt="warn_text", influx="",
      fmt="warntext", info="Bezeichnung der höchsten Warnung, z. B. „Sturmböen“ oder „Starkregen“."),
    V("WarnSevere", "Warnings2", "Unwetter (Stufe 3 oder 4)", "", ALARM, 3600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="warn_severe"),
    V("WarnWind", "Warnings2", "Warnstufe Wind", "", BFT, 3600, 0, 0, thresholds=False, decimals=0, mqtt="warn_wind"),
    V("WarnHeavyRain", "Warnings2", "Warnstufe Starkregen", "", BFT, 3600, 0, 0, thresholds=False, decimals=0, mqtt="warn_heavy_rain"),
    V("WarnLongRain", "Warnings2", "Warnstufe Dauerregen", "", BFT, 3600, 0, 0, thresholds=False, decimals=0, mqtt="warn_long_rain"),
    V("WarnFrost", "Warnings2", "Warnstufe Frost/Glätte", "", BFT, 3600, 0, 0, thresholds=False, decimals=0, mqtt="warn_frost"),
    V("WarnHeat", "Warnings2", "Warnstufe Hitze", "", BFT, 3600, 0, 0, thresholds=False, decimals=0, mqtt="warn_heat"),
    V("WarnUv", "Warnings2", "Warnstufe UV", "", BFT, 3600, 0, 0, thresholds=False, decimals=0, mqtt="warn_uv"),
    # Ab ETS 0.14: amtliche DWD-Warnungen fuer eine Warnzelle (Internet)
    V("DwdLevel", "DwdOfficial", "DWD-Warnstufe aktuell (0-4)", "", BFT, 3600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="dwd_level", info="Höchste derzeit gültige amtliche Warnung: 1 Wetterwarnung, 2 markant, 3 Unwetter, 4 extremes Unwetter."),
    V("DwdLevelUpcoming", "DwdOfficial", "DWD-Warnstufe inkl. angekündigter Warnungen", "", BFT, 3600, 0, 0, thresholds=False,
      decimals=0, mqtt="dwd_level_upcoming", info="Höchste Stufe aller vorliegenden Warnungen, auch solcher, die erst später beginnen."),
    V("DwdEvent", "DwdOfficial", "DWD-Warnung (Ereignis)", "", TEXT, 3600, 0, 0, True, thresholds=False, decimals=0, mqtt="dwd_event",
      influx="", fmt="dwdtext", info="Ereignis der höchsten gültigen Warnung, z. B. „STURMBÖEN“ (max. 14 Zeichen)."),
    V("DwdSevere", "DwdOfficial", "DWD-Unwetterwarnung (Stufe 3 oder 4)", "", ALARM, 3600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="dwd_severe"),
    V("DwdCount", "DwdOfficial", "Anzahl gültiger DWD-Warnungen", "", BFT, 3600, 0, 0, thresholds=False, decimals=0, mqtt="dwd_count"),
    V("DwdFault", "DwdOfficial", "Störung DWD-Abruf", "", ALARM, 3600, 0, 0, True, thresholds=False, decimals=0, mqtt="dwd_fault",
      info="1, wenn seit drei Abrufintervallen keine Daten vom DWD kamen (Internet, DNS oder Server)."),
    # Regensummen nach Kalender (ab 1.1); hinten angefuegt, damit bestehende KO-Nummern bleiben
    V("RainYesterday", "Rain", "Regenmenge gestern", "mm", RAIN9, 3600, 0.5, 0, True, decimals=2, mqtt="rain_yesterday",
      influx="raincounter_yesterday",
      info="Tagessumme des Vortags (0-24 Uhr Ortszeit), wird um Mitternacht übernommen und bleibt über Neustarts erhalten. "
           "Ungültig, solange noch kein vollständiger Tag gemessen wurde oder die Station über Mitternacht aus war."),
    V("RainWeek", "Rain", "Regenmenge diese Woche", "mm", RAIN, 3600, 1, 0, decimals=1, mqtt="rain_week", influx="raincounter_week",
      info="Seit Montag 0 Uhr. Im ersten Zeitraum nach der Inbetriebnahme nur seit dem Start."),
    V("RainMonth", "Rain", "Regenmenge dieser Monat", "mm", RAIN, 3600, 1, 0, decimals=1, mqtt="rain_month", influx="raincounter_month",
      info="Seit dem Monatsersten 0 Uhr. Im ersten Zeitraum nach der Inbetriebnahme nur seit dem Start."),
    V("RainYear", "Rain", "Regenmenge dieses Jahr", "mm", RAIN, 3600, 5, 0, decimals=1, mqtt="rain_year", influx="raincounter_year",
      info="Seit dem 1. Januar 0 Uhr. Im ersten Zeitraum nach der Inbetriebnahme nur seit dem Start. DPT 9 löst große Werte "
           "grob auf (bei 1000 mm etwa 0,6 mm), DPT 14 ist genau."),
    # Feinstaub-Auswertung (ab 1.2): Mittelwerte, WHO-Richtwerte, Luftqualitaetsindex, Ampel
    V("Pm25Mean1h", "Particles", "PM2,5 Mittel 1 Stunde", "µg/m³", PM, 600, 2, 0, sensor="sds", mqtt="pm25_1h",
      info="Gleitendes Mittel der letzten Stunde."),
    V("Pm10Mean1h", "Particles", "PM10 Mittel 1 Stunde", "µg/m³", PM, 600, 2, 0, sensor="sds", mqtt="pm10_1h",
      info="Gleitendes Mittel der letzten Stunde."),
    V("Pm25Mean24h", "Particles", "PM2,5 Mittel 24 Stunden", "µg/m³", PM, 1800, 1, 0, True, sensor="sds", mqtt="pm25_24h",
      info="Gleitendes Mittel der letzten 24 Stundenmittel; gültig, sobald 18 Stunden mit Messwerten vorliegen. Grundlage für WHO-Richtwert und Luftqualitätsindex."),
    V("Pm10Mean24h", "Particles", "PM10 Mittel 24 Stunden", "µg/m³", PM, 1800, 1, 0, True, sensor="sds", mqtt="pm10_24h",
      info="Gleitendes Mittel der letzten 24 Stundenmittel; gültig, sobald 18 Stunden mit Messwerten vorliegen."),
    V("Pm25Yesterday", "Particles", "PM2,5 Tagesmittel gestern", "µg/m³", PM, 3600, 1, 0, sensor="sds", mqtt="pm25_yesterday",
      info="Mittel des Vortags (0-24 Uhr Ortszeit); ungültig, wenn die Station über Mitternacht aus war."),
    V("Pm10Yesterday", "Particles", "PM10 Tagesmittel gestern", "µg/m³", PM, 3600, 1, 0, sensor="sds", mqtt="pm10_yesterday",
      info="Mittel des Vortags (0-24 Uhr Ortszeit); ungültig, wenn die Station über Mitternacht aus war."),
    V("Pm25YearMean", "Particles", "PM2,5 Jahresmittel (laufendes Jahr)", "µg/m³", PM, 3600, 0.5, 0, sensor="sds", mqtt="pm25_year",
      info="Mittel der Tagesmittel seit 1. Januar. WHO-Leitwert 5 µg/m³, EU-Grenzwert 25 µg/m³ (ab 2030: 10)."),
    V("Pm10YearMean", "Particles", "PM10 Jahresmittel (laufendes Jahr)", "µg/m³", PM, 3600, 0.5, 0, sensor="sds", mqtt="pm10_year",
      info="Mittel der Tagesmittel seit 1. Januar. WHO-Leitwert 15 µg/m³, EU-Grenzwert 40 µg/m³ (ab 2030: 20)."),
    V("Pm25WhoExceeded", "Particles", "PM2,5 über WHO-Richtwert (24 h)", "", ALARM, 3600, 0, 0, True, sensor="sds", thresholds=False,
      decimals=0, mqtt="pm25_who_exceeded", info="1, solange das 24-Stunden-Mittel über 15 µg/m³ liegt (WHO 2021)."),
    V("Pm10WhoExceeded", "Particles", "PM10 über WHO-Richtwert (24 h)", "", ALARM, 3600, 0, 0, True, sensor="sds", thresholds=False,
      decimals=0, mqtt="pm10_who_exceeded", info="1, solange das 24-Stunden-Mittel über 45 µg/m³ liegt (WHO 2021)."),
    V("Pm25ExceedDays", "Particles", "PM2,5 Überschreitungstage (laufendes Jahr)", "", COUNT, 3600, 0, 0, sensor="sds",
      thresholds=False, decimals=0, mqtt="pm25_exceed_days",
      info="Tage seit 1. Januar mit Tagesmittel über der eingestellten Schwelle (unten auf dieser Seite)."),
    V("Pm10ExceedDays", "Particles", "PM10 Überschreitungstage (laufendes Jahr)", "", COUNT, 3600, 0, 0, sensor="sds",
      thresholds=False, decimals=0, mqtt="pm10_exceed_days",
      info="Tage seit 1. Januar mit Tagesmittel über der eingestellten Schwelle (unten auf dieser Seite)."),
    V("AqiPm25", "Particles", "Luftqualitätsindex PM2,5 (1-5)", "", BFT, 3600, 0, 0, sensor="sds", thresholds=False, decimals=0,
      mqtt="aqi_pm25", info="Umweltbundesamt, 24-Stunden-Mittel: 1 sehr gut (bis 10), 2 gut (bis 20), 3 mäßig (bis 25), 4 schlecht (bis 50), 5 sehr schlecht."),
    V("AqiPm10", "Particles", "Luftqualitätsindex PM10 (1-5)", "", BFT, 3600, 0, 0, sensor="sds", thresholds=False, decimals=0,
      mqtt="aqi_pm10", info="Umweltbundesamt, 24-Stunden-Mittel: 1 sehr gut (bis 20), 2 gut (bis 35), 3 mäßig (bis 50), 4 schlecht (bis 100), 5 sehr schlecht."),
    V("Aqi", "Particles", "Luftqualitätsindex Feinstaub (1-5)", "", BFT, 3600, 0, 0, True, sensor="sds", thresholds=False, decimals=0,
      mqtt="aqi", info="Schlechterer Wert aus PM2,5 und PM10."),
    V("AqiText", "Particles", "Luftqualität Feinstaub (Text)", "", TEXT, 3600, 0, 0, True, sensor="sds", thresholds=False, decimals=0,
      mqtt="aqi_text", influx="", fmt="aqitext", info="„sehr gut“, „gut“, „mäßig“, „schlecht“ oder „sehr schlecht“."),
    V("PmTrafficLight", "Particles", "Feinstaub-Ampel (0 grün, 1 gelb, 2 rot)", "", BFT, 3600, 0, 0, True, sensor="sds",
      thresholds=False, decimals=0, mqtt="pm_traffic_light",
      info="0 grün (Index sehr gut oder gut), 1 gelb (mäßig), 2 rot (schlecht oder sehr schlecht)."),
    V("FeelsLike", "Comfort", "Gefühlte Temperatur", "°C", TEMP, 600, 1, 0, mqtt="feels_like",
      info="Wie in Wetter-Apps: bis 10 °C der Windchill, ab 27 °C der Hitzeindex, dazwischen die Lufttemperatur."),
]

# Behaglichkeitsfeld nach Frank je Raum (hinten angefuegt, damit bestehende KO-Nummern bleiben)
for _n in range(1, ROOMS + 1):
    _c = f"Room{_n}Active"
    VALUES += [
        V(f"Room{_n}Comfort", "Ventilation", f"Behaglichkeit Raum {_n} (0 behaglich, 1 noch, 2 unbehaglich)", "", BFT, 1800, 0, 0, True,
          thresholds=False, decimals=0, mqtt=f"room{_n}_comfort", cond=_c,
          info="Behaglichkeitsfeld nach Frank (1975) aus Raumtemperatur und relativer Raumfeuchte."),
        V(f"Room{_n}ComfortText", "Ventilation", f"Behaglichkeit Raum {_n} (Text)", "", TEXT, 1800, 0, 0, True, thresholds=False,
          decimals=0, mqtt=f"room{_n}_comfort_text", influx="", fmt="comforttext", cond=_c,
          info="„behaglich“, sonst der Grund, z. B. „zu trocken“ oder „kalt, feucht“."),
        V(f"Room{_n}ComfortDev", "Ventilation", f"Abweichung vom Behaglichkeitsoptimum Raum {_n}", "", NUMBER, 1800, 0.5, 0,
          decimals=1, mqtt=f"room{_n}_comfort_deviation", cond=_c,
          info="Abstand zu 21 °C / 55 %: 1 entspricht 1 K oder 5 % relativer Feuchte."),
        V(f"Room{_n}TooHumid", "Ventilation", f"Raum {_n} zu feucht (über 65 %)", "", BOOL, 1800, 0, 0, thresholds=False, decimals=0,
          mqtt=f"room{_n}_too_humid", cond=_c),
        V(f"Room{_n}TooDry", "Ventilation", f"Raum {_n} zu trocken (unter 30 %)", "", BOOL, 1800, 0, 0, thresholds=False, decimals=0,
          mqtt=f"room{_n}_too_dry", cond=_c),
        V(f"Room{_n}TooCold", "Ventilation", f"Raum {_n} zu kalt", "", BOOL, 1800, 0, 0, thresholds=False, decimals=0,
          mqtt=f"room{_n}_too_cold", cond=_c, info="Außerhalb des behaglichen Feldes und unter 18,9 °C."),
        V(f"Room{_n}TooWarm", "Ventilation", f"Raum {_n} zu warm", "", BOOL, 1800, 0, 0, thresholds=False, decimals=0,
          mqtt=f"room{_n}_too_warm", cond=_c, info="Außerhalb des behaglichen Feldes und über 22 °C."),
    ]

# Stoerungs-KOs vor den Messwerten
FAULTS = [
    ("FaultWn90", "Störung Wetterstation", None),
    ("FaultSds", "Störung Feinstaubsensor", "sds"),
    ("Fault1W", "Störung 1-Wire-Sensor", "onewire"),
]


# Eingangs-KOs (vom Bus geschrieben): Raumwerte fuer die Lueftungsempfehlung, Bewaesserung erfolgt
INPUTS = []
for _n in range(1, ROOMS + 1):
    INPUTS.append(dict(key=f"Room{_n}TempIn", text=f"Raumtemperatur Raum {_n}", dpt="DPST-9-1", size="2 Bytes", group="Ventilation",
                       cond=f"Room{_n}Active"))
    INPUTS.append(dict(key=f"Room{_n}HumIn", text=f"Raumfeuchte Raum {_n}", dpt="DPST-9-7", size="2 Bytes", group="Ventilation",
                       cond=f"Room{_n}Active"))
INPUTS.append(dict(key="IrrigationDone", text="Bewässerung erfolgt", dpt="DPST-1-17", size="1 Bit", group="Irrigation", cond=None))

SENSOR_PARAM = {"sds": "SdsPresent", "onewire": "OneWirePresent"}

# Allgemeine Parameter: (Name, Typ, Offset, Bit, Text, Default, Suffix)
GENERAL_SIZE = 472  # frei: 447; 448-471 Raeume 4-6
GENERAL = [
    ("SdsPresent", "%AID%_PT-OnOffYesNo", 0, 0, "Feinstaubsensor SDS011 vorhanden", "0", None),
    ("OneWirePresent", "%AID%_PT-OnOffYesNo", 0, 1, "Temperatursensor 1-Wire (DS18B20) vorhanden", "0", None),
    ("CalcTempSource", "%AID%_PT-WxTempSource", 0, 2, "Temperatur für Berechnungen", "0", None),
    ("SdsDutyCycle", "%AID%_PT-WxDutyCycle", 1, 0, "Messintervall Feinstaubsensor (0 = Dauerbetrieb)", "5", "min"),
    ("Altitude", "%AID%_PT-WxAltitude", 2, 0, "Höhe des Standorts über Meeresspiegel", "0", "m"),
    ("TempOffset", "%AID%_PT-WxTempOffset", 4, 0, "Korrektur der Lufttemperatur", "0", "K"),
    ("RainStopDelay", "%AID%_PT-WxRainStop", 8, 0, "Regen endet, wenn so lange kein Niederschlag gemessen wurde", "15", "min"),
    ("MqttInterval", "%AID%_PT-WxCycle", 9, 0, "Messwerte per MQTT senden alle (0 = nie)", "60", "s"),
    ("RainWindow", "%AID%_PT-WxRainWindow", 11, 0, "Regen beginnt bei Niederschlag innerhalb von", "10", "min"),
    ("RainMinAmount", "%AID%_PT-WxRainAmount", 12, 0, "von mindestens", "0.1", "mm"),
    ("SymHeavyRain", "%AID%_PT-WxRainRate", 16, 0, "Starker Regen ab", "10", "mm/h"),
    ("SymStormGust", "%AID%_PT-WxBft", 20, 0, "Gewitter: starker Regen und Böen ab", "7", "Bft"),
    ("SymUvHigh", "%AID%_PT-WxUv", 21, 0, "Hoher UV-Index ab", "6", None),
    ("SymSunnyRatio", "%AID%_PT-WxPercent", 22, 0, "Sonnig ab Anteil der Helligkeit bei klarem Himmel", "60", "%"),
    ("TwilightDelay", "%AID%_PT-WxDelayMin", 23, 0, "Verzögerung", "2", "min"),
    ("TwilightNight", "%AID%_PT-WxLux", 24, 0, "Nacht unterhalb von", "30", "lx"),
    ("TwilightDay", "%AID%_PT-WxLux", 28, 0, "Tag oberhalb von", "60", "lx"),
    ("IceThreshold", "%AID%_PT-WxIceTemp", 32, 0, "Glättegefahr bei Temperatur bis", "1", "°C"),
    ("HaDiscovery", "%AID%_PT-OnOffYesNo", 36, 0, "Home Assistant MQTT Discovery", "0", None),
    ("SwMethod", "%AID%_PT-WxSwMethod", 64, 0, "Tagesmitteltemperatur", "0", None),
    ("SwEvening", "%AID%_PT-WxSwEvening", 64, 1, "Abendtermin (nur DIN)", "1", None),
    ("SwDays", "%AID%_PT-WxSwDays", 65, 0, "Mittel über", "3", "Tage"),
    ("SwThreshold", "%AID%_PT-WxSwThreshold", 66, 0, "Sommer bei Mitteltemperatur über", "16", "°C"),
    ("SwHysteresis", "%AID%_PT-WxSwHysteresis", 70, 0, "Hysterese (±)", "0", "K"),
    ("RadFactor", "%AID%_PT-WxRadFactor", 74, 0, "Korrekturfaktor Globalstrahlung", "100", "%"),
    ("Albedo", "%AID%_PT-WxAlbedo", 75, 0, "Bodenreflexion (Albedo)", "20", "%"),
    ("PvPeak", "%AID%_PT-WxKwp", 80, 0, "Anlagenleistung", "5", "kWp"),
    ("PvTilt", "%AID%_PT-WxElev", 84, 0, "Neigung der Module", "30", "°"),
    ("PvAzimuth", "%AID%_PT-WxAzimuth", 85, 0, "Ausrichtung der Module (0° = Nord, 180° = Süd)", "180", "°"),
    ("PvPr", "%AID%_PT-WxPercent", 87, 0, "Systemwirkungsgrad (Performance Ratio)", "80", "%"),
    ("Facade1Active", "%AID%_PT-OnOffYesNo", 88, 7, "Fassade 1 auswerten", "0", None),
    ("Facade1Azimuth", "%AID%_PT-WxAzimuth", 89, 0, "Ausrichtung (Blickrichtung nach außen, 0° = Nord, 90° = Ost)", "90", "°"),
    ("Facade1Range", "%AID%_PT-WxRange", 91, 0, "Sonne erfasst bis zu einer seitlichen Abweichung von ±", "80", "°"),
    ("Facade1MinElev", "%AID%_PT-WxElev", 92, 0, "Mindesthöhe der Sonne", "5", "°"),
    ("Facade1MaxElev", "%AID%_PT-WxElev", 93, 0, "Höchste Sonnenhöhe (z. B. Dachüberstand)", "90", "°"),
    ("Facade1Threshold", "%AID%_PT-WxIrr", 94, 0, "Sonne ab Einstrahlung auf die Fassade von", "200", "W/m²"),
    ("Facade1OnDelay", "%AID%_PT-WxDelayMin", 96, 0, "Sonne meldet nach", "2", "min"),
    ("Facade1OffDelay", "%AID%_PT-WxDelayMinAlarm", 97, 0, "Sonne weg nach", "15", "min"),
    ("Facade2Active", "%AID%_PT-OnOffYesNo", 100, 7, "Fassade 2 auswerten", "0", None),
    ("Facade2Azimuth", "%AID%_PT-WxAzimuth", 101, 0, "Ausrichtung (Blickrichtung nach außen, 0° = Nord, 90° = Ost)", "180", "°"),
    ("Facade2Range", "%AID%_PT-WxRange", 103, 0, "Sonne erfasst bis zu einer seitlichen Abweichung von ±", "80", "°"),
    ("Facade2MinElev", "%AID%_PT-WxElev", 104, 0, "Mindesthöhe der Sonne", "5", "°"),
    ("Facade2MaxElev", "%AID%_PT-WxElev", 105, 0, "Höchste Sonnenhöhe (z. B. Dachüberstand)", "90", "°"),
    ("Facade2Threshold", "%AID%_PT-WxIrr", 106, 0, "Sonne ab Einstrahlung auf die Fassade von", "200", "W/m²"),
    ("Facade2OnDelay", "%AID%_PT-WxDelayMin", 108, 0, "Sonne meldet nach", "2", "min"),
    ("Facade2OffDelay", "%AID%_PT-WxDelayMinAlarm", 109, 0, "Sonne weg nach", "15", "min"),
    ("Facade3Active", "%AID%_PT-OnOffYesNo", 112, 7, "Fassade 3 auswerten", "0", None),
    ("Facade3Azimuth", "%AID%_PT-WxAzimuth", 113, 0, "Ausrichtung (Blickrichtung nach außen, 0° = Nord, 90° = Ost)", "270", "°"),
    ("Facade3Range", "%AID%_PT-WxRange", 115, 0, "Sonne erfasst bis zu einer seitlichen Abweichung von ±", "80", "°"),
    ("Facade3MinElev", "%AID%_PT-WxElev", 116, 0, "Mindesthöhe der Sonne", "5", "°"),
    ("Facade3MaxElev", "%AID%_PT-WxElev", 117, 0, "Höchste Sonnenhöhe (z. B. Dachüberstand)", "90", "°"),
    ("Facade3Threshold", "%AID%_PT-WxIrr", 118, 0, "Sonne ab Einstrahlung auf die Fassade von", "200", "W/m²"),
    ("Facade3OnDelay", "%AID%_PT-WxDelayMin", 120, 0, "Sonne meldet nach", "2", "min"),
    ("Facade3OffDelay", "%AID%_PT-WxDelayMinAlarm", 121, 0, "Sonne weg nach", "15", "min"),
    ("Facade4Active", "%AID%_PT-OnOffYesNo", 124, 7, "Fassade 4 auswerten", "0", None),
    ("Facade4Azimuth", "%AID%_PT-WxAzimuth", 125, 0, "Ausrichtung (Blickrichtung nach außen, 0° = Nord, 90° = Ost)", "0", "°"),
    ("Facade4Range", "%AID%_PT-WxRange", 127, 0, "Sonne erfasst bis zu einer seitlichen Abweichung von ±", "80", "°"),
    ("Facade4MinElev", "%AID%_PT-WxElev", 128, 0, "Mindesthöhe der Sonne", "5", "°"),
    ("Facade4MaxElev", "%AID%_PT-WxElev", 129, 0, "Höchste Sonnenhöhe (z. B. Dachüberstand)", "90", "°"),
    ("Facade4Threshold", "%AID%_PT-WxIrr", 130, 0, "Sonne ab Einstrahlung auf die Fassade von", "200", "W/m²"),
    ("Facade4OnDelay", "%AID%_PT-WxDelayMin", 132, 0, "Sonne meldet nach", "2", "min"),
    ("Facade4OffDelay", "%AID%_PT-WxDelayMinAlarm", 133, 0, "Sonne weg nach", "15", "min"),
    ("Facade5Active", "%AID%_PT-OnOffYesNo", 136, 7, "Fassade 5 auswerten", "0", None),
    ("Facade5Azimuth", "%AID%_PT-WxAzimuth", 137, 0, "Ausrichtung (Blickrichtung nach außen, 0° = Nord, 90° = Ost)", "135", "°"),
    ("Facade5Range", "%AID%_PT-WxRange", 139, 0, "Sonne erfasst bis zu einer seitlichen Abweichung von ±", "80", "°"),
    ("Facade5MinElev", "%AID%_PT-WxElev", 140, 0, "Mindesthöhe der Sonne", "5", "°"),
    ("Facade5MaxElev", "%AID%_PT-WxElev", 141, 0, "Höchste Sonnenhöhe (z. B. Dachüberstand)", "90", "°"),
    ("Facade5Threshold", "%AID%_PT-WxIrr", 142, 0, "Sonne ab Einstrahlung auf die Fassade von", "200", "W/m²"),
    ("Facade5OnDelay", "%AID%_PT-WxDelayMin", 144, 0, "Sonne meldet nach", "2", "min"),
    ("Facade5OffDelay", "%AID%_PT-WxDelayMinAlarm", 145, 0, "Sonne weg nach", "15", "min"),
    ("Facade6Active", "%AID%_PT-OnOffYesNo", 148, 7, "Fassade 6 auswerten", "0", None),
    ("Facade6Azimuth", "%AID%_PT-WxAzimuth", 149, 0, "Ausrichtung (Blickrichtung nach außen, 0° = Nord, 90° = Ost)", "225", "°"),
    ("Facade6Range", "%AID%_PT-WxRange", 151, 0, "Sonne erfasst bis zu einer seitlichen Abweichung von ±", "80", "°"),
    ("Facade6MinElev", "%AID%_PT-WxElev", 152, 0, "Mindesthöhe der Sonne", "5", "°"),
    ("Facade6MaxElev", "%AID%_PT-WxElev", 153, 0, "Höchste Sonnenhöhe (z. B. Dachüberstand)", "90", "°"),
    ("Facade6Threshold", "%AID%_PT-WxIrr", 154, 0, "Sonne ab Einstrahlung auf die Fassade von", "200", "W/m²"),
    ("Facade6OnDelay", "%AID%_PT-WxDelayMin", 156, 0, "Sonne meldet nach", "2", "min"),
    ("Facade6OffDelay", "%AID%_PT-WxDelayMinAlarm", 157, 0, "Sonne weg nach", "15", "min"),
    ("Alarm1Source", "%AID%_PT-WxWindSource", 37, 0, "Auslösen durch", "0", None),
    ("Alarm1Threshold", "%AID%_PT-WxWindMs", 38, 0, "Schwelle", "10", "m/s"),
    ("Alarm1OnDelay", "%AID%_PT-WxDelaySec", 42, 0, "Auslösen nach", "3", "s"),
    ("Alarm1OffDelay", "%AID%_PT-WxDelayMinAlarm", 43, 0, "Zurücknehmen nach (unter der Schwelle)", "10", "min"),
    ("Alarm2Source", "%AID%_PT-WxWindSource", 46, 0, "Auslösen durch", "0", None),
    ("Alarm2Threshold", "%AID%_PT-WxWindMs", 47, 0, "Schwelle", "13", "m/s"),
    ("Alarm2OnDelay", "%AID%_PT-WxDelaySec", 51, 0, "Auslösen nach", "3", "s"),
    ("Alarm2OffDelay", "%AID%_PT-WxDelayMinAlarm", 52, 0, "Zurücknehmen nach (unter der Schwelle)", "10", "min"),
    ("Alarm3Source", "%AID%_PT-WxWindSource", 55, 0, "Auslösen durch", "0", None),
    ("Alarm3Threshold", "%AID%_PT-WxWindMs", 56, 0, "Schwelle", "17", "m/s"),
    ("Alarm3OnDelay", "%AID%_PT-WxDelaySec", 60, 0, "Auslösen nach", "3", "s"),
    ("Alarm3OffDelay", "%AID%_PT-WxDelayMinAlarm", 61, 0, "Zurücknehmen nach (unter der Schwelle)", "10", "min"),
    ("InfluxActive", "%AID%_PT-OnOffYesNo", 160, 7, "Messwerte in InfluxDB 1.x schreiben", "0", None),
    ("InfluxPort", "%AID%_PT-WxPort", 161, 0, "Port", "8086", None),
    ("InfluxInterval", "%AID%_PT-WxInfluxInterval", 163, 0, "Schreiben alle", "60", "s"),
    ("InfluxServer", "%AID%_PT-WxText40", 165, 0, "Server (Name oder IP-Adresse, ohne http://)", "", None),
    ("InfluxDatabase", "%AID%_PT-WxText20", 205, 0, "Datenbank", "", None),
    ("InfluxRp", "%AID%_PT-WxText20", 225, 0, "Retention Policy (leer = Standard)", "", None),
    ("InfluxUser", "%AID%_PT-WxText20", 245, 0, "Benutzer (leer = ohne Anmeldung)", "", None),
    ("InfluxPassword", "%AID%_PT-WxText20", 265, 0, "Passwort", "", None),
    ("InfluxMeasurement", "%AID%_PT-WxText20", 285, 0, "Measurement", "weather", None),
    ("InfluxTagFloor", "%AID%_PT-WxText20", 305, 0, "Tag floor", "Aussen", None),
    ("InfluxTagRoom", "%AID%_PT-WxText20", 325, 0, "Tag room", "Garten", None),
    ("InfluxTagSensor", "%AID%_PT-WxText20", 345, 0, "Tag sensor (1-Wire: _1wire, SDS011: _sds011 angehängt)", "wn90", None),
    ("SunDnMode", "%AID%_PT-WxSunDnMode", 368, 7, "Verfahren", "0", None),
    ("SunDnOffsetRise", "%AID%_PT-WxOffsetMin", 369, 0, "Versatz zum Sonnenaufgang", "0", "min"),
    ("SunDnOffsetSet", "%AID%_PT-WxOffsetMin", 371, 0, "Versatz zum Sonnenuntergang", "0", "min"),
    ("SunDnElevRise", "%AID%_PT-WxElevSigned", 373, 0, "Morgens Tag ab Elevation", "0", "°"),
    ("SunDnElevSet", "%AID%_PT-WxElevSigned", 377, 0, "Abends Nacht unter Elevation", "0", "°"),
    ("SunshineThreshold", "%AID%_PT-WxIrr", 381, 0, "Sonnenschein ab Globalstrahlung", "120", "W/m²"),
    ("Room1Active", "%AID%_PT-OnOffYesNo", 384, 7, "Raum 1 auswerten", "0", None),
    ("Room1MinDiff", "%AID%_PT-WxHumDiff", 385, 0, "Lüften ab einem Feuchteunterschied von", "1", "g/m³"),
    ("Room1MinRh", "%AID%_PT-WxPercent0", 389, 0, "nur wenn die Raumfeuchte mindestens beträgt", "50", "%"),
    ("Room1MinIndoorT", "%AID%_PT-WxTempLimit", 390, 0, "nur wenn die Raumtemperatur mindestens beträgt", "10", "°C"),
    ("Room1MinOutdoorT", "%AID%_PT-WxTempLimit", 391, 0, "nur wenn die Außentemperatur mindestens beträgt", "-5", "°C"),
    ("Room2Active", "%AID%_PT-OnOffYesNo", 392, 7, "Raum 2 auswerten", "0", None),
    ("Room2MinDiff", "%AID%_PT-WxHumDiff", 393, 0, "Lüften ab einem Feuchteunterschied von", "1", "g/m³"),
    ("Room2MinRh", "%AID%_PT-WxPercent0", 397, 0, "nur wenn die Raumfeuchte mindestens beträgt", "50", "%"),
    ("Room2MinIndoorT", "%AID%_PT-WxTempLimit", 398, 0, "nur wenn die Raumtemperatur mindestens beträgt", "10", "°C"),
    ("Room2MinOutdoorT", "%AID%_PT-WxTempLimit", 399, 0, "nur wenn die Außentemperatur mindestens beträgt", "-5", "°C"),
    ("Room3Active", "%AID%_PT-OnOffYesNo", 400, 7, "Raum 3 auswerten", "0", None),
    ("Room3MinDiff", "%AID%_PT-WxHumDiff", 401, 0, "Lüften ab einem Feuchteunterschied von", "1", "g/m³"),
    ("Room3MinRh", "%AID%_PT-WxPercent0", 405, 0, "nur wenn die Raumfeuchte mindestens beträgt", "50", "%"),
    ("Room3MinIndoorT", "%AID%_PT-WxTempLimit", 406, 0, "nur wenn die Raumtemperatur mindestens beträgt", "10", "°C"),
    ("Room3MinOutdoorT", "%AID%_PT-WxTempLimit", 407, 0, "nur wenn die Außentemperatur mindestens beträgt", "-5", "°C"),
    ("IrrKc", "%AID%_PT-WxPercent", 408, 0, "Pflanzenfaktor (Kc, Rasen ca. 80 %)", "80", "%"),
    ("IrrThreshold", "%AID%_PT-WxMm", 409, 0, "Bewässern ab einem Defizit von", "5", "mm"),
    ("IrrStorage", "%AID%_PT-WxMm", 413, 0, "Bodenspeicher (Überschuss darüber versickert)", "20", "mm"),
    ("IrrAmount", "%AID%_PT-WxMm", 417, 0, "Gutschrift je Meldung „Bewässerung erfolgt“", "10", "mm"),
    ("DdIndoor", "%AID%_PT-WxSwThreshold", 421, 0, "Heizgradtage: Innentemperatur", "20", "°C"),
    ("DdHeatLimit", "%AID%_PT-WxSwThreshold", 425, 0, "Heizgradtage: Heizgrenze (Tagesmittel unter)", "15", "°C"),
    ("DdCoolBase", "%AID%_PT-WxSwThreshold", 429, 0, "Kühlgradtage: Basistemperatur (Tagesmittel über)", "18", "°C"),
    ("StationHeight", "%AID%_PT-WxHeight", 433, 0, "Montagehöhe der Wetterstation über Grund", "2", "m"),
    ("WarnHold", "%AID%_PT-WxDelayMinAlarm", 434, 0, "Warnung bleibt nach Ende der Bedingung bestehen für", "30", "min"),
    ("DwdActive", "%AID%_PT-OnOffYesNo", 435, 7, "Amtliche DWD-Warnungen abrufen (benötigt Internet)", "0", None),
    ("DwdCell", "%AID%_PT-WxWarnCell", 436, 0, "Warnzellen-ID (WarnCellID, 9-stellig)", "0", None),
    ("DwdInterval", "%AID%_PT-WxDwdInterval", 440, 0, "Abruf alle", "5", "min"),
    ("SendSpacing", "%AID%_PT-WxSendSpacing", 441, 0, "Mindestabstand zwischen zwei Telegrammen", "100", "ms"),
    ("LedMode", "%AID%_PT-WxLedMode", 443, 0, "Statusanzeige", "2", None),
    ("LedBrightness", "%AID%_PT-WxLedBrightness", 444, 0, "Helligkeit der Statusanzeige", "10", "%"),
    ("LedHeartbeat", "%AID%_PT-OnOffYesNo", 445, 7, "Lebenszeichen (kurz grün alle 10 s)", "0", None),
    ("PmLimit", "%AID%_PT-WxPmLimit", 446, 0, "Überschreitungstage zählen nach", "0", None),
] + [p for _n in range(4, ROOMS + 1) for _b in [448 + (_n - 4) * 8] for p in (
    (f"Room{_n}Active", "%AID%_PT-OnOffYesNo", _b, 7, f"Raum {_n} auswerten", "0", None),
    (f"Room{_n}MinDiff", "%AID%_PT-WxHumDiff", _b + 1, 0, "Lüften ab einem Feuchteunterschied von", "1", "g/m³"),
    (f"Room{_n}MinRh", "%AID%_PT-WxPercent0", _b + 5, 0, "nur wenn die Raumfeuchte mindestens beträgt", "50", "%"),
    (f"Room{_n}MinIndoorT", "%AID%_PT-WxTempLimit", _b + 6, 0, "nur wenn die Raumtemperatur mindestens beträgt", "10", "°C"),
    (f"Room{_n}MinOutdoorT", "%AID%_PT-WxTempLimit", _b + 7, 0, "nur wenn die Außentemperatur mindestens beträgt", "-5", "°C"),
)]

# Home Assistant: Geraeteklasse und Einheit je Einheit, Sonderfaelle je Schluessel
HA_BY_UNIT = {
    "°C": ("temperature", "°C"), "%": ("humidity", "%"), "g/m³": ("absolute_humidity", "g/m³"),
    "m/s": ("wind_speed", "m/s"), "hPa": ("atmospheric_pressure", "hPa"), "lx": ("illuminance", "lx"),
    "mm": ("precipitation", "mm"), "W/m²": ("irradiance", "W/m²"), "W": ("power", "W"), "kWh": ("energy", "kWh"),
    "kWh/m²": (None, "kWh/m²"), "mm/h": ("precipitation_intensity", "mm/h"), "°": (None, "°"),
}
HA_BY_KEY = {
    **{f"Room{_n}Vent": (None, None) for _n in range(1, ROOMS + 1)}, "Irrigate": (None, None),
    "WaterBalance": (None, "mm"), "Et0Today": (None, "mm"), "Et0Yesterday": (None, "mm"),
    "DayLength": ("duration", "s"), "SunshineToday": ("duration", "h"), "SunshineYesterday": ("duration", "h"),
    "MoonAge": (None, "d"), "MoonIllumination": (None, "%"), "DayProgress": (None, "%"),
    "CloudCover": (None, "%"),
    "SummerWinter": (None, None), "SummerDay": (None, None), "HotDay": ("heat", None), "FrostDay": ("cold", None),
    "IceDay": ("cold", None),
    "Trend1h": (None, "hPa"), "Trend3h": (None, "hPa"), "Pm25": ("pm25", "µg/m³"), "Pm10": ("pm10", "µg/m³"),
    "Pm25Norm": ("pm25", "µg/m³"), "Pm10Norm": ("pm10", "µg/m³"), "Raining": ("moisture", None),
    "WindAlarm1": ("safety", None), "WindAlarm2": ("safety", None), "WindAlarm3": ("safety", None),
    "Frost": ("cold", None), "IceWarning": ("cold", None), "DayNight": (None, None),
}


def ha_meta(v):
    if v["key"] in HA_BY_KEY:
        return HA_BY_KEY[v["key"]]
    return HA_BY_UNIT.get(v["unit"], (None, None))


def cstr(text):
    """C-Stringliteral, Nicht-ASCII als \\u-Escape (UTF-8 im Programm, ASCII im Quelltext)."""
    if text is None:
        return "nullptr"
    out = "".join(c if ord(c) < 128 else f"\\u{ord(c):04x}" for c in text)
    return f'"{out}"'


DPT_TYPES = {}  # Labels -> ParameterType-Id


def dpt_type(dpts):
    labels = tuple(d[0] for d in dpts)
    if labels not in DPT_TYPES:
        DPT_TYPES[labels] = f"%AID%_PT-WxDpt{len(DPT_TYPES) + 1}"
    return DPT_TYPES[labels]


def fmt(num):
    return f"{num:g}"


def cfloat(num):
    s = f"{num:g}"
    if "." not in s and "e" not in s:
        s += ".0"
    return s + "f"


IDS_FILE = ROOT / "scripts" / "param_ids.json"
KO_FILE = ROOT / "scripts" / "ko_numbers.json"


class KoNumbers:
    """KO-Nummern (relativ zu KO_OFFSET), dauerhaft in scripts/ko_numbers.json.
    Gruppenadressen und Logik-Verknuepfungen haengen an den Nummern; sie duerfen sich nie aendern."""

    def __init__(self):
        import json
        self.bykey = json.loads(KO_FILE.read_text()) if KO_FILE.exists() else {}
        self.next = max(self.bykey.values(), default=0) + 1

    def get(self, key):
        if key not in self.bykey:
            self.bykey[key] = self.next
            self.next += 1
        return self.bykey[key]

    def save(self):
        import json
        KO_FILE.write_text(json.dumps(self.bykey, indent=1) + "\n", encoding="utf-8")


class Ids:
    """Parameter-Ids (5-stellig wie in OpenKNX ueblich), dauerhaft in scripts/param_ids.json.

    Die ETS ordnet beim Aktualisieren der Applikation Parameterwerte ueber diese Ids zu.
    Bestehende Ids duerfen sich daher nie aendern; neue Parameter erhalten die naechste freie Id."""

    def __init__(self):
        import json
        self.byname = json.loads(IDS_FILE.read_text()) if IDS_FILE.exists() else {}
        self.next = max(self.byname.values(), default=0) + 1
        self.used = set()

    def add(self, name):
        if name not in self.byname:
            self.byname[name] = self.next
            self.next += 1
        self.used.add(name)
        return self.byname[name]

    def save(self):
        import json
        IDS_FILE.write_text(json.dumps(self.byname, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")

    def ref(self, name):
        n = self.byname[name]
        return f"%AID%_UP-%TT%{n:05d}_R-%TT%{n:05d}01"


def build():
    ids = Ids()
    params, refs = [], []

    def param(name, ptype, offset, bit, text, value, suffix=None):
        n = ids.add(name)
        s = f' SuffixText="{escape(suffix)}"' if suffix else ""
        params.append(f'                <Parameter Id="%AID%_UP-%TT%{n:05d}" Name="{name}" ParameterType="{ptype}" '
                      f'Offset="{offset}" BitOffset="{bit}" Text="{escape(text)}" Value="{value}"{s} />')
        refs.append(f'              <ParameterRef Id="{ids.ref(name)}" RefId="%AID%_UP-%TT%{n:05d}" />')

    for g in GENERAL:
        param(*g)

    offset = GENERAL_SIZE
    for v in VALUES:
        v["offset"] = offset
        k = v["key"]
        param(f"{k}Active", "%AID%_PT-CheckBox", offset, 7, "Aktiv", "1" if v["active"] else "0")
        if len(v["dpts"]) > 1:
            param(f"{k}Dpt", dpt_type(v["dpts"]), offset, 5, "Datenpunkttyp", str(v["dpt"]))
        param(f"{k}Cycle", "%AID%_PT-WxCycle", offset + 1, 0, "Zyklisch senden (0 = nie)", v["cycle"], "s")
        if v["thresholds"]:
            unit = v["unit"] or None
            param(f"{k}Abs", "%AID%_PT-WxAbs", offset + 3, 0, "Senden bei Änderung um (0 = nie)", fmt(v["abs"]), unit)
            param(f"{k}Rel", "%AID%_PT-WxRel", offset + 7, 0, "Senden bei relativer Änderung um (0 = nie)",
                  fmt(v["rel"]), "%")
        offset += VALUE_SIZE
    total = offset

    # KOs: Stoerungen, dann Messwerte (je DPT ein ComObjectRef)
    kos, korefs = [], []
    knum = KoNumbers()
    for name, text, _ in FAULTS:
        kono = knum.get(name)
        kos.append(f'              <ComObject Id="%AID%_O-%TT%{kono:05d}" Name="{name}" Text="{escape(text)}" Number="{kono}" '
                   'FunctionText="Alarm" ObjectSize="1 Bit" ReadFlag="Enabled" WriteFlag="Disabled" CommunicationFlag="Enabled" '
                   'TransmitFlag="Enabled" UpdateFlag="Disabled" ReadOnInitFlag="Disabled" DatapointType="DPST-1-5" />')
        korefs.append(f'              <ComObjectRef Id="%AID%_O-%TT%{kono:05d}_R-%TT%{kono:05d}01" RefId="%AID%_O-%TT%{kono:05d}" />')
    for v in VALUES:
        kono = knum.get(v["key"])
        v["ko"] = kono
        d0 = v["dpts"][0]
        kos.append(f'              <ComObject Id="%AID%_O-%TT%{kono:05d}" Name="{v["key"]}" Text="{escape(v["text"])}" '
                   f'Number="{kono}" FunctionText="Ausgang" ObjectSize="{d0[2]}" ReadFlag="Enabled" WriteFlag="Disabled" '
                   'CommunicationFlag="Enabled" TransmitFlag="Enabled" UpdateFlag="Disabled" ReadOnInitFlag="Disabled" '
                   f'DatapointType="{d0[1]}" />')
        for i, d in enumerate(v["dpts"]):
            korefs.append(f'              <ComObjectRef Id="%AID%_O-%TT%{kono:05d}_R-%TT%{kono:05d}{i + 1:02d}" '
                          f'RefId="%AID%_O-%TT%{kono:05d}" ObjectSize="{d[2]}" DatapointType="{d[1]}" />')
    # Eingangs-KOs (vom Bus geschrieben)
    for inp in INPUTS:
        kono = knum.get(inp["key"])
        inp["ko"] = kono
        kos.append(f'              <ComObject Id="%AID%_O-%TT%{kono:05d}" Name="{inp["key"]}" Text="{escape(inp["text"])}" '
                   f'Number="{kono}" FunctionText="Eingang" ObjectSize="{inp["size"]}" ReadFlag="Disabled" WriteFlag="Enabled" '
                   'CommunicationFlag="Enabled" TransmitFlag="Disabled" UpdateFlag="Enabled" ReadOnInitFlag="Disabled" '
                   f'DatapointType="{inp["dpt"]}" />')
        korefs.append(f'              <ComObjectRef Id="%AID%_O-%TT%{kono:05d}_R-%TT%{kono:05d}01" RefId="%AID%_O-%TT%{kono:05d}" />')
    kono = knum.next
    knum.save()

    # Parametertypen
    ptypes = [
        '              <ParameterType Id="%AID%_PT-WxCycle" Name="WxCycle">',
        '                <TypeNumber SizeInBit="16" Type="unsignedInt" minInclusive="0" maxInclusive="43200" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxAbs" Name="WxAbs">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0" maxInclusive="300000" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxRel" Name="WxRel">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0" maxInclusive="100" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxDutyCycle" Name="WxDutyCycle">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="0" maxInclusive="30" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxAltitude" Name="WxAltitude">',
        '                <TypeNumber SizeInBit="16" Type="signedInt" minInclusive="-500" maxInclusive="9000" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxTempOffset" Name="WxTempOffset">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="-10" maxInclusive="10" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxRainWindow" Name="WxRainWindow">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="60" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxRainAmount" Name="WxRainAmount">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0.01" maxInclusive="10" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxDelayMin" Name="WxDelayMin">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="0" maxInclusive="60" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxDelayMinAlarm" Name="WxDelayMinAlarm">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="120" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxDelaySec" Name="WxDelaySec">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="0" maxInclusive="60" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxLux" Name="WxLux">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="1" maxInclusive="10000" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxIceTemp" Name="WxIceTemp">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="-5" maxInclusive="5" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxWindMs" Name="WxWindMs">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="1" maxInclusive="50" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxWindSource" Name="WxWindSource">',
        '                <TypeRestriction Base="Value" SizeInBit="2">',
        '                  <Enumeration Text="Böe (Momentanwert)" Value="0" Id="%ENID%" />',
        '                  <Enumeration Text="Windgeschwindigkeit (Momentanwert)" Value="1" Id="%ENID%" />',
        '                  <Enumeration Text="Windgeschwindigkeit (10-min-Mittel)" Value="2" Id="%ENID%" />',
        '                </TypeRestriction>',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxSwMethod" Name="WxSwMethod">',
        '                <TypeRestriction Base="Value" SizeInBit="1">',
        '                  <Enumeration Text="DIN (7, 14 und Abendtermin, Abendwert doppelt)" Value="0" Id="%ENID%" />',
        '                  <Enumeration Text="Mittel aller Messwerte des Tages" Value="1" Id="%ENID%" />',
        '                </TypeRestriction>',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxSwEvening" Name="WxSwEvening">',
        '                <TypeRestriction Base="Value" SizeInBit="1">',
        '                  <Enumeration Text="21 Uhr (Mannheimer Stunden)" Value="0" Id="%ENID%" />',
        '                  <Enumeration Text="22 Uhr" Value="1" Id="%ENID%" />',
        '                </TypeRestriction>',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxSwDays" Name="WxSwDays">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="7" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxSwThreshold" Name="WxSwThreshold">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="5" maxInclusive="25" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxSwHysteresis" Name="WxSwHysteresis">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0" maxInclusive="5" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxRadFactor" Name="WxRadFactor">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="50" maxInclusive="200" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxAlbedo" Name="WxAlbedo">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="0" maxInclusive="90" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxKwp" Name="WxKwp">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0.1" maxInclusive="1000" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxElev" Name="WxElev">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="0" maxInclusive="90" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxAzimuth" Name="WxAzimuth">',
        '                <TypeNumber SizeInBit="16" Type="unsignedInt" minInclusive="0" maxInclusive="359" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxRange" Name="WxRange">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="10" maxInclusive="90" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxIrr" Name="WxIrr">',
        '                <TypeNumber SizeInBit="16" Type="unsignedInt" minInclusive="0" maxInclusive="1000" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxPort" Name="WxPort">',
        '                <TypeNumber SizeInBit="16" Type="unsignedInt" minInclusive="1" maxInclusive="65535" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxInfluxInterval" Name="WxInfluxInterval">',
        '                <TypeNumber SizeInBit="16" Type="unsignedInt" minInclusive="10" maxInclusive="3600" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxText20" Name="WxText20">',
        '                <TypeText SizeInBit="160" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxText40" Name="WxText40">',
        '                <TypeText SizeInBit="320" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxSunDnMode" Name="WxSunDnMode">',
        '                <TypeRestriction Base="Value" SizeInBit="1">',
        '                  <Enumeration Text="Sonnenauf-/untergang mit Versatz" Value="0" Id="%ENID%" />',
        '                  <Enumeration Text="Elevationsschwellen" Value="1" Id="%ENID%" />',
        '                </TypeRestriction>',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxOffsetMin" Name="WxOffsetMin">',
        '                <TypeNumber SizeInBit="16" Type="signedInt" minInclusive="-240" maxInclusive="240" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxElevSigned" Name="WxElevSigned">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="-18" maxInclusive="30" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxHumDiff" Name="WxHumDiff">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0.1" maxInclusive="10" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxPercent0" Name="WxPercent0">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="0" maxInclusive="100" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxTempLimit" Name="WxTempLimit">',
        '                <TypeNumber SizeInBit="8" Type="signedInt" minInclusive="-30" maxInclusive="40" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxMm" Name="WxMm">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0.5" maxInclusive="100" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxHeight" Name="WxHeight">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="50" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxWarnCell" Name="WxWarnCell">',
        '                <TypeNumber SizeInBit="32" Type="unsignedInt" minInclusive="0" maxInclusive="999999999" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxDwdInterval" Name="WxDwdInterval">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="2" maxInclusive="60" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxSendSpacing" Name="WxSendSpacing">',
        '                <TypeNumber SizeInBit="16" Type="unsignedInt" minInclusive="20" maxInclusive="1000" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxLedMode" Name="WxLedMode">',
        '                <TypeRestriction Base="Value" SizeInBit="8">',
        '                  <Enumeration Text="nur Programmiermodus" Value="0" Id="%ENID%" />',
        '                  <Enumeration Text="Programmiermodus und Störungen" Value="1" Id="%ENID%" />',
        '                  <Enumeration Text="alle Zustände" Value="2" Id="%ENID%" />',
        '                </TypeRestriction>',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxPmLimit" Name="WxPmLimit">',
        '                <TypeRestriction Base="Value" SizeInBit="8">',
        '                  <Enumeration Text="WHO 2021 (Tagesmittel PM2,5 über 15, PM10 über 45 µg/m³)" Value="0" Id="%ENID%" />',
        '                  <Enumeration Text="EU ab 2030 (Tagesmittel PM2,5 über 25, PM10 über 45 µg/m³)" Value="1" Id="%ENID%" />',
        '                  <Enumeration Text="EU bis 2029 (Tagesmittel PM10 über 50 µg/m³, PM2,5 über 25 µg/m³)" Value="2" Id="%ENID%" />',
        '                </TypeRestriction>',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxLedBrightness" Name="WxLedBrightness">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="100" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxTempSource" Name="WxTempSource">',
        '                <TypeRestriction Base="Value" SizeInBit="1">',
        '                  <Enumeration Text="Wetterstation" Value="0" Id="%ENID%" />',
        '                  <Enumeration Text="1-Wire-Sensor" Value="1" Id="%ENID%" />',
        '                </TypeRestriction>',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxRainRate" Name="WxRainRate">',
        '                <TypeFloat Encoding="IEEE-754 Single" minInclusive="0.5" maxInclusive="100" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxBft" Name="WxBft">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="12" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxUv" Name="WxUv">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="15" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxPercent" Name="WxPercent">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="10" maxInclusive="100" />',
        '              </ParameterType>',
        '              <ParameterType Id="%AID%_PT-WxRainStop" Name="WxRainStop">',
        '                <TypeNumber SizeInBit="8" Type="unsignedInt" minInclusive="1" maxInclusive="240" />',
        '              </ParameterType>',
    ]
    for labels, tid in DPT_TYPES.items():
        ptypes.append(f'              <ParameterType Id="{tid}" Name="{tid.split("PT-")[1]}">')
        ptypes.append('                <TypeRestriction Base="Value" SizeInBit="2">')
        for i, label in enumerate(labels):
            ptypes.append(f'                  <Enumeration Text="{escape(label)}" Value="{i}" Id="%ENID%" />')
        ptypes.append('                </TypeRestriction>')
        ptypes.append('              </ParameterType>')

    # Oberflaeche
    dyn = []
    w = dyn.append
    w('            <Channel Id="%AID%_CH-%PREFIX%" Number="%PREFIX%" Name="Main" Text="Wetterstation" Icon="weather-partly-snowy-rainy">')
    w('              <ParameterBlock Id="%AID%_PB-nnn" Name="General" Text="Allgemein" Icon="cog-outline">')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Sensoren" UIHint="Headline" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Die Wetterstation WN90LP wird immer abgefragt (Modbus RTU, alle 2 s)." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("SdsPresent")}" IndentLevel="1" />')
    w(f'                <choose ParamRefId="{ids.ref("SdsPresent")}">')
    w('                  <when test="1">')
    w(f'                    <ParameterRefRef RefId="{ids.ref("SdsDutyCycle")}" IndentLevel="2" />')
    w('                  </when>')
    w('                </choose>')
    w(f'                <ParameterRefRef RefId="{ids.ref("OneWirePresent")}" IndentLevel="1" />')
    w(f'                <choose ParamRefId="{ids.ref("OneWirePresent")}">')
    w('                  <when test="1">')
    w(f'                    <ParameterRefRef RefId="{ids.ref("CalcTempSource")}" IndentLevel="2" />')
    w('                    <ParameterSeparator Id="%AID%_PS-nnn" Text="Gilt für Taupunkt, Frostpunkt, absolute Feuchte, Behaglichkeitswerte und Luftdruck auf Meereshöhe. Die Luftfeuchte stammt immer von der Wetterstation. Fällt der 1-Wire-Sensor aus, wird die Temperatur der Wetterstation verwendet." UIHint="Information" />')
    w('                  </when>')
    w('                </choose>')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Standort und Korrektur" UIHint="Headline" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("Altitude")}" IndentLevel="1" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("TempOffset")}" IndentLevel="1" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Regen" UIHint="Headline" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Einzelne Zählerschritte (z. B. durch Tau oder Erschütterungen) lösen so noch keinen Regen aus." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("RainWindow")}" IndentLevel="1" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("RainMinAmount")}" IndentLevel="1" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("RainStopDelay")}" IndentLevel="1" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Senden" UIHint="Headline" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Begrenzt die Telegrammrate der Wetterstation (100 ms = höchstens 10 Telegramme pro Sekunde). Wichtig vor allem nach dem Start, wenn alle aktiven Werte einmal gesendet werden, und wenn die Gruppenadressen über einen IP-Router auf eine TP-Linie gelangen." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("SendSpacing")}" IndentLevel="1" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Status-LED" UIHint="Headline" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Die RGB-LED auf der Platine zeigt immer den Programmiermodus (rot). Zusätzlich, nach Vorrang: OTA-Update (violett pulsierend), WLAN getrennt (blau blinkend), Sensorstörung WN90LP/SDS011/1-Wire (gelb blinkend), bei „alle Zustände“ außerdem ausgelöster Windalarm (orange) und das Lebenszeichen. Sonst bleibt die LED aus." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("LedMode")}" IndentLevel="1" />')
    w(f'                <choose ParamRefId="{ids.ref("LedMode")}">')
    w('                  <when test="1">')
    w(f'                    <ParameterRefRef RefId="{ids.ref("LedBrightness")}" IndentLevel="2" />')
    w('                  </when>')
    w('                  <when test="2">')
    w(f'                    <ParameterRefRef RefId="{ids.ref("LedBrightness")}" IndentLevel="2" />')
    w(f'                    <ParameterRefRef RefId="{ids.ref("LedHeartbeat")}" IndentLevel="2" />')
    w('                  </when>')
    w('                </choose>')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Nach der ersten Inbetriebnahme enthalten die Regenmengen nur die Zeit seit dem Start (z. B. 72 Stunden erst nach drei Tagen vollständig). Danach bleiben sie über Neustarts erhalten." UIHint="Information" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="MQTT" UIHint="Headline" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Broker, Zugangsdaten und Themenpräfix werden auf der Seite Netzwerk / MQTT eingestellt. Gesendet wird ein JSON-Objekt an openknx/&lt;Präfix&gt;/state." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("MqttInterval")}" IndentLevel="1" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("HaDiscovery")}" IndentLevel="1" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Das direkte Schreiben in eine InfluxDB wird auf der Seite Datenexport (InfluxDB) eingestellt." UIHint="Information" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Discovery meldet alle aktiven Messwerte unter homeassistant/... an (retained); Home Assistant legt sie dann automatisch als Sensoren an." UIHint="Information" />')
    # Stoerungs-KOs (nur Sichtbarkeit, keine Parameter)
    w('                <ComObjectRefRef RefId="%AID%_O-%TT%00001_R-%TT%0000101" />')
    w(f'                <choose ParamRefId="{ids.ref("SdsPresent")}">')
    w('                  <when test="1">')
    w('                    <ComObjectRefRef RefId="%AID%_O-%TT%00002_R-%TT%0000201" />')
    w('                  </when>')
    w('                </choose>')
    w(f'                <choose ParamRefId="{ids.ref("OneWirePresent")}">')
    w('                  <when test="1">')
    w('                    <ComObjectRefRef RefId="%AID%_O-%TT%00003_R-%TT%0000301" />')
    w('                  </when>')
    w('                </choose>')
    w('              </ParameterBlock>')
    w('              <ParameterBlock Id="%AID%_PB-nnn" Name="Influx" Text="Datenexport (InfluxDB)" Icon="application-cog-outline">')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Schreibt alle gültigen Messwerte direkt per HTTP in eine InfluxDB 1.x (Schnittstelle /write, ohne TLS). Drei Punkte je Intervall: Wetterstation, 1-Wire und SDS011, unterschieden über den Tag sensor. Zahlen werden als Gleitkommawerte geschrieben, den Zeitstempel setzt der Server." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("InfluxActive")}" IndentLevel="1" />')
    w(f'                <choose ParamRefId="{ids.ref("InfluxActive")}">')
    w('                  <when test="1">')
    for n in ("InfluxServer", "InfluxPort", "InfluxDatabase", "InfluxRp", "InfluxUser", "InfluxPassword", "InfluxInterval"):
        w(f'                    <ParameterRefRef RefId="{ids.ref(n)}" IndentLevel="2" />')
    w('                    <ParameterSeparator Id="%AID%_PS-nnn" Text="Datenstruktur" UIHint="Headline" />')
    for n in ("InfluxMeasurement", "InfluxTagFloor", "InfluxTagRoom", "InfluxTagSensor"):
        w(f'                    <ParameterRefRef RefId="{ids.ref(n)}" IndentLevel="2" />')
    w('                    <ParameterSeparator Id="%AID%_PS-nnn" Text="Das Passwort steht im ETS-Projekt und im Gerät im Klartext." UIHint="Information" />')
    w('                  </when>')
    w('                </choose>')
    w('              </ParameterBlock>')

    def value_ui(v, ind):
        p = " " * ind
        k = v["key"]
        kono = v["ko"]
        w(f'{p}<ParameterSeparator Id="%AID%_PS-nnn" Text="{escape(v["text"])}" UIHint="Headline" />')
        if v["info"]:
            w(f'{p}<ParameterSeparator Id="%AID%_PS-nnn" Text="{escape(v["info"])}" UIHint="Information" />')
        w(f'{p}<ParameterRefRef RefId="{ids.ref(k + "Active")}" IndentLevel="1" />')
        w(f'{p}<choose ParamRefId="{ids.ref(k + "Active")}">')
        w(f'{p}  <when test="1">')
        if len(v["dpts"]) > 1:
            w(f'{p}    <ParameterRefRef RefId="{ids.ref(k + "Dpt")}" IndentLevel="2" />')
        w(f'{p}    <ParameterRefRef RefId="{ids.ref(k + "Cycle")}" IndentLevel="2" />')
        for n in v["extra"]:
            w(f'{p}    <ParameterRefRef RefId="{ids.ref(n)}" IndentLevel="2" />')
        if v["thresholds"]:
            w(f'{p}    <ParameterRefRef RefId="{ids.ref(k + "Abs")}" IndentLevel="2" />')
            w(f'{p}    <ParameterRefRef RefId="{ids.ref(k + "Rel")}" IndentLevel="2" />')
        if len(v["dpts"]) > 1:
            w(f'{p}    <choose ParamRefId="{ids.ref(k + "Dpt")}">')
            for i in range(len(v["dpts"])):
                w(f'{p}      <when test="{i}">')
                w(f'{p}        <ComObjectRefRef RefId="%AID%_O-%TT%{kono:05d}_R-%TT%{kono:05d}{i + 1:02d}" />')
                w(f'{p}      </when>')
            w(f'{p}    </choose>')
        else:
            w(f'{p}    <ComObjectRefRef RefId="%AID%_O-%TT%{kono:05d}_R-%TT%{kono:05d}01" />')
        w(f'{p}  </when>')
        w(f'{p}</choose>')

    for gkey, gtext, icon, gsensor in GROUPS:
        ind = 14
        if gsensor:
            w(f'{" " * ind}<choose ParamRefId="{ids.ref(SENSOR_PARAM[gsensor])}">')
            w(f'{" " * ind}  <when test="1">')
            ind += 4
        w(f'{" " * ind}<ParameterBlock Id="%AID%_PB-nnn" Name="{gkey}" Text="{escape(gtext)}" Icon="{icon}">')
        if gkey == "Forecast":
            q = " " * (ind + 2)
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Wettersymbol" UIHint="Headline" />')
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Vorrang: Gewitter, starker Regen, Schauer (Regen aktiv), hoher UV-Index, sonnig/bewölkt. '
              'Sonnig/bewölkt vergleicht tagsüber die Helligkeit mit dem Wert bei klarem Himmel für den Sonnenstand '
              '(benötigt Uhrzeit und Gerätestandort); nachts entscheidet die Prognose." UIHint="Information" />')
            for n in ("SymHeavyRain", "SymStormGust", "SymUvHigh", "SymSunnyRatio"):
                w(f'{q}<ParameterRefRef RefId="{ids.ref(n)}" IndentLevel="1" />')
        if gkey == "WindAlarm":
            w(f'{" " * (ind + 2)}<ParameterSeparator Id="%AID%_PS-nnn" Text="Schutz für Markisen und Jalousien: Der Alarm wird ausgelöst, wenn der Wind die Schwelle für die eingestellte Zeit überschreitet, und erst zurückgenommen, wenn er die eingestellte Zeit darunter bleibt. Beaufort 6 ≈ 10,8 m/s, 7 ≈ 13,9 m/s, 8 ≈ 17,2 m/s. Bei einer Störung der Wetterstation wird der Alarm ausgelöst." UIHint="Information" />')
        if gkey == "MinMax":
            w(f'{" " * (ind + 2)}<ParameterSeparator Id="%AID%_PS-nnn" Text="Werte seit Mitternacht bzw. des Vortags (benötigt gültige Uhrzeit), bleiben über Neustarts erhalten. Temperatur = Temperatur für Berechnungen." UIHint="Information" />')
        q = " " * (ind + 2)
        if gkey == "Facades":
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Berechnet aus Globalstrahlung und Sonnenstand (benötigt Uhrzeit und Gerätestandort): direkte Sonne auf die Fassade plus diffuser und reflektierter Anteil. Die WN90LP misst nur waagerecht; Verschattung durch Nachbargebäude oder Bäume wird nicht erkannt." UIHint="Information" />')
        if gkey == "Ventilation":
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Taupunktlüftung: Lüften ist sinnvoll, wenn die Außenluft weniger Wasser enthält als die Raumluft (absolute Feuchte). Raumtemperatur und Raumfeuchte kommen über die Eingangsobjekte vom Bus; ohne Werte seit 2 Stunden wird keine Empfehlung gegeben." UIHint="Information" />')
        if gkey == "Irrigation":
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Tägliche Wasserbilanz für Garten/Rasen. Die Bilanz bleibt über Neustarts erhalten." UIHint="Information" />')
            for n in ("IrrKc", "IrrThreshold", "IrrStorage", "IrrAmount", "StationHeight"):
                w(f'{q}<ParameterRefRef RefId="{ids.ref(n)}" IndentLevel="1" />')
        if gkey == "Warnings2":
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Die Station wendet die Warnkriterien des Deutschen Wetterdienstes auf ihre eigenen Messwerte an und meldet, wenn die Bedingungen tatsächlich eintreten. Das ist keine amtliche Warnung und keine Vorhersage; Gewitter und Schnee werden nicht erkannt. Wind: Böen (10-min-Maximum), Starkregen: 1 h / 6 h, Dauerregen: 24 / 48 / 72 h, Frost/Glätte, Hitze: gefühlte Temperatur, UV-Index." UIHint="Information" />')
            w(f'{q}<ParameterRefRef RefId="{ids.ref("WarnHold")}" IndentLevel="1" />')
        if gkey == "DwdOfficial":
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Diese Funktion benötigt Internetzugang: Die Station ruft die amtlichen Warnungen für die eingestellte Warnzelle per HTTPS von maps.dwd.de ab (offene Daten des Deutschen Wetterdienstes, Datenbasis: DWD). Alle anderen Funktionen arbeiten ohne Internet." UIHint="Information" />')
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="WarnCellID: 9-stellige Nummer der Gemeinde (beginnt mit 8) oder des Landkreises (beginnt mit 1), Liste beim DWD unter „cap_warncellids_csv“ (z. B. 811000000 = Berlin)." UIHint="Information" />')
            w(f'{q}<ParameterRefRef RefId="{ids.ref("DwdActive")}" IndentLevel="1" />')
            w(f'{q}<choose ParamRefId="{ids.ref("DwdActive")}">')
            w(f'{q}  <when test="1">')
            w(f'{q}    <ParameterRefRef RefId="{ids.ref("DwdCell")}" IndentLevel="2" />')
            w(f'{q}    <ParameterRefRef RefId="{ids.ref("DwdInterval")}" IndentLevel="2" />')
            w(f'{q}  </when>')
            w(f'{q}</choose>')
        if gkey == "Particles":
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Auswertung" UIHint="Headline" />')
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Mittelwerte aus den feuchtekorrigierten Messwerten (ohne Luftfeuchte aus den Rohwerten). Der Luftqualitätsindex folgt dem Umweltbundesamt (24-Stunden-Mittel), die Ampel fasst ihn zusammen: grün = sehr gut/gut, gelb = mäßig, rot = schlecht/sehr schlecht. Der SDS011 ist ein preiswerter optischer Sensor; die Werte eignen sich für Tendenz und Ampel, nicht als Nachweis von Grenzwerten." UIHint="Information" />')
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="WHO-Leitlinie 2021: Tagesmittel PM2,5 15, PM10 45 µg/m³ (höchstens 3-4 Tage im Jahr darüber), Jahresmittel PM2,5 5, PM10 15 µg/m³. EU-Grenzwerte ab 2030: Tagesmittel PM2,5 25, PM10 45 µg/m³ (höchstens 18 Tage), Jahresmittel PM2,5 10, PM10 20 µg/m³." UIHint="Information" />')
            w(f'{q}<ParameterRefRef RefId="{ids.ref("PmLimit")}" IndentLevel="1" />')
        if gkey == "DegreeDays":
            w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="Gradtagzahl nach VDI 3807 (G20/15) und Kühlgradtage aus der Tagesmitteltemperatur (Verfahren wie bei Sommer/Winter), für den witterungsbereinigten Vergleich des Energieverbrauchs. Wird um Mitternacht fortgeschrieben und bleibt über Neustarts erhalten." UIHint="Information" />')
            for n in ("DdIndoor", "DdHeatLimit", "DdCoolBase"):
                w(f'{q}<ParameterRefRef RefId="{ids.ref(n)}" IndentLevel="1" />')
        for inp in INPUTS:
            if inp["group"] == gkey and not inp["cond"]:
                w(f'{q}<ComObjectRefRef RefId="%AID%_O-%TT%{inp["ko"]:05d}_R-%TT%{inp["ko"]:05d}01" />')
        if gkey in SECTIONS:
            count, prefix, title, pnames = SECTIONS[gkey]
            for n in range(1, count + 1):
                w(f'{q}<ParameterSeparator Id="%AID%_PS-nnn" Text="{title} {n}" UIHint="Headline" />')
                w(f'{q}<ParameterRefRef RefId="{ids.ref(f"{prefix}{n}Active")}" IndentLevel="1" />')
                w(f'{q}<choose ParamRefId="{ids.ref(f"{prefix}{n}Active")}">')
                w(f'{q}  <when test="1">')
                for pn in pnames:
                    w(f'{q}    <ParameterRefRef RefId="{ids.ref(f"{prefix}{n}{pn}")}" IndentLevel="2" />')
                for inp in INPUTS:
                    if inp["cond"] == f"{prefix}{n}Active":
                        w(f'{q}    <ComObjectRefRef RefId="%AID%_O-%TT%{inp["ko"]:05d}_R-%TT%{inp["ko"]:05d}01" />')
                for v in VALUES:
                    if v["cond"] == f"{prefix}{n}Active":
                        value_ui(v, ind + 6)
                w(f'{q}  </when>')
                w(f'{q}</choose>')
        for v in VALUES:
            if v["group"] != gkey or v["cond"]:
                continue
            if v["sensor"] and v["sensor"] != gsensor:
                w(f'{" " * (ind + 2)}<choose ParamRefId="{ids.ref(SENSOR_PARAM[v["sensor"]])}">')
                w(f'{" " * (ind + 2)}  <when test="1">')
                value_ui(v, ind + 6)
                w(f'{" " * (ind + 2)}  </when>')
                w(f'{" " * (ind + 2)}</choose>')
            else:
                value_ui(v, ind + 2)
        w(f'{" " * ind}</ParameterBlock>')
        if gsensor:
            ind -= 4
            w(f'{" " * ind}  </when>')
            w(f'{" " * ind}</choose>')
    w('            </Channel>')

    xml = f'''<?xml version="1.0" encoding="utf-8"?>
<!-- ERZEUGT durch scripts/generate_ets.py - nicht von Hand aendern -->
<KNX xmlns="http://knx.org/xml/project/20" xmlns:op="http://github.com/OpenKNX/OpenKNXproducer">
  <ManufacturerData>
    <Manufacturer>
      <ApplicationPrograms>
        <ApplicationProgram>
          <Static>
            <ParameterTypes>
{chr(10).join(ptypes)}
            </ParameterTypes>
            <Parameters>
              <Union SizeInBit="{total * 8}">
                <Memory CodeSegment="%MID%" Offset="0" BitOffset="0" />
{chr(10).join(params)}
              </Union>
            </Parameters>
            <ParameterRefs>
{chr(10).join(refs)}
            </ParameterRefs>
            <ComObjectTable>
{chr(10).join(kos)}
            </ComObjectTable>
            <ComObjectRefs>
{chr(10).join(korefs)}
            </ComObjectRefs>
          </Static>
          <Dynamic>
{chr(10).join(dyn)}
          </Dynamic>
        </ApplicationProgram>
      </ApplicationPrograms>
    </Manufacturer>
  </ManufacturerData>
</KNX>
'''
    SHARE.write_text(xml, encoding="utf-8")
    ids.save()

    # Firmware-Tabelle
    h = []
    h.append("#pragma once")
    h.append("// ERZEUGT durch scripts/generate_ets.py - nicht von Hand aendern")
    h.append('#include "knxprod.h"')
    h.append("#include <stdint.h>")
    h.append("")
    h.append("enum class WxId : uint8_t")
    h.append("{")
    for v in VALUES:
        h.append(f"    {v['key']},")
    h.append("    Count")
    h.append("};")
    h.append("")
    h.append("enum class WxSensor : uint8_t")
    h.append("{")
    h.append("    None,")
    h.append("    Sds,")
    h.append("    OneWire")
    h.append("};")
    h.append("")
    h.append("enum class WxFmt : uint8_t")
    h.append("{")
    h.append("    Number,")
    h.append("    Time,      // Sekunden seit Mitternacht (Ortszeit)")
    h.append("    Date,      // Tage seit 1.1.2000 (Ortszeit)")
    h.append("    Duration,  // Sekunden")
    h.append("    Forecast,  // Zambretti-Nummer, Text")
    h.append("    MoonPhase, // 0..7")
    h.append("    MoonText,  // 0..7, Text")
    h.append("    WarnText,  // Kategorie * 10 + Stufe, Text")
    h.append("    DwdText,   // Aenderungszaehler, Text aus dem DWD-Abruf")
    h.append("    AqiText,   // Luftqualitaetsindex 1..5, Text")
    h.append("    ComfortText // Behaglichkeit: Stufe * 16 + Gruende (Comfort.h), Text")
    h.append("};")
    h.append("")
    h.append("struct WxDptOption")
    h.append("{")
    h.append("    uint8_t main;   // Hauptgruppe fuer den knx-Stack")
    h.append("    uint16_t sub;   // Untergruppe fuer den knx-Stack (Kodierung, nicht ETS-DPT)")
    h.append("    float factor;   // Umrechnung Messwert -> Buswert")
    h.append("};")
    h.append("")
    h.append("struct WxValueDef")
    h.append("{")
    h.append("    const char *name;     // Anzeige (Konsole, ASCII)")
    h.append("    const char *label;    // Anzeige (Webseite, UTF-8)")
    h.append("    const char *group;    // ETS-Seite (Webseite, UTF-8)")
    h.append("    const char *unitUtf8; // Einheit (Webseite, UTF-8)")
    h.append("    const char *mqttKey;  // Feldname im MQTT-JSON")
    h.append("    const char *influxKey; // Feldname in InfluxDB (nullptr = nicht schreiben)")
    h.append("    const char *unit;")
    h.append("    uint16_t paramOffset; // erstes Byte des Parameterblocks")
    h.append("    uint16_t koNumber;")
    h.append("    WxSensor sensor;")
    h.append("    uint16_t condParam;   // Parameter, der aktiv sein muss (Bit 0), 0xFFFF = keiner")
    h.append("    bool thresholds;      // false: bei jeder Aenderung senden")
    h.append("    uint8_t decimals;")
    h.append("    WxFmt fmt;            // Darstellung (Zahl, Uhrzeit, Datum, Dauer, Text)")
    h.append("    const char *haClass;  // Home Assistant device_class (nullptr = keine)")
    h.append("    const char *haUnit;   // Home Assistant unit_of_measurement (nullptr = keine)")
    h.append("    uint8_t dptCount;")
    h.append("    WxDptOption dpts[3];")
    h.append("};")
    h.append("")
    h.append(f"#define WX_VALUE_COUNT {len(VALUES)}")
    h.append("#define WX_KO_FAULT_WN90 (WX_KoFaultWn90)")
    h.append("#define WX_KO_FAULT_SDS (WX_KoFaultSds)")
    h.append("#define WX_KO_FAULT_1W (WX_KoFault1W)")
    h.append("")
    h.append("static const WxValueDef WX_VALUES[WX_VALUE_COUNT] = {")
    sensors = {None: "WxSensor::None", "sds": "WxSensor::Sds", "onewire": "WxSensor::OneWire"}
    for v in VALUES:
        opts = ", ".join(f"{{{d[3]}, {d[4]}, {cfloat(d[5])}}}" for d in v["dpts"])
        unit = v["unit"].replace("°C", "C").replace("°", "Grad").replace("²", "2").replace("³", "3").replace("µ", "u")
        name = v["text"]
        for a, b in (("ä", "ae"), ("ö", "oe"), ("ü", "ue"), ("Ä", "Ae"), ("Ö", "Oe"), ("Ü", "Ue"), ("ß", "ss"),
                     (" °C", " C"), ("°", " Grad"), ("²", "2"), ("³", "3"), ("µ", "u"), ("–", "-")):
            name = name.replace(a, b)
        gtext = next(g[1] for g in GROUPS if g[0] == v["group"])
        h.append(f'    {{"{name}", {cstr(v["text"])}, {cstr(gtext)}, {cstr(v["unit"])}, "{v["mqtt"]}", {"nullptr" if v["influx"] == "" else ('"' + (v["influx"] or v["mqtt"]) + '"')}, "{unit}", WX_{v["key"]}Active, '
                 f'WX_Ko{v["key"]}, {sensors[v["sensor"]]}, {("WX_" + v["cond"]) if v["cond"] else "0xFFFF"}, {"true" if v["thresholds"] else "false"}, '
                 f'{v["decimals"]}, WxFmt::{ {"number": "Number", "time": "Time", "date": "Date", "duration": "Duration", "forecast": "Forecast", "moonphase": "MoonPhase", "moontext": "MoonText", "warntext": "WarnText", "dwdtext": "DwdText", "aqitext": "AqiText", "comforttext": "ComfortText"}[v["fmt"]] }, {cstr(ha_meta(v)[0])}, {cstr(ha_meta(v)[1])}, {len(v["dpts"])}, {{{opts}}}}},')
    h.append("};")
    h.append("")
    HEADER.write_text("\n".join(h) + "\n", encoding="utf-8")
    # KO-Liste fuer die Doku und fuer Logikkanaele ("bestehendes KO")
    gnames = {g[0]: g[1] for g in GROUPS}
    md = ["# Kommunikationsobjekte der Wetterstation", "",
          "ERZEUGT durch `scripts/generate_ets.py`. Die Nummern bleiben bei Updates gleich (neue Werte werden hinten angefügt).", "",
          "Für Logikkanäle („Bestehendes KO“): Das KO muss in der ETS aktiv (sichtbar) sein, der DPT im Logikkanal muss zur",
          "gewählten DPT-Variante passen, und der Wert erreicht die Logik nur, wenn er gesendet wird (zyklisch oder bei Änderung).", "",
          "| KO | Name | ETS-Seite | DPT (Auswahl, erste = Standard) |", "|---:|---|---|---|"]
    for n, (name, text, _) in enumerate(FAULTS, start=1):
        md.append(f"| {KO_OFFSET + n} | {text} | Allgemein | 1.005 |")
    for v in VALUES:
        def dptname(d):
            parts = d[1].replace("DPST-", "").replace("DPT-", "").split("-")
            return f"{parts[0]}.{int(parts[1]):03d}" if len(parts) > 1 else f"{parts[0]}.x"
        dpts = ", ".join(dptname(d) for d in v["dpts"])
        md.append(f"| {KO_OFFSET + v['ko']} | {v['text']} | {gnames[v['group']]} | {dpts} |")
    for inp in INPUTS:
        md.append(f"| {KO_OFFSET + inp['ko']} | {inp['text']} (Eingang) | {gnames[inp['group']]} | "
                  f"{inp['dpt'].replace('DPST-', '').split('-')[0]}.{int(inp['dpt'].split('-')[2]):03d} |")
    md.append("")
    md.append("Logikmodul: 30 Kanäle, KOs 300–389 (je Kanal Eingang 1, Eingang 2, Ausgang).")
    KOLIST.parent.mkdir(exist_ok=True)
    KOLIST.write_text("\n".join(md) + "\n", encoding="utf-8")

    print(f"{SHARE.relative_to(ROOT)}: {len(VALUES)} Messwerte, {total} Byte Parameter, KO 1-{kono - 1}")
    print(f"{HEADER.relative_to(ROOT)} geschrieben")


if __name__ == "__main__":
    build()
