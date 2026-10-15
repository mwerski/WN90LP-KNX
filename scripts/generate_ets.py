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

# Gruppen = ETS-Seiten
GROUPS = [
    ("Temperature", "Temperatur", "thermometer", None),
    ("Humidity", "Luftfeuchte", "water-percent", None),
    ("Comfort", "Behaglichkeit", "calculator-variant-outline", None),
    ("Wind", "Wind", "windsock", None),
    ("Pressure", "Luftdruck", "gauge", None),
    ("Light", "Helligkeit und UV", "white-balance-sunny", None),
    ("Rain", "Regen", "weather-pouring", None),
    ("Particles", "Feinstaub", "blur", "sds"),
    ("Forecast", "Prognose und Display", "weather-partly-snowy-rainy", None),
]


def V(key, group, text, unit, dpts, cycle, abs_, rel, active=False, sensor=None,
      thresholds=True, decimals=1, mqtt=None, absmax=100000, info=None, dpt=0):
    return dict(key=key, group=group, text=text, unit=unit, dpts=dpts, cycle=cycle,
                abs=abs_, rel=rel, active=active, sensor=sensor, thresholds=thresholds,
                decimals=decimals, mqtt=mqtt or key.lower(), absmax=absmax, info=info, dpt=dpt)


# Reihenfolge = KO-Reihenfolge und Reihenfolge im Parameterspeicher
VALUES = [
    V("Temp", "Temperature", "Temperatur", "°C", TEMP, 300, 0.5, 0, True, mqtt="temperature"),
    V("Temp1W", "Temperature", "Temperatur 1-Wire", "°C", TEMP, 300, 0.5, 0, True, sensor="onewire", decimals=2,
      mqtt="temperature1"),
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
      info="Gefühlte Temperatur bei Kälte und Wind. Gilt nur bis 10 °C und ab 5 km/h, sonst wird die Lufttemperatur gesendet."),
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
    V("Trend1h", "Pressure", "Luftdrucktendenz 1 Stunde", "hPa", PRESSURE, 900, 0.5, 0, mqtt="pressuretrend1",
      info="Differenz zum Luftdruck vor einer Stunde; verfügbar frühestens eine Stunde nach dem Start."),
    V("Trend3h", "Pressure", "Luftdrucktendenz 3 Stunden", "hPa", PRESSURE, 900, 0.5, 0, mqtt="pressuretrend3",
      info="Differenz zum Luftdruck vor drei Stunden; verfügbar frühestens drei Stunden nach dem Start."),
    V("Light", "Light", "Helligkeit", "lx", [T9(4, "DPT 9.004 (2 Byte, lx)")], 300, 0, 20, True, decimals=0,
      mqtt="light", absmax=300000),
    V("UvIndex", "Light", "UV-Index", "",
      [("DPT 9 (2 Byte)", "DPT-9", "2 Bytes", 9, 1, 1.0), ("DPT 5.010 (1 Byte, ganzzahlig)", "DPST-5-10", "1 Byte", 5, 10, 1.0)],
      300, 1, 0, True, mqtt="uvindex", absmax=15),
    V("RainTotal", "Rain", "Regenmenge gesamt", "mm", RAIN, 3600, 1, 0, True, decimals=2, mqtt="raincounter", dpt=1,
      info="Fortlaufender Zähler seit Inbetriebnahme, bleibt bei Neustart und Update erhalten. "
           "DPT 14 empfohlen: DPT 9 wird bei großen Werten grob (ab 1000 mm in Schritten von 0,64 mm)."),
    V("RainRate", "Rain", "Regenintensität", "mm/h", [("DPT 9 (2 Byte, mm/h)", "DPT-9", "2 Bytes", 9, 1, 1.0)],
      600, 1, 0, True, decimals=1, mqtt="rain_rate",
      info="Hochgerechnet aus der Regenmenge der letzten 10 Minuten."),
    V("Rain5m", "Rain", "Regenmenge letzte 5 Minuten", "mm", RAIN9, 300, 0.2, 0, decimals=2, mqtt="rain_5min"),
    V("Rain1h", "Rain", "Regenmenge letzte Stunde", "mm", RAIN9, 600, 0.5, 0, True, decimals=2, mqtt="rain_1h"),
    V("Rain6h", "Rain", "Regenmenge letzte 6 Stunden", "mm", RAIN9, 1800, 1, 0, decimals=2, mqtt="rain_6h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("Rain24h", "Rain", "Regenmenge letzte 24 Stunden", "mm", RAIN9, 3600, 1, 0, True, decimals=2, mqtt="rain_24h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("Rain48h", "Rain", "Regenmenge letzte 48 Stunden", "mm", RAIN9, 3600, 1, 0, decimals=2, mqtt="rain_48h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("Rain72h", "Rain", "Regenmenge letzte 72 Stunden", "mm", RAIN9, 3600, 1, 0, decimals=2, mqtt="rain_72h",
      info="Wird stündlich aktualisiert und umfasst je nach Zeitpunkt bis zu eine Stunde mehr."),
    V("RainToday", "Rain", "Regenmenge heute", "mm", RAIN9, 3600, 0.5, 0, True, decimals=2, mqtt="rain_today",
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
      mqtt="forecast_text", info="Kurztext der Zambretti-Prognose, z. B. „Wechsel,Regen“."),
    V("WeatherSymbol", "Forecast", "Wettersymbol (Display)", "", BFT, 600, 0, 0, True, thresholds=False, decimals=0,
      mqtt="weather_symbol",
      info="0 sonnig, 1 bewölkt, 2 Schauer, 3 starker Regen, 4 Gewitter, 5 hoher UV-Index (z. B. für Tantron-Displays). "
           "Gewitter ist eine Schätzung aus starkem Regen und Böen, die WN90LP misst keine Blitze."),
]

# Stoerungs-KOs vor den Messwerten
FAULTS = [
    ("FaultWn90", "Störung Wetterstation", None),
    ("FaultSds", "Störung Feinstaubsensor", "sds"),
    ("Fault1W", "Störung 1-Wire-Sensor", "onewire"),
]

SENSOR_PARAM = {"sds": "SdsPresent", "onewire": "OneWirePresent"}

# Allgemeine Parameter: (Name, Typ, Offset, Bit, Text, Default, Suffix)
GENERAL_SIZE = 24
GENERAL = [
    ("SdsPresent", "%AID%_PT-OnOffYesNo", 0, 0, "Feinstaubsensor SDS011 vorhanden", "0", None),
    ("OneWirePresent", "%AID%_PT-OnOffYesNo", 0, 1, "Temperatursensor 1-Wire (DS18B20) vorhanden", "0", None),
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
]

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


class Ids:
    """Fortlaufende Parameter-Ids (5-stellig wie in OpenKNX ueblich)."""

    def __init__(self):
        self.next = 1
        self.byname = {}

    def add(self, name):
        self.byname[name] = self.next
        self.next += 1
        return self.byname[name]

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
    kono = 1
    for name, text, _ in FAULTS:
        kos.append(f'              <ComObject Id="%AID%_O-%TT%{kono:05d}" Name="{name}" Text="{escape(text)}" Number="{kono}" '
                   'FunctionText="Alarm" ObjectSize="1 Bit" ReadFlag="Enabled" WriteFlag="Disabled" CommunicationFlag="Enabled" '
                   'TransmitFlag="Enabled" UpdateFlag="Disabled" ReadOnInitFlag="Disabled" DatapointType="DPST-1-5" />')
        korefs.append(f'              <ComObjectRef Id="%AID%_O-%TT%{kono:05d}_R-%TT%{kono:05d}01" RefId="%AID%_O-%TT%{kono:05d}" />')
        kono += 1
    for v in VALUES:
        v["ko"] = kono
        d0 = v["dpts"][0]
        kos.append(f'              <ComObject Id="%AID%_O-%TT%{kono:05d}" Name="{v["key"]}" Text="{escape(v["text"])}" '
                   f'Number="{kono}" FunctionText="Ausgang" ObjectSize="{d0[2]}" ReadFlag="Enabled" WriteFlag="Disabled" '
                   'CommunicationFlag="Enabled" TransmitFlag="Enabled" UpdateFlag="Disabled" ReadOnInitFlag="Disabled" '
                   f'DatapointType="{d0[1]}" />')
        for i, d in enumerate(v["dpts"]):
            korefs.append(f'              <ComObjectRef Id="%AID%_O-%TT%{kono:05d}_R-%TT%{kono:05d}{i + 1:02d}" '
                          f'RefId="%AID%_O-%TT%{kono:05d}" ObjectSize="{d[2]}" DatapointType="{d[1]}" />')
        kono += 1

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
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Standort und Korrektur" UIHint="Headline" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("Altitude")}" IndentLevel="1" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("TempOffset")}" IndentLevel="1" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Regen" UIHint="Headline" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Einzelne Zählerschritte (z. B. durch Tau oder Erschütterungen) lösen so noch keinen Regen aus." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("RainWindow")}" IndentLevel="1" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("RainMinAmount")}" IndentLevel="1" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("RainStopDelay")}" IndentLevel="1" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Nach der ersten Inbetriebnahme enthalten die Regenmengen nur die Zeit seit dem Start (z. B. 72 Stunden erst nach drei Tagen vollständig). Danach bleiben sie über Neustarts erhalten." UIHint="Information" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="MQTT" UIHint="Headline" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Broker, Zugangsdaten und Themenpräfix werden auf der Seite Netzwerk / MQTT eingestellt. Gesendet wird ein JSON-Objekt an openknx/&lt;Präfix&gt;/state." UIHint="Information" />')
    w(f'                <ParameterRefRef RefId="{ids.ref("MqttInterval")}" IndentLevel="1" />')
    w('                <ParameterSeparator Id="%AID%_PS-nnn" Text="Störungsmeldungen" UIHint="Headline" />')
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
        for v in VALUES:
            if v["group"] != gkey:
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
    h.append("struct WxDptOption")
    h.append("{")
    h.append("    uint8_t main;   // Hauptgruppe fuer den knx-Stack")
    h.append("    uint16_t sub;   // Untergruppe fuer den knx-Stack (Kodierung, nicht ETS-DPT)")
    h.append("    float factor;   // Umrechnung Messwert -> Buswert")
    h.append("};")
    h.append("")
    h.append("struct WxValueDef")
    h.append("{")
    h.append("    const char *name;     // Anzeige (Konsole)")
    h.append("    const char *mqttKey;  // Feldname im MQTT-JSON")
    h.append("    const char *unit;")
    h.append("    uint16_t paramOffset; // erstes Byte des Parameterblocks")
    h.append("    uint16_t koNumber;")
    h.append("    WxSensor sensor;")
    h.append("    bool thresholds;      // false: bei jeder Aenderung senden")
    h.append("    uint8_t decimals;")
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
        unit = v["unit"].replace("°C", "C").replace("°", "Grad").replace("³", "3").replace("µ", "u")
        name = v["text"]
        for a, b in (("ä", "ae"), ("ö", "oe"), ("ü", "ue"), ("Ä", "Ae"), ("Ö", "Oe"), ("Ü", "Ue"), ("ß", "ss")):
            name = name.replace(a, b)
        h.append(f'    {{"{name}", "{v["mqtt"]}", "{unit}", WX_{v["key"]}Active, '
                 f'WX_Ko{v["key"]}, {sensors[v["sensor"]]}, {"true" if v["thresholds"] else "false"}, '
                 f'{v["decimals"]}, {len(v["dpts"])}, {{{opts}}}}},')
    h.append("};")
    h.append("")
    HEADER.write_text("\n".join(h) + "\n", encoding="utf-8")
    print(f"{SHARE.relative_to(ROOT)}: {len(VALUES)} Messwerte, {total} Byte Parameter, KO 1-{kono - 1}")
    print(f"{HEADER.relative_to(ROOT)} geschrieben")


if __name__ == "__main__":
    build()
