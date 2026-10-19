# Kommunikationsobjekte der Wetterstation

ERZEUGT durch `scripts/generate_ets.py`. Die Nummern bleiben bei Updates gleich (neue Werte werden hinten angefügt).

Für Logikkanäle („Bestehendes KO“): Das KO muss in der ETS aktiv (sichtbar) sein, der DPT im Logikkanal muss zur
gewählten DPT-Variante passen, und der Wert erreicht die Logik nur, wenn er gesendet wird (zyklisch oder bei Änderung).

| KO | Name | ETS-Seite | DPT (Auswahl, erste = Standard) |
|---:|---|---|---|
| 21 | Störung Wetterstation | Allgemein | 1.005 |
| 22 | Störung Feinstaubsensor | Allgemein | 1.005 |
| 23 | Störung 1-Wire-Sensor | Allgemein | 1.005 |
| 24 | Temperatur | Temperatur | 9.001, 14.068 |
| 25 | Temperatur 1-Wire | Temperatur | 9.001, 14.068 |
| 26 | Relative Luftfeuchte | Luftfeuchte | 9.007, 5.001 |
| 27 | Absolute Luftfeuchte | Luftfeuchte | 9.029 |
| 28 | Taupunkt | Luftfeuchte | 9.001, 14.068 |
| 29 | Frostpunkt | Luftfeuchte | 9.001, 14.068 |
| 30 | Humidex | Behaglichkeit | 9.001, 14.068 |
| 31 | Hitzeindex | Behaglichkeit | 9.001, 14.068 |
| 32 | Windchill | Behaglichkeit | 9.001, 14.068 |
| 33 | Discomfort-Index | Behaglichkeit | 9.001, 14.068 |
| 34 | Windgeschwindigkeit | Wind | 9.005, 9.028, 14.065 |
| 35 | Böengeschwindigkeit | Wind | 9.005, 9.028, 14.065 |
| 36 | Windstärke (Beaufort) | Wind | 5.010 |
| 37 | Böenstärke (Beaufort) | Wind | 5.010 |
| 38 | Windrichtung | Wind | 5.003, 14.007 |
| 39 | Luftdruck (absolut) | Luftdruck | 9.006, 14.058 |
| 40 | Luftdruck (auf Meereshöhe) | Luftdruck | 9.006, 14.058 |
| 41 | Luftdrucktendenz 1 Stunde | Luftdruck | 9.006, 14.058 |
| 42 | Luftdrucktendenz 3 Stunden | Luftdruck | 9.006, 14.058 |
| 43 | Helligkeit | Helligkeit und UV | 9.004 |
| 44 | UV-Index | Helligkeit und UV | 9.x, 5.010 |
| 45 | Regenmenge gesamt | Regen | 9.026, 14.x |
| 46 | Regenintensität | Regen | 9.x |
| 47 | Regenmenge letzte 5 Minuten | Regen | 9.026 |
| 48 | Regenmenge letzte Stunde | Regen | 9.026 |
| 49 | Regenmenge letzte 6 Stunden | Regen | 9.026 |
| 50 | Regenmenge letzte 24 Stunden | Regen | 9.026 |
| 51 | Regenmenge letzte 48 Stunden | Regen | 9.026 |
| 52 | Regenmenge letzte 72 Stunden | Regen | 9.026 |
| 53 | Regenmenge heute | Regen | 9.026 |
| 54 | Regen | Regen | 1.002 |
| 55 | Feinstaub PM2,5 | Feinstaub | 9.030 |
| 56 | Feinstaub PM10 | Feinstaub | 9.030 |
| 57 | Feinstaub PM2,5 (feuchtekorrigiert) | Feinstaub | 9.030 |
| 58 | Feinstaub PM10 (feuchtekorrigiert) | Feinstaub | 9.030 |
| 59 | Wettertendenz | Prognose und Display | 6.010 |
| 60 | Wetterprognose (Zambretti-Nummer) | Prognose und Display | 5.010 |
| 61 | Wetterprognose (Text) | Prognose und Display | 16.001 |
| 62 | Wettersymbol (Display) | Prognose und Display | 5.010 |
| 63 | Windgeschwindigkeit (10-min-Mittel) | Wind | 9.005, 9.028, 14.065 |
| 64 | Böe (Maximum 10 min) | Wind | 9.005, 9.028, 14.065 |
| 65 | Windalarm Stufe 1 | Windalarm | 1.005 |
| 66 | Windalarm Stufe 2 | Windalarm | 1.005 |
| 67 | Windalarm Stufe 3 | Windalarm | 1.005 |
| 68 | Sonnenstand Azimut | Sonne und Dämmerung | 14.007 |
| 69 | Sonnenstand Elevation | Sonne und Dämmerung | 14.007 |
| 70 | Tag/Nacht (Dämmerung) | Sonne und Dämmerung | 1.024 |
| 71 | Temperatur Minimum heute | Minimum und Maximum | 9.001, 14.068 |
| 72 | Temperatur Maximum heute | Minimum und Maximum | 9.001, 14.068 |
| 73 | Temperatur Minimum gestern | Minimum und Maximum | 9.001, 14.068 |
| 74 | Temperatur Maximum gestern | Minimum und Maximum | 9.001, 14.068 |
| 75 | Stärkste Böe heute | Minimum und Maximum | 9.005, 9.028, 14.065 |
| 76 | Stärkste Böe gestern | Minimum und Maximum | 9.005, 9.028, 14.065 |
| 77 | Frost | Warnungen | 1.005 |
| 78 | Glättegefahr | Warnungen | 1.005 |
| 79 | Sommer/Winter | Sommer/Winter und Kenntage | 1.002, 1.100 |
| 80 | Mittlere Außentemperatur (mehrere Tage) | Sommer/Winter und Kenntage | 9.001, 14.068 |
| 81 | Tagesmitteltemperatur gestern | Sommer/Winter und Kenntage | 9.001, 14.068 |
| 82 | Sommertag (Maximum ab 25 °C) | Sommer/Winter und Kenntage | 1.002 |
| 83 | Heißer Tag (Maximum ab 30 °C) | Sommer/Winter und Kenntage | 1.002 |
| 84 | Frosttag (Minimum unter 0 °C) | Sommer/Winter und Kenntage | 1.002 |
| 85 | Eistag (Maximum unter 0 °C) | Sommer/Winter und Kenntage | 1.002 |
| 86 | Globalstrahlung (geschätzt) | Strahlung und PV | 9.022 |
| 87 | Bewölkungsgrad (geschätzt) | Strahlung und PV | 5.001 |
| 88 | Einstrahlung heute | Strahlung und PV | 14.x |
| 89 | Einstrahlung gestern | Strahlung und PV | 14.x |
| 90 | PV-Leistung (geschätzt) | Strahlung und PV | 14.056 |
| 91 | PV-Ertrag heute (geschätzt) | Strahlung und PV | 13.010, 14.x |
| 92 | PV-Ertrag gestern (geschätzt) | Strahlung und PV | 13.010, 14.x |
| 93 | Sonne auf Fassade 1 | Fassaden | 1.002 |
| 94 | Einstrahlung Fassade 1 (geschätzt) | Fassaden | 9.022 |
| 95 | Profilwinkel Fassade 1 | Fassaden | 14.007 |
| 96 | Sonne auf Fassade 2 | Fassaden | 1.002 |
| 97 | Einstrahlung Fassade 2 (geschätzt) | Fassaden | 9.022 |
| 98 | Profilwinkel Fassade 2 | Fassaden | 14.007 |
| 99 | Sonne auf Fassade 3 | Fassaden | 1.002 |
| 100 | Einstrahlung Fassade 3 (geschätzt) | Fassaden | 9.022 |
| 101 | Profilwinkel Fassade 3 | Fassaden | 14.007 |
| 102 | Sonne auf Fassade 4 | Fassaden | 1.002 |
| 103 | Einstrahlung Fassade 4 (geschätzt) | Fassaden | 9.022 |
| 104 | Profilwinkel Fassade 4 | Fassaden | 14.007 |
| 105 | Sonne auf Fassade 5 | Fassaden | 1.002 |
| 106 | Einstrahlung Fassade 5 (geschätzt) | Fassaden | 9.022 |
| 107 | Profilwinkel Fassade 5 | Fassaden | 14.007 |
| 108 | Sonne auf Fassade 6 | Fassaden | 1.002 |
| 109 | Einstrahlung Fassade 6 (geschätzt) | Fassaden | 9.022 |
| 110 | Profilwinkel Fassade 6 | Fassaden | 14.007 |
| 111 | Sonnenaufgang | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 112 | Sonnenuntergang | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 113 | Sonnenhöchststand (Transit) | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 114 | Beginn bürgerliche Dämmerung | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 115 | Ende bürgerliche Dämmerung | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 116 | Beginn nautische Dämmerung | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 117 | Ende nautische Dämmerung | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 118 | Beginn astronomische Dämmerung | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 119 | Ende astronomische Dämmerung | Sonnenzeiten | 10.001, 19.001, 12.001 |
| 120 | Azimut bei Sonnenaufgang | Sonnenzeiten | 14.007 |
| 121 | Azimut bei Sonnenuntergang | Sonnenzeiten | 14.007 |
| 122 | Elevation beim Höchststand | Sonnenzeiten | 14.007 |
| 123 | Tageslänge | Sonnenzeiten | 13.100, 7.006 |
| 124 | Tagesfortschritt (Aufgang bis Untergang) | Sonnenzeiten | 5.001 |
| 125 | Sonne steigt (0) / fällt (1) | Sonnenzeiten | 1.002 |
| 126 | Tag/Nacht (Sonnenstand) | Sonnenzeiten | 1.024 |
| 127 | Tag/Nacht (bürgerliche Dämmerung, -6°) | Sonnenzeiten | 1.024 |
| 128 | Tag/Nacht (nautische Dämmerung, -12°) | Sonnenzeiten | 1.024 |
| 129 | Tag/Nacht (astronomische Dämmerung, -18°) | Sonnenzeiten | 1.024 |
| 130 | Globalstrahlung bei klarem Himmel | Sonnenzeiten | 9.022 |
| 131 | Globalstrahlung bei klarem Himmel, Tagesmaximum | Sonnenzeiten | 9.022 |
| 132 | Sonnenscheindauer heute | Sonnenzeiten | 9.x, 14.x |
| 133 | Sonnenscheindauer gestern | Sonnenzeiten | 9.x, 14.x |
| 134 | Mondphase (0-7) | Mond | 5.010 |
| 135 | Mondphase (Text) | Mond | 16.001 |
| 136 | Mond beleuchtet | Mond | 5.001 |
| 137 | Mondalter (Tage seit Neumond) | Mond | 9.x |
| 138 | Mond Azimut | Mond | 14.007 |
| 139 | Mond Elevation | Mond | 14.007 |
| 140 | Mondaufgang | Mond | 10.001, 19.001, 12.001 |
| 141 | Monduntergang | Mond | 10.001, 19.001, 12.001 |
| 142 | Nächster Vollmond | Mond | 11.001, 19.001, 12.001 |
| 143 | Nächster Neumond | Mond | 11.001, 19.001, 12.001 |
| 144 | Lüften sinnvoll Raum 1 | Lüftungsempfehlung | 1.002 |
| 145 | Absolute Feuchte Raum 1 | Lüftungsempfehlung | 9.029 |
| 146 | Taupunkt Raum 1 | Lüftungsempfehlung | 9.001, 14.068 |
| 147 | Feuchteunterschied innen − außen Raum 1 | Lüftungsempfehlung | 9.029 |
| 148 | Lüften sinnvoll Raum 2 | Lüftungsempfehlung | 1.002 |
| 149 | Absolute Feuchte Raum 2 | Lüftungsempfehlung | 9.029 |
| 150 | Taupunkt Raum 2 | Lüftungsempfehlung | 9.001, 14.068 |
| 151 | Feuchteunterschied innen − außen Raum 2 | Lüftungsempfehlung | 9.029 |
| 152 | Lüften sinnvoll Raum 3 | Lüftungsempfehlung | 1.002 |
| 153 | Absolute Feuchte Raum 3 | Lüftungsempfehlung | 9.029 |
| 154 | Taupunkt Raum 3 | Lüftungsempfehlung | 9.001, 14.068 |
| 155 | Feuchteunterschied innen − außen Raum 3 | Lüftungsempfehlung | 9.029 |
| 156 | Verdunstung heute (ET0) | Bewässerung | 9.026 |
| 157 | Verdunstung gestern (ET0) | Bewässerung | 9.026 |
| 158 | Wasserbilanz | Bewässerung | 9.x |
| 159 | Bewässern | Bewässerung | 1.002 |
| 160 | Heizgradtage laufender Monat | Gradtage | 9.x, 14.x |
| 161 | Heizgradtage Vormonat | Gradtage | 9.x, 14.x |
| 162 | Heizgradtage laufendes Jahr | Gradtage | 9.x, 14.x |
| 163 | Heizgradtage Vorjahr | Gradtage | 9.x, 14.x |
| 164 | Kühlgradtage laufender Monat | Gradtage | 9.x, 14.x |
| 165 | Kühlgradtage Vormonat | Gradtage | 9.x, 14.x |
| 166 | Kühlgradtage laufendes Jahr | Gradtage | 9.x, 14.x |
| 167 | Kühlgradtage Vorjahr | Gradtage | 9.x, 14.x |
| 175 | Warnstufe gesamt (0-4) | Wetterwarnungen (DWD-Kriterien) | 5.010 |
| 176 | Warnung (Text) | Wetterwarnungen (DWD-Kriterien) | 16.001 |
| 177 | Unwetter (Stufe 3 oder 4) | Wetterwarnungen (DWD-Kriterien) | 1.005 |
| 178 | Warnstufe Wind | Wetterwarnungen (DWD-Kriterien) | 5.010 |
| 179 | Warnstufe Starkregen | Wetterwarnungen (DWD-Kriterien) | 5.010 |
| 180 | Warnstufe Dauerregen | Wetterwarnungen (DWD-Kriterien) | 5.010 |
| 181 | Warnstufe Frost/Glätte | Wetterwarnungen (DWD-Kriterien) | 5.010 |
| 182 | Warnstufe Hitze | Wetterwarnungen (DWD-Kriterien) | 5.010 |
| 183 | Warnstufe UV | Wetterwarnungen (DWD-Kriterien) | 5.010 |
| 184 | DWD-Warnstufe aktuell (0-4) | DWD-Warnungen (Internet) | 5.010 |
| 185 | DWD-Warnstufe inkl. angekündigter Warnungen | DWD-Warnungen (Internet) | 5.010 |
| 186 | DWD-Warnung (Ereignis) | DWD-Warnungen (Internet) | 16.001 |
| 187 | DWD-Unwetterwarnung (Stufe 3 oder 4) | DWD-Warnungen (Internet) | 1.005 |
| 188 | Anzahl gültiger DWD-Warnungen | DWD-Warnungen (Internet) | 5.010 |
| 189 | Störung DWD-Abruf | DWD-Warnungen (Internet) | 1.005 |
| 190 | Regenmenge gestern | Regen | 9.026 |
| 191 | Regenmenge diese Woche | Regen | 9.026, 14.x |
| 192 | Regenmenge dieser Monat | Regen | 9.026, 14.x |
| 193 | Regenmenge dieses Jahr | Regen | 9.026, 14.x |
| 168 | Raumtemperatur Raum 1 (Eingang) | Lüftungsempfehlung | 9.001 |
| 169 | Raumfeuchte Raum 1 (Eingang) | Lüftungsempfehlung | 9.007 |
| 170 | Raumtemperatur Raum 2 (Eingang) | Lüftungsempfehlung | 9.001 |
| 171 | Raumfeuchte Raum 2 (Eingang) | Lüftungsempfehlung | 9.007 |
| 172 | Raumtemperatur Raum 3 (Eingang) | Lüftungsempfehlung | 9.001 |
| 173 | Raumfeuchte Raum 3 (Eingang) | Lüftungsempfehlung | 9.007 |
| 174 | Bewässerung erfolgt (Eingang) | Bewässerung | 1.017 |

Logikmodul: 30 Kanäle, KOs 300–389 (je Kanal Eingang 1, Eingang 2, Ausgang).
