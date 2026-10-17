#!/usr/bin/env python3
"""Korrigiert fehlerhafte Windchill-Werte (Firmware bis 1.0.0) in InfluxDB 1.x.

Berechnet windchill aus temp und windspeed (m/s) desselben Punkts neu, wenn der gespeicherte Wert mehr
als 5 K darunter liegt, und schreibt den Wert mit
gleichem Zeitstempel und gleichen Tags zurueck (InfluxDB ueberschreibt nur dieses Feld).
Ohne --apply wird nur angezeigt, was geaendert wuerde.

Aufruf:  python3 fix_windchill.py [--days 30] [--apply]
Zugangsdaten (falls noetig) ueber Umgebungsvariablen INFLUX_USER / INFLUX_PASSWORD.
"""
import argparse
import json
import math
import os
import urllib.parse
import urllib.request

URL = "http://192.168.2.151:8096"
DB = "p3"
MEASUREMENT = "weather"
TAGS = {"floor": "Aussen", "room": "Garten", "sensor": "wn90"}


def auth_params():
    user, pw = os.environ.get("INFLUX_USER"), os.environ.get("INFLUX_PASSWORD")
    return {"u": user, "p": pw} if user else {}


def query(q):
    params = {"db": DB, "epoch": "ns", "q": q, **auth_params()}
    with urllib.request.urlopen(f"{URL}/query?{urllib.parse.urlencode(params)}", timeout=120) as r:
        res = json.load(r)["results"][0]
    if "error" in res:
        raise SystemExit(f"Abfragefehler: {res['error']}")
    return res.get("series", [])


def write(lines):
    params = {"db": DB, "precision": "ns", **auth_params()}
    req = urllib.request.Request(f"{URL}/write?{urllib.parse.urlencode(params)}", data="\n".join(lines).encode(), method="POST")
    with urllib.request.urlopen(req, timeout=120) as r:
        if r.status != 204:
            raise SystemExit(f"Schreibfehler: HTTP {r.status}")


def windchill(t, ms):
    kmh = ms * 3.6
    if t > 10.0 or kmh < 5.0:
        return t
    return min(t, 13.12 + 0.6215 * t + (0.3965 * t - 11.37) * kmh ** 0.16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", type=int, default=30, help="Zeitraum rueckwirkend in Tagen (Standard 30)")
    ap.add_argument("--apply", action="store_true", help="Korrekturen tatsaechlich schreiben")
    args = ap.parse_args()

    where = " AND ".join(f"\"{k}\" = '{v}'" for k, v in TAGS.items())
    tagstr = "".join(f",{k}={v}" for k, v in TAGS.items())
    lines, undo, worst, total = [], [], [], 0
    # Tageweise abfragen, damit die Antworten klein bleiben
    for day in range(args.days, 0, -1):
        q = (f"SELECT temp, windspeed, windchill FROM {MEASUREMENT} WHERE {where} "
             f"AND time > now() - {day}d AND time <= now() - {day - 1}d")
        for s in query(q):
            for ts, t, ws, wc in s["values"]:
                if t is None or ws is None or wc is None:
                    continue
                total += 1
                new = round(windchill(t, ws), 2)
                # Nur eindeutig fehlerhafte Werte: der Fehler drueckt den Windchill um mindestens ~20 K.
                # Kleinere Abweichungen sind korrekt (z. B. 1-Wire als Temperatur fuer Berechnungen).
                if new - wc > 5.0:
                    lines.append(f"{MEASUREMENT}{tagstr} windchill={new} {ts}")
                    undo.append(f"{MEASUREMENT}{tagstr} windchill={wc} {ts}")
                    worst.append((wc, new, t, ws))

    print(f"{total} Punkte geprueft, {len(lines)} zu korrigieren.")
    for wc, new, t, ws in sorted(worst)[:10]:
        print(f"  alt {wc:8.2f}  neu {new:6.2f}  (temp {t:.1f} C, wind {ws:.1f} m/s)")
    if not lines:
        return
    if not args.apply:
        print("Probelauf. Zum Schreiben mit --apply aufrufen.")
        return
    # Alte Werte sichern; Rueckgabe mit: curl -XPOST '<URL>/write?db=<DB>&precision=ns' --data-binary @<Datei>
    with open("windchill_undo.lp", "w") as f:
        f.write("\n".join(undo) + "\n")
    print("Alte Werte gesichert in windchill_undo.lp (Line Protocol).")
    for i in range(0, len(lines), 5000):
        write(lines[i:i + 5000])
    print(f"{len(lines)} Werte korrigiert.")


if __name__ == "__main__":
    main()
