#include "DwdWarnings.h"
#include "DwdRootCa.h"
#include "MeasuredValue.h"
#include "OpenKNX.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <WiFi.h>

DwdWarnings dwdWarnings;
std::string DwdWarnings::s_event;

// "2026-10-15T22:52:00Z" -> Unixzeit (0 bei Fehler)
static int64_t parseIso(const char *s)
{
    int y, mo, d, h, mi, se;
    if (!s || sscanf(s, "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &se) != 6)
        return 0;
    return (int64_t)MeasuredValue::daysFromCivil(y, mo, d) * 86400 + h * 3600 + mi * 60 + se;
}

static uint8_t severityLevel(const char *s)
{
    if (!s)
        return 0;
    if (!strcmp(s, "Extreme"))
        return 4;
    if (!strcmp(s, "Severe"))
        return 3;
    if (!strcmp(s, "Moderate"))
        return 2;
    if (!strcmp(s, "Minor"))
        return 1;
    return 0;
}

void DwdWarnings::setup(uint32_t warnCell, uint8_t intervalMin)
{
    if (warnCell < 100000000)
    {
        logError("Weather", "DWD: ungueltige Warnzellen-ID %lu, Abruf deaktiviert", warnCell);
        return;
    }
    // Landkreise (1xxxxxxxx) und Gemeinden (sonst) liegen in verschiedenen Ebenen mit unterschiedlichem Filterfeld
    const bool district = warnCell / 100000000 == 1;
    char url[400];
    snprintf(url, sizeof(url),
             "https://maps.dwd.de/geoserver/dwd/ows?service=WFS&version=2.0.0&request=GetFeature&typeName=dwd:%s"
             "&propertyName=EVENT,SEVERITY,HEADLINE,ONSET,EXPIRES&CQL_FILTER=%s%%3D%%27%09lu%%27&outputFormat=application/json",
             district ? "Warnungen_Landkreise" : "Warnungen_Gemeinden", district ? "GC_WARNCELLID" : "WARNCELLID", warnCell);
    _url = url;
    _intervalMs = (uint32_t)max<uint8_t>(intervalMin, 2) * 60000UL;
    // TLS braucht reichlich Stack
    _enabled = xTaskCreate(taskEntry, "DwdWarnings", 12288, this, 1, &_task) == pdPASS;
    logInfo("Weather", "DWD-Warnungen fuer Warnzelle %lu alle %u min", warnCell, intervalMin);
}

void DwdWarnings::loop(int64_t nowUtc)
{
    if (!_enabled || _busy || !WiFi.isConnected())
        return;
    // Erster Abruf 30 s nach dem Start, danach im Intervall
    if (_first ? millis() < 30000 : !delayCheck(_timer, _intervalMs))
        return;
    _first = false;
    _timer = millis();
    _requestNow = nowUtc;
    _busy = true;
    xTaskNotifyGive(_task);
}

void DwdWarnings::taskEntry(void *arg)
{
    static_cast<DwdWarnings *>(arg)->run();
}

void DwdWarnings::run()
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        try
        {
            fetch(_requestNow);
        }
        catch (const std::bad_alloc &)
        {
            // Speichermangel (TLS braucht viel): Abruf als Fehler werten, naechstes Intervall erneut
            std::lock_guard<std::mutex> guard(_lock);
            _result.lastCode = -200;
            _result.errors++;
        }
        _busy = false;
    }
}

void DwdWarnings::fetch(int64_t nowUtc)
{
    NetworkClientSecure client;
    client.setCACert(DWD_ROOT_CA);
    client.setTimeout(10);
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(10000);

    int code = -1;
    Result r;
    {
        std::lock_guard<std::mutex> guard(_lock);
        r = _result; // Zaehler und letzte Daten behalten
    }
    if (http.begin(client, _url.c_str()))
    {
        code = http.GET();
        if (code == 200)
        {
            // Nur die benoetigten Felder einlesen (Datenstrom-Filter)
            JsonDocument filter;
            JsonObject props = filter["features"][0]["properties"].to<JsonObject>();
            for (const char *k : {"EVENT", "SEVERITY", "HEADLINE", "ONSET", "EXPIRES"})
                props[k] = true;
            JsonDocument doc;
            const DeserializationError err = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
            if (err)
                code = -100;
            else
            {
                uint8_t levelNow = 0, levelAll = 0, countNow = 0, bestLevel = 0;
                int64_t bestOnset = INT64_MAX;
                bool bestIsNow = false, haveBest = false;
                std::string event, headline, onset, expires;
                for (JsonObject f : doc["features"].as<JsonArray>())
                {
                    JsonObject p = f["properties"];
                    const uint8_t level = severityLevel(p["SEVERITY"]);
                    const int64_t on = parseIso(p["ONSET"]);
                    const int64_t ex = parseIso(p["EXPIRES"]);
                    // Ohne Uhrzeit gelten alle gelieferten Warnungen als aktuell
                    const bool now = nowUtc == 0 || ((on == 0 || on <= nowUtc) && (ex == 0 || nowUtc < ex));
                    levelAll = max(levelAll, level);
                    if (now)
                    {
                        countNow++;
                        levelNow = max(levelNow, level);
                    }
                    // Massgebliche Warnung: zuerst gueltige vor angekuendigten, dann hoechste Stufe, dann frueherer Beginn
                    const bool better = !haveBest || (now && !bestIsNow) ||
                                        (now == bestIsNow && (level > bestLevel || (level == bestLevel && on < bestOnset)));
                    if (better)
                    {
                        haveBest = true;
                        bestIsNow = now;
                        bestLevel = level;
                        bestOnset = on;
                        event = p["EVENT"] | "";
                        headline = p["HEADLINE"] | "";
                        onset = p["ONSET"] | "";
                        expires = p["EXPIRES"] | "";
                    }
                }
                r.valid = true;
                r.levelNow = levelNow;
                r.levelAll = levelAll;
                r.countNow = countNow;
                r.event = event;
                r.headline = headline;
                r.onset = onset;
                r.expires = expires;
                r.updated = millis();
            }
        }
        http.end();
    }
    r.lastCode = code;
    if (code == 200)
        r.ok++;
    else
        r.errors++;
    std::lock_guard<std::mutex> guard(_lock);
    _result = r;
}

DwdWarnings::Result DwdWarnings::result()
{
    std::lock_guard<std::mutex> guard(_lock);
    return _result;
}

bool DwdWarnings::faulty()
{
    std::lock_guard<std::mutex> guard(_lock);
    // Stoerung, wenn seit drei Intervallen kein erfolgreicher Abruf (bzw. noch nie nach 3 Intervallen)
    const uint32_t since = _result.updated ? millis() - _result.updated : millis();
    return _enabled && since > 3 * _intervalMs + 60000;
}

const char *DwdWarnings::eventText(bool utf8)
{
    s_event = dwdWarnings.result().event;
    if (s_event.empty())
        s_event = "Keine Warnung";
    if (utf8)
        return s_event.c_str();
    // UTF-8 -> ISO 8859-1 (2-Byte-Folgen C2/C3), auf 14 Zeichen begrenzt
    static char iso[15];
    uint8_t n = 0;
    for (const uint8_t *s = (const uint8_t *)s_event.c_str(); *s && n < 14; s++)
    {
        if ((*s == 0xC2 || *s == 0xC3) && s[1])
        {
            iso[n++] = (char)(((*s & 0x03) << 6) | (s[1] & 0x3F));
            s++;
        }
        else if (*s < 0x80)
            iso[n++] = (char)*s;
    }
    iso[n] = 0;
    return iso;
}
