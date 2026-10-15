#include "InfluxWriter.h"
#include "OpenKNX.h"
#include <HTTPClient.h>
#include <WiFi.h>

// Prozentkodierung fuer Datenbank- und Retention-Policy-Namen in der URL
static std::string urlEncode(const std::string &s)
{
    static const char hex[] = "0123456789ABCDEF";
    std::string out;
    for (unsigned char c : s)
    {
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
            out += (char)c;
        else
        {
            out += '%';
            out += hex[c >> 4];
            out += hex[c & 0x0F];
        }
    }
    return out;
}

void InfluxWriter::setup(const std::string &host, uint16_t port, const std::string &db, const std::string &rp,
                         const std::string &user, const std::string &password, uint32_t intervalMs)
{
    if (host.empty() || db.empty())
    {
        logError("Weather", "InfluxDB: Server oder Datenbank fehlt, Schreiben deaktiviert");
        return;
    }
    _url = "http://" + host + ":" + std::to_string(port) + "/write?db=" + urlEncode(db) + "&precision=s";
    if (!rp.empty())
        _url += "&rp=" + urlEncode(rp);
    _user = user;
    _password = password;
    _intervalMs = max<uint32_t>(intervalMs, 10000);
    _timer = millis();
    _enabled = xTaskCreate(taskEntry, "InfluxWriter", 6144, this, 1, &_task) == pdPASS;
    logInfo("Weather", "InfluxDB: %s alle %lu s%s", _url.c_str(), _intervalMs / 1000, _enabled ? "" : " (Task-Fehler)");
}

bool InfluxWriter::due()
{
    if (!_enabled || _busy || !WiFi.isConnected() || !delayCheck(_timer, _intervalMs))
        return false;
    _timer = millis();
    return true;
}

void InfluxWriter::submit(std::string &&body)
{
    if (!_enabled || body.empty())
        return;
    {
        std::lock_guard<std::mutex> guard(_lock);
        _body = std::move(body);
    }
    _busy = true;
    xTaskNotifyGive(_task);
}

void InfluxWriter::taskEntry(void *arg)
{
    static_cast<InfluxWriter *>(arg)->run();
}

void InfluxWriter::run()
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        std::string body;
        {
            std::lock_guard<std::mutex> guard(_lock);
            body.swap(_body);
        }

        WiFiClient client;
        HTTPClient http;
        http.setConnectTimeout(3000);
        http.setTimeout(5000);
        int code = -1;
        if (http.begin(client, _url.c_str()))
        {
            if (!_user.empty())
                http.setAuthorization(_user.c_str(), _password.c_str());
            http.addHeader("Content-Type", "text/plain; charset=utf-8");
            code = http.POST((uint8_t *)body.data(), body.size());
            if (code != 204)
            {
                // Kein Log aus dieser Task (Logger nicht task-sicher); die Hauptschleife meldet den Fehler
                String detail = code > 0 ? http.getString() : HTTPClient::errorToString(code);
                std::lock_guard<std::mutex> guard(_lock);
                _lastError = std::string(detail.c_str()).substr(0, 120);
            }
            http.end();
        }
        _lastCode = code;
        if (code == 204)
        {
            _ok = _ok + 1;
            _lastOk = millis();
        }
        else
            _errors = _errors + 1;
        _busy = false;
    }
}

std::string InfluxWriter::lastError()
{
    std::lock_guard<std::mutex> guard(_lock);
    return _lastError;
}
