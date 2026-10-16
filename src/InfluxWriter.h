#pragma once
#include <Arduino.h>
#include <mutex>
#include <string>

// Schreibt Line-Protocol-Daten per HTTP POST an InfluxDB 1.x (/write, ohne TLS).
// Die Anfrage laeuft in einer eigenen Task, damit ein langsamer Server die Hauptschleife nicht blockiert.
class InfluxWriter
{
  public:
    void setup(const std::string &host, uint16_t port, const std::string &db, const std::string &rp, const std::string &user,
               const std::string &password, uint32_t intervalMs);
    bool enabled() const { return _enabled; }
    // true, wenn das Intervall abgelaufen und keine Anfrage mehr unterwegs ist
    bool due();
    void submit(std::string &&body);

    uint32_t okCount() const { return _ok; }
    uint32_t errorCount() const { return _errors; }
    int lastCode() const { return _lastCode; }
    uint32_t lastOkAge() const { return _lastOk ? (millis() - _lastOk) / 1000 : 0; }
    const std::string &url() const { return _url; }
    std::string lastError();
    uint8_t failStreak() const { return _failStreak; }
    uint32_t waitSeconds() const { return _waitMs / 1000; }

  private:
    static void taskEntry(void *arg);
    void run();

    bool _enabled = false;
    std::string _url;
    std::string _user;
    std::string _password;
    uint32_t _intervalMs = 60000;
    uint32_t _timer = 0;
    uint32_t _waitMs = 60000;          // aktuelle Wartezeit (Intervall, bei Fehlern verlaengert)
    volatile uint8_t _failStreak = 0;  // Fehler in Folge

    std::mutex _lock;
    std::string _body;
    volatile bool _busy = false;
    TaskHandle_t _task = nullptr;

    volatile uint32_t _ok = 0;
    volatile uint32_t _errors = 0;
    volatile int _lastCode = 0;
    volatile uint32_t _lastOk = 0;
    std::string _lastError; // unter _lock, wird nur aus der Hauptschleife geloggt
};
