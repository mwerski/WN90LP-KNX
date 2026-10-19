#pragma once
#include "OpenKNX.h"
#include "ValueTable.h"
#include <string>

// Ein Messwert mit KO: zyklisch senden, senden bei absoluter/relativer Aenderung,
// Leseanfragen beantwortet der knx-Stack aus dem KO-Wert (wird ohne Senden aktualisiert).
class MeasuredValue
{
  public:
    void init(const WxValueDef *def, bool configured);
    void set(float value);
    void invalidate();
    // Sendet hoechstens ein Telegramm; true, wenn gesendet wurde
    bool loop();

    bool active() const { return _active; }
    bool valid() const { return _valid; }
    float value() const { return _value; }
    bool sent() const { return _sent; }
    float lastSent() const { return _lastSent; }
    const WxValueDef *def() const { return _def; }

    // Darstellung fuer Konsole/MQTT: Uhrzeit HH:MM:SS, Datum JJJJ-MM-TT, Dauer, Texte, sonst Zahl
    // full = Volltext statt Kurztext (14 Zeichen, DPT 16) fuer Prognose, Mond und Warnungen
    static std::string format(float value, const WxValueDef *def, bool withUnit, bool full = false);
    static void civilFromDays(long daysSince1970, int &y, int &m, int &d);
    static long daysFromCivil(int y, int m, int d);

    // Bezug fuer Zeitpunkte: Uhrzeiten (Sekunden seit Mitternacht) gelten fuer den aktuellen Tag
    static void setDayContext(int64_t localMidnightUtc, int32_t tzOffset);
    // Unixzeit eines Uhrzeit- oder Datumswerts; false ohne Tagesbezug oder bei anderen Formaten
    static bool toUnix(float value, const WxValueDef *def, int64_t &unix);

  private:
    static int64_t s_localMidnightUtc;
    static int32_t s_tzOffset;
    static bool s_dayContext;

    bool changeExceedsThreshold() const;
    void send();
    KNXValue busValue() const;
    Dpt dpt() const;

    const WxValueDef *_def = nullptr;
    bool _active = false;
    uint8_t _dptIndex = 0;
    uint32_t _cycleMs = 0;
    float _absChange = 0;
    float _relChange = 0;

    bool _valid = false;
    float _value = 0;
    bool _sent = false;
    float _lastSent = 0;
    bool _sendPending = false;
    uint32_t _lastSendTime = 0;
};
