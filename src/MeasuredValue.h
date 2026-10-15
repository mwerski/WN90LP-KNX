#pragma once
#include "OpenKNX.h"
#include "ValueTable.h"

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

  private:
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
