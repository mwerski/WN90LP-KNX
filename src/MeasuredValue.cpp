#include "MeasuredValue.h"

void MeasuredValue::init(const WxValueDef *def, bool configured)
{
    _def = def;
    if (!configured)
    {
        _active = false;
        return;
    }

    // Layout siehe scripts/generate_ets.py: +0 Aktiv/DPT, +1 Zyklus, +3 abs., +7 rel.
    const uint16_t base = def->paramOffset;
    const uint8_t flags = knx.paramByte(base);
    _active = flags & 0x01;
    _dptIndex = def->dptCount > 1 ? (flags >> 1) & 0x03 : 0;
    if (_dptIndex >= def->dptCount)
        _dptIndex = 0;
    _cycleMs = (uint32_t)knx.paramWord(base + 1) * 1000;
    if (def->thresholds)
    {
        _absChange = (float)knx.paramFloat(base + 3, Float_Enc_IEEE754Single);
        _relChange = (float)knx.paramFloat(base + 7, Float_Enc_IEEE754Single);
    }
}

void MeasuredValue::set(float value)
{
    if (isnan(value))
    {
        invalidate();
        return;
    }
    _value = value;
    _valid = true;
    if (!_active)
        return;

    // Leseanfragen sollen immer den aktuellen Wert liefern
    knx.getGroupObject(_def->koNumber).valueNoSend(busValue(), dpt());

    if (!_sent || changeExceedsThreshold())
        _sendPending = true;
}

void MeasuredValue::invalidate()
{
    _valid = false;
    _sendPending = false;
}

bool MeasuredValue::loop()
{
    if (!_active || !_valid)
        return false;

    if (_sendPending || (_cycleMs > 0 && delayCheck(_lastSendTime, _cycleMs)))
    {
        send();
        return true;
    }
    return false;
}

bool MeasuredValue::changeExceedsThreshold() const
{
    const float diff = fabsf(_value - _lastSent);
    if (!_def->thresholds)
        return diff > 0.0001f;
    if (_absChange > 0 && diff >= _absChange)
        return true;
    if (_relChange > 0)
    {
        // Von 0 aus ist jede Aenderung relativ unendlich gross (z. B. Helligkeit am Morgen)
        if (fabsf(_lastSent) < 0.0001f)
            return diff > 0.0001f;
        if (diff / fabsf(_lastSent) * 100.0f >= _relChange)
            return true;
    }
    return false;
}

void MeasuredValue::send()
{
    knx.getGroupObject(_def->koNumber).value(busValue(), dpt());
    _lastSent = _value;
    _sent = true;
    _sendPending = false;
    _lastSendTime = millis();
}

KNXValue MeasuredValue::busValue() const
{
    const WxDptOption &opt = _def->dpts[_dptIndex];
    const float bus = _value * opt.factor;
    switch (opt.main)
    {
        case 1:
            return KNXValue(bus >= 0.5f);
        case 5:
            if (opt.sub == 10)
                return KNXValue((uint8_t)constrain(lroundf(bus), 0L, 255L));
            return KNXValue((double)bus); // 5.001 / 5.003 skaliert der Stack selbst
        default:
            return KNXValue((double)bus);
    }
}

Dpt MeasuredValue::dpt() const
{
    const WxDptOption &opt = _def->dpts[_dptIndex];
    return Dpt(opt.main, opt.sub);
}
