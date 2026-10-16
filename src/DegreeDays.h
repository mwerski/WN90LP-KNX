#pragma once
#include "OpenKNX.h"

// Heizgradtage nach VDI 3807 (Gt20/15) und Kuehlgradtage, je Monat und Jahr (mit Vorperiode)
class DegreeDays
{
  public:
    static constexpr uint16_t FLASH_SIZE = 4 + 2 + 8 * 4;

    void setup(float indoor, float heatLimit, float coolBase)
    {
        _indoor = indoor;
        _heatLimit = heatLimit;
        _coolBase = coolBase;
    }

    // Periode beim ersten Erhalt der Uhrzeit festlegen (Werte ab dann gueltig, beginnend mit 0)
    void start(uint32_t dayKey)
    {
        if (_monthKey == 0)
        {
            _monthKey = dayKey / 100;
            _yearKey = dayKey / 10000;
        }
    }

    // Abgeschlossener Tag (JJJJMMTT) mit seiner Mitteltemperatur
    void addDay(uint32_t dayKey, float mean)
    {
        const uint32_t month = dayKey / 100;
        const uint16_t year = dayKey / 10000;
        if (_monthKey != 0 && month != _monthKey)
        {
            _hddLastMonth = _hddMonth;
            _cddLastMonth = _cddMonth;
            _hddMonth = _cddMonth = 0;
        }
        if (_yearKey != 0 && year != _yearKey)
        {
            _hddLastYear = _hddYear;
            _cddLastYear = _cddYear;
            _hddYear = _cddYear = 0;
        }
        _monthKey = month;
        _yearKey = year;
        if (isnan(mean))
            return;
        if (mean < _heatLimit)
        {
            _hddMonth += _indoor - mean;
            _hddYear += _indoor - mean;
        }
        if (mean > _coolBase)
        {
            _cddMonth += mean - _coolBase;
            _cddYear += mean - _coolBase;
        }
    }

    bool valid() const { return _monthKey != 0; }
    float hddMonth() const { return valid() ? _hddMonth : NAN; }
    float hddLastMonth() const { return _hddLastMonth; }
    float hddYear() const { return valid() ? _hddYear : NAN; }
    float hddLastYear() const { return _hddLastYear; }
    float cddMonth() const { return valid() ? _cddMonth : NAN; }
    float cddLastMonth() const { return _cddLastMonth; }
    float cddYear() const { return valid() ? _cddYear : NAN; }
    float cddLastYear() const { return _cddLastYear; }

    void writeFlash()
    {
        openknx.flash.writeInt(_monthKey);
        openknx.flash.writeWord(_yearKey);
        for (float v : {_hddMonth, _hddLastMonth, _hddYear, _hddLastYear, _cddMonth, _cddLastMonth, _cddYear, _cddLastYear})
            openknx.flash.writeFloat(v);
    }

    void readFlash()
    {
        _monthKey = openknx.flash.readInt();
        _yearKey = openknx.flash.readWord();
        float *fields[] = {&_hddMonth, &_hddLastMonth, &_hddYear, &_hddLastYear, &_cddMonth, &_cddLastMonth, &_cddYear, &_cddLastYear};
        for (float *f : fields)
            *f = openknx.flash.readFloat();
        if (_monthKey == 0xFFFFFFFF)
            _monthKey = 0;
        for (float *f : {&_hddMonth, &_hddYear, &_cddMonth, &_cddYear})
            if (isnan(*f))
                *f = 0;
    }

  private:
    float _indoor = 20, _heatLimit = 15, _coolBase = 18;
    uint32_t _monthKey = 0; // JJJJMM der laufenden Periode
    uint16_t _yearKey = 0;
    float _hddMonth = 0, _hddLastMonth = NAN, _hddYear = 0, _hddLastYear = NAN;
    float _cddMonth = 0, _cddLastMonth = NAN, _cddYear = 0, _cddLastYear = NAN;
};
