#include "Irrigation.h"
#include "OpenKNX.h"

void Irrigation::setup(float kc, float threshold, float storage, float amount)
{
    _kc = kc;
    _threshold = max(threshold, 0.5f);
    _storage = max(storage, 0.5f);
    _amount = amount;
}

// FAO-56, Gleichung 53 (stuendlich): ET0 in mm/h
float Irrigation::et0Hourly(float t, float rh, float wind, float rs, float rso, float p)
{
    const float es = 0.6108f * expf(17.27f * t / (t + 237.3f)); // Saettigungsdampfdruck kPa
    const float ea = es * rh / 100.0f;                          // aktueller Dampfdruck kPa
    const float delta = 4098.0f * es / powf(t + 237.3f, 2);     // Steigung der Dampfdruckkurve
    const float gamma = 0.000665f * p / 10.0f;                  // Psychrometerkonstante (p in kPa)

    const float rsMj = rs * 0.0036f; // W/m2 -> MJ/m2/h
    const float rns = 0.77f * rsMj;  // Albedo Gras 0,23
    // Langwellige Nettoabstrahlung; Bewoelkung aus Rs/Rso, nachts typischer Wert
    const float ratio = rso > 10.0f ? constrain(rs / rso, 0.3f, 1.0f) : 0.7f;
    const float tk = t + 273.16f;
    const float rnl = 2.043e-10f * tk * tk * tk * tk * (0.34f - 0.14f * sqrtf(max(ea, 0.0f))) * (1.35f * ratio - 0.35f);
    const float rn = rns - rnl;
    const float g = rn > 0 ? 0.1f * rn : 0.5f * rn; // Bodenwaermestrom Tag/Nacht
    const bool day = rso > 10.0f;
    const float cd = day ? 0.24f : 0.96f; // ASCE-Nennerkoeffizient Tag/Nacht

    const float et = (0.408f * delta * (rn - g) + gamma * (37.0f / tk) * wind * (es - ea)) / (delta + gamma * (1.0f + cd * wind));
    return max(et, 0.0f);
}

void Irrigation::update(float dtHours, float t, float rh, float wind, float rs, float rso, float p, double rainTotal)
{
    // Regen seit dem letzten Aufruf (der Gesamtzaehler steigt nur)
    if (!isnan(rainTotal))
    {
        if (!isnan(_lastRain) && rainTotal > _lastRain)
            _balance += (float)(rainTotal - _lastRain);
        _lastRain = rainTotal;
    }

    if (dtHours > 0 && dtHours < 0.1f && !isnan(t) && !isnan(rh) && !isnan(rs))
    {
        _lastRate = et0Hourly(t, rh, isnan(wind) ? 2.0f : wind, rs, rso, isnan(p) ? 1013.0f : p);
        const float et = _lastRate * dtHours;
        _et0Today += et;
        _balance -= _kc * et;
    }
    clampAndEvaluate();
}

void Irrigation::credit()
{
    _balance += _amount;
    clampAndEvaluate();
    logInfo("Weather", "Bewaesserung gemeldet: +%.1f mm, Bilanz %.1f mm", _amount, _balance);
}

void Irrigation::clampAndEvaluate()
{
    // Ueberschuss ueber dem Bodenspeicher versickert; Defizit auf 200 mm begrenzt
    _balance = constrain(_balance, -200.0f, _storage);
    if (_balance <= -_threshold)
        _irrigate = true;
    else if (_balance > -_threshold / 2)
        _irrigate = false;
}

void Irrigation::dayRollover(bool consecutive)
{
    _et0Yesterday = consecutive ? _et0Today : NAN;
    _et0Today = 0;
}

void Irrigation::writeFlash()
{
    openknx.flash.writeFloat(_balance);
    openknx.flash.writeFloat(_et0Today);
    openknx.flash.writeFloat(_et0Yesterday);
    openknx.flash.writeFloat(0); // Reserve
}

void Irrigation::readFlash()
{
    _balance = openknx.flash.readFloat();
    _et0Today = openknx.flash.readFloat();
    _et0Yesterday = openknx.flash.readFloat();
    openknx.flash.readFloat();
    if (isnan(_balance))
        _balance = 0;
    if (isnan(_et0Today))
        _et0Today = 0;
    clampAndEvaluate();
}
