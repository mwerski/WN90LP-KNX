#pragma once
#include <Arduino.h>

// Strahlungsmodelle fuer die Schaetzung aus der waagerechten Globalstrahlung
namespace Solar
{
    // Globalstrahlung bei klarem Himmel nach Haurwitz in W/m2
    inline float clearSky(float elevationDeg)
    {
        if (elevationDeg <= 0)
            return 0;
        const float s = sinf(elevationDeg * DEG_TO_RAD);
        return 1098.0f * s * expf(-0.057f / s);
    }

    struct Split
    {
        float direct;  // Direktnormalstrahlung DNI in W/m2
        float diffuse; // Diffusstrahlung auf die Horizontale DHI in W/m2
    };

    // Aufteilung in Direkt- und Diffusanteil nach Erbs (Klarheitsindex gegen extraterrestrische Strahlung)
    inline Split split(float ghi, float elevationDeg, uint16_t dayOfYear)
    {
        Split r = {0, ghi};
        if (elevationDeg <= 1.0f || ghi <= 0)
            return r;
        const float s = sinf(elevationDeg * DEG_TO_RAD);
        const float i0 = 1367.0f * (1.0f + 0.033f * cosf(TWO_PI * dayOfYear / 365.0f));
        const float kt = constrain(ghi / (i0 * s), 0.0f, 1.0f);
        float kd;
        if (kt <= 0.22f)
            kd = 1.0f - 0.09f * kt;
        else if (kt <= 0.8f)
            kd = 0.9511f - 0.1604f * kt + 4.388f * kt * kt - 16.638f * kt * kt * kt + 12.336f * kt * kt * kt * kt;
        else
            kd = 0.165f;
        r.diffuse = kd * ghi;
        r.direct = (ghi - r.diffuse) / s;
        return r;
    }

    // Kosinus des Einfallswinkels auf eine Flaeche (Neigung, Ausrichtung in Grad; 0 = Nord)
    inline float cosIncidence(float elevationDeg, float sunAzimuthDeg, float tiltDeg, float planeAzimuthDeg)
    {
        const float h = elevationDeg * DEG_TO_RAD;
        const float b = tiltDeg * DEG_TO_RAD;
        return sinf(h) * cosf(b) + cosf(h) * sinf(b) * cosf((sunAzimuthDeg - planeAzimuthDeg) * DEG_TO_RAD);
    }

    // Einstrahlung auf eine geneigte Flaeche: direkt + diffus (isotrop) + Bodenreflexion
    inline float plane(float ghi, const Split &sp, float elevationDeg, float sunAzimuthDeg, float tiltDeg, float planeAzimuthDeg,
                       float albedo)
    {
        const float b = tiltDeg * DEG_TO_RAD;
        float beam = 0;
        if (elevationDeg > 0)
            beam = sp.direct * max(cosIncidence(elevationDeg, sunAzimuthDeg, tiltDeg, planeAzimuthDeg), 0.0f);
        return beam + sp.diffuse * (1.0f + cosf(b)) / 2.0f + ghi * albedo * (1.0f - cosf(b)) / 2.0f;
    }

    // Seitliche Abweichung Sonne zu Fassadennormale in Grad (-180..180)
    inline float azimuthOffset(float sunAzimuthDeg, float facadeAzimuthDeg)
    {
        float d = fmodf(sunAzimuthDeg - facadeAzimuthDeg + 540.0f, 360.0f) - 180.0f;
        return d;
    }

    // Profilwinkel: Sonnenhoehe projiziert auf die Ebene senkrecht zur Fassade
    inline float profileAngle(float elevationDeg, float sunAzimuthDeg, float facadeAzimuthDeg)
    {
        const float c = cosf(azimuthOffset(sunAzimuthDeg, facadeAzimuthDeg) * DEG_TO_RAD);
        if (c <= 0.01f)
            return NAN;
        return atanf(tanf(elevationDeg * DEG_TO_RAD) / c) * RAD_TO_DEG;
    }

    inline uint16_t dayOfYear(uint16_t year, uint8_t month, uint8_t day)
    {
        static const uint16_t start[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
        const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
        return start[(month - 1) % 12] + day + (leap && month > 2 ? 1 : 0);
    }
} // namespace Solar
