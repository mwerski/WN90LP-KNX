#include "Astro.h"

namespace Astro
{
    static inline double rad(double d) { return d * M_PI / 180.0; }
    static inline double deg(double r) { return r * 180.0 / M_PI; }
    static inline double norm360(double d)
    {
        d = fmod(d, 360.0);
        return d < 0 ? d + 360.0 : d;
    }
    static inline double julianDay(time_t utc) { return utc / 86400.0 + 2440587.5; }

    // ------------------------------------------------------------------------------------------
    // Sonne (NOAA Solar Calculator)
    // ------------------------------------------------------------------------------------------
    struct SunCoords
    {
        double declination; // Grad
        double eqTime;      // Zeitgleichung in Minuten
        double longitude;   // scheinbare ekliptikale Laenge in Grad
    };

    static SunCoords sunCoords(double jd)
    {
        const double t = (jd - 2451545.0) / 36525.0;
        const double l0 = norm360(280.46646 + t * (36000.76983 + t * 0.0003032));
        const double m = 357.52911 + t * (35999.05029 - 0.0001537 * t);
        const double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
        const double c = sin(rad(m)) * (1.914602 - t * (0.004817 + 0.000014 * t)) + sin(rad(2 * m)) * (0.019993 - 0.000101 * t) +
                         sin(rad(3 * m)) * 0.000289;
        const double omega = 125.04 - 1934.136 * t;
        const double appLong = l0 + c - 0.00569 - 0.00478 * sin(rad(omega));
        const double eps0 = 23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0;
        const double eps = eps0 + 0.00256 * cos(rad(omega));
        const double y = pow(tan(rad(eps / 2)), 2);

        SunCoords sc;
        sc.declination = deg(asin(sin(rad(eps)) * sin(rad(appLong))));
        sc.eqTime = 4.0 * deg(y * sin(2 * rad(l0)) - 2 * e * sin(rad(m)) + 4 * e * y * sin(rad(m)) * cos(2 * rad(l0)) -
                              0.5 * y * y * sin(4 * rad(l0)) - 1.25 * e * e * sin(2 * rad(m)));
        sc.longitude = norm360(appLong);
        return sc;
    }

    static Position horizontal(double hourAngleDeg, double decDeg, double latDeg)
    {
        const double h = rad(hourAngleDeg), d = rad(decDeg), l = rad(latDeg);
        const double sinEl = sin(l) * sin(d) + cos(l) * cos(d) * cos(h);
        const double el = asin(constrain(sinEl, -1.0, 1.0));
        double az = deg(atan2(sin(h), cos(h) * sin(l) - tan(d) * cos(l))) + 180.0;
        return {(float)norm360(az), (float)deg(el)};
    }

    Position sunPosition(time_t utc, float lat, float lon)
    {
        const SunCoords sc = sunCoords(julianDay(utc));
        const double minutes = fmod((double)utc, 86400.0) / 60.0;
        double ha = fmod(minutes + sc.eqTime + 4.0 * lon, 1440.0) / 4.0 - 180.0;
        if (ha < -180)
            ha += 360;
        return horizontal(ha, sc.declination, lat);
    }

    // Zeitpunkt (Minuten UTC ab utcDay0) fuer Sonnenhoehe -(zenith-90) vor (rising) bzw. nach Mittag; NAN = keiner
    static double sunEvent(double jdDay0, float lat, float lon, double zenith, bool rising)
    {
        double minutes = 720.0 - 4.0 * lon; // Startwert: mittlerer Mittag
        for (uint8_t i = 0; i < 3; i++)
        {
            const SunCoords sc = sunCoords(jdDay0 + minutes / 1440.0);
            const double arg = cos(rad(zenith)) / (cos(rad(lat)) * cos(rad(sc.declination))) - tan(rad(lat)) * tan(rad(sc.declination));
            if (arg < -1.0 || arg > 1.0)
                return NAN;
            const double ha = deg(acos(arg));
            const double noon = 720.0 - 4.0 * lon - sc.eqTime;
            minutes = rising ? noon - 4.0 * ha : noon + 4.0 * ha;
        }
        return minutes;
    }

    SunDay sunDay(time_t localMidnightUtc, int32_t tzOffset, float lat, float lon)
    {
        SunDay d;
        // UTC-Tagesbeginn, auf den sich die NOAA-Minuten beziehen
        const time_t utcDay0 = (localMidnightUtc + tzOffset) / 86400 * 86400;
        const double jd0 = julianDay(utcDay0);
        auto local = [&](double utcMinutes) -> float {
            if (isnan(utcMinutes))
                return NAN;
            double s = utcDay0 + utcMinutes * 60.0 - (double)localMidnightUtc;
            s = fmod(s, 86400.0);
            return (float)(s < 0 ? s + 86400.0 : s);
        };

        // Transit
        double noon = 720.0 - 4.0 * lon;
        for (uint8_t i = 0; i < 2; i++)
            noon = 720.0 - 4.0 * lon - sunCoords(jd0 + noon / 1440.0).eqTime;
        d.transit = local(noon);
        const SunCoords scNoon = sunCoords(jd0 + noon / 1440.0);
        d.transitElevation = 90.0f - (float)fabs(lat - scNoon.declination);

        const double riseMin = sunEvent(jd0, lat, lon, 90.833, true);
        const double setMin = sunEvent(jd0, lat, lon, 90.833, false);
        d.rise = local(riseMin);
        d.set = local(setMin);
        d.civilDawn = local(sunEvent(jd0, lat, lon, 96.0, true));
        d.civilDusk = local(sunEvent(jd0, lat, lon, 96.0, false));
        d.nauticalDawn = local(sunEvent(jd0, lat, lon, 102.0, true));
        d.nauticalDusk = local(sunEvent(jd0, lat, lon, 102.0, false));
        d.astroDawn = local(sunEvent(jd0, lat, lon, 108.0, true));
        d.astroDusk = local(sunEvent(jd0, lat, lon, 108.0, false));

        d.riseAzimuth = isnan(riseMin) ? NAN : sunPosition(utcDay0 + (time_t)(riseMin * 60), lat, lon).azimuth;
        d.setAzimuth = isnan(setMin) ? NAN : sunPosition(utcDay0 + (time_t)(setMin * 60), lat, lon).azimuth;
        if (!isnan(riseMin) && !isnan(setMin))
            d.dayLength = (float)((setMin - riseMin) * 60.0);
        else
            d.dayLength = NAN;
        return d;
    }

    // ------------------------------------------------------------------------------------------
    // Mond (Meeus, gekuerzte Reihen)
    // ------------------------------------------------------------------------------------------
    struct MoonCoords
    {
        double longitude; // ekliptikale Laenge, Grad
        double latitude;  // ekliptikale Breite, Grad
        double ra;        // Rektaszension, Grad
        double dec;       // Deklination, Grad
    };

    static MoonCoords moonCoords(double jd)
    {
        const double d = jd - 2451545.0;
        const double l = 218.316 + 13.176396 * d; // mittlere Laenge
        const double m = 134.963 + 13.064993 * d; // mittlere Anomalie Mond
        const double ms = 357.529 + 0.98560028 * d; // mittlere Anomalie Sonne
        const double f = 93.272 + 13.229350 * d;  // Argument der Breite
        const double dd = 297.850 + 12.190749 * d; // mittlere Elongation

        MoonCoords mc;
        mc.longitude = norm360(l + 6.289 * sin(rad(m)) + 1.274 * sin(rad(2 * dd - m)) + 0.658 * sin(rad(2 * dd)) +
                               0.214 * sin(rad(2 * m)) - 0.186 * sin(rad(ms)) - 0.114 * sin(rad(2 * f)));
        mc.latitude = 5.128 * sin(rad(f)) + 0.281 * sin(rad(m + f)) + 0.278 * sin(rad(m - f)) + 0.173 * sin(rad(2 * dd - f));
        const double eps = rad(23.439 - 0.0000004 * d);
        const double lam = rad(mc.longitude), bet = rad(mc.latitude);
        mc.ra = norm360(deg(atan2(sin(lam) * cos(eps) - tan(bet) * sin(eps), cos(lam))));
        mc.dec = deg(asin(sin(bet) * cos(eps) + cos(bet) * sin(eps) * sin(lam)));
        return mc;
    }

    static Position moonPositionJd(double jd, float lat, float lon)
    {
        const MoonCoords mc = moonCoords(jd);
        const double gmst = norm360(280.46061837 + 360.98564736629 * (jd - 2451545.0));
        return horizontal(norm360(gmst + lon - mc.ra), mc.dec, lat);
    }

    // Elongation (Mondlaenge - Sonnenlaenge), 0..360 Grad
    static double elongation(double jd)
    {
        return norm360(moonCoords(jd).longitude - sunCoords(jd).longitude);
    }

    MoonState moon(time_t utc, float lat, float lon)
    {
        const double jd = julianDay(utc);
        const MoonCoords mc = moonCoords(jd);
        const double elong = norm360(mc.longitude - sunCoords(jd).longitude);
        const double cosE = cos(rad(elong)) * cos(rad(mc.latitude));

        MoonState s;
        s.illumination = (float)((1.0 - cosE) / 2.0);
        s.age = (float)(elong / 360.0 * 29.530588853);
        s.phase = (uint8_t)(floor((elong + 22.5) / 45.0)) % 8;
        s.position = moonPositionJd(jd, lat, lon);
        return s;
    }

    void moonRiseSet(time_t localMidnightUtc, float lat, float lon, float &rise, float &set)
    {
        // Geozentrische Hoehe des Mondmittelpunkts bei Auf-/Untergang (Parallaxe, Refraktion, Radius)
        static constexpr double H0 = 0.125;
        static constexpr uint16_t STEP = 600; // 10 min
        rise = set = NAN;
        double prev = moonPositionJd(julianDay(localMidnightUtc), lat, lon).elevation - H0;
        for (uint32_t t = STEP; t <= 86400; t += STEP)
        {
            const double cur = moonPositionJd(julianDay(localMidnightUtc + t), lat, lon).elevation - H0;
            if ((prev < 0) != (cur < 0))
            {
                const float at = (float)(t - STEP + STEP * prev / (prev - cur));
                if (cur > 0 && isnan(rise))
                    rise = at;
                else if (cur <= 0 && isnan(set))
                    set = at;
            }
            prev = cur;
        }
    }

    time_t nextMoonPhase(time_t fromUtc, float targetDeg)
    {
        static constexpr double SYNODIC = 29.530588853;
        double jd = julianDay(fromUtc);
        double ahead = norm360(targetDeg - elongation(jd));
        if (ahead < 1.0)
            ahead += 360.0; // gerade erreicht: den naechsten nehmen
        jd += ahead / 360.0 * SYNODIC;
        // Verfeinern (der Mond laeuft ungleichmaessig, 11..15 Grad/Tag relativ zur Sonne)
        for (uint8_t i = 0; i < 4; i++)
        {
            double diff = norm360(targetDeg - elongation(jd) + 180.0) - 180.0;
            jd += diff / 12.19;
        }
        return (time_t)((jd - 2440587.5) * 86400.0);
    }

    const char *moonPhaseText(uint8_t phase, bool utf8)
    {
        // max. 14 Zeichen (DPT 16); ASCII, daher fuer beide Kodierungen gleich
        static const char *const TEXT[8] = {"Neumond", "Zun. Sichel", "Erstes Viertel", "Zun. Mond",
                                            "Vollmond", "Abn. Mond", "Letzt. Viertel", "Abn. Sichel"};
        (void)utf8;
        return phase < 8 ? TEXT[phase] : "";
    }

    const char *moonPhaseLongText(uint8_t phase)
    {
        static const char *const TEXT[8] = {"Neumond", "Zunehmende Sichel", "Erstes Viertel", "Zunehmender Mond",
                                            "Vollmond", "Abnehmender Mond", "Letztes Viertel", "Abnehmende Sichel"};
        return phase < 8 ? TEXT[phase] : "";
    }
} // namespace Astro
