#pragma once
#include <cmath>
#include <string>

// Behaglichkeitsfeld nach Frank (1975): Raumtemperatur gegen relative Feuchte.
// Felder und Grenzen wie im EDOMI-Baustein "Behaglichkeitsfeld nach Frank" (0.6).
namespace Comfort
{
    enum Reason : uint8_t
    {
        Cold = 1,
        Warm = 2,
        Dry = 4,
        Humid = 8
    };

    struct Result
    {
        uint8_t level;   // 0 behaglich, 1 noch behaglich, 2 unbehaglich
        uint8_t reasons; // Reason-Bits
        float deviation; // Abstand zum Optimum (1 = 1 K bzw. 5 % rF)
        uint8_t code() const { return level * 16 + reasons; }
    };

    struct Point
    {
        float t, rh;
    };

    static constexpr Point OPTIMAL[] = {{17.6f, 73.3f}, {22.0f, 67.0f}, {24.2f, 33.3f}, {18.9f, 39.4f}};
    static constexpr Point SUBOPTIMAL[] = {{19.3f, 20.0f}, {25.3f, 17.0f}, {26.7f, 31.8f}, {24.5f, 60.0f},
                                           {20.5f, 81.8f}, {17.0f, 87.3f}, {16.0f, 76.4f}, {17.1f, 38.8f}};
    static constexpr Point PEAK = {21.0f, 55.0f};

    // Punkt im Polygon (Strahlverfahren); Punkte auf dem Rand zaehlen als innen
    template <size_t N>
    inline bool inside(const Point (&poly)[N], float t, float rh)
    {
        bool in = false;
        for (size_t i = 0, j = N - 1; i < N; j = i++)
        {
            const Point &a = poly[i], &b = poly[j];
            const float cross = (b.t - a.t) * (rh - a.rh) - (b.rh - a.rh) * (t - a.t);
            if (fabsf(cross) < 1e-4f && t >= fminf(a.t, b.t) && t <= fmaxf(a.t, b.t) && rh >= fminf(a.rh, b.rh) &&
                rh <= fmaxf(a.rh, b.rh))
                return true;
            if ((a.rh > rh) != (b.rh > rh) && t < (b.t - a.t) * (rh - a.rh) / (b.rh - a.rh) + a.t)
                in = !in;
        }
        return in;
    }

    inline Result evaluate(float t, float rh)
    {
        Result r;
        r.level = inside(OPTIMAL, t, rh) ? 0 : inside(SUBOPTIMAL, t, rh) ? 1 : 2;
        r.reasons = 0;
        if (rh < 30.0f)
            r.reasons |= Dry;
        if (rh > 65.0f)
            r.reasons |= Humid;
        if (r.level > 0)
        {
            if (t < 18.9f)
                r.reasons |= Cold;
            if (t > 22.0f)
                r.reasons |= Warm;
        }
        const float dt = PEAK.t - t, dh = (PEAK.rh - rh) / 5.0f;
        r.deviation = sqrtf(dt * dt + dh * dh);
        return r;
    }

    // Kurztext fuer das KO (max. 14 Zeichen, ASCII): bei Abweichung der Grund
    inline std::string text(uint8_t code)
    {
        const uint8_t level = code / 16, reasons = code % 16;
        if (level == 0)
            return "behaglich";
        const char *temp = (reasons & Cold) ? "kalt" : (reasons & Warm) ? "warm" : nullptr;
        const char *hum = (reasons & Dry) ? "trocken" : (reasons & Humid) ? "feucht" : nullptr;
        if (temp && hum)
            return std::string(temp) + ", " + hum;
        if (temp || hum)
            return std::string("zu ") + (temp ? temp : hum);
        return level == 1 ? "noch behaglich" : "unbehaglich";
    }

    // Langtext fuer Statusseite und MQTT, z. B. "noch behaglich (zu kalt, zu trocken)"
    inline std::string longText(uint8_t code)
    {
        const uint8_t level = code / 16, reasons = code % 16;
        std::string s = level == 0 ? "behaglich" : level == 1 ? "noch behaglich" : "unbehaglich";
        std::string why;
        for (auto [bit, name] : {std::pair<uint8_t, const char *>{Cold, "zu kalt"}, {Warm, "zu warm"}, {Dry, "zu trocken"}, {Humid, "zu feucht"}})
        {
            if (!(reasons & bit))
                continue;
            if (!why.empty())
                why += ", ";
            why += name;
        }
        if (!why.empty())
            s += " (" + why + ")";
        return s;
    }
} // namespace Comfort
