#pragma once
#include "OpenKNX.h"

// Gleitendes 10-Minuten-Mittel der Windgeschwindigkeit und der Windrichtung, staerkste Boee, in Minutenbloecken
class WindStats
{
  public:
    void feed(float wind, float gust, float dir)
    {
        if (!_started)
        {
            _started = true;
            _timer = millis();
        }
        rotate();
        if (!isnan(wind))
        {
            _cur.sum += wind;
            _cur.count++;
        }
        if (!isnan(gust) && (isnan(_cur.maxGust) || gust > _cur.maxGust))
            _cur.maxGust = gust;
        // Richtung als Einheitsvektor mitteln (sonst ergaeben 350 und 10 Grad im Mittel 180); bei Windstille bedeutungslos
        if (!isnan(dir) && !isnan(wind) && wind >= CALM)
        {
            const float rad = dir * (float)DEG_TO_RAD;
            _cur.sinSum += sinf(rad);
            _cur.cosSum += cosf(rad);
            _cur.dirCount++;
        }
    }

    float average() const
    {
        float sum = _cur.sum;
        uint32_t count = _cur.count;
        for (uint8_t i = 0; i < _filled; i++)
        {
            sum += _blocks[i].sum;
            count += _blocks[i].count;
        }
        return count ? sum / count : NAN;
    }

    // Mittlere Herkunftsrichtung 0..360 Grad; NAN bei Windstille oder umlaufendem Wind (Richtungen heben sich auf)
    float averageDir() const
    {
        float s = _cur.sinSum, c = _cur.cosSum;
        uint32_t count = _cur.dirCount;
        for (uint8_t i = 0; i < _filled; i++)
        {
            s += _blocks[i].sinSum;
            c += _blocks[i].cosSum;
            count += _blocks[i].dirCount;
        }
        if (!count || sqrtf(s * s + c * c) / count < MIN_STEADINESS)
            return NAN;
        const float deg = atan2f(s, c) * (float)RAD_TO_DEG;
        return deg < 0 ? deg + 360.0f : deg;
    }

    float maxGust() const
    {
        float m = _cur.maxGust;
        for (uint8_t i = 0; i < _filled; i++)
            if (!isnan(_blocks[i].maxGust) && (isnan(m) || _blocks[i].maxGust > m))
                m = _blocks[i].maxGust;
        return m;
    }

  private:
    struct Block
    {
        float sum = 0;
        uint32_t count = 0;
        float maxGust = NAN;
        float sinSum = 0, cosSum = 0;
        uint32_t dirCount = 0;
    };
    static constexpr uint8_t BLOCKS = 9;              // 9 volle Minuten + laufende Minute = 10 min
    static constexpr float CALM = 1.0f;               // m/s, darunter keine Richtung
    static constexpr float MIN_STEADINESS = 0.3f;     // Laenge des mittleren Einheitsvektors

    void rotate()
    {
        while (delayCheck(_timer, 60000UL))
        {
            _timer += 60000UL;
            _blocks[_head] = _cur;
            _head = (_head + 1) % BLOCKS;
            if (_filled < BLOCKS)
                _filled++;
            _cur = Block();
        }
    }

    Block _blocks[BLOCKS];
    Block _cur;
    uint8_t _head = 0;
    uint8_t _filled = 0;
    uint32_t _timer = 0;
    bool _started = false;
};
