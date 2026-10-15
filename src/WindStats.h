#pragma once
#include "OpenKNX.h"

// Gleitendes 10-Minuten-Mittel der Windgeschwindigkeit und staerkste Boee, in Minutenbloecken
class WindStats
{
  public:
    void feed(float wind, float gust)
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
    };
    static constexpr uint8_t BLOCKS = 9; // 9 volle Minuten + laufende Minute = 10 min

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
