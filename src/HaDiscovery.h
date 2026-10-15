#pragma once
#include "MeasuredValue.h"

// Home Assistant MQTT Discovery: meldet alle aktiven Messwerte einzeln (retained) an.
// Die Werte selbst stehen im JSON-Objekt <Geraetepraefix>state.
class HaDiscovery
{
  public:
    void setup(bool enabled) { _enabled = enabled; }
    // values: Messwerte in Tabellenreihenfolge
    void loop(const MeasuredValue *values, uint8_t count);
    void restart() { _index = 0; }

  private:
    bool publish(const MeasuredValue &v);

    bool _enabled = false;
    bool _wasConnected = false;
    uint8_t _index = 255; // 255 = fertig
    uint32_t _timer = 0;
};
