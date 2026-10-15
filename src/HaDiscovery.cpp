#include "HaDiscovery.h"
#include "NetworkModule.h"
#include "WeatherModule.h"
#include <ArduinoJson.h>

#ifdef OPENKNX_MQTT

// Abstand zwischen zwei Discovery-Nachrichten (Sendewarteschlange von OFM-Network: 32 Eintraege)
static constexpr uint32_t SPACING = 100;

void HaDiscovery::loop(const MeasuredValue *values, uint8_t count)
{
    if (!_enabled)
        return;
    const bool connected = openknxNetwork.mqtt.connected();
    // Nach jedem (Neu-)Verbinden erneut melden, falls der Broker nicht persistent speichert
    if (connected && !_wasConnected)
        _index = 0;
    _wasConnected = connected;
    if (!connected || _index >= count || !delayCheck(_timer, SPACING))
        return;
    _timer = millis();

    // Inaktive Werte ueberspringen, aktive einzeln senden
    while (_index < count && !values[_index].active())
        _index++;
    if (_index < count && publish(values[_index]))
        _index++;
}

bool HaDiscovery::publish(const MeasuredValue &v)
{
    const WxValueDef *def = v.def();
    const bool binary = def->dpts[0].main == 1;

    // Knoten-Id aus dem Geraetepraefix "openknx/<praefix>/" -> "openknx_<praefix>"
    std::string prefix = openknxNetwork.mqtt.devicePrefix();
    std::string node = prefix;
    while (!node.empty() && node.back() == '/')
        node.pop_back();
    for (char &c : node)
        if (!isalnum((unsigned char)c) && c != '_' && c != '-')
            c = '_';

    JsonDocument doc;
    doc["name"] = def->name;
    doc["unique_id"] = node + "_" + def->mqttKey;
    doc["object_id"] = node + "_" + def->mqttKey;
    doc["state_topic"] = prefix + "state";
    std::string tpl;
    if (binary)
        tpl = std::string("{{ 'ON' if value_json.") + def->mqttKey + " == 1 else 'OFF' }}";
    else
        tpl = std::string("{{ value_json.") + def->mqttKey + " }}";
    doc["value_template"] = tpl;
    if (def->haClass)
        doc["device_class"] = def->haClass;
    if (!binary && def->haUnit)
    {
        doc["unit_of_measurement"] = def->haUnit;
        doc["state_class"] = strcmp(def->mqttKey, "raincounter") == 0 ? "total_increasing" : "measurement";
    }
    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"].to<JsonArray>().add(node);
    dev["name"] = "WN90LP Wetterstation";
    dev["manufacturer"] = "OpenKNX";
    dev["model"] = "WN90LP Gateway (T-RSS3)";
    dev["sw_version"] = WX_FIRMWARE_VERSION;

    std::string payload;
    serializeJson(doc, payload);
    const std::string topic = std::string("homeassistant/") + (binary ? "binary_sensor/" : "sensor/") + node + "/" + def->mqttKey + "/config";
    return openknxNetwork.mqtt.publish(topic, payload, 0, true);
}

#else

void HaDiscovery::loop(const MeasuredValue *, uint8_t) {}

#endif
