#include "Sds011Sensor.h"
#include "OpenKNX.h"

void Sds011Sensor::setup(uint8_t dutyCycleMin)
{
    _dutyCycle = dutyCycleMin > 30 ? 30 : dutyCycleMin;
    WX_SDS_SERIAL.begin(9600, SERIAL_8N1, WX_SDS_RX_PIN, WX_SDS_TX_PIN);
    // Antworten (C5-Rahmen) werden vom Parser ignoriert, daher ohne Warten senden
    sendCommand(0x06, 0x01, 0x01);       // aufwecken
    sendCommand(0x02, 0x01, 0x00);       // aktiver Meldemodus
    sendCommand(0x08, 0x01, _dutyCycle); // Messintervall
    _start = millis();
}

void Sds011Sensor::sendCommand(uint8_t cmd, uint8_t data2, uint8_t data3)
{
    // 19 Byte: AA B4 Data1..Data15 Pruefsumme AB, Data14/15 = FF FF (alle Sensoren)
    uint8_t frame[19] = {0xAA, 0xB4, cmd, data2, data3};
    frame[15] = 0xFF;
    frame[16] = 0xFF;
    uint8_t sum = 0;
    for (uint8_t i = 2; i <= 16; i++)
        sum += frame[i];
    frame[17] = sum;
    frame[18] = 0xAB;
    WX_SDS_SERIAL.write(frame, sizeof(frame));
}

bool Sds011Sensor::loop()
{
    bool updated = false;
    while (WX_SDS_SERIAL.available())
    {
        const uint8_t b = WX_SDS_SERIAL.read();
        if (_pos == 0 && b != 0xAA)
            continue;
        if (_pos == 1 && b != 0xC0)
        {
            _pos = b == 0xAA ? 1 : 0;
            continue;
        }
        _buf[_pos++] = b;
        if (_pos < sizeof(_buf))
            continue;
        _pos = 0;

        uint8_t sum = 0;
        for (uint8_t i = 2; i <= 7; i++)
            sum += _buf[i];
        if (_buf[9] != 0xAB || sum != _buf[8])
        {
            _checksumErrors++;
            continue;
        }
        _pm25 = (_buf[2] | (_buf[3] << 8)) / 10.0f;
        _pm10 = (_buf[4] | (_buf[5] << 8)) / 10.0f;
        _frames++;
        _lastFrame = millis();
        _faulty = false;
        updated = true;
    }

    // Stoerung, wenn laenger als zwei Messintervalle (mind. 2 min) kein Messrahmen kam
    const uint32_t timeout = max<uint32_t>(_dutyCycle * 2 * 60000UL, 120000UL);
    const uint32_t since = _frames ? _lastFrame : _start;
    if (!_faulty && millis() - since > timeout)
    {
        _faulty = true;
        _pm25 = NAN;
        _pm10 = NAN;
    }
    return updated;
}
