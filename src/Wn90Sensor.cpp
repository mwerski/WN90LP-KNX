#include "Wn90Sensor.h"
#include "OpenKNX.h"
#include <NonBlockingModbusMaster.h>

static NonBlockingModbusMaster modbus;
static constexpr uint8_t WN90_SLAVE = 0x90;
static constexpr uint16_t WN90_FIRST_REG = 0x0165;

static void rs485Transmit()
{
    digitalWrite(WX_RS485_DE_PIN, HIGH);
}

static void rs485Receive()
{
    // Die Bibliothek ruft das direkt nach write() auf: erst warten, bis alles gesendet ist
    WX_RS485_SERIAL.flush();
    digitalWrite(WX_RS485_DE_PIN, LOW);
}

void Wn90Sensor::setup()
{
    pinMode(WX_RS485_DE_PIN, OUTPUT);
    digitalWrite(WX_RS485_DE_PIN, LOW);
    // Pegel invertiert wie in der bisherigen Firmware (T-RSS3)
    WX_RS485_SERIAL.begin(9600, SERIAL_8N1, WX_RS485_RX_PIN, WX_RS485_TX_PIN, true);
    modbus.initialize(WX_RS485_SERIAL, 0, 0, 500);
    modbus.preTransmission(rs485Transmit);
    modbus.postTransmission(rs485Receive);
}

bool Wn90Sensor::loop()
{
    if (!_busy)
    {
        if (!delayCheck(_lastPoll, POLL_INTERVAL))
            return false;
        _lastPoll = millis();
        _busy = modbus.readHoldingRegisters(WN90_SLAVE, WN90_FIRST_REG, REG_COUNT);
        return false;
    }

    if (!modbus.justFinished())
        return false;
    _busy = false;

    _lastError = modbus.getError();
    if (_lastError != 0 || modbus.getResponseBufferLength() < REG_COUNT)
    {
        _errors++;
        if (_consecutiveErrors < 255)
            _consecutiveErrors++;
        if (faulty())
            _ok = false;
        return false;
    }

    for (uint8_t i = 0; i < REG_COUNT; i++)
        _raw[i] = modbus.getResponseBuffer(i);
    _ok = true;
    _reads++;
    _consecutiveErrors = 0;
    return true;
}
