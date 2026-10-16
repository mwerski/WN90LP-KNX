#pragma once
#include "Diag.h"
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <new>
#include <string>

// Serialisiert in einem Stueck (Groesse vorab gemessen, kein schrittweises Vergroessern) und nur,
// wenn ein ausreichend grosser zusammenhaengender Speicherblock frei ist. Bei Speichermangel
// false statt Abbruch durch std::bad_alloc; der Fall wird fuer "wx perf" gezaehlt.
inline bool serializeJsonSafe(const JsonDocument &doc, std::string &out)
{
    if (doc.overflowed())
    {
        Diag::noMemory();
        return false;
    }
    const size_t n = measureJson(doc);
    if (heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) < n + 8192)
    {
        Diag::noMemory();
        return false;
    }
    try
    {
        out.reserve(n + 1);
    }
    catch (const std::bad_alloc &)
    {
        Diag::noMemory();
        return false;
    }
    serializeJson(doc, out);
    return true;
}
