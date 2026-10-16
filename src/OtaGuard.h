#pragma once

// Sicherheitsnetz fuer OTA-Updates (Bootloader mit App-Rollback): Eine neu eingespielte Firmware
// wird erst bestaetigt, wenn sie stabil im Netz laeuft. Absturz oder Watchdog vorher, oder kein
// Netz nach 10 min, fuehren zurueck zur vorherigen Firmware. Ein gewollter Neustart (ETS, Konsole)
// bestaetigt die Firmware vorher, sonst wuerde z. B. die ETS-Programmierung ein Rollback ausloesen.
namespace OtaGuard
{
    void setup();
    void loop(bool networkEstablished);
    void confirm(const char *reason);
    bool pending();
} // namespace OtaGuard
