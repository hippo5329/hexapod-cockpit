// Robot settings over Wi-Fi (lib/hexa_settings has the format and the rules).
//
// tools/push_settings.py uploads a settings blob with ArduinoOTA's SPIFFS command (the same
// password and MD5 check as a firmware update) into the `spiffs` partition, which nothing
// else uses; the board reboots. At boot, before anything reads NVS, apply_staged() takes it:
// every line checked, then the keys written to the `hexa` namespace and the blob erased, so
// it applies once. A refused blob is erased too; the serial port says why.
//
// A change of link, ssid or psk is on trial: the old values are kept in NVS, and if the
// board has not joined the network 60 s after boot, trial_loop() puts them back and reboots.
#pragma once

#include <Preferences.h>

namespace hexa_settings {

void apply_staged(Preferences& prefs);
// Every loop, with the network's state.
void trial_loop(Preferences& prefs, bool network_up);
// "none", "applied N keys", "refused: <why>", for info.
const char* last_result();

}  // namespace hexa_settings
