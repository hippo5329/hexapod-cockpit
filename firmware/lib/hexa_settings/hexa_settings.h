// Robot settings sent over Wi-Fi: a text blob that tools/push_settings.py uploads through
// ArduinoOTA (password and MD5 checked) into the unused `spiffs` partition. At the next boot
// the firmware parses it, writes the keys to NVS and erases it (src/settings.cpp).
//
//   hexa-settings 1
//   KEY=VALUE        one per line, # comments; the blob ends at a NUL (or erased flash)
//
// Only the keys given change; an empty value removes a key (only those with a default:
// app, sim, port, trims, imu_level, idle_s, low_bat_mv, syslog_ip, syslog_port). Every line must pass or nothing is taken. The
// rules are make_nvs.py's, plus what would strand a robot reached only over Wi-Fi: link
// may only be wifi, and ssid, psk, agent and ota_pw cannot be removed.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace hexa {

constexpr char kSettingsMagic[] = "hexa-settings 1\n";
constexpr size_t kSettingsMax = 4096;  // the first flash sector of the partition

struct Setting {
    std::string key, value;  // value empty: remove the key
};

// The blob starts with the magic line: it is a settings upload, not leftover flash.
bool settings_has_magic(const char* blob, size_t len);

// The settings in the blob, all checked; false (out untouched) with why, which names a line
// or a key but never a value.
bool parse_settings(const char* blob, size_t len, std::vector<Setting>* out, std::string* why);

}  // namespace hexa
