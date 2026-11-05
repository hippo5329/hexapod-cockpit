#include "settings.h"

#include <Arduino.h>
#include <IPAddress.h>
#include <esp_partition.h>

#include <string>
#include <vector>

#include "hexa_settings.h"

namespace hexa_settings {
namespace {

constexpr uint32_t kTrialMs = 60000;
// The values a Wi-Fi change replaced; "prev_<key>" absent = the key was absent.
const char* const kTrialKeys[] = {"link", "ssid", "psk"};
String result = "none";

void put(Preferences& prefs, const hexa::Setting& s) {
    if (s.value.empty()) prefs.remove(s.key.c_str());
    else if (s.key == "sim") prefs.putBool("sim", s.value == "1");  // a bool, as make_nvs.py writes it
    else if (s.key == "port") prefs.putUShort("port", static_cast<uint16_t>(atoi(s.value.c_str())));
    else prefs.putString(s.key.c_str(), s.value.c_str());
}

bool is_trial_key(const std::string& k) {
    for (const char* t : kTrialKeys)
        if (k == t) return true;
    return false;
}

}  // namespace

void apply_staged(Preferences& prefs) {
    const esp_partition_t* p =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, nullptr);
    if (!p) return;
    static char buf[hexa::kSettingsMax];
    if (esp_partition_read(p, 0, buf, sizeof buf) != ESP_OK || !hexa::settings_has_magic(buf, sizeof buf)) return;
    std::vector<hexa::Setting> s;
    std::string why;
    bool ok = hexa::parse_settings(buf, sizeof buf, &s, &why);
    if (ok) {
        // What the link will be with these keys over the ones in NVS: link.cpp falls back to
        // serial for a Wi-Fi link without a network or an agent, which strands the robot.
        String link = prefs.getString("link", "serial"), ssid = prefs.getString("ssid", ""),
               agent = prefs.getString("agent", "");
        for (const hexa::Setting& o : s) {
            if (o.key == "link") link = o.value.c_str();
            if (o.key == "ssid") ssid = o.value.c_str();
            if (o.key == "agent") agent = o.value.c_str();
        }
        if (link == "wifi" && (ssid.isEmpty() || !IPAddress().fromString(agent))) {
            ok = false;
            why = "link=wifi needs ssid and agent (in the blob or already in NVS)";
        }
    }
    if (ok) {
        bool wifi_change = false;
        for (const hexa::Setting& o : s)
            if (is_trial_key(o.key) && prefs.getString(o.key.c_str(), "") != o.value.c_str()) wifi_change = true;
        // A second change before the first one joined keeps the first one's old values: those worked.
        if (wifi_change && !prefs.getBool("wifi_trial", false)) {
            for (const char* k : kTrialKeys) {
                const String prev = String("prev_") + k;
                if (prefs.isKey(k)) prefs.putString(prev.c_str(), prefs.getString(k, ""));
                else prefs.remove(prev.c_str());
            }
            prefs.putBool("wifi_trial", true);
        }
        String names;
        for (const hexa::Setting& o : s) {
            put(prefs, o);
            names += names.isEmpty() ? "" : ", ";
            names += o.key.c_str();
        }
        result = String("applied ") + s.size() + " keys (" + names + ")" + (wifi_change ? ", Wi-Fi on trial" : "");
    } else {
        result = String("refused: ") + why.c_str();
    }
    esp_partition_erase_range(p, 0, hexa::kSettingsMax);
    Serial.printf("settings: %s\n", result.c_str());
}

void trial_loop(Preferences& prefs, bool network_up) {
    static bool done = false;
    if (done) return;
    if (!prefs.getBool("wifi_trial", false)) {
        done = true;
        return;
    }
    if (network_up) {
        for (const char* k : kTrialKeys) prefs.remove((String("prev_") + k).c_str());
        prefs.remove("wifi_trial");
        Serial.println("settings: the new Wi-Fi settings joined; kept");
        done = true;
    } else if (millis() > kTrialMs) {
        for (const char* k : kTrialKeys) {
            const String prev = String("prev_") + k;
            if (prefs.isKey(prev.c_str())) prefs.putString(k, prefs.getString(prev.c_str(), ""));
            else prefs.remove(k);
            prefs.remove(prev.c_str());
        }
        prefs.remove("wifi_trial");
        Serial.println("settings: no network in 60 s with the new Wi-Fi settings; the old ones put back, rebooting");
        delay(100);
        ESP.restart();
    }
}

const char* last_result() { return result.c_str(); }

}  // namespace hexa_settings
