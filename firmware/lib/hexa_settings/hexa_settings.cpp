#include "hexa_settings.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace hexa {
namespace {

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r')) --b;
    return s.substr(a, b - a);
}

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> f;
    size_t from = 0;
    for (;;) {
        const size_t at = s.find(sep, from);
        f.push_back(trim(s.substr(from, at == std::string::npos ? std::string::npos : at - from)));
        if (at == std::string::npos) return f;
        from = at + 1;
    }
}

bool is_int(const std::string& s, long lo, long hi) {
    size_t i = (!s.empty() && (s[0] == '-' || s[0] == '+')) ? 1 : 0;
    if (i == s.size() || s.size() > 8) return false;
    for (; i < s.size(); ++i)
        if (s[i] < '0' || s[i] > '9') return false;
    const long v = std::strtol(s.c_str(), nullptr, 10);
    return v >= lo && v <= hi;
}

bool is_unsigned(const std::string& s, long lo, long hi) { return !s.empty() && s[0] != '-' && s[0] != '+' && is_int(s, lo, hi); }

// Digits with at most one point, optional sign (no exponent, no "inf").
bool is_float(const std::string& s, float bound) {
    size_t i = (!s.empty() && (s[0] == '-' || s[0] == '+')) ? 1 : 0;
    int digits = 0, points = 0;
    for (; i < s.size(); ++i) {
        if (s[i] >= '0' && s[i] <= '9') ++digits;
        else if (s[i] == '.') ++points;
        else return false;
    }
    return digits > 0 && points <= 1 && std::fabs(std::strtof(s.c_str(), nullptr)) <= bound;
}

bool is_ipv4(const std::string& s) {
    const std::vector<std::string> f = split(s, '.');
    if (f.size() != 4) return false;
    for (const std::string& o : f)
        if (o.size() > 3 || !is_unsigned(o, 0, 255)) return false;
    return true;
}

bool removable(const std::string& key) {
    for (const char* k : {"app", "sim", "port", "trims", "imu_level", "idle_s", "low_bat_mv"})
        if (key == k) return true;
    return false;
}

// Empty when the value is good, else why (never the value).
std::string check(const std::string& key, const std::string& v) {
    if (v.empty()) return removable(key) ? "" : key + " cannot be removed over Wi-Fi";
    for (char c : v)
        if (static_cast<unsigned char>(c) < 0x20 || c == 0x7f) return key + " has a control character";
    if (key == "app") return v == "base" || v == "console" ? "" : "app must be base or console";
    if (key == "sim") return v == "0" || v == "1" ? "" : "sim must be 0 or 1";
    if (key == "link")
        return v == "wifi" ? "" : v == "serial" ? "link=serial would cut the Wi-Fi link: set it over USB"
                                                : "link must be wifi";
    if (key == "ssid") return v.size() <= 32 ? "" : "ssid is too long";
    if (key == "psk") return v.size() >= 8 && v.size() <= 63 ? "" : "psk must be 8-63 characters";
    if (key == "agent") return is_ipv4(v) ? "" : "agent must be an IPv4 address";
    if (key == "port") return is_unsigned(v, 1, 65535) ? "" : "port must be 1-65535";
    if (key == "ota_pw") return v.size() <= 64 ? "" : "ota_pw is too long";
    if (key == "idle_s") return is_unsigned(v, 0, 3600) ? "" : "idle_s must be 0-3600 (seconds)";
    if (key == "low_bat_mv")
        return v == "0" || is_unsigned(v, 5600, 8000) ? "" : "low_bat_mv must be 0 or 5600-8000 (mV)";
    if (key == "imu_level") {
        const std::vector<std::string> f = split(v, ',');
        return f.size() == 2 && is_float(f[0], 10) && is_float(f[1], 10)
                   ? ""
                   : "imu_level must be roll,pitch in degrees, each within +-10";
    }
    if (key == "trims") {
        const std::vector<std::string> f = split(v, ',');
        bool ok = f.size() == 18;
        for (size_t i = 0; ok && i < f.size(); ++i) ok = is_int(f[i], -200, 200);
        return ok ? "" : "trims must be 18 comma-separated integers within +-200";
    }
    return "not a known key";
}

}  // namespace

bool settings_has_magic(const char* blob, size_t len) {
    const size_t n = sizeof kSettingsMagic - 1;
    return len >= n && std::memcmp(blob, kSettingsMagic, n) == 0;
}

bool parse_settings(const char* blob, size_t len, std::vector<Setting>* out, std::string* why) {
    if (!settings_has_magic(blob, len)) {
        *why = "no hexa-settings header";
        return false;
    }
    size_t end = sizeof kSettingsMagic - 1;
    while (end < len && blob[end] != '\0' && static_cast<unsigned char>(blob[end]) != 0xff) ++end;
    if (end == len) {
        *why = "no end (NUL) within the blob";
        return false;
    }
    std::vector<Setting> s;
    const std::string text(blob + sizeof kSettingsMagic - 1, end - (sizeof kSettingsMagic - 1));
    size_t from = 0;
    for (int n = 2; from < text.size(); ++n) {
        size_t nl = text.find('\n', from);
        if (nl == std::string::npos) nl = text.size();
        const std::string line = trim(text.substr(from, nl - from));
        from = nl + 1;
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        const std::string key = eq == std::string::npos ? "" : trim(line.substr(0, eq));
        if (eq == std::string::npos || key.empty()) {
            *why = "line " + std::to_string(n) + ": not a KEY=VALUE line";
            return false;
        }
        const std::string value = trim(line.substr(eq + 1));
        const std::string bad = check(key, value);
        if (!bad.empty()) {
            *why = "line " + std::to_string(n) + ": " + bad;
            return false;
        }
        for (const Setting& o : s)
            if (o.key == key) {
                *why = "line " + std::to_string(n) + ": " + key + " given twice";
                return false;
            }
        s.push_back({key, value});
    }
    if (s.empty()) {
        *why = "no settings";
        return false;
    }
    *out = s;
    return true;
}

}  // namespace hexa
