#include "syslog.h"

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>

namespace hexa_log {
namespace {

constexpr const char* kHost = "minihexa";
constexpr const char* kApp = "hexapod";
constexpr uint16_t kDefaultPort = 5140;
constexpr int kBacklog = 12;
constexpr size_t kLine = 200;

WiFiUDP udp;
IPAddress ip;
uint16_t port = 0;
bool wifi = false, serial_out = true, up = false;
char sink_str[24] = "off";
struct Held {
    uint8_t sev;
    char msg[kLine];
} held[kBacklog];
int n_held = 0, dropped = 0;

void send(uint8_t sev, const char* msg) {
    if (!udp.beginPacket(ip, port)) return;
    // Facility user (1): the line is the robot's, not the kernel's.
    udp.printf("<%u>1 - %s %s - - - \xEF\xBB\xBF%s", static_cast<unsigned>((1 << 3) | sev), kHost, kApp, msg);
    udp.endPacket();
}

}  // namespace

void begin(Preferences& prefs, bool wifi_link, bool serial_free) {
    wifi = wifi_link;
    serial_out = serial_free;
    const String s = prefs.getString("syslog_ip", "");
    const String p = prefs.getString("syslog_port", "");
    port = p.isEmpty() ? kDefaultPort : static_cast<uint16_t>(p.toInt());
    if (!wifi || s.isEmpty() || !ip.fromString(s) || port == 0) {
        port = 0;
        return;
    }
    snprintf(sink_str, sizeof sink_str, "%s:%u", s.c_str(), static_cast<unsigned>(port));
}

void log(Severity sev, const char* fmt, ...) {
    char msg[kLine];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof msg, fmt, args);
    va_end(args);
    if (serial_out) Serial.println(msg);
    if (!port) return;
    if (up && WiFi.status() == WL_CONNECTED) {
        send(sev, msg);
    } else if (n_held < kBacklog) {
        held[n_held].sev = sev;
        strncpy(held[n_held].msg, msg, kLine);
        ++n_held;
    } else {
        ++dropped;
    }
}

void loop(bool network_up) {
    up = network_up && wifi;
    if (!port || !up || (!n_held && !dropped)) return;
    for (int i = 0; i < n_held; ++i) send(held[i].sev, held[i].msg);
    if (dropped) {
        char m[48];
        snprintf(m, sizeof m, "syslog: %d boot lines dropped (backlog full)", dropped);
        send(kWarning, m);
    }
    n_held = dropped = 0;
}

const char* sink() { return sink_str; }

}  // namespace hexa_log
