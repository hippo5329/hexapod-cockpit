// Remote logging over UDP syslog, linorobot2-cockpit's (firmware/common/lib/syslog): the same RFC 5424
// line ("<PRI>1 - host app - - - msg"), one datagram per call, so the cockpit's receiver
// (web/backend/syslog_manager.py) reads it as it reads a linorobot2 board's. The sink is NVS
// `syslog_ip` / `syslog_port` (default 5140, the cockpit's); nothing is sent without one, on a
// serial link (no radio: a UDP send there is an lwIP abort, as the cockpit measured) or before the
// network is up. Lines from before that (the boot's) wait in a small backlog and go out when it is.
//
// log() also prints the line on Serial when Serial is free (a Wi-Fi link, or the console app).
#pragma once

#include <Preferences.h>
#include <stdint.h>

namespace hexa_log {

enum Severity : uint8_t { kErr = 3, kWarning = 4, kNotice = 5, kInfo = 6 };

// host and app name the sender in each line (the camera board shares this module: camera/).
void begin(Preferences& prefs, bool wifi_link, bool serial_free, const char* host = "minihexa",
           const char* app = "hexapod");
void log(Severity sev, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
// Every loop: the backlog out once the network is up.
void loop(bool network_up);
// "192.168.4.1:5140", or "off" (no sink, or a serial link), for info.
const char* sink();

}  // namespace hexa_log
