// The micro-ROS link and the OTA service, from the NVS `hexa` namespace:
//
//   link     "serial" (default) or "wifi"
//   ssid     Wi-Fi network (2.4 GHz)       psk     its key
//   agent    the micro-ROS agent's IPv4    port    its UDP port (default 8888)
//   ota_pw   the OTA password; no OTA without one
//
// Serial: micro-ROS over USB serial (921600). Wi-Fi: micro-ROS over UDP, and ArduinoOTA on
// :3232 once the network is up; Serial then carries a status line. The keys are written to
// NVS with the flash (tools/make_nvs.py) or over Wi-Fi (tools/push_settings.py,
// settings.h) and never printed.
#pragma once

#include <Arduino.h>
#include <Preferences.h>

namespace hexa_link {

enum class Kind { kSerial, kWifi };

struct Config {
    Kind kind = Kind::kSerial;
    String ssid, psk, agent, ota_pw;
    uint16_t port = 8888;
};

Config load(Preferences& prefs);

// Starts the network (Wi-Fi); OTA follows when it is up. Both apps call it.
void begin(const Config& c);
// Points micro-ROS at the link. The base app only.
void set_microros_transport(const Config& c);
// The OTA service and the status line; every loop.
void loop();
// Called when an OTA update starts (stop the gait), and when one fails.
void on_ota(void (*start)(), void (*error)());

bool network_up();  // serial: true
// UDP datagrams received so far (any: replies, heartbeats, data): the agent is alive while it grows.
uint32_t rx_count();
void print_status(Print& out);
const char* running_partition();
const char* elf_sha();  // first 16 hex digits of the running app's ELF SHA-256

}  // namespace hexa_link
