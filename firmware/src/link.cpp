#include "link.h"

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_app_desc.h>
#include <esp_ota_ops.h>
#include <esp_wifi.h>

#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include "board_minihexa.h"

namespace hexa_link {
namespace {

Config cfg;
WiFiUDP udp;
IPAddress agent_ip;
bool ota_started = false;
uint32_t tx_ok = 0, tx_fail = 0, rx_n = 0;  // UDP datagrams, for the status line
uint32_t tx_max_us = 0;                     // slowest send since the last status line
void (*ota_start_cb)() = nullptr;
void (*ota_error_cb)() = nullptr;

// ---------------------------------------------------------------- micro-ROS over USB serial
// Framed (the XRCE client adds HDLC-style framing on a byte stream).
bool serial_open(uxrCustomTransport*) { return true; }
bool serial_close(uxrCustomTransport*) { return true; }
size_t serial_write(uxrCustomTransport*, const uint8_t* buf, size_t len, uint8_t*) { return Serial.write(buf, len); }
size_t serial_read(uxrCustomTransport*, uint8_t* buf, size_t len, int timeout, uint8_t*) {
    Serial.setTimeout(timeout);
    return Serial.readBytes(reinterpret_cast<char*>(buf), len);
}

// ---------------------------------------------------------------- micro-ROS over UDP
// One datagram per XRCE message, not framed. A read with timeout 0 (the base loop's) checks
// for one waiting datagram and returns.
bool udp_open(uxrCustomTransport*) { return udp.begin(cfg.port); }
bool udp_close(uxrCustomTransport*) {
    udp.stop();
    return true;
}
size_t udp_write(uxrCustomTransport*, const uint8_t* buf, size_t len, uint8_t*) {
    if (WiFi.status() != WL_CONNECTED) return 0;
    const uint32_t t0 = micros();
    size_t sent = 0;
    if (udp.beginPacket(agent_ip, cfg.port)) {
        sent = udp.write(buf, len);
        if (!udp.endPacket()) sent = 0;  // the driver's send queue is full: the datagram is dropped
    }
    tx_max_us = max(tx_max_us, micros() - t0);
    if (sent) ++tx_ok;
    else ++tx_fail;
    return sent;
}
size_t udp_read(uxrCustomTransport*, uint8_t* buf, size_t len, int timeout, uint8_t*) {
    const uint32_t t0 = millis();
    do {
        if (udp.parsePacket() > 0) {
            ++rx_n;
            const int n = udp.read(buf, len);
            return n > 0 ? static_cast<size_t>(n) : 0;
        }
        if (timeout > 0) delay(1);
    } while (static_cast<int32_t>(millis() - t0) < timeout);
    return 0;
}

// ---------------------------------------------------------------- OTA
void ota_begin() {
    ArduinoOTA.setPort(3232);
    ArduinoOTA.setPassword(cfg.ota_pw.c_str());
    ArduinoOTA.setMdnsEnabled(false);
    ArduinoOTA.onStart([]() {
        // An update blocks the loop for its whole length: the loop watchdog would reset
        // the board half-way. It is armed again if the update fails; success reboots.
        disableLoopWDT();
        if (ota_start_cb) ota_start_cb();
        Serial.println(ArduinoOTA.getCommand() == U_SPIFFS ? "ota: settings upload started" : "ota: update started");
    });
    ArduinoOTA.onEnd([]() { Serial.println("ota: update written, rebooting"); });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("ota: update failed (error %d)\n", static_cast<int>(e));
        enableLoopWDT();
        if (ota_error_cb) ota_error_cb();
    });
    ArduinoOTA.begin();
    ota_started = true;
}

char elf_sha_buf[17];

}  // namespace

Config load(Preferences& prefs) {
    Config c;
    c.kind = prefs.getString("link", "serial") == "wifi" ? Kind::kWifi : Kind::kSerial;
    c.ssid = prefs.getString("ssid", "");
    c.psk = prefs.getString("psk", "");
    c.agent = prefs.getString("agent", "");
    c.port = prefs.getUShort("port", 8888);
    c.ota_pw = prefs.getString("ota_pw", "");
    // A Wi-Fi link without a network or an agent cannot work: fall back to serial.
    if (c.kind == Kind::kWifi && (c.ssid.isEmpty() || !IPAddress().fromString(c.agent))) c.kind = Kind::kSerial;
    return c;
}

void begin(const Config& c) {
    cfg = c;
    if (cfg.kind != Kind::kWifi) return;
    agent_ip.fromString(cfg.agent);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char host[24];
    snprintf(host, sizeof host, "minihexa-%02x%02x%02x", mac[3], mac[4], mac[5]);
    WiFi.persistent(false);  // the keys live in our namespace only
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(host);
    WiFi.setSleep(false);  // modem sleep adds ~100 ms to every reply
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfg.ssid.c_str(), cfg.psk.c_str());
}

void set_microros_transport(const Config& c) {
    if (c.kind == Kind::kWifi) {
        rmw_uros_set_custom_transport(false, nullptr, udp_open, udp_close, udp_write, udp_read);
    } else {
        rmw_uros_set_custom_transport(true, nullptr, serial_open, serial_close, serial_write, serial_read);
    }
}

void on_ota(void (*start)(), void (*error)()) {
    ota_start_cb = start;
    ota_error_cb = error;
}

bool network_up() { return cfg.kind != Kind::kWifi || WiFi.status() == WL_CONNECTED; }

void loop() {
    if (cfg.kind != Kind::kWifi) return;
    static bool was_up = false;
    static uint32_t last_status = 0;
    const bool up = WiFi.status() == WL_CONNECTED;
    // Modem sleep holds every frame to the AP for up to a beacon interval; set off again on
    // every join (the driver may restore its default on a reconnect).
    if (up && !was_up) esp_wifi_set_ps(WIFI_PS_NONE);
    if (up && !ota_started && !cfg.ota_pw.isEmpty()) ota_begin();
    if (ota_started) ArduinoOTA.handle();
    // Serial is free on a Wi-Fi link: a status line on every change and every 30 s.
    if (up != was_up || millis() - last_status > 30000) {
        was_up = up;
        last_status = millis();
        print_status(Serial);
    }
}

void print_status(Print& out) {
    if (cfg.kind != Kind::kWifi) {
        out.printf("link serial %d baud  app %s  elf %s\n", HEXA_SERIAL_BAUD, running_partition(), elf_sha());
        return;
    }
    const bool up = WiFi.status() == WL_CONNECTED;
    out.printf("link wifi ssid %s %s", cfg.ssid.c_str(), up ? "up" : "down");
    if (up) out.printf(" ip %s rssi %d ch %d", WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_wifi_get_ps(&ps);
    out.printf("  ps %s", ps == WIFI_PS_NONE ? "none" : ps == WIFI_PS_MIN_MODEM ? "min" : "max");
    out.printf("  agent %s:%u  udp tx %u fail %u rx %u max %u us  ota %s  app %s  elf %s\n", cfg.agent.c_str(), cfg.port,
               static_cast<unsigned>(tx_ok), static_cast<unsigned>(tx_fail), static_cast<unsigned>(rx_n),
               static_cast<unsigned>(tx_max_us), cfg.ota_pw.isEmpty() ? "off (no password)" : (ota_started ? "on :3232" : "waiting"),
               running_partition(), elf_sha());
    tx_max_us = 0;
}

uint32_t rx_count() { return rx_n; }

const char* running_partition() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    return p ? p->label : "?";
}

const char* elf_sha() {
    if (!elf_sha_buf[0]) esp_app_get_elf_sha256(elf_sha_buf, sizeof elf_sha_buf);
    return elf_sha_buf;
}

}  // namespace hexa_link
