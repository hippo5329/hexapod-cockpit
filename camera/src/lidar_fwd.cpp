// Copyright (c) 2023 Thomas Chou
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Ported from linorobot2-cockpit firmware/common/lib/lidar/lidar.cpp (the UDP forwarder) to the camera
// board: UART2, settings from the camera's NVS, counters for /status and the health line.
#include "lidar_fwd.h"

#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "syslog.h"

namespace lidar_fwd {
namespace {

// cockpit's LIDAR_RX_BUFFER_SIZE: an LD19 streams ~17.6 KB/s (375 packets/s of 47 bytes); 4 KB holds
// ~230 ms, longer than any stall here. A dropped byte desyncs the framing until the next header.
constexpr size_t kRxBuffer = 4096;
constexpr size_t kBuf = 512;

HardwareSerial comm(2);
WiFiUDP udp;
uint8_t buf[kBuf];
size_t len = 0;
IPAddress server_ip;
uint16_t server_port = 8889;
bool forwarding = false;
volatile bool paused = false;
volatile uint32_t n_bytes = 0, n_packets = 0, n_errors = 0;
int rx = -1;
uint32_t rx_baud = 230400;
bool started = false;

// Counted only: the UART's event task never calls syslog (hexa_log is the loop's); report() logs them,
// once a second at most (an OTA write overflows this FIFO hundreds of times a second, and a syslog
// packet for each starved the update: cockpit, 2026-10-10).
void rx_err_callback(hardwareSerial_error_t) { n_errors = n_errors + 1; }

// Its own task (not the UART's event task, which runs near the top priority): drains the UART and sends
// a datagram once more than 128 bytes are in.
void fwd_task(void*) {
    for (;;) {
        const int a = comm.available();
        if (a <= 0) {
            vTaskDelay(2);
            continue;
        }
        const size_t cc = comm.read(buf + len, kBuf - len);
        n_bytes = n_bytes + cc;
        // No radio, no forwarding: drop the bytes, keep draining.
        if (paused || WiFi.status() != WL_CONNECTED) {
            len = 0;
            continue;
        }
        if ((len += cc) && len > 128) {  // larger UDP packets
            udp.beginPacket(server_ip, server_port);
            udp.write(buf, len);
            udp.endPacket();
            n_packets = n_packets + 1;
            len = 0;
        }
    }
}

}  // namespace

void begin(int rx_pin, const IPAddress& server, uint16_t port, uint32_t baud) {
    rx = rx_pin;
    rx_baud = baud;
    server_ip = server;
    server_port = port;
    forwarding = rx_pin >= 0 && server != IPAddress(0, 0, 0, 0);
}

void start() {
    if (started) return;
    started = true;
    if (!forwarding) {
        hexa_log::log(hexa_log::kInfo, "lidar: forwarder off (rx %d, server %s)", rx, server_ip.toString().c_str());
        return;
    }
    comm.setRxBufferSize(kRxBuffer);
    comm.onReceiveError(rx_err_callback);
    comm.begin(rx_baud, SERIAL_8N1, rx, -1);
    xTaskCreatePinnedToCore(fwd_task, "lidar_fwd", 4096, nullptr, 3, nullptr, 0);
    hexa_log::log(hexa_log::kNotice, "lidar: forwarding GPIO %d (%u baud) to %s:%u", rx,
                  static_cast<unsigned>(rx_baud), server_ip.toString().c_str(), static_cast<unsigned>(server_port));
}

void report() {
    static uint32_t last_ms = 0, last_err = 0;
    if (n_errors == last_err || millis() - last_ms < 1000) return;
    hexa_log::log(hexa_log::kInfo, "lidar: rx errors %u (+%u)", static_cast<unsigned>(n_errors),
                  static_cast<unsigned>(n_errors - last_err));
    last_err = n_errors;
    last_ms = millis();
}

void pause() { paused = true; }
bool on() { return forwarding; }
uint32_t bytes() { return n_bytes; }
uint32_t packets() { return n_packets; }
uint32_t errors() { return n_errors; }

}  // namespace lidar_fwd
