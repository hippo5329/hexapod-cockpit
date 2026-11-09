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

// The motor's PWM (lidar_fwd.h). 30 kHz by default, LDROBOT's typical; 10 bits (80 MHz / 50 kHz holds 10).
uint32_t pwm_freq = 30000;
constexpr uint8_t kPwmBits = 10;
constexpr uint8_t kPwmChannel = 2;  // timer 1: the camera's XCLK is on timer 0, channel 0
int pwm = -1, duty_pct = -1;
bool run = true, attached = false;

// LD19 packets, for the motor's speed: 0x54 0x2C, speed (u16 LE, deg/s), ..., CRC-8 (poly 0x4D, as the
// ldlidar driver's table), 47 bytes.
constexpr size_t kPkt = 47;
uint8_t pkt[kPkt];
size_t pn = 0;
volatile uint32_t n_frames = 0, n_crc = 0, last_frame_ms = 0;
volatile uint16_t speed_dps = 0;

uint8_t crc8(const uint8_t* p, size_t n) {
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = c & 0x80 ? static_cast<uint8_t>(c << 1) ^ 0x4D : static_cast<uint8_t>(c << 1);
    }
    return c;
}

void parse(const uint8_t* b, size_t n) {
    for (size_t i = 0; i < n; i++) {
        const uint8_t x = b[i];
        if (pn == 0) {
            if (x == 0x54) pkt[pn++] = x;
            continue;
        }
        if (pn == 1) {
            if (x == 0x2C) pkt[pn++] = x;
            else if (x != 0x54) pn = 0;
            continue;
        }
        pkt[pn++] = x;
        if (pn < kPkt) continue;
        pn = 0;
        if (crc8(pkt, kPkt - 1) != pkt[kPkt - 1]) {
            n_crc = n_crc + 1;
            continue;
        }
        speed_dps = static_cast<uint16_t>(pkt[2] | pkt[3] << 8);
        last_frame_ms = millis();
        n_frames = n_frames + 1;
    }
}

// The pin's level: released (input: the LiDAR's own control), or the PWM at the duty, or stopped.
void apply_pwm() {
    if (pwm < 0) return;
    if (run && duty_pct < 0) {
        if (attached) ledcDetach(pwm);
        attached = false;
        pinMode(pwm, INPUT);
        return;
    }
    if (!attached) attached = ledcAttachChannel(pwm, pwm_freq, kPwmBits, kPwmChannel);
    if (!attached) {
        hexa_log::log(hexa_log::kErr, "lidar: PWM on GPIO %d failed", pwm);
        return;
    }
    const uint32_t full = 1u << kPwmBits;  // full = always high
    ledcWrite(pwm, run ? full * static_cast<uint32_t>(duty_pct) / 100 : 0);
}

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
        parse(buf + len, cc);
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
    hexa_log::log(hexa_log::kNotice, "lidar: forwarding GPIO %d (%u baud) to %s:%u; motor PWM GPIO %d duty %s %u Hz", rx,
                  static_cast<unsigned>(rx_baud), server_ip.toString().c_str(), static_cast<unsigned>(server_port), pwm,
                  duty_pct < 0 ? "auto" : String(duty_pct).c_str(), static_cast<unsigned>(pwm_freq));
}

void report() {
    static uint32_t last_ms = 0, last_err = 0;
    if (n_errors == last_err || millis() - last_ms < 1000) return;
    hexa_log::log(hexa_log::kInfo, "lidar: rx errors %u (+%u)", static_cast<unsigned>(n_errors),
                  static_cast<unsigned>(n_errors - last_err));
    last_err = n_errors;
    last_ms = millis();
}

void motor_begin(int pwm_pin, int duty, uint32_t hz) {
    pwm = pwm_pin;
    duty_pct = duty;
    pwm_freq = hz;
    apply_pwm();
}

bool set_hz(uint32_t hz) {
    if (pwm < 0) return false;
    pwm_freq = hz;
    if (attached && !ledcChangeFrequency(pwm, pwm_freq, kPwmBits))
        hexa_log::log(hexa_log::kErr, "lidar: PWM %u Hz failed", static_cast<unsigned>(hz));
    apply_pwm();
    return true;
}

uint32_t pwm_hz() { return pwm_freq; }

bool set_duty(int duty) {
    if (pwm < 0) return false;
    duty_pct = duty;
    apply_pwm();
    return true;
}

bool set_motor(bool on) {
    if (pwm < 0) return false;
    run = on;
    apply_pwm();
    return true;
}

int pwm_pin() { return pwm; }
int duty() { return duty_pct; }
bool motor_on() { return run; }
float scan_hz() { return frame_age_ms() > 500 ? 0.0f : speed_dps / 360.0f; }
uint32_t frames() { return n_frames; }
uint32_t crc_errors() { return n_crc; }
uint32_t frame_age_ms() { return n_frames ? millis() - last_frame_ms : UINT32_MAX; }

void pause() { paused = true; }
bool on() { return forwarding; }
uint32_t bytes() { return n_bytes; }
uint32_t packets() { return n_packets; }
uint32_t errors() { return n_errors; }

}  // namespace lidar_fwd
