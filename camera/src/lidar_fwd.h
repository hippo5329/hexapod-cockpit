// The LiDAR forwarder: an LD19 (or any LiDAR that streams on a UART) wired to this board's UART2 RX, its
// bytes sent as UDP datagrams to the robot computer, where linorobot2-cockpit's ldlidar node in
// `udp_server` mode makes /scan. Ported from linorobot2-cockpit firmware/common/lib/lidar/lidar.cpp
// (Apache-2.0, Thomas Chou): raw bytes, not aligned to LD19 packets, more than 128 at a time; dropped while
// the radio is down or an OTA runs; receive errors to syslog once a second at most, from the loop.
//
// Settings (NVS namespace hexacam, /control?lidar_rx=..&lidar_ip=..&lidar_port=.., kept, reboots):
//   lidar_rx    the GPIO wired to the LiDAR's TX; -1 (default): off
//   lidar_ip    the robot computer (the ldlidar node's host); none: off
//   lidar_port  its UDP port (default 8889, cockpit's lidar.udp_port)
//   lidar_baud  default 230400 (LD19)
//
// The motor: an LD19's PWM input (20-50 kHz; floating or low, the LiDAR keeps its own 10 Hz) on a GPIO
// of this board, LEDC channel 2 = timer 1 (the camera's XCLK holds timer 0, channel 0):
//   lidar_pwm   the GPIO wired to the LiDAR's PWM; -1 (default): none, kept, reboots
//   lidar_duty  the PWM duty, % (0-100), or `auto` (default): the pin released, the LiDAR's own speed
//               control; applied at once, kept
//   lidar_pwm_hz  the PWM frequency, 20000-50000 Hz (default 30000, LDROBOT's typical); at once, kept
// and /control?lidar_motor=0|1 stops it / runs it at lidar_duty now (not kept: every boot runs it). The
// scan rate is read back from the packets: each LD19 packet carries the motor's speed, deg/s.
#pragma once
#include <Arduino.h>
#include <IPAddress.h>

namespace lidar_fwd {
// The settings, at boot; nothing is opened.
void begin(int rx_pin, const IPAddress& server, uint16_t port, uint32_t baud);
// Once the network is up, from the loop: opens UART2 and starts the forwarding task (its own, priority 3).
void start();
// From the loop: the receive errors, once a second at most (the UART's task only counts them).
void report();
// The OTA hook: nothing goes out on the radio but the update; the UART is still drained.
void pause();
bool on();
uint32_t bytes();     // received from the LiDAR, since boot
uint32_t packets();   // datagrams sent
uint32_t errors();    // the driver's receive errors (onReceiveError)

// The motor (above): at boot, before the network.
void motor_begin(int pwm_pin, int duty_pct, uint32_t hz);  // duty -1: released (auto)
bool set_hz(uint32_t hz);                     // false without a pin
uint32_t pwm_hz();
bool set_duty(int duty_pct);                  // -1 auto; false without a pin
bool set_motor(bool run);                     // false without a pin
int pwm_pin();
int duty();                                   // the setting, -1 auto
bool motor_on();
float scan_hz();          // the motor's speed from the last good packet / 360; 0 when none came for 500 ms
uint32_t frames();        // good LD19 packets (CRC passed)
uint32_t crc_errors();    // packets with a bad CRC
uint32_t frame_age_ms();  // since the last good packet; UINT32_MAX when none yet
}  // namespace lidar_fwd
