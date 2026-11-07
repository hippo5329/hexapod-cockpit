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
}  // namespace lidar_fwd
