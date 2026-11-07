// Who holds the interrupts off (2026-11-07: the robot's UART lost bytes in the camera's first seconds).
// The prebuilt IDF has no CONFIG_UART_ISR_IN_IRAM, so every flash operation -- the cache off, every
// non-IRAM interrupt masked, an erase up to CONFIG_SPI_FLASH_ERASE_YIELD_DURATION_MS (10 ms) at a time
// -- stops the UART's interrupt, and its 128-byte FIFO overflows in 0.85 ms at 1.5 Mbaud. The link wraps
// (platformio.ini -Wl,--wrap=...) the IDF's cache disable/enable pair, timing each flash operation, and
// the NVS writes, keeping each one's caller (an address: firmware.elf and addr2line name it).
#pragma once
#include <Print.h>
#include <stdint.h>

namespace irq_watch {
// What was recorded since the last call, one line each (and cleared): flash operations that held the
// interrupts off for 100 us or more, and NVS writes with their callers. Logged from the loop.
void report(void (*line)(const char* text));
// The robot's UART overflowed now (its onReceiveError).
void mark_overflow();
uint32_t flash_ops();       // all, since boot
uint32_t flash_max_us();    // the longest, since boot
}  // namespace irq_watch
