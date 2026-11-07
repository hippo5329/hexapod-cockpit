// The UART to the camera board (board_minihexa.h HEXA_CAM_UART_*: Serial2, 1.5 Mbaud). For now a link
// test only: the camera echoes every byte (camera/src/main.cpp, NVS `uart=echo`), and test() sends a
// pseudo-random stream, checks what comes back and reports it.
#pragma once

#include <Arduino.h>

namespace cam_uart {

void begin(uint32_t baud = 0);  // 0: HEXA_CAM_UART_BAUD
// Console `uart <seconds> [baud]`: round trips of one byte, then the stream for `seconds`. Blocks.
void test(Print& out, float seconds, uint32_t baud);

}  // namespace cam_uart
