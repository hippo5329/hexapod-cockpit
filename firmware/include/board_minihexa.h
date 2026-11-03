// Hiwonder miniHexa controller board: pin map.
//
// NOT IDENTIFIED YET (docs/hardware-minihexa.md, open questions 1-5). Until the board
// has been read on the bench, HEXA_BOARD_VERIFIED stays 0 and the firmware refuses
// real mode: it runs simulated servos only, so a wrong guess here can never drive a
// servo into a stop. The board has 18 servos and a classic ESP32 has 16 LEDC
// channels (an S3 has 8), so the servo outputs are probably not plain GPIO PWM;
// the output backend is chosen once the board is known.
#pragma once

#include <stdint.h>

#define HEXA_BOARD_VERIFIED 0

// Classic ESP32 defaults until read off the board.
#define HEXA_I2C_SDA 21
#define HEXA_I2C_SCL 22

// The devkit BOOT button. It selects the console app when pressed in the first
// seconds after boot (held through reset, it starts the ROM bootloader instead).
#define HEXA_BOOT_PIN 0

#define HEXA_SERIAL_BAUD 921600
