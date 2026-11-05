// Hiwonder miniHexa controller board: pin map.
//
// Pins and buses as Hiwonder's published miniHexa sketch uses them (github.com/Hiwonder/MiniHexa;
// taken as interface facts, no code copied), checked against the robot's own firmware image.
// The servos are not on ESP32 pins: a servo controller on the base board makes the 18 pulses
// and takes frames on a UART (hexa_servo.h, servo_bus_frame). Verified on miniHexa #1
// (2026-11-05): every servo moves alone on its id, and every leg's foot goes where the
// console's `foot` command (gait IK + servo calibration + trims) puts it, on two cameras.
// Real mode (NVS sim=0) drives the servos from the gait.
#pragma once

#include <stdint.h>

#define HEXA_BOARD_VERIFIED 1

// Servo controller: Serial1, 8N1.
#define HEXA_SERVO_UART_BAUD 230400
#define HEXA_SERVO_UART_RX 16
#define HEXA_SERVO_UART_TX 17

// I2C: the QMI8658 IMU (0x6A), the Hiwonder I2C ultrasonic (0x77, distance in mm at
// register 0), and on V1.0 boards the battery monitor (0x46, mV at register 0).
#define HEXA_I2C_SDA 22
#define HEXA_I2C_SCL 23
#define HEXA_IMU_ADDR 0x6A
#define HEXA_SONAR_ADDR 0x77
#define HEXA_BAT_I2C_ADDR 0x46

// Battery on V1.1 boards: ADC1 channel 5 (ADC1 works with Wi-Fi on) behind 100k/10k.
#define HEXA_BAT_ADC_PIN 33
#define HEXA_BAT_DIVIDER 11

#define HEXA_BUZZER_PIN 21
#define HEXA_LED_PIN 2

// The BOOT button. It selects the console app when pressed in the first seconds after
// boot (held through reset, it starts the ROM bootloader instead).
#define HEXA_BOOT_PIN 0

#define HEXA_SERIAL_BAUD 921600
