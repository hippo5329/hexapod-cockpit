// The miniHexa board's buses for the bench console: one servo at a time on the servo
// controller, the battery, the ultrasonic, the IMU's identity. None of it runs unless
// asked; the gait never drives the servos (real mode is refused, board_minihexa.h).
#pragma once

#include <stdint.h>

namespace board_io {

void begin();  // Serial1 to the servo controller, Wire on the board's pins
// One servo (id 1-18) to a pulse (500-2500 us) over ms; false if refused.
bool servo(uint8_t id, uint16_t us, uint16_t ms);
int battery_adc_mv();  // GPIO33 x 11, mean of 32 calibrated reads; -1 when the pin reads 0
int battery_i2c_mv();  // the V1.0 boards' monitor at 0x46; -1 when absent
int sonar_mm();        // the I2C ultrasonic; -1 when absent
int imu_who_am_i();    // QMI8658 register 0 (0x05); -1 when absent

}  // namespace board_io
