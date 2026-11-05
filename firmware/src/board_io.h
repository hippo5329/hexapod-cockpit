// The miniHexa board's buses: the servo controller (the gait's frames in real mode, one
// servo at a time from the bench console in sim mode), the battery, the ultrasonic, the
// IMU's identity.
#pragma once

#include <stdint.h>

#include "hexa_servo.h"

namespace board_io {

void begin();  // Serial1 to the servo controller, Wire on the board's pins
// One servo (id 1-18) to a pulse (500-2500 us) over ms; false if refused.
bool servo(uint8_t id, uint16_t us, uint16_t ms);
// Several servos in one frame (the gait's 18 every tick); false if refused.
bool servos(const hexa::BusServo* s, int n, uint16_t ms);
int battery_adc_mv();  // GPIO33 x 11, mean of 32 calibrated reads; -1 when the pin reads 0
int battery_i2c_mv();  // the V1.0 boards' monitor at 0x46; -1 when absent
int sonar_mm();        // the I2C ultrasonic; -1 when absent
int imu_who_am_i();    // QMI8658 register 0 (0x05); -1 when absent
// The QMI8658: accelerometer +-4 g and gyroscope +-512 dps at 224 Hz, low-pass filtered;
// false when the chip does not answer or does not take the configuration.
bool imu_begin();
// The latest sample in the chip's axes: specific force in m/s^2, rate in rad/s.
bool imu_read(float accel[3], float gyro[3]);

}  // namespace board_io
