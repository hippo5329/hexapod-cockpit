// The miniHexa board's buses: the servo controller (the gait's frames in real mode, one
// servo at a time from the bench console in sim mode), the battery, the ultrasonic, the
// IMU's identity.
#pragma once

#include <Print.h>
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
// The ultrasonic's two RGB lights, solid (its register map: mode 2, colours 3-8); false when absent.
bool sonar_rgb(uint8_t r, uint8_t g, uint8_t b);
int imu_who_am_i();    // QMI8658 register 0 (0x05); -1 when absent
// An I2C device answers at addr (the Advanced kit's modules: vision 0x52, voice 0x34).
bool i2c_present(uint8_t addr);
// The QMI8658 (qmi8658.h, linorobot2-cockpit's driver): false when the chip does not
// answer or does not take the configuration.
bool imu_begin();
// One sample in the chip's axes: specific force in m/s^2, rate in rad/s.
bool imu_read(float accel[3], float gyro[3]);
void imu_status(Print& out);
// Before the final shutdown's deep sleep: the servo line held idle (high) through the sleep,
// so the controller never reads a floating pin as a frame. begin() releases it at the next boot.
void before_deep_sleep();

}  // namespace board_io
