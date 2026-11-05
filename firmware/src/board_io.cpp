#include "board_io.h"

#include <Arduino.h>
#include <Wire.h>

#include "board_minihexa.h"
#include "hexa_servo.h"

namespace board_io {
namespace {

bool started = false;

// Reads n bytes from register reg of an I2C device; false when it does not answer.
bool i2c_read(uint8_t addr, uint8_t reg, uint8_t* buf, int n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(addr, static_cast<uint8_t>(n)) != n) return false;
    for (int i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

}  // namespace

void begin() {
    if (started) return;
    Serial1.begin(HEXA_SERVO_UART_BAUD, SERIAL_8N1, HEXA_SERVO_UART_RX, HEXA_SERVO_UART_TX);
    Wire.begin(HEXA_I2C_SDA, HEXA_I2C_SCL);
    analogSetPinAttenuation(HEXA_BAT_ADC_PIN, ADC_11db);
    started = true;
}

bool servo(uint8_t id, uint16_t us, uint16_t ms) {
    if (id < 1 || id > 18) return false;
    begin();
    const hexa::BusServo s{id, us};
    uint8_t frame[16];
    const size_t n = hexa::servo_bus_frame(&s, 1, ms, frame, sizeof frame);
    if (!n) return false;
    Serial1.write(frame, n);
    Serial1.flush();
    return true;
}

int battery_adc_mv() {
    begin();
    uint32_t sum = 0;
    for (int i = 0; i < 32; ++i) sum += analogReadMilliVolts(HEXA_BAT_ADC_PIN);
    if (sum == 0) return -1;
    return static_cast<int>(sum / 32 * HEXA_BAT_DIVIDER);
}

int battery_i2c_mv() {
    begin();
    uint8_t b[2];
    return i2c_read(HEXA_BAT_I2C_ADDR, 0, b, 2) ? (b[0] | b[1] << 8) : -1;
}

int sonar_mm() {
    begin();
    uint8_t b[2];
    return i2c_read(HEXA_SONAR_ADDR, 0, b, 2) ? (b[0] | b[1] << 8) : -1;
}

int imu_who_am_i() {
    begin();
    uint8_t b;
    return i2c_read(HEXA_IMU_ADDR, 0, &b, 1) ? b : -1;
}

}  // namespace board_io
