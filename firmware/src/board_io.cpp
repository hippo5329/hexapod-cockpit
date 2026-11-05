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
    Wire.setClock(400000);  // an IMU sample (12 bytes) every tick: ~0.4 ms at 400 kHz
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

bool servos(const hexa::BusServo* s, int n, uint16_t ms) {
    begin();
    uint8_t frame[80];
    const size_t len = hexa::servo_bus_frame(s, n, ms, frame, sizeof frame);
    if (!len) return false;
    Serial1.write(frame, len);  // 63 bytes for 18: 2.7 ms at 230400, into the UART's buffer
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

namespace {

bool i2c_write(uint8_t addr, uint8_t reg, uint8_t v) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(v);
    return Wire.endTransmission() == 0;
}

// QMI8658 registers and settings (datasheet): CTRL1 address auto-increment, little-endian;
// CTRL2 accel +-4 g (8192 LSB/g); CTRL3 gyro +-512 dps (64 LSB/dps); both ODR 224.2 Hz in
// 6-axis mode; CTRL5 both low-pass filters on, mode 0 (2.66 % of ODR, ~6 Hz); CTRL7 both on.
constexpr uint8_t kCtrl1 = 0x02, kCtrl2 = 0x03, kCtrl3 = 0x04, kCtrl5 = 0x06, kCtrl7 = 0x08, kReset = 0x60;
constexpr uint8_t kAccelX = 0x35;  // AX_L .. GZ_H: 12 bytes
constexpr float kG = 9.80665f;
constexpr float kAccelLsb = 8192.0f, kGyroLsb = 64.0f;
bool imu_ok = false;

}  // namespace

bool imu_begin() {
    begin();
    imu_ok = false;
    if (imu_who_am_i() != 0x05) return false;
    i2c_write(HEXA_IMU_ADDR, kReset, 0xB0);
    delay(20);
    const uint8_t cfg[][2] = {{kCtrl1, 0x40}, {kCtrl2, 0x15}, {kCtrl3, 0x55}, {kCtrl5, 0x11}, {kCtrl7, 0x03}};
    for (const auto& c : cfg) {
        uint8_t back = 0;
        if (!i2c_write(HEXA_IMU_ADDR, c[0], c[1]) || !i2c_read(HEXA_IMU_ADDR, c[0], &back, 1) || back != c[1])
            return false;
    }
    delay(50);  // the first samples after enabling
    imu_ok = true;
    return true;
}

bool imu_read(float accel[3], float gyro[3]) {
    uint8_t b[12];
    if (!imu_ok || !i2c_read(HEXA_IMU_ADDR, kAccelX, b, sizeof b)) return false;
    for (int i = 0; i < 3; ++i) {
        accel[i] = static_cast<int16_t>(b[2 * i] | b[2 * i + 1] << 8) / kAccelLsb * kG;
        gyro[i] = static_cast<int16_t>(b[6 + 2 * i] | b[7 + 2 * i] << 8) / kGyroLsb * (3.14159265f / 180.0f);
    }
    return true;
}

int imu_who_am_i() {
    begin();
    uint8_t b;
    return i2c_read(HEXA_IMU_ADDR, 0, &b, 1) ? b : -1;
}

}  // namespace board_io
