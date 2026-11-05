// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Ported from linorobot2-cockpit (qmi8658.h says what and how).
#include "qmi8658.h"

#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint8_t REG_WHO_AM_I = 0x00;   // -> 0x05
constexpr uint8_t REG_REVISION = 0x01;
constexpr uint8_t REG_CTRL1 = 0x02;      // SIM | ADDR_AI | BE | INT2_EN | INT1_EN | FIFO_INT_SEL | - | SensorDisable
constexpr uint8_t REG_CTRL2 = 0x03;      // aST | aFS[2:0] | aODR[3:0]
constexpr uint8_t REG_CTRL3 = 0x04;      // gST | gFS[2:0] | gODR[3:0]
constexpr uint8_t REG_CTRL5 = 0x06;      // - | gLPF_MODE[1:0] | gLPF_EN | - | aLPF_MODE[1:0] | aLPF_EN
constexpr uint8_t REG_CTRL7 = 0x08;      // syncSmpl | - | DRDY_DIS | gSN | - | - | gEN | aEN
constexpr uint8_t REG_STATUSINT = 0x2D;  // CmdDone | ... | Avail | Locked
constexpr uint8_t REG_RESET = 0x60;      // write 0xB0

constexpr uint8_t CTRL1_ADDR_AI = 0x40;  // auto-increment the register address in a burst
constexpr uint8_t CTRL1_BE = 0x20;       // "big endian" in QST's naming: L byte at the lower address
constexpr uint8_t CTRL7_GEN = 0x02;
constexpr uint8_t CTRL7_AEN = 0x01;

constexpr uint8_t CTRL2_8G_224HZ = 0x25;       // aFS 010 (+/-8 g) | aODR 0101 (224.2 Hz)
constexpr uint8_t CTRL3_1024DPS_224HZ = 0x65;  // gFS 110 (+/-1024 dps) | gODR 0101 (224.2 Hz)
constexpr uint8_t CTRL5_LPF_BOTH = 0x77;       // gLPF mode 3 + enable, aLPF mode 3 + enable

constexpr float ACCEL_LSB_PER_G = 4096.0f;
constexpr float GYRO_LSB_PER_DPS = 32.0f;
constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
constexpr float G_TO_ACCEL = 9.81f;  // the cockpit's g

constexpr uint8_t BURST_FIRST = REG_STATUSINT;  // 0x2D .. 0x40 inclusive
constexpr uint8_t BURST_LEN = 20;

int16_t le16(const uint8_t* p) { return static_cast<int16_t>(static_cast<uint16_t>(p[1]) << 8 | p[0]); }

}  // namespace

void Qmi8658::write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t Qmi8658::read_reg(uint8_t reg) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom(static_cast<int>(addr_), 1) != 1) return 0xFF;
    return Wire.read();
}

bool Qmi8658::read_block(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    const uint8_t got = Wire.requestFrom(static_cast<int>(addr_), static_cast<int>(len));
    for (uint8_t i = 0; i < got; i++) {
        const uint8_t b = Wire.read();
        if (i < len) buf[i] = b;
    }
    return got == len;
}

bool Qmi8658::sample() {
    uint8_t b[BURST_LEN];
    if (!read_block(BURST_FIRST, b, BURST_LEN)) {
        bursts_failed_++;
        return false;
    }
    bursts_ok_++;
    // b[0] STATUSINT, b[1] STATUS0, b[2] STATUS1, b[3..5] TIMESTAMP_L/M/H, b[6..7] TEMP_L/H,
    // b[8..13] AX..AZ, b[14..19] GX..GZ
    sample_count_ = static_cast<uint32_t>(b[5]) << 16 | static_cast<uint32_t>(b[4]) << 8 | b[3];
    temperature_c_ = le16(&b[6]) / 256.0f;
    for (int i = 0; i < 3; ++i) {
        accel_[i] = le16(&b[8 + 2 * i]) / ACCEL_LSB_PER_G * G_TO_ACCEL;
        gyro_[i] = le16(&b[14 + 2 * i]) / GYRO_LSB_PER_DPS * DEG_TO_RAD_F;
    }
    return true;
}

bool Qmi8658::begin() {
    bool found = false;
    const uint8_t cand[2] = {0x6B, 0x6A};
    for (uint8_t i = 0; i < 2 && !found; i++) {
        addr_ = cand[i];
        found = read_reg(REG_WHO_AM_I) == 0x05;
    }
    if (!found) return false;
    write_reg(REG_RESET, 0xB0);  // soft reset: every CTRL back to default
    delay(20);
    revision_ = read_reg(REG_REVISION);
    const uint8_t ctrl7 = CTRL7_GEN | CTRL7_AEN;
    write_reg(REG_CTRL1, CTRL1_ADDR_AI | CTRL1_BE);  // INT pins stay high-Z
    write_reg(REG_CTRL2, CTRL2_8G_224HZ);
    write_reg(REG_CTRL3, CTRL3_1024DPS_224HZ);
    write_reg(REG_CTRL5, CTRL5_LPF_BOTH);
    write_reg(REG_CTRL7, ctrl7);
    delay(100);  // filters settle; first samples land
    if (read_reg(REG_CTRL7) != ctrl7) return false;
    return sample();
}

bool Qmi8658::read(float accel[3], float gyro[3]) {
    const bool ok = sample();
    for (int i = 0; i < 3; ++i) {
        accel[i] = accel_[i];
        gyro[i] = gyro_[i];
    }
    return ok && bursts_ok_ > 0;
}
